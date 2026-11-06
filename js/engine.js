// Рушій позиціонування: об'єднує обробку датчиків, фільтр по графу, GPS (із захистом від підміни)
// та ручні корекції. Працює однаково на телефоні та в симуляторі.

import { MotionProcessor } from './motion.js';
import { GraphParticleFilter } from './filter.js';
import { makeProjection, dist } from './geo.js';

export class NavEngine {
  constructor(graph, opts = {}) {
    this.g = graph;
    this.proj = makeProjection(graph.origin[0], graph.origin[1]);
    this.motion = new MotionProcessor(opts.motion);
    this.pf = new GraphParticleFilter(graph, opts.filter);
    this.gyroHeading = 0;        // інтегрований курс гіроскопа (рад, + ліворуч)
    this.lastStepHeading = 0;
    this.lastStepT = null;
    this.stepInterval = opts.stepInterval ?? 0.5; // с між кроками фільтра
    this.pendingDist = 0;
    this.distSinceSnap = 0;
    this.state = { mode: 'init', xy: null, latlon: null, heading: 0, speed: 0, confidence: 0, spread: 0, gps: 'none', spoof: false, spoofByNet: false, net: null };
    this.lastGoodGps = null;     // {t, xy} — останній довірений точний фікс
    this.lastNet = null;         // {t, xy, acc} — останній мережевий (вежі/Wi-Fi) фікс
    this.gpsTrust = 0;           // 0..1
    this.log = [];
    this.recording = false;
  }

  // ---------- вхід датчиків ----------
  pushMotion(sample) {
    if (this.recording) this.log.push({ k: 'm', ...sample });
    const r = this.motion.pushSample(sample);
    if (!r || !r.ready) { this.state.mode = r && !r.ready ? 'calibrating' : this.state.mode; return; }
    this.gyroHeading += r.yawRate * r.dt;
    this.pendingDist += r.speed * r.dt;
    this.state.speed = r.speed;
    if (this.lastStepT === null) { this.lastStepT = sample.t; this.lastStepHeading = this.gyroHeading; }
    if (sample.t - this.lastStepT >= this.stepInterval) {
      const dpsi = this.gyroHeading - this.lastStepHeading;
      if (this.pf.p.length) {
        this.pf.step(this.pendingDist, dpsi);
        this.distSinceSnap += this.pendingDist;
        // Самокалібрування масштабу швидкості за «прищіпками» на поворотах:
        // якщо частинки систематично відстають від перехрестя — швидкість занижена.
        const snap = this.pf.takeSnapStats();
        if (snap && snap.count > this.pf.N * 0.3 && this.distSinceSnap > 150) {
          const ratio = 1 + snap.mean / this.distSinceSnap;
          if (ratio > 0.6 && ratio < 1.6) {
            const m = this.motion;
            m.accelScale = Math.max(0.6, Math.min(1.8, m.accelScale * (1 + 0.5 * (ratio - 1))));
          }
          this.distSinceSnap = 0;
        }
      }
      this.pendingDist = 0;
      this.lastStepHeading = this.gyroHeading;
      this.lastStepT = sample.t;
      this._updateEstimate();
    }
  }

  // Фікс позиції від системи: lat, lon, accuracy (м), speed (м/с або null).
  // iOS не дає застосунку сирі вежі зв'язку й Wi-Fi, але сама визначає по них позицію
  // і повертає її через той самий geolocation — лише з великим радіусом похибки.
  // Тому класифікуємо фікс за точністю, а не за джерелом:
  //   ≤40 м  — супутники (GNSS): точна прив'язка;
  //   ≤250 м — змішаний (Wi-Fi поруч): помірна;
  //   ≤6 км  — мережа (вежі зв'язку): груба, але її РЕБ не глушить;
  //   більше — ігноруємо.
  pushGps(fix) {
    if (this.recording) this.log.push({ k: 'g', ...fix });
    const xy = this.proj.toXY(fix.lat, fix.lon);
    const acc = fix.accuracy ?? 30;
    const kind = acc <= 40 ? 'gnss' : acc <= 250 ? 'mixed' : acc <= 6000 ? 'net' : 'junk';
    if (kind === 'junk') return;

    // Перший фікс будь-якої якості задає початкове коло пошуку: груба позиція від
    // мережі накриває багато доріг, але фільтр звужує варіанти вже за кілька поворотів.
    if (!this.pf.p.length) {
      this.pf.initAround(xy, Math.max(60, Math.min(acc * 1.5, 3000)));
      this.state.mode = 'tracking';
      this.state.gps = kind === 'gnss' ? 'ok' : kind === 'mixed' ? 'weak' : 'net';
      if (kind !== 'net') this.lastGoodGps = { t: fix.t, xy };
      if (kind === 'net') this.lastNet = { t: fix.t, xy, acc };
      this._updateEstimate();
      return;
    }

    if (kind === 'net') {
      // Мережевий фікс не веде, а лише утримує від розбігання: підтягуємо тільки тоді,
      // коли власна оцінка вийшла за коло похибки, тобто суперечить мережі.
      this.lastNet = { t: fix.t, xy, acc };
      this.state.net = { xy, acc, t: fix.t };
      if (this.state.gps !== 'ok' && this.state.gps !== 'weak') this.state.gps = 'net';
      if (this.state.xy) {
        const off = dist(this.state.xy, xy);
        if (off > acc) {
          // Ми поза колом похибки — наша гіпотеза суперечить незалежному джерелу.
          // Саме лише зважування тут не рятує: пересіваємо частину гіпотез у колі.
          this.pf.reseedAround(xy, acc, 0.6);
        }
        this.pf.weightByPosition(xy, acc * 0.6);
        this._updateEstimate();
      }
      return;
    }

    // --- точний фікс: перевіряємо на підміну ---
    let plausible = true;
    if (this.lastGoodGps) {
      const dt = Math.max(0.5, fix.t - this.lastGoodGps.t);
      const maxMove = 50 * dt + acc + 30;
      if (dist(xy, this.lastGoodGps.xy) > maxMove) plausible = false;
    }
    if (plausible && this.state.xy && this.state.confidence > 0.5 && this.state.spread < 60) {
      if (dist(xy, this.state.xy) > 250 + 3 * acc) plausible = false;
    }
    // Перехресна перевірка мережею: РЕБ підміняє супутниковий сигнал, але не вежі зв'язку.
    // Якщо «точний» фікс далеко за межами свіжого мережевого кола — йому не місце.
    if (plausible && this.lastNet && fix.t - this.lastNet.t < 180) {
      if (dist(xy, this.lastNet.xy) > this.lastNet.acc + 3 * acc + 300) { plausible = false; this.state.spoofByNet = true; }
    }

    if (!plausible) {
      this.gpsTrust = Math.max(0, this.gpsTrust - 0.4);
      this.state.spoof = true; this.state.gps = 'rejected';
      return;
    }
    this.gpsTrust = Math.min(1, this.gpsTrust + 0.25);
    this.state.spoof = false; this.state.spoofByNet = false;
    this.state.gps = kind === 'gnss' ? 'ok' : 'weak';
    this.lastGoodGps = { t: fix.t, xy };
    if (this.gpsTrust >= 0.5) this.pf.weightByPosition(xy, Math.max(15, acc));
    if (kind === 'gnss' && fix.speed != null && fix.speed >= 0) this.motion.calibrateWithSpeed(fix.speed);
    this._updateEstimate();
  }

  // Давно не було жодного точного фікса — позначаємо, що ведемо автономно
  noteGpsSilence(now) {
    if (this.lastGoodGps && now - this.lastGoodGps.t > 12 && this.state.gps === 'ok') this.state.gps = 'jammed';
  }

  // ---------- ручні корекції ----------
  setPositionLatLon(lat, lon, r = 80) {
    const xy = this.proj.toXY(lat, lon);
    this.setPositionXY(xy, r);
  }
  setPositionXY(xy, r = 80) {
    this.pf.initAround(xy, r);
    this.state.mode = 'tracking';
    this._updateEstimate();
  }
  // «Я на цьому перехресті»
  setAtNode(nodeId, r = 40) { this.setPositionXY(this.g.nodeXY(nodeId), r); }

  flipTurns() { this.motion.flipGravitySign(); }

  startRecording() { this.log = []; this.recording = true; }
  stopRecording() { this.recording = false; return this.log; }

  _updateEstimate() {
    const est = this.pf.estimate();
    if (!est) return;
    this.state.xy = est.xy;
    this.state.latlon = this.proj.toLL(est.xy[0], est.xy[1]);
    this.state.heading = est.heading;
    this.state.confidence = est.confidence;
    this.state.spread = est.spread;
    this.state.edge = est.edge;
    this.state.s = est.s;
    this.state.mode = this.motion.ready ? 'tracking' : 'calibrating';
  }
}
