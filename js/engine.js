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
    this.state = { mode: 'init', xy: null, latlon: null, heading: 0, speed: 0, confidence: 0, spread: 0, gps: 'none', spoof: false };
    this.lastGoodGps = null;     // {t, xy}
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

  // GPS: lat, lon, accuracy (м), speed (м/с, може бути null)
  pushGps(fix) {
    if (this.recording) this.log.push({ k: 'g', ...fix });
    const xy = this.proj.toXY(fix.lat, fix.lon);
    const acc = fix.accuracy ?? 30;
    let plausible = true;
    // Захист від підміни: стрибок, який авто фізично не могло зробити
    if (this.lastGoodGps) {
      const dt = Math.max(0.5, fix.t - this.lastGoodGps.t);
      const maxMove = 50 * dt + acc + 30;
      if (dist(xy, this.lastGoodGps.xy) > maxMove) plausible = false;
    }
    // Стрибок відносно власної оцінки
    if (plausible && this.state.xy && this.state.confidence > 0.5 && this.state.spread < 60) {
      if (dist(xy, this.state.xy) > 250 + 3 * acc) plausible = false;
    }
    if (acc > 150) plausible = false;

    if (!plausible) {
      this.gpsTrust = Math.max(0, this.gpsTrust - 0.4);
      this.state.spoof = true; this.state.gps = 'rejected';
      return;
    }
    this.gpsTrust = Math.min(1, this.gpsTrust + 0.25);
    this.state.spoof = false; this.state.gps = 'ok';
    this.lastGoodGps = { t: fix.t, xy };
    if (!this.pf.p.length) { this.pf.initAround(xy, Math.max(60, acc * 2)); this.state.mode = 'tracking'; }
    else if (this.gpsTrust >= 0.5) this.pf.weightByPosition(xy, Math.max(15, acc));
    if (fix.speed != null && fix.speed >= 0) this.motion.calibrateWithSpeed(fix.speed);
    this._updateEstimate();
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
