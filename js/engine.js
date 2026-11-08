// Рушій позиціонування: об'єднує обробку датчиків, фільтр по графу, GPS (із захистом від підміни)
// та ручні корекції. Працює однаково на телефоні та в симуляторі.

import { MotionProcessor } from './motion.js';
import { GraphParticleFilter } from './filter.js';
import { makeProjection, dist } from './geo.js';

// Правдоподібна верхня межа швидкості за класом дороги, м/с
// (1 — проїзди й двори … 5 — магістралі). З запасом, щоб не різати реальний рух.
const EDGE_SPEED_CAP = { 1: 11, 2: 19, 3: 22, 4: 26, 5: 36 };

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
    this.state = { mode: 'init', xy: null, latlon: null, heading: 0, speed: 0, confidence: 0, spread: 0, gps: 'none', spoof: false, spoofByNet: false, net: null, hint: null, source: 'none' };
    // Час у секундах стінного годинника — ОДНА шкала для всіх джерел.
    // (Раніше фікси мали epoch, а датчики — час від старту сторінки, тож
    //  порівняння «чи давно був GPS» ніколи не спрацьовувало.)
    this.lastFixWall = 0;        // будь-який прийнятий фікс
    this.lastGnssWall = 0;       // саме супутниковий
    this.gpsLostAfter = opts.gpsLostAfter ?? 8;  // с без GNSS → автономний режим
    // Джерело часу можна підмінити — так симулятор перевіряє ті самі переходи, що й телефон.
    this._now = opts.now ?? (() => Date.now() / 1000);
    this.gpsSnap = null;     // фікс, прив'язаний до дороги — те, що показуємо в режимі GPS
    this.lastGoodGps = null;     // {t, xy} — останній довірений точний фікс
    this.lastNet = null;         // {t, xy, acc} — останній мережевий (вежі/Wi-Fi) фікс
    this.gpsTrust = 0;           // 0..1
    // Запис датчиків для розбору поїздки на комп'ютері. Семпли йдуть ~50 раз/с,
    // тому тримаємо їх не об'єктами, а колонками Float32 — це вдесятеро менше пам'яті,
    // і година їзди вміщується без проблем. Старі семпли витісняються по колу.
    this.recording = false;
    this.maxSamples = 180000;   // ~60 хв при 50 Гц
    this._rec = null;
    this.fixLog = [];
  }

  // ---------- вхід датчиків ----------
  pushMotion(sample) {
    if (this.recording) this._recordMotion(sample);
    const r = this.motion.pushSample(sample);
    if (!r || !r.ready) { this.state.mode = r && !r.ready ? 'calibrating' : this.state.mode; return; }
    this.gyroHeading += r.yawRate * r.dt;
    this.pendingDist += r.speed * r.dt;
    if (!(this.state.source === 'gps' && this.now() - this.lastGnssWall < 2.5)) {
      this.state.speed = r.speed;
      // Офлайн карта сама обмежує брехню: по дворовому проїзду 120 км/год не буває.
      if (this.state.source === 'dr' && this.state.edge >= 0) {
        const cap = EDGE_SPEED_CAP[this.g.level(this.state.edge)] || 30;
        if (this.motion.speed > cap) { this.motion.speed = cap; this.state.speed = cap; }
      }
    }
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
  //   ≤40 м   — супутники (GNSS): точна прив'язка;
  //   ≤250 м  — змішаний (Wi-Fi поруч): помірна;
  //   ≤3 км   — мережа (вежі зв'язку): груба, але її РЕБ не глушить;
  //   ≤50 км  — «приблизна геолокація» iOS або одна далека вежа. Прив'язати до
  //             вулиці неможливо (у колі тисячі доріг), але показати коло варто:
  //             людина бачить, де вона приблизно, і уточнює кнопкою «Я тут»;
  //   більше  — ігноруємо.
  pushGps(fix) {
    if (this.recording) { this.fixLog.push({ k: 'g', ...fix }); if (this.fixLog.length > 20000) this.fixLog.splice(0, 5000); }
    const xy = this.proj.toXY(fix.lat, fix.lon);
    const acc = fix.accuracy ?? 30;
    const kind = acc <= 40 ? 'gnss' : acc <= 250 ? 'mixed' : acc <= 3000 ? 'net'
      : acc <= 50000 ? 'coarse' : 'junk';
    if (kind === 'junk') return;

    if (kind === 'coarse') {
      this.state.hint = { xy, acc, t: fix.t };
      if (!this.pf.p.length) { this.state.gps = 'coarse'; this.state.mode = 'hint'; }
      return;
    }

    // Перший фікс будь-якої якості задає початкове коло пошуку: груба позиція від
    // мережі накриває багато доріг, але фільтр звужує варіанти вже за кілька поворотів.
    if (!this.pf.p.length) {
      this.state.hint = null;
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
      this.lastFixWall = this.now();
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
    const prevGood = this.lastGoodGps;
    this.gpsTrust = Math.min(1, this.gpsTrust + 0.25);
    this.state.spoof = false; this.state.spoofByNet = false;
    this.state.gps = kind === 'gnss' ? 'ok' : 'weak';
    this.lastGoodGps = { t: fix.t, xy };
    this.lastFixWall = this.now();
    if (kind === 'gnss') { this.lastGnssWall = this.lastFixWall; this.state.source = 'gps'; }

    // Поки GPS живий — він веде. Тягнемо хмару частинок щільно до фікса,
    // щоб позиція була саме GPS-ом, прив'язаним до дороги, а не компромісом із датчиками.
    if (this.gpsTrust >= 0.5) {
      const sigma = kind === 'gnss' ? Math.max(12, acc * 0.6) : Math.max(25, acc);
      this.pf.weightByPosition(xy, sigma);
      if (kind === 'gnss' && acc <= 25 && dist(xy, this.state.xy || xy) > Math.max(60, acc * 3)) {
        // далеко відійшли від GPS — пересіваємо, а не повземо до нього хвилину
        this.pf.reseedAround(xy, Math.max(40, acc * 2), 0.7);
      }
    }
    // Поки GPS веде — показуємо саме його, прив'язаного до найближчої дороги.
    // Раніше на екран ішло середнє по хмарі частинок, а воно доганяє фікс кілька
    // секунд: саме через це рух виглядав із затримкою.
    if (kind === 'gnss' && acc <= 60) {
      const hdg = (fix.heading != null && fix.heading >= 0 && isFinite(fix.heading) && (fix.speed ?? 0) > 1.5)
        ? fix.heading * Math.PI / 180 : null;
      // Ребра напрямлені, і найближче з них може дивитися назад. Якщо знаємо курс —
      // вибираємо те, що з ним узгоджене, інакше точка «поїде» задом по своїй же вулиці.
      let best = null;
      for (const c of this.g.edgesNear(xy, Math.max(35, acc * 1.5))) {
        let cost = c.d;
        if (hdg !== null) {
          let d = c.edge.hdg - hdg;
          while (d > Math.PI) d -= 2 * Math.PI;
          while (d <= -Math.PI) d += 2 * Math.PI;
          cost += Math.abs(d) * 60;
        }
        if (!best || cost < best.cost) best = { cost, c };
      }
      const m = best && best.c;
      this.gpsSnap = m
        ? { xy: m.point.slice(), e: m.edge.id, s: m.s, len: m.edge.len, hdg, acc, t: this.lastFixWall }
        : { xy: xy.slice(), e: -1, s: 0, len: 0, hdg, acc, t: this.lastFixWall };
    }
    // Курс від GPS знімає двозначність «туди/назад» по одній вулиці.
    if (fix.heading != null && fix.heading >= 0 && isFinite(fix.heading) && (fix.speed ?? 0) > 2.5) {
      this.pf.weightByHeading(fix.heading * Math.PI / 180, 0.5);
    }
    // Швидкість: беремо GPS-ову напряму. Інтегрування прискорення — лише
    // підміна на час, коли GPS немає; тримати його як основне джерело не можна.
    let vGps = (fix.speed != null && fix.speed >= 0 && isFinite(fix.speed)) ? fix.speed : null;
    // Не кожен пристрій дає coords.speed. Тоді беремо її з відстані між фіксами —
    // інакше на екрані 0 км/год, хоча точка впевнено їде.
    if (vGps === null && prevGood) {
      const dtf = fix.t - prevGood.t;
      if (dtf > 0.3 && dtf < 10) {
        const v = dist(xy, prevGood.xy) / dtf;
        if (v < 70) vGps = v;
      }
    }
    if (vGps !== null) {
      this.motion.setSpeed(vGps);
      this.state.speed = vGps;
    }
    this._updateEstimate();
  }

  now() { return this._now(); }

  // Хто веде зараз. Поки є свіжий супутниковий фікс — GPS. Зник або підміняють —
  // переходимо на датчики + карту. Викликається з таймера UI кілька разів на секунду.
  updateSource() {
    const age = this.now() - this.lastGnssWall;
    const hadGnss = this.lastGnssWall > 0;
    if (this.state.spoof || this.state.spoofByNet) {
      if (this.state.source === 'gps') this.motion.coastFrom(this.state.speed);
      this.state.source = 'dr';
      this.state.gps = 'rejected';
      return 'dr';
    }
    // Коли авто стоїть, система може не надсилати нових фіксів — і це не РЕБ.
    // Без цієї перевірки на припаркованій машині писало б «GPS глушиться».
    if (hadGnss && age > this.gpsLostAfter && this.motion.stationary && this.state.source === 'gps') {
      return 'gps';
    }
    if (hadGnss && age <= this.gpsLostAfter) {
      this.state.source = 'gps';
      if (this.state.gps === 'jammed') this.state.gps = 'ok';
    } else {
      const was = this.state.source;
      this.state.source = this.pf.p.length ? 'dr' : 'none';
      if (hadGnss && this.state.gps === 'ok') this.state.gps = 'jammed';
      // Щойно втратили GPS: авто майже напевно їде далі з тією ж швидкістю.
      if (was === 'gps' && this.state.source === 'dr') this.motion.coastFrom(this.state.speed);
    }
    return this.state.source;
  }

  // Сумісність зі старим викликом
  noteGpsSilence() { return this.updateSource(); }

  // ---------- ручні корекції ----------
  setPositionLatLon(lat, lon, r = 80) {
    const xy = this.proj.toXY(lat, lon);
    this.setPositionXY(xy, r);
  }
  setPositionXY(xy, r = 80) {
    this.state.hint = null;
    this.pf.initAround(xy, r);
    this.state.mode = 'tracking';
    this._updateEstimate();
  }
  // «Я на цьому перехресті»
  setAtNode(nodeId, r = 40) { this.setPositionXY(this.g.nodeXY(nodeId), r); }

  flipTurns() { this.motion.flipGravitySign(); }

  startRecording(opts = {}) {
    if (opts.maxSamples) this.maxSamples = opts.maxSamples;
    const n = this.maxSamples;
    this._rec = { n, i: 0, wrapped: false, t: new Float64Array(n), a: new Float32Array(n * 3), g: new Float32Array(n * 3) };
    this.fixLog = [];
    this.recording = true;
  }

  _recordMotion(s) {
    const r = this._rec; if (!r) return;
    const i = r.i;
    r.t[i] = s.t;
    r.a[i * 3] = s.acc[0]; r.a[i * 3 + 1] = s.acc[1]; r.a[i * 3 + 2] = s.acc[2];
    r.g[i * 3] = s.gyro[0]; r.g[i * 3 + 1] = s.gyro[1]; r.g[i * 3 + 2] = s.gyro[2];
    r.i = i + 1;
    if (r.i >= r.n) { r.i = 0; r.wrapped = true; }
  }

  get recordedSamples() { const r = this._rec; return r ? (r.wrapped ? r.n : r.i) : 0; }

  // Той самий формат, що читає tools/replay: [{k:'m',t,acc,gyro}, {k:'g',...}] у часовому порядку
  exportLog() {
    const r = this._rec, out = [];
    if (r) {
      const count = r.wrapped ? r.n : r.i;
      const start = r.wrapped ? r.i : 0;
      for (let k = 0; k < count; k++) {
        const i = (start + k) % r.n;
        out.push({ k: 'm', t: +r.t[i].toFixed(3), acc: [r.a[i * 3], r.a[i * 3 + 1], r.a[i * 3 + 2]].map(x => +x.toFixed(4)), gyro: [r.g[i * 3], r.g[i * 3 + 1], r.g[i * 3 + 2]].map(x => +x.toFixed(5)) });
      }
    }
    for (const f of this.fixLog) out.push(f);
    return out;
  }

  stopRecording() { this.recording = false; return this.exportLog(); }
  clearRecording() { this._rec = null; this.fixLog = []; if (this.recording) this.startRecording(); }

  // Викликається з циклу малювання. Між фіксами точку треба вести самим, інакше
  // рух виглядає стрибками: на телефоні фікси приходять раз на секунду, а коли iOS
  // притримує застосунок — і рідше.
  tick() {
    const t = this.now();
    if (this._lastTick) this._advanceSnap(Math.min(1.5, t - this._lastTick));
    this._lastTick = t;
    return this.state;
  }

  // Просування прив'язки між фіксами: GPS приходить раз на секунду, а точка має
  // їхати рівно, а не стрибати на 11 метрів.
  _advanceSnap(dt) {
    const g = this.gpsSnap;
    if (!g || g.e < 0 || !(dt > 0) || this.state.source !== 'gps') return;
    if (this.now() - g.t > 8) return;
    const ds = this.state.speed * dt;
    if (!(ds > 0)) return;
    g.s = Math.min(g.len, g.s + ds);
    g.xy = this.g.pointOnEdge(g.e, g.s);
    this.state.xy = g.xy;
    this.state.latlon = this.proj.toLL(g.xy[0], g.xy[1]);
    this.state.s = g.s;
  }

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
    this.state.mode = (this.motion.ready || this.state.source === 'gps') ? 'tracking' : 'calibrating';

    // У режимі GPS малюємо фікс, а не хмару — хмара лишається напоготові для офлайну.
    const g = this.gpsSnap;
    if (g && this.state.source === 'gps' && this.now() - g.t < 8) {
      this.state.xy = g.xy;
      this.state.latlon = this.proj.toLL(g.xy[0], g.xy[1]);
      if (g.hdg !== null) this.state.heading = g.hdg;
      else if (g.e >= 0) this.state.heading = this.g.hdg[g.e];
      this.state.spread = Math.max(6, g.acc);
      this.state.confidence = Math.max(this.state.confidence, 0.9);
      if (g.e >= 0) { this.state.edge = g.e; this.state.s = g.s; }
    }
  }
}
