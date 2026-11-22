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
    this.gpsLostAfter = opts.gpsLostAfter ?? 5;  // с без GNSS → Wi-Fi або автономний режим
    this.leadLostAfter = opts.leadLostAfter ?? 25; // с без Wi-Fi-фіксу → автономний режим
    this.lastWifiWall = 0;       // останній фікс Wi-Fi/мережа (40–250 м)
    this.hist = [];              // фікси за 40 с: швидкість і курс там, де система їх не дає
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
    this.maxSamples = 90000;    // ~30 хв при 50 Гц (~3 МБ)
    this._rec = null;
    this.fixLog = [];
  }

  // ---------- вхід датчиків ----------
  pushMotion(sample) {
    this.lastMotionWall = this.now();
    if (this.recording) this._recordMotion(sample);
    const r = this.motion.pushSample(sample);
    if (!r || !r.ready) { this.state.mode = r && !r.ready ? 'calibrating' : this.state.mode; return; }
    this.gyroHeading += r.yawRate * r.dt;
    this.pendingDist += r.speed * r.dt;
    if (!(this.state.source === 'gps' && this.now() - this.lastGnssWall < 2.5)) {
      this.state.speed = r.speed;
      // Офлайн карта сама обмежує брехню: по дворовому проїзду 120 км/год не буває.
      if (this.state.source === 'dr' && this.state.edge) {
        const cap = EDGE_SPEED_CAP[this.g.level(this.state.edge.id)] || 30;
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
  // iOS не дає застосунку ні веж зв'язку, ні списку Wi-Fi — лише свою готову позицію через
  // geolocation. Коли супутники заглушені, iOS сама рахує місце по Wi-Fi-точках і вежах і
  // віддає його тим самим каналом, тільки з більшою похибкою. Тому класифікуємо за точністю:
  //   ≤40 м   — супутники (або дуже щільний Wi-Fi): веде, як GPS;
  //   ≤250 м  — Wi-Fi / змішаний: ТЕЖ ВЕДЕ, коли супутників нема (саме це й є «без GPS» у місті);
  //   ≤3 км   — лише вежі зв'язку: тримає від розбігання, коли нічого точнішого немає;
  //   ≤50 км  — «приблизна геопозиція» iOS: лише коло на карті;
  //   більше  — ігноруємо.
  pushGps(fix) {
    if (this.recording) { this.fixLog.push({ k: 'g', ...fix }); if (this.fixLog.length > 20000) this.fixLog.splice(0, 5000); }
    const xy = this.proj.toXY(fix.lat, fix.lon);
    const acc = fix.accuracy ?? 30;
    const kind = acc <= 40 ? 'gnss' : acc <= 250 ? 'wifi' : acc <= 3000 ? 'net'
      : acc <= 50000 ? 'coarse' : 'junk';
    this.lastKind = kind; this.lastAcc = acc; this.lastAnyWall = this.now();
    if (kind === 'junk') return;

    if (kind === 'coarse') {
      this.state.hint = { xy, acc, t: fix.t };
      if (!this.pf.p.length) { this.state.gps = 'coarse'; this.state.mode = 'hint'; }
      return;
    }

    // Перший фікс будь-якої якості задає початкове коло пошуку.
    if (!this.pf.p.length) {
      this.state.hint = null;
      this.pf.initAround(xy, Math.max(40, Math.min(acc * 1.2, 3000)));
      this.state.mode = 'tracking';
      if (kind === 'net') {
        this.state.gps = 'net';
        this.lastNet = { t: fix.t, xy, acc };
        this.state.net = { xy, acc, t: fix.t };
        this.lastFixWall = this.now();
        this._updateEstimate();
        return;
      }
      // gnss / wifi — далі звичайною дорогою: прив'язка до дороги, швидкість, курс
    }

    if (kind === 'net') {
      // Лише вежі: не веде, а не дає розбігтися. Коли нічого точнішого давно немає —
      // пересіваємо гіпотези в колі, щойно власна оцінка з нього вийшла.
      this.lastNet = { t: fix.t, xy, acc };
      this.state.net = { xy, acc, t: fix.t };
      this.lastFixWall = this.now();
      if (this.state.gps !== 'ok' && this.state.gps !== 'wifi') this.state.gps = 'net';
      if (this.state.xy) {
        const leadFresh = this.now() - Math.max(this.lastGnssWall, this.lastWifiWall) < this.leadLostAfter;
        const off = dist(this.state.xy, xy);
        if (!leadFresh && off > acc * 0.6) this.pf.reseedAround(xy, acc, 0.6);
        if (!leadFresh) this.pf.weightByPosition(xy, acc * 0.6);
        this._updateEstimate();
      }
      return;
    }

    // --- точний (gnss) або Wi-Fi-фікс: перевіряємо на підміну ---
    let plausible = true;
    if (this.lastGoodGps) {
      const dt = Math.max(0.5, fix.t - this.lastGoodGps.t);
      const maxMove = 50 * dt + acc + 30;
      if (dist(xy, this.lastGoodGps.xy) > maxMove) plausible = false;
    }
    // Стрибок відносно власної впевненої оцінки — лише поки останній добрий фікс свіжий.
    // Інакше після довгої їзди наосліп застосунок міг назавжди відкидати справжній GPS.
    if (plausible && this.lastGoodGps && fix.t - this.lastGoodGps.t < 30 && this.state.xy && this.state.confidence > 0.5 && this.state.spread < 60) {
      if (dist(xy, this.state.xy) > 250 + 3 * acc) plausible = false;
    }
    // Після довгої перерви (РЕБ) перший же «точний» фікс не приймаємо наосліп: підміна часто
    // стрибає по кілометру щосекунди. Приймаємо, лише коли два фікси поспіль узгоджені між собою.
    const cand = this.prevCand; this.prevCand = { t: fix.t, xy, acc };
    if (plausible && (!this.lastGoodGps || fix.t - this.lastGoodGps.t > 30) && this.pf.p.length && this.state.xy) {
      if (!cand || fix.t - cand.t > 6 || dist(xy, cand.xy) > 50 * Math.max(0.5, fix.t - cand.t) + acc + cand.acc + 20) plausible = false;
    }
    // Перехресна перевірка вежами: РЕБ підміняє супутниковий сигнал, але не вежі зв'язку.
    if (plausible && kind === 'gnss' && this.lastNet && fix.t - this.lastNet.t < 180) {
      if (dist(xy, this.lastNet.xy) > this.lastNet.acc + 3 * acc + 300) { plausible = false; this.state.spoofByNet = true; }
    }
    if (!plausible) {
      this.gpsTrust = Math.max(0, this.gpsTrust - 0.4);
      this.state.spoof = true; this.state.gps = 'rejected';
      return;
    }

    const now = this.now();
    this.gpsTrust = Math.min(1, this.gpsTrust + 0.25);
    this.state.spoof = false; this.state.spoofByNet = false;
    this.state.gps = kind === 'gnss' ? 'ok' : 'wifi';
    this.state.leadAcc = acc;
    this.lastGoodGps = { t: fix.t, xy };
    this.lastFixWall = now;
    if (kind === 'gnss') { this.lastGnssWall = now; this.state.source = 'gps'; }
    else {
      this.lastWifiWall = now;
      if (!(this.lastGnssWall > 0 && now - this.lastGnssWall <= this.gpsLostAfter)) {
        if (this.state.source !== 'wifi') { this.coastV = this.state.speed; this.coastSince = now; }
        this.state.source = 'wifi';
      }
    }

    // Курс: від системи (лише в русі) або з руху між Wi-Fi-фіксами.
    let hdg = (fix.heading != null && fix.heading >= 0 && isFinite(fix.heading) && (fix.speed ?? 0) > 1.5)
      ? fix.heading * Math.PI / 180 : null;

    // Історія фіксів за 40 с — швидкість і курс там, де система їх не дає (Wi-Fi).
    this.hist.push({ t: fix.t, xy });
    while (this.hist.length > 2 && fix.t - this.hist[0].t > 40) this.hist.shift();
    let vFix = (fix.speed != null && fix.speed >= 0 && isFinite(fix.speed)) ? fix.speed : null;
    const minSpan = kind === 'gnss' ? 0.3 : 4;
    // для Wi-Fi беремо найдавніший фікс у межах 25 с — шум ±65 м на 5 с дав би фантомні 40 км/год
    let ref = null;
    for (const h of this.hist) { if (fix.t - h.t <= (kind === 'gnss' ? 10 : 25) && fix.t - h.t >= minSpan) { ref = h; break; } }
    if (ref) {
      const d = dist(xy, ref.xy), dtf = fix.t - ref.t;
      const noise = kind === 'gnss' ? 0 : Math.min(acc, 80) * 0.8;
      if (vFix === null) { const v = Math.max(0, d - noise) / dtf; if (v < 55) vFix = kind === 'gnss' ? v : (this.state.speed > 0 ? 0.5 * this.state.speed + 0.5 * v : v); }
      if (hdg === null && d > Math.max(kind === 'gnss' ? 15 : 30, noise * 1.5)) hdg = Math.atan2(xy[0] - ref.xy[0], xy[1] - ref.xy[1]);
    }

    // Частинки: щільно до фікса; якщо далеко відійшли — пересіваємо, а не повземо хвилину.
    const sigma = kind === 'gnss' ? Math.max(12, acc * 0.6) : Math.max(20, acc * 0.7);
    if (this.gpsTrust >= 0.5 || kind === 'wifi') this.pf.weightByPosition(xy, sigma);
    // Порівнюємо з хмарою частинок, а не з показаною точкою: показана — це сама прив'язка до фіксу,
    // і хмара могла лишитися далеко позаду (тоді при втраті сигналу точка стрибала б назад).
    const far = kind === 'gnss' ? Math.max(60, acc * 3) : Math.max(50, acc);
    const pe = this.pf.estimate();
    if (!pe || (kind === 'gnss' ? (acc <= 25 && dist(xy, this.state.xy || xy) > far) : (dist(xy, pe.xy) > far || pe.spread > acc * 3))) {
      this.pf.reseedAround(xy, kind === 'gnss' ? Math.max(40, acc * 2) : Math.max(35, acc * 0.8), 0.7);
    }
    if (hdg !== null && (fix.speed ?? vFix ?? 0) > 2.5) this.pf.weightByHeading(hdg, kind === 'gnss' ? 0.5 : 0.8);

    // Прив'язка до дороги: показуємо саме її. Неперервність — бонус тому самому ребру й наступним.
    const prevE = this.gpsSnap && this.gpsSnap.e >= 0 ? this.gpsSnap.e : -1;
    let best = null;
    for (const c of this.g.edgesNear(xy, Math.max(35, Math.min(acc * 1.2, 150)))) {
      let cost = c.d;
      if (hdg !== null) {
        let d = c.edge.hdg - hdg;
        while (d > Math.PI) d -= 2 * Math.PI;
        while (d <= -Math.PI) d += 2 * Math.PI;
        cost += Math.abs(d) * (kind === 'gnss' ? 60 : 40);
      }
      if (prevE >= 0 && kind === 'wifi') {
        if (c.edge.id === prevE || this.g.a[c.edge.id] === this.g.b[prevE]) cost -= Math.min(30, acc * 0.4);
        else if (this.g.rev[prevE] === c.edge.id) cost += 25;
      }
      if (!best || cost < best.cost) best = { cost, c };
    }
    const m = best && best.c;
    this.gpsSnap = m
      ? { xy: m.point.slice(), e: m.edge.id, s: m.s, len: m.edge.len, hdg, acc, t: now, kind }
      : { xy: xy.slice(), e: -1, s: 0, len: 0, hdg, acc, t: now, kind };

    if (vFix !== null) {
      if (m && kind !== 'gnss') vFix = Math.min(vFix, EDGE_SPEED_CAP[this.g.level(m.edge.id)] || 30);
      this.motion.setSpeed(vFix);
      this.state.speed = vFix;
      this.coastV = vFix;
    }
    this._updateEstimate();
  }

  now() { return this._now(); }

  // Хто веде зараз:
  //   gps  — свіжий супутниковий фікс;
  //   wifi — супутників нема, але iOS дає позицію по Wi-Fi/вежах з похибкою до 250 м;
  //   dr   — нічого не приходить: датчики руху + карта, а без датчиків — «котимося» з останньою швидкістю.
  updateSource() {
    const now = this.now();
    const was = this.state.source;
    if (this.state.spoof || this.state.spoofByNet) {
      if (was === 'gps' || was === 'wifi') { this.motion.coastFrom(this.state.speed); this.coastV = this.state.speed; this.coastSince = now; }
      this.state.source = 'dr';
      this.state.gps = 'rejected';
      return 'dr';
    }
    const hadGnss = this.lastGnssWall > 0, gAge = now - this.lastGnssWall;
    const wAge = now - this.lastWifiWall;
    const sensorsLive = this.lastMotionWall && now - this.lastMotionWall < 1.5 && this.motion.ready;
    // Коли авто стоїть, система може не надсилати нових фіксів — і це не РЕБ.
    const stopped = sensorsLive ? this.motion.stationary : this.state.speed < 1;
    if (hadGnss && was === 'gps' && gAge > this.gpsLostAfter && gAge < 60 && stopped) return 'gps';

    if (hadGnss && gAge <= this.gpsLostAfter) {
      this.state.source = 'gps';
      if (this.state.gps === 'jammed') this.state.gps = 'ok';
    } else if (this.lastWifiWall > 0 && wAge <= this.leadLostAfter) {
      this.state.source = 'wifi';
      this.state.gps = 'wifi';
    } else {
      this.state.source = this.pf.p.length ? 'dr' : 'none';
      if (this.state.gps === 'ok' || this.state.gps === 'wifi') this.state.gps = hadGnss ? 'jammed' : 'lost';
    }
    if (was !== this.state.source && (was === 'gps' || was === 'wifi') && this.state.source !== 'gps') {
      this.motion.coastFrom(this.state.speed);
      this.coastV = this.state.speed; this.coastSince = now;
    }
    return this.state.source;
  }

  // Рух без датчиків (iOS не дав дозволу або вони мовчать). Без цього між фіксами точка стоїть.
  //   wifi — між Wi-Fi-фіксами їдемо з виміряною по них швидкістю;
  //   dr   — фіксів нема: 90 с рівно з останньою швидкістю, далі плавно до нуля (≈4 хв).
  coastTick() {
    const now = this.now();
    const last = this._lastCoast ?? now; this._lastCoast = now;
    const src = this.state.source;
    if (!this.pf.p.length) { this.state.coasting = false; return; }
    if (this.lastMotionWall && now - this.lastMotionWall < 1.5 && this.motion.ready) { this.state.coasting = false; return; }
    const dt = Math.min(1, now - last); if (!(dt > 0)) return;
    if (src === 'gps') {
      // GPS веде сам, але хмара частинок має їхати разом із ним — вона стане опорою, щойно GPS зникне.
      this.state.coasting = false;
      if (this.state.speed > 0.8) this.pf.step(this.state.speed * dt, 0);
      return;
    }
    if (src !== 'dr' && src !== 'wifi') { this.state.coasting = false; return; }
    let v = src === 'wifi' ? this.state.speed : (this.coastV ?? 0);
    const age = now - (this.coastSince ?? now);
    if (src === 'dr' && age > 90) v *= Math.max(0, 1 - (age - 90) / 150);
    if (this.state.edge) v = Math.min(v, EDGE_SPEED_CAP[this.g.level(this.state.edge.id)] || 30);
    this.state.coasting = src === 'dr';
    if (this.state.mode === 'calibrating') this.state.mode = 'tracking';
    if (!(v > 0.8)) { if (src === 'dr') this.state.speed = 0; return; }
    if (src === 'dr') this.state.speed = v;
    this.pf.step(v * dt, 0);
    this._updateEstimate();
    if (src === 'dr') {
      // Без датчиків ми лише припускаємо, що їдемо далі тим самим темпом: чесно показуємо, що це оцінка.
      this.state.spread = Math.max(this.state.spread, 25 + age * 4);
      this.state.confidence = Math.min(this.state.confidence, 0.4);
    }
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
    const src = this.state.source;
    if (!g || g.e < 0 || !(dt > 0) || (src !== 'gps' && src !== 'wifi')) return;
    if (this.now() - g.t > (src === 'gps' ? 8 : this.leadLostAfter)) return;
    let ds = this.state.speed * dt;
    if (!(ds > 0)) return;
    // Між рідкими Wi-Fi-фіксами їдемо далі за перехрестя — найпрямішим продовженням.
    let guard = 0;
    while (g.s + ds > g.len && src === 'wifi' && guard++ < 8) {
      ds -= g.len - g.s;
      const outs = this.g.outEdges(this.g.b[g.e]);
      let nb = -1, bd = 1e9;
      for (let k = 0; k < outs.length; k++) {
        const e2 = outs[k]; if (e2 === this.g.rev[g.e]) continue;
        let d = this.g.hdg[e2] - this.g.hdg[g.e];
        while (d > Math.PI) d -= 2 * Math.PI;
        while (d <= -Math.PI) d += 2 * Math.PI;
        if (Math.abs(d) < bd) { bd = Math.abs(d); nb = e2; }
      }
      if (nb < 0 || bd > 0.8) { ds = 0; g.s = g.len; break; }   // різкий поворот — чекаємо наступного фіксу
      g.e = nb; g.s = 0; g.len = this.g.len[nb];
    }
    g.s = Math.min(g.len, g.s + ds);
    g.xy = this.g.pointOnEdge(g.e, g.s);
    this.state.xy = g.xy;
    this.state.latlon = this.proj.toLL(g.xy[0], g.xy[1]);
    this.state.edge = this.g.edge(g.e);
    this.state.s = g.s;
    this.state.heading = this.g.hdg[g.e];
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
    this.state.mode = (this.motion.ready || this.state.source === 'gps' || this.state.source === 'wifi' || this.state.coasting) ? 'tracking' : 'calibrating';

    // У режимі GPS малюємо фікс, а не хмару — хмара лишається напоготові для офлайну.
    const g = this.gpsSnap;
    const src = this.state.source;
    if (g && ((src === 'gps' && this.now() - g.t < 5) || (src === 'wifi' && this.now() - g.t < this.leadLostAfter))) {
      this.state.xy = g.xy;
      this.state.latlon = this.proj.toLL(g.xy[0], g.xy[1]);
      if (g.hdg !== null) this.state.heading = g.hdg;
      else if (g.e >= 0) this.state.heading = this.g.hdg[g.e];
      this.state.spread = Math.max(6, g.acc);
      this.state.confidence = Math.max(this.state.confidence, 0.9);
      if (g.e >= 0) { this.state.edge = this.g.edge(g.e); this.state.s = g.s; }
    }
  }
}
