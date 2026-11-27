// Рушій позиціонування: об'єднує обробку датчиків, фільтр по графу, GPS (із захистом від підміни)
// та ручні корекції. Працює однаково на телефоні та в симуляторі.

import { MotionProcessor } from './motion.js';
import { GraphParticleFilter } from './filter.js';
import { makeProjection, dist } from './geo.js';

// Правдоподібна верхня межа швидкості за класом дороги, м/с
// (1 — проїзди й двори … 5 — магістралі). З запасом, щоб не різати реальний рух.
const EDGE_SPEED_CAP = { 1: 11, 2: 19, 3: 22, 4: 26, 5: 36 };
// Типова швидкість руху (м/с) за рівнем дороги — опора, коли виміряної швидкості нема:
// двір/проїзд ~18, житлова ~30, районна ~40, магістраль ~50, швидкісна ~60 км/год (місто, зі світлофорами).
const TYPICAL_SPEED = { 1: 5, 2: 8.5, 3: 11, 4: 13.5, 5: 17 };

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
    // Виміряна швидкість (м/с): GPS (доплер) або з руху між фіксами. Акселерометр швидкість НЕ дає:
    // інтегрування його похибки на справжньому телефоні виганяло показ до стелі 130 км/год, а
    // детектор зупинки скидав у 0 — звідси «то 130, то 0».
    this.vMeas = 0; this.vMeasT = 0; this.vMeasKind = '';
    this._fixClockOffset = null; // різниця між часом фіксів (epoch) і часом рушія
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
    const now = this.now();
    this.lastMotionWall = now;
    if (this.recording) this._recordMotion(sample);
    const r = this.motion.pushSample(sample);
    if (!r || !r.ready) { if (r && !r.ready) this.state.mode = 'calibrating'; return; }
    // Поворот авто — з гіроскопа навколо вертикалі. Якщо телефон крутять у руках — це не поворот.
    if (!r.handled) this.gyroHeading += r.yawRate * r.dt;
    this.state.stopped = r.stationary;
    this.state.handled = !!r.handled;
    const src = this.state.source;
    const gpsFresh = src === 'gps' && now - this.lastGnssWall < 2.5;
    const vBase = this._driveSpeed(now, r.stationary);
    this.pendingDist += vBase * r.dt;
    if (r.vTurn != null) { this._vTurnSum = (this._vTurnSum || 0) + r.vTurn; this._vTurnN = (this._vTurnN || 0) + 1; }
    if (!gpsFresh) {
      const k = src === 'dr' ? Math.max(0.6, Math.min(1.5, this.pf.kMean())) : 1;
      this._showSpeed(vBase * k, r.dt);
    }
    if (this.lastStepT === null) { this.lastStepT = sample.t; this.lastStepHeading = this.gyroHeading; }
    if (sample.t - this.lastStepT >= this.stepInterval) {
      const dpsi = this.gyroHeading - this.lastStepHeading;
      if (this.pf.p.length) {
        this.pf.step(this.pendingDist, dpsi);
        // Швидкість із повороту — незалежний вимір: гіпотези з близьким темпом важчають.
        if (this._vTurnN >= 5 && src === 'dr' && vBase > 1) {
          const vt = this._vTurnSum / this._vTurnN;
          this.pf.weightBySpeed(vBase, vt, Math.max(2, 0.3 * vt));
        }
      }
      this._vTurnSum = 0; this._vTurnN = 0;
      this.pendingDist = 0;
      this.lastStepHeading = this.gyroHeading;
      this.lastStepT = sample.t;
      this._updateEstimate();
    }
  }

  _sensorsLive(now = this.now()) { return !!(this.lastMotionWall && now - this.lastMotionWall < 1.5 && this.motion.ready); }

  // З якою швидкістю (м/с) котити гіпотези зараз.
  _driveSpeed(now, stopped) {
    if (stopped) return 0;
    const src = this.state.source;
    const age = this.vMeasT ? now - this.vMeasT : 1e9;
    if (src === 'gps' && age < 3) return this.vMeas;
    if (src === 'wifi' && age < 15) {
      return this._wifiStill ? 0 : this.vMeas;
    }
    // Наосліп рух вигадувати не можна: без доказу, що авто їде (виміряна швидкість за останні 2 хв
    // або вібрація рівня «їду», вивчена з GPS), точка стоїть. Інакше вона «їхала» 50 км/год, коли
    // телефон лежав в офісі.
    if (!this._movingEvidence(now)) return 0;
    // Остання виміряна швидкість поволі (≈2 хв) переходить у типову для цієї дороги.
    // Кожна частинка ще й множить її на свій k — справжній темп відбирають повороти.
    const lvl = this.state.edge ? this.g.level(this.state.edge.id) : 3;
    const prior = TYPICAL_SPEED[lvl] || 11;
    const last = this.vMeas > 2 ? this.vMeas : prior;
    const w = Math.exp(-age / 120);
    return w * last + (1 - w) * prior;
  }

  // Чи є докази, що авто зараз їде: свіжа виміряна швидкість, або дорожня вібрація з телефоном,
  // що не крутиться в руках (у тримачі крен/тангаж малі), протягом кількох секунд поспіль.
  _movingEvidence(now) {
    if (this.vMeas > 2 && this.vMeasT && now - this.vMeasT < 120) return true;
    if (!this._sensorsLive(now)) return false;
    const m = this.motion;
    const learned = m.lvlParked != null && m.lvlMoving != null && m.lvlMoving > m.lvlParked * 2;
    const vib = learned ? m._lvl > m.lvlParked + 0.5 * (m.lvlMoving - m.lvlParked) : m._lvl > 0.12;
    const steady = (m._tiltF ?? 1) < 0.25 && !this.state.handled;
    if (vib && steady && !m.stationary) { if (!this._vibSince) this._vibSince = now; this._vibLast = now; }
    else if (this._vibLast && now - this._vibLast > 2) this._vibSince = 0;
    if (this._vibSince && now - this._vibSince > 4) this._vibMoveT = now;
    return !!(this._vibMoveT && now - this._vibMoveT < 20);
  }

  // Показ швидкості: плавно і з фізичними межами (авто не розганяється на 50 км/год за секунду).
  _showSpeed(target, dt) {
    const cur = this.state.speed || 0;
    const up = 3.5 * dt, down = 8 * dt;
    let v = target > cur ? Math.min(target, cur + up) : Math.max(target, cur - down);
    if (this.state.edge) {
      const lvl = this.g.level(this.state.edge.id);
      // Без супутників швидкість — оцінка; вона не може бути більшою за реальну для цієї вулиці.
      const cap = this.state.source === 'gps' ? (EDGE_SPEED_CAP[lvl] || 36) : Math.min(EDGE_SPEED_CAP[lvl] || 36, (TYPICAL_SPEED[lvl] || 11) * 1.7);
      v = Math.min(v, cap);
    }
    this.state.speed = Math.max(0, v);
  }

  // Виміряна швидкість від фіксів — з перевіркою на фізику: стрибок 0→130 за 2 с відкидаємо як шум.
  _setMeasuredSpeed(v, kind, trusted = false) {
    if (!(v >= 0) || !isFinite(v)) return;
    const now = this.now();
    // Для сумнівних вимірів (Wi-Fi, слабкий фікс) — медіана трьох останніх за 8 с: поодинокий викид
    // (стрибок фіксу, вигадана iOS швидкість) її не зрушить. Добрий супутниковий фікс — як є, без затримки.
    this._vRaw = (this._vRaw || []).filter(r => now - r[0] < 8); this._vRaw.push([now, v]);
    if (!trusted && this._vRaw.length >= 3) { const l = this._vRaw.slice(-3).map(r => r[1]).sort((a, b) => a - b); v = l[1]; }
    v = Math.min(v, kind === 'gnss' ? 55 : 36);
    if (this.vMeasT && now - this.vMeasT < 10) {
      const dtm = Math.max(0.3, now - this.vMeasT);
      const hi = this.vMeas + 3.5 * dtm + 1.5, lo = this.vMeas - 8 * dtm - 1.5;
      v = Math.max(lo, Math.min(hi, v));
    }
    this.vMeas = Math.max(0, v); this.vMeasT = now; this.vMeasKind = kind;
  }

  // Маршрут як підказка для ведення наосліп (як «тунельний режим» навігаторів).
  setRoute(edges) { this.pf.routeSet = edges && edges.length ? new Set(edges) : null; }

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
    if (isFinite(fix.t)) this._fixClockOffset = fix.t - this.now();
    // Підміна «в Перу»: координата далеко за межами карти — одразу відкидаємо, ні на що не впливає.
    const B = this.g.bounds;
    if (B && (xy[0] < B.minX - 30000 || xy[0] > B.maxX + 30000 || xy[1] < B.minY - 30000 || xy[1] > B.maxY + 30000)) {
      this.state.spoof = true; this.state.gps = 'rejected'; this.state.spoofFar = (this.state.spoofFar || 0) + 1;
      this.gpsTrust = 0; this.lastKind = 'spoof';
      return;
    }
    // Справжній супутниковий фікс iOS приходить зі швидкістю (і курсом, коли їдемо). Wi-Fi-позиція —
    // без них, навіть якщо iOS пише похибку 25–40 м. Так було в журналі тривоги: «±38 м» без швидкості —
    // це Wi-Fi, а не GPS; раніше через них програма перемикалась «GPS ↔ Wi-Fi» і показ стрибав.
    const hasSpeed = fix.speed != null && fix.speed >= 0 && isFinite(fix.speed);
    const kind = (acc <= 15 || (acc <= 40 && hasSpeed)) ? 'gnss' : acc <= 250 ? 'wifi' : acc <= 3000 ? 'net'
      : acc <= 50000 ? 'coarse' : 'junk';
    this.lastKind = kind; this.lastAcc = acc; this.lastAnyWall = this.now();
    if (kind === 'junk') return;
    // iOS під тривогою повертає старі позиції: ту саму координату з новим часом (часто ще й з «розмитою»
    // похибкою 480 → 1300 м) або давню, що вже була кілька хвилин тому. Нової інформації в них нема.
    if (kind !== 'gnss' && kind !== 'coarse') {
      const key = fix.lat.toFixed(6) + ',' + fix.lon.toFixed(6), nowW = this.now();
      this._seen = this._seen || new Map();
      const first = this._seen.get(key);
      if (first !== undefined) {
        // Та сама координата, що й щойно, без «розмивання» похибки — це «ви досі тут» (у приміщенні iOS
        // так і робить). Не новий рух, але й не «сигналу нема»: тримаємо Wi-Fi живим і кажемо «стоїмо».
        const L = this._lastNetFix;
        if (L && L.key === key && acc <= L.acc * 1.3 + 5 && kind === 'wifi' && !this.state.spoof && !this._movingEvidence(nowW)) {
          if (this.lastGoodGps) this.lastGoodGps.t = fix.t;      // місце підтверджене — це не «перерва»
          this.prevCand = { t: fix.t, xy: this.proj.toXY(fix.lat, fix.lon), acc };
          this.lastWifiWall = nowW; this.lastFixWall = nowW; L.acc = Math.min(L.acc, acc);
          this._wifiStill = true; this._setMeasuredSpeed(0, 'wifi');
          if (this.state.source !== 'gps') this.state.source = 'wifi';
          this.state.gps = 'wifi'; this.state.leadAcc = acc;
          return;
        }
        // Щойно відкинута координата повторилась — це другий узгоджений фікс: перевіряємо ще раз.
        if (!(L && L.key === key && this.state.spoof)) {
        // Давня координата, що повернулась, або та сама з «розмитою» похибкою — застаріла: ігноруємо.
        this.state.staleFixes = (this.state.staleFixes || 0) + 1; this.lastFixWall = nowW; return;
        }
      } else {
        this._seen.set(key, nowW);
        this._lastNetFix = { key, acc };
      }
      if (this._seen.size > 3000) { for (const [k, v] of this._seen) { if (nowW - v > 900) this._seen.delete(k); } }
    }

    if (kind === 'coarse') {
      this.state.hint = { xy, acc, t: fix.t };
      if (!this.pf.p.length) { this.state.gps = 'coarse'; this.state.mode = 'hint'; }
      return;
    }

    // Перший фікс будь-якої якості задає початкове коло пошуку.
    if (!this.pf.p.length) {
      this.state.hint = null;
      // Якщо поруч доріг нема (ви в глибині кварталу, у дворі, у великому ТЦ) — розширюємо коло.
      // Раніше тут гіпотез не з'являлося зовсім, і кожен наступний фікс падав з помилкою.
      let ok = false;
      for (const r of [Math.max(40, Math.min(acc * 1.2, 3000)), 150, 400, 1000]) { if (this.pf.initAround(xy, r)) { ok = true; break; } }
      if (!ok) { this.state.hint = { xy, acc: Math.max(acc, 300), t: fix.t }; this.state.mode = 'hint'; return; }
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
        // iOS завищує похибку «веж»: у журналі тривоги центр кола ±1400 м був ~200 м від справжнього місця.
        const rEff = Math.min(acc * 0.6, 600);
        if (!leadFresh && off > rEff) this.pf.reseedAround(xy, Math.min(acc, 700), 0.6);
        if (!leadFresh) this.pf.weightByPosition(xy, Math.min(acc * 0.5, 450));
        this._updateEstimate();
      }
      return;
    }

    // iOS під час глушіння часто віддає ту саму стару Wi-Fi-позицію знову і знову з новим часом.
    // Це не нова інформація: не тягнемо до неї точку і не вважаємо Wi-Fi «живим».

    // --- точний (gnss) або Wi-Fi-фікс: перевіряємо на підміну ---
    let plausible = true;
    if (this.lastGoodGps) {
      const dt = Math.max(0.5, fix.t - this.lastGoodGps.t);
      const maxMove = 35 * dt + acc + 30;   // 126 км/год у середньому — більше в місті не буває
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
      if (!cand || fix.t - cand.t > 15 || dist(xy, cand.xy) > 35 * Math.max(0.5, fix.t - cand.t) + acc + cand.acc + 20) plausible = false;
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
    // Швидкість від iOS (доплер) — лише від хорошого фіксу: під глушінням вона буває вигадана.
    let vFix = (fix.speed != null && fix.speed >= 0 && isFinite(fix.speed) && acc <= 20) ? fix.speed : null;
    // Стоїмо в заторі? Порівнюємо центри Wi-Fi-фіксів першої й другої половини останніх 16 с:
    // поодинокі фікси стрибають на десятки метрів, а середнє — ні. Якщо центр не зсувається — ми стоїмо.
    if (kind === 'wifi') {
      const win = this.hist.filter(h => fix.t - h.t <= 30);
      this._wifiStill = false;
      if (win.length >= 3 && win[win.length - 1].t - win[0].t >= 12) {
        const mid = (win[0].t + win[win.length - 1].t) / 2, a = win.filter(h => h.t <= mid), b = win.filter(h => h.t > mid);
        const c = (arr) => [arr.reduce((p, h) => p + h.xy[0], 0) / arr.length, arr.reduce((p, h) => p + h.xy[1], 0) / arr.length];
        if (a.length && b.length && dist(c(a), c(b)) < Math.max(15, 0.35 * acc)) { this._wifiStill = true; vFix = 0; }
      }
    } else this._wifiStill = false;
    const minSpan = kind === 'gnss' ? 0.3 : 4;
    // для Wi-Fi беремо найдавніший фікс у межах 25 с — шум ±65 м на 5 с дав би фантомні 40 км/год
    let ref = null;
    // Wi-Fi: вікно 15 с (зупинку видно швидко), а якщо фікси рідкі — 30 с, щоб було хоч два виміри.
    const winW = kind === 'gnss' ? 10 : (this.hist.filter(h => fix.t - h.t <= 15).length >= 3 ? 15 : 30);
    for (const h of this.hist) { if (fix.t - h.t <= winW && fix.t - h.t >= minSpan) { ref = h; break; } }
    if (ref) {
      const d = dist(xy, ref.xy), dtf = fix.t - ref.t;
      // Шум двох Wi-Fi-фіксів віднімаємо «в квадратурі»: просте віднімання занижувало швидкість на третину
      // (у журналі тривоги точка повзла 5–15 км/год, коли авто їхало 40).
      const noise = (kind === 'gnss' && hasSpeed) ? 0 : Math.min(acc, 100) * 0.6;   // без доплера — і для «GPS» шум враховуємо
      const dCorr = Math.sqrt(Math.max(0, d * d - noise * noise));
      if (vFix === null) { const v = dCorr / dtf; if (v < 55) vFix = kind === 'gnss' ? v : (this.state.speed > 0 ? 0.4 * this.state.speed + 0.6 * v : v); }
      if (hdg === null && d > Math.max(kind === 'gnss' ? 15 : 30, noise)) hdg = Math.atan2(xy[0] - ref.xy[0], xy[1] - ref.xy[1]);
    }

    if (hdg === null && kind === 'wifi' && this.hist.length >= 3) {
      const old = this.hist.filter(h => fix.t - h.t <= 30);
      if (old.length >= 3) {
        const k = Math.max(1, Math.floor(old.length / 2)), c = (arr) => [arr.reduce((p, h) => p + h.xy[0], 0) / arr.length, arr.reduce((p, h) => p + h.xy[1], 0) / arr.length];
        const a = c(old.slice(0, k)), b = c(old.slice(-k));
        if (dist(a, b) > Math.max(30, acc * 0.5)) hdg = Math.atan2(b[0] - a[0], b[1] - a[1]);
      }
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
        let okDir = true;
        if (hdg !== null) { let d2 = c.edge.hdg - hdg; while (d2 > Math.PI) d2 -= 2 * Math.PI; while (d2 <= -Math.PI) d2 += 2 * Math.PI; okDir = Math.abs(d2) < Math.PI / 2; }
        if (okDir && (c.edge.id === prevE || this.g.a[c.edge.id] === this.g.b[prevE])) cost -= Math.min(30, acc * 0.4);
        else if (this.g.rev[prevE] === c.edge.id) cost += 25;
      }
      if (!best || cost < best.cost) best = { cost, c };
    }
    let m = best && best.c;
    // Wi-Fi: замість стрибка до кожного фіксу — зважуємо прогноз (куди ми доїхали) і фікс за їхньою
    // точністю (одновимірний фільтр Калмана), і лише результат кладемо на дорогу.
    if (kind === 'wifi' && m && this.gpsSnap && this.gpsSnap.e >= 0 && this.state.source === 'wifi' && now - this.gpsSnap.t < this.leadLostAfter) {
      const pred = this.gpsSnap.xy, dtp = now - this.gpsSnap.t;
      const sp = 15 + 3.5 * dtp, sf = Math.max(20, acc * 0.6);
      const dd = dist(pred, m.point);
      // Великий розрив: один фікс може бути викидом (у журналі ±144 м «стрибнув» на 300 м і назад),
      // тож перший раз зсуваємося на третину, а повністю — коли другий фікс підтвердить.
      let K = (sp * sp) / (sp * sp + sf * sf);
      if (dd > 3 * sf + sp) {
        const prevBig = this._bigDisc && fix.t - this._bigDisc.t < 15 && dist(this._bigDisc.xy, xy) < 1.5 * (this._bigDisc.acc + acc) / 2 + 30;
        K = prevBig ? 1 : 0.35;
        this._bigDisc = { t: fix.t, xy, acc };
      } else this._bigDisc = null;
      const bxy = [pred[0] + K * (m.point[0] - pred[0]), pred[1] + K * (m.point[1] - pred[1])];
      let bb = null;
      // Напрямок руху: курс із фіксів або напрям «прогноз → фікс». На двосторонній вулиці це єдине,
      // що відрізняє «туди» від «назад» (у журналі точку інколи несло у зворотний бік).
      const dirH = hdg !== null ? hdg : (dd > 40 ? Math.atan2(m.point[0] - pred[0], m.point[1] - pred[1]) : null);
      for (const c of this.g.edgesNear(bxy, 40)) {
        let cost = c.d, ang = 0;
        if (dirH !== null) { ang = c.edge.hdg - dirH; while (ang > Math.PI) ang -= 2 * Math.PI; while (ang <= -Math.PI) ang += 2 * Math.PI; ang = Math.abs(ang); cost += ang * 40; }
        if (ang < Math.PI / 2 && (c.edge.id === this.gpsSnap.e || this.g.a[c.edge.id] === this.g.b[this.gpsSnap.e] || c.edge.id === m.edge.id)) cost -= 15;
        if (!bb || cost < bb.cost) bb = { cost, c };
      }
      if (bb) m = bb.c;
    }
    this.gpsSnap = m
      ? { xy: m.point.slice(), e: m.edge.id, s: m.s, len: m.edge.len, hdg, acc, t: now, kind }
      : { xy: xy.slice(), e: -1, s: 0, len: 0, hdg, acc, t: now, kind };
    // Фікс приходить із запізненням (iOS ~0.5–1.5 с): одразу «доганяємо» його на цю відстань, інакше
    // точка щоразу відскакує назад і потім знову наздоганяє.
    this._snapLag = Math.max(0, Math.min(2, fix.lag ?? 0));

    if (vFix !== null) {
      if (m && kind !== 'gnss') vFix = Math.min(vFix, EDGE_SPEED_CAP[this.g.level(m.edge.id)] || 30);
      const trusted = kind === 'gnss' && acc <= 15;
      this._setMeasuredSpeed(vFix, kind, trusted);
      // пороги вібрації «стою/їду» вчимо лише від надійної швидкості — інакше вони псуються
      if (trusted) this.motion.setSpeed(this.vMeas);
      if (kind === 'gnss') this.state.speed = this.vMeas; else this._showSpeed(this.vMeas, 1);
      this.coastV = this.vMeas;
      if (kind === 'wifi' && this.pf.kSpread > 0) this.pf.weightBySpeed(this.vMeas || 1, this.vMeas, 4);
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
      if (was === 'gps' || was === 'wifi') this._enterBlind(now);
      this.state.source = this.pf.p.length ? 'dr' : 'none';
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
    if (was !== this.state.source) {
      if (this.state.source === 'dr' && (was === 'gps' || was === 'wifi')) this._enterBlind(now);
      else if (this.state.source === 'gps') this.pf.collapseSpeed();
      else if (this.state.source === 'wifi') this.pf.collapseSpeed();
    }
    return this.state.source;
  }

  // Перехід «наосліп»: авто майже напевно їде далі тим самим темпом; кожна гіпотеза — зі своїм множником.
  _enterBlind(now) {
    this.motion.coastFrom(this.vMeas);
    this.coastV = this.vMeas; this.coastSince = now;
    if (!(this.pf.kSpread > 0)) this.pf.diversifySpeed(0.28);
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
      if (this.vMeas > 0.8) this.pf.step(this.vMeas * dt, 0);
      return;
    }
    if (src !== 'dr' && src !== 'wifi') { this.state.coasting = false; return; }
    // Без датчиків зупинку не видно й рушання теж: котимося лише з ВИМІРЯНОЮ швидкістю
    // (90 с рівно, далі плавно до нуля за ≈4 хв). Вигадувати рух, якого ніхто не бачив, не можна.
    let v = src === 'wifi' ? (now - this.vMeasT < 15 ? this.vMeas : 0) : (this.coastV ?? 0);
    const age = now - (this.coastSince ?? now);
    if (src === 'dr' && age > 90) v *= Math.max(0, 1 - (age - 90) / 150);
    if (this.state.edge) v = Math.min(v, EDGE_SPEED_CAP[this.g.level(this.state.edge.id)] || 30);
    this.state.coasting = src === 'dr';
    if (this.state.mode === 'calibrating') this.state.mode = 'tracking';
    if (!(v > 0.8)) { if (src === 'dr') this.state.speed = 0; return; }
    this._showSpeed(v * (src === 'dr' ? this.pf.kMean() : 1), dt);
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
    // Ручна позиція — якір: підмінені фікси, далекі від неї, тепер не перекинуть точку
    // (раніше перший же «точний» фікс з Перу чи аеропорту затирав ручне «Я тут»).
    const tFix = this.now() + (this._fixClockOffset ?? 0);
    this.lastGoodGps = { t: tFix, xy: xy.slice() };
    this.prevCand = null;
    this.gpsSnap = null;
    if (!(this.lastGnssWall > 0 && this.now() - this.lastGnssWall <= this.gpsLostAfter)) {
      this.state.source = 'dr';
      if (!(this.pf.kSpread > 0)) this.pf.diversifySpeed(0.28);
      this.coastSince = this.now();
    }
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
    if (src === 'wifi' && (this._wifiStill || this.state.stopped)) { this._snapLag = 0; return; }   // стоїмо — точку не штовхаємо
    let ds = this.state.speed * (dt + (this._snapLag || 0));
    this._snapLag = 0;
    if (!(ds > 0)) return;
    // Між рідкими Wi-Fi-фіксами їдемо далі за перехрестя — найпрямішим продовженням.
    let guard = 0;
    while (g.s + ds > g.len && guard++ < 8) {
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
