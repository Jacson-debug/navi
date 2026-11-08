// Обробка датчиків телефону в кріпленні:
//  1) визначення осей авто у системі координат телефону (вгору, вперед, вправо);
//  2) швидкість повороту навколо вертикалі (yaw rate);
//  3) оцінка швидкості без GPS: інтегрування поздовжнього прискорення
//     з обнуленням на зупинках і незалежною оцінкою з поворотів (v = a_lat / ω).
//
// Вхід: pushSample({ t (с), acc:[x,y,z] (м/с², разом із гравітацією), gyro:[x,y,z] (рад/с) })
// Вихід після кожного семплу: { dt, speed, yawRate, stationary, ready }

const v3 = {
  dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
  cross: (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]],
  norm: (a) => Math.hypot(a[0], a[1], a[2]),
  scale: (a, k) => [a[0] * k, a[1] * k, a[2] * k],
  sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
  unit(a) { const n = v3.norm(a) || 1; return [a[0] / n, a[1] / n, a[2] / n]; },
};
const lp = (old, val, dt, tau) => { const a = 1 - Math.exp(-dt / tau); return old + (val - old) * a; };

export class MotionProcessor {
  constructor(opts = {}) {
    // gravitySign: -1, якщо датчик показує вектор уздовж гравітації (iOS Safari),
    //              +1, якщо проти неї (специфікація W3C, Android Chrome).
    this.gravitySign = opts.gravitySign ?? -1;
    this.maxSpeed = opts.maxSpeed ?? 36;      // м/с (~130 км/год)
    this.accelScale = opts.accelScale ?? 1.0; // калібрування масштабу (вчиться з GPS)
    this.reset();
  }

  reset() {
    this.t = null;
    this.g = null;          // оцінка вектора гравітації у системі телефону (сирий знак датчика)
    this.up = null;
    this.fwd = null;        // одиничний вектор «вперед» у системі телефону
    this.fwdVotes = [0, 0, 0];
    this.fwdConfidence = 0;
    this.speed = 0;
    this.yawRate = 0;
    this.gyroBias = [0, 0, 0];
    this.accBiasFwd = 0;
    this.stationary = true;
    this.stillTime = 0;
    this._win = [];
    this._innovVec = [0, 0, 0]; this._innovSlow = [0, 0, 0];
    this._aLatF = 0; this._yawF = 0;
    this.gpsMoveUntil = 0;   // GPS щойно бачив рух — не вірити детектору зупинки
    this._aFwdPrev = 0;
    this._brakeT = -1e9;     // коли останнє відчутне гальмування
    this._everMoved = false; // інтегратор бачив справжній рух після останньої зупинки
    this._quietT = 0;        // як довго тримається «тиша»
    this._lvl = 0;           // поточний рівень вібрації (середнє |лінійного прискорення|)
    // Рівень вібрації різний у кожному авто й тримачі, тож поріг зупинки не можна
    // зашити числом — вчимо його з GPS: як трясе на стоянці і як трясе в руху.
    this.lvlParked = null;
    this.lvlMoving = null;
    this.turnSpeedEst = null;
    this.distance = 0;
  }

  get ready() { return !!(this.up && this.fwd); }

  pushSample(s) {
    if (this.t === null) { this.t = s.t; this.g = s.acc.slice(); this.up = v3.unit(v3.scale(this.g, this.gravitySign)); return null; }
    let dt = s.t - this.t;
    if (dt <= 0 || dt > 1) { this.t = s.t; return null; }
    this.t = s.t;

    const gyro = v3.sub(s.gyro, this.gyroBias);

    // --- гравітація: комплементарний фільтр з гіроскопом ---
    // Вектор гравітації у системі телефону обертається разом із телефоном: dg/dt = -ω × g.
    this.g = v3.sub(this.g, v3.scale(v3.cross(gyro, this.g), dt));
    this.g = v3.scale(this.g, 9.81 / (v3.norm(this.g) || 1));
    // Розгін/гальмування — це ПЕРЕХІДНИЙ процес (секунди), а похибка оцінки гравітації — стала.
    // Тому порівнюємо швидке (0.7 с) і повільне (15 с) згладжування нев'язки: велика різниця = авто прискорюється.
    const innov = v3.sub(s.acc, this.g);
    const ai = 1 - Math.exp(-dt / 1.2), as = 1 - Math.exp(-dt / 15);
    this._innovVec = v3.add(v3.scale(this._innovVec, 1 - ai), v3.scale(innov, ai));
    this._innovSlow = v3.add(v3.scale(this._innovSlow, 1 - as), v3.scale(innov, as));
    const yawNow = v3.dot(gyro, this.up);
    const transient = v3.norm(v3.sub(this._innovVec, this._innovSlow)) > 0.4;
    const accelerating = transient || Math.abs(yawNow) > 0.08;
    // акселерометру віримо швидко у спокої, помірно на рівному ході, майже ніяк під час розгону/повороту
    const tau = this.stationary ? 0.5 : (accelerating ? 300 : 10);
    const a = 1 - Math.exp(-dt / tau);
    this.g = v3.add(v3.scale(this.g, 1 - a), v3.scale(s.acc, a));
    this.up = v3.unit(v3.scale(this.g, this.gravitySign));

    // лінійне прискорення без гравітації та його горизонтальна складова
    const lin = v3.sub(s.acc, this.g);
    const linH = v3.sub(lin, v3.scale(this.up, v3.dot(lin, this.up)));

    // --- детектор зупинки: мала дисперсія прискорення і малий гіроскоп протягом ~1.5 с ---
    this._win.push({ a: v3.norm(lin), w: v3.norm(gyro) });
    if (this._win.length > Math.max(5, Math.round(1.0 / dt))) this._win.shift();
    const n = this._win.length;
    const meanA = this._win.reduce((p, c) => p + c.a, 0) / n;
    const varA = this._win.reduce((p, c) => p + (c.a - meanA) ** 2, 0) / n;
    const meanW = this._win.reduce((p, c) => p + c.w, 0) / n;
    // Детектор зупинки спирається на вібрацію, а вона залежить від авто й кріплення:
    // на рівній дорозі з жорстким тримачем рівний хід схожий на стоянку, і швидкість
    // помилково обнулялася. Тому GPS, якщо він щойно бачив рух, має право вето.
    const gpsSaysMoving = this.gpsMoveUntil > 0 && this.t < this.gpsMoveUntil;
    // І головне: щоб стояти, треба було спершу загальмувати. Гальмування — це
    // великий і добре помітний сигнал (−1..−3 м/с²), а рівний хід на гладкому
    // асфальті його не має. Без цієї умови детектор «паркував» авто на ходу.
    // Для акселерометра рівний хід і стоянка однакові — прискорення нуль і там, і там.
    // Відрізняє їх лише трясіння дороги, і саме його РІВЕНЬ, а не розкид: раніше тут
    // стояла дисперсія, яка майже однакова і на стоянці, і на ходу, тому детектор
    // «парковав» авто на рівному ходу й обнуляв швидкість.
    this._lvl = meanA;
    let vibQuiet;
    if (this.lvlParked !== null && this.lvlMoving !== null && this.lvlMoving > this.lvlParked * 2) {
      vibQuiet = meanA < this.lvlParked + 0.35 * (this.lvlMoving - this.lvlParked);
    } else {
      vibQuiet = meanA < 0.12 && varA < 0.05;
    }
    const quiet = n >= 5 && vibQuiet && meanW < 0.05;
    this._quietT = quiet ? this._quietT + dt : 0;
    // Зупинка мусить тривати: короткі «тихі» секунди на рівному ходу її не оголошують.
    const stationaryNow = quiet && !gpsSaysMoving;
    this.stillTime = stationaryNow ? this.stillTime + dt : 0;
    this.stationary = this.stillTime > 1.5;

    if (this.stationary) {
      // калібрування зсувів у спокої
      const k = 1 - Math.exp(-dt / 1.5);
      this.gyroBias = v3.add(v3.scale(this.gyroBias, 1 - k), v3.scale(s.gyro, k));
      if (this.fwd) this.accBiasFwd = this.accBiasFwd * (1 - k) + v3.dot(linH, this.fwd) * k;
      this.speed = 0;
      this.turnSpeedEst = null;
      this._everMoved = false;
    }

    // --- швидкість обертання навколо вертикалі (+ = проти годинникової = ліворуч) ---
    this.yawRate = v3.dot(gyro, this.up);

    // --- напрямок «вперед»: голосування за домінантний горизонтальний напрямок під час розгону/гальмування ---
    const hMag = v3.norm(linH);
    if (!this.stationary && hMag > 1.2 && Math.abs(this.yawRate) < 0.08) {
      const dir = v3.unit(linH);
      // перший розгін із місця задає знак; далі гальмування (протилежний напрямок) голосує за той самий «вперед»
      const sign = this.fwd ? (v3.dot(dir, this.fwd) >= 0 ? 1 : -1) : 1;
      this.fwdVotes = v3.add(this.fwdVotes, v3.scale(dir, hMag * dt * sign));
      const vn = v3.norm(this.fwdVotes);
      if (vn > 2.0) {
        const f = v3.unit(this.fwdVotes);
        this.fwd = v3.unit(v3.sub(f, v3.scale(this.up, v3.dot(f, this.up))));
        this.fwdConfidence = Math.min(1, vn / 10);
      }
    }
    if (!this.fwd) return { dt, speed: 0, yawRate: this.yawRate, stationary: this.stationary, ready: false };

    this.fwd = v3.unit(v3.sub(this.fwd, v3.scale(this.up, v3.dot(this.fwd, this.up))));
    const right = v3.cross(this.fwd, this.up);
    const rawAFwd = v3.dot(linH, this.fwd);
    // На рівному ході середнє поздовжнє прискорення за десятки секунд ≈ 0 (швидкість не росте нескінченно),
    // тож повільно вчимо залишковий зсув (сенсор + «витік» гравітації через похибку вертикалі).
    // Умова «!accelerating» у місті майже не виконується (світлофори, перебудування),
    // тому зсув не вчився і швидкість тікала вгору до стелі. За хвилину руху середнє
    // поздовжнє прискорення все одно ≈ 0, тож вчимо завжди в руху, але повільно.
    // На рівному ходу справжнє середнє прискорення = 0, тож усе, що бачимо, — це зсув
    // (сенсор + «витік» гравітації після розгону). Тому в сталому режимі вчимо швидко:
    // інакше цей залишок інтегрується і швидкість поволі з'їдається до нуля.
    if (!this.stationary && this.speed > 2) this.accBiasFwd = lp(this.accBiasFwd, rawAFwd, dt, accelerating ? 150 : 4);
    let aFwd = (rawAFwd - this.accBiasFwd) * this.accelScale;
    if (!accelerating && Math.abs(aFwd) < 0.06) aFwd = 0;   // мертва зона проти повзучого дрейфу
    const aLat = v3.dot(linH, right);          // + = вправо
    this._aFwdPrev = aFwd;

    // --- швидкість ---
    if (!this.stationary) {
      this.speed = Math.max(0, Math.min(this.maxSpeed, this.speed + aFwd * dt));
      // незалежна оцінка з повороту: доцентрове прискорення a = v·ω.
      // Поворот ліворуч (ω>0) дає прискорення ліворуч (aLat<0): v = -aLat/ω
      this._aLatF = lp(this._aLatF, aLat, dt, 0.5);
      this._yawF = lp(this._yawF, this.yawRate, dt, 0.5);
      if (Math.abs(this._yawF) > 0.25) {
        const vTurn = -this._aLatF / this._yawF;
        if (vTurn > 0.5 && vTurn < this.maxSpeed) {
          this.turnSpeedEst = this.turnSpeedEst === null ? vTurn : lp(this.turnSpeedEst, vTurn, dt, 0.5);
          this.speed = lp(this.speed, this.turnSpeedEst, dt, 1.0);
        }
      }
    }
    this.distance += this.speed * dt;
    return { dt, speed: this.speed, yawRate: this.yawRate, stationary: this.stationary, ready: true, aFwd, aLat };
  }

  // Жорстко задати швидкість (надійний GPS). Інтегратор повністю переставляється,
  // інакше накопичена похибка лишається і повзе далі.
  setSpeed(v) {
    if (!(v >= 0)) return;
    if (v > 2) { this.gpsMoveUntil = (this.t ?? 0) + 6; this.stillTime = 0; this.stationary = false; }
    else if (v < 0.5) { this.gpsMoveUntil = 0; }
    // вчимо рівень вібрації для свого авто й тримача
    const k = 0.06;
    if (v < 0.5) this.lvlParked = this.lvlParked === null ? this._lvl : this.lvlParked * (1 - k) + this._lvl * k;
    else if (v > 5) this.lvlMoving = this.lvlMoving === null ? this._lvl : this.lvlMoving * (1 - k) + this._lvl * k;
    if (this.speed > 1 && v > 1) {
      const ratio = v / this.speed;
      if (ratio > 0.4 && ratio < 2.5) {
        this.accelScale = Math.max(0.5, Math.min(1.8, this.accelScale * (0.97 + 0.03 * ratio)));
      }
    }
    this.speed = Math.min(this.maxSpeed, v);
    this.turnSpeedEst = null;
  }

  // GPS зник (РЕБ): авто майже напевно продовжує рух із тією ж швидкістю.
  // Зупинити точку на карті — гірша з можливих відповідей, тому «котимося» далі.
  coastFrom(v) {
    if (v > 1 && this.speed < v * 0.5) this.speed = v;
    this.gpsMoveUntil = 0;
  }

  // Калібрування масштабу прискорення за надійним GPS
  calibrateWithSpeed(vGps) {
    if (this.stationary || vGps < 3 || this.speed < 1) return;
    const ratio = vGps / this.speed;
    if (ratio > 0.5 && ratio < 2) {
      this.accelScale = this.accelScale * 0.95 + this.accelScale * ratio * 0.05;
      this.accelScale = Math.max(0.6, Math.min(1.6, this.accelScale));
      this.speed = this.speed * 0.5 + vGps * 0.5;
    }
  }

  // Перемикання конвенції знаку (якщо повороти «дзеркальні»)
  flipGravitySign() { this.gravitySign = -this.gravitySign; }
}
