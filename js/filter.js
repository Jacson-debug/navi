// Фільтр частинок, прив'язаний до графа доріг.
// Частинка = (орієнтоване ребро, відстань від початку, накопичений поворот гіроскопа на цьому ребрі).
// Рух: пройдена відстань = швидкість × dt (+ шум). На вузлі обирається наступне ребро за
// узгодженістю з напрямком, який показує гіроскоп. Абсолютний курс не потрібен:
// гіроскоп дає лише ЗМІНУ курсу, її дрейф «скидається» геометрією кожного ребра.

import { angleDiff, randn, dist } from './geo.js';

export class GraphParticleFilter {
  constructor(graph, opts = {}) {
    this.g = graph;
    this.Nbase = opts.particles ?? 600;
    this.N = this.Nbase;
    this.Nmax = opts.maxParticles ?? 6000;
    this.rng = opts.rng ?? Math.random;
    this.distNoise = opts.distNoise ?? 0.2;   // відносний шум відстані
    this.turnSigma = opts.turnSigma ?? 0.35;   // рад, допуск узгодження курсу на вузлі
    this.straightSigma = opts.straightSigma ?? 0.25; // рад, допуск повороту на прямому ребрі
    this.uturnProb = opts.uturnProb ?? 0.02;
    this.p = [];
    this._snaps = [];
    this.reseedOnSurprise = opts.reseedOnSurprise ?? false;
    this.roughen = opts.roughen ?? false;
  }

  // Ініціалізація навколо точки (м) з радіусом r
  // Розсіює гіпотези по дорогах у радіусі r. Чим більший радіус (груба позиція від
  // веж зв'язку), тим більше потрібно частинок, інакше вони не накриють усі вулиці.
  initAround(xy, r = 150) {
    const cands = this.g.edgesNear(xy, r);
    this.p = [];
    if (!cands.length) return false;
    this.N = Math.max(this.Nbase, Math.min(this.Nmax, Math.round(this.Nbase * r / 120)));
    for (let i = 0; i < this.N; i++) {
      // Випадковий вибір серед кандидатів: послідовний перебір брав би лише перші N
      // ребер у порядку сітки, тобто один кут кола.
      const c = cands[Math.floor(this.rng() * cands.length)];
      const s = Math.max(0, Math.min(c.edge.len, c.s + randn(this.rng) * 15));
      this.p.push({ e: c.edge.id, s, turn: 0, w: Math.exp(-(c.d * c.d) / (2 * (r / 2) ** 2)) });
    }
    this._normalize();
    return true;
  }

  // Коли гіпотези зійшлися, зайві частинки лише їдять батарею — повертаємось до базової кількості
  _shrinkIfConverged() {
    if (this.N <= this.Nbase) return;
    const est = this.estimate();
    if (!est || est.spread > 90 || est.confidence < 0.5) return;
    this.N = this.Nbase;
    this._resample();
  }

  // Повна невизначеність у межах мапи (повільно, але рятує після втрати)
  initEverywhere(center, r = 3000) {
    return this.initAround(center, r);
  }

  // Один крок: пройдена відстань d (м), зміна курсу гіроскопа dψ (рад, + = ліворуч)
  step(d, dpsi) {
    if (!this.p.length) return;
    const g = this.g;
    const wBefore = this.p.reduce((s, q) => s + q.w, 0);
    for (const q of this.p) {
      // гіроскоп: + ліворуч (проти годинникової). Курс ребра hdg: 0=північ, за годинниковою.
      // Тож поворот ліворуч ЗМЕНШУЄ hdg: ожидана зміна hdg = -dψ
      q.turn += -dpsi;
      let rem = d * (1 + this.distNoise * randn(this.rng)) + 0.3 * randn(this.rng);
      if (rem < 0) rem = 0;
      let e = g.edge(q.e);
      let guard = 0;
      while (q.s + rem > e.len && guard++ < 20) {
        rem -= e.len - q.s;
        this._transit(q, 0);   // перехід через вузол: вибір наступного ребра за гіроскопом
        if (q.w === 0) { rem = 0; break; }
        e = g.edge(q.e);
      }
      q.s = Math.min(e.len, q.s + rem);
      // Виразний поворот посеред прямого ребра означає, що оцінка відстані помилилась:
      // авто насправді вже на перехресті. «Прищіпаємо» частинку до ближчого вузла і
      // виконуємо поворот там. Штраф росте з довжиною стрибка.
      if (Math.abs(q.turn) > 0.45 && e.len > 20 && q.s > 8 && q.s < e.len - 8) {
        const toEnd = e.len - q.s, toStart = q.s;
        // куди стрибати — ймовірнісно, щоб обидві гіпотези («відстали» / «перескочили») жили далі
        const pEnd = Math.exp(-toEnd / 120) / (Math.exp(-toEnd / 120) + Math.exp(-toStart / 120));
        if (this.rng() < pEnd) {
          this._snaps.push(toEnd);
          q.w *= Math.exp(-toEnd / 150);
          q.s = e.len;
          this._transit(q, 0);           // пройти вузол із нульовим залишком
          e = g.edge(q.e);
        } else {
          this._snaps.push(-toStart);
          q.w *= Math.exp(-toStart / 150) * 0.5;
          // повертаємось у початковий вузол: шукаємо ребро з того вузла, узгоджене з курсом
          const predicted = e.hdg + q.turn;
          const outs = g.outEdges(e.a);
          let best = null, bd = Infinity;
          for (const oid of outs) { const dd = Math.abs(angleDiff(g.hdg[oid], predicted)); if (dd < bd) { bd = dd; best = oid; } }
          if (best !== null) { q.e = best; q.s = 0; q.turn = angleDiff(predicted, g.hdg[best]); e = g.edge(best); }
        }
      }
      // на прямому ребрі авто не має крутити: штраф за накопичений поворот
      if (e.len > 25) {
        const over = Math.max(0, Math.abs(q.turn) - 0.15);
        q.w *= Math.exp(-(over * over) / (2 * this.straightSigma ** 2));
      }
    }
    // «Здивування»: якщо майже всі частинки отримали штраф, поточна гіпотеза, ймовірно, хибна.
    // Підсіваємо частину частинок на сусідні дороги навколо оцінки, щоб правильна гіпотеза могла відродитись.
    const wAfter = this.p.reduce((s, q) => s + q.w, 0);
    const mult = wAfter / Math.max(1e-12, wBefore);
    this._normalize();
    this._shrinkIfConverged();
    if (this.reseedOnSurprise && mult < 0.25) this._reseed(0.25, 350);
    else if (this.roughen && this.rng() < 0.3) this._reseed(0.01, 200);   // легке постійне «розпушення»
    if (this._neff() < this.N / 3) this._resample();
  }

  // Пересіває частку частинок по дорогах у радіусі r навколо заданої точки.
  // Потрібно, коли незалежне джерело (вежі зв'язку) каже, що ми не там:
  // саме лише зважування не допоможе, якщо всі гіпотези вже зібрались у хибному місці.
  reseedAround(xy, r, fraction = 0.6) {
    const cands = this.g.edgesNear(xy, r);
    if (!cands.length) return false;
    const want = Math.max(this.Nbase, Math.min(this.Nmax, Math.round(this.Nbase * r / 120)));
    if (want > this.N) { // повертаємо різноманіття: добиваємо кількість частинок копіями
      while (this.p.length < want) this.p.push({ ...this.p[Math.floor(this.rng() * this.p.length)] });
      this.N = want;
    }
    const k = Math.max(1, Math.round(this.p.length * fraction));
    const idx = this.p.map((q, i) => i).sort((a, b) => this.p[a].w - this.p[b].w).slice(0, k);
    const wNew = 1 / this.p.length;
    for (const i of idx) {
      const c = cands[Math.floor(this.rng() * cands.length)];
      this.p[i] = { e: c.edge.id, s: this.rng() * c.edge.len, turn: 0, w: wNew };
    }
    this._normalize();
    return true;
  }

  // Переносить частку найлегших частинок на випадкові ребра в радіусі r від поточної оцінки
  _reseed(fraction, r) {
    const est = this.estimate();
    if (!est) return;
    const cands = this.g.edgesNear(est.xy, r);
    if (!cands.length) return;
    const k = Math.max(1, Math.round(this.p.length * fraction));
    const idx = this.p.map((q, i) => i).sort((a, b) => this.p[a].w - this.p[b].w).slice(0, k);
    const wNew = Math.max(1e-6, 0.5 / this.p.length);
    for (const i of idx) {
      const c = cands[Math.floor(this.rng() * cands.length)];
      this.p[i] = { e: c.edge.id, s: this.rng() * c.edge.len, turn: 0, w: wNew };
    }
    this._normalize();
  }

  // Статистика «прищіпок» з останнього кроку: + = частинки відставали (швидкість занижена)
  takeSnapStats() {
    const s = this._snaps; this._snaps = [];
    if (!s.length) return null;
    const mean = s.reduce((p, c) => p + c, 0) / s.length;
    return { mean, count: s.length };
  }

  // Перехід частинки через кінцевий вузол поточного ребра з залишком відстані rem
  _transit(q, rem) {
    const g = this.g;
    let e = g.edge(q.e);
    const predicted = e.hdg + q.turn;
    const outs = g.outEdges(e.b);
    if (!outs.length) { q.w = 0; return; }
    let best = null, bestScore = -Infinity;
    const scores = [];
    for (const oid of outs) {
      const oh = g.hdg[oid];
      const dd = angleDiff(oh, predicted);
      let sc = -(dd * dd) / (2 * this.turnSigma ** 2);
      if (g.b[oid] === e.a) sc += Math.log(this.uturnProb);
      scores.push([oid, sc]);
      if (sc > bestScore) { bestScore = sc; best = oid; }
    }
    let total = 0;
    for (const sc of scores) { sc[1] = Math.exp(sc[1] - bestScore); total += sc[1]; }
    let u = this.rng() * total, chosen = best;
    for (const [oid, pr] of scores) { u -= pr; if (u <= 0) { chosen = oid; break; } }
    const ne = g.edge(chosen);
    const dd = angleDiff(ne.hdg, predicted);
    q.w *= Math.exp(-(dd * dd) / (2 * this.turnSigma ** 2)) + 1e-6;
    q.turn = angleDiff(predicted, ne.hdg);
    q.e = chosen; q.s = Math.min(ne.len, rem);
  }

  // Зважування за зовнішньою позицією (надійний GPS або ручна корекція)
  weightByPosition(xy, sigma) {
    for (const q of this.p) {
      const pos = this.g.pointOnEdge(q.e, q.s);
      const d = dist(pos, xy);
      q.w *= Math.exp(-(d * d) / (2 * sigma * sigma)) + 1e-9;
    }
    this._normalize();
    this._resample();
  }

  _normalize() {
    let s = 0; for (const q of this.p) s += q.w;
    if (s <= 0 || !isFinite(s)) { for (const q of this.p) q.w = 1 / this.p.length; return; }
    for (const q of this.p) q.w /= s;
  }
  _neff() { let s = 0; for (const q of this.p) s += q.w * q.w; return 1 / s; }
  _resample() {
    const n = this.N, out = [];
    const src = this.p.length;
    let u = this.rng() / n, c = this.p[0].w, i = 0;
    for (let m = 0; m < n; m++) {
      while (u > c && i < src - 1) { i++; c += this.p[i].w; }
      const q = this.p[i];
      out.push({ e: q.e, s: q.s, turn: q.turn, w: 1 / n });
      u += 1 / n;
    }
    this.p = out;
  }

  // Оцінка: найважче ребро, зважена позиція на ньому, частка ваги (упевненість)
  estimate() {
    if (!this.p.length) return null;
    const byEdge = new Map();
    for (const q of this.p) {
      let r = byEdge.get(q.e);
      if (!r) byEdge.set(q.e, (r = { w: 0, s: 0 }));
      r.w += q.w; r.s += q.w * q.s;
    }
    let bestE = -1, bestW = 0;
    for (const [e, r] of byEdge) if (r.w > bestW) { bestW = r.w; bestE = e; }
    const r = byEdge.get(bestE);
    const edge = this.g.edge(bestE);
    const s = r.s / r.w;
    const xy = this.g.pointOnEdge(edge, s);
    // розкид усіх частинок навколо оцінки (м)
    let spread = 0;
    for (const q of this.p) spread += q.w * dist(this.g.pointOnEdge(q.e, q.s), xy);
    return { edge, s, xy, heading: edge.hdg, confidence: bestW, spread, alternatives: byEdge.size };
  }
}
