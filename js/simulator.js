// Симулятор поїздки: з маршруту по графу породжує «правду» (позиція, курс, швидкість)
// і сирі дані датчиків телефону, закріпленого в авто під довільним кутом.
// Використовується і в консольних тестах, і в демо-режимі застосунку.
import { route } from './astar.js';
import { randn, wrapAngle, mulberry32, heading as hdgOf, dist } from './geo.js';

// Матриця повороту з кутів (рад): телефон відносно авто (осі авто: x вправо, y вперед, z вгору)
export function rotationMatrix(yaw, pitch, roll) {
  const cy = Math.cos(yaw), sy = Math.sin(yaw), cp = Math.cos(pitch), sp = Math.sin(pitch), cr = Math.cos(roll), sr = Math.sin(roll);
  // R = Rz(yaw)·Rx(pitch)·Ry(roll); стовпці = осі телефону в системі авто
  const Rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]];
  const Rx = [[1, 0, 0], [0, cp, -sp], [0, sp, cp]];
  const Ry = [[cr, 0, sr], [0, 1, 0], [-sr, 0, cr]];
  return mul(mul(Rz, Rx), Ry);
}
function mul(A, B) { const C = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]; for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) for (let k = 0; k < 3; k++) C[i][j] += A[i][k] * B[k][j]; return C; }
// вектор з системи авто в систему телефону: v_phone = R^T v_car
function toPhone(R, v) { return [R[0][0] * v[0] + R[1][0] * v[1] + R[2][0] * v[2], R[0][1] * v[0] + R[1][1] * v[1] + R[2][1] * v[2], R[0][2] * v[0] + R[1][2] * v[1] + R[2][2] * v[2]]; }

export function simulateDrive(graph, fromNode, toNode, opts = {}) {
  const rng = mulberry32(opts.seed ?? 1);
  const hz = opts.hz ?? 50, dt = 1 / hz;
  const cruise = opts.cruise ?? 13, vTurn = 4.5, aMax = 2.2, dMax = 3.0;
  const ios = opts.ios ?? true;            // конвенція знаку акселерометра
  const R = opts.R ?? rotationMatrix(0.4, 1.1, -0.2); // телефон у кріпленні
  const accNoise = opts.accNoise ?? 0.18, gyrNoise = opts.gyrNoise ?? 0.012;
  const accBias = [0.06, -0.04, 0.03].map(b => b * (opts.biasScale ?? 1));
  const gyrBias = [0.002, -0.003, 0.0025].map(b => b * (opts.biasScale ?? 1));
  const stopProb = opts.stopProb ?? 0.35;

  const r = route(graph, fromNode, toNode);
  if (!r) throw new Error('маршрут не знайдено');
  const pts = r.nodes.map(n => graph.nodeXY(n));
  // план: сегменти з курсами, повороти на вузлах, зупинки
  const FREE = { motorway: 25, trunk: 19, primary: 14, secondary: 12.5, tertiary: 11, residential: 8.5, service: 5.5, living_street: 4 };
  const segs = [];
  for (let i = 0; i < pts.length - 1; i++) {
    const ge = graph.edge(r.edges[i]);
    const vmax = Math.min(cruise, FREE[ge.cls] ?? cruise) * (ge.jam ?? 1);
    segs.push({ a: pts[i], b: pts[i + 1], len: dist(pts[i], pts[i + 1]), hdg: hdgOf(pts[i], pts[i + 1]), vmax: Math.max(2, vmax) });
  }
  const stops = segs.map(() => rng() < stopProb ? 2 + rng() * 6 : 0); // зупинка на початку сегмента i (перехрестя)

  const truth = [], samples = [];
  let t = 0, v = 0, si = 0, s = 0; // поточний сегмент і відстань на ньому
  let hdg = segs[0].hdg, pos = pts[0].slice();
  let stopLeft = opts.initialStop ?? 3; // стоїмо на старті
  let turnFrom = null, turnTo = null, turnLen = 0, turnS = 0; // згладжений поворот
  const total = segs.reduce((p, c) => p + c.len, 0);
  let travelled = 0;
  while (si < segs.length && t < 3600) {
    const seg = segs[si];
    const remain = seg.len - s;
    const nextTurn = si + 1 < segs.length ? Math.abs(wrapAngle(segs[si + 1].hdg - seg.hdg)) : 0;
    const needStopNext = si + 1 < segs.length && stops[si + 1] > 0;
    // цільова швидкість: гальмуємо перед поворотом/зупинкою
    let vEnd = si + 1 < segs.length ? (needStopNext ? 0 : (nextTurn > 0.4 ? Math.min(vTurn, segs[si + 1].vmax) : segs[si + 1].vmax)) : 0;
    let vTarget = Math.min(seg.vmax, Math.sqrt(vEnd * vEnd + 2 * dMax * Math.max(0, remain - 3)));
    if (vTarget < 1.5) vTarget = 1.5; // доповзаємо до самого вузла, зупинка відбувається на нім
    let a = 0;
    if (stopLeft > 0) { stopLeft -= dt; v = 0; a = 0; }
    else {
      if (v < vTarget) a = Math.min(aMax, (vTarget - v) / dt);
      else a = Math.max(-dMax, (vTarget - v) / dt);
      v = Math.max(0, v + a * dt);
    }
    // курс: згладжений поворот після вузла протягом ~10 м
    let omega = 0;
    if (turnLen > 0) {
      const step = v * dt;
      const frac = Math.min(1, step / Math.max(1e-6, turnLen - turnS));
      const d = wrapAngle(turnTo - hdg) * frac;
      omega = v > 0.01 ? -d / dt : 0; // гіроскоп: + ліворуч; hdg за годинниковою, тож ω = -dhdg/dt
      hdg = wrapAngle(hdg + d); turnS += step;
      if (turnS >= turnLen - 1e-6) { turnLen = 0; hdg = turnTo; }
    }
    // рух
    const ds = v * dt;
    pos = [pos[0] + Math.sin(hdg) * ds, pos[1] + Math.cos(hdg) * ds];
    s += ds; travelled += ds;
    if (s >= seg.len) {
      s -= seg.len; si++;
      if (si < segs.length) {
        const ns = segs[si];
        const dh = Math.abs(wrapAngle(ns.hdg - hdg));
        if (dh > 0.05) { turnFrom = hdg; turnTo = ns.hdg; turnLen = Math.max(6, 9 * dh); turnS = 0; }
        else hdg = ns.hdg;
        pos = ns.a.slice(); // прив'язка до вузла (прибирає накопичену похибку інтегрування)
        pos = [pos[0] + Math.sin(hdg) * s, pos[1] + Math.cos(hdg) * s];
        if (stops[si] > 0) { stopLeft = stops[si]; v = 0; }
      }
    }
    // --- датчики ---
    // система авто: x вправо, y вперед, z вгору. Доцентрове: поворот ліворуч (ω>0) → прискорення ліворуч (x<0)
    const aCar = [-v * omega, a, 0];
    const f = [aCar[0], aCar[1], aCar[2] + 9.81];           // питома сила (W3C: показ датчика)
    const vib = v > 0.3 ? 0.3 + 0.03 * v : 0;              // вібрація двигуна/дороги росте зі швидкістю
    const fPhone = toPhone(R, f).map((x, i) => x + accBias[i] + (accNoise + vib) * randn(rng));
    const acc = ios ? fPhone.map(x => -x) : fPhone;
    const wCar = [0.02 * randn(rng) * (v > 0.3 ? 1 : 0), 0.02 * randn(rng) * (v > 0.3 ? 1 : 0), omega];
    const gyro = toPhone(R, wCar).map((x, i) => x + gyrBias[i] + gyrNoise * randn(rng));
    samples.push({ t, acc, gyro });
    truth.push({ t, x: pos[0], y: pos[1], v, hdg, seg: si });
    t += dt;
  }
  return { route: r, truth, samples, total, duration: t };
}
