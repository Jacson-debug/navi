// Геометрія: перетворення координат і допоміжні функції.
// Локальна площина в метрах навколо опорної точки (достатньо для області).

export const R = 6371008.8;
const D2R = Math.PI / 180;

// x масштабується за cos(широти точки), а не центру: тоді локальні відстані точні по всій області.
export function makeProjection(lat0, lon0) {
  return {
    lat0, lon0,
    toXY(lat, lon) {
      return [(lon - lon0) * D2R * R * Math.cos(lat * D2R), (lat - lat0) * D2R * R];
    },
    toLL(x, y) {
      const lat = lat0 + y / R / D2R;
      return [lat, lon0 + x / (R * Math.cos(lat * D2R)) / D2R];
    },
  };
}

export function dist(a, b) {
  const dx = a[0] - b[0], dy = a[1] - b[1];
  return Math.hypot(dx, dy);
}

// Азимут напрямку a→b у радіанах: 0 = північ (+y), за годинниковою.
export function heading(a, b) {
  return Math.atan2(b[0] - a[0], b[1] - a[1]);
}

export function wrapAngle(a) {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a <= -Math.PI) a += 2 * Math.PI;
  return a;
}

export function angleDiff(a, b) {
  return wrapAngle(a - b);
}

// Проєкція точки p на відрізок ab: повертає {t, point, d}
export function projectOnSegment(p, a, b) {
  const abx = b[0] - a[0], aby = b[1] - a[1];
  const L2 = abx * abx + aby * aby;
  let t = L2 === 0 ? 0 : ((p[0] - a[0]) * abx + (p[1] - a[1]) * aby) / L2;
  t = Math.max(0, Math.min(1, t));
  const pt = [a[0] + abx * t, a[1] + aby * t];
  return { t, point: pt, d: dist(p, pt) };
}

export function lerp(a, b, t) {
  return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
}

// Випадкове нормальне число (Box–Muller)
export function randn(rng = Math.random) {
  let u = 0, v = 0;
  while (u === 0) u = rng();
  while (v === 0) v = rng();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

// Детермінований ГВЧ для відтворюваних тестів
export function mulberry32(seed) {
  let a = seed >>> 0;
  return function () {
    a |= 0; a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
