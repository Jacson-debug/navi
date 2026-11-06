// «Пам'ять пробок»: застосунок запам'ятовує, з якою середньою швидкістю ви реально проїжджали
// кожну ділянку в різні години (будні/вихідні окремо). Працює повністю офлайн.
// Це ТИПОВА картина за вашими поїздками, а не стан доріг прямо зараз.

// Вільна швидкість за класом дороги (км/год) — коли спостережень ще немає
const FREEFLOW = { motorway: 90, trunk: 70, primary: 50, secondary: 45, tertiary: 40, unclassified: 35, residential: 30, living_street: 15, service: 20, road: 30 };
export const VMAX = 25; // м/с, для евристики A* (90 км/год)

export class TrafficMemory {
  constructor(graphKey, storageKey = 'navi.traffic') {
    this.storageKey = storageKey;
    this.graphKey = graphKey;
    this.data = {};       // edgeId -> { slot -> [секунди, метри] }
    this.dirty = false;
    this._load();
  }

  _load() {
    try {
      const raw = JSON.parse(localStorage.getItem(this.storageKey) || 'null');
      if (raw && raw.graphKey === this.graphKey) this.data = raw.e || {};
    } catch { this.data = {}; }
  }
  save() {
    if (!this.dirty) return;
    try { localStorage.setItem(this.storageKey, JSON.stringify({ v: 1, graphKey: this.graphKey, e: this.data })); this.dirty = false; } catch { }
  }
  clear() { this.data = {}; this.dirty = true; this.save(); }
  get hasData() { for (const k in this.data) return true; return false; }

  // Часовий слот: година доби (0..23) + 24 для вихідних
  static slot(date = new Date()) {
    const d = date.getDay();
    return date.getHours() + ((d === 0 || d === 6) ? 24 : 0);
  }

  // Спостереження: на ребрі edgeId провели seconds секунд і проїхали meters метрів
  observe(edgeId, seconds, meters, date = new Date()) {
    if (seconds < 1 || !isFinite(meters)) return;
    const s = TrafficMemory.slot(date);
    const rec = this.data[edgeId] || (this.data[edgeId] = {});
    const cell = rec[s] || (rec[s] = [0, 0]);
    // експоненційне забування: старі поїздки важать менше за нові (вага ~ останні 5 поїздок)
    const k = cell[0] > 300 ? 0.8 : 1;
    cell[0] = cell[0] * k + seconds; cell[1] = cell[1] * k + meters;
    this.dirty = true;
  }

  freeflow(edge) { return (FREEFLOW[edge.cls] ?? 35) / 3.6; }

  // Очікувана швидкість (м/с) на ребрі в цьому слоті; сусідні години теж рахуються з меншою вагою
  expected(edge, slot = TrafficMemory.slot()) {
    const rec = this.data[edge.id];
    if (!rec) return null;
    let t = 0, m = 0;
    const base = slot >= 24 ? 24 : 0, h = slot - base;
    for (const [dh, w] of [[0, 1], [-1, 0.4], [1, 0.4]]) {
      const c = rec[base + ((h + dh + 24) % 24)];
      if (c) { t += c[0] * w; m += c[1] * w; }
    }
    if (t < 20) return null;           // замало даних
    return Math.max(0.3, m / t);
  }

  // Співвідношення до вільної швидкості: 1 = вільно, 0.3 = стоїмо. null — даних немає
  ratio(edge, slot) {
    const v = this.expected(edge, slot);
    return v === null ? null : Math.min(1, v / this.freeflow(edge));
  }

  // Функція швидкості для маршрутизації за часом
  speedFn(slot = TrafficMemory.slot()) {
    return (edge) => this.expected(edge, slot) ?? this.freeflow(edge);
  }

  stats() {
    let edges = 0, secs = 0;
    for (const id in this.data) { edges++; for (const s in this.data[id]) secs += this.data[id][s][0]; }
    return { edges, hours: secs / 3600 };
  }
}

// Колір для шару пробок
export function trafficColor(r) {
  if (r === null) return null;
  if (r >= 0.65) return '#2ec27e';
  if (r >= 0.4) return '#f5c211';
  if (r >= 0.2) return '#ff6b35';
  return '#e01b24';
}
