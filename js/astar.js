// Пошук маршруту A* по орієнтованому графу доріг.
//
// Чому так складно: граф Києва з областю — 640 тис. вузлів і 1,4 млн ребер. Перша
// версія працювала на Map/Set і мільйонах дрібних масивів; коли ціль виявлялась
// недосяжною (2 % вузлів не з'єднані з рештою), вона обходила ВЕСЬ граф, їла сотні
// мегабайтів і підвішувала телефон, після чого Safari вбивав сторінку. Тепер:
//   • усі робочі масиви типізовані й виділяються один раз на граф;
//   • є стеля на кількість розкритих вузлів — краще чесне «не знайшов», ніж зависання;
//   • швидкість ребра береться за id, без створення об'єктів у гарячому циклі.

// Швидкість за класом дороги (м/с) — для режиму «найшвидший маршрут».
// Порядок класів як у graph.js: service, living_street, residential, road, unclassified,
// tertiary, secondary, primary, trunk, motorway
const FREEFLOW_BY_CLS = new Float32Array(16).fill(9);
FREEFLOW_BY_CLS.set([20, 15, 30, 30, 35, 40, 45, 50, 70, 90].map(v => v / 3.6));

class MinHeap {
  constructor(cap) { this.k = new Float64Array(cap); this.v = new Int32Array(cap); this.n = 0; }
  clear() { this.n = 0; }
  push(key, val) {
    if (this.n >= this.k.length) this._grow();
    let i = this.n++;
    const k = this.k, v = this.v;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (k[p] <= key) break;
      k[i] = k[p]; v[i] = v[p]; i = p;
    }
    k[i] = key; v[i] = val;
  }
  pop() {
    const k = this.k, v = this.v;
    const topV = v[0];
    const n = --this.n;
    if (n > 0) {
      const key = k[n], val = v[n];
      let i = 0;
      for (;;) {
        let l = 2 * i + 1;
        if (l >= n) break;
        const r = l + 1;
        if (r < n && k[r] < k[l]) l = r;
        if (k[l] >= key) break;
        k[i] = k[l]; v[i] = v[l]; i = l;
      }
      k[i] = key; v[i] = val;
    }
    return topV;
  }
  _grow() {
    const k = new Float64Array(this.k.length * 2), v = new Int32Array(this.v.length * 2);
    k.set(this.k); v.set(this.v); this.k = k; this.v = v;
  }
}

// Робочі буфери живуть на графі й перевикористовуються між викликами (~10 МБ один раз).
export function prepareRouting(graph) { return scratch(graph); }
function scratch(graph) {
  if (graph._astar) return graph._astar;
  const n = graph.nNodes;
  return (graph._astar = {
    g: new Float32Array(n),       // вартість від старту
    prev: new Int32Array(n),      // ребро, яким прийшли
    stamp: new Int32Array(n),     // «покоління» запису — щоб не чистити масиви щоразу
    closed: new Int32Array(n),    // Int32, а не Uint32: JavaScriptCore не любить uint32 > 2^31 у гарячому циклі
    gen: 0,
    heap: new MinHeap(1 << 16),
  });
}

/**
 * route(graph, fromNode, toNode, speedById?, vmax?, opts?)
 *   speedById(eid) → м/с. Якщо задано — найшвидший маршрут, інакше найкоротший.
 *   opts.maxExpand — стеля розкритих вузлів (типово 300 000).
 * Повертає { nodes, edges, length, time, expanded, capped } або null.
 */
// Внутрішній крок: спільний для синхронної та покрокової версій.
function _prepare(graph, fromNode, toNode, speedById, vmax, maxExpand) {
  const S = scratch(graph);
  if (S.gen >= 0x7ffffff0) { S.stamp.fill(0); S.closed.fill(0); S.gen = 0; }
  const gen = ++S.gen;
  S.heap.clear();
  const X = graph.nodeX, Y = graph.nodeY;
  const gx = X[toNode], gy = Y[toNode];
  const hdiv = speedById ? vmax : 1;
  const h = (node) => { const dx = X[node] - gx, dy = Y[node] - gy; return Math.sqrt(dx * dx + dy * dy) / hdiv; };
  // режим швидкості: 0 — найкоротший; 1 — вільна швидкість за класом + Map «пам'яті пробок»; 2 — довільна функція
  let mode = 0, ov = null, custom = null;
  if (speedById) { if (speedById.ff) { mode = 1; ov = speedById.ov || null; } else { mode = 2; custom = speedById; } }
  S.stamp[fromNode] = gen; S.g[fromNode] = 0; S.prev[fromNode] = -1;
  S.heap.push(h(fromNode), fromNode);
  return { S, gen, h, mode, ov, custom, hdiv, gx, gy, h0: h(fromNode), expanded: 0, found: false, capped: false, maxExpand, fromNode, toNode, speedById, graph, bestH: h(fromNode) };
}

// Виконує до `budget` розкриттів. Повертає true, коли пошук завершено.
function _step(st, budget) {
  const { S, gen, h, graph, toNode, mode, ov, custom } = st;
  const cls = graph.cls, FF = FREEFLOW_BY_CLS;
  const { g, prev, stamp, closed, heap } = S;
  const len = graph.len, B = graph.b, outOff = graph.outOff, outList = graph.outList;
  let n = 0;
  while (heap.n) {
    const u = heap.pop();
    if (u === toNode) { st.found = true; return true; }
    if (closed[u] === gen) continue;
    closed[u] = gen;
    if (++st.expanded > st.maxExpand) { st.capped = true; return true; }
    const gu = g[u];
    // без subarray(): у JavaScriptCore створення view на кожен вузол — помітна ціна
    for (let k = outOff[u], end = outOff[u + 1]; k < end; k++) {
      const eid = outList[k], nb = B[eid];
      if (closed[nb] === gen) continue;
      let sp = 1;
      if (mode === 1) { sp = FF[cls[eid]]; if (ov !== null) { const v = ov.get(eid); if (v !== undefined) sp = v; } }
      else if (mode === 2) sp = Math.max(0.3, custom(eid));
      const ng = mode === 0 ? gu + len[eid] : gu + len[eid] / sp;
      if (stamp[nb] !== gen || ng < g[nb]) {
        stamp[nb] = gen; g[nb] = ng; prev[nb] = eid;
        const hn = h(nb); if (hn < st.bestH) st.bestH = hn;
        heap.push(ng + hn, nb);
      }
    }
    if (++n >= budget) return false;
  }
  return true;
}

// «Холодний» крок. На iPhone Safari гарантовано падає, щойно JIT-компілятор оптимізує гарячий
// цикл пошуку (щоразу після ~3000 розкритих вузлів, без жодної помилки JS). Тому тут цикл
// створюється заново (new Function) на кожні ~200 вузлів: лічильники «гарячості» щоразу з нуля,
// код лишається в інтерпретаторі/базовому JIT і до оптимізатора не доходить. Купа вбудована в цикл.
const COLD_SRC = `
const S = st.S, g = S.g, prev = S.prev, stamp = S.stamp, closed = S.closed, heap = S.heap;
let hk = heap.k, hv = heap.v, hn = heap.n;
const graph = st.graph, len = graph.len, B = graph.b, outOff = graph.outOff, outList = graph.outList;
const X = graph.nodeX, Y = graph.nodeY, cls = graph.cls, FF = FREEFLOW;
const gen = st.gen, toNode = st.toNode, mode = st.mode, ov = st.ov, custom = st.custom, hdiv = st.hdiv, gx = st.gx, gy = st.gy, maxExpand = st.maxExpand;
let expanded = st.expanded, bestH = st.bestH, n = 0, done = true;
while (hn > 0) {
  const u = hv[0]; hn--;
  if (hn > 0) {
    const key = hk[hn], val = hv[hn]; let i = 0;
    for (;;) { let l = 2 * i + 1; if (l >= hn) break; const r = l + 1; if (r < hn && hk[r] < hk[l]) l = r; if (hk[l] >= key) break; hk[i] = hk[l]; hv[i] = hv[l]; i = l; }
    hk[i] = key; hv[i] = val;
  }
  if (u === toNode) { st.found = true; break; }
  if (closed[u] === gen) continue;
  closed[u] = gen;
  if (++expanded > maxExpand) { st.capped = true; break; }
  const gu = g[u];
  for (let k = outOff[u], end = outOff[u + 1]; k < end; k++) {
    const eid = outList[k], nb = B[eid];
    if (closed[nb] === gen) continue;
    let sp = 1;
    if (mode === 1) { sp = FF[cls[eid]]; if (ov !== null) { const v = ov.get(eid); if (v !== undefined) sp = v; } }
    else if (mode === 2) sp = Math.max(0.3, custom(eid));
    const ng = mode === 0 ? gu + len[eid] : gu + len[eid] / sp;
    if (stamp[nb] !== gen || ng < g[nb]) {
      stamp[nb] = gen; g[nb] = ng; prev[nb] = eid;
      const dx = X[nb] - gx, dy = Y[nb] - gy, hh = Math.sqrt(dx * dx + dy * dy) / hdiv;
      if (hh < bestH) bestH = hh;
      if (hn >= hk.length) { const nk = new Float64Array(hk.length * 2), nv = new Int32Array(hv.length * 2); nk.set(hk); nv.set(hv); hk = nk; hv = nv; }
      const pk = ng + hh; let i = hn++;
      while (i > 0) { const p = (i - 1) >> 1; if (hk[p] <= pk) break; hk[i] = hk[p]; hv[i] = hv[p]; i = p; }
      hk[i] = pk; hv[i] = nb;
    }
  }
  if (++n >= budget) { done = false; break; }
}
heap.k = hk; heap.v = hv; heap.n = hn; st.expanded = expanded; st.bestH = bestH;
return done;`;
function _stepCold(st, budget) {
  return new Function('st', 'budget', 'FREEFLOW', COLD_SRC)(st, budget, FREEFLOW_BY_CLS);
}

function _finish(st) {
  const { S, graph, fromNode, toNode, mode, ov, custom, expanded } = st;
  route.lastExpanded = expanded; route.lastCapped = st.capped;
  if (!st.found) return null;
  const len = graph.len, prev = S.prev;
  const edges = [], nodes = [toNode];
  let cur = toNode;
  while (cur !== fromNode) { const eid = prev[cur]; edges.push(eid); cur = graph.a[eid]; nodes.push(cur); }
  edges.reverse(); nodes.reverse();
  let length = 0, time = 0;
  for (const eid of edges) { length += len[eid]; let sp = 11;
    if (mode === 1) { sp = FREEFLOW_BY_CLS[graph.cls[eid]]; if (ov !== null) { const v = ov.get(eid); if (v !== undefined) sp = v; } }
    else if (mode === 2) sp = Math.max(0.3, custom(eid));
    time += len[eid] / sp; }
  return { nodes, edges, length, time, expanded, capped: false };
}

/**
 * route(graph, fromNode, toNode, speedById?, vmax?, opts?) — синхронно (для симуляцій і тестів).
 */
export function route(graph, fromNode, toNode, speedById = null, vmax = 25, opts = {}) {
  if (fromNode === toNode) return { nodes: [fromNode], edges: [], length: 0, time: 0, expanded: 0, capped: false };
  const st = _prepare(graph, fromNode, toNode, speedById, vmax, opts.maxExpand ?? 300000);
  _step(st, Infinity);
  return _finish(st);
}

/**
 * routeAsync(...) — те саме, але шматками по ~sliceMs, не блокуючи інтерфейс.
 * onProgress(0..1) викликається між шматками. opts.timeoutMs — стеля часу (типово 8 с).
 */
export async function routeAsync(graph, fromNode, toNode, speedById = null, vmax = 25, opts = {}, onProgress = null) {
  if (fromNode === toNode) return { nodes: [fromNode], edges: [], length: 0, time: 0, expanded: 0, capped: false };
  const mark = opts.onStage || (() => {});
  mark('prep');
  const st = _prepare(graph, fromNode, toNode, speedById, vmax, opts.maxExpand ?? 300000);
  mark('prep-ok');
  const sliceMs = opts.sliceMs ?? 12, t0 = performance.now(), timeoutMs = opts.timeoutMs ?? 8000;
  let done = false, first = true, steps = 0;
  const trace = opts.trace || null, cold = !!opts.cold;
  while (!done) {
    const ts = performance.now();
    while (!done && performance.now() - ts < sliceMs) {
      if (trace && (++steps <= 12 || (steps & 3) === 0)) trace(st.expanded, st.S.heap.n, steps);
      done = cold ? _stepCold(st, first ? 50 : 200) : _step(st, first ? 50 : 1000);
      if (first) { first = false; mark('step1-ok'); }
    }
    if (done) break;
    if (performance.now() - t0 > timeoutMs) { st.capped = true; break; }
    if (onProgress) onProgress(Math.max(0, Math.min(0.99, 1 - st.bestH / Math.max(1, st.h0))));
    await new Promise(r => setTimeout(r, 0));
  }
  return _finish(st);
}

// Швидкість ребра за id з «пам'яті пробок»: спостереження, якщо є, інакше вільна за класом.
export function speedByIdFrom(graph, traffic, slot) {
  // Вільна швидкість — таблицею за класом дороги; спостереження з «пам'яті пробок» — лише
  // для ребер, де вони є, в Map. Жодних великих масивів на ребро (це десятки МБ на iPhone).
  let ov = null;
  if (traffic && traffic.hasData && !globalThis.NAVI_NO_TRAFFIC) {
    ov = new Map();
    const edge = { id: 0 };
    for (const k in traffic.data) {
      edge.id = k;
      const v = traffic.expected(edge, slot);
      const e = +k;
      if (v !== null && isFinite(v) && e >= 0 && e < graph.n) ov.set(e, Math.max(0.3, v));
    }
  }
  const cls = graph.cls;
  const fn = (eid) => { if (ov !== null) { const v = ov.get(eid); if (v !== undefined) return v; } return FREEFLOW_BY_CLS[cls[eid]]; };
  fn.ff = true; fn.ov = ov;
  return fn;
}

// Перетворює маршрут на список підказок: повороти з відстанями.
export function instructions(graph, r) {
  const out = [];
  let acc = 0;
  for (let i = 0; i < r.edges.length; i++) {
    const e = graph.edge(r.edges[i]);
    acc += e.len;
    const next = i + 1 < r.edges.length ? graph.edge(r.edges[i + 1]) : null;
    if (!next) { out.push({ dist: acc, text: 'Прибуття', turn: 'end' }); break; }
    let dh = next.hdg - e.hdg;
    while (dh > Math.PI) dh -= 2 * Math.PI; while (dh < -Math.PI) dh += 2 * Math.PI;
    const deg = dh * 180 / Math.PI;
    if (Math.abs(deg) > 30) {
      const turn = deg > 0 ? 'right' : 'left';
      const sharp = Math.abs(deg) > 120 ? 'різко ' : '';
      const name = next.name ? ` на ${next.name}` : '';
      out.push({ dist: acc, text: `${sharp}${turn === 'right' ? 'праворуч' : 'ліворуч'}${name}`, turn });
      acc = 0;
    }
  }
  return out;
}
