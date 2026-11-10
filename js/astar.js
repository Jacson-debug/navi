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
const FREEFLOW_BY_CLS = new Float32Array([20, 15, 30, 30, 35, 40, 45, 50, 70, 90].map(v => v / 3.6));

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
function scratch(graph) {
  if (graph._astar) return graph._astar;
  const n = graph.nNodes;
  return (graph._astar = {
    g: new Float32Array(n),       // вартість від старту
    prev: new Int32Array(n),      // ребро, яким прийшли
    stamp: new Uint32Array(n),    // «покоління» запису — щоб не чистити масиви щоразу
    closed: new Uint32Array(n),
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
export function route(graph, fromNode, toNode, speedById = null, vmax = 25, opts = {}) {
  if (fromNode === toNode) return { nodes: [fromNode], edges: [], length: 0, time: 0, expanded: 0, capped: false };
  const maxExpand = opts.maxExpand ?? 300000;
  const S = scratch(graph);
  if (S.gen >= 0xfffffff0) { S.stamp.fill(0); S.closed.fill(0); S.gen = 0; }
  const gen = ++S.gen;
  const { g, prev, stamp, closed, heap } = S;
  heap.clear();

  const X = graph.nodeX, Y = graph.nodeY;
  const gx = X[toNode], gy = Y[toNode];
  const hdiv = speedById ? vmax : 1;
  const h = (node) => { const dx = X[node] - gx, dy = Y[node] - gy; return Math.sqrt(dx * dx + dy * dy) / hdiv; };

  const len = graph.len, B = graph.b;

  stamp[fromNode] = gen; g[fromNode] = 0; prev[fromNode] = -1;
  heap.push(h(fromNode), fromNode);
  let expanded = 0, found = false, capped = false;

  while (heap.n) {
    const u = heap.pop();
    if (u === toNode) { found = true; break; }
    if (closed[u] === gen) continue;
    closed[u] = gen;
    if (++expanded > maxExpand) { capped = true; break; }
    const gu = g[u];
    const outs = graph.outEdges(u);
    for (let k = 0; k < outs.length; k++) {
      const eid = outs[k], nb = B[eid];
      if (closed[nb] === gen) continue;
      const ng = gu + (speedById ? len[eid] / Math.max(0.3, speedById(eid)) : len[eid]);
      if (stamp[nb] !== gen || ng < g[nb]) {
        stamp[nb] = gen; g[nb] = ng; prev[nb] = eid;
        heap.push(ng + h(nb), nb);
      }
    }
  }
  if (!found) { route.lastExpanded = expanded; route.lastCapped = capped; return null; }

  const edges = [], nodes = [toNode];
  let cur = toNode;
  while (cur !== fromNode) { const eid = prev[cur]; edges.push(eid); cur = graph.a[eid]; nodes.push(cur); }
  edges.reverse(); nodes.reverse();
  let length = 0, time = 0;
  for (const eid of edges) { length += len[eid]; time += speedById ? len[eid] / Math.max(0.3, speedById(eid)) : len[eid] / 11; }
  route.lastExpanded = expanded; route.lastCapped = false;
  return { nodes, edges, length, time, expanded, capped: false };
}

// Швидкість ребра за id з «пам'яті пробок»: спостереження, якщо є, інакше вільна за класом.
export function speedByIdFrom(graph, traffic, slot) {
  const cls = graph.cls;
  const hasData = !!(traffic && traffic.hasData);
  return (eid) => {
    if (hasData && traffic.data[eid]) {
      const v = traffic.expected({ id: eid }, slot);
      if (v !== null) return v;
    }
    return FREEFLOW_BY_CLS[cls[eid]] || 9;
  };
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
