// Пошук маршруту A* по орієнтованому графу доріг.
import { dist } from './geo.js';

class MinHeap {
  constructor() { this.a = []; }
  push(k, v) { this.a.push([k, v]); this._up(this.a.length - 1); }
  pop() {
    const top = this.a[0], last = this.a.pop();
    if (this.a.length) { this.a[0] = last; this._down(0); }
    return top;
  }
  get size() { return this.a.length; }
  _up(i) { const a = this.a; while (i > 0) { const p = (i - 1) >> 1; if (a[p][0] <= a[i][0]) break; [a[p], a[i]] = [a[i], a[p]]; i = p; } }
  _down(i) { const a = this.a, n = a.length; for (;;) { let l = 2 * i + 1, r = l + 1, m = i; if (l < n && a[l][0] < a[m][0]) m = l; if (r < n && a[r][0] < a[m][0]) m = r; if (m === i) break; [a[m], a[i]] = [a[i], a[m]]; i = m; } }
}

// Повертає { nodes:[...], edges:[...], length (м), time (с) } або null.
// speedFn(edge) → м/с: якщо задано, шукаємо найшвидший маршрут, інакше найкоротший.
export function route(graph, fromNode, toNode, speedFn = null, vmax = 25) {
  const goal = graph.nodeXY(toNode);
  const cost = speedFn ? (e) => e.len / Math.max(0.3, speedFn(e)) : (e) => e.len;
  const h = speedFn ? (p) => dist(p, goal) / vmax : (p) => dist(p, goal);
  const g = new Map([[fromNode, 0]]);
  const prev = new Map();
  const open = new MinHeap();
  open.push(h(graph.nodeXY(fromNode)), fromNode);
  const closed = new Set();
  while (open.size) {
    const [, u] = open.pop();
    if (u === toNode) break;
    if (closed.has(u)) continue;
    closed.add(u);
    const outs = graph.outEdges(u), gu = g.get(u);
    for (let k = 0; k < outs.length; k++) {
      const eid = outs[k], nb = graph.b[eid];
      const ng = gu + (speedFn ? graph.len[eid] / Math.max(0.3, speedFn(graph.edge(eid))) : graph.len[eid]);
      if (ng < (g.get(nb) ?? Infinity)) {
        g.set(nb, ng); prev.set(nb, eid);
        open.push(ng + h(graph.nodeXY(nb)), nb);
      }
    }
  }
  if (!g.has(toNode)) return null;
  const edges = [], nodes = [toNode];
  let cur = toNode;
  while (cur !== fromNode) { const eid = prev.get(cur); edges.push(eid); cur = graph.a[eid]; nodes.push(cur); }
  edges.reverse(); nodes.reverse();
  let length = 0, time = 0;
  for (const eid of edges) { const e = graph.edge(eid); length += e.len; time += speedFn ? e.len / Math.max(0.3, speedFn(e)) : e.len / 11; }
  return { nodes, edges, length, time };
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
