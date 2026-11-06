// Граф доріг на типізованих масивах — Київ і область (1.2 млн орієнтованих ребер) має
// вміщатися в пам'ять iPhone. Джерела: graph.bin (див. tools/graph_bin.py) або JSON тестового міста.
//
// Публічний інтерфейс:
//   g.nodeXY(i) → [x,y];  g.edge(id) → {id,a,b,len,hdg,name,cls,jam,level};  g.outEdges(node) → Int32Array (підмасив)
//   g.pointOnEdge(e, s), g.edgesNear(p, r), g.nearestEdge(p, r), g.nearestNode(p), g.edgesInBox(...)

import { projectOnSegment } from './geo.js';

export const CLASSES = ['service', 'living_street', 'residential', 'road', 'unclassified', 'tertiary', 'secondary', 'primary', 'trunk', 'motorway'];
// Рівень важливості для відмальовування: 1 — дрібні проїзди … 5 — магістралі
export const LEVEL = { service: 1, living_street: 1, residential: 2, road: 2, unclassified: 2, tertiary: 3, secondary: 4, primary: 5, trunk: 5, motorway: 5 };
const LEVEL_BY_CLS = new Uint8Array(CLASSES.map(c => LEVEL[c] || 2));

export class RoadGraph {
  // raw: { origin:[lat,lon], meta, names:[], nodeX:Float32Array, nodeY:Float32Array,
  //        ea:Int32Array, eb:Int32Array, flags:Uint8Array, cls:Uint8Array, nameIdx:Int32Array, jam?:Float32Array }
  constructor(raw) {
    this.origin = raw.origin; this.meta = raw.meta || {}; this.names = raw.names || [];
    this.nodeX = raw.nodeX; this.nodeY = raw.nodeY; this.nNodes = raw.nodeX.length;
    const m = raw.ea.length;
    let md = 0; for (let i = 0; i < m; i++) md += (raw.flags[i] & 1) ? 1 : 2;
    this.n = md;
    this.a = new Int32Array(md); this.b = new Int32Array(md); this.len = new Float32Array(md); this.hdg = new Float32Array(md);
    this.cls = new Uint8Array(md); this.nameIdx = new Int32Array(md); this.jam = raw.jam ? new Float32Array(md) : null; this.rev = new Int32Array(md).fill(-1);
    let k = 0;
    for (let i = 0; i < m; i++) {
      const a = raw.ea[i], b = raw.eb[i], ax = raw.nodeX[a], ay = raw.nodeY[a], bx = raw.nodeX[b], by = raw.nodeY[b];
      const L = Math.hypot(bx - ax, by - ay), h = Math.atan2(bx - ax, by - ay);
      this.a[k] = a; this.b[k] = b; this.len[k] = L; this.hdg[k] = h; this.cls[k] = raw.cls[i]; this.nameIdx[k] = raw.nameIdx[i]; if (this.jam) this.jam[k] = raw.jam[i];
      if (!(raw.flags[i] & 1)) {
        const r = k + 1;
        this.a[r] = b; this.b[r] = a; this.len[r] = L; this.hdg[r] = h > 0 ? h - Math.PI : h + Math.PI; this.cls[r] = raw.cls[i]; this.nameIdx[r] = raw.nameIdx[i]; if (this.jam) this.jam[r] = raw.jam[i];
        this.rev[k] = r; this.rev[r] = k; k += 2;
      } else k += 1;
    }
    this._buildAdjacency();
    this._buildGrid();
  }

  static fromJSON(data) {
    const n = data.nodes.length, m = data.edges.length;
    const nodeX = new Float32Array(n), nodeY = new Float32Array(n);
    for (let i = 0; i < n; i++) { nodeX[i] = data.nodes[i][0]; nodeY[i] = data.nodes[i][1]; }
    const ea = new Int32Array(m), eb = new Int32Array(m), flags = new Uint8Array(m), cls = new Uint8Array(m), nameIdx = new Int32Array(m), jam = new Float32Array(m);
    const names = [], nm = new Map();
    for (let i = 0; i < m; i++) {
      const e = data.edges[i];
      ea[i] = e.a; eb[i] = e.b; flags[i] = e.oneway ? 1 : 0;
      cls[i] = Math.max(0, CLASSES.indexOf(e.cls || 'residential')); jam[i] = e.jam ?? 1;
      if (e.name) { let ix = nm.get(e.name); if (ix === undefined) { ix = names.length; nm.set(e.name, ix); names.push(e.name); } nameIdx[i] = ix; } else nameIdx[i] = -1;
    }
    return new RoadGraph({ origin: data.origin, meta: data.meta, names, nodeX, nodeY, ea, eb, flags, cls, nameIdx, jam });
  }

  static fromBinary(buf) {
    const dv = new DataView(buf);
    if (String.fromCharCode(dv.getUint8(0), dv.getUint8(1), dv.getUint8(2), dv.getUint8(3)) !== 'NAVI') throw new Error('не graph.bin');
    let o = 4;
    const version = dv.getUint32(o, true); o += 4;
    const n = dv.getUint32(o, true); o += 4;
    const m = dv.getUint32(o, true); o += 4;
    const lat0 = dv.getFloat64(o, true); o += 8;
    const lon0 = dv.getFloat64(o, true); o += 8;
    const metaLen = dv.getUint32(o, true); o += 4;
    const meta = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, o, metaLen))); o += metaLen;
    // вирівнювання у файлі не гарантоване — копіюємо у власні масиви
    const take = (T, count) => { const bytes = count * T.BYTES_PER_ELEMENT; const arr = new T(count); new Uint8Array(arr.buffer).set(new Uint8Array(buf, o, bytes)); o += bytes; return arr; };
    const nodeX = take(Float32Array, n), nodeY = take(Float32Array, n);
    const ea = take(Int32Array, m), eb = take(Int32Array, m), flags = take(Uint8Array, m), cls = take(Uint8Array, m), nameIdx = take(Int32Array, m);
    const names = meta.names || []; delete meta.names;
    if (version !== 1) console.warn('graph.bin version', version);
    return new RoadGraph({ origin: [lat0, lon0], meta, names, nodeX, nodeY, ea, eb, flags, cls, nameIdx });
  }

  _buildAdjacency() {
    const n = this.nNodes, m = this.n;
    const off = new Int32Array(n + 1);
    for (let i = 0; i < m; i++) off[this.a[i] + 1]++;
    for (let i = 0; i < n; i++) off[i + 1] += off[i];
    this.outOff = off; this.outList = new Int32Array(m);
    const fill = new Int32Array(n);
    for (let i = 0; i < m; i++) { const a = this.a[i]; this.outList[off[a] + fill[a]++] = i; }
  }

  // Просторовий індекс (CSR): клітинки 200 м; індексуємо лише одне ребро з пари (зворотне = rev)
  _buildGrid() {
    const cell = this.cell = 200;
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    for (let i = 0; i < this.nNodes; i++) { const x = this.nodeX[i], y = this.nodeY[i]; if (x < minX) minX = x; if (x > maxX) maxX = x; if (y < minY) minY = y; if (y > maxY) maxY = y; }
    this.bounds = { minX, minY, maxX, maxY };
    this.gx0 = Math.floor(minX / cell); this.gy0 = Math.floor(minY / cell);
    this.gw = Math.floor(maxX / cell) - this.gx0 + 1; this.gh = Math.floor(maxY / cell) - this.gy0 + 1;
    const nc = this.gw * this.gh, m = this.n;
    const cnt = new Int32Array(nc + 1);
    const visit = (i, fn) => {
      const ax = this.nodeX[this.a[i]], ay = this.nodeY[this.a[i]], bx = this.nodeX[this.b[i]], by = this.nodeY[this.b[i]];
      const x0 = Math.floor(Math.min(ax, bx) / cell) - this.gx0, x1 = Math.floor(Math.max(ax, bx) / cell) - this.gx0;
      const y0 = Math.floor(Math.min(ay, by) / cell) - this.gy0, y1 = Math.floor(Math.max(ay, by) / cell) - this.gy0;
      for (let gx = x0; gx <= x1; gx++) for (let gy = y0; gy <= y1; gy++) fn(gy * this.gw + gx);
    };
    for (let i = 0; i < m; i++) if (this.rev[i] < i) visit(i, c => cnt[c + 1]++);
    for (let c = 0; c < nc; c++) cnt[c + 1] += cnt[c];
    this.gridOff = cnt; this.gridList = new Int32Array(cnt[nc]);
    const fill = new Int32Array(nc);
    for (let i = 0; i < m; i++) if (this.rev[i] < i) visit(i, c => { this.gridList[cnt[c] + fill[c]++] = i; });
    this._seen = new Uint8Array(m);
    // магістралі для дуже дрібного масштабу
    this.major = []; for (let i = 0; i < m; i++) if (this.rev[i] < i && LEVEL_BY_CLS[this.cls[i]] >= 5) this.major.push(i);
  }

  // ---------- доступ ----------
  nodeXY(i) { return [this.nodeX[i], this.nodeY[i]]; }
  edge(id) {
    const c = this.cls[id];
    return { id, a: this.a[id], b: this.b[id], len: this.len[id], hdg: this.hdg[id], name: this.nameIdx[id] >= 0 ? this.names[this.nameIdx[id]] : '', cls: CLASSES[c], jam: this.jam ? this.jam[id] : 1, level: LEVEL_BY_CLS[c] };
  }
  outEdges(node) { return this.outList.subarray(this.outOff[node], this.outOff[node + 1]); }
  isTwoWay(id) { return this.rev[id] >= 0; }
  level(id) { return LEVEL_BY_CLS[this.cls[id]]; }

  pointOnEdge(e, s) {
    const id = typeof e === 'number' ? e : e.id;
    const L = this.len[id], t = L === 0 ? 0 : Math.max(0, Math.min(1, s / L));
    const a = this.a[id], b = this.b[id];
    return [this.nodeX[a] + (this.nodeX[b] - this.nodeX[a]) * t, this.nodeY[a] + (this.nodeY[b] - this.nodeY[a]) * t];
  }

  cellCount(x0, y0, x1, y1) {
    const w = Math.floor(x1 / this.cell) - Math.floor(x0 / this.cell) + 1, h = Math.floor(y1 / this.cell) - Math.floor(y0 / this.cell) + 1;
    return Math.max(0, w) * Math.max(0, h);
  }
  // Масив id «прямих» ребер (без дублікатів) у прямокутнику з рівнем ≥ minLevel
  edgesInBox(x0, y0, x1, y1, minLevel = 1) {
    const out = [], seen = this._seen, cls = this.cls;
    const gx0 = Math.max(0, Math.floor(x0 / this.cell) - this.gx0), gx1 = Math.min(this.gw - 1, Math.floor(x1 / this.cell) - this.gx0);
    const gy0 = Math.max(0, Math.floor(y0 / this.cell) - this.gy0), gy1 = Math.min(this.gh - 1, Math.floor(y1 / this.cell) - this.gy0);
    for (let gy = gy0; gy <= gy1; gy++) for (let gx = gx0; gx <= gx1; gx++) {
      const c = gy * this.gw + gx;
      for (let k = this.gridOff[c], end = this.gridOff[c + 1]; k < end; k++) {
        const id = this.gridList[k];
        if (!seen[id] && LEVEL_BY_CLS[cls[id]] >= minLevel) { seen[id] = 1; out.push(id); }
      }
    }
    for (let i = 0; i < out.length; i++) seen[out[i]] = 0;
    return out;
  }

  // Усі орієнтовані ребра в радіусі r від точки p: [{edge, s, d, point}]
  edgesNear(p, r) {
    const out = [];
    const ids = this.edgesInBox(p[0] - r, p[1] - r, p[0] + r, p[1] + r, 1);
    for (const id of ids) {
      const a = this.a[id], b = this.b[id];
      const pr = projectOnSegment(p, [this.nodeX[a], this.nodeY[a]], [this.nodeX[b], this.nodeY[b]]);
      if (pr.d > r) continue;
      const e = this.edge(id);
      out.push({ edge: e, s: pr.t * e.len, d: pr.d, point: pr.point });
      const rv = this.rev[id];
      if (rv >= 0) { const er = this.edge(rv); out.push({ edge: er, s: (1 - pr.t) * er.len, d: pr.d, point: pr.point }); }
    }
    return out;
  }

  nearestEdge(p, r = 300) {
    let best = null;
    for (const c of this.edgesNear(p, r)) if (!best || c.d < best.d) best = c;
    return best;
  }

  // До k найближчих вузлів у радіусі r на дорогах рівня ≥ minLevel (для надійної маршрутизації)
  nearestNodes(p, r = 400, minLevel = 2, k = 6) {
    const ids = this.edgesInBox(p[0] - r, p[1] - r, p[0] + r, p[1] + r, minLevel);
    const cand = new Map();
    for (const id of ids) for (const nd of [this.a[id], this.b[id]]) if (!cand.has(nd)) cand.set(nd, Math.hypot(this.nodeX[nd] - p[0], this.nodeY[nd] - p[1]));
    return [...cand.entries()].sort((x, y) => x[1] - y[1]).slice(0, k).map(x => x[0]);
  }

  nearestNode(p) {
    for (const r of [300, 1500, 8000]) {
      const ids = this.edgesInBox(p[0] - r, p[1] - r, p[0] + r, p[1] + r, 1);
      let best = -1, bd = Infinity;
      for (const id of ids) { const a = this.a[id], b = this.b[id];
        let d = Math.hypot(this.nodeX[a] - p[0], this.nodeY[a] - p[1]); if (d < bd) { bd = d; best = a; }
        d = Math.hypot(this.nodeX[b] - p[0], this.nodeY[b] - p[1]); if (d < bd) { bd = d; best = b; } }
      if (best >= 0) return best;
    }
    let best = 0, bd = Infinity;
    for (let i = 0; i < this.nNodes; i++) { const d = Math.hypot(this.nodeX[i] - p[0], this.nodeY[i] - p[1]); if (d < bd) { bd = d; best = i; } }
    return best;
  }
}
