// Пошук маршруту у WebAssembly. Чому: на iPhone (Safari, iOS 27) JavaScript-версія циклу A*
// стабільно вбивала сторінку за кілька мілісекунд без жодної помилки — це збій JIT-оптимізатора
// JavaScript. WebAssembly компілюється іншим шляхом, а сам цикл — тривіальний (див. tools/astar.c).
import { WASM_B64 } from './wasm-bin.js';
import { FREEFLOW_BY_CLS } from './astar.js';

// індекси в блоці параметрів — той самий порядок, що в enum у tools/astar.c
const P = { N: 0, M: 1, GEN: 2, HCAP: 3, MAXEXP: 4, FROM: 5, TO: 6, EXPANDED: 7, FOUND: 8, CAPPED: 9, PX: 10, PY: 11, POFF: 12, PLIST: 13, PB: 14, PLEN: 15, PSPD: 16, PG: 17, PPREV: 18, PSTAMP: 19, PCLOSED: 20, PHK: 21, PHV: 22, HDIVBITS: 23, HEAPN: 24, RESUME: 25, BUDGET: 26, BESTBITS: 27, PCLS: 28, PTABLE: 29 };

export function wasmSupported() {
  try { return typeof WebAssembly === 'object' && typeof WebAssembly.instantiate === 'function'; } catch { return false; }
}

export async function createWasmRouter(graph, hooks = {}) {
  const mark = hooks.stage || (() => {});
  const n = graph.nNodes, m = graph.n, HCAP = 1 << 19;
  let off = 1 << 20;
  const take = (bytes) => { const o = off; off += (bytes + 15) & ~15; return o; };
  const o = {
    params: take(256), table: take(64), X: take(n * 4), Y: take(n * 4), OFF: take((n + 1) * 4), LIST: take(m * 4), B: take(m * 4),
    LEN: take(m * 4), SPD: take(m * 4), CLS: take(m), G: take(n * 4), PREV: take(n * 4), STAMP: take(n * 4), CLOSED: take(n * 4),
    HK: take(HCAP * 8), HV: take(HCAP * 4),
  };
  const pages = Math.ceil(off / 65536) + 1;
  mark('wasm:memory');
  const memory = new WebAssembly.Memory({ initial: pages });
  const bin = Uint8Array.from(atob(WASM_B64), (c) => c.charCodeAt(0));
  mark('wasm:compile');
  const { instance } = await WebAssembly.instantiate(bin, { env: { memory } });
  const ex = instance.exports;
  const buf = memory.buffer;
  mark('wasm:copy');
  new Float32Array(buf, o.X, n).set(graph.nodeX); new Float32Array(buf, o.Y, n).set(graph.nodeY);
  new Int32Array(buf, o.OFF, n + 1).set(graph.outOff); new Int32Array(buf, o.LIST, m).set(graph.outList);
  new Int32Array(buf, o.B, m).set(graph.b); new Float32Array(buf, o.LEN, m).set(graph.len); new Uint8Array(buf, o.CLS, m).set(graph.cls);
  new Float32Array(buf, o.table, 16).set(FREEFLOW_BY_CLS);
  const p = new Int32Array(buf, o.params, 64);
  p[P.N] = n; p[P.M] = m; p[P.HCAP] = HCAP;
  p[P.PX] = o.X; p[P.PY] = o.Y; p[P.POFF] = o.OFF; p[P.PLIST] = o.LIST; p[P.PB] = o.B; p[P.PLEN] = o.LEN; p[P.PSPD] = o.SPD;
  p[P.PG] = o.G; p[P.PPREV] = o.PREV; p[P.PSTAMP] = o.STAMP; p[P.PCLOSED] = o.CLOSED; p[P.PHK] = o.HK; p[P.PHV] = o.HV;
  p[P.PCLS] = o.CLS; p[P.PTABLE] = o.table;
  mark('wasm:speeds');
  ex.fill_spd(o.params);
  const spd = new Float32Array(buf, o.SPD, m), prev = new Int32Array(buf, o.PREV, n);
  const stamp = new Int32Array(buf, o.STAMP, n), closed = new Int32Array(buf, o.CLOSED, n);
  const f32 = new Float32Array(1), i32 = new Int32Array(f32.buffer);
  let gen = 0, overridden = [];
  mark('wasm:ready');

  return {
    /**
     * route(from, to, { vmax, maxExpand, ov, budget }) → { nodes, edges, length, time, expanded } | null
     * ov — Map ребро → м/с («пам'ять пробок»); решта ребер їдуть з вільною швидкістю за класом.
     */
    async route(from, to, opts = {}) {
      if (from === to) return { nodes: [from], edges: [], length: 0, time: 0, expanded: 0, capped: false };
      for (const e of overridden) spd[e] = FREEFLOW_BY_CLS[graph.cls[e]];
      overridden = [];
      if (opts.ov) for (const [e, v] of opts.ov) if (e >= 0 && e < m) { spd[e] = Math.max(0.3, v); overridden.push(e); }
      if (++gen >= 0x7ffffff0) { stamp.fill(0); closed.fill(0); gen = 1; }
      f32[0] = opts.vmax ?? 25;
      p[P.GEN] = gen; p[P.FROM] = from; p[P.TO] = to; p[P.MAXEXP] = opts.maxExpand ?? 300000;
      p[P.HDIVBITS] = i32[0]; p[P.BUDGET] = opts.budget ?? 40000; p[P.RESUME] = 0;
      const t0 = performance.now();
      mark('wasm:run');
      let r = ex.astar(o.params);
      while (r === 2) {
        if (performance.now() - t0 > (opts.timeoutMs ?? 15000)) { p[P.CAPPED] = 1; break; }
        await new Promise((res) => setTimeout(res, 0));
        p[P.RESUME] = 1; r = ex.astar(o.params);
      }
      mark('wasm:done');
      const expanded = p[P.EXPANDED];
      this.lastExpanded = expanded; this.lastMs = performance.now() - t0;
      if (r !== 1) return null;
      const edges = [], nodes = [to];
      let cur = to, guard = 0;
      while (cur !== from) { const eid = prev[cur]; if (eid < 0 || ++guard > 5_000_000) return null; edges.push(eid); cur = graph.a[eid]; nodes.push(cur); }
      edges.reverse(); nodes.reverse();
      let length = 0, time = 0;
      for (const eid of edges) { length += graph.len[eid]; time += graph.len[eid] / spd[eid]; }
      return { nodes, edges, length, time, expanded, capped: false };
    },
    lastExpanded: 0, lastMs: 0,
    memoryMB: Math.round(pages * 65536 / 1048576),
  };
}
