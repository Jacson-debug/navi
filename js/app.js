// Наві — застосунок. Карта на canvas, датчики, GPS, АЗС зі статусами, маршрут, демо.
import { RoadGraph } from './graph.js';
import { NavEngine } from './engine.js';
import { route, instructions } from './astar.js';
import { dist, angleDiff } from './geo.js';
import { TrafficMemory, trafficColor, VMAX } from './traffic.js';
import { Buildings, normStreet, shortStreet } from './buildings.js';
import { Voice, turnPhrase } from './voice.js';

const $ = (id) => document.getElementById(id);
const canvas = $('map'), ctx = canvas.getContext('2d');
const DPR = Math.min(2, window.devicePixelRatio || 1);
const IS_IOS = /iPhone|iPad|iPod/.test(navigator.userAgent);

// ---------- стан ----------
const S = {
  graph: null, fuel: [], engine: null,
  view: { cx: 0, cy: 0, scale: 1.2 /* px per m */, rotate: false },
  follow: true, showParticles: false,
  routeInfo: null, target: null,
  hereMode: false, searchTab: 'all', hintShown: false,
  fuelStatus: JSON.parse(localStorage.getItem('navi.fuelStatus') || '{}'),
  sensorsOn: false, gpsWatch: null, lastDraw: 0, recording: false,
  traffic: null, showTraffic: true, track: null,
  bld: null, bldLoading: false, places: [], voice: null, lastSpoken: null,
  netTimer: null, netPending: false, lastNetAt: 0, lastFixAt: 0, lastGeoErr: '',
};

// ---------- завантаження даних ----------
async function loadData() {
  hint('Завантажую карту…');
  try {
    const [gb, f] = await Promise.all([fetch('data/graph.bin'), fetch('data/fuel.json')]);
    let graph;
    if (gb.ok) graph = RoadGraph.fromBinary(await gb.arrayBuffer());
    else {
      const gj = await fetch('data/graph.json');
      if (!gj.ok) throw new Error('немає data/graph.bin');
      graph = RoadGraph.fromJSON(await gj.json());
    }
    const fuel = f.ok ? await f.json() : [];
    $('dataInfo').textContent = `${graph.meta?.name || 'карта'}: ${graph.nNodes} вузлів, ${graph.n} доріг, ${fuel.length} АЗС`;
    hint('');
    S.bldLoading = true;
    fetch('data/places.json').then(async r => { if (r.ok) { S.places = await r.json(); draw(); } }).catch(() => { });
    fetch('data/buildings.bin').then(async r => { if (!r.ok) return; S.bld = Buildings.fromBinary(await r.arrayBuffer()); $('dataInfo').textContent += `, ${S.bld.count} будинків, ${S.bld.entranceCount} під'їздів`; S.bldLoading = false; draw(); }).catch(() => { S.bldLoading = false; });
    return { graph, fuel, real: true };
  } catch (e) {
    // Краще чесно сказати, що карти немає, ніж підсунути вигадане місто
    $('dataInfo').textContent = 'Карта не завантажилась: ' + e.message;
    hint('Карта не завантажилась. Відкрийте застосунок з інтернетом хоча б раз, щоб він її зберіг.');
    toast('Немає даних карти');
    throw e;
  }
}

// ---------- карта ----------
function resize() {
  canvas.width = Math.floor(innerWidth * DPR); canvas.height = Math.floor(innerHeight * DPR);
  canvas.style.width = innerWidth + 'px'; canvas.style.height = innerHeight + 'px';
  draw();
}
addEventListener('resize', resize);

function worldToScreen(x, y) {
  const v = S.view, w = canvas.width, h = canvas.height;
  let dx = (x - v.cx) * v.scale * DPR, dy = -(y - v.cy) * v.scale * DPR;
  if (v.rotate && S.engine?.state.xy) {
    const a = S.engine.state.heading; // повертаємо так, щоб курс дивився вгору
    const c = Math.cos(a), s = Math.sin(a);
    const rx = dx * c - dy * s, ry = dx * s + dy * c;
    dx = rx; dy = ry;
  }
  return [w / 2 + dx, h * (v.rotate ? 0.65 : 0.5) + dy];
}
function screenToWorld(sx, sy) {
  const v = S.view, w = canvas.width, h = canvas.height;
  let dx = sx * DPR - w / 2, dy = sy * DPR - h * (v.rotate ? 0.65 : 0.5);
  if (v.rotate && S.engine?.state.xy) {
    const a = -S.engine.state.heading;
    const c = Math.cos(a), s = Math.sin(a);
    const rx = dx * c - dy * s, ry = dx * s + dy * c;
    dx = rx; dy = ry;
  }
  return [v.cx + dx / (v.scale * DPR), v.cy - dy / (v.scale * DPR)];
}

function draw() {
  if (!S.graph) return;
  const g = S.graph, w = canvas.width, h = canvas.height, v = S.view;
  ctx.fillStyle = '#0b0f14'; ctx.fillRect(0, 0, w, h);
  // видима область (з запасом на поворот)
  const halfW = (w / 2) / (v.scale * DPR), halfH = (h / 2) / (v.scale * DPR);
  const R = Math.hypot(halfW, halfH) * 1.1;
  // чим дрібніший масштаб, тим важливіші лише дороги малюємо
  const minLevel = v.scale >= 0.9 ? 1 : v.scale >= 0.35 ? 2 : v.scale >= 0.12 ? 3 : v.scale >= 0.05 ? 4 : 5;
  const box = [v.cx - R, v.cy - R, v.cx + R, v.cy + R];
  const ids = g.cellCount(...box) > 80000 ? g.major : g.edgesInBox(...box, minLevel);
  const nX = g.nodeX, nY = g.nodeY, eA = g.a, eB = g.b;
  const drawEdge = (id) => {
    const a = worldToScreen(nX[eA[id]], nY[eA[id]]), b = worldToScreen(nX[eB[id]], nY[eB[id]]);
    ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]);
  };
  ctx.lineCap = 'round'; ctx.lineJoin = 'round';
  const lw = Math.max(1, Math.min(10, 6 * v.scale)) * DPR;
  // дороги: спершу дрібні тонкі, потім магістралі товщі
  const widthFor = (lvl) => lvl >= 5 ? lw * 1.6 : lvl >= 4 ? lw * 1.3 : lvl >= 3 ? lw * 1.1 : lvl >= 2 ? lw : lw * 0.6;
  if (ids === g.major) {
    ctx.strokeStyle = '#6f8299'; ctx.lineWidth = Math.max(1, lw); ctx.beginPath();
    for (let i = 0; i < ids.length; i++) drawEdge(ids[i]);
    ctx.stroke();
  } else
  for (let lvl = minLevel; lvl <= 5; lvl++) {
    ctx.strokeStyle = '#2a3542'; ctx.lineWidth = widthFor(lvl) + 2 * DPR; ctx.beginPath();
    for (let i = 0; i < ids.length; i++) if (g.level(ids[i]) === lvl) drawEdge(ids[i]);
    ctx.stroke();
    ctx.strokeStyle = lvl >= 4 ? '#6f8299' : '#55657a'; ctx.lineWidth = widthFor(lvl); ctx.beginPath();
    for (let i = 0; i < ids.length; i++) if (g.level(ids[i]) === lvl) drawEdge(ids[i]);
    ctx.stroke();
  }
  // будинки (контури при наближенні, номери — ще ближче), під'їзди
  if (S.bld && v.scale >= 0.5) {
    const bx0 = v.cx - halfW * 1.2, by0 = v.cy - halfH * 1.2, bx1 = v.cx + halfW * 1.2, by1 = v.cy + halfH * 1.2;
    const B = S.bld, bids = B.buildingsInBox(bx0, by0, bx1, by1);
    ctx.fillStyle = 'rgba(120,135,155,.16)'; ctx.strokeStyle = 'rgba(120,135,155,.35)'; ctx.lineWidth = 1 * DPR; ctx.beginPath();
    for (const i of bids) {
      const o = B.outline(i); if (o.length < 6) continue;
      const p0 = worldToScreen(o[0], o[1]); ctx.moveTo(p0[0], p0[1]);
      for (let k = 2; k < o.length; k += 2) { const p = worldToScreen(o[k], o[k + 1]); ctx.lineTo(p[0], p[1]); }
      ctx.closePath();
    }
    ctx.fill(); ctx.stroke();
    if (v.scale >= 1.4) {
      ctx.fillStyle = '#aab6c4'; ctx.font = `${10 * DPR}px -apple-system,system-ui,sans-serif`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      for (const i of bids) { if (!B.num[i]) continue; const p = worldToScreen(B.x[i], B.y[i]); ctx.fillText(B.num[i], p[0], p[1]); }
      ctx.textBaseline = 'alphabetic';
    }
    if (v.scale >= 2.2) {
      const eids = B.entrancesInBox(bx0, by0, bx1, by1);
      for (const i of eids) {
        const p = worldToScreen(B.ex[i], B.ey[i]);
        ctx.fillStyle = '#ffb020'; ctx.beginPath(); ctx.arc(p[0], p[1], 3 * DPR, 0, 7); ctx.fill();
        if (B.eref[i] && v.scale >= 3) { ctx.fillStyle = '#ffd27a'; ctx.font = `${9 * DPR}px sans-serif`; ctx.textAlign = 'left'; ctx.fillText(B.eref[i], p[0] + 5 * DPR, p[1] + 3 * DPR); }
      }
    }
  }
  // Дрібні вулиці ховаються при віддаленні, але дорога під вами має бути видна завжди.
  // Малюємо навколо позиції круглу «лінзу» з усіма дорогами і підсвічуємо поточне ребро.
  const me = S.engine?.state;
  if (me && me.xy && v.scale >= 0.02) {
    const rr = Math.min(1500, Math.max(250, 90 / v.scale));
    const c = worldToScreen(...me.xy);
    const rpx = rr * v.scale * DPR;
    ctx.save();
    ctx.beginPath(); ctx.arc(c[0], c[1], rpx, 0, 7); ctx.clip();
    const local = g.edgesInBox(me.xy[0] - rr, me.xy[1] - rr, me.xy[0] + rr, me.xy[1] + rr, v.scale < 0.12 ? 2 : 1);
    ctx.strokeStyle = '#55657a'; ctx.lineWidth = Math.max(1.5 * DPR, lw * 0.8); ctx.beginPath();
    for (let i = 0; i < local.length; i++) drawEdge(local[i]);
    ctx.stroke();
    ctx.restore();
    if (me.edge) {
      ctx.strokeStyle = '#8fa3ba'; ctx.lineWidth = Math.max(2.5 * DPR, lw * 1.1); ctx.beginPath();
      drawEdge(me.edge.id); ctx.stroke();
    }
  }
  // пробки з пам'яті поїздок: колір за типовою швидкістю в цю годину, зі зсувом у бік руху
  if (S.traffic && S.showTraffic && S.traffic.hasData) {
    const slot = TrafficMemory.slot();
    const off = Math.max(1.5, lw * 0.35);
    for (let i = 0; i < ids.length * 2; i++) {
      const id = i < ids.length ? ids[i] : g.rev[ids[i - ids.length]];
      if (id < 0) continue;
      const e = g.edge(id);
      const col = trafficColor(S.traffic.ratio(e, slot));
      if (!col) continue;
      const a = worldToScreen(nX[e.a], nY[e.a]), b = worldToScreen(nX[e.b], nY[e.b]);
      const L = Math.hypot(b[0] - a[0], b[1] - a[1]) || 1;
      const nx = (b[1] - a[1]) / L * off, ny = -(b[0] - a[0]) / L * off; // перпендикуляр праворуч від напрямку руху
      ctx.strokeStyle = col; ctx.lineWidth = Math.max(1.5 * DPR, lw * 0.5); ctx.beginPath();
      ctx.moveTo(a[0] + nx, a[1] + ny); ctx.lineTo(b[0] + nx, b[1] + ny); ctx.stroke();
    }
  }
  // одностороння стрілочка (лише при великому масштабі)
  if (v.scale > 0.6) {
    ctx.fillStyle = '#8fa3ba';
    for (const id of ids) {
      if (g.isTwoWay(id)) continue;
      const e = g.edge(id);
      const m = g.pointOnEdge(e, e.len / 2); const p = worldToScreen(...m);
      const p2 = worldToScreen(...g.pointOnEdge(e, Math.min(e.len, e.len / 2 + 5)));
      const ang = Math.atan2(p2[1] - p[1], p2[0] - p[0]);
      ctx.save(); ctx.translate(p[0], p[1]); ctx.rotate(ang); ctx.beginPath(); ctx.moveTo(4 * DPR, 0); ctx.lineTo(-3 * DPR, 3 * DPR); ctx.lineTo(-3 * DPR, -3 * DPR); ctx.closePath(); ctx.fill(); ctx.restore();
    }
  }
  // назви вулиць
  if (v.scale > 1.6) {
    ctx.fillStyle = '#9fb0c3'; ctx.font = `${11 * DPR}px -apple-system,system-ui,sans-serif`; ctx.textAlign = 'center';
    const seen = new Set();
    for (const id of ids) {
      if (g.nameIdx[id] < 0 || g.len[id] < 60) continue;
      const e = g.edge(id); if (seen.has(e.name + '|' + e.a)) continue; seen.add(e.name + '|' + e.a); seen.add(e.name + '|' + e.b);
      const m = worldToScreen(...g.pointOnEdge(e, e.len / 2)); const p2 = worldToScreen(...g.pointOnEdge(e, e.len / 2 + 5));
      let ang = Math.atan2(p2[1] - m[1], p2[0] - m[0]); if (ang > Math.PI / 2 || ang < -Math.PI / 2) ang += Math.PI;
      ctx.save(); ctx.translate(m[0], m[1]); ctx.rotate(ang); ctx.fillText(e.name, 0, -lw / 2 - 3 * DPR); ctx.restore();
    }
  }
  // маршрут
  if (S.routeInfo) {
    ctx.strokeStyle = '#4da3ff'; ctx.lineWidth = lw + 1 * DPR; ctx.globalAlpha = .9; ctx.beginPath();
    for (const eid of S.routeInfo.edges) drawEdge(eid);
    ctx.stroke(); ctx.globalAlpha = 1;
  }
  // АЗС: здалеку вони заливають карту, тому показуємо лише при достатньому наближенні
  // (а також завжди — позначені вами та поточну ціль).
  const fuelZoom = v.scale >= 0.45;
  ctx.textAlign = 'center';
  for (const f of S.fuel) {
    const st = S.fuelStatus[f.id]?.status;
    const isTarget = S.target && S.target.id === f.id;
    if (!fuelZoom && !st && !isTarget) continue;
    const p = worldToScreen(f.x, f.y);
    if (p[0] < -40 || p[1] < -40 || p[0] > w + 40 || p[1] > h + 40) continue;
    const col = st === 'ok' ? '#3ddc84' : st === 'bad' ? '#ff5a5f' : '#c08a3e';
    const r = (st || isTarget ? 5 : 4) * DPR;
    ctx.fillStyle = col; ctx.beginPath(); ctx.arc(p[0], p[1], r, 0, 7); ctx.fill();
    ctx.strokeStyle = '#0b0f14'; ctx.lineWidth = 1.5 * DPR; ctx.stroke();
    if (v.scale > 1.6) { ctx.fillStyle = col; ctx.font = `${10 * DPR}px sans-serif`; ctx.textBaseline = 'alphabetic'; ctx.fillText(f.brand || f.name || '', p[0], p[1] - r - 4 * DPR); }
    if (isTarget) { ctx.strokeStyle = '#4da3ff'; ctx.lineWidth = 2 * DPR; ctx.beginPath(); ctx.arc(p[0], p[1], r + 6 * DPR, 0, 7); ctx.stroke(); }
  }
  // Приблизне коло від системи, коли точної позиції ще немає
  const hint0 = S.engine?.state.hint;
  if (hint0 && !S.engine.state.xy) {
    const p = worldToScreen(...hint0.xy);
    const rpx = hint0.acc * v.scale * DPR;
    ctx.fillStyle = 'rgba(255,176,32,.10)'; ctx.strokeStyle = 'rgba(255,176,32,.55)'; ctx.lineWidth = 2 * DPR;
    ctx.beginPath(); ctx.arc(p[0], p[1], rpx, 0, 7); ctx.fill(); ctx.stroke();
    ctx.fillStyle = '#ffb020'; ctx.beginPath(); ctx.arc(p[0], p[1], 5 * DPR, 0, 7); ctx.fill();
  }
  // ціль (адреса)
  if (S.target && S.target.kind === 'addr') {
    const p = worldToScreen(S.target.x, S.target.y);
    ctx.strokeStyle = '#4da3ff'; ctx.fillStyle = 'rgba(77,163,255,.25)'; ctx.lineWidth = 2 * DPR; ctx.beginPath(); ctx.arc(p[0], p[1], 9 * DPR, 0, 7); ctx.fill(); ctx.stroke();
  }
  // частинки
  const eng = S.engine;
  if (eng && S.showParticles) {
    ctx.fillStyle = 'rgba(255,176,32,.55)';
    for (const q of eng.pf.p) { const p = worldToScreen(...g.pointOnEdge(q.e, q.s)); ctx.fillRect(p[0] - DPR, p[1] - DPR, 2 * DPR, 2 * DPR); }
  }
  // «правда» в демо
  // позиція
  if (eng && eng.state.xy) {
    const st = eng.state; const p = worldToScreen(...st.xy);
    const spread = Math.min(300, st.spread) * v.scale * DPR;
    if (spread > 10 * DPR) { ctx.fillStyle = 'rgba(77,163,255,.12)'; ctx.beginPath(); ctx.arc(p[0], p[1], spread, 0, 7); ctx.fill(); }
    const a = worldToScreen(st.xy[0] + Math.sin(st.heading), st.xy[1] + Math.cos(st.heading));
    const ang = Math.atan2(a[1] - p[1], a[0] - p[0]);
    ctx.save(); ctx.translate(p[0], p[1]); ctx.rotate(ang);
    ctx.fillStyle = st.spoof ? '#ffb020' : '#4da3ff'; ctx.strokeStyle = '#fff'; ctx.lineWidth = 2 * DPR;
    ctx.beginPath(); ctx.moveTo(16 * DPR, 0); ctx.lineTo(-10 * DPR, 9 * DPR); ctx.lineTo(-6 * DPR, 0); ctx.lineTo(-10 * DPR, -9 * DPR); ctx.closePath(); ctx.fill(); ctx.stroke();
    ctx.restore();
  }
  // режим «Я тут»
  if (S.hereMode) {
    ctx.strokeStyle = '#ffb020'; ctx.lineWidth = 2 * DPR; ctx.beginPath(); ctx.moveTo(w / 2 - 14 * DPR, h / 2); ctx.lineTo(w / 2 + 14 * DPR, h / 2); ctx.moveTo(w / 2, h / 2 - 14 * DPR); ctx.lineTo(w / 2, h / 2 + 14 * DPR); ctx.stroke();
  }
  // Назви міст, сіл і районів. Що дрібніший масштаб, то лише найбільші;
  // щоб підписи не злипались, зайняті місця позначаємо в грубій сітці.
  if (S.places.length && v.scale < 0.6) {
    const RANK = { city: 1, borough: 2, town: 2, suburb: 3, village: 4, quarter: 5, neighbourhood: 5, hamlet: 5, isolated_dwelling: 6 };
    const maxRank = v.scale < 0.004 ? 2 : v.scale < 0.012 ? 3 : v.scale < 0.05 ? 4 : 6;
    const taken = new Set();
    const cell = 64 * DPR;
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.lineJoin = 'round'; ctx.miterLimit = 2;
    for (const pl of S.places) {
      const rank = RANK[pl.k] || 6;
      if (rank > maxRank) continue;
      const p = worldToScreen(pl.x, pl.y);
      if (p[0] < 0 || p[1] < 60 * DPR || p[0] > w || p[1] > h - 150 * DPR) continue;
      const key = Math.round(p[0] / cell) + ',' + Math.round(p[1] / cell);
      if (taken.has(key)) continue;
      const size = (rank <= 1 ? 16 : rank <= 2 ? 14 : rank <= 3 ? 12.5 : 11) * DPR;
      ctx.font = `${rank <= 2 ? '600 ' : ''}${size}px -apple-system,system-ui,sans-serif`;
      // обрізаний підпис гірший за його відсутність — пропускаємо, якщо не влазить
      const half = ctx.measureText(pl.n).width / 2;
      if (p[0] - half < 6 * DPR || p[0] + half > w - 6 * DPR) continue;
      taken.add(key);
      ctx.strokeStyle = 'rgba(11,15,20,.85)'; ctx.lineWidth = 3 * DPR;
      ctx.strokeText(pl.n, p[0], p[1]);
      ctx.fillStyle = rank <= 2 ? '#e8eef5' : '#b9c6d4';
      ctx.fillText(pl.n, p[0], p[1]);
    }
    ctx.textBaseline = 'alphabetic';
  }
  // масштабна лінійка
  const meters = [10, 20, 50, 100, 200, 500, 1000, 2000, 5000].find(m => m * v.scale * DPR > 60 * DPR) || 5000;
  const px = meters * v.scale * DPR;
  ctx.strokeStyle = '#c8d3df'; ctx.lineWidth = 2 * DPR; ctx.beginPath(); ctx.moveTo(14 * DPR, h - 150 * DPR); ctx.lineTo(14 * DPR + px, h - 150 * DPR); ctx.stroke();
  ctx.fillStyle = '#c8d3df'; ctx.font = `${11 * DPR}px sans-serif`; ctx.textAlign = 'left'; ctx.fillText(meters >= 1000 ? (meters / 1000) + ' км' : meters + ' м', 14 * DPR, h - 156 * DPR);
}

// ---------- жести ----------
// Нижня межа масштабу — щоб уся область влізла в екран (понад 250 км)
const clampScale = (v) => Math.max(0.0012, Math.min(14, v));
let gesture = null;
canvas.addEventListener('pointerdown', (e) => { canvas.setPointerCapture(e.pointerId); gesture = gesture || { pts: new Map(), moved: false }; gesture.pts.set(e.pointerId, [e.clientX, e.clientY]); gesture.start = [...gesture.pts.values()]; gesture.startView = { ...S.view }; });
canvas.addEventListener('pointermove', (e) => {
  if (!gesture || !gesture.pts.has(e.pointerId)) return;
  gesture.pts.set(e.pointerId, [e.clientX, e.clientY]);
  const pts = [...gesture.pts.values()];
  if (pts.length === 1) {
    const dx = pts[0][0] - gesture.start[0][0], dy = pts[0][1] - gesture.start[0][1];
    if (Math.hypot(dx, dy) > 4) gesture.moved = true;
    if (gesture.moved) { S.follow = false; const a = S.view.rotate && S.engine?.state.xy ? -S.engine.state.heading : 0; const c = Math.cos(a), s = Math.sin(a); const rx = dx * c - dy * s, ry = dx * s + dy * c; S.view.cx = gesture.startView.cx - rx / S.view.scale; S.view.cy = gesture.startView.cy + ry / S.view.scale; }
  } else if (pts.length >= 2 && gesture.start.length >= 2) {
    const d0 = Math.hypot(gesture.start[0][0] - gesture.start[1][0], gesture.start[0][1] - gesture.start[1][1]);
    const d1 = Math.hypot(pts[0][0] - pts[1][0], pts[0][1] - pts[1][1]);
    S.view.scale = clampScale(gesture.startView.scale * d1 / Math.max(1, d0)); gesture.moved = true;
  }
  draw();
});
const endGesture = (e) => {
  if (!gesture) return;
  if (!gesture.moved && gesture.pts.size === 1) onTap(e.clientX, e.clientY);
  gesture.pts.delete(e.pointerId); if (!gesture.pts.size) gesture = null; else { gesture.start = [...gesture.pts.values()]; gesture.startView = { ...S.view }; gesture.moved = true; }
};
canvas.addEventListener('pointerup', endGesture); canvas.addEventListener('pointercancel', endGesture);
canvas.addEventListener('wheel', (e) => { e.preventDefault(); S.view.scale = clampScale(S.view.scale * (e.deltaY < 0 ? 1.15 : 0.87)); draw(); }, { passive: false });

function onTap(sx, sy) {
  const p = screenToWorld(sx, sy);
  if (S.hereMode) {
    S.engine.setPositionXY(p, 80); S.hereMode = false; S.follow = true; $('btnHere').classList.remove('active'); hint('');
    const bi = S.bld ? S.bld.nearest(p, 60) : -1;
    toast(bi >= 0 ? `Позицію встановлено біля ${S.bld.label(bi)}` : 'Позицію встановлено'); draw(); return;
  }
  // тап по АЗС — одразу пропонуємо маршрут
  let best = null, bd = 25 / S.view.scale;
  for (const f of S.fuel) { const d = dist(p, [f.x, f.y]); if (d < bd) { bd = d; best = f; } }
  if (best) {
    const item = { kind: 'fuel', id: best.id, x: best.x, y: best.y, name: best.name || best.brand || 'АЗС' };
    S.target = item; recentAdd(item); routeTo(item); return;
  }
  // тап по будинку — показуємо адресу
  if (S.bld && S.view.scale >= 0.8) {
    const i = S.bld.nearest(p, 40 / S.view.scale);
    if (i >= 0) { toast(S.bld.label(i)); return; }
  }
}

// ---------- АЗС ----------
function setFuelStatus(id, status) {
  if (status) S.fuelStatus[id] = { status, t: Date.now() }; else delete S.fuelStatus[id];
  localStorage.setItem('navi.fuelStatus', JSON.stringify(S.fuelStatus));
  renderSearch(); draw();
}
function ago(t) { const m = Math.round((Date.now() - t) / 60000); return m < 1 ? 'щойно' : m < 60 ? `${m} хв тому` : m < 1440 ? `${Math.round(m / 60)} год тому` : `${Math.round(m / 1440)} дн тому`; }
// ---------- маршрут ----------
function routeTo(f) {
  const st = S.engine?.state;
  if (!st?.xy) { toast('Спочатку визначте позицію: «Я тут» або ⋯ → «Визначити по мережі»'); return; }
  const g = S.graph;
  // Беремо кілька варіантів старту й фінішу: вузол може виявитись у глухому куті
  // або в маленькій відрізаній частині графа, тоді пробуємо наступний.
  const froms = [st.edge ? st.edge.b : null, ...g.nearestNodes(st.xy, 600, 1, 6)].filter(x => x !== null);
  const tos = [...g.nearestNodes([f.x, f.y], 600, 1, 8), g.nearestNode([f.x, f.y])];
  let r = null;
  for (const from of froms) { for (const to of tos) { r = route(g, from, to, S.traffic.speedFn(), VMAX); if (r) break; } if (r) break; }
  if (!r) {
    toast(`Не вдалося прокласти шлях до «${f.name || 'цілі'}» — можливо, туди немає проїзду по карті`);
    S.target = f; S.follow = false; S.view.cx = f.x; S.view.cy = f.y; draw();
    return;
  }
  S.routeInfo = r; S.target = f; S.routeSteps = instructions(g, r); S.lastSpoken = null;
  S.follow = true;
  const km = (r.length / 1000).toFixed(1), min = Math.max(1, Math.round(r.time / 60));
  toast(`${f.name || 'Маршрут'} · ${km} км · ~${min} хв`);
  S.voice?.say(`Маршрут побудовано, ${km.replace('.', ' і ')} кілометра, приблизно ${min} хвилин`, 'route', 0);
  draw(); updateTurn();
}

function updateTurn() {
  const el = $('turn');
  const g = S.graph, st = S.engine?.state;
  if (!S.routeInfo || !st?.edge) { el.classList.remove('show'); return; }

  const edges = S.routeInfo.edges;
  let idx = edges.indexOf(st.edge.id);
  let d;           // відстань до наступного маневру
  let from;        // з якого ребра маршруту рахувати повороти
  if (idx >= 0) {
    d = st.edge.len - (st.s ?? 0);
    from = idx;
  } else if (edges.length && g.a[edges[0]] === st.edge.b) {
    // Ще не на маршруті, але прямуємо в його початок — підказка вже потрібна
    d = st.edge.len - (st.s ?? 0);
    from = -1;
  } else {
    // З'їхали з маршруту — перебудовуємо від поточного місця
    if (S.target && S.rebuildAt !== st.edge.id) {
      S.rebuildAt = st.edge.id;
      const r = route(g, st.edge.b, g.nearestNode([S.target.x, S.target.y]), S.traffic.speedFn(), VMAX);
      if (r) { S.routeInfo = r; S.routeSteps = instructions(g, r); S.lastSpoken = null; toast('Перебудовую маршрут'); }
    }
    el.classList.remove('show');
    return;
  }

  let text = 'Прибуття', spoken = 'Прибуття', arrow = '🏁', turnKey = 'end';
  for (let i = from + 1; i < edges.length; i++) {
    const e = g.edge(edges[i]);
    const prev = i === 0 ? st.edge : g.edge(edges[i - 1]);
    const deg = angleDiff(e.hdg, prev.hdg) * 180 / Math.PI;
    if (Math.abs(deg) > 30) {
      const dir = deg > 0 ? 'праворуч' : 'ліворуч';
      text = dir + (e.name ? ', ' + shortStreet(e.name) : '');
      spoken = dir + (e.name ? ', ' + e.name : '');
      arrow = deg > 0 ? '↱' : '↰'; turnKey = i + '|' + text;
      break;
    }
    d += e.len;
  }

  $('turnDist').textContent = `${arrow} ${d < 1000 ? Math.round(d / 10) * 10 + ' м' : (d / 1000).toFixed(1) + ' км'}`;
  $('turnText').textContent = text;
  el.classList.add('show');
  hint('');

  // голос: на 500 м, 200 м і перед самим поворотом — по одному разу на маневр
  const stage = d > 500 ? 0 : d > 200 ? 1 : d > 60 ? 2 : 3;
  if (!S.lastSpoken || S.lastSpoken.key !== turnKey) S.lastSpoken = { key: turnKey, stage: -1 };
  if (stage > S.lastSpoken.stage && stage >= 1) {
    S.lastSpoken.stage = stage;
    S.voice?.say(turnPhrase(stage === 3 ? 0 : stage === 2 ? 200 : 500, spoken), turnKey + stage, 0);
  }
  if (d < 30 && turnKey === 'end') {
    toast('Ви на місці'); S.voice?.say('Ви прибули', 'arrive', 0);
    S.routeInfo = null; S.target = null; el.classList.remove('show');
  }
}

// ---------- датчики та GPS ----------
const motionNeedsPermission = () => typeof DeviceMotionEvent !== 'undefined' && typeof DeviceMotionEvent.requestPermission === 'function';

// Дозвіл на датчики руху iOS дає лише у відповідь на дотик, тому це окремо від решти.
async function enableMotion(opts = {}) {
  if (S.motionOn) return true;
  let motion = true;
  try {
    if (motionNeedsPermission()) {
      const r = await DeviceMotionEvent.requestPermission();
      if (r !== 'granted') { motion = false; if (!opts.quiet) toast('Без дозволу на рух вестиму лише по мережі — точність буде гіршою'); }
    }
  } catch (e) { motion = false; if (!opts.quiet) toast('Датчики руху недоступні: ' + e.message); }
  if (!('DeviceMotionEvent' in window)) { motion = false; if (!opts.quiet) toast('Цей браузер не дає датчиків руху — лише груба позиція по мережі'); }

  if (motion) {
    S.t0 = null;
    addEventListener('devicemotion', onMotion);
    try { localStorage.setItem('navi.motionGranted', '1'); } catch (e) {}
    if (!opts.quiet) {
      hint('Закріпіть телефон у тримачі. Перші секунди стійте на місці, потім рушайте — застосунок сам визначить, де «вперед».');
      setTimeout(() => hint(''), 9000);
    }
    if (autoRecOn()) startAutoRec();
  }
  S.motionOn = motion;
  return motion;
}

// Якщо дозвіл просити рано (немає дотику) — вмикаємо датчики на першому ж доторку
// до екрана, щоб не треба було шукати кнопку перед виїздом.
let motionArmed = false;
function armMotionOnFirstTouch() {
  if (motionArmed || S.motionOn) return;
  motionArmed = true;
  const go = () => {
    removeEventListener('pointerdown', go, true);
    removeEventListener('touchstart', go, true);
    enableMotion({ quiet: true });
  };
  addEventListener('pointerdown', go, true);
  addEventListener('touchstart', go, true);
}

function markRunning(on) {
  $('btnSensors').textContent = on ? '⏹ Стоп' : '▶ Старт';
  $('btnSensors').classList.toggle('primary', !on);
}

async function startSensors() {
  if (S.sensorsOn) { stopSensors(); return; }
  // Позицію вмикаємо завжди й першою: навіть якщо датчики руху заборонені або їх немає,
  // мережа зв'язку дасть хоча б грубе місцезнаходження.
  S.sensorsOn = true;
  markRunning(true);
  startGps(); requestWakeLock();
  await enableMotion();
}

// Вмикається саме́ при відкритті: позиція — відразу, датчики руху — якщо дозвіл уже
// давали раніше, інакше на першому доторку до екрана.
async function autoStart() {
  S.sensorsOn = true;
  markRunning(true);
  startGps(); requestWakeLock();
  const granted = (() => { try { return localStorage.getItem('navi.motionGranted') === '1'; } catch (e) { return false; } })();
  let ok = false;
  if (granted || !motionNeedsPermission()) ok = await enableMotion({ quiet: true });
  if (!ok) armMotionOnFirstTouch();
}

function stopSensors() { removeEventListener('devicemotion', onMotion); S.motionOn = false; motionArmed = false; Awake.off(); clearInterval(S.netTimer); S.netTimer = null; S.sensorsOn = false; markRunning(false); if (S.gpsWatch) navigator.geolocation.clearWatch(S.gpsWatch); S.gpsWatch = null; }
function onMotion(e) {
  const ag = e.accelerationIncludingGravity, rr = e.rotationRate;
  if (!ag || ag.x == null || !rr || rr.alpha == null) return;
  const t = (e.timeStamp || performance.now()) / 1000;
  // W3C: rotationRate.alpha — навколо z, beta — навколо x, gamma — навколо y (град/с)
  const D2R = Math.PI / 180;
  S.engine.pushMotion({ t, acc: [ag.x, ag.y, ag.z], gyro: [rr.beta * D2R, rr.gamma * D2R, rr.alpha * D2R] });
}
function startGps() {
  if (!('geolocation' in navigator)) { $('txtGps').textContent = 'Геолокація недоступна'; return; }
  // 1) Спершу просимо грубу позицію: її iOS бере з веж зв'язку та Wi-Fi, без супутників.
  //    Вона приходить за секунду-дві й працює там, де РЕБ глушить GPS.
  findMe(true);
  // 2) Паралельно — звичайне стеження з максимальною точністю (супутники, коли є).
  S.gpsWatch = navigator.geolocation.watchPosition((pos) => {
    const c = pos.coords;
    S.lastFixAt = Date.now();
    S.engine.pushGps({ t: pos.timestamp / 1000, lat: c.latitude, lon: c.longitude, accuracy: c.accuracy, speed: c.speed, heading: c.heading });
  }, (err) => {
    S.lastGeoErr = geoErrText(err);
    if (err.code === 1) { $('txtGps').textContent = 'Геолокація заборонена'; $('dotGps').className = 'dot bad'; }
  }, { enableHighAccuracy: true, maximumAge: 1000, timeout: 15000 });
  // 3) Кожні 25 с перепитуємо грубу позицію — це незалежне джерело, яким можна
  //    і підстрахувати дрейф, і впіймати підміну координат.
  clearInterval(S.netTimer);
  S.netTimer = setInterval(() => locateByNetwork(false), 25000);
}

// Чому не вийшло — зрозумілим текстом, а не кодом помилки
function geoErrText(err) {
  if (err.code === 1) return 'Геолокацію заборонено. Параметри → Приватність і безпека → Служби геолокації → увімкнути, знайти Safari (або «Наві») → «При використанні» і «Точне місце».';
  if (err.code === 2) return 'Система не змогла визначити місце: поруч немає ні супутників, ні знайомих їй мереж. Спробуйте ще раз або вкажіть себе кнопкою «Я тут».';
  return 'Місце не визначилось вчасно. Натисніть «Де я» ще раз.';
}

// Явний пошук себе: питаємо і грубу позицію (вежі, Wi-Fi), і точну (супутники).
// Що прийде першим, те й показуємо; точна потім уточнить тихо.
function findMe(announce = true) {
  if (!('geolocation' in navigator)) { toast('Геолокація недоступна в цьому браузері'); return; }
  if (!S.engine) { toast('Карта ще завантажується'); return; }
  if (announce) toast('Шукаю вас…');
  let answered = false;
  const ok = (pos) => {
    const c = pos.coords;
    S.lastNetAt = Date.now(); S.lastGeoErr = '';
    S.engine.pushGps({ t: pos.timestamp / 1000, lat: c.latitude, lon: c.longitude, accuracy: c.accuracy, speed: c.speed, heading: c.heading });
    S.follow = true;
    if (announce && !answered) {
      answered = true;
      const a = c.accuracy;
      const r = a < 1000 ? Math.round(a) + ' м' : (a / 1000).toFixed(1) + ' км';
      if (a <= 40) toast(`Знайшов по супутниках, ±${r}`);
      else if (a <= 3000) toast(`Знайшов по мережі зв'язку, ±${r} — уточню під час руху`);
      else toast(`Система дала лише приблизне коло ±${r} — цього замало, щоб визначити вулицю. Увімкніть «Точне місце» в налаштуваннях iOS або вкажіть себе кнопкою «Я тут».`);
    }
    draw();
  };
  const fail = (err) => { S.lastGeoErr = geoErrText(err); if (announce && !answered) { answered = true; toast(S.lastGeoErr); } };
  navigator.geolocation.getCurrentPosition(ok, fail, { enableHighAccuracy: false, maximumAge: 30000, timeout: 15000 });
  navigator.geolocation.getCurrentPosition(ok, () => { }, { enableHighAccuracy: true, maximumAge: 0, timeout: 25000 });
}

// Груба позиція «по мережі»: enableHighAccuracy:false просить систему не вмикати
// супутниковий приймач, тож відповідь приходить від веж зв'язку / Wi-Fi.
function locateByNetwork(initial) {
  if (!('geolocation' in navigator)) return;
  S.netPending = true;
  navigator.geolocation.getCurrentPosition((pos) => {
    S.netPending = false;
    const c = pos.coords;
    S.lastNetAt = Date.now();
    S.engine.pushGps({ t: pos.timestamp / 1000, lat: c.latitude, lon: c.longitude, accuracy: c.accuracy, speed: null, heading: null });
    if (initial) {
      const a = Math.round(c.accuracy);
      toast(a <= 40 ? 'Позицію визначено по супутниках' : `Позицію визначено по мережі зв'язку, ±${a < 1000 ? a + ' м' : (a / 1000).toFixed(1) + ' км'} — уточню під час руху`);
      S.follow = true;
    }
  }, (err) => {
    S.netPending = false;
    if (initial && err.code !== 1) toast('Позицію по мережі визначити не вдалося — вкажіть її кнопкою «Я тут»');
  }, { enableHighAccuracy: false, maximumAge: initial ? 60000 : 10000, timeout: 20000 });
}

let wakeLock = null;
// ---------- екран не має гаснути ----------
// Раніше wake lock просився один раз, і якщо не вдалося (а на старті сторінка ще
// може бути не «видимою»), повторних спроб не було — екран гас посеред поїздки.
// Тому: повторні спроби, перезапит після згортання, і резервний шлях для iOS —
// беззвучне відео в циклі, яке Safari вважає приводом не гасити екран.
const KEEP_VIDEO = 'data:video/mp4;base64,AAAAIGZ0eXBpc29tAAACAGlzb21pc28yYXZjMW1wNDEAAAM2bW9vdgAAAGxtdmhkAAAAAAAAAAAAAAAAAAAD6AAAA+gAAQAAAQAAAAAAAAAAAAAAAAEAAAAAAAAAAAAAAAAAAAABAAAAAAAAAAAAAAAAAABAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAgAAAmB0cmFrAAAAXHRraGQAAAADAAAAAAAAAAAAAAABAAAAAAAAA+gAAAAAAAAAAAAAAAAAAAAAAAEAAAAAAAAAAAAAAAAAAAABAAAAAAAAAAAAAAAAAABAAAAAAEAAAABAAAAAAAAkZWR0cwAAABxlbHN0AAAAAAAAAAEAAAPoAAAAAAABAAAAAAHYbWRpYQAAACBtZGhkAAAAAAAAAAAAAAAAAAAoAAAAKABVxAAAAAAALWhkbHIAAAAAAAAAAHZpZGUAAAAAAAAAAAAAAABWaWRlb0hhbmRsZXIAAAABg21pbmYAAAAUdm1oZAAAAAEAAAAAAAAAAAAAACRkaW5mAAAAHGRyZWYAAAAAAAAAAQAAAAx1cmwgAAAAAQAAAUNzdGJsAAAAt3N0c2QAAAAAAAAAAQAAAKdhdmMxAAAAAAAAAAEAAAAAAAAAAAAAAAAAAAAAAEAAQABIAAAASAAAAAAAAAABFUxhdmM2MC4zMS4xMDIgbGlieDI2NAAAAAAAAAAAAAAAGP//AAAALWF2Y0MBQsAe/+EAFmdCwB7aEJsBEAAAAwAQAAADAKDxYuoBAARozg/IAAAAEHBhc3AAAAABAAAAAQAAABRidHJ0AAAAAAAAFcgAABXIAAAAGHN0dHMAAAAAAAAAAQAAAAUAAAgAAAAAFHN0c3MAAAAAAAAAAQAAAAEAAAAcc3RzYwAAAAAAAAABAAAAAQAAAAUAAAABAAAAKHN0c3oAAAAAAAAAAAAAAAUAAAKRAAAACgAAAAoAAAAKAAAACgAAABRzdGNvAAAAAAAAAAEAAANmAAAAYnVkdGEAAABabWV0YQAAAAAAAAAhaGRscgAAAAAAAAAAbWRpcmFwcGwAAAAAAAAAAAAAAAAtaWxzdAAAACWpdG9vAAAAHWRhdGEAAAABAAAAAExhdmY2MC4xNi4xMDAAAAAIZnJlZQAAAsFtZGF0AAACbwYF//9r3EXpvebZSLeWLNgg2SPu73gyNjQgLSBjb3JlIDE2NCByMzEwOCAzMWUxOWY5IC0gSC4yNjQvTVBFRy00IEFWQyBjb2RlYyAtIENvcHlsZWZ0IDIwMDMtMjAyMyAtIGh0dHA6Ly93d3cudmlkZW9sYW4ub3JnL3gyNjQuaHRtbCAtIG9wdGlvbnM6IGNhYmFjPTAgcmVmPTEgZGVibG9jaz0xOjA6MCBhbmFseXNlPTB4MToweDExMSBtZT1oZXggc3VibWU9MiBwc3k9MSBwc3lfcmQ9MS4wMDowLjAwIG1peGVkX3JlZj0wIG1lX3JhbmdlPTE2IGNocm9tYV9tZT0xIHRyZWxsaXM9MCA4eDhkY3Q9MCBjcW09MCBkZWFkem9uZT0yMSwxMSBmYXN0X3Bza2lwPTEgY2hyb21hX3FwX29mZnNldD0wIHRocmVhZHM9MiBsb29rYWhlYWRfdGhyZWFkcz0xIHNsaWNlZF90aHJlYWRzPTAgbnI9MCBkZWNpbWF0ZT0xIGludGVybGFjZWQ9MCBibHVyYXlfY29tcGF0PTAgY29uc3RyYWluZWRfaW50cmE9MCBiZnJhbWVzPTAgd2VpZ2h0cD0wIGtleWludD0yNTAga2V5aW50X21pbj01IHNjZW5lY3V0PTQwIGludHJhX3JlZnJlc2g9MCByY19sb29rYWhlYWQ9MTAgcmM9Y3JmIG1idHJlZT0xIGNyZj0yMy4wIHFjb21wPTAuNjAgcXBtaW49MCBxcG1heD02OSBxcHN0ZXA9NCBpcF9yYXRpbz0xLjQwIGFxPTE6MS4wMACAAAAAGmWIhAR///8PRQABKr8nJyddddddddddddeAAAAABkGaIBHwjAAAAAZBmkAR8IwAAAAGQZpgEPCMAAAABkGagD/CMA==';
const Awake = {
  lock: null, video: null, mode: 'off', timer: null,
  async on() {
    if (!this.timer) this.timer = setInterval(() => { if (S.sensorsOn && document.visibilityState === 'visible') this._ensure(); }, 15000);
    if (!this._vis) {
      this._vis = true;
      document.addEventListener('visibilitychange', () => {
        if (document.visibilityState === 'visible' && S.sensorsOn) this._ensure();
      });
    }
    await this._ensure();
    return this.mode;
  },
  async _ensure() {
    if (this.lock && !this.lock.released) return;
    if ('wakeLock' in navigator) {
      try {
        this.lock = await navigator.wakeLock.request('screen');
        this.lock.addEventListener('release', () => { this.lock = null; });
        this.mode = 'lock';
        this._videoOff();
        return;
      } catch (e) { this.lock = null; }
    }
    this._videoOn();
  },
  _videoOn() {
    if (!this.video) {
      const v = document.createElement('video');
      v.setAttribute('playsinline', ''); v.setAttribute('muted', ''); v.setAttribute('loop', '');
      v.muted = true; v.loop = true; v.playsInline = true; v.src = KEEP_VIDEO;
      v.style.cssText = 'position:fixed;left:0;bottom:0;width:1px;height:1px;opacity:0.01;pointer-events:none';
      document.body.appendChild(v);
      this.video = v;
    }
    this.video.play().then(() => { this.mode = 'video'; }).catch(() => {
      this.mode = 'off';
      // play() без дотику Safari може заблокувати — пробуємо на першому ж доторку
      const go = () => { removeEventListener('pointerdown', go, true); this.video.play().then(() => { this.mode = 'video'; }).catch(() => {}); };
      addEventListener('pointerdown', go, true);
    });
  },
  _videoOff() { if (this.video) { this.video.pause(); } },
  off() { try { this.lock?.release(); } catch (e) {} this.lock = null; this._videoOff(); this.mode = 'off'; clearInterval(this.timer); this.timer = null; }
};
async function requestWakeLock() { return Awake.on(); }

// ---------- демо ----------
// ---------- інтерфейс ----------
function toast(msg) { const t = $('toast'); t.textContent = msg; t.classList.add('show'); clearTimeout(t._h); t._h = setTimeout(() => t.classList.remove('show'), 3500); }
function hint(msg) { const h = $('hint'); h.textContent = msg; h.classList.toggle('show', !!msg); }
function closeSheets() { document.querySelectorAll('.sheet').forEach(s => s.classList.remove('show')); }
document.querySelectorAll('[data-close]').forEach(b => b.addEventListener('click', () => $(b.dataset.close).classList.remove('show')));

$('btnSensors').addEventListener('click', () => { S.voice?.warmup(); startSensors(); });
$('btnHere').addEventListener('click', () => { S.hereMode = !S.hereMode; $('btnHere').classList.toggle('active', S.hereMode); hint(S.hereMode ? 'Перетягніть карту так, щоб перехрестя, на якому ви стоїте, було під прицілом, і торкніться його.' : ''); draw(); });
// ---------- єдиний пошук: адреси, вулиці, АЗС ----------
const fmtDist = (d) => d < 1000 ? Math.round(d) + ' м' : (d / 1000).toFixed(1) + ' км';
const recentGet = () => { try { return JSON.parse(localStorage.getItem('navi.recent') || '[]'); } catch { return []; } };
function recentAdd(item) {
  const list = recentGet().filter(r => !(r.kind === item.kind && r.name === item.name));
  list.unshift({ kind: item.kind, name: item.name, x: item.x, y: item.y, id: item.id || null });
  try { localStorage.setItem('navi.recent', JSON.stringify(list.slice(0, 8))); } catch { }
}

function openSearch() {
  $('sheetSearch').classList.add('show');
  $('addrInput').value = '';
  renderSearch();
  setTimeout(() => $('addrInput').focus(), 50);
}

// Збирає результати за запитом. Порожній запит показує нещодавні цілі та АЗС поруч.
function searchItems(q, tab) {
  const here = S.engine?.state.xy || [S.view.cx, S.view.cy];
  const D = (x, y) => Math.hypot(x - here[0], y - here[1]);
  const out = [];
  const norm = (t) => (t || '').toLowerCase().replace(/[’'`ʼ]/g, '').replace(/ґ/g, 'г');
  const qn = norm(q).trim();

  if (!qn) {
    if (tab !== 'fuel') for (const r of recentGet()) out.push({ ...r, sub: 'нещодавнє', d: D(r.x, r.y), group: 'Нещодавні' });
    if (tab !== 'addr') {
      const near = S.fuel.map(f => ({ f, d: D(f.x, f.y) })).sort((a, b) => a.d - b.d).slice(0, 10);
      for (const { f, d } of near) out.push(fuelItem(f, d));
    }
    return out;
  }

  if (tab !== 'addr') {
    for (const f of S.fuel) {
      if (!norm(f.name).includes(qn) && !norm(f.brand).includes(qn)) continue;
      out.push(fuelItem(f, D(f.x, f.y)));
      if (out.length > 40) break;
    }
    out.sort((a, b) => a.d - b.d);
  }
  if (tab !== 'fuel' && S.bld && qn.length >= 2) {
    for (const r of S.bld.search(q, here, 20)) {
      out.push({ kind: r.streetOnly ? 'street' : 'addr', name: r.text, x: r.x, y: r.y, d: r.d,
                 sub: r.streetOnly ? 'вулиця' : 'адреса', group: 'Адреси' });
    }
  }
  return out.slice(0, 40);
}

function fuelItem(f, d) {
  const st = S.fuelStatus[f.id];
  const sub = st ? (st.status === 'ok' ? 'працює · ' : 'не працює · ') + ago(st.t) : 'статус невідомий';
  return { kind: 'fuel', id: f.id, name: f.name || f.brand || 'АЗС', x: f.x, y: f.y, d, sub, group: 'АЗС' };
}

function renderSearch() {
  const q = $('addrInput').value;
  if (!S.bld && !S.fuel.length) { $('addrList').innerHTML = '<div class="row"><div class="name" style="color:var(--muted)">Завантажую дані…</div></div>'; return; }
  const items = searchItems(q, S.searchTab);
  S._res = items;
  if (!items.length) {
    $('addrList').innerHTML = `<div class="row"><div class="name" style="color:var(--muted)">${q.trim() ? 'Нічого не знайдено. Спробуйте «Хрещатик 22» або «ОККО».' : 'Почніть вводити адресу або назву'}</div></div>`;
    return;
  }
  let html = '', group = '';
  items.forEach((r, k) => {
    if (r.group !== group) { group = r.group; html += `<div style="color:var(--muted);font-size:13px;margin:10px 0 2px">${group}</div>`; }
    const icon = r.kind === 'fuel' ? '⛽' : r.kind === 'street' ? '🛣' : '🏠';
    const status = r.kind === 'fuel' ? `<div class="seg"><button data-st="ok" data-k="${k}" class="${S.fuelStatus[r.id]?.status === 'ok' ? 'on ok' : ''}">є</button><button data-st="bad" data-k="${k}" class="${S.fuelStatus[r.id]?.status === 'bad' ? 'on bad' : ''}">нема</button></div>` : '';
    html += `<div class="row"><div class="name">${icon} ${r.name}<small>${r.sub} · ${fmtDist(r.d)}</small></div>${status}<button data-show="1" data-k="${k}" style="flex:none">👁</button><button data-go="1" data-k="${k}" style="flex:none">Маршрут</button></div>`;
  });
  $('addrList').innerHTML = html;
}

$('btnMore').addEventListener('click', () => $('sheetMore').classList.add('show'));
$('btnFlip').addEventListener('click', () => { S.engine.flipTurns(); toast('Знак поворотів перемкнено'); });
$('btnParticles').addEventListener('click', () => { S.showParticles = !S.showParticles; toast(S.showParticles ? 'Гіпотези показано' : 'Гіпотези сховано'); draw(); });
function setOrient(rotate, say = true) {
  S.view.rotate = rotate;
  const b = $('btnOrient');
  if (b) { b.textContent = rotate ? '⬆︎ Курс' : '⬆︎ Пн'; b.classList.toggle('on', rotate); }
  const r = $('btnRotate'); if (r) r.textContent = rotate ? 'Карта за курсом' : 'Карта на північ';
  if (say) toast(rotate ? 'Карта крутиться за курсом' : 'Карта зафіксована на північ');
  try { localStorage.setItem('navi.rotate', rotate ? '1' : '0'); } catch (e) {}
  draw();
}
$('btnRotate').addEventListener('click', () => setOrient(!S.view.rotate));
$('btnOrient').addEventListener('click', () => setOrient(!S.view.rotate));
try { setOrient(localStorage.getItem('navi.rotate') === '1', false); } catch (e) { setOrient(false, false); }
$('btnSearch').addEventListener('click', openSearch);
$('btnCenter').addEventListener('click', () => {
  const st = S.engine?.state;
  if (st?.xy) { S.follow = true; S.view.scale = Math.max(S.view.scale, 1.2); S.view.cx = st.xy[0]; S.view.cy = st.xy[1]; draw(); }
  findMe(true);   // навіть якщо точка вже є — просимо свіжу
});
$('addrInput').addEventListener('input', renderSearch);
document.querySelector('#sheetSearch .tabs').addEventListener('click', (e) => {
  const b = e.target.closest('button'); if (!b) return;
  S.searchTab = b.dataset.tab;
  document.querySelectorAll('#sheetSearch .tabs button').forEach(x => x.classList.toggle('on', x === b));
  renderSearch();
});
$('addrList').addEventListener('click', (e) => {
  const b = e.target.closest('button'); if (!b) return;
  const r = S._res[+b.dataset.k];
  if (!r) return;
  if (b.dataset.st !== undefined) {  // позначка статусу АЗС
    const cur = S.fuelStatus[r.id]?.status;
    setFuelStatus(r.id, cur === b.dataset.st ? null : b.dataset.st);
    renderSearch();
    return;
  }
  closeSheets();
  S.target = { kind: r.kind, id: r.id, x: r.x, y: r.y, name: r.name };
  if (b.dataset.show !== undefined) {
    S.follow = false; S.view.cx = r.x; S.view.cy = r.y; S.view.scale = Math.max(S.view.scale, 1.8);
    draw(); toast(r.name);
    return;
  }
  recentAdd(r);
  routeTo(S.target);
});

$('btnLocateNet').addEventListener('click', () => { closeSheets(); findMe(true); });
$('btnVoice').addEventListener('click', () => { S.voice.setEnabled(!S.voice.enabled); S.voice.warmup(); toast(S.voice.enabled ? (S.voice.hasUk ? 'Голос увімкнено' : 'Голос увімкнено, але українського голосу в системі немає — буде англійський') : 'Голос вимкнено'); $('voiceState').textContent = S.voice.enabled ? 'увімкнено' : 'вимкнено'; });
function updateTrafficInfo() {
  const st = S.traffic.stats();
  $('trafficInfo').textContent = st.edges ? `${st.edges} ділянок, ${st.hours.toFixed(1)} год поїздок` : 'ще нічого не записано';
}
$('btnTraffic').addEventListener('click', () => { S.showTraffic = !S.showTraffic; toast(S.showTraffic ? 'Шар пробок увімкнено' : 'Шар пробок вимкнено'); draw(); });
$('btnTrafficClear').addEventListener('click', () => { S.traffic.clear(); updateTrafficInfo(); draw(); toast('Пам\'ять пробок очищено'); });
setInterval(updateTrafficInfo, 10000);
// --- автозапис поїздок ---
const autoRecOn = () => { try { return localStorage.getItem('navi.autorec') !== '0'; } catch (e) { return true; } };

function startAutoRec() {
  if (S.recording || !S.engine) return;
  S.engine.startRecording();
  S.recording = true;
  updateRecInfo();
}

function updateScreenInfo() {
  const el = $('screenInfo'); if (!el) return;
  el.textContent = { lock: 'екран не гасне (системне утримання)', video: 'екран не гасне (резервний спосіб)', off: 'не вдалося втримати — екран може гаснути' }[Awake.mode] || '—';
}

function updateRecInfo() {
  const el = $('recInfo'); if (!el) return;
  const b = $('btnAutoRec');
  if (!autoRecOn()) { el.textContent = 'вимкнено'; if (b) b.classList.remove('active'); return; }
  if (b) b.classList.add('active');
  const n = S.engine ? S.engine.recordedSamples : 0;
  el.textContent = n ? `записано ${Math.round(n / 50 / 60)} хв (${n.toLocaleString('uk')} семплів)` : 'увімкнено, чекаю на датчики руху';
}
setInterval(() => { updateRecInfo(); updateScreenInfo(); }, 5000);

$('btnAutoRec').addEventListener('click', () => {
  const on = !autoRecOn();
  try { localStorage.setItem('navi.autorec', on ? '1' : '0'); } catch (e) {}
  if (on) { startAutoRec(); toast('Поїздки записуються автоматично'); }
  else { S.engine?.stopRecording(); S.recording = false; toast('Автозапис вимкнено'); }
  updateRecInfo();
});

$('btnRec').addEventListener('click', () => {
  const log = S.engine ? S.engine.exportLog() : [];
  if (!log.length) { toast('Записувати ще нічого — увімкніть датчики руху й проїдьте трохи'); return; }
  const blob = new Blob([JSON.stringify({ origin: S.graph.origin, ver: $('ver')?.textContent || '', log })], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `navi-log-${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.json`;
  document.body.appendChild(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 30000);
  toast(`Збережено ${log.length.toLocaleString('uk')} записів`);
});

// Поточне місце словами: вулиця, по якій їдемо, і найближчий будинок з номером.
let whereCache = { t: 0, text: '' };
function updateWhere() {
  const st = S.engine?.state;
  const el = $('where');
  if (!st || !st.xy) { el.innerHTML = ''; return; }
  const now = Date.now();
  if (now - whereCache.t < 1500) { el.innerHTML = whereCache.text ? `<span>${whereCache.text}</span>` : ''; return; }
  whereCache.t = now;
  let street = st.edge && st.edge.name ? st.edge.name : '';
  let house = '';
  if (S.bld) { const i = S.bld.nearest(st.xy, 80); if (i >= 0) house = S.bld.label(i); }
  // Якщо будинок на тій самій вулиці — не повторюємо її назву двічі
  const sameStreet = street && house && normStreet(house).startsWith(normStreet(street));
  let text;
  if (house && (sameStreet || !street)) text = '📍 ' + house;
  else if (house && street) text = `📍 ${street} · біля ${house}`;
  else if (street) text = '📍 ' + street;
  else text = '📍 поза вулицями';
  whereCache.text = text;
  el.innerHTML = `<span>${text}</span>`;
}

function updateStatus() {
  const eng = S.engine; if (!eng) return;
  const st = eng.tick();
  // Без GPS швидкість — оцінка з датчиків і карти, а не вимір. Показуємо її з «≈»,
  // щоб число не читалося як точне: акселерометр принципово не бачить сталої швидкості.
  const kmh = Math.round(st.speed * 3.6);
  $('speed').textContent = (st.source === 'gps' || !S.sensorsOn) ? String(kmh) : '≈' + kmh;
  if (S.sensorsOn) eng.updateSource();
  const netAcc = st.net ? (st.net.acc < 1000 ? Math.round(st.net.acc) + ' м' : (st.net.acc / 1000).toFixed(1) + ' км') : '';
  const gpsTxt = {
    none: 'Позиції нема', ok: 'Супутники', weak: 'Wi-Fi / слабкий сигнал',
    net: netAcc ? `Мережа зв'язку ±${netAcc}` : "Мережа зв'язку",
    coarse: st.hint ? `Лише приблизно ±${(st.hint.acc / 1000).toFixed(1)} км` : 'Лише приблизно',
    rejected: st.spoofByNet ? 'ПІДМІНА (не збігається з мережею)' : 'ПІДМІНА — ігнорую',
    jammed: netAcc ? `GPS глушиться · мережа ±${netAcc}` : 'GPS глушиться',
  }[st.gps] || '—';
  $('txtGps').textContent = gpsTxt;
  $('dotGps').className = 'dot ' + (st.gps === 'ok' ? 'ok' : st.gps === 'weak' || st.gps === 'net' ? 'warn' : st.gps === 'rejected' || st.gps === 'jammed' ? 'bad' : '');
  let modeTxt = { init: 'немає позиції', hint: 'уточніть «Я тут»', calibrating: 'калібрую осі…', tracking: 'веду' }[st.mode] || st.mode;
  if (st.mode === 'tracking') modeTxt = st.source === 'gps' ? 'веду по GPS' : 'офлайн: датчики + карта';
  $('txtMode').textContent = modeTxt;
  $('dotMode').className = 'dot ' + (st.mode === 'tracking' ? (st.source === 'gps' ? 'ok' : 'warn') : st.mode === 'calibrating' ? 'warn' : '');
  // Частка ваги найкращої дороги мало що каже людині — показуємо радіус і стан словами
  if (!st.xy) $('conf').textContent = st.hint ? `ви десь у колі ±${(st.hint.acc / 1000).toFixed(1)} км` : 'позицію не визначено';
  else {
    const r = st.spread < 1000 ? Math.round(st.spread) + ' м' : (st.spread / 1000).toFixed(1) + ' км';
    const word = st.spread < 40 && st.confidence > 0.5 ? 'точно' : st.spread < 150 ? 'приблизно' : 'уточнюю';
    $('conf').textContent = `${word} ±${r}`;
  }
  if (S.follow && st.xy) { S.view.cx += (st.xy[0] - S.view.cx) * 0.3; S.view.cy += (st.xy[1] - S.view.cy) * 0.3; }
  else if (S.follow && st.hint && !S.hintShown) {
    // показуємо коло цілком, щоб було видно, про яку частину області йдеться
    S.hintShown = true;
    S.view.cx = st.hint.xy[0]; S.view.cy = st.hint.xy[1];
    S.view.scale = clampScale(Math.min(innerWidth, innerHeight) * 0.42 / st.hint.acc);
  }
  $('btnCenter').classList.add('show');
  $('btnCenter').classList.toggle('on', !!S.follow);
  // Діагностика в «⋯»: що саме відповіла система останнього разу
  const gi = $('geoInfo');
  if (gi) {
    if (S.lastGeoErr) gi.textContent = S.lastGeoErr;
    else if (S.lastNetAt) gi.textContent = `останній відгук ${Math.round((Date.now() - S.lastNetAt) / 1000)} с тому` + (st.net ? `, ±${Math.round(st.net.acc)} м` : '');
    else gi.textContent = S.sensorsOn ? 'запит надіслано, відповіді ще немає' : 'натисніть «Старт» або «Де я»';
  }
  updateWhere();
  updateTurn();
}
// Пам'ять пробок: скільки часу й метрів проведено на кожному ребрі (за годинником датчиків)
function trackTraffic() {
  const eng = S.engine; if (!eng || !S.traffic) return;
  const st = eng.state, t = eng.lastStepT;
  if (t == null || !st.edge) { S.track = null; return; }
  const tr = S.track;
  if (!tr || tr.edge !== st.edge.id) {
    if (tr && tr.time >= 1 && tr.good) S.traffic.observe(tr.edge, tr.time, tr.dist);
    S.track = { edge: st.edge.id, t, time: 0, dist: 0, good: true };
    return;
  }
  const dt = t - tr.t; if (dt <= 0 || dt > 5) { tr.t = t; return; }
  tr.t = t; tr.time += dt; tr.dist += st.speed * dt;
  if (st.confidence < 0.6 || st.spread > 60) tr.good = false; // непевна позиція — не вчимось
}
setInterval(() => S.traffic && S.traffic.save(), 30000);
addEventListener('pagehide', () => S.traffic && S.traffic.save());

function loop(now) {
  if (now - S.lastDraw > 66) { updateStatus(); trackTraffic(); draw(); S.lastDraw = now; }
  requestAnimationFrame(loop);
}

// ---------- старт ----------
(async function main() {
  const { graph, fuel, real } = await loadData();
  S.graph = graph; S.fuel = fuel;
  S.traffic = new TrafficMemory(`${graph.meta?.name || 'map'}|${graph.nNodes}|${graph.n}`);
  updateTrafficInfo();
  S.engine = new NavEngine(S.graph, { motion: { gravitySign: IS_IOS ? -1 : 1 } });
  S.voice = new Voice(); $('voiceState').textContent = S.voice.enabled ? 'увімкнено' : 'вимкнено';
  // центр карти: остання збережена позиція, інакше опорна точка (центр Києва) або середина тестового міста
  const saved = JSON.parse(localStorage.getItem('navi.view') || 'null');
  if (saved && real) { S.view.cx = saved.cx; S.view.cy = saved.cy; S.view.scale = saved.scale; }
  else if (real) { S.view.cx = 0; S.view.cy = 0; S.view.scale = 0.6; }
  else { S.view.cx = (graph.bounds.minX + graph.bounds.maxX) / 2; S.view.cy = (graph.bounds.minY + graph.bounds.maxY) / 2; }
  setInterval(() => { try { localStorage.setItem('navi.view', JSON.stringify({ cx: S.view.cx, cy: S.view.cy, scale: S.view.scale })); } catch { } }, 5000);
  resize(); requestAnimationFrame(loop);
  if ('serviceWorker' in navigator) navigator.serviceWorker.register('sw.js').catch(() => { });
  updateRecInfo(); updateScreenInfo();
  // Нічого натискати не треба: позиція вмикається сама, датчики руху — щойно буде дотик.
  if (real) autoStart();
  const seen = localStorage.getItem('navi.seen');
  if (!seen) { hint('Застосунок уже шукає вас. Торкніться екрана — увімкнуться датчики руху, і він вестиме по них і по карті, навіть коли GPS глушать. «Пошук» — адреси та АЗС.'); localStorage.setItem('navi.seen', '1'); setTimeout(() => hint(''), 14000); }
})();
