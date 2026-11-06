// Наві — застосунок. Карта на canvas, датчики, GPS, АЗС зі статусами, маршрут, демо.
import { RoadGraph } from './graph.js';
import { NavEngine } from './engine.js';
import { route, instructions } from './astar.js';
import { makeTestCity } from './testcity.js';
import { simulateDrive } from './simulator.js';
import { dist, angleDiff } from './geo.js';
import { TrafficMemory, trafficColor, VMAX } from './traffic.js';
import { Buildings } from './buildings.js';
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
  demo: null, hereMode: false,
  fuelStatus: JSON.parse(localStorage.getItem('navi.fuelStatus') || '{}'),
  sensorsOn: false, gpsWatch: null, lastDraw: 0, recording: false,
  traffic: null, showTraffic: true, track: null,
  bld: null, voice: null, lastSpoken: null,
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
    fetch('data/buildings.bin').then(async r => { if (!r.ok) return; S.bld = Buildings.fromBinary(await r.arrayBuffer()); $('dataInfo').textContent += `, ${S.bld.count} будинків, ${S.bld.entranceCount} під'їздів`; draw(); }).catch(() => { });
    return { graph, fuel, real: true };
  } catch (e) {
    const city = makeTestCity(7);
    const graph = RoadGraph.fromJSON(city);
    $('dataInfo').textContent = `Тестове місто (справжньої карти ще немає): ${graph.nNodes} вузлів, ${graph.n} вулиць, ${city.fuel.length} АЗС`;
    toast('Реальна карта не знайдена — працюємо на тестовому місті');
    hint('');
    return { graph, fuel: city.fuel, real: false };
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
  // АЗС
  for (const f of S.fuel) {
    const p = worldToScreen(f.x, f.y);
    if (p[0] < -40 || p[1] < -40 || p[0] > w + 40 || p[1] > h + 40) continue;
    const st = S.fuelStatus[f.id]?.status;
    const col = st === 'ok' ? '#3ddc84' : st === 'bad' ? '#ff5a5f' : '#8a98a8';
    const r = 7 * DPR;
    ctx.fillStyle = col; ctx.beginPath(); ctx.arc(p[0], p[1], r, 0, 7); ctx.fill();
    ctx.fillStyle = '#0b0f14'; ctx.font = `bold ${9 * DPR}px sans-serif`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText('⛽', p[0], p[1] + 0.5 * DPR);
    if (v.scale > 1.2) { ctx.fillStyle = col; ctx.font = `${10 * DPR}px sans-serif`; ctx.textBaseline = 'alphabetic'; ctx.fillText(f.brand || f.name || '', p[0], p[1] - r - 3 * DPR); }
    if (S.target && S.target.id === f.id) { ctx.strokeStyle = '#4da3ff'; ctx.lineWidth = 2 * DPR; ctx.beginPath(); ctx.arc(p[0], p[1], r + 5 * DPR, 0, 7); ctx.stroke(); }
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
  if (S.demo && S.demo.truth) {
    const t = S.demo.truth; const p = worldToScreen(t.x, t.y);
    ctx.strokeStyle = '#3ddc84'; ctx.lineWidth = 2 * DPR; ctx.setLineDash([4 * DPR, 3 * DPR]); ctx.beginPath(); ctx.arc(p[0], p[1], 10 * DPR, 0, 7); ctx.stroke(); ctx.setLineDash([]);
  }
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
  // масштабна лінійка
  const meters = [10, 20, 50, 100, 200, 500, 1000, 2000, 5000].find(m => m * v.scale * DPR > 60 * DPR) || 5000;
  const px = meters * v.scale * DPR;
  ctx.strokeStyle = '#c8d3df'; ctx.lineWidth = 2 * DPR; ctx.beginPath(); ctx.moveTo(14 * DPR, h - 150 * DPR); ctx.lineTo(14 * DPR + px, h - 150 * DPR); ctx.stroke();
  ctx.fillStyle = '#c8d3df'; ctx.font = `${11 * DPR}px sans-serif`; ctx.textAlign = 'left'; ctx.fillText(meters >= 1000 ? (meters / 1000) + ' км' : meters + ' м', 14 * DPR, h - 156 * DPR);
}

// ---------- жести ----------
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
    S.view.scale = Math.max(0.02, Math.min(12, gesture.startView.scale * d1 / Math.max(1, d0))); gesture.moved = true;
  }
  draw();
});
const endGesture = (e) => {
  if (!gesture) return;
  if (!gesture.moved && gesture.pts.size === 1) onTap(e.clientX, e.clientY);
  gesture.pts.delete(e.pointerId); if (!gesture.pts.size) gesture = null; else { gesture.start = [...gesture.pts.values()]; gesture.startView = { ...S.view }; gesture.moved = true; }
};
canvas.addEventListener('pointerup', endGesture); canvas.addEventListener('pointercancel', endGesture);
canvas.addEventListener('wheel', (e) => { e.preventDefault(); S.view.scale = Math.max(0.02, Math.min(12, S.view.scale * (e.deltaY < 0 ? 1.15 : 0.87))); draw(); }, { passive: false });

function onTap(sx, sy) {
  const p = screenToWorld(sx, sy);
  if (S.hereMode) {
    S.engine.setPositionXY(p, 80); S.hereMode = false; S.follow = true; $('btnHere').classList.remove('active'); hint('');
    const bi = S.bld ? S.bld.nearest(p, 60) : -1;
    toast(bi >= 0 ? `Позицію встановлено біля ${S.bld.label(bi)}` : 'Позицію встановлено'); draw(); return;
  }
  // тап по АЗС
  let best = null, bd = 25 / S.view.scale;
  for (const f of S.fuel) { const d = dist(p, [f.x, f.y]); if (d < bd) { bd = d; best = f; } }
  if (best) { openFuel(best); return; }
}

// ---------- АЗС ----------
function setFuelStatus(id, status) {
  if (status) S.fuelStatus[id] = { status, t: Date.now() }; else delete S.fuelStatus[id];
  localStorage.setItem('navi.fuelStatus', JSON.stringify(S.fuelStatus));
  renderFuelList(); draw();
}
function ago(t) { const m = Math.round((Date.now() - t) / 60000); return m < 1 ? 'щойно' : m < 60 ? `${m} хв тому` : m < 1440 ? `${Math.round(m / 60)} год тому` : `${Math.round(m / 1440)} дн тому`; }
function renderFuelList() {
  const here = S.engine?.state.xy || [S.view.cx, S.view.cy];
  const list = S.fuel.map(f => ({ f, d: dist(here, [f.x, f.y]) })).sort((a, b) => a.d - b.d).slice(0, 25);
  $('fuelList').innerHTML = list.map(({ f, d }) => {
    const st = S.fuelStatus[f.id];
    return `<div class="row" data-id="${f.id}">
      <div class="name">${f.name || f.brand || 'АЗС'}<small>${f.brand && f.name !== f.brand ? f.brand + ' · ' : ''}${d < 1000 ? Math.round(d) + ' м' : (d / 1000).toFixed(1) + ' км'}${st ? ' · ' + (st.status === 'ok' ? 'працює' : 'не працює') + ', ' + ago(st.t) : ' · статус невідомий'}</small></div>
      <div class="seg"><button data-st="ok" class="${st?.status === 'ok' ? 'on ok' : ''}">працює</button><button data-st="bad" class="${st?.status === 'bad' ? 'on bad' : ''}">ні</button><button data-st="">?</button></div>
      <button data-go="${f.id}" style="flex:none">→</button></div>`;
  }).join('');
}
$('fuelList').addEventListener('click', (e) => {
  const b = e.target.closest('button'); if (!b) return;
  const id = b.closest('.row').dataset.id;
  if (b.dataset.go !== undefined) { routeTo(S.fuel.find(f => f.id === id)); closeSheets(); return; }
  setFuelStatus(id, b.dataset.st || null);
});
function openFuel(f) { renderFuelList(); $('sheetFuel').classList.add('show'); const row = $('fuelList').querySelector(`[data-id="${f.id}"]`); row?.scrollIntoView({ block: 'center' }); row && (row.style.background = 'rgba(77,163,255,.12)'); }

// ---------- маршрут ----------
function routeTo(f) {
  if (!S.engine?.state.xy) { toast('Спочатку визначте позицію («Я тут» або GPS)'); return; }
  const g = S.graph;
  const froms = [S.engine.state.edge ? S.engine.state.edge.b : null, ...g.nearestNodes(S.engine.state.xy, 400, 2, 4)].filter(x => x !== null);
  const tos = [...g.nearestNodes([f.x, f.y], 400, 2, 6), g.nearestNode([f.x, f.y])];
  let r = null;
  for (const from of froms) { for (const to of tos) { r = route(g, from, to, S.traffic.speedFn(), VMAX); if (r) break; } if (r) break; }
  if (!r) { toast('Маршрут не знайдено'); return; }
  S.routeInfo = r; S.target = f; S.routeSteps = instructions(g, r); S.lastSpoken = null;
  const msg = `Маршрут: ${(r.length / 1000).toFixed(1)} км · ~${Math.max(1, Math.round(r.time / 60))} хв`;
  toast(msg + (S.traffic.hasData ? ' з урахуванням типових пробок' : ''));
  S.voice?.say(`Маршрут побудовано, ${(r.length / 1000).toFixed(1).replace('.', ' і ')} кілометра`, 'route', 0);
  draw(); updateTurn();
}
function routeToNearestWorking() {
  const here = S.engine?.state.xy; if (!here) { toast('Спочатку визначте позицію («Я тут» або GPS)'); return; }
  const ok = S.fuel.filter(f => S.fuelStatus[f.id]?.status === 'ok');
  const pool = ok.length ? ok : S.fuel.filter(f => S.fuelStatus[f.id]?.status !== 'bad');
  if (!pool.length) { toast('Немає АЗС без позначки «не працює»'); return; }
  const best = pool.map(f => ({ f, d: dist(here, [f.x, f.y]) })).sort((a, b) => a.d - b.d)[0].f;
  if (!ok.length) toast('Робочих АЗС не позначено — веду до найближчої з невідомим статусом');
  routeTo(best);
}
function updateTurn() {
  const el = $('turn');
  if (!S.routeInfo || !S.engine?.state.edge) { el.classList.remove('show'); return; }
  const g = S.graph, st = S.engine.state;
  // знайти поточне ребро у маршруті; якщо немає — перебудувати
  const idx = S.routeInfo.edges.indexOf(st.edge.id);
  if (idx < 0) { if (S.target) { const from = st.edge.b; let r = null; for (const to of g.nearestNodes([S.target.x, S.target.y], 400, 2, 6)) { r = route(g, from, to, S.traffic.speedFn(), VMAX); if (r) break; } if (r) { S.routeInfo = r; S.routeSteps = instructions(g, r); } } el.classList.remove('show'); return; }
  // відстань до наступного повороту
  let d = st.edge.len - (st.s ?? 0), text = 'Прибуття', arrow = '🏁';
  for (let i = idx + 1; i < S.routeInfo.edges.length; i++) {
    const e = g.edge(S.routeInfo.edges[i]), prev = g.edge(S.routeInfo.edges[i - 1]);
    const deg = angleDiff(e.hdg, prev.hdg) * 180 / Math.PI;
    if (Math.abs(deg) > 30) { text = (deg > 0 ? 'праворуч' : 'ліворуч') + (e.name ? ' на ' + e.name : ''); arrow = deg > 0 ? '↱' : '↰'; break; }
    d += e.len;
  }
  $('turnDist').textContent = `${arrow} ${d < 1000 ? Math.round(d / 10) * 10 + ' м' : (d / 1000).toFixed(1) + ' км'}`;
  $('turnText').textContent = text; el.classList.add('show');
  // голос: на 500 м, 200 м і ~60 м до повороту (кожен поріг — один раз на поворот)
  const stage = d > 500 ? 0 : d > 200 ? 1 : d > 60 ? 2 : 3;
  const key = idx + '|' + text;
  if (!S.lastSpoken || S.lastSpoken.key !== key) S.lastSpoken = { key, stage: -1 };
  if (stage > S.lastSpoken.stage && stage >= 1) { S.lastSpoken.stage = stage; S.voice?.say(turnPhrase(stage === 3 ? 0 : stage === 2 ? 200 : 500, text), key + stage, 0); }
  if (d < 30 && text === 'Прибуття') { toast('Ви на місці'); S.voice?.say('Ви прибули', 'arrive', 0); S.routeInfo = null; S.target = null; el.classList.remove('show'); }
}

// ---------- датчики та GPS ----------
async function startSensors() {
  if (S.sensorsOn) { stopSensors(); return; }
  try {
    if (typeof DeviceMotionEvent !== 'undefined' && typeof DeviceMotionEvent.requestPermission === 'function') {
      const r = await DeviceMotionEvent.requestPermission(); if (r !== 'granted') { toast('Немає дозволу на датчики руху'); return; }
    }
  } catch (e) { toast('Дозвіл на датчики: ' + e.message); }
  if (!('DeviceMotionEvent' in window)) { toast('Датчики руху недоступні в цьому браузері'); return; }
  S.t0 = null;
  addEventListener('devicemotion', onMotion);
  S.sensorsOn = true; $('btnSensors').textContent = '⏹ Датчики'; $('btnSensors').classList.remove('primary');
  startGps(); requestWakeLock();
  hint('Закріпіть телефон у тримачі. Перші секунди стійте на місці, потім рушайте — застосунок сам визначить, де «вперед».');
  setTimeout(() => hint(''), 9000);
}
function stopSensors() { removeEventListener('devicemotion', onMotion); S.sensorsOn = false; $('btnSensors').textContent = '▶ Датчики'; $('btnSensors').classList.add('primary'); if (S.gpsWatch) navigator.geolocation.clearWatch(S.gpsWatch); S.gpsWatch = null; }
function onMotion(e) {
  const ag = e.accelerationIncludingGravity, rr = e.rotationRate;
  if (!ag || ag.x == null || !rr || rr.alpha == null) return;
  const t = (e.timeStamp || performance.now()) / 1000;
  // W3C: rotationRate.alpha — навколо z, beta — навколо x, gamma — навколо y (град/с)
  const D2R = Math.PI / 180;
  S.engine.pushMotion({ t, acc: [ag.x, ag.y, ag.z], gyro: [rr.beta * D2R, rr.gamma * D2R, rr.alpha * D2R] });
}
function startGps() {
  if (!('geolocation' in navigator)) return;
  S.gpsWatch = navigator.geolocation.watchPosition((pos) => {
    const c = pos.coords;
    S.engine.pushGps({ t: pos.timestamp / 1000, lat: c.latitude, lon: c.longitude, accuracy: c.accuracy, speed: c.speed });
    // якщо немає GPS-фіксу всередині карти — повідомити
  }, (err) => { $('txtGps').textContent = 'GPS: ' + (err.code === 1 ? 'заборонено' : 'немає сигналу'); $('dotGps').className = 'dot bad'; }, { enableHighAccuracy: true, maximumAge: 1000, timeout: 10000 });
}
let wakeLock = null;
async function requestWakeLock() { try { if ('wakeLock' in navigator) { wakeLock = await navigator.wakeLock.request('screen'); document.addEventListener('visibilitychange', async () => { if (document.visibilityState === 'visible' && S.sensorsOn) wakeLock = await navigator.wakeLock.request('screen'); }); } } catch { } }

// ---------- демо ----------
// Випадковий вузол неподалік центру екрана (на великій карті — у радіусі ~2.5 км)
function randomNode() {
  const g = S.graph;
  if (g.nNodes < 5000) return Math.floor(Math.random() * g.nNodes);
  for (let t = 0; t < 50; t++) {
    const p = [S.view.cx + (Math.random() - 0.5) * 5000, S.view.cy + (Math.random() - 0.5) * 5000];
    const ids = g.edgesInBox(p[0] - 300, p[1] - 300, p[0] + 300, p[1] + 300, 2);
    if (ids.length) return g.a[ids[Math.floor(Math.random() * ids.length)]];
  }
  return Math.floor(Math.random() * g.nNodes);
}
function startDemo() {
  if (S.demo) { stopDemo(); return; }
  const g = S.graph;
  // випадкова поїздка завдовжки понад 1.5 км
  let sim = null;
  for (let tries = 0; tries < 30 && !sim; tries++) {
    const a = randomNode(), b = randomNode();
    if (dist(g.nodeXY(a), g.nodeXY(b)) < 1200) continue;
    try { sim = simulateDrive(g, a, b, { seed: Math.floor(Math.random() * 1e6), ios: true, hz: 50 }); } catch { }
  }
  if (!sim) { toast('Не вдалося згенерувати поїздку'); return; }
  S.engine = new NavEngine(g, { motion: { gravitySign: -1 } });
  S.engine.setPositionXY([sim.truth[0].x, sim.truth[0].y], 100);
  S.demo = { sim, i: 0, truth: sim.truth[0], gpsUntil: 40, lastGps: -10, t0: performance.now(), speedup: 2 };
  S.routeInfo = null; S.target = null; S.follow = true;
  $('btnDemo').textContent = '⏹ Демо'; $('btnDemo').classList.add('active');
  hint('Демо: зелене пунктирне коло — справжнє положення авто, синя стрілка — оцінка без GPS. GPS «працює» лише перші 40 с, далі — РЕБ.');
  setTimeout(() => hint(''), 8000);
  requestAnimationFrame(demoTick);
}
function stopDemo() { S.demo = null; $('btnDemo').textContent = '🎬 Демо'; $('btnDemo').classList.remove('active'); }
function demoTick(now) {
  const d = S.demo; if (!d) return;
  const simT = (now - d.t0) / 1000 * d.speedup;
  while (d.i < d.sim.samples.length && d.sim.samples[d.i].t <= simT) {
    const s = d.sim.samples[d.i], tr = d.sim.truth[d.i];
    S.engine.pushMotion(s);
    if (s.t < d.gpsUntil && s.t - d.lastGps >= 1) {
      d.lastGps = s.t; const [lat, lon] = S.engine.proj.toLL(tr.x + (Math.random() - .5) * 10, tr.y + (Math.random() - .5) * 10);
      S.engine.pushGps({ t: s.t, lat, lon, accuracy: 8, speed: tr.v });
    } else if (s.t >= d.gpsUntil) { S.engine.state.gps = 'jammed'; }
    d.truth = tr; d.i++;
  }
  if (d.i >= d.sim.samples.length) { toast('Демо-поїздка завершена'); stopDemo(); }
  requestAnimationFrame(demoTick);
}

// ---------- інтерфейс ----------
function toast(msg) { const t = $('toast'); t.textContent = msg; t.classList.add('show'); clearTimeout(t._h); t._h = setTimeout(() => t.classList.remove('show'), 3500); }
function hint(msg) { const h = $('hint'); h.textContent = msg; h.classList.toggle('show', !!msg); }
function closeSheets() { document.querySelectorAll('.sheet').forEach(s => s.classList.remove('show')); }
document.querySelectorAll('[data-close]').forEach(b => b.addEventListener('click', () => $(b.dataset.close).classList.remove('show')));

$('btnSensors').addEventListener('click', () => { S.voice?.warmup(); startSensors(); });
$('btnDemo').addEventListener('click', () => S.voice?.warmup(), { capture: true });
$('btnHere').addEventListener('click', () => { S.hereMode = !S.hereMode; $('btnHere').classList.toggle('active', S.hereMode); hint(S.hereMode ? 'Перетягніть карту так, щоб перехрестя, на якому ви стоїте, було під прицілом, і торкніться його.' : ''); draw(); });
$('btnFuel').addEventListener('click', () => { renderFuelList(); $('sheetFuel').classList.add('show'); });
$('btnRoute').addEventListener('click', routeToNearestWorking);
$('btnSearch').addEventListener('click', () => { if (!S.bld) { toast('Адреси ще не завантажені (немає data/buildings.bin)'); return; } $('sheetSearch').classList.add('show'); $('addrInput').value = ''; $('addrList').innerHTML = ''; setTimeout(() => $('addrInput').focus(), 50); });
$('addrInput').addEventListener('input', () => {
  const q = $('addrInput').value; const near = S.engine?.state.xy || [S.view.cx, S.view.cy];
  const res = S.bld.search(q, near);
  $('addrList').innerHTML = res.map((r, k) => `<div class="row" data-k="${k}"><div class="name">${r.text}<small>${r.d < 1000 ? Math.round(r.d) + ' м' : (r.d / 1000).toFixed(1) + ' км'}</small></div><button data-show="${k}" style="flex:none">Показати</button><button data-go="${k}" style="flex:none">→</button></div>`).join('') || (q.length > 1 ? '<div class="row"><div class="name" style="color:var(--muted)">Нічого не знайдено. Спробуйте «Хрещатик 22».</div></div>' : '');
  S._addrRes = res;
});
$('addrList').addEventListener('click', (e) => {
  const b = e.target.closest('button'); if (!b) return;
  const r = S._addrRes[+(b.dataset.go ?? b.dataset.show)]; if (!r) return;
  closeSheets();
  if (b.dataset.show !== undefined || !S.engine?.state.xy) { S.follow = false; S.view.cx = r.x; S.view.cy = r.y; S.view.scale = Math.max(S.view.scale, 1.8); S.target = { kind: 'addr', x: r.x, y: r.y, name: r.text }; draw(); if (!S.engine?.state.xy && b.dataset.go !== undefined) toast('Спочатку визначте позицію («Я тут» або GPS)'); return; }
  routeTo({ kind: 'addr', id: 'addr', x: r.x, y: r.y, name: r.text });
});
$('btnDemo').addEventListener('click', startDemo);
$('btnMore').addEventListener('click', () => $('sheetMore').classList.add('show'));
$('btnFlip').addEventListener('click', () => { S.engine.flipTurns(); toast('Знак поворотів перемкнено'); });
$('btnParticles').addEventListener('click', () => { S.showParticles = !S.showParticles; draw(); });
$('btnRotate').addEventListener('click', () => { S.view.rotate = !S.view.rotate; draw(); });
function updateTrafficInfo() {
  const st = S.traffic.stats();
  $('trafficInfo').textContent = st.edges ? `${st.edges} ділянок, ${st.hours.toFixed(1)} год поїздок` : 'ще нічого не записано';
}
$('btnVoice').addEventListener('click', () => { S.voice.setEnabled(!S.voice.enabled); S.voice.warmup(); toast(S.voice.enabled ? (S.voice.hasUk ? 'Голос увімкнено' : 'Голос увімкнено, але українського голосу в системі немає — буде англійський') : 'Голос вимкнено'); $('voiceState').textContent = S.voice.enabled ? 'увімкнено' : 'вимкнено'; });
$('btnTraffic').addEventListener('click', () => { S.showTraffic = !S.showTraffic; toast(S.showTraffic ? 'Шар пробок увімкнено' : 'Шар пробок вимкнено'); draw(); });
$('btnTrafficClear').addEventListener('click', () => { S.traffic.clear(); updateTrafficInfo(); draw(); toast('Пам\'ять пробок очищено'); });
$('btnTrafficDemo').addEventListener('click', () => {
  // Швидко «наїжджаємо» історію: 25 випадкових поїздок по тестовому місту за годинником зараз
  const g = S.graph; let n = 0;
  for (let k = 0; k < 25; k++) {
    const a = randomNode(), b = randomNode();
    if (dist(g.nodeXY(a), g.nodeXY(b)) < 800) continue;
    let sim; try { sim = simulateDrive(g, a, b, { seed: Math.floor(Math.random() * 1e6), hz: 10 }); } catch { continue; }
    let cur = -1, time = 0, distM = 0, lastT = 0;
    for (const tr of sim.truth) {
      const eid = sim.route.edges[tr.seg];
      if (eid !== cur) { if (cur >= 0) S.traffic.observe(cur, time, distM); cur = eid; time = 0; distM = 0; }
      const dt = tr.t - lastT; lastT = tr.t; time += dt; distM += tr.v * dt;
    }
    if (cur >= 0) S.traffic.observe(cur, time, distM);
    n++;
  }
  S.traffic.save(); updateTrafficInfo(); closeSheets(); draw();
  toast(`Записано ${n} демо-поїздок — червоним показано типово повільні вулиці о цій годині`);
});
setInterval(updateTrafficInfo, 10000);
$('btnRec').addEventListener('click', () => {
  if (!S.recording) { S.engine.startRecording(); S.recording = true; $('btnRec').textContent = 'Стоп і зберегти'; toast('Запис датчиків увімкнено'); }
  else { const log = S.engine.stopRecording(); S.recording = false; $('btnRec').textContent = 'Старт'; const blob = new Blob([JSON.stringify({ origin: S.graph.origin, log })], { type: 'application/json' }); const a = document.createElement('a'); a.href = URL.createObjectURL(blob); a.download = `navi-log-${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.json`; document.body.appendChild(a); a.click(); a.remove(); toast(`Збережено ${log.length} записів`); }
});

function updateStatus() {
  const eng = S.engine; if (!eng) return;
  const st = eng.state;
  $('speed').textContent = Math.round(st.speed * 3.6);
  const gpsTxt = { none: 'GPS: нема даних', ok: 'GPS: є', rejected: 'GPS: ПІДМІНА — ігнорую', jammed: 'GPS: глушиться' }[st.gps] || 'GPS: —';
  $('txtGps').textContent = gpsTxt; $('dotGps').className = 'dot ' + (st.gps === 'ok' ? 'ok' : st.gps === 'rejected' ? 'warn' : st.gps === 'jammed' ? 'bad' : '');
  const modeTxt = { init: 'немає позиції', calibrating: 'калібрую осі…', tracking: 'веду без GPS' }[st.mode] || st.mode;
  $('txtMode').textContent = modeTxt; $('dotMode').className = 'dot ' + (st.mode === 'tracking' ? 'ok' : st.mode === 'calibrating' ? 'warn' : '');
  $('conf').textContent = st.xy ? `упевненість ${Math.round(st.confidence * 100)}% · ±${Math.round(st.spread)} м` : 'упевненість —';
  if (S.follow && st.xy) { S.view.cx += (st.xy[0] - S.view.cx) * 0.3; S.view.cy += (st.xy[1] - S.view.cy) * 0.3; }
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
  const seen = localStorage.getItem('navi.seen');
  if (!seen) { hint('Це прототип навігації без GPS. Натисніть «Демо», щоб побачити, як працює алгоритм на тестовому місті, або «Датчики» в авто.'); localStorage.setItem('navi.seen', '1'); setTimeout(() => hint(''), 12000); }
})();
