// Консольна перевірка точності: node tools/sim.mjs [кількість поїздок]
import { makeTestCity } from '../js/testcity.js';
import { RoadGraph } from '../js/graph.js';
import { NavEngine } from '../js/engine.js';
import { simulateDrive, rotationMatrix } from '../js/simulator.js';
import { dist, mulberry32 } from '../js/geo.js';

const N = parseInt(process.argv[2] || '12', 10);
const city = makeTestCity(7);
const graph = RoadGraph.fromJSON(city);
const rng = mulberry32(99);

function pct(arr, p) { const a = [...arr].sort((x, y) => x - y); return a[Math.min(a.length - 1, Math.floor(p * a.length))]; }

function runTrip(k, scenario) {
  let from, to, sim;
  for (let tries = 0; tries < 20; tries++) {
    from = Math.floor(rng() * graph.nNodes); to = Math.floor(rng() * graph.nNodes);
    if (from === to || dist(graph.nodeXY(from), graph.nodeXY(to)) < 900) continue;
    try { sim = simulateDrive(graph, from, to, { seed: 1000 + k, ios: scenario.ios, R: rotationMatrix(rng() * 6.28, 0.6 + rng() * 0.9, (rng() - 0.5) * 0.6), cruise: 10 + rng() * 6 }); break; } catch { }
  }
  if (!sim) return null;
  const eng = new NavEngine(graph, { motion: { gravitySign: scenario.ios ? -1 : 1 }, filter: { rng: mulberry32(5 + k) } });
  const gpsUntil = scenario.gpsSeconds;             // GPS працює перші N секунд, далі РЕБ
  const proj = eng.proj;
  const errs = [], errsAll = []; let onEdge = 0, cnt = 0, firstEst = null;
  if (scenario.manualStart) eng.setPositionXY([sim.truth[0].x, sim.truth[0].y], 120);
  let lastGps = -10;
  for (let i = 0; i < sim.samples.length; i++) {
    const smp = sim.samples[i], tr = sim.truth[i];
    eng.pushMotion(smp);
    if (smp.t < gpsUntil && smp.t - lastGps >= 1) {
      lastGps = smp.t;
      const [lat, lon] = proj.toLL(tr.x + 6 * (rng() - 0.5) * 2, tr.y + 6 * (rng() - 0.5) * 2);
      eng.pushGps({ t: smp.t, lat, lon, accuracy: 8, speed: tr.v });
    } else if (scenario.spoof && smp.t - lastGps >= 1) {
      lastGps = smp.t; // підміна: координати стрибають на 1–3 км
      const [lat, lon] = proj.toLL(tr.x + 1000 + rng() * 2000, tr.y - 1500 + rng() * 3000);
      eng.pushGps({ t: smp.t, lat, lon, accuracy: 5, speed: 0 });
    }
    if (eng.state.xy && smp.t >= gpsUntil && i % 25 === 0) {
      const e = dist(eng.state.xy, [tr.x, tr.y]);
      errs.push(e); cnt++;
      if (e < 40) onEdge++;
      if (firstEst === null) firstEst = smp.t;
    }
  }
  const dr = eng.motion;
  return { k, len: Math.round(sim.total), dur: Math.round(sim.duration), gpsOff: Math.round(sim.duration - gpsUntil), med: pct(errs, 0.5), p90: pct(errs, 0.9), max: Math.max(...errs), ok: onEdge / Math.max(1, cnt), scale: dr.accelScale, fwdConf: dr.fwdConfidence };
}

const scenarios = [
  { name: 'GPS 60 с, далі РЕБ (iOS)', gpsSeconds: 60, ios: true },
  { name: 'GPS 60 с, далі ПІДМІНА координат (iOS)', gpsSeconds: 60, ios: true, spoof: true },
  { name: 'Без GPS взагалі, ручний старт ±120 м (iOS)', gpsSeconds: -1, ios: true, manualStart: true },
  { name: 'GPS 60 с, далі РЕБ (Android)', gpsSeconds: 60, ios: false },
];
for (const sc of scenarios) {
  console.log('\n=== ' + sc.name + ' ===');
  console.log('поїздка  довж.м  час.с  безGPS.с  медіана.м  p90.м  макс.м  на_дорозі%  масштаб  вперед');
  const agg = [];
  for (let k = 0; k < N; k++) {
    const r = runTrip(k, sc); if (!r) continue; agg.push(r);
    console.log(`${String(r.k).padStart(7)}  ${String(r.len).padStart(6)}  ${String(r.dur).padStart(5)}  ${String(r.gpsOff).padStart(8)}  ${r.med.toFixed(0).padStart(9)}  ${r.p90.toFixed(0).padStart(5)}  ${r.max.toFixed(0).padStart(6)}  ${(r.ok * 100).toFixed(0).padStart(10)}  ${r.scale.toFixed(2).padStart(7)}  ${r.fwdConf.toFixed(2).padStart(6)}`);
  }
  const medAll = pct(agg.map(a => a.med), 0.5), p90All = pct(agg.map(a => a.p90), 0.5), okAll = agg.reduce((p, a) => p + a.ok, 0) / agg.length;
  console.log(`РАЗОМ: медіана похибки ${medAll.toFixed(0)} м, типовий p90 ${p90All.toFixed(0)} м, на правильній дорозі ${(okAll * 100).toFixed(0)}% часу`);
}
