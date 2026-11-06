// Синтетичне тестове місто для перевірки алгоритму, поки немає реальної карти.
// Нерегулярна сітка з односторонніми вулицями, пропущеними зв'язками, діагоналями та АЗС.
import { mulberry32 } from './geo.js';

const STREETS_NS = ['Соборна', 'Шевченка', 'Франка', 'Лесі Українки', 'Грушевського', 'Богуна', 'Сагайдачного', 'Мазепи', 'Хмельницького', 'Вишневецького', 'Орлика', 'Дорошенка'];
const STREETS_EW = ['Перемоги', 'Героїв', 'Незалежності', 'Садова', 'Польова', 'Лісова', 'Вокзальна', 'Заводська', 'Київська', 'Шкільна', 'Молодіжна', 'Зелена'];

export function makeTestCity(seed = 7, nx = 11, ny = 9, step = 220) {
  const rng = mulberry32(seed);
  const nodes = [], id = (i, j) => i * ny + j;
  for (let i = 0; i < nx; i++) for (let j = 0; j < ny; j++) {
    // нерегулярність: зсув вузлів до 25% кроку
    nodes.push([i * step + (rng() - 0.5) * step * 0.5, j * step + (rng() - 0.5) * step * 0.5]);
  }
  const edges = [];
  for (let i = 0; i < nx; i++) for (let j = 0; j < ny; j++) {
    // кожна 3-тя вулиця — магістраль; дві магістралі «забиті» (демо пробок)
    const clsEW = j % 3 === 1 ? 'primary' : 'residential', clsNS = i % 3 === 1 ? 'primary' : 'residential';
    const jamEW = (j === 4) ? 0.25 : 1, jamNS = (i === 4) ? 0.35 : 1;
    if (i + 1 < nx && rng() > 0.08) edges.push({ a: id(i, j), b: id(i + 1, j), oneway: rng() < 0.18, name: 'вул. ' + STREETS_EW[j % STREETS_EW.length], cls: clsEW, jam: jamEW });
    if (j + 1 < ny && rng() > 0.08) edges.push({ a: id(i, j), b: id(i, j + 1), oneway: rng() < 0.18, name: 'вул. ' + STREETS_NS[i % STREETS_NS.length], cls: clsNS, jam: jamNS });
    if (i + 1 < nx && j + 1 < ny && rng() < 0.06) edges.push({ a: id(i, j), b: id(i + 1, j + 1), oneway: false, name: 'пров. Діагональний', cls: 'service' });
  }
  // АЗС: ~12 станцій біля вузлів
  const brands = ['OKKO', 'WOG', 'SOCAR', 'UPG', 'Shell', 'БРСМ', 'Укрнафта', 'KLO'];
  const fuel = [];
  for (let k = 0; k < 12; k++) {
    const n = Math.floor(rng() * nodes.length);
    const p = nodes[n];
    fuel.push({ id: 'f' + k, x: p[0] + (rng() - 0.5) * 60, y: p[1] + (rng() - 0.5) * 60, brand: brands[k % brands.length], name: brands[k % brands.length] + ' №' + (k + 1) });
  }
  // origin: умовно центр Києва, щоб координати були правдоподібні
  return { origin: [50.4501, 30.5234], nodes, edges, fuel, meta: { name: 'Тестове місто', nx, ny } };
}
