// Service worker: офлайн-робота + оновлення коду.
//
// Два окремі сховища:
//   SHELL — код (html/js/іконки). Кожна версія — цілісний набір у своєму кеші.
//   DATA  — карта, будинки, АЗС, назви місцевостей: десятки мегабайтів. Лежить окремо,
//           щоб оновлення коду НЕ змушувало телефон качати їх знову.
const SHELL_CACHE = 'navi-shell-v0.8.29';
const DATA_CACHE = 'navi-data-v1';
const KEEP = [SHELL_CACHE, DATA_CACHE];
const SHELL = ['./', './index.html', './manifest.webmanifest', './keep.mp4',
  './js/app.js', './js/engine.js', './js/motion.js', './js/filter.js', './js/graph.js',
  './js/astar.js', './js/wasm-route.js', './js/wasm-bin.js', './js/geo.js', './js/traffic.js', './js/buildings.js', './js/voice.js',
  './icons/icon-180.png', './icons/icon-192.png', './icons/icon-512.png'];

// Код встановлюється лише цілим набором однієї версії: усі js і index.html мають завантажитися,
// інакше встановлення не вдається й лишається попередня (цілісна) версія — браузер спробує
// ще раз пізніше. Іконки й відео необов'язкові. Так телефон не отримає суміш старих і нових модулів.
const REQUIRED = (u) => u === './' || u.endsWith('.js') || u.endsWith('.html');
self.addEventListener('install', (e) => {
  e.waitUntil((async () => {
    const rs = await Promise.allSettled(SHELL.map(async (u) => {
      const r = await fetch(u, { cache: 'no-store' });
      if (!r.ok) throw new Error(u + ' ' + r.status);
      return [u, r];
    }));
    const bad = SHELL.filter((u, i) => rs[i].status !== 'fulfilled' && REQUIRED(u));
    if (bad.length) throw new Error('неповне оновлення: ' + bad.join(', '));
    const c = await caches.open(SHELL_CACHE);
    for (const r of rs) if (r.status === 'fulfilled') await c.put(r.value[0], r.value[1]);
    await self.skipWaiting();
  })());
});

// Сторінка може попросити версію SW або повне очищення коду
self.addEventListener('message', (e) => {
  if (e.data === 'version') e.source?.postMessage({ swVersion: SHELL_CACHE });
  if (e.data === 'purge') e.waitUntil(caches.keys().then(ks => Promise.all(ks.filter(k => k !== DATA_CACHE).map(k => caches.delete(k)))).then(() => self.registration.unregister()));
});

self.addEventListener('activate', (e) => {
  e.waitUntil(caches.keys()
    .then(keys => Promise.all(keys.filter(k => !KEEP.includes(k)).map(k => caches.delete(k))))
    .then(() => self.clients.claim()));
});

self.addEventListener('fetch', (e) => {
  const req = e.request;
  if (req.method !== 'GET') return;
  const url = new URL(req.url);
  if (url.origin !== location.origin) return;
  // діагностика й запити з ?fresh= / ?nocache= — завжди з мережі, повз кеш
  if (url.searchParams.has('fresh') || url.searchParams.has('nocache') || url.pathname.endsWith('/diag.html')) return;

  e.respondWith((async () => {
    if (url.pathname.includes('/data/')) {
      const cache = await caches.open(DATA_CACHE);
      const hit = await cache.match(req, { ignoreSearch: true });
      if (hit) return hit;
      const res = await fetch(req);
      if (res.ok) cache.put(req, res.clone());
      return res;
    }
    // Код — лише з кешу своєї версії, без фонової підміни окремих файлів (саме вона давала
    // суміш версій). Нова версія приходить цілим набором через встановлення нового SW.
    const cache = await caches.open(SHELL_CACHE);
    const hit = await cache.match(req, { ignoreSearch: true });
    if (hit) return hit;
    try {
      return await fetch(req, { cache: 'no-store' });
    } catch (err) {
      return (await cache.match('./index.html')) || Response.error();
    }
  })());
});
