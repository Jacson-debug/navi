// Service worker: офлайн-робота + автоматичне оновлення коду.
//
// Два окремі сховища, і це важливо:
//   SHELL — код (html/js/іконки). Оновлюється самостійно: сторінка відкривається
//           з кешу миттєво, а свіжа версія довантажується у фоні й застосовується
//           з наступного запуску. Версію можна піднімати скільки завгодно.
//   DATA  — карта, будинки, АЗС, назви місцевостей: десятки мегабайтів.
//           Лежить окремо, щоб оновлення коду НЕ змушувало телефон качати їх знову
//           через мобільний інтернет.
const SHELL_CACHE = 'navi-shell-v0.8.16';
const DATA_CACHE = 'navi-data-v1';
const KEEP = [SHELL_CACHE, DATA_CACHE];
const SHELL = ['./', './index.html', './manifest.webmanifest', './keep.mp4',
  './js/app.js', './js/engine.js', './js/motion.js', './js/filter.js', './js/graph.js',
  './js/astar.js', './js/geo.js', './js/traffic.js', './js/buildings.js', './js/voice.js',
  './icons/icon-180.png', './icons/icon-192.png', './icons/icon-512.png'];

// Кожен файл окремо: якщо один не завантажився, решта все одно в кеші й нова
// версія встановлюється. Раніше addAll падав цілком через будь-який 404 — і телефон
// міг назавжди лишитися на старій версії.
self.addEventListener('install', (e) => {
  e.waitUntil(caches.open(SHELL_CACHE)
    .then(c => Promise.allSettled(SHELL.map(u => fetch(u, { cache: 'no-store' }).then(r => { if (r.ok) return c.put(u, r); }))))
    .then(() => self.skipWaiting()));
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

  e.respondWith((async () => {
    if (url.pathname.includes('/data/')) {
      const cache = await caches.open(DATA_CACHE);
      const hit = await cache.match(req, { ignoreSearch: true });
      if (hit) return hit;
      const res = await fetch(req);
      if (res.ok) cache.put(req, res.clone());
      return res;
    }
    const cache = await caches.open(SHELL_CACHE);
    const hit = await cache.match(req, { ignoreSearch: true });
    const fresh = fetch(req).then(res => { if (res.ok) cache.put(req, res.clone()); return res; }).catch(() => null);
    if (hit) { e.waitUntil(fresh); return hit; }
    const res = await fresh;
    return res || cache.match('./index.html');
  })());
});
