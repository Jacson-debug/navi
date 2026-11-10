// Service worker: офлайн-робота + автоматичне оновлення коду.
//
// Два окремі сховища, і це важливо:
//   SHELL — код (html/js/іконки). Оновлюється самостійно: сторінка відкривається
//           з кешу миттєво, а свіжа версія довантажується у фоні й застосовується
//           з наступного запуску. Версію можна піднімати скільки завгодно.
//   DATA  — карта, будинки, АЗС, назви місцевостей: десятки мегабайтів.
//           Лежить окремо, щоб оновлення коду НЕ змушувало телефон качати їх знову
//           через мобільний інтернет.
const SHELL_CACHE = 'navi-shell-v0.8.5';
const DATA_CACHE = 'navi-data-v1';
const KEEP = [SHELL_CACHE, DATA_CACHE];
const SHELL = ['./', './index.html', './manifest.webmanifest', './keep.mp4',
  './js/app.js', './js/engine.js', './js/motion.js', './js/filter.js', './js/graph.js',
  './js/astar.js', './js/geo.js', './js/traffic.js', './js/buildings.js', './js/voice.js',
  './icons/icon-180.png', './icons/icon-192.png', './icons/icon-512.png'];

self.addEventListener('install', (e) => {
  e.waitUntil(caches.open(SHELL_CACHE).then(c => c.addAll(SHELL)).then(() => self.skipWaiting()));
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
