// Service worker: офлайн-робота + автоматичне оновлення коду.
//  - код (html/js/manifest/іконки): «з кешу, але одразу довантажуємо свіже» —
//    сторінка відкривається миттєво й офлайн, а наступний запуск бачить оновлення
//    без ручного скидання кешу;
//  - дані (data/*): лише з кешу після першого завантаження — це десятки мегабайт,
//    їх не можна перекачувати щоразу через мобільний інтернет.
const VERSION = 'navi-v0.6.0';
const SHELL = ['./', './index.html', './manifest.webmanifest',
  './js/app.js', './js/engine.js', './js/motion.js', './js/filter.js', './js/graph.js',
  './js/astar.js', './js/geo.js', './js/traffic.js', './js/buildings.js', './js/voice.js',
  './js/testcity.js', './js/simulator.js',
  './icons/icon-180.png', './icons/icon-192.png', './icons/icon-512.png'];

self.addEventListener('install', (e) => {
  e.waitUntil(caches.open(VERSION).then(c => c.addAll(SHELL)).then(() => self.skipWaiting()));
});
self.addEventListener('activate', (e) => {
  e.waitUntil(caches.keys()
    .then(keys => Promise.all(keys.filter(k => k !== VERSION).map(k => caches.delete(k))))
    .then(() => self.clients.claim()));
});
self.addEventListener('fetch', (e) => {
  const req = e.request;
  if (req.method !== 'GET') return;
  const url = new URL(req.url);
  if (url.origin !== location.origin) return;
  const isData = url.pathname.includes('/data/');

  e.respondWith((async () => {
    const cache = await caches.open(VERSION);
    const hit = await cache.match(req, { ignoreSearch: true });
    if (isData) {
      if (hit) return hit;
      const res = await fetch(req);
      if (res.ok) cache.put(req, res.clone());
      return res;
    }
    // код: віддаємо кеш одразу, паралельно оновлюємо його з мережі
    const fresh = fetch(req).then(res => {
      if (res.ok) cache.put(req, res.clone());
      return res;
    }).catch(() => null);
    if (hit) { e.waitUntil(fresh); return hit; }
    const res = await fresh;
    return res || cache.match('./index.html');
  })());
});
