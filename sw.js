// Service worker: офлайн-робота. Усе, що один раз завантажилось, лишається на телефоні.
const VERSION = 'navi-v0.4.0';
const SHELL = ['./', './index.html', './manifest.webmanifest', './js/app.js', './js/engine.js', './js/motion.js', './js/filter.js', './js/graph.js', './js/astar.js', './js/geo.js', './js/testcity.js', './js/simulator.js', './js/traffic.js', './js/buildings.js', './js/voice.js', './icons/icon-180.png', './icons/icon-192.png', './icons/icon-512.png'];

self.addEventListener('install', (e) => {
  e.waitUntil(caches.open(VERSION).then(c => c.addAll(SHELL)).then(() => self.skipWaiting()));
});
self.addEventListener('activate', (e) => {
  e.waitUntil(caches.keys().then(keys => Promise.all(keys.filter(k => k !== VERSION).map(k => caches.delete(k)))).then(() => self.clients.claim()));
});
self.addEventListener('fetch', (e) => {
  if (e.request.method !== 'GET') return;
  e.respondWith(
    caches.match(e.request).then(hit => hit || fetch(e.request).then(res => {
      // дані карти (data/*) теж кешуємо після першого завантаження
      if (res.ok && new URL(e.request.url).origin === location.origin) {
        const copy = res.clone(); caches.open(VERSION).then(c => c.put(e.request, copy));
      }
      return res;
    }).catch(() => caches.match('./index.html')))
  );
});
