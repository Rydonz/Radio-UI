/* Cache-first so the app opens in the car with no signal. Bump CACHE on release. */
const CACHE = "delsol-v6";
const ASSETS = [
  "./index.html",
  "./manifest.webmanifest",
  "./App-Icon.png",
  "./icon-192.png",
  "./icon-maskable.png",
  "./fonts/barlow-400.woff2",
  "./fonts/barlow-500.woff2",
  "./fonts/barlow-600.woff2",
  "./fonts/barlow-700.woff2",
  "./fonts/dm-mono-400.woff2",
  "./fonts/dm-mono-500.woff2",
  "./fonts/share-tech-mono-400.woff2",
];

self.addEventListener("install", e => {
  e.waitUntil(caches.open(CACHE).then(c => c.addAll(ASSETS)).then(() => self.skipWaiting()));
});

self.addEventListener("activate", e => {
  e.waitUntil(
    caches.keys()
      .then(keys => Promise.all(keys.filter(k => k !== CACHE).map(k => caches.delete(k))))
      .then(() => self.clients.claim())
  );
});

self.addEventListener("fetch", e => {
  if (e.request.method !== "GET") return;

  // the update manifest must be fresh — network-first, cache only as offline fallback
  if (new URL(e.request.url).pathname.endsWith("/firmware.json")) {
    e.respondWith(fetch(e.request).catch(() => caches.match(e.request)));
    return;
  }

  e.respondWith(
    caches.match(e.request).then(hit => hit || fetch(e.request).then(res => {
      // keep the cache warm for same-origin assets, ignore opaque/cross-origin
      if (res.ok && new URL(e.request.url).origin === location.origin) {
        const copy = res.clone();
        caches.open(CACHE).then(c => c.put(e.request, copy));
      }
      return res;
    }).catch(() => caches.match("./index.html")))
  );
});
