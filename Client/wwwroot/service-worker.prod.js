const version = '1.0.2';
const cacheBuster = '?v=' + version;

const cacheName = 'maglevsim-cache-v' + version;
const assetsToCache = [
    './' + cacheBuster,  // Alias for 'index.html'
    'css/bootstrap/bootstrap.min.css' + cacheBuster,
    'css/app.css' + cacheBuster,
    'favicon.png' + cacheBuster,
    'Animations.Client.styles.css' + cacheBuster,
    'manifest.json' + cacheBuster,
    'js/bundle.js' + cacheBuster,
    '_framework/blazor.webassembly.js' + cacheBuster,
    '_framework/blazor.boot.json' + cacheBuster,
    '_framework/dotnet.wasm' + cacheBuster,
    '_framework/dotnet.7.0.11.sf8asx8yhf.js' + cacheBuster,
];

self.addEventListener('install', (event) => {
    // Pre-cache all required assets
    event.waitUntil(
        caches.open(cacheName)
            .then((cache) => {
                return cache.addAll(assetsToCache);
            })
    );
});

self.addEventListener('activate', (event) => {
    // Clean up old caches
    event.waitUntil(
        caches.keys().then((cacheNames) => {
            return Promise.all(
                cacheNames.filter((name) => name !== cacheName)
                    .map((name) => caches.delete(name))
            );
        })
    );
});

self.addEventListener('fetch', (event) => {
    // Serve cached content when offline
    event.respondWith(
        caches.match(event.request).then((response) => {
            return response || fetch(event.request);
        })
    );
});