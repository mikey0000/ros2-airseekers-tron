import {defineConfig} from 'vite'
import react from '@vitejs/plugin-react'
import {VitePWA} from 'vite-plugin-pwa'
import {randomUUID} from 'node:crypto'

const API_TARGET = process.env.MOWGLI_API_TARGET || 'http://localhost:4006';
const targetUrl = new URL(API_TARGET);
const targetOrigin = `${targetUrl.protocol}//${targetUrl.host}`;

// The web build has its own identity: a web-only patch need not rebuild Go.
// Embed the same ID in the client and its served manifest, including rebuilds
// from the same source revision with different assets/configuration.
const webBuild = {id: randomUUID(), revision: process.env.VITE_BUILD_REVISION || '',
    version: process.env.VITE_BUILD_VERSION || '', built_at: process.env.VITE_BUILD_TIME || ''};

// https://vitejs.dev/config/
export default defineConfig({
    define: {'import.meta.env.VITE_WEB_BUILD_ID': JSON.stringify(webBuild.id)},
    plugins: [
        react(),
        {
            name: 'served-web-build',
            generateBundle() {
                this.emitFile({type: 'asset', fileName: 'web-build.json', source: JSON.stringify(webBuild)});
            },
            configureServer(server) {
                server.middlewares.use('/web-build.json', (_request, response) => {
                    response.setHeader('Content-Type', 'application/json');
                    response.setHeader('Cache-Control', 'no-store');
                    response.end(JSON.stringify(webBuild));
                });
            },
        },
        // Offline shell (docs/DESIGN_ble_transport_2026-10.md section 4): the service
        // worker precaches the built app so it opens without Wi-Fi and can offer
        // Bluetooth. Backend responses are never cached. `injectRegister: 'auto'` adds
        // registerSW.js to index.html, so main.tsx needs no import.
        VitePWA({
            registerType: 'autoUpdate',
            injectRegister: 'auto',
            // public/manifest.json stays the web app manifest.
            manifest: false,
            devOptions: {enabled: false},
            workbox: {
                globPatterns: ['**/*.{js,css,html,svg,woff2,png,json}'],
                // web-build.json drives update detection (useInstalledVersions) and
                // must always come from the server.
                globIgnores: ['web-build.json', '**/*.gz'],
                // The largest chunk (mapbox-gl) is ~1.7 MB, close to the 2 MiB default;
                // a chunk over the limit would silently be left out of the shell.
                maximumFileSizeToCacheInBytes: 4 * 1024 * 1024,
                // Hash router: every navigation is to "/", serve the cached shell.
                navigateFallback: '/index.html',
                navigateFallbackDenylist: [/^\/api\//, /^\/swagger/, /^\/tiles/, /^\/web-build\.json$/],
                cleanupOutdatedCaches: true,
                // Third-party stylesheets and fonts linked from index.html, so the
                // offline shell keeps its typography and the map CSS. Opaque
                // (no-cors) responses are cached as-is.
                runtimeCaching: [{
                    urlPattern: ({url}) => ['fonts.googleapis.com', 'fonts.gstatic.com', 'api.fontshare.com',
                        'cdn.fontshare.com'].includes(url.hostname)
                        || (url.hostname === 'api.mapbox.com' && url.pathname.endsWith('.css')),
                    handler: 'StaleWhileRevalidate',
                    options: {
                        cacheName: 'shell-third-party',
                        cacheableResponse: {statuses: [0, 200]},
                        expiration: {maxEntries: 60},
                    },
                }],
            },
        }),
    ],
    server: {
        host: '0.0.0.0',
        proxy: {
            '/api': {
                target: API_TARGET,
                ws: true,
                changeOrigin: true,
                // Robot's Go backend gates WebSocket upgrades on the Origin
                // header. Rewrite it so cross-origin dev (frontend on :5173,
                // backend on the robot) passes the check.
                headers: {
                    Origin: targetOrigin,
                },
                configure: (proxy) => {
                    proxy.on('proxyReqWs', (proxyReq: { setHeader?: (k: string, v: string) => void; }) => {
                        proxyReq.setHeader?.('Origin', targetOrigin);
                        proxyReq.setHeader?.('Host', targetUrl.host);
                    });
                },
            }
        }
    }
})
