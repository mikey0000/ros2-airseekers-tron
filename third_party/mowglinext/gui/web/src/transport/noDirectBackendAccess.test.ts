import {describe, expect, it} from "vitest";

// Everything the GUI sends to the backend must go through apiFetch / openApiSocket so
// the active transport (Wi-Fi or Bluetooth) carries it. This scans the sources for
// the ways the seam used to be bypassed.
const sources = import.meta.glob<string>("/src/**/*.{ts,tsx}", {query: "?raw", import: "default", eager: true});

const EXCLUDED = [
    /\.test\.tsx?$/,
    /^\/src\/test\//,
    /^\/src\/transport\//,
    /^\/src\/api\/Api\.ts$/,
    /^\/src\/utils\/apiHost\.ts$/,
];

const FORBIDDEN: Array<[string, RegExp]> = [
    ["new WebSocket(", /\bnew WebSocket\(/],
    ["fetch(\"/api", /(?<![\w.])fetch\(\s*"\/api/],
    ["fetch('/api", /(?<![\w.])fetch\(\s*'\/api/],
    ["fetch(`/api", /(?<![\w.])fetch\(\s*`\/api/],
    ["fetch(`${httpBase()", /(?<![\w.])fetch\(\s*`\$\{httpBase\(\)/],
    ["fetch of /web-build.json", /(?<![\w.])fetch\(\s*["'`]\/web-build\.json/],
    // Any other global fetch: every caller in the app talks to the backend, so it goes
    // through apiFetch too. Comment lines are skipped below.
    ["fetch(", /(?<![\w.])fetch\(/],
    ["import of httpBase/wsBase", /import\s*(type\s*)?\{[^}]*\b(httpBase|wsBase)\b[^}]*\}\s*from/],
];

describe("transport seam", () => {
    it("scans the sources", () => {
        expect(Object.keys(sources).length).toBeGreaterThan(100);
    });

    it("nothing outside src/transport talks to the backend directly", () => {
        const hits: string[] = [];
        for (const [file, text] of Object.entries(sources)) {
            if (EXCLUDED.some((re) => re.test(file))) continue;
            text.split("\n").forEach((line, i) => {
                if (/^\s*(\/\/|\*|\/\*)/.test(line)) return;
                for (const [label, re] of FORBIDDEN) {
                    if (re.test(line)) hits.push(`${file}:${i + 1}: ${label} (use apiFetch / openApiSocket from src/transport)`);
                }
            });
        }
        expect(hits).toEqual([]);
    });
});
