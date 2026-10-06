import {afterEach, describe, expect, it, vi} from "vitest";
import {act, renderHook, waitFor} from "@testing-library/react";
import {useImagery} from "./useImagery.ts";
import {applyCpEdit, type AlignSession} from "../pages/map/hooks/useImageryAlign.ts";
import {imageToMap} from "../utils/imagery.ts";

const ov = (name: string, extra: object = {}) => ({
    name, label: name, kind: "geotiff", status: "ready", progress: 1, opacity: 1, visible: true, sizeBytes: 1, createdAt: 1, ...extra,
});

function mockFetch(handler: (url: string, init?: RequestInit) => unknown) {
    const fn = vi.fn((url: string, init?: RequestInit) =>
        Promise.resolve(new Response(JSON.stringify(handler(url, init)), {status: 200})));
    vi.stubGlobal("fetch", fn);
    return fn;
}

afterEach(() => { vi.unstubAllGlobals(); });

describe("useImagery", () => {
    it("loads the list, patches opacity optimistically, reorders and deletes", async () => {
        const fetchMock = mockFetch((url, init) => {
            if (url.endsWith("/imagery") && !init?.method) return {overlays: [ov("a"), ov("b")]};
            if (init?.method === "PATCH") return {...ov("a"), ...JSON.parse(init.body as string)};
            if (url.endsWith("/imagery-order")) return {overlays: [ov("b"), ov("a")]};
            if (init?.method === "DELETE") return {};
            return {};
        });
        const {result} = renderHook(() => useImagery());
        await waitFor(() => expect(result.current.overlays).toHaveLength(2));

        await act(async () => { await result.current.update("a", {opacity: 0.4, visible: false}); });
        expect(result.current.overlays[0]).toMatchObject({name: "a", opacity: 0.4, visible: false});
        const patch = fetchMock.mock.calls.find(([, i]) => i?.method === "PATCH")!;
        expect(patch[0]).toContain("/api/mowglinext/imagery/a");
        expect(JSON.parse(patch[1]!.body as string)).toEqual({opacity: 0.4, visible: false});

        await act(async () => { await result.current.reorder(["b", "a"]); });
        expect(result.current.overlays.map((o) => o.name)).toEqual(["b", "a"]);

        await act(async () => { await result.current.remove("b"); });
        expect(result.current.overlays.map((o) => o.name)).toEqual(["a"]);
    });
});

describe("alignment control points", () => {
    it("solves the placement once both points have image + ground positions", () => {
        const truth = {centerX: 10, centerY: 5, metersPerPixel: 0.02, rotationDeg: -20};
        let s: AlignSession = {
            name: "x", width: 2000, height: 1000, opacity: 0.7,
            placement: {centerX: 0, centerY: 0, metersPerPixel: 0.03, rotationDeg: 0},
            cps: [{}, {}],
        };
        const a = imageToMap(truth, 2000, 1000, 0.2, 0.3);
        const b = imageToMap(truth, 2000, 1000, 0.85, 0.6);
        s = applyCpEdit(s, 0, {u: 0.2, v: 0.3});
        s = applyCpEdit(s, 0, {x: a[0], y: a[1]});
        s = applyCpEdit(s, 1, {u: 0.85, v: 0.6});
        expect(s.placement.metersPerPixel).toBe(0.03); // not solved yet
        s = applyCpEdit(s, 1, {x: b[0], y: b[1]});
        expect(s.placement.centerX).toBeCloseTo(10, 9);
        expect(s.placement.centerY).toBeCloseTo(5, 9);
        expect(s.placement.metersPerPixel).toBeCloseTo(0.02, 12);
        expect(s.placement.rotationDeg).toBeCloseTo(-20, 9);
    });
});
