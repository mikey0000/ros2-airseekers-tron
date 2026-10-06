import {describe, expect, it} from "vitest";
import {
    clampOpacity,
    drawableOverlays,
    imageCornersMap,
    imageToMap,
    imageryTileUrl,
    mapToImage,
    moveOverlay,
    normalizeDeg,
    placementFromCornerDrag,
    quadToLngLat,
    solveTwoPointPlacement,
    validateUpload,
    type ImageryOverlay,
    type ImageryPlacement,
} from "./imagery.ts";
import {itranspose} from "./map.tsx";

const W = 4000, H = 3000;
const P: ImageryPlacement = {centerX: 3, centerY: -7, metersPerPixel: 0.025, rotationDeg: 33};

const overlay = (o: Partial<ImageryOverlay>): ImageryOverlay => ({
    name: "a", label: "a", kind: "geotiff", status: "ready", progress: 1, opacity: 1, visible: true, sizeBytes: 1, createdAt: 1, ...o,
});

describe("imagery georeference maths", () => {
    it("imageToMap / mapToImage are inverse", () => {
        for (const [u, v] of [[0, 0], [1, 1], [0.25, 0.8], [0.5, 0.5]]) {
            const [x, y] = imageToMap(P, W, H, u, v);
            const [u2, v2] = mapToImage(P, W, H, x, y);
            expect(u2).toBeCloseTo(u, 9);
            expect(v2).toBeCloseTo(v, 9);
        }
        expect(imageToMap(P, W, H, 0.5, 0.5)).toEqual([3, -7]);
    });

    it("unrotated image: top edge is north, width = px * m/px", () => {
        const p = {...P, rotationDeg: 0};
        const [tl, tr, br, bl] = imageCornersMap(p, W, H);
        expect(tr[0] - tl[0]).toBeCloseTo(W * 0.025);
        expect(tl[1]).toBeGreaterThan(bl[1]);
        expect(br[1]).toBeCloseTo(bl[1]);
    });

    it("two control points recover the exact placement", () => {
        const a = {u: 0.1, v: 0.2, x: 0, y: 0};
        const b = {u: 0.9, v: 0.7, x: 0, y: 0};
        [a.x, a.y] = imageToMap(P, W, H, a.u, a.v);
        [b.x, b.y] = imageToMap(P, W, H, b.u, b.v);
        const s = solveTwoPointPlacement(a, b, W, H)!;
        expect(s.centerX).toBeCloseTo(P.centerX, 9);
        expect(s.centerY).toBeCloseTo(P.centerY, 9);
        expect(s.metersPerPixel).toBeCloseTo(P.metersPerPixel, 12);
        expect(s.rotationDeg).toBeCloseTo(P.rotationDeg, 9);
        // And maps the control points onto their targets.
        expect(imageToMap(s, W, H, a.u, a.v)[0]).toBeCloseTo(a.x, 9);
        expect(imageToMap(s, W, H, b.u, b.v)[1]).toBeCloseTo(b.y, 9);
    });

    it("rejects degenerate control points", () => {
        expect(solveTwoPointPlacement({u: 0.5, v: 0.5, x: 0, y: 0}, {u: 0.5, v: 0.5, x: 5, y: 5}, W, H)).toBeNull();
        expect(solveTwoPointPlacement({u: 0.1, v: 0.5, x: 1, y: 1}, {u: 0.9, v: 0.5, x: 1, y: 1}, W, H)).toBeNull();
    });

    it("corner drag sets scale and rotation, keeps the centre", () => {
        const p0 = {...P, rotationDeg: 0};
        const target = imageToMap(P, W, H, 1, 0); // where TR is under P
        const p = placementFromCornerDrag(p0, W, H, target);
        expect(p.centerX).toBe(p0.centerX);
        expect(p.metersPerPixel).toBeCloseTo(P.metersPerPixel, 12);
        expect(p.rotationDeg).toBeCloseTo(P.rotationDeg, 9);
    });

    it("normalizeDeg wraps into (-180, 180]", () => {
        expect(normalizeDeg(190)).toBeCloseTo(-170);
        expect(normalizeDeg(-180)).toBe(180);
        expect(normalizeDeg(720 + 5)).toBeCloseTo(5);
    });

    it("map-frame quad goes through the same projection as the areas (offset applied)", () => {
        const datum: [number, number, number] = [50.1, 8.6, 0];
        const q = quadToLngLat(imageCornersMap(P, W, H), 0.4, -0.2, datum);
        const back = itranspose(0.4, -0.2, datum, q[0][1], q[0][0]);
        const tl = imageToMap(P, W, H, 0, 0);
        expect(back[0]).toBeCloseTo(tl[0], 6);
        expect(back[1]).toBeCloseTo(tl[1], 6);
    });
});

describe("imagery overlay list helpers", () => {
    it("moveOverlay swaps neighbours and clamps at the ends", () => {
        expect(moveOverlay(["a", "b", "c"], "a", "up")).toEqual(["b", "a", "c"]);
        expect(moveOverlay(["a", "b", "c"], "c", "up")).toEqual(["a", "b", "c"]);
        expect(moveOverlay(["a", "b", "c"], "b", "down")).toEqual(["b", "a", "c"]);
        expect(moveOverlay(["a", "b", "c"], "a", "down")).toEqual(["a", "b", "c"]);
        expect(moveOverlay(["a"], "zz", "up")).toEqual(["a"]);
    });

    it("drawableOverlays keeps ready+visible, and placed plain images only", () => {
        const list = [
            overlay({name: "t"}),
            overlay({name: "hidden", visible: false}),
            overlay({name: "busy", status: "processing"}),
            overlay({name: "img-unplaced", kind: "image", width: 10, height: 10}),
            overlay({name: "img", kind: "image", width: 10, height: 10, placement: P}),
        ];
        expect(drawableOverlays(list).map((o) => o.name)).toEqual(["t", "img"]);
    });

    it("clampOpacity", () => {
        expect(clampOpacity(1.4)).toBe(1);
        expect(clampOpacity(-1)).toBe(0);
        expect(clampOpacity(Number.NaN)).toBe(1);
        expect(clampOpacity(0.35)).toBe(0.35);
    });

    it("tile URL template carries the cache-busting version and extension", () => {
        expect(imageryTileUrl({name: "drone june", createdAt: 42, tileExt: ".jpg"}))
            .toBe("/api/mowglinext/tiles/drone%20june/{z}/{x}/{y}.jpg?v=42");
        expect(imageryTileUrl({name: "a", createdAt: 1})).toContain("{y}.png");
    });

    it("validateUpload checks type and the 300 MB limit", () => {
        expect(validateUpload({name: "o.TIF", size: 10})).toBeNull();
        expect(validateUpload({name: "o.exe", size: 10})).toBe("type");
        expect(validateUpload({name: "o.tif", size: 301 * 1024 * 1024})).toBe("size");
    });
});
