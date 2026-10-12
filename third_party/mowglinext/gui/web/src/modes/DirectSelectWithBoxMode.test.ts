import {describe, expect, it, vi} from 'vitest';
import DirectSelectWithBoxMode, {type LngLatPair, ROTATION_HANDLE_GAP_PX} from './DirectSelectWithBoxMode';

type Px = {x: number; y: number};
type Rect = {left: number; top: number; right: number; bottom: number};
const {ringCentroid, rotateRings, rotationHandlePixel, isPointInHandle} = DirectSelectWithBoxMode.rotationMath as {
    ringCentroid: (ring: LngLatPair[]) => LngLatPair;
    rotateRings: (rings: LngLatPair[][], center: LngLatPair, angleRad: number) => LngLatPair[][];
    rotationHandlePixel: (c: Px, v: Px[], angleRad?: number, gapPx?: number) => Px;
    isPointInHandle: (p: Px, handle: Rect, container: Rect, slopPx?: number) => boolean;
};

const LAT = 48.8;
const K = Math.cos(LAT * Math.PI / 180);

/** Square 10 m x 10 m-ish around (2, LAT), closed ring. */
const square = (): LngLatPair[] => {
    const d = 0.0001;
    return [
        [2 - d / K, LAT - d], [2 + d / K, LAT - d], [2 + d / K, LAT + d], [2 - d / K, LAT + d], [2 - d / K, LAT - d],
    ];
};

/** Local-metric (equirectangular) distance, enough to compare shapes. */
const dist = (a: LngLatPair, b: LngLatPair) => Math.hypot((a[0] - b[0]) * K, a[1] - b[1]);

describe('ringCentroid', () => {
    it('is the centre of a square (closed or open ring)', () => {
        const c = ringCentroid(square());
        expect(c[0]).toBeCloseTo(2, 10);
        expect(c[1]).toBeCloseTo(LAT, 10);
        const o = ringCentroid(square().slice(0, -1));
        expect(o[0]).toBeCloseTo(2, 10);
        expect(o[1]).toBeCloseTo(LAT, 10);
    });
    it('is not biased by the duplicated closing vertex or vertex density', () => {
        // Square with many extra vertices on one edge: the area centroid stays put.
        const d = 0.0001;
        const ring: LngLatPair[] = [[0, 0], [d, 0], [d, d * 0.25], [d, d * 0.5], [d, d * 0.75], [d, d], [0, d], [0, 0]];
        const c = ringCentroid(ring);
        expect(c[0]).toBeCloseTo(d / 2, 9);
        expect(c[1]).toBeCloseTo(d / 2, 9);
    });
});

describe('rotateRings', () => {
    it('keeps vertex count and ring closure', () => {
        const ring = square();
        const [out] = rotateRings([ring], ringCentroid(ring), 0.7);
        expect(out).toHaveLength(ring.length);
        expect(out[out.length - 1]).toEqual(out[0]);
    });
    it('rotates about the centroid: distances to it are preserved and it is a fixed point', () => {
        const ring = square();
        const c = ringCentroid(ring);
        const [out] = rotateRings([ring], c, 1.1);
        for (let i = 0; i < ring.length; i++) {
            expect(dist(out[i], c)).toBeCloseTo(dist(ring[i], c), 12);
        }
        const c2 = ringCentroid(out);
        expect(c2[0]).toBeCloseTo(c[0], 10);
        expect(c2[1]).toBeCloseTo(c[1], 10);
    });
    it('preserves the ground shape (edge lengths) at high latitude', () => {
        const ring = square();
        const [out] = rotateRings([ring], ringCentroid(ring), Math.PI / 5);
        for (let i = 0; i < ring.length - 1; i++) {
            expect(dist(out[i], out[i + 1])).toBeCloseTo(dist(ring[i], ring[i + 1]), 12);
        }
    });
    it('a quarter turn CCW maps east to north', () => {
        const c: LngLatPair = [2, LAT];
        const east: LngLatPair = [2 + 0.001 / K, LAT];
        const [[p]] = rotateRings([[east]], c, Math.PI / 2);
        expect(p[0]).toBeCloseTo(2, 10);
        expect(p[1]).toBeCloseTo(LAT + 0.001, 10);
    });
});

describe('rotationHandlePixel', () => {
    it('sits straight above the centroid, gap px beyond the farthest vertex', () => {
        const c = {x: 400, y: 300};
        const verts = [{x: 350, y: 250}, {x: 450, y: 250}, {x: 450, y: 350}, {x: 350, y: 350}];
        const p = rotationHandlePixel(c, verts);
        const r = Math.hypot(50, 50) + ROTATION_HANDLE_GAP_PX;
        expect(p.x).toBeCloseTo(400, 9);
        expect(p.y).toBeCloseTo(300 - r, 9);
    });
    it('is the same screen distance from the shape at any scale (zoom)', () => {
        const c = {x: 0, y: 0};
        const small = rotationHandlePixel(c, [{x: 0, y: -10}]);
        const big = rotationHandlePixel(c, [{x: 0, y: -1000}]);
        expect(-10 - small.y).toBeCloseTo(ROTATION_HANDLE_GAP_PX, 9);
        expect(-1000 - big.y).toBeCloseTo(ROTATION_HANDLE_GAP_PX, 9);
    });
    it('follows a given screen angle', () => {
        const p = rotationHandlePixel({x: 0, y: 0}, [{x: 10, y: 0}], 0, 5);
        expect(p.x).toBeCloseTo(15, 9);
        expect(p.y).toBeCloseTo(0, 9);
    });
});

describe('isPointInHandle', () => {
    // Map container at (88, 75) in the viewport; handle drawn at container (750, 23).
    const container = {left: 88, top: 75, right: 1440, bottom: 900};
    const handle = {left: 88 + 750, top: 75 + 23, right: 88 + 750 + 28, bottom: 75 + 23 + 28};

    it('hits for a container point inside the handle', () => {
        expect(isPointInHandle({x: 764, y: 37}, handle, container)).toBe(true);
    });
    it('does not hit at the old, offset (viewport) coordinates', () => {
        // The pre-fix test compared container px to viewport px: a click on the
        // handle at container (764, 37) was "missed", and a click 88/75 px
        // further down-right "hit".
        expect(isPointInHandle({x: 764 + 88, y: 37 + 75}, handle, container)).toBe(false);
    });
    it('allows a few px of slop around the edge', () => {
        expect(isPointInHandle({x: 747, y: 37}, handle, container)).toBe(true);
        expect(isPointInHandle({x: 740, y: 37}, handle, container)).toBe(false);
    });
});

describe('rotation gesture on a mapbox-gl-draw polygon', () => {
    /**
     * Minimal stand-ins: a Polygon that, like mapbox-gl-draw's, stores OPEN
     * rings (getCoordinates appends the closing vertex, setCoordinates does
     * not strip it) and a map with a linear projection.
     */
    function setup() {
        let rings = [square().slice(0, -1)];
        const feature = {
            type: 'Polygon',
            getCoordinates: () => rings.map(r => [...r, r[0]]),
            setCoordinates: (c: LngLatPair[][]) => { rings = c; },
            toGeoJSON: () => ({type: 'Feature', geometry: {type: 'Polygon', coordinates: feature.getCoordinates()}}),
        };
        const scale = 1e6;
        const container = document.createElement('div');
        const fire = vi.fn();
        const map = {
            project: ([lng, lat]: LngLatPair) => ({x: (lng - 2) * K * scale + 500, y: -(lat - LAT) * scale + 400}),
            getContainer: () => container,
            dragPan: {enable: vi.fn(), disable: vi.fn()},
            fire,
            on: vi.fn(),
            off: vi.fn(),
        };
        const mode = Object.assign(Object.create(DirectSelectWithBoxMode), {
            map,
            _ctx: {store: {render: vi.fn()}},
            getSelected: () => [feature],
        });
        const state: Record<string, unknown> = {feature, selectedCoordPaths: []};
        return {mode, state, fire, map, rings: () => rings, feature};
    }

    it('rotates without growing the ring and fires draw.update once at the end', () => {
        const {mode, state, fire, rings, feature} = setup();
        const before = feature.getCoordinates()[0];
        mode.renderRotationHandle(state);
        const el = state.rotationHandleElement as HTMLElement;
        expect(el.id).toBe('rotation-handle');
        expect(el.textContent).toBe('↻');
        expect(el.style.top).toBe('0px');
        expect(el.style.left).toBe('0px');
        expect(parseInt(el.style.width)).toBeGreaterThanOrEqual(24);
        expect(el.title).toBeTruthy();

        // Drag from straight above the centre (500, 400) round to its right.
        mode.startRotation(state, {x: 500, y: 300});
        mode.updateRotation(state, {point: {x: 560, y: 320}});
        mode.updateRotation(state, {point: {x: 590, y: 380}});
        mode.updateRotation(state, {point: {x: 600, y: 400}});
        expect(fire).not.toHaveBeenCalled();
        mode.stopRotation(state);
        expect(fire).toHaveBeenCalledTimes(1);
        expect(fire.mock.calls[0][0]).toBe('draw.update');

        expect(rings()[0]).toHaveLength(4);
        const after = feature.getCoordinates()[0];
        expect(after).toHaveLength(5);
        expect(after[4]).toEqual(after[0]);
        // A quarter turn clockwise on screen: the SW corner goes to NW.
        expect(after[0][0]).toBeCloseTo(before[3][0], 9);
        expect(after[0][1]).toBeCloseTo(before[3][1], 9);
        expect(state.rotating).toBe(false);
    });

    it('Escape-style cancel restores the original shape and fires nothing', () => {
        const {mode, state, fire, feature} = setup();
        const before = feature.getCoordinates();
        mode.startRotation(state, {x: 500, y: 300});
        mode.updateRotation(state, {point: {x: 600, y: 400}});
        mode.stopRotation(state, false);
        expect(fire).not.toHaveBeenCalled();
        expect(feature.getCoordinates()).toEqual(before);
    });

    it('onStop removes the handle and the move listener', () => {
        const {mode, state, map} = setup();
        mode.renderRotationHandle(state);
        state.onMapMove = () => {};
        mode.clearSelectedCoordinates = vi.fn();
        mode.onStop(state);
        expect(state.rotationHandleElement).toBeUndefined();
        expect(map.getContainer().querySelector('#rotation-handle')).toBeNull();
        expect(map.off).toHaveBeenCalledWith('move', expect.any(Function));
    });
});
