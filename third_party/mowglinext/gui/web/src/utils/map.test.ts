import {describe, it, expect} from 'vitest';
import {dedupePoints, getQuaternionFromHeading, transpose, itranspose, pointInPolygon, LINE_SOURCE_MAXZOOM} from './map.tsx';

describe('dedupePoints', () => {
    it('returns empty array for empty input', () => {
        expect(dedupePoints([])).toEqual([]);
    });

    it('returns single point unchanged', () => {
        const points = [{x: 1, y: 2, z: 0}];
        expect(dedupePoints(points)).toEqual(points);
    });

    it('keeps distinct points', () => {
        const points = [
            {x: 0, y: 0, z: 0},
            {x: 1, y: 0, z: 0},
            {x: 1, y: 1, z: 0},
        ];
        expect(dedupePoints(points)).toEqual(points);
    });

    it('removes duplicate consecutive points within epsilon', () => {
        const points = [
            {x: 0, y: 0, z: 0},
            {x: 0.0001, y: 0.0001, z: 0},
            {x: 1, y: 1, z: 0},
        ];
        const result = dedupePoints(points);
        expect(result).toHaveLength(2);
        expect(result[0]).toEqual({x: 0, y: 0, z: 0});
        expect(result[1]).toEqual({x: 1, y: 1, z: 0});
    });

    it('keeps points farther apart than epsilon', () => {
        const points = [
            {x: 0, y: 0, z: 0},
            {x: 0.002, y: 0, z: 0},
            {x: 0.004, y: 0, z: 0},
        ];
        const result = dedupePoints(points, 0.001);
        expect(result).toHaveLength(3);
    });

    it('uses custom epsilon', () => {
        const points = [
            {x: 0, y: 0, z: 0},
            {x: 0.5, y: 0, z: 0},
            {x: 1.0, y: 0, z: 0},
        ];
        const result = dedupePoints(points, 0.6);
        expect(result).toHaveLength(2);
        expect(result[0]).toEqual({x: 0, y: 0, z: 0});
        expect(result[1]).toEqual({x: 1.0, y: 0, z: 0});
    });

    it('handles multiple consecutive duplicates', () => {
        const points = [
            {x: 0, y: 0, z: 0},
            {x: 0.0001, y: 0, z: 0},
            {x: 0.0002, y: 0, z: 0},
            {x: 0.0003, y: 0, z: 0},
            {x: 5, y: 5, z: 0},
        ];
        const result = dedupePoints(points);
        expect(result).toHaveLength(2);
    });

    it('preserves z values', () => {
        const points = [
            {x: 0, y: 0, z: 10},
            {x: 1, y: 1, z: 20},
        ];
        const result = dedupePoints(points);
        expect(result[0].z).toBe(10);
        expect(result[1].z).toBe(20);
    });
});

describe('getQuaternionFromHeading', () => {
    it('returns identity quaternion for heading 0', () => {
        const q = getQuaternionFromHeading(0);
        expect(q.w).toBeCloseTo(1);
        expect(q.x).toBe(0);
        expect(q.y).toBe(0);
        expect(q.z).toBeCloseTo(0);
    });

    it('returns correct quaternion for PI/2', () => {
        const q = getQuaternionFromHeading(Math.PI / 2);
        expect(q.w).toBeCloseTo(Math.cos(Math.PI / 4));
        expect(q.z).toBeCloseTo(Math.sin(Math.PI / 4));
    });

    it('returns correct quaternion for PI', () => {
        const q = getQuaternionFromHeading(Math.PI);
        expect(q.w).toBeCloseTo(0, 5);
        expect(q.z).toBeCloseTo(1);
    });

    it('handles negative heading', () => {
        const q = getQuaternionFromHeading(-Math.PI / 2);
        expect(q.w).toBeCloseTo(Math.cos(-Math.PI / 4));
        expect(q.z).toBeCloseTo(Math.sin(-Math.PI / 4));
    });
});

describe('transpose / itranspose', () => {
    const datum: [number, number, number] = [500000, 5500000, 32];

    it('transpose converts UTM to lat/lon', () => {
        const [lon, lat] = transpose(0, 0, datum, 0, 0);
        expect(typeof lon).toBe('number');
        expect(typeof lat).toBe('number');
        expect(lon).not.toBeNaN();
        expect(lat).not.toBeNaN();
    });

    it('itranspose inverts transpose', () => {
        const [lon, lat] = transpose(0, 0, datum, 10, 20);
        const [x, y] = itranspose(0, 0, datum, lat, lon);
        expect(x).toBeCloseTo(20, 0);
        expect(y).toBeCloseTo(10, 0);
    });

    it('applies offset correctly', () => {
        const [lon1] = transpose(0, 0, datum, 0, 0);
        const [lon2] = transpose(100, 0, datum, 0, 0);
        // With offset X, the longitude should differ
        expect(lon2).not.toBeCloseTo(lon1, 5);
    });

    it('itranspose with offset inverts correctly', () => {
        const offsetX = 5;
        const offsetY = -3;
        const [lon, lat] = transpose(offsetX, offsetY, datum, 10, 20);
        const [x, y] = itranspose(offsetX, offsetY, datum, lat, lon);
        expect(x).toBeCloseTo(20, 0);
        expect(y).toBeCloseTo(10, 0);
    });
});

describe("pointInPolygon", () => {
    const square = [[[0, 0], [10, 0], [10, 10], [0, 10], [0, 0]]];
    it("hits the inside and misses the outside", () => {
        expect(pointInPolygon([5, 5], square)).toBe(true);
        expect(pointInPolygon([15, 5], square)).toBe(false);
    });
    it("excludes holes", () => {
        const withHole = [...square, [[4, 4], [6, 4], [6, 6], [4, 6], [4, 4]]];
        expect(pointInPolygon([5, 5], withHole)).toBe(false);
        expect(pointInPolygon([2, 2], withHole)).toBe(true);
    });
});

describe('LINE_SOURCE_MAXZOOM', () => {
    // Mapbox GL snaps GeoJSON vertices to the 8192-unit grid of the source's maxzoom tile.
    const gridM = (z: number, latDeg: number) => 40075016.686 * Math.cos(latDeg * Math.PI / 180) / (2 ** z * 8192);

    it('the default maxzoom 18 grid is about a centimetre, a pixel at zoom 22 (the row wobble)', () => {
        expect(gridM(18, -40)).toBeGreaterThan(0.014);
        expect(gridM(18, -40) / (40075016.686 * Math.cos(-40 * Math.PI / 180) / (512 * 2 ** 22))).toBeCloseTo(1, 6);
    });

    it('line sources tile to the map maximum zoom, grid below 1.2 mm at any latitude', () => {
        expect(LINE_SOURCE_MAXZOOM).toBeGreaterThanOrEqual(22);
        expect(gridM(LINE_SOURCE_MAXZOOM, 0)).toBeLessThan(0.0012);
    });

    it('transpose keeps a straight 5 cm-spaced row straight to well below a millimetre', () => {
        const datum: [number, number, number] = [-40.5, 172.25, 0];  // made-up datum
        const d = 105 * Math.PI / 180;
        const pts = Array.from({length: 200}, (_, k) => [3 + 0.05 * k * Math.cos(d), -2 + 0.05 * k * Math.sin(d)]);
        for (const [x, y] of pts) {
            const [lon, lat] = transpose(0, 0, datum, y, x);
            const [bx, by] = itranspose(0, 0, datum, lat, lon);
            expect(Math.abs((bx - 3) * -Math.sin(d) + (by + 2) * Math.cos(d))).toBeLessThan(1e-6);
        }
    });
});
