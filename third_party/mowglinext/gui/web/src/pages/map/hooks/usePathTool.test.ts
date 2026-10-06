import {describe, it, expect} from 'vitest';
import {computeCorridor} from './usePathTool.ts';
import {lonLatToLocal, localToLonLat} from '../../../utils/corridor.ts';

const datum: [number, number, number] = [48.85, 2.35, 0];
const ll = (x: number, y: number) => localToLonLat(datum, [x, y]);
const local = (ps: number[][]) => ps.map((p) => lonLatToLocal(datum, p));
const area = [ll(-10, -5), ll(0, -5), ll(0, 5), ll(-10, 5), ll(-10, -5)];

describe('computeCorridor', () => {
    it('buffers the clicked polyline with square caps and reports loose ends', () => {
        const r = computeCorridor(datum, [ll(3, 0), ll(10, 0)], 0.7, null, false);
        expect(r.centerline).toHaveLength(2);
        expect(r.polygons).toHaveLength(1);
        const ring = local(r.polygons[0].coordinates[0]);
        const ys = ring.map((p) => p[1]);
        const xs = ring.map((p) => p[0]);
        expect(Math.max(...ys) - Math.min(...ys)).toBeCloseTo(0.7, 4);
        expect(Math.min(...xs)).toBeCloseTo(3 - 0.35, 4);
        expect(r.start).toBe('none');
        expect(r.end).toBe('none');
    });

    it('snaps an end near an area outline onto it and an end near the dock to the approach pose', () => {
        const dock = {lonLat: ll(12, 0), heading: Math.PI}; // facing west, approach at x=11.2
        const r = computeCorridor(datum, [ll(0.3, 0.1), ll(11.0, 0.2)], 0.7, dock, false, [area]);
        const c = local(r.centerline);
        expect(r.start).toBe('area');
        expect(c[0][0]).toBeCloseTo(0, 4);
        expect(r.end).toBe('dock');
        expect(c[1][0]).toBeCloseTo(11.2, 4);
        expect(c[1][1]).toBeCloseTo(0, 4);
        expect(c).toHaveLength(2);
    });

    it('continues into the dock only when asked', () => {
        const dock = {lonLat: ll(12, 0), heading: Math.PI};
        const r = computeCorridor(datum, [ll(-2, 0), ll(11.1, 0)], 0.7, dock, true, [area]);
        const c = local(r.centerline);
        expect(r.start).toBe('inside');
        expect(c).toHaveLength(3);
        expect(c[2][0]).toBeCloseTo(12, 4);
    });

    it('ignores a far dock', () => {
        const dock = {lonLat: ll(30, 0), heading: Math.PI};
        const r = computeCorridor(datum, [ll(0, 0), ll(10, 0)], 0.7, dock, true);
        expect(r.centerline).toHaveLength(2);
        expect(r.end).toBe('none');
    });
});
