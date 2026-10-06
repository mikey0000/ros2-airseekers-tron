import {describe, it, expect} from 'vitest';
import {computeCorridor} from './usePathTool.ts';
import {lonLatToLocal, localToLonLat} from '../../../utils/corridor.ts';

const datum: [number, number, number] = [48.85, 2.35, 0];
const ll = (x: number, y: number) => localToLonLat(datum, [x, y]);

describe('computeCorridor', () => {
    it('buffers the clicked polyline without a dock', () => {
        const {centerline, polygons} = computeCorridor(datum, [ll(0, 0), ll(10, 0)], 0.8, null, true);
        expect(centerline).toHaveLength(2);
        expect(polygons).toHaveLength(1);
        const ys = polygons[0].coordinates[0].map((p) => lonLatToLocal(datum, p)[1]);
        expect(Math.max(...ys) - Math.min(...ys)).toBeCloseTo(0.8, 4);
    });

    it('snaps the end nearest the dock through the approach point', () => {
        const dock = {lonLat: ll(12, 0), heading: Math.PI}; // facing west, approach at x=11.2
        const {centerline} = computeCorridor(datum, [ll(0, 0), ll(10, 0)], 0.8, dock, true);
        const local = centerline.map((p) => lonLatToLocal(datum, p));
        expect(local).toHaveLength(4);
        expect(local[2][0]).toBeCloseTo(11.2, 4);
        expect(local[3][0]).toBeCloseTo(12, 4);
    });

    it('ignores the dock when snapping is off', () => {
        const dock = {lonLat: ll(12, 0), heading: Math.PI};
        const {centerline} = computeCorridor(datum, [ll(0, 0), ll(10, 0)], 0.8, dock, false);
        expect(centerline).toHaveLength(2);
    });
});
