import {describe, expect, it} from 'vitest';
import {
    BoundaryLineFeature,
    boundaryLineFeatureId,
    featuresFromJSON,
    serializeFeatures,
    type BoundaryLine,
    type MowingFeature,
} from '../../../types/map.ts';
import {transpose} from '../../../utils/map.tsx';
import {boundaryLineFeaturesFromMap, boundaryLinesForSave, newBoundaryLineId, ringAreaM2} from './boundaryLines.ts';

const datum: [number, number, number] = [48.0, 2.0, 0];
const offX = 1.5, offY = -0.5;

const lines: BoundaryLine[] = [
    {id: 'bl-a', name: 'Pond edge', points: [[1, 2], [4.5, 2.25], [7, -3]], width_m: 0.25},
    {id: 'bl-b', name: '', points: [[-2, 0], [-2, 10]], width_m: 0},
    {id: 'bl-c', name: 'Roses', points: [[3, 3], [6, 3]], width_m: 0.1, mode: 'blade'},
];

describe('boundary lines: map <-> features <-> PUT payload', () => {
    it('loads map.boundary_lines as BoundaryLineFeatures in display lon/lat', () => {
        const feats = boundaryLineFeaturesFromMap(lines, offX, offY, datum);
        const a = feats[boundaryLineFeatureId('bl-a')];
        expect(a).toBeInstanceOf(BoundaryLineFeature);
        expect(a.getName()).toBe('Pond edge');
        expect(a.getWidthM()).toBe(0.25);
        expect(a.getCoordinates()[1]).toEqual(transpose(offX, offY, datum, 2.25, 4.5));
        expect(Object.keys(feats)).toHaveLength(3);
        expect(a.getMode()).toBe('body');                                   // no mode = body
        expect(feats[boundaryLineFeatureId('bl-c')].getMode()).toBe('blade');
    });

    it('treats a missing field as no lines and skips lines with < 2 points', () => {
        expect(boundaryLineFeaturesFromMap(undefined, 0, 0, datum)).toEqual({});
        const one: BoundaryLine = {id: 'bl-x', name: 'x', points: [[0, 0]], width_m: 0};
        expect(boundaryLineFeaturesFromMap([one], 0, 0, datum)).toEqual({});
    });

    it('round-trips to the PUT payload in map metres with ids, names and widths', () => {
        const feats = boundaryLineFeaturesFromMap(lines, offX, offY, datum);
        const out = boundaryLinesForSave(feats, offX, offY, datum);
        expect(out.map((l) => [l.id, l.name, l.width_m, l.mode])).toEqual([
            ['bl-a', 'Pond edge', 0.25, 'body'], ['bl-b', '', 0, 'body'], ['bl-c', 'Roses', 0.1, 'blade']]);
        out.forEach((l, i) => l.points.forEach((p, j) => {
            expect(p[0]).toBeCloseTo(lines[i].points[j][0], 6);
            expect(p[1]).toBeCloseTo(lines[i].points[j][1], 6);
        }));
    });

    it('sends an empty set when every line was deleted (clears the stored lines)', () => {
        expect(boundaryLinesForSave({}, 0, 0, datum)).toEqual([]);
    });

    it('survives an undo/redo snapshot (serializeFeatures -> featuresFromJSON)', () => {
        const feats: Record<string, MowingFeature> = boundaryLineFeaturesFromMap(lines, offX, offY, datum);
        const restored = featuresFromJSON(serializeFeatures(feats));
        const a = restored[boundaryLineFeatureId('bl-a')];
        expect(a).toBeInstanceOf(BoundaryLineFeature);
        expect((restored[boundaryLineFeatureId('bl-c')] as BoundaryLineFeature).getMode()).toBe('blade');
        expect(boundaryLinesForSave(restored, offX, offY, datum)).toEqual(boundaryLinesForSave(feats, offX, offY, datum));
    });

    it('clamps width to 0..2 m and the name to 64 characters', () => {
        const f = new BoundaryLineFeature('bl-c', [[0, 0], [1, 1]], 'n'.repeat(80), 5);
        expect(f.getWidthM()).toBe(2);
        expect(f.getName()).toHaveLength(64);
        expect(f.withProps('wall', -1).getWidthM()).toBe(0);
        expect(f.withProps('wall', 0.3).id).toBe(f.id);
        expect(f.getMode()).toBe('body');
        expect(f.withProps('wall', 0.3, 'blade').getMode()).toBe('blade');
        expect(f.withProps('wall', 0.3, 'blade').withProps('bed', 0).getMode()).toBe('blade');   // kept
        expect(new BoundaryLineFeature('bl-d', [[0, 0], [1, 1]], '', 0, 'nonsense' as never).getMode()).toBe('body');
    });

    it('generates distinct short ids', () => {
        const a = newBoundaryLineId(), b = newBoundaryLineId();
        expect(a).toMatch(/^bl-[a-z0-9]+$/);
        expect(a).not.toBe(b);
    });
});

describe('ringAreaM2', () => {
    it('computes the area of a ring in map metres', () => {
        const ring = [[0, 0], [10, 0], [10, 5], [0, 5], [0, 0]].map(([x, y]) => transpose(offX, offY, datum, y, x));
        expect(ringAreaM2(ring, offX, offY, datum)).toBeCloseTo(50, 6);
    });
});
