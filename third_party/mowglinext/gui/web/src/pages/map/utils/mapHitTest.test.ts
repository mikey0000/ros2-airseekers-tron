import {describe, expect, it} from 'vitest';
import type {Position} from 'geojson';
import {
    BoundaryLineFeature,
    MowingAreaFeature,
    NavigationFeature,
    ObstacleFeature,
    type MowingFeature,
} from '../../../types/map.ts';
import {transpose} from '../../../utils/map.tsx';
import {distanceToPolylineM, hitTestMap, metresPerPixel} from './mapHitTest.ts';

const datum: [number, number, number] = [48.0, 2.0, 0];
/** Map metres [x, y] -> [lon, lat]. */
const ll = (x: number, y: number): [number, number] => transpose(0, 0, datum, y, x);
const rect = (x0: number, y0: number, x1: number, y1: number): Position[] =>
    [ll(x0, y0), ll(x1, y0), ll(x1, y1), ll(x0, y1), ll(x0, y0)];

function fixture() {
    const area = new MowingAreaFeature('area-0-area-0', 1);
    area.setGeometry({type: 'Polygon', coordinates: [rect(0, 0, 20, 20)]});
    const nogo = new ObstacleFeature('area-0-obstacle-0', area);
    nogo.setGeometry({type: 'Polygon', coordinates: [rect(5, 5, 8, 8)]});
    const nav = new NavigationFeature('navigation-0-area-0');
    nav.setGeometry({type: 'Polygon', coordinates: [rect(20, 0, 30, 2)]});
    const wall = new BoundaryLineFeature('bl-1', [ll(12, 0), ll(12, 20)]);
    const features: Record<string, MowingFeature> = {
        [area.id]: area, [nogo.id]: nogo, [nav.id]: nav, [wall.id]: wall,
    };
    return {features, area, nogo, nav, wall};
}

describe('hitTestMap', () => {
    it('routes a click inside a no-go zone (inside its area) to the no-go zone, not the area', () => {
        const {features, nogo} = fixture();
        expect(hitTestMap(ll(6, 6), features, 0.3)).toEqual({kind: 'nogo', id: nogo.id});
    });

    it('routes a click in the area only to the mowing area', () => {
        const {features, area} = fixture();
        expect(hitTestMap(ll(2, 15), features, 0.3)).toEqual({kind: 'area', id: area.id});
    });

    it('routes a click in a navigation area to it', () => {
        const {features, nav} = fixture();
        expect(hitTestMap(ll(25, 1), features, 0.3)).toEqual({kind: 'nav', id: nav.id});
    });

    it('selects a boundary line within the tolerance and misses it just outside', () => {
        const {features, wall, area} = fixture();
        expect(hitTestMap(ll(12.25, 10), features, 0.3)).toEqual({kind: 'boundary', id: wall.id});
        expect(hitTestMap(ll(12.4, 10), features, 0.3)).toEqual({kind: 'area', id: area.id});
    });

    it('prefers a boundary line over a no-go zone it crosses', () => {
        const {features, wall} = fixture();
        const crossing = new BoundaryLineFeature('bl-2', [ll(6, 0), ll(6, 20)]);
        features[crossing.id] = crossing;
        expect(hitTestMap(ll(6.1, 6), features, 0.3)).toEqual({kind: 'boundary', id: crossing.id});
        expect(wall.id).not.toBe(crossing.id);
    });

    it('returns null when nothing is under the click', () => {
        const {features} = fixture();
        expect(hitTestMap(ll(-10, -10), features, 0.3)).toBeNull();
    });

    it('picks the inner area of two nested mowing areas', () => {
        const outer = new MowingAreaFeature('area-0-area-0', 1);
        outer.setGeometry({type: 'Polygon', coordinates: [rect(0, 0, 20, 20)]});
        const inner = new MowingAreaFeature('area-1-area-0', 2);
        inner.setGeometry({type: 'Polygon', coordinates: [rect(2, 2, 6, 6)]});
        expect(hitTestMap(ll(3, 3), {[outer.id]: outer, [inner.id]: inner}, 0.3)).toEqual({kind: 'area', id: inner.id});
    });
});

describe('distanceToPolylineM / metresPerPixel', () => {
    it('measures the perpendicular distance and the distance past an end', () => {
        const line = [ll(0, 0), ll(10, 0)];
        expect(distanceToPolylineM(ll(5, 2), line)).toBeCloseTo(2, 2);
        expect(distanceToPolylineM(ll(13, 4), line)).toBeCloseTo(5, 2);
    });

    it('halves the ground resolution per zoom level', () => {
        expect(metresPerPixel(0, 0)).toBeCloseTo(78271.5, 0);
        expect(metresPerPixel(48, 21) / metresPerPixel(48, 20)).toBeCloseTo(0.5, 6);
    });
});
