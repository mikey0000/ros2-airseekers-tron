import {describe, it, expect} from 'vitest';
import {MapStyle, type_sort_key} from './MapStyle';

describe('MapStyle polygon draw order (2026-10-09)', () => {
    it('sorts obstacles above paths above mowing areas', () => {
        // ['case', obstacleCond, 2, navigationCond, 1, 0]
        expect(type_sort_key[2]).toBe(2);
        expect(type_sort_key[4]).toBe(1);
        expect(type_sort_key[5]).toBe(0);
    });

    it('applies the sort key to the inactive fill and stroke layers', () => {
        const byId = Object.fromEntries(MapStyle.map((l: any) => [l.id, l]));
        expect(byId['gl-draw-polygon-fill-inactive'].layout['fill-sort-key']).toBe(type_sort_key);
        expect(byId['gl-draw-polygon-stroke-inactive'].layout['line-sort-key']).toBe(type_sort_key);
    });
});
