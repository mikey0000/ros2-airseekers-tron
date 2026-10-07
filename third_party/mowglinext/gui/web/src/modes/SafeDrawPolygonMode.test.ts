import {describe, expect, it} from 'vitest';
import {groundDistanceM, shouldFinishOnVertexTap} from './SafeDrawPolygonMode';

describe('shouldFinishOnVertexTap', () => {
    it('4th corner of a square tapped inside the 25 px buffer of the 3rd corner adds a corner', () => {
        // 3 m away on the ground, 20 px on screen: stock draw finished -> triangle
        expect(shouldFinishOnVertexTap(2, 3, 20, 3)).toBe(false);
    });
    it('4th corner landing near (but not on) the first corner adds a corner', () => {
        expect(shouldFinishOnVertexTap(0, 3, 22, 0.8)).toBe(false);
    });
    it('tapping the first corner marker with >=3 corners finishes', () => {
        expect(shouldFinishOnVertexTap(0, 4, 4, 0.5)).toBe(true);
    });
    it('first marker cannot finish a 2-corner shape', () => {
        expect(shouldFinishOnVertexTap(0, 2, 1, 0.01)).toBe(false);
    });
    it('double tap on the same spot (<=5 cm) finishes', () => {
        expect(shouldFinishOnVertexTap(3, 4, 30, 0.03)).toBe(true);
        expect(shouldFinishOnVertexTap(3, 4, 3, 0.2)).toBe(false);
    });
});

describe('groundDistanceM', () => {
    it('3 m east at mid latitude', () => {
        const lat = 52, dLng = 3 / (111195 * Math.cos(lat * Math.PI / 180));
        expect(groundDistanceM([0, lat], [dLng, lat])).toBeCloseTo(3, 2);
    });
});
