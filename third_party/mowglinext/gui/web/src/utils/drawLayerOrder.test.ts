import {describe, it, expect} from 'vitest';
import {drawLayerMove} from './drawLayerOrder';

const L = (id: string, type = 'line') => ({id, type});

describe('drawLayerMove', () => {
    it('moves the draw block above a raster added after it (mow progress / imagery)', () => {
        const layers = [L('satellite', 'raster'), L('gl-draw-polygon-fill-inactive.cold', 'fill'),
            L('gl-draw-polygon-stroke-inactive.cold'), L('mow-progress-layer', 'raster'),
            L('terrain-cluster-hull'), L('robot-image-layer', 'raster')];
        expect(drawLayerMove(layers)).toEqual({
            ids: ['gl-draw-polygon-fill-inactive.cold', 'gl-draw-polygon-stroke-inactive.cold'],
            before: 'terrain-cluster-hull',
        });
    });

    it('is a no-op when the draw block is already above every raster (no move loop)', () => {
        const layers = [L('satellite', 'raster'), L('imagery-x', 'raster'), L('mower', 'symbol'),
            L('gl-draw-polygon-fill-inactive.cold', 'fill'), L('robot-image-layer', 'raster')];
        expect(drawLayerMove(layers)).toBeNull();
    });

    it('ignores the robot / dock image markers that keep themselves on top', () => {
        const layers = [L('gl-draw-polygon-fill-inactive.cold', 'fill'), L('dock-image-layer', 'raster')];
        expect(drawLayerMove(layers)).toBeNull();
    });

    it('moves to the top when the raster is the last layer', () => {
        const layers = [L('gl-draw-a', 'fill'), L('mow-progress-layer', 'raster')];
        expect(drawLayerMove(layers)).toEqual({ids: ['gl-draw-a'], before: undefined});
    });
});
