/**
 * Keep the mapbox-gl-draw layers (area / path / obstacle polygons) above every raster
 * layer (2026-10-09). react-map-gl appends a declarative <Layer> at the TOP of the style
 * when it mounts, so a raster that appears after the draw control (mow-progress 70 %,
 * terrain heat map 75 %, lidar map, custom imagery tiles) covered the polygons and the
 * areas were no longer visible. Only the draw block is moved, to just above the last
 * raster; vector overlays declared later (lines, labels) stay above it, and the robot /
 * dock image markers (`*-image-layer`, which put themselves on top) are left alone, so
 * the two "keep on top" rules never fight.
 */
export interface StyleLayerLike {
    id: string;
    type: string;
}

const isDraw = (id: string) => id.startsWith('gl-draw-');
const isMarker = (id: string) => id.endsWith('-image-layer');

/** The draw layer ids to move and the layer to insert them before (undefined = top),
 *  or null when the order is already right. */
export function drawLayerMove(layers: StyleLayerLike[]): {ids: string[]; before?: string} | null {
    const ids = layers.filter((l) => isDraw(l.id)).map((l) => l.id);
    if (!ids.length) return null;
    let lastRaster = -1;
    layers.forEach((l, i) => {
        if (l.type === 'raster' && !isDraw(l.id) && !isMarker(l.id)) lastRaster = i;
    });
    const firstDraw = layers.findIndex((l) => isDraw(l.id));
    if (firstDraw > lastRaster) return null;
    let before: string | undefined;
    for (let i = lastRaster + 1; i < layers.length; i++) {
        if (!isDraw(layers[i].id)) { before = layers[i].id; break; }
    }
    return {ids, before};
}
