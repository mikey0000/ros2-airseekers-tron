import MapboxDraw from '@mapbox/mapbox-gl-draw';

/**
 * draw_polygon with a safe "finish on vertex tap".
 *
 * Stock mapbox-gl-draw finishes the polygon whenever a click/tap hits one of
 * the two vertex markers it shows while drawing (the first vertex and the
 * last placed one). On touch the hit box is `touchBuffer` = 25 px, so on a
 * zoomed-out phone map a corner tapped metres away from the previous corner
 * (or from the first one) is swallowed: the polygon finishes WITHOUT that
 * corner and a drawn square becomes a triangle.
 *
 * Here a vertex hit only finishes the polygon when it is unambiguous:
 *  - the first vertex, with >= 3 corners placed, tapped within
 *    FINISH_FIRST_VERTEX_PX on screen (the marker itself), or
 *  - the last placed vertex tapped again within FINISH_SAME_POINT_M on the
 *    ground (a deliberate double tap / double click on the same spot).
 * Anything else adds the tap as a new corner.
 */
export const FINISH_FIRST_VERTEX_PX = 10;
export const FINISH_SAME_POINT_M = 0.05;

const EARTH_R = 6371008.8;

export function groundDistanceM(a: [number, number], b: [number, number]): number {
    const toRad = Math.PI / 180;
    const lat = ((a[1] + b[1]) / 2) * toRad;
    const dx = (b[0] - a[0]) * toRad * Math.cos(lat) * EARTH_R;
    const dy = (b[1] - a[1]) * toRad * EARTH_R;
    return Math.hypot(dx, dy);
}

/**
 * Decide whether a tap on a drawing-vertex marker finishes the polygon.
 * @param vertexIndex index of the hit vertex in the ring
 * @param placed number of corners already placed (currentVertexPosition)
 * @param tapPxDist screen distance tap -> vertex (px)
 * @param tapGroundM ground distance tap -> vertex (m)
 */
export function shouldFinishOnVertexTap(
    vertexIndex: number, placed: number, tapPxDist: number, tapGroundM: number,
): boolean {
    if (vertexIndex === 0) {
        if (placed >= 3 && tapPxDist <= FINISH_FIRST_VERTEX_PX) return true;
        // a 1-corner "polygon": tapping the only corner again = same point
        return placed >= 3 && tapGroundM <= FINISH_SAME_POINT_M;
    }
    if (vertexIndex === placed - 1) return tapGroundM <= FINISH_SAME_POINT_M;
    return false;
}

const DrawPolygon = MapboxDraw.modes.draw_polygon as Record<string, any>;
const SafeDrawPolygonMode: Record<string, any> = {...DrawPolygon};

SafeDrawPolygonMode.clickOnVertex = function (state: any, e: any) {
    const path: string = e?.featureTarget?.properties?.coord_path ?? '';
    const idx = Number(path.split('.').pop());
    const vertex = state.polygon.coordinates?.[0]?.[idx] as [number, number] | undefined;
    if (vertex && e?.lngLat && Number.isFinite(idx)) {
        const tap: [number, number] = [e.lngLat.lng, e.lngLat.lat];
        let px = Infinity;
        try {
            const p = this.map.project(vertex);
            px = Math.hypot(p.x - e.point.x, p.y - e.point.y);
        } catch { /* no map projection: rely on ground distance */ }
        if (!shouldFinishOnVertexTap(idx, state.currentVertexPosition, px, groundDistanceM(vertex, tap))) {
            return this.clickAnywhere(state, e);
        }
    }
    return DrawPolygon.clickOnVertex.call(this, state, e);
};

export default SafeDrawPolygonMode;
