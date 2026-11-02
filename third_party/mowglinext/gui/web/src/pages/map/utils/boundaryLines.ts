import type {Position} from "geojson";
import {
    BoundaryLineFeature,
    boundaryLineMode,
    type BoundaryLine,
    type MowingFeature,
} from "../../../types/map.ts";
import {itranspose, transpose} from "../../../utils/map.tsx";

type Datum = [number, number, number];

/** Short unique id for a boundary line drawn in this GUI ("bl-" + time + random). */
export function newBoundaryLineId(): string {
    return `bl-${Date.now().toString(36)}${Math.random().toString(36).slice(2, 6)}`;
}

/** Map.boundary_lines (map metres) -> editable features keyed by feature id. Absent = none. */
export function boundaryLineFeaturesFromMap(
    lines: BoundaryLine[] | undefined,
    offsetX: number,
    offsetY: number,
    datum: Datum,
): Record<string, BoundaryLineFeature> {
    const result: Record<string, BoundaryLineFeature> = {};
    for (const line of lines ?? []) {
        if (!line?.id || (line.points?.length ?? 0) < 2) continue;
        const coords = line.points.map(([x, y]) => transpose(offsetX, offsetY, datum, y, x));
        const f = new BoundaryLineFeature(line.id, coords, line.name ?? "", line.width_m ?? 0,
            boundaryLineMode(line.mode));
        result[f.id] = f;
    }
    return result;
}

/** Every boundary line of `features` in map metres, for PUT /mowglinext/map (full set). */
export function boundaryLinesForSave(
    features: Record<string, MowingFeature>,
    offsetX: number,
    offsetY: number,
    datum: Datum,
): BoundaryLine[] {
    return Object.values(features)
        .filter((f): f is BoundaryLineFeature => f instanceof BoundaryLineFeature)
        .filter((f) => f.getCoordinates().length >= 2)
        .map((f) => ({
            id: f.getLineId(),
            name: f.getName(),
            points: f.getCoordinates().map((c) => itranspose(offsetX, offsetY, datum, c[1], c[0])),
            width_m: f.getWidthM(),
            mode: f.getMode(),
        }));
}

/** Area in m² of a [lon, lat] ring, computed in map metres (shoelace). */
export function ringAreaM2(ring: Position[], offsetX: number, offsetY: number, datum: Datum): number {
    const pts = ring.map((c) => itranspose(offsetX, offsetY, datum, c[1], c[0]));
    let s = 0;
    for (let i = 0, j = pts.length - 1; i < pts.length; j = i++) {
        s += pts[j][0] * pts[i][1] - pts[i][0] * pts[j][1];
    }
    return Math.abs(s / 2);
}
