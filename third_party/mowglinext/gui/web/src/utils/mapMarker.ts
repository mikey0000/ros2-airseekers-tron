// Raster/vector map markers a robot profile can supply instead of the
// URDF-derived silhouette (see MowerModel.mapMarker / dockMarker).
//
// The image is drawn as a Mapbox `image` source whose four corners are placed
// in metres, so it scales with the robot's real footprint at every zoom and is
// resampled by the GPU at the display's device pixel ratio (no fixed-pixel
// icon). It rotates with the pose heading.

import {transpose} from "./map.tsx";

export type MapMarker = {
    /** URL of the image (served from web/public, e.g. "/robots/x.png"). */
    src: string;
    /** Real-world size of the image's full extent, metres. */
    widthM: number;
    lengthM: number;
    /**
     * Rotation from "image nose points up" to the robot's +x (forward) axis,
     * degrees counter-clockwise. 0 for a top-down image whose nose is at the top.
     */
    headingOffsetDeg?: number;
    /**
     * Where the pose origin (base_link) sits in the image, normalised 0..1
     * (x to the right, y downwards, image unrotated). Default: derived from
     * the preset's chassis_center_x / chassis_length, else the centre.
     */
    anchor?: {x: number; y: number};
};

/** Image corners [top-left, top-right, bottom-right, bottom-left] as [lon, lat]. */
export type MarkerCorners = [[number, number], [number, number], [number, number], [number, number]];

type ModelLike = {mapMarker?: MapMarker; dockMarker?: MapMarker; defaults?: Record<string, number>};

/**
 * Anchor of `marker` for a model: the explicit one, else base_link placed
 * chassis_center_x behind the image centre (the image spans the chassis), else
 * the centre.
 */
export function resolveAnchor(marker: MapMarker, defaults?: Record<string, number>): {x: number; y: number} {
    if (marker.anchor) return marker.anchor;
    const cx = defaults?.chassis_center_x;
    if (cx !== undefined && Number.isFinite(cx) && marker.lengthM > 0) {
        return {x: 0.5, y: Math.min(1, Math.max(0, 0.5 + cx / marker.lengthM))};
    }
    return {x: 0.5, y: 0.5};
}

/** The robot marker of a model/profile, or undefined = keep the stock silhouette. */
export function selectRobotMarker(model: ModelLike | null | undefined): (MapMarker & {anchor: {x: number; y: number}}) | undefined {
    const m = model?.mapMarker;
    if (!m || !m.src || !(m.widthM > 0) || !(m.lengthM > 0)) return undefined;
    return {...m, anchor: resolveAnchor(m, model?.defaults)};
}

/** The dock marker of a model/profile, or undefined = keep the stock dot. */
export function selectDockMarker(model: ModelLike | null | undefined): (MapMarker & {anchor: {x: number; y: number}}) | undefined {
    const m = model?.dockMarker;
    if (!m || !m.src || !(m.widthM > 0) || !(m.lengthM > 0)) return undefined;
    return {...m, anchor: m.anchor ?? {x: 0.5, y: 0.5}};
}

/**
 * Corners of the marker image in the ROS map frame (x east, y north), for a
 * pose (x, y, heading rad CCW from east). Order: image top-left, top-right,
 * bottom-right, bottom-left (what Mapbox image sources expect).
 */
export function markerCornersLocal(
    marker: MapMarker & {anchor: {x: number; y: number}},
    x: number, y: number, heading: number,
): [number, number][] {
    const yaw = heading + ((marker.headingOffsetDeg ?? 0) * Math.PI) / 180;
    const cos = Math.cos(yaw);
    const sin = Math.sin(yaw);
    const {widthM: w, lengthM: l, anchor} = marker;
    // Image pixel (u, v) in 0..1 -> robot frame (forward f, left s):
    // the image top is forward, the image left is the robot's left.
    const pt = (u: number, v: number): [number, number] => {
        const f = (anchor.y - v) * l;
        const s = (anchor.x - u) * w;
        return [x + f * cos - s * sin, y + f * sin + s * cos];
    };
    return [pt(0, 0), pt(1, 0), pt(1, 1), pt(0, 1)];
}

/** markerCornersLocal projected to [lon, lat] for the map. */
export function markerCorners(
    offsetX: number, offsetY: number, datum: [number, number, number],
    marker: MapMarker & {anchor: {x: number; y: number}},
    x: number, y: number, heading: number,
): MarkerCorners {
    return markerCornersLocal(marker, x, y, heading)
        .map(([px, py]) => transpose(offsetX, offsetY, datum, py, px)) as MarkerCorners;
}
