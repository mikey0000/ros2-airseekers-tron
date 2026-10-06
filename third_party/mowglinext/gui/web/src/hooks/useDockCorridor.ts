import type {Point32} from "../types/ros.ts";
import {useTopic} from "./useTopic.ts";

/** Dock corridor outline in map-frame metres (x east, y north). */
export type DockCorridor = { x: number; y: number }[];

// geometry_msgs/PolygonStamped arrives as {header, polygon: {points: Point32[]}}.
// Anything else (no message yet, malformed payload) maps to an empty ring.
export const selectDockCorridor = (raw: unknown): DockCorridor => {
    const pts = (raw as { polygon?: { points?: Point32[] } } | null | undefined)?.polygon?.points;
    if (!Array.isArray(pts)) return [];
    return pts
        .filter((p) => Number.isFinite(p?.x) && Number.isFinite(p?.y))
        .map((p) => ({x: p.x as number, y: p.y as number}));
};

/**
 * Latched `/map_server_node/dock_corridor` (topic key `dockCorridor`): the
 * corridor the map server frees in the navigation mask between the dock, its
 * approach pose and the nearest area. Map servers that do not publish it just
 * leave the ring empty, which the map treats as "nothing to draw".
 */
export const useDockCorridor = (enabled = true): DockCorridor =>
    useTopic<DockCorridor>("dockCorridor", [], {select: selectDockCorridor, enabled}).data;
