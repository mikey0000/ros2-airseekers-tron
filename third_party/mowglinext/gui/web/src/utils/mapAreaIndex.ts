import type {Map} from "../types/ros.ts";

// The map topic separates navigation areas; array position is not a ROS ID.
export function mowingAreaIndex(map: Map | undefined, position: number | undefined): number | undefined {
    if (position === undefined || !Number.isInteger(position) || position < 0 || position >= (map?.working_area?.length ?? 0)) return;
    const index = map?.working_area_indices?.[position];
    return index !== undefined && Number.isInteger(index) && index >= 0 ? index : undefined;
}

export type MowingAreaChoice = {index: number; name: string};

/** Mowing areas of the /map topic as (map-server index, display name) pairs. */
export function mowingAreaChoices(map: Map | undefined, unnamed: (order: number) => string): MowingAreaChoice[] {
    const out: MowingAreaChoice[] = [];
    (map?.working_area ?? []).forEach((area, position) => {
        const index = mowingAreaIndex(map, position);
        if (index === undefined) return;
        out.push({index, name: area.name?.trim() || unnamed(position + 1)});
    });
    return out;
}
