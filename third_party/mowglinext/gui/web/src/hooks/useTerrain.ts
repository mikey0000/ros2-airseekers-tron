// Terrain memory (map server): latched JSON summary of per-area slope and
// incident clusters, plus POST /mowglinext/terrain/action for cluster actions.
import {Api, ContentType} from "../api/Api.ts";
import {useTopic} from "./useTopic.ts";
import {parseStringMsgJson} from "../utils/areaSettings.ts";
import type {TerrainSummary} from "../utils/terrain.ts";

export function selectTerrainSummary(raw: unknown): TerrainSummary | undefined {
    const d = parseStringMsgJson(raw);
    if (!d || !Array.isArray(d.areas)) return undefined;
    return d as unknown as TerrainSummary;
}

/** Latched /map_server_node/terrain_summary (topic key `terrainSummary`). */
export function useTerrainSummary(enabled = true): TerrainSummary | null {
    const {data} = useTopic<TerrainSummary | null>("terrainSummary", null, {
        select: (raw) => selectTerrainSummary(raw),
        enabled,
    });
    return enabled ? data : null;
}

export type TerrainAction = "keepout" | "confirm" | "dismiss" | "clear";

interface HttpLike {error?: {error?: string; message?: string}; data?: {message?: string}}

/** area_index 255 = all areas. cluster_id is required except for "clear". */
export async function postTerrainAction(api: Api<unknown>, action: TerrainAction, areaIndex = 255, clusterId?: number): Promise<void> {
    const body: Record<string, unknown> = {action, area_index: areaIndex};
    if (clusterId !== undefined) body.cluster_id = clusterId;
    try {
        await api.request({path: "/mowglinext/terrain/action", method: "POST", body, type: ContentType.Json, format: "json"});
    } catch (e) {
        const h = e as HttpLike;
        throw new Error(h?.error?.error || h?.error?.message || h?.data?.message || (e as Error)?.message || String(e));
    }
}
