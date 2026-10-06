import {useTopic} from "./useTopic.ts";
import {MissionPlan, MissionProgress, parseMissionPlan, parseMissionProgress} from "../utils/missionProgress.ts";

/** Current area's drivable sub-paths (latched, republished on plan change). */
export const useMissionPlan = (enabled = true): MissionPlan | null =>
    useTopic<MissionPlan | null>("missionPlan", null, {
        enabled,
        select: (raw) => parseMissionPlan(raw) ?? undefined,
    }).data;

/** Live mission progress (1 Hz, latched); null until the mission published one. */
export const useMissionProgress = (enabled = true): MissionProgress | null =>
    useTopic<MissionProgress | null>("missionProgress", null, {
        enabled,
        select: (raw) => parseMissionProgress(raw) ?? undefined,
    }).data;
