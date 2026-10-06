import type {RobotCamera} from "../constants/robotProfiles.ts";

/** Minimal shape of an accumulated /diagnostics entry (see hooks/useDiagnostics.ts). */
export type FreshnessStatus = {name?: string; hardware_id?: string; level: number; receivedAt: number};

export type CameraFreshness = {
    total: number;
    /** Cameras whose image topic is reported fresh (diagnostic OK/WARN, recent). */
    live: number;
    /** Cameras with any freshness signal at all; the rest are "unknown". */
    reporting: number;
};

/** A topic-frequency status older than this no longer counts as fresh. */
export const CAMERA_FRESH_MS = 15_000;

/**
 * Freshness of the robot's camera image topics, from /diagnostics.
 *
 * A camera counts as reporting when a diagnostic status names its image topic
 * (diagnostic_updater's TopicDiagnostic names its status
 * "<node>: <topic> topic status") or its id as hardware_id. It is live when
 * that status is OK or WARN and was received within CAMERA_FRESH_MS. Without
 * any such status the camera is "unknown" rather than faked as streaming,
 * the same rule the anatomy applies to the LiDAR.
 *
 * Returns undefined for a robot without cameras.
 */
export function cameraFreshness(
    cameras: readonly RobotCamera[],
    statuses: readonly FreshnessStatus[] | undefined,
    nowMs: number,
): CameraFreshness | undefined {
    if (cameras.length === 0) return undefined;
    let live = 0;
    let reporting = 0;
    for (const cam of cameras) {
        const match = (statuses ?? []).find((s) =>
            (s.name ?? "").includes(cam.topic) || (s.hardware_id !== undefined && s.hardware_id === cam.id));
        if (!match) continue;
        reporting += 1;
        if (match.level < 2 && nowMs - match.receivedAt < CAMERA_FRESH_MS) live += 1;
    }
    return {total: cameras.length, live, reporting};
}
