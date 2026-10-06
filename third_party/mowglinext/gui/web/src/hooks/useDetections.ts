import {useTopic} from "./useTopic.ts";

/** Reduced Detection2DArray summary produced by the backend (pkg/providers/vision.go). */
/** One detection: top-left corner + size in source image pixels. */
export interface DetectionBox {
    class: string;
    score: number;
    x: number;
    y: number;
    w: number;
    h: number;
}

export interface DetectionSummary {
    count: number;
    /** Per-detection boxes (best first); absent on older backends. */
    boxes?: DetectionBox[];
    classes: string[];
    max_score: number;
    stamp?: { sec: number; nanosec: number };
    frame_id?: string;
}

export const EMPTY_DETECTIONS: DetectionSummary = {count: 0, classes: [], max_score: 0};

/**
 * Latest detection summary. Pass `enabled=false` to drop the upstream
 * subscription (hidden tab, camera stack unavailable).
 */
export const useDetections = (enabled = true) =>
    useTopic<DetectionSummary>("detections", EMPTY_DETECTIONS, {
        // Unthrottled: det_ros alternates cameras (frame_id) at <= 5 Hz each; a
        // throttle would keep one camera and drop the other.
        withTimestamp: true,
        enabled,
        select: (raw) => {
            const d = raw as Partial<DetectionSummary> | null;
            if (!d || typeof d.count !== "number") return undefined;
            return {
                ...d,
                classes: Array.isArray(d.classes) ? d.classes : [],
                boxes: Array.isArray(d.boxes) ? d.boxes : [],
            } as DetectionSummary;
        },
    });

/** std_msgs/Bool latch: the vision obstacle detector says something is close. */
export const useVisionObstacleClose = (enabled = true): boolean =>
    useTopic<boolean>("visionObstacleClose", false, {
        enabled,
        select: (raw) => {
            if (typeof raw === "boolean") return raw;
            const d = raw as { data?: unknown } | null;
            return typeof d?.data === "boolean" ? d.data : undefined;
        },
    }).data;
