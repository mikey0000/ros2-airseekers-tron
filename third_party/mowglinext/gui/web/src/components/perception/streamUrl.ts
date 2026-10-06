import type {CameraInfo, CameraStreamDefaults} from "../../hooks/useCameras.ts";

export type StreamVariant = "raw" | "annotated";

export interface StreamParams {
    variant: StreamVariant;
    quality: number;
    fps: number;
}

export function clamp(v: number, lo: number, hi: number): number {
    return Math.min(hi, Math.max(lo, v));
}

/** Append variant/quality/fps to a camera's streamUrl, bounded by the server limits. */
export function buildStreamUrl(cam: CameraInfo, p: StreamParams, defaults: CameraStreamDefaults): string {
    const q = new URLSearchParams();
    if (p.variant === "annotated" && cam.annotatedTopic) q.set("variant", "annotated");
    q.set("quality", String(Math.round(clamp(p.quality, 1, 100))));
    q.set("fps", String(clamp(p.fps, 1, defaults.maxFps)));
    const sep = cam.streamUrl.includes("?") ? "&" : "?";
    return `${cam.streamUrl}${sep}${q.toString()}`;
}

/** Which camera produced a detection frame (frame_id e.g. "left_oa_camera"). */
export function cameraForFrame(cameras: CameraInfo[], frameId?: string): CameraInfo | undefined {
    if (!frameId) return undefined;
    const id = frameId.replace(/^\//, "");
    return cameras.find((c) =>
        c.topic.split("/").includes(id) ||
        (c.annotatedTopic ?? "").split("/").includes(id) ||
        c.id === id || `${c.id}_camera` === id);
}
