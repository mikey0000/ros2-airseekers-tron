import {useEffect, useState} from "react";
import type {CameraStatus} from "./useCameras.ts";

/**
 * Polls GET /api/cameras/:id/health while `enabled` (the tile is streaming):
 * fps and last-frame age as measured by the MJPEG proxy. Keeps the last
 * answer on a failed poll; `initial` (from GET /api/cameras) seeds it.
 */
export function useCameraHealth(healthUrl: string | undefined, enabled: boolean, initial?: CameraStatus,
                                intervalMs = 1000): CameraStatus | undefined {
    const [status, setStatus] = useState<CameraStatus | undefined>(initial);
    useEffect(() => {
        if (!healthUrl || !enabled) return;
        let cancelled = false;
        let timer: number | undefined;
        const ctrl = new AbortController();
        const poll = async () => {
            try {
                const res = await fetch(healthUrl, {signal: ctrl.signal, cache: "no-store"});
                if (res.ok) {
                    const body = await res.json() as { status?: CameraStatus };
                    if (!cancelled && body?.status) setStatus(body.status);
                }
            } catch {
                // keep the last value
            }
            if (!cancelled) timer = window.setTimeout(poll, intervalMs);
        };
        void poll();
        return () => {
            cancelled = true;
            ctrl.abort();
            if (timer !== undefined) window.clearTimeout(timer);
        };
    }, [healthUrl, enabled, intervalMs]);
    return status;
}
