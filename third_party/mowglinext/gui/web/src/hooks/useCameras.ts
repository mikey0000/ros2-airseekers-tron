import {useCallback, useEffect, useState} from "react";
import {useApi} from "./useApi.ts";

export interface CameraInfo {
    id: string;
    label: string;
    topic: string;
    annotatedTopic?: string;
    width?: number;
    height?: number;
    streamUrl: string;
    snapshotUrl: string;
}

export interface CameraStreamDefaults {
    quality: number;
    fps: number;
    maxFps: number;
}

export interface CamerasResponse {
    cameras: CameraInfo[];
    source?: string;
    available: boolean;
    hint?: string;
    defaults: CameraStreamDefaults;
}

export const FALLBACK_DEFAULTS: CameraStreamDefaults = {quality: 50, fps: 5, maxFps: 15};

export interface UseCamerasResult {
    data: CamerasResponse | null;
    loading: boolean;
    /** True when the request itself failed (backend without the route / offline). */
    error: boolean;
    reload: () => void;
}

function isCamerasResponse(d: unknown): d is CamerasResponse {
    const r = d as CamerasResponse | null;
    return !!r && Array.isArray(r.cameras) && typeof r.available === "boolean";
}

/** Fetches GET /api/cameras once on mount; `reload` re-probes the stream server. */
export const useCameras = (): UseCamerasResult => {
    const api = useApi();
    const [data, setData] = useState<CamerasResponse | null>(null);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState(false);
    const [nonce, setNonce] = useState(0);

    useEffect(() => {
        let cancelled = false;
        setLoading(true);
        (async () => {
            try {
                const res = await api.request<CamerasResponse>({path: "/cameras", method: "GET", format: "json"});
                if (cancelled) return;
                if (isCamerasResponse(res.data)) {
                    setData({...res.data, defaults: {...FALLBACK_DEFAULTS, ...res.data.defaults}});
                    setError(false);
                } else {
                    setError(true);
                }
            } catch {
                if (!cancelled) setError(true);
            } finally {
                if (!cancelled) setLoading(false);
            }
        })();
        return () => {
            cancelled = true;
        };
    }, [api, nonce]);

    const reload = useCallback(() => setNonce((n) => n + 1), []);
    return {data, loading, error, reload};
};
