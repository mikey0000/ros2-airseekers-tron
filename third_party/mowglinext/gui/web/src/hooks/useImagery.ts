import {useCallback, useEffect, useRef, useState} from "react";
import {httpBase} from "../utils/apiHost.ts";
import type {ImageryControlPoint, ImageryOverlay, ImageryPlacement} from "../utils/imagery.ts";

const BASE = "/api/mowglinext";

async function jsonFetch<T>(path: string, init?: RequestInit): Promise<T> {
    const res = await fetch(`${httpBase()}${BASE}${path}`, {
        ...init,
        headers: init?.body ? {"Content-Type": "application/json"} : undefined,
    });
    const text = await res.text();
    const body = (text ? JSON.parse(text) : {}) as {error?: string};
    if (!res.ok) throw new Error(body.error ?? `HTTP ${res.status}`);
    return body as T;
}

export interface ImageryPatch {
    label?: string;
    opacity?: number;
    visible?: boolean;
    placement?: ImageryPlacement;
    controlPoints?: ImageryControlPoint[];
}

export interface UploadState {
    fileName: string;
    /** 0..1 of the HTTP transfer. */
    progress: number;
}

/**
 * Custom imagery overlays (bottom-most first). Polls while any overlay is
 * still being converted on the robot.
 */
export function useImagery(enabled = true) {
    const [overlays, setOverlays] = useState<ImageryOverlay[]>([]);
    const [loaded, setLoaded] = useState(false);
    const [upload, setUpload] = useState<UploadState | null>(null);
    const xhrRef = useRef<XMLHttpRequest | null>(null);

    const refresh = useCallback(async () => {
        try {
            const r = await jsonFetch<{overlays: ImageryOverlay[]}>("/imagery");
            setOverlays(r.overlays ?? []);
        } catch {
            /* robot unreachable — keep the last list */
        } finally {
            setLoaded(true);
        }
    }, []);

    useEffect(() => {
        if (enabled) void refresh();
    }, [enabled, refresh]);

    const processing = overlays.some((o) => o.status === "processing");
    useEffect(() => {
        if (!enabled || !processing) return;
        const id = setInterval(() => { void refresh(); }, 1500);
        return () => clearInterval(id);
    }, [enabled, processing, refresh]);

    const update = useCallback(async (name: string, patch: ImageryPatch) => {
        // Optimistic so sliders feel immediate.
        setOverlays((l) => l.map((o) => o.name === name ? {...o, ...patch} : o));
        try {
            const o = await jsonFetch<ImageryOverlay>(`/imagery/${encodeURIComponent(name)}`, {method: "PATCH", body: JSON.stringify(patch)});
            setOverlays((l) => l.map((x) => x.name === name ? o : x));
            return o;
        } catch (e) {
            void refresh();
            throw e;
        }
    }, [refresh]);

    const reorder = useCallback(async (names: string[]) => {
        setOverlays((l) => names.map((n) => l.find((o) => o.name === n)).filter((o): o is ImageryOverlay => !!o));
        const r = await jsonFetch<{overlays: ImageryOverlay[]}>("/imagery-order", {method: "POST", body: JSON.stringify({names})});
        setOverlays(r.overlays ?? []);
    }, []);

    const remove = useCallback(async (name: string) => {
        await jsonFetch(`/imagery/${encodeURIComponent(name)}`, {method: "DELETE"});
        setOverlays((l) => l.filter((o) => o.name !== name));
    }, []);

    const uploadFile = useCallback((file: File, opts: {label?: string; scheme?: string} = {}) => {
        return new Promise<ImageryOverlay>((resolve, reject) => {
            const fd = new FormData();
            if (opts.label) fd.append("label", opts.label);
            if (opts.scheme) fd.append("scheme", opts.scheme);
            fd.append("file", file); // last: the server reads fields before the file
            const xhr = new XMLHttpRequest();
            xhrRef.current = xhr;
            xhr.open("POST", `${httpBase()}${BASE}/imagery`);
            setUpload({fileName: file.name, progress: 0});
            xhr.upload.onprogress = (ev) => {
                if (ev.lengthComputable) setUpload({fileName: file.name, progress: ev.loaded / ev.total});
            };
            xhr.onload = () => {
                xhrRef.current = null;
                setUpload(null);
                let body: {error?: string} & Partial<ImageryOverlay> = {};
                try { body = JSON.parse(xhr.responseText || "{}") as typeof body; } catch { /* not json */ }
                if (xhr.status >= 200 && xhr.status < 300) {
                    setOverlays((l) => [...l, body as ImageryOverlay]);
                    resolve(body as ImageryOverlay);
                } else {
                    reject(new Error(body.error ?? `HTTP ${xhr.status}`));
                }
            };
            xhr.onerror = () => { xhrRef.current = null; setUpload(null); reject(new Error("network error")); };
            xhr.onabort = () => { xhrRef.current = null; setUpload(null); reject(new Error("cancelled")); };
            xhr.send(fd);
        });
    }, []);

    const cancelUpload = useCallback(() => { xhrRef.current?.abort(); }, []);

    return {overlays, loaded, refresh, update, reorder, remove, uploadFile, upload, cancelUpload};
}

export type ImageryApi = ReturnType<typeof useImagery>;
