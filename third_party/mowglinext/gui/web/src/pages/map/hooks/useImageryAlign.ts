import {useCallback, useState} from "react";
import {itranspose} from "../../../utils/map.tsx";
import {
    defaultPlacement,
    mapToImage,
    solveTwoPointPlacement,
    type ImageryControlPoint,
    type ImageryOverlay,
    type ImageryPlacement,
    type XY,
} from "../../../utils/imagery.ts";
import type {ImageryPatch} from "../../../hooks/useImagery.ts";

export type CpDraft = Partial<ImageryControlPoint>;
export type PickMode = {index: 0 | 1; what: "image" | "target"} | null;

export interface AlignSession {
    name: string;
    width: number;
    height: number;
    placement: ImageryPlacement;
    opacity: number;
    cps: [CpDraft, CpDraft];
}

export function isCpComplete(c: CpDraft): c is ImageryControlPoint {
    return [c.u, c.v, c.x, c.y].every((v) => typeof v === "number" && Number.isFinite(v));
}

/**
 * Apply a control-point edit and, once both points have an image and a ground
 * position, solve the placement from them.
 */
export function applyCpEdit(s: AlignSession, index: 0 | 1, patch: CpDraft): AlignSession {
    const cps: [CpDraft, CpDraft] = [{...s.cps[0]}, {...s.cps[1]}];
    cps[index] = {...cps[index], ...patch};
    let placement = s.placement;
    if (isCpComplete(cps[0]) && isCpComplete(cps[1])) {
        placement = solveTwoPointPlacement(cps[0], cps[1], s.width, s.height) ?? placement;
    }
    return {...s, cps, placement};
}

export interface AlignDeps {
    offsetX: number;
    offsetY: number;
    datum: [number, number, number];
    update: (name: string, patch: ImageryPatch) => Promise<unknown>;
}

/** State machine of the plain-image alignment tool. */
export function useImageryAlign({offsetX, offsetY, datum, update}: AlignDeps) {
    const [session, setSession] = useState<AlignSession | null>(null);
    const [pick, setPick] = useState<PickMode>(null);
    const [saving, setSaving] = useState(false);

    const toMap = useCallback((lng: number, lat: number): XY => {
        const [x, y] = itranspose(offsetX, offsetY, datum, lat, lng);
        return [x, y];
    }, [offsetX, offsetY, datum]);

    const start = useCallback((o: ImageryOverlay, viewCenter: XY) => {
        if (o.kind !== "image" || !o.width || !o.height) return;
        const cps = (o.controlPoints ?? []).slice(0, 2);
        setSession({
            name: o.name,
            width: o.width,
            height: o.height,
            placement: o.placement ?? defaultPlacement(viewCenter),
            opacity: Math.min(o.opacity, 0.7),
            cps: [cps[0] ?? {}, cps[1] ?? {}],
        });
        setPick(null);
    }, []);

    const cancel = useCallback(() => { setSession(null); setPick(null); }, []);

    const save = useCallback(async () => {
        if (!session) return;
        setSaving(true);
        try {
            await update(session.name, {
                placement: session.placement,
                controlPoints: session.cps.filter(isCpComplete),
            });
            setSession(null);
            setPick(null);
        } finally {
            setSaving(false);
        }
    }, [session, update]);

    const setPlacement = useCallback((p: Partial<ImageryPlacement>) => {
        setSession((s) => s ? {...s, placement: {...s.placement, ...p}} : s);
    }, []);
    const setOpacity = useCallback((opacity: number) => setSession((s) => s ? {...s, opacity} : s), []);

    const setCp = useCallback((index: 0 | 1, patch: CpDraft) => {
        setSession((s) => s ? applyCpEdit(s, index, patch) : s);
    }, []);
    const clearCps = useCallback(() => setSession((s) => s ? {...s, cps: [{}, {}]} : s), []);

    /** Map click while picking. Returns true when consumed. */
    const handleMapClick = useCallback((lng: number, lat: number): boolean => {
        if (!session || !pick) return false;
        const [x, y] = toMap(lng, lat);
        if (pick.what === "image") {
            const [u, v] = mapToImage(session.placement, session.width, session.height, x, y);
            setCp(pick.index, {u, v});
        } else {
            setCp(pick.index, {x, y});
        }
        setPick(null);
        return true;
    }, [session, pick, toMap, setCp]);

    return {session, pick, setPick, start, cancel, save, saving, setPlacement, setOpacity, setCp, clearCps, handleMapClick, toMap};
}

export type ImageryAlignApi = ReturnType<typeof useImageryAlign>;

