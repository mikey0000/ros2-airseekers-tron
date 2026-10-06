import {useCallback, useEffect, useState} from "react";
import type {CameraInfo} from "../../../hooks/useCameras.ts";

/** Persisted state of the manual-drive camera PiP. */
export interface DrivePipPrefs {
    /** Camera picked by the operator; null = the profile's driving camera. */
    cameraId: string | null;
    autoReverse: boolean;
    annotated: boolean;
    collapsed: boolean;
    driveView: boolean;
    /** Panel position/size in px relative to the map container; null = default. */
    rect: { x: number; y: number; w: number; h: number } | null;
}

export const DRIVE_PIP_STORAGE_KEY = "mowgli.drivePip";
export const DEFAULT_PREFS: DrivePipPrefs = {
    cameraId: null, autoReverse: true, annotated: false, collapsed: false, driveView: false, rect: null,
};

export function loadPrefs(): DrivePipPrefs {
    try {
        const raw = window.localStorage.getItem(DRIVE_PIP_STORAGE_KEY);
        if (!raw) return {...DEFAULT_PREFS};
        const p = JSON.parse(raw) as Partial<DrivePipPrefs>;
        return {...DEFAULT_PREFS, ...(p && typeof p === "object" ? p : {})};
    } catch {
        return {...DEFAULT_PREFS};
    }
}

export function savePrefs(p: DrivePipPrefs) {
    try {
        window.localStorage.setItem(DRIVE_PIP_STORAGE_KEY, JSON.stringify(p));
    } catch {
        // private window / blocked storage: prefs are a convenience only
    }
}

/** The forward camera: profile drivingCamera if listed, else any "front" camera, else the first. */
export function defaultDrivingCamera(cameras: readonly CameraInfo[], drivingCamera?: string): CameraInfo | undefined {
    if (cameras.length === 0) return undefined;
    return cameras.find((c) => c.id === drivingCamera)
        ?? cameras.find((c) => c.id.includes("front"))
        ?? cameras[0];
}

/**
 * Which camera the PiP shows: the reverse camera while reversing (when
 * auto-reverse is on and it exists), else the operator's pick if still
 * listed, else the default driving camera.
 */
export function chooseCamera(cameras: readonly CameraInfo[], opts: {
    selectedId: string | null; drivingCamera?: string; reverseCamera?: string;
    reversing: boolean; autoReverse: boolean;
}): CameraInfo | undefined {
    if (opts.reversing && opts.autoReverse) {
        const rear = cameras.find((c) => c.id === (opts.reverseCamera ?? "rear"));
        if (rear) return rear;
    }
    return cameras.find((c) => c.id === opts.selectedId) ?? defaultDrivingCamera(cameras, opts.drivingCamera);
}

/** Joystick y < -deadband means driving backwards. */
export const REVERSE_DEADBAND = 0.15;
export function isReversing(y: number | null | undefined): boolean {
    return (y ?? 0) < -REVERSE_DEADBAND;
}

/** Persisted PiP prefs; every update is written back to localStorage. */
export function useDrivePipPrefs() {
    const [prefs, setPrefs] = useState<DrivePipPrefs>(loadPrefs);
    useEffect(() => savePrefs(prefs), [prefs]);
    const update = useCallback((patch: Partial<DrivePipPrefs>) => setPrefs((p) => ({...p, ...patch})), []);
    return {prefs, update};
}
