import {useEffect, useSyncExternalStore} from "react";
import {useApi} from "./useApi.ts";
import type {Api} from "../api/Api.ts";
import {
    type FeatureId,
    type RobotProfile,
    findRobotProfile,
    getRobotProfile,
    hasFeature,
} from "../constants/robotProfiles.ts";

/**
 * Where the active profile id came from:
 *  - "override": the caller passed a model (e.g. an unsaved picker choice)
 *  - "settings": `mower_model` set explicitly in mowgli_robot.yaml
 *  - "env":      the backend's ROBOT_PROFILE env fallback
 *  - "default":  nothing set; the schema default model
 *  - "fallback": the backend was unreachable or named an unknown profile
 */
export type RobotProfileSource = "override" | "settings" | "env" | "default" | "fallback";

/** Body of GET /api/robot/profile. */
export type RobotProfileResponse = {
    id: string;
    source: "settings" | "env" | "default";
};

export type ResolvedRobotProfile = {
    profile: RobotProfile;
    source: RobotProfileSource;
    /** True until the backend has answered (the stock profile is used meanwhile). */
    loading: boolean;
};

// One request per page load, shared by every consumer. The profile only
// changes when mowgli_robot.yaml or the container env changes, so callers that
// save a new mower_model call refreshRobotProfile().
type Snapshot = {response: RobotProfileResponse | null; loading: boolean};
let snapshot: Snapshot = {response: null, loading: true};
let inflight: Promise<void> | null = null;
const listeners = new Set<() => void>();

function publish(next: Snapshot) {
    snapshot = next;
    listeners.forEach((l) => l());
}

function isProfileResponse(data: unknown): data is RobotProfileResponse {
    const d = data as RobotProfileResponse | null;
    return !!d && typeof d.id === "string" && typeof d.source === "string";
}

function load(api: Api<unknown>): Promise<void> {
    if (!inflight) {
        inflight = (async () => {
            let response: RobotProfileResponse | null = null;
            try {
                const res = await api.request<RobotProfileResponse>({
                    path: "/robot/profile",
                    method: "GET",
                    format: "json",
                });
                if (isProfileResponse(res.data)) response = res.data;
            } catch {
                // Older backend without the route, or offline: stock profile.
            }
            publish({response, loading: false});
        })();
    }
    return inflight;
}

/** Drop the cached answer and ask the backend again (after saving mower_model). */
export function refreshRobotProfile(api: Api<unknown>): Promise<void> {
    inflight = null;
    publish({...snapshot, loading: true});
    return load(api);
}

/** Test hook: forget everything, as on a fresh page load. */
export function resetRobotProfileCache() {
    inflight = null;
    snapshot = {response: null, loading: true};
    listeners.clear();
}

function subscribe(listener: () => void) {
    listeners.add(listener);
    return () => listeners.delete(listener);
}

/**
 * Pure resolution rule, exported for tests: an explicit override wins, then
 * the backend's answer (which already applies yaml > ROBOT_PROFILE env >
 * schema default), then the stock profile.
 */
export function resolveRobotProfile(
    override: string | null | undefined,
    response: RobotProfileResponse | null,
): {profile: RobotProfile; source: RobotProfileSource} {
    const fromOverride = findRobotProfile(override);
    if (fromOverride) return {profile: fromOverride, source: "override"};
    const fromBackend = findRobotProfile(response?.id);
    if (fromBackend && response) return {profile: fromBackend, source: response.source};
    return {profile: getRobotProfile(undefined), source: "fallback"};
}

/**
 * The active robot profile.
 *
 * @param override a model id that should win over the saved one, e.g. the
 *   picker's unsaved selection. Do NOT pass `settings.mower_model` from
 *   GET /settings/yaml as-is: that map is filled with the schema default
 *   ("YardForce500") when the yaml does not set the key, which would mask the
 *   ROBOT_PROFILE env fallback. The backend route already reads the yaml.
 */
export function useRobotProfile(override?: string | null): ResolvedRobotProfile {
    const guiApi = useApi();
    const current = useSyncExternalStore(subscribe, () => snapshot);
    useEffect(() => {
        void load(guiApi);
    }, [guiApi]);
    const {profile, source} = resolveRobotProfile(override, current.response);
    return {profile, source, loading: current.loading && source !== "override"};
}

/** `true` when the active robot has `feature`. */
export function useFeature(feature: FeatureId): boolean {
    return hasFeature(useRobotProfile().profile, feature);
}
