// Settings -> ROS parameter bindings, per robot profile (frontend mirror).
//
// The backend owns the full table (pkg/api/param_bindings.go: node, param,
// type, value mapping) and pushes bound keys to the running nodes after a
// save. This file mirrors only what the forms need synchronously: which keys
// a profile binds and which other keys it still uses. The Go test
// TestParamBindingsFrontendParity parses this file, so keep the literal shape
// (one quoted key per array entry) when editing.
//
// A profile without an entry consumes mowgli_robot.yaml directly (every stock
// MowgliNext robot): every key is "used", nothing is flagged.

import type {RobotProfile} from "./robotProfiles.ts";

export type ParamBindingProfile = {
    /** Settings keys pushed to a ROS parameter of this robot. */
    boundKeys: readonly string[];
    /** Unbound keys that still matter (read by the GUI or straight from the yaml). */
    guiKeys: readonly string[];
};

export const PARAM_BINDING_PROFILES: Readonly<Record<string, ParamBindingProfile>> = {
    AirseekersTron: {
        boundKeys: [
            "tool_width",
            "mowing_speed",
            "transit_speed",
            "undock_distance",
            "undock_speed",
            "battery_low_percent",
            "battery_full_percent",
            "rain_mode",
            "rain_delay_minutes",
            "rain_debounce_sec",
            "mow_angle_deg",
            "dock_approach_distance",
            "dock_max_retries",
            "ntrip_host",
            "ntrip_port",
            "ntrip_mountpoint",
            "ntrip_user",
            "ntrip_password",
            "ntrip_enabled",
            "angular_trim_radps",
            "angular_deadband_radps",
            "heading_hold",
            "heading_hold_kp",
            "heading_hold_kd",
            "heading_hold_max_radps",
        ],
        guiKeys: [
            "mower_model",
            "datum_lat",
            "datum_lon",
            "datum_alt",
            "dock_pose_x",
            "dock_pose_y",
            "dock_pose_yaw",
            "camera_stream_base_url",
        ],
    },
};

type ProfileRef = Pick<RobotProfile, "value"> | string | null | undefined;

function profileId(profile: ProfileRef): string | undefined {
    return typeof profile === "string" ? profile : profile?.value;
}

/** The profile's binding table; undefined for a robot that reads the yaml itself. */
export function getParamBindingProfile(profile: ProfileRef): ParamBindingProfile | undefined {
    const id = profileId(profile);
    return id ? PARAM_BINDING_PROFILES[id] : undefined;
}

/** True when the profile has a binding table (its stack does not read mowgli_robot.yaml). */
export function hasParamBindings(profile: ProfileRef): boolean {
    return getParamBindingProfile(profile) !== undefined;
}

/** True when saving `key` pushes a ROS parameter on this robot. */
export function isKeyBound(profile: ProfileRef, key: string): boolean {
    return getParamBindingProfile(profile)?.boundKeys.includes(key) ?? false;
}

/**
 * False only for a key that has no effect on this robot: the profile has a
 * binding table and the key is neither bound nor read by the GUI. Stock
 * profiles always answer true.
 */
export function isKeyUsedByRobot(profile: ProfileRef, key: string): boolean {
    const table = getParamBindingProfile(profile);
    if (!table) return true;
    return table.boundKeys.includes(key) || table.guiKeys.includes(key);
}

/** One entry of `param_bindings` in the POST /settings/yaml response. */
export type ParamApplyStatus =
    | "applied" | "rejected" | "restart_required" | "reset" | "invalid" | "unavailable";

export type ParamApplyResult = {
    key: string;
    param: string;
    status: ParamApplyStatus;
    value?: unknown;
    message?: string;
};

/** Pull the per-key apply results out of a save response body (absent for stock robots). */
export function paramApplyResults(body: unknown): ParamApplyResult[] {
    const list = (body as {param_bindings?: unknown} | null)?.param_bindings;
    return Array.isArray(list) ? (list as ParamApplyResult[]) : [];
}

export type ParamApplySummary = {
    /** Running nodes use the new value now. */
    applied: ParamApplyResult[];
    /** Saved; the robot picks it up at its next restart. */
    onRestart: ParamApplyResult[];
    /** Saved to the yaml, but the live push failed (node down, value refused). */
    failed: ParamApplyResult[];
};

/** Group save results for a toast: live now / next restart / failed. */
export function summarizeParamApply(results: readonly ParamApplyResult[]): ParamApplySummary {
    const out: ParamApplySummary = {applied: [], onRestart: [], failed: []};
    for (const r of results) {
        if (r.status === "applied") out.applied.push(r);
        else if (r.status === "restart_required" || r.status === "reset") out.onRestart.push(r);
        else out.failed.push(r);
    }
    return out;
}
