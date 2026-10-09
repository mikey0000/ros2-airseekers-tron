// Per-area mowing settings ("area settings"): the contract shared with the map
// server (GET/PUT /api/mowglinext/areas/:index/settings, latched
// /map_server_node/area_settings, /behavior_tree_node/active_area_settings).
// The backend validates the same ranges; these mirror pkg/api/area_settings.go.

export const PATH_MODES = ["zigzag", "cross", "alternate", "spiral", "contour_only"] as const;
export type PathMode = typeof PATH_MODES[number];
/** Swath visiting order (Fields2Cover route planner). */
export const ROUTE_ORDERS = ["boustrophedon", "snake", "spiral", "racetrack"] as const;
export type RouteOrder = typeof ROUTE_ORDERS[number];
/** Swath-to-swath turn: auto = loop, else reverse-then-curve, else pivot. */
export const TURN_TYPES = ["auto", "loop", "reverse", "pivot"] as const;
export type TurnType = typeof TURN_TYPES[number];
/** Obstacle detection sensitivity: none = bumper only. */
export const OBSTACLE_DETECTIONS = ["none", "standard", "sensitive"] as const;
export type ObstacleDetection = typeof OBSTACLE_DETECTIONS[number];
export const BLADE_POLICIES = ["continuous", "conservative"] as const;
export type BladePolicy = typeof BLADE_POLICIES[number];
export const TRANSIT_VARIATIONS = ["none", "lanes", "perimeter", "mixed"] as const;
export type TransitVariation = typeof TRANSIT_VARIATIONS[number];
/** Slope-aware mow angle (terrain memory; only used when mow_angle_deg is auto). */
export const SLOPE_MODES = ["off", "auto", "contour", "updown"] as const;
export type SlopeMode = typeof SLOPE_MODES[number];

export type AreaSettings = {
    cutter_height_mm: number;
    perimeter_laps: number;
    path_mode: PathMode;
    /** -1 = automatic (planner picks the longest-edge angle). */
    mow_angle_deg: number;
    cut_speed_mps: number;
    swath_overlap_m: number;
    /** Path (swath) spacing in metres, used directly by the planner. */
    swath_width_m: number;
    /** Blade edge distance inside the recorded boundary, metres. */
    edge_margin_m: number;
    edge_first: boolean;
    repeat: number;
    alternate_angle_offset_deg: number;
    route_order: RouteOrder;
    route_spiral_size: number;
    /** 0 = pivot in place between swaths. */
    min_turn_radius_m: number;
    turn_type: TurnType;
    obstacle_detection: ObstacleDetection;
    blade_policy: BladePolicy;
    transit_variation: TransitVariation;
    slope_mode: SlopeMode;
    /** Contour (across the slope) above this slope in "auto" slope mode, degrees. */
    slope_contour_above_deg: number;
};

/**
 * Read-only values the map server derives from terrain memory and adds to a
 * GET response. Never sent back (sanitizeAreaSettings drops them; the backend
 * also strips them from a PUT).
 */
export type AreaSlopeDerived = {
    slope_mow_angle_deg: number | null;
    slope_angle_why: string;
};

export function extractSlopeDerived(raw: unknown): AreaSlopeDerived | null {
    if (!raw || typeof raw !== "object") return null;
    const r = raw as Record<string, unknown>;
    if (!("slope_mow_angle_deg" in r) && !("slope_angle_why" in r)) return null;
    const a = r.slope_mow_angle_deg;
    return {
        slope_mow_angle_deg: typeof a === "number" && Number.isFinite(a) ? a : null,
        slope_angle_why: typeof r.slope_angle_why === "string" ? r.slope_angle_why : "",
    };
}

export type AreaSettingsKey = keyof AreaSettings;

export const AREA_SETTINGS_DEFAULTS: AreaSettings = {
    cutter_height_mm: 50,
    perimeter_laps: 2,
    path_mode: "zigzag",
    mow_angle_deg: -1,
    cut_speed_mps: 0.3,
    swath_overlap_m: 0.02,
    swath_width_m: 0.18,
    edge_margin_m: 0.05,
    edge_first: true,
    repeat: 1,
    alternate_angle_offset_deg: 90,
    route_order: "racetrack",
    route_spiral_size: 6,
    min_turn_radius_m: 0.5,
    turn_type: "auto",
    obstacle_detection: "standard",
    blade_policy: "continuous",
    transit_variation: "lanes",
    slope_mode: "off",
    slope_contour_above_deg: 10,
};

export const AREA_SETTINGS_RANGES = {
    cutter_height_mm: {min: 30, max: 90, step: 5},
    perimeter_laps: {min: 0, max: 4, step: 1},
    cut_speed_mps: {min: 0.1, max: 0.5, step: 0.05},
    swath_overlap_m: {min: 0, max: 0.1, step: 0.01},
    swath_width_m: {min: 0.10, max: 0.40, step: 0.01},
    edge_margin_m: {min: 0, max: 0.5, step: 0.01},
    repeat: {min: 1, max: 5, step: 1},
    alternate_angle_offset_deg: {min: 0, max: 180, step: 5},
    route_spiral_size: {min: 2, max: 20, step: 1},
    min_turn_radius_m: {min: 0, max: 2, step: 0.05},
    slope_contour_above_deg: {min: 2, max: 30, step: 0.5},
} as const;

export const MOW_ANGLE_AUTO = -1;
/** Area index addressing the robot-wide defaults. */
export const DEFAULTS_INDEX = 255;

const KEYS = Object.keys(AREA_SETTINGS_DEFAULTS) as AreaSettingsKey[];

/** Keep only known keys with a plausible type; anything else is dropped. */
export function sanitizeAreaSettings(raw: unknown): Partial<AreaSettings> {
    const out: Partial<AreaSettings> = {};
    if (!raw || typeof raw !== "object") return out;
    const r = raw as Record<string, unknown>;
    for (const k of KEYS) {
        const v = r[k];
        const want = typeof AREA_SETTINGS_DEFAULTS[k];
        if (typeof v !== want) continue;
        if (k === "path_mode" && !PATH_MODES.includes(v as PathMode)) continue;
        if (k === "route_order" && !ROUTE_ORDERS.includes(v as RouteOrder)) continue;
        if (k === "turn_type" && !TURN_TYPES.includes(v as TurnType)) continue;
        if (k === "obstacle_detection" && !OBSTACLE_DETECTIONS.includes(v as ObstacleDetection)) continue;
        if (k === "blade_policy" && !BLADE_POLICIES.includes(v as BladePolicy)) continue;
        if (k === "transit_variation" && !TRANSIT_VARIATIONS.includes(v as TransitVariation)) continue;
        if (k === "slope_mode" && !SLOPE_MODES.includes(v as SlopeMode)) continue;
        (out as Record<string, unknown>)[k] = v;
    }
    return out;
}

/** Effective settings: built-in defaults < robot defaults < area overrides. */
export function effectiveAreaSettings(
    defaults: Partial<AreaSettings> | null | undefined,
    overrides?: Partial<AreaSettings> | null,
): AreaSettings {
    return {...AREA_SETTINGS_DEFAULTS, ...sanitizeAreaSettings(defaults), ...sanitizeAreaSettings(overrides)};
}

/** Client-side mirror of the backend validation. Returns the offending key. */
export function invalidAreaSettingKey(s: Partial<AreaSettings>): AreaSettingsKey | null {
    for (const [k, v] of Object.entries(s) as [AreaSettingsKey, unknown][]) {
        if (k === "path_mode") {
            if (!PATH_MODES.includes(v as PathMode)) return k;
        } else if (k === "route_order") {
            if (!ROUTE_ORDERS.includes(v as RouteOrder)) return k;
        } else if (k === "turn_type") {
            if (!TURN_TYPES.includes(v as TurnType)) return k;
        } else if (k === "obstacle_detection") {
            if (!OBSTACLE_DETECTIONS.includes(v as ObstacleDetection)) return k;
        } else if (k === "blade_policy") {
            if (!BLADE_POLICIES.includes(v as BladePolicy)) return k;
        } else if (k === "transit_variation") {
            if (!TRANSIT_VARIATIONS.includes(v as TransitVariation)) return k;
        } else if (k === "slope_mode") {
            if (!SLOPE_MODES.includes(v as SlopeMode)) return k;
        } else if (k === "edge_first") {
            if (typeof v !== "boolean") return k;
        } else if (k === "mow_angle_deg") {
            if (typeof v !== "number" || !(v === MOW_ANGLE_AUTO || (v >= 0 && v <= 360))) return k;
        } else if (k in AREA_SETTINGS_RANGES) {
            const r = AREA_SETTINGS_RANGES[k as keyof typeof AREA_SETTINGS_RANGES];
            if (typeof v !== "number" || v < r.min || v > r.max) return k;
            if (r.step === 1 || k === "cutter_height_mm") {
                if (!Number.isInteger(v)) return k;
            }
        } else {
            return k;
        }
    }
    return null;
}

/** Parse a std_msgs/String JSON payload ({data:"..."}) into an object. */
export function parseStringMsgJson(raw: unknown): Record<string, unknown> | undefined {
    const data = (raw as {data?: unknown} | null | undefined)?.data;
    if (typeof data !== "string" || data.trim() === "") return undefined;
    try {
        const parsed = JSON.parse(data);
        return parsed && typeof parsed === "object" ? parsed as Record<string, unknown> : undefined;
    } catch {
        return undefined;
    }
}

/** Normalise a compass angle to [0, 360). */
export function normaliseAngle(deg: number): number {
    const n = deg % 360;
    return n < 0 ? n + 360 : n;
}

/** Angle (deg, 0 = up/north, clockwise) of a point relative to a centre. */
export function angleFromPoint(dx: number, dy: number): number {
    return normaliseAngle(Math.round(Math.atan2(dx, -dy) * 180 / Math.PI));
}

/**
 * Merge patch saving `draft` as an area's settings: keys that differ from the
 * defaults are overrides, keys equal to the defaults are reset (null) so the
 * area keeps following the defaults for them.
 */
export function buildAreaPatch(
    draft: AreaSettings, defaults: AreaSettings,
): {[K in AreaSettingsKey]?: AreaSettings[K] | null} {
    const out: Record<string, unknown> = {};
    for (const k of KEYS) out[k] = draft[k] === defaults[k] ? null : draft[k];
    return out as {[K in AreaSettingsKey]?: AreaSettings[K] | null};
}

/**
 * Body for saving the robot-wide area defaults (index 255). Every key is sent
 * as-is EXCEPT swath_width_m: unchanged from what was loaded -> left out (so a
 * defaults save never pins the disc-derived standard width into
 * area_settings.yaml); changed back to the standard width -> null (follow the
 * blade-disc setting again).
 */
export function buildDefaultsBody(
    draft: AreaSettings, loaded: AreaSettings, standardSwathWidth?: number,
): {[K in AreaSettingsKey]?: AreaSettings[K] | null} {
    const out: Record<string, unknown> = {...draft};
    if (draft.swath_width_m === loaded.swath_width_m) delete out.swath_width_m;
    else if (standardSwathWidth !== undefined && Math.abs(draft.swath_width_m - standardSwathWidth) < 1e-6)
        out.swath_width_m = null;
    return out as {[K in AreaSettingsKey]?: AreaSettings[K] | null};
}

/**
 * Keys an area overrides. The latched topic's per-area object is
 * authoritative; without it, fall back to "differs from the defaults".
 */
export function overriddenKeys(
    effective: AreaSettings, defaults: AreaSettings, topicOverrides?: Partial<AreaSettings>,
): Set<AreaSettingsKey> {
    if (topicOverrides) return new Set(Object.keys(sanitizeAreaSettings(topicOverrides)) as AreaSettingsKey[]);
    return new Set(KEYS.filter((k) => effective[k] !== defaults[k]));
}
