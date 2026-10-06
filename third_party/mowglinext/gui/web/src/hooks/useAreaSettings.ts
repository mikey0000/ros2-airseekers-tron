import {useCallback, useEffect, useState, useSyncExternalStore} from "react";
import {Api, ContentType} from "../api/Api.ts";
import {useApi} from "./useApi.ts";
import {useGate} from "./useProfileGates.ts";
import {useTopic} from "./useTopic.ts";
import {
    type AreaSettings,
    DEFAULTS_INDEX,
    parseStringMsgJson,
    sanitizeAreaSettings,
} from "../utils/areaSettings.ts";

/** Merge patch: a key set to null resets it to the default. */
export type AreaSettingsPatch = {[K in keyof AreaSettings]?: AreaSettings[K] | null};

/** "defaults" or a 0-based map-server area index. */
export type AreaSettingsTarget = number | "defaults";

export type AreaSettingsResult = {
    supported: boolean;
    settings: Partial<AreaSettings>;
};

export class AreaSettingsUnsupportedError extends Error {
    constructor() {
        super("area settings not supported by this robot");
    }
}

const pathFor = (target: AreaSettingsTarget) =>
    `/mowglinext/areas/${target === "defaults" || target === DEFAULTS_INDEX ? "defaults" : target}/settings`;

type HttpLike = {status?: number; error?: {error?: string} | null; data?: unknown};

function errorMessage(e: unknown): string {
    const h = e as HttpLike;
    return h?.error?.error ?? (e instanceof Error ? e.message : String(e));
}

/** GET; resolves {supported:false} when the map server lacks the service (501). */
export async function fetchAreaSettings(api: Api<unknown>, target: AreaSettingsTarget): Promise<AreaSettingsResult> {
    try {
        const res = await api.request<{supported: boolean; settings: unknown}>({
            path: pathFor(target), method: "GET", format: "json",
        });
        return {supported: res.data?.supported !== false, settings: sanitizeAreaSettings(res.data?.settings)};
    } catch (e) {
        if ((e as HttpLike)?.status === 501) return {supported: false, settings: {}};
        throw new Error(errorMessage(e));
    }
}

/**
 * PUT; `settings: null` means "use the defaults" (clears every override).
 * Throws AreaSettingsUnsupportedError on 501.
 */
export async function saveAreaSettings(
    api: Api<unknown>, target: AreaSettingsTarget, settings: AreaSettingsPatch | null,
): Promise<void> {
    try {
        await api.request({
            path: pathFor(target), method: "PUT", format: "json", type: ContentType.Json,
            body: settings === null ? {use_defaults: true} : {settings},
        });
    } catch (e) {
        if ((e as HttpLike)?.status === 501) throw new AreaSettingsUnsupportedError();
        throw new Error(errorMessage(e));
    }
}

// ── Support probe ─────────────────────────────────────────────────────────
// One GET of the defaults per page load answers "does this robot's map server
// implement area settings?". null = not known yet.
let probe: {supported: boolean | null; inflight: Promise<void> | null} = {supported: null, inflight: null};
const listeners = new Set<() => void>();

function runProbe(api: Api<unknown>) {
    if (probe.inflight) return;
    probe.inflight = (async () => {
        let supported = false;
        try {
            supported = (await fetchAreaSettings(api, "defaults")).supported;
        } catch {
            // Bridge offline / transient error: treat as unsupported until retried.
            supported = false;
            setTimeout(() => {
                probe = {supported: probe.supported, inflight: null};
            }, 10000);
        }
        probe = {...probe, supported};
        listeners.forEach((l) => l());
    })();
}

/** Test hook. */
export function resetAreaSettingsProbe(supported: boolean | null = null) {
    probe = {supported, inflight: supported === null ? null : Promise.resolve()};
    listeners.forEach((l) => l());
}

export type AreaSettingsSupport = {
    /** The robot profile enables the feature (UI may render). */
    enabled: boolean;
    /** The map server answered; null while probing. */
    supported: boolean | null;
};

/** Profile gate + live service probe for the area-settings feature. */
export function useAreaSettingsSupport(): AreaSettingsSupport {
    const api = useApi();
    const enabled = useGate("feature:area_settings");
    const supported = useSyncExternalStore(
        (l) => {
            listeners.add(l);
            return () => listeners.delete(l);
        },
        () => probe.supported,
    );
    useEffect(() => {
        if (enabled) runProbe(api);
    }, [enabled, api]);
    return {enabled, supported: enabled ? supported : false};
}

/**
 * Loads the effective (fully merged) settings of one target plus the
 * effective robot defaults. Which keys an area overrides comes from the
 * latched topic (see useAreaOverrides).
 */
export function useAreaSettings(target: AreaSettingsTarget | null) {
    const api = useApi();
    const [state, setState] = useState<{
        loading: boolean; supported: boolean; effective: Partial<AreaSettings>;
        defaults: Partial<AreaSettings>; error?: string;
    }>({loading: true, supported: true, effective: {}, defaults: {}});
    const [nonce, setNonce] = useState(0);

    useEffect(() => {
        if (target === null) return;
        let cancelled = false;
        setState((s) => ({...s, loading: true, error: undefined}));
        (async () => {
            try {
                const defaults = await fetchAreaSettings(api, "defaults");
                const own = target === "defaults" || !defaults.supported ? defaults : await fetchAreaSettings(api, target);
                if (cancelled) return;
                setState({
                    loading: false,
                    supported: defaults.supported && own.supported,
                    defaults: defaults.settings,
                    effective: own.settings,
                });
            } catch (e) {
                if (!cancelled) setState((s) => ({...s, loading: false, error: errorMessage(e)}));
            }
        })();
        return () => {
            cancelled = true;
        };
    }, [api, target, nonce]);

    const reload = useCallback(() => setNonce((n) => n + 1), []);
    return {...state, reload};
}

type AreaSettingsTopic = {defaults: Partial<AreaSettings>; areas: Record<string, Partial<AreaSettings>>};

/** Latched /map_server_node/area_settings: full defaults + per-area overrides by name. */
export function useAreaSettingsTopic(enabled = true): AreaSettingsTopic | null {
    return useTopic<AreaSettingsTopic | null>("areaSettings", null, {
        enabled,
        select: (raw) => {
            const d = parseStringMsgJson(raw);
            if (!d) return undefined;
            const areas: Record<string, Partial<AreaSettings>> = {};
            const rawAreas = d.areas && typeof d.areas === "object" ? d.areas as Record<string, unknown> : {};
            for (const [name, v] of Object.entries(rawAreas)) areas[name] = sanitizeAreaSettings(v);
            return {defaults: sanitizeAreaSettings(d.defaults), areas};
        },
    }).data;
}

export type ActiveAreaSettings = Partial<AreaSettings> & {
    area?: string;
    areaIndex?: number;
    run?: number;
    runs?: number;
    runMowAngleDeg?: number;
};

/** Effective settings the mission is mowing with right now (null when idle: {}). */
export function useActiveAreaSettings(enabled = true): ActiveAreaSettings | null {
    const {data} = useTopic<Record<string, unknown> | null>("activeAreaSettings", null, {
        select: (raw) => parseStringMsgJson(raw) ?? null,
        enabled,
    });
    return enabled ? parseActiveAreaSettings(data) : null;
}

export function parseActiveAreaSettings(data: Record<string, unknown> | null | undefined): ActiveAreaSettings | null {
    if (!data || Object.keys(data).length === 0) return null;
    const num = (v: unknown) => (typeof v === "number" && Number.isFinite(v) ? v : undefined);
    return {
        ...sanitizeAreaSettings(data),
        area: typeof data.area_name === "string" && data.area_name ? data.area_name : undefined,
        areaIndex: num(data.area_index),
        run: num(data.run),
        runs: num(data.runs),
        runMowAngleDeg: num(data.run_mow_angle_deg),
    };
}
