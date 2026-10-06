import {useEffect, useState} from "react";
import {useApi} from "../../../hooks/useApi.ts";
import {useRobotProfile} from "../../../hooks/useRobotProfile.ts";
import type {Api} from "../../../api/Api.ts";
import type {RobotTeleop} from "../../../constants/robotProfiles.ts";

/**
 * Joystick velocity caps. The robot profile says how fast manual driving
 * should go (`profile.teleop`); a teleop relay that clamps on the robot side
 * (Tron: /cmd_vel_ws_relay max_linear / max_angular, read live via
 * GET /api/params) lowers them further so the joystick's full deflection never
 * asks for more than the relay will pass. Without a relay parameter the
 * profile alone applies.
 */

/** Parameter names (suffix after the node) of a robot-side teleop clamp. */
const RELAY_LINEAR = "cmd_vel_ws_relay.max_linear";
const RELAY_ANGULAR = "cmd_vel_ws_relay.max_angular";

export type RelayCaps = {maxLinear?: number; maxAngular?: number};

type RosParameter = {name: string; value: unknown};

function positive(value: unknown): number | undefined {
    return typeof value === "number" && Number.isFinite(value) && value > 0 ? value : undefined;
}

/** Pick the relay clamps out of a GET /api/params list. */
export function relayCapsFromParams(params: readonly RosParameter[] | undefined): RelayCaps {
    const caps: RelayCaps = {};
    for (const p of params ?? []) {
        const name = p.name.replace(/^\/+/, "");
        if (name === RELAY_LINEAR || name.endsWith("/" + RELAY_LINEAR)) caps.maxLinear = positive(p.value);
        if (name === RELAY_ANGULAR || name.endsWith("/" + RELAY_ANGULAR)) caps.maxAngular = positive(p.value);
    }
    return caps;
}

/** Profile limits, each capped by the relay's clamp when known. */
export function resolveTeleopLimits(profile: RobotTeleop, relay: RelayCaps): RobotTeleop {
    return {
        maxLinear: relay.maxLinear !== undefined ? Math.min(profile.maxLinear, relay.maxLinear) : profile.maxLinear,
        maxAngular: relay.maxAngular !== undefined ? Math.min(profile.maxAngular, relay.maxAngular) : profile.maxAngular,
    };
}

// One read per page load: the relay clamps are launch parameters.
let relayCache: Promise<RelayCaps> | null = null;

function loadRelayCaps(api: Api<unknown>): Promise<RelayCaps> {
    if (!relayCache) {
        relayCache = api
            .request<{parameters?: RosParameter[]}>({path: "/params", method: "GET", format: "json"})
            .then((res) => relayCapsFromParams(res.data?.parameters))
            .catch(() => {
                relayCache = null; // retry on the next mount
                return {};
            });
    }
    return relayCache;
}

/** Test hook: forget the cached relay read. */
export function resetTeleopLimitsCache() {
    relayCache = null;
}

export function useTeleopLimits(): RobotTeleop {
    const api = useApi();
    const {profile} = useRobotProfile();
    const [relay, setRelay] = useState<RelayCaps>({});
    useEffect(() => {
        let alive = true;
        void loadRelayCaps(api).then((caps) => {
            if (alive) setRelay(caps);
        });
        return () => {
            alive = false;
        };
    }, [api]);
    return resolveTeleopLimits(profile.teleop, relay);
}
