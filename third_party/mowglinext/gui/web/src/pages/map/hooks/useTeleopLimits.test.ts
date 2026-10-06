import {describe, it, expect} from "vitest";
import {relayCapsFromParams, resolveTeleopLimits} from "./useTeleopLimits.ts";
import {DEFAULT_TELEOP} from "../../../constants/robotProfiles.ts";

describe("teleop limits", () => {
    it("reads the relay clamps from the /api/params list", () => {
        expect(relayCapsFromParams([
            {name: "/cmd_vel_ws_relay.max_linear", value: 0.5},
            {name: "/cmd_vel_ws_relay.max_angular", value: 1.0},
            {name: "/cmd_vel_ws_relay.port", value: 8766},
        ])).toEqual({maxLinear: 0.5, maxAngular: 1.0});
        expect(relayCapsFromParams(undefined)).toEqual({});
        expect(relayCapsFromParams([{name: "/other.max_linear", value: 0.1}])).toEqual({});
        // Non-positive / non-numeric clamps are ignored.
        expect(relayCapsFromParams([{name: "/cmd_vel_ws_relay.max_linear", value: 0}]).maxLinear).toBeUndefined();
    });

    it("uses the profile when no relay is reported", () => {
        expect(resolveTeleopLimits(DEFAULT_TELEOP, {})).toEqual(DEFAULT_TELEOP);
    });

    it("never exceeds the relay clamp", () => {
        expect(resolveTeleopLimits({maxLinear: 0.8, maxAngular: 1.5}, {maxLinear: 0.5, maxAngular: 1.0}))
            .toEqual({maxLinear: 0.5, maxAngular: 1.0});
        expect(resolveTeleopLimits({maxLinear: 0.25, maxAngular: 0.6}, {maxLinear: 0.5, maxAngular: 1.0}))
            .toEqual({maxLinear: 0.25, maxAngular: 0.6});
    });
});
