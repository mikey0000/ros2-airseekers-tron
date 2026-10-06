import {describe, expect, it} from "vitest";
import {AIRSEEKERS_TRON_PROFILE_ID, getRobotProfile} from "../constants/robotProfiles.ts";
import {filterNavItems, navItemsFor} from "./navItems.ts";

// The side-rail before robot profiles existed (AppShell NAV), in order.
const STOCK_NAV = [
    "/mowglinext", "/map", "/schedule", "/diagnostics", "/statistics",
    "/settings", "/parameters", "/logs", "/onboarding",
];

describe("nav items per robot profile", () => {
    it("YardForce500 shows exactly today's nav", () => {
        expect(navItemsFor(getRobotProfile("YardForce500")).map((n) => n.key)).toEqual(STOCK_NAV);
    });

    it("an unknown robot falls back to the stock nav", () => {
        expect(navItemsFor(getRobotProfile("NoSuchRobot")).map((n) => n.key)).toEqual(STOCK_NAV);
    });

    it("Tron keeps onboarding and gains Perception", () => {
        expect(navItemsFor(getRobotProfile(AIRSEEKERS_TRON_PROFILE_ID)).map((n) => n.key)).toEqual([
            "/mowglinext", "/map", "/schedule", "/diagnostics", "/perception", "/statistics",
            "/settings", "/parameters", "/logs", "/onboarding",
        ]);
    });

    it("shows no gated entry while the profile is loading", () => {
        const keys = filterNavItems(() => false).map((n) => n.key);
        expect(keys).toContain("/onboarding"); // ungated: profile-aware page
        expect(keys).not.toContain("/perception");
        expect(keys).toContain("/settings");
    });

    it("keeps the bottom-nav set unchanged", () => {
        expect(navItemsFor(getRobotProfile("YardForce500")).filter((n) => n.showInBottom).map((n) => n.key))
            .toEqual(["/mowglinext", "/map", "/schedule", "/diagnostics"]);
    });
});
