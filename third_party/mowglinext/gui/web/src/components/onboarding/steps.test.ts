import {describe, expect, it} from "vitest";
import {
    ALL_ONBOARDING_STEPS,
    onboardingStepsFor,
    readinessCheckApplies,
    resolveReadinessCta,
} from "./steps.ts";
import {getRobotProfile} from "../../constants/robotProfiles.ts";

const stock = getRobotProfile("YardForce500");
const tron = getRobotProfile("AirseekersTron");

describe("onboarding steps per profile", () => {
    it("keeps the full upstream sequence for a stock robot", () => {
        expect(onboardingStepsFor(stock)).toEqual([
            "welcome", "robotModel", "firmware", "ntrip", "gps",
            "datum", "sensors", "calibration", "complete",
        ]);
    });

    it("skips firmware, GNSS configuration and the calibration drive on the Tron, and adds a dock step", () => {
        const steps = onboardingStepsFor(tron);
        expect(steps).toEqual(["welcome", "robotModel", "datum", "sensors", "dock", "complete"]);
        // Order always follows the master list.
        expect([...steps].sort((a, b) => ALL_ONBOARDING_STEPS.indexOf(a) - ALL_ONBOARDING_STEPS.indexOf(b))).toEqual(steps);
    });

    it("drops readiness checks the robot cannot pass, keeping datum, dock and area", () => {
        const ids = ["rtk", "corrections", "datum", "localizer", "localizerConfidence", "firmware", "dock", "imuBias", "imuYaw", "mag", "area"];
        expect(ids.filter((id) => readinessCheckApplies(id, stock))).toEqual(ids);
        expect(ids.filter((id) => readinessCheckApplies(id, tron))).toEqual(["rtk", "corrections", "datum", "dock", "mag", "area"]);
    });

    it("resolves readiness CTAs to steps the robot has, else to a route", () => {
        const stockSteps = onboardingStepsFor(stock);
        const tronSteps = onboardingStepsFor(tron);
        expect(resolveReadinessCta("gps", "rtk", stockSteps)).toEqual({step: "gps"});
        expect(resolveReadinessCta("calibration", "dock", stockSteps)).toEqual({step: "calibration"});
        expect(resolveReadinessCta("calibration", "dock", tronSteps)).toEqual({step: "dock"});
        expect(resolveReadinessCta("ntrip", "corrections", tronSteps)).toEqual({route: "/settings"});
        expect(resolveReadinessCta("datum", "datum", tronSteps)).toEqual({step: "datum"});
        expect(resolveReadinessCta("map", "area", tronSteps)).toEqual({route: "/map"});
        expect(resolveReadinessCta("diagnostics", "mag", tronSteps)).toEqual({route: "/diagnostics"});
    });
});
