// Single source for the onboarding wizard's steps. Shared by the wizard
// (navigation + save gating) and the readiness step (CTA deep-links), so the
// two never drift out of lockstep.
//
// Steps are identified by id, not index: which steps a robot gets depends on
// its profile (constants/robotProfiles.ts). A stock MowgliNext robot gets the
// full upstream sequence; a robot without an STM32 board skips the firmware
// step, one without the GNSS sidecar skips receiver and NTRIP configuration,
// and a vision-docking robot sets its dock in a dedicated step instead of the
// charger-contact calibration drive.

import {hasFeature, type RobotProfile} from "../../constants/robotProfiles.ts";
import type {ReadinessCtaTarget} from "./readinessChecks.ts";

export type OnboardingStepId =
    | "welcome"
    | "robotModel"
    | "firmware"
    | "ntrip"
    | "gps"
    | "datum"
    | "sensors"
    | "calibration"
    | "dock"
    | "complete";

/** Every step, in wizard order. */
export const ALL_ONBOARDING_STEPS: readonly OnboardingStepId[] = [
    "welcome",
    "robotModel",
    "firmware",
    "ntrip",
    "gps",
    "datum",
    "sensors",
    "calibration",
    "dock",
    "complete",
];

/** Whether `step` applies to a robot with `profile`. */
export function onboardingStepApplies(step: OnboardingStepId, profile: RobotProfile): boolean {
    switch (step) {
        case "firmware":
            return hasFeature(profile, "stm32_firmware");
        case "ntrip":
        case "gps":
            // Receiver configuration and the NTRIP client both live in the
            // mowgli-gps sidecar.
            return hasFeature(profile, "gnss_sidecar");
        case "calibration":
            // The IMU-yaw / dock calibration drive (calibrate_imu_yaw_node).
            return hasFeature(profile, "imu_yaw_calibration");
        case "dock":
            // Charger-contact robots capture the dock pose during the
            // calibration drive; a vision-docking robot sets it here.
            return profile.docking === "vision_marker";
        default:
            return true;
    }
}

/** The ordered steps for a robot with `profile`. */
export function onboardingStepsFor(profile: RobotProfile): OnboardingStepId[] {
    return ALL_ONBOARDING_STEPS.filter((step) => onboardingStepApplies(step, profile));
}

/** Steps whose Next saves the wizard's settings (they edit settings values). */
export const SETTINGS_STEPS: ReadonlySet<OnboardingStepId> = new Set<OnboardingStepId>([
    "robotModel",
    "ntrip",
    "gps",
    "datum",
    "sensors",
    "calibration",
]);

/** In-wizard step a readiness CTA target opens, when it is not a route. */
const CTA_STEP: Record<ReadinessCtaTarget, OnboardingStepId | null> = {
    gps: "gps",
    ntrip: "ntrip",
    datum: "datum",
    firmware: "firmware",
    calibration: "calibration",
    diagnostics: null,
    map: null,
};

/** Route a readiness CTA falls back to when it has no step in this wizard. */
export type ReadinessCtaRoute = "/map" | "/diagnostics" | "/settings";

/**
 * Resolve a readiness CTA to a step of this robot's wizard, or to a route.
 * The dock check opens the dock step on robots that have one. A target whose
 * step this robot skips (e.g. NTRIP on a robot with radio corrections) opens
 * Settings, where the replacement UI lives.
 */
export function resolveReadinessCta(
    target: ReadinessCtaTarget,
    checkId: string,
    steps: readonly OnboardingStepId[],
): {step: OnboardingStepId} | {route: ReadinessCtaRoute} {
    if (checkId === "dock" && steps.includes("dock")) return {step: "dock"};
    const step = CTA_STEP[target];
    if (step && steps.includes(step)) return {step};
    if (target === "map") return {route: "/map"};
    if (target === "diagnostics") return {route: "/diagnostics"};
    return {route: "/settings"};
}

/**
 * Whether a readiness check (readinessChecks.ts id) applies to a robot with
 * `profile`. Checks for hardware or nodes the robot does not have would stay
 * "pending" forever and gate Finish for no reason.
 */
export function readinessCheckApplies(checkId: string, profile: RobotProfile): boolean {
    switch (checkId) {
        case "firmware":
            return hasFeature(profile, "stm32_firmware");
        case "localizer":
        case "localizerConfidence":
            // Both read fusion_graph_node diagnostics.
            return hasFeature(profile, "fusion_graph");
        case "imuBias":
        case "imuYaw":
            // Both are produced by calibrate_imu_yaw_node.
            return hasFeature(profile, "imu_yaw_calibration");
        default:
            return true;
    }
}
