// Profile gates: which screens, nav entries, settings sections and in-page
// controls a robot shows, decided by its RobotProfile (constants/robotProfiles.ts).
//
// Every gated piece of UI has a stable gate id and the features it needs. A
// stock MowgliNext robot has every feature it has always had
// (UPSTREAM_FEATURES), so it sees exactly today's UI; a robot that lacks a
// subsystem switches the feature off in its profile and the UI that drives
// that subsystem disappears. Nothing is deleted, only not rendered.
//
// Ids: router paths start with "/", Settings sections are "settings:<id>",
// in-page panels/buttons are "feature:<id>". This replaces the build-time
// TRON_HIDDEN_PAGES list (and its VITE_TRON_FEATURES switch): the old ids are
// kept verbatim so a gate can be traced back to it.

import {
    type FeatureId,
    type RobotProfile,
    FEATURE_IDS,
    UPSTREAM_FEATURES,
    hasFeature,
    isSettingKeyHidden,
} from "./robotProfiles.ts";

/** Gate id -> features it needs (ALL of them must be present). */
export const PROFILE_GATES = {
    // Routes
    // Camera streams + detections. Additive: no stock robot has cameras.
    "/perception": ["cameras"],

    // Settings sections
    "settings:updates": ["docker_host"],        // host updater (docker image pulls)
    "settings:remote_access": ["docker_host"],  // Tailscale sidecar created over the docker socket
    "settings:drive_motor": ["drive_tuning"],   // drive / PID auto-tuning (docker exec)
    "settings:leds": ["status_leds"],           // mowgli_leds SPI strip

    // In-page features
    "feature:firmware_flash": ["stm32_firmware"],      // dashboard "flash firmware" CTA
    "feature:firmware_debug": ["stm32_firmware"],      // Diagnostics firmware-debug log card
    "feature:gnss_configurator": ["gnss_sidecar"],     // GNSS receiver plan/apply/factory-reset card
    "feature:host_updater": ["docker_host"],           // side-rail running-version / updates links
    "feature:rosbag": ["docker_host"],                 // Diagnostics rosbag recorder (docker exec)
    "feature:containers": ["docker_host"],             // Diagnostics container table + health badge
    "feature:restart_ros2": ["docker_host"],           // Settings "Restart ROS2" (docker restart)
    "feature:lidar": ["lidar"],                        // LiDAR settings, scan + LiDAR-map layers, anchor tiles
    "feature:fusion_graph": ["fusion_graph"],          // /fusion_graph_node/* diagnostics + commands
    "feature:imu_yaw_calibration": ["imu_yaw_calibration"], // /calibrate_imu_yaw_node/* buttons
    "feature:dock_calibration": ["dock_calibration"],  // CalibrateDock wizard (charger contacts)
} as const satisfies Record<string, readonly FeatureId[]>;

export type GateId = keyof typeof PROFILE_GATES;

export const GATE_IDS = Object.keys(PROFILE_GATES) as GateId[];

/** True when the robot has every feature `id` needs. Unknown ids are visible. */
export function isGateVisible(profile: RobotProfile, id: string): boolean {
    const needs = (PROFILE_GATES as Record<string, readonly FeatureId[]>)[id];
    return needs === undefined || needs.every((f) => hasFeature(profile, f));
}

/** Every gate this robot hides, in declaration order. */
export function hiddenGates(profile: RobotProfile): GateId[] {
    return GATE_IDS.filter((id) => !isGateVisible(profile, id));
}

/**
 * Settings keys that only mean something with a LiDAR. Hidden (but still
 * claimed by their section, so they never leak into Advanced) when the robot
 * has no LiDAR.
 */
const LIDAR_SETTING_KEYS = ["lidar_*", "use_lidar", "use_lidar_map_anchor"] as const;

const matchesKey = (pattern: string, key: string) =>
    pattern.endsWith("*") ? key.startsWith(pattern.slice(0, -1)) : key === pattern;

/** The profile's own hiddenSettingKeys plus the keys of features it lacks. */
export function isProfileSettingKeyHidden(profile: RobotProfile, key: string): boolean {
    if (isSettingKeyHidden(profile, key)) return true;
    return !hasFeature(profile, "lidar") && LIDAR_SETTING_KEYS.some((p) => matchesKey(p, key));
}

/** Upstream mqtt_bridge_node default; also the schema default. */
export const UPSTREAM_MQTT_TOPIC_PREFIX = "mowgli";

/** True for a robot with exactly the stock MowgliNext feature set. */
export function isUpstreamFeatureSet(profile: RobotProfile): boolean {
    return FEATURE_IDS.every((f) => hasFeature(profile, f) === UPSTREAM_FEATURES[f]);
}

/**
 * MQTT topic prefix a robot should publish under when the operator has not set
 * one. Stock MowgliNext robots keep the upstream "mowgli"; any other robot
 * uses its profile id (lower-cased, MQTT-safe) so two kinds of robot on one
 * broker do not collide.
 */
export function defaultMqttTopicPrefix(profile: RobotProfile): string {
    if (isUpstreamFeatureSet(profile)) return UPSTREAM_MQTT_TOPIC_PREFIX;
    return profile.value.toLowerCase().replace(/[^a-z0-9_-]/g, "_");
}

export function isGateId(id: string): id is GateId {
    return Object.prototype.hasOwnProperty.call(PROFILE_GATES, id);
}
