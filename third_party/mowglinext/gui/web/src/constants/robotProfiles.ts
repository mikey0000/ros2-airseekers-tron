// Robot profiles: the `mower_model` key promoted from "a table of physical
// presets" to "a description of what this robot is". A profile is the
// existing MOWER_MODELS preset (value, labels, defaults — unchanged, so the
// preset parity guard in pkg/api/model_preset_parity_test.go still holds) plus
// what the GUI needs to know to show the right screens: which optional
// subsystems exist, how it perceives obstacles, how it docks, its battery
// window, cameras and teleop limits.
//
// Every MowgliNext model resolves to UPSTREAM_FEATURES, i.e. the UI exactly as
// it is today. A robot that lacks a subsystem (no STM32 to flash, no Docker
// socket, ...) switches it off in its profile; screens ask hasFeature() rather
// than testing the model name.
//
// The active profile id comes from `mower_model` in mowgli_robot.yaml, else the
// backend's ROBOT_PROFILE env fallback (GET /api/robot/profile), see
// hooks/useRobotProfile.ts.

import {MOWER_MODELS, type MowerModel} from "./mowerModels.ts";

/** Optional subsystems a screen can depend on. */
export type FeatureId =
    | "stm32_firmware"      // flashable STM32 board + firmware debug log
    | "gnss_sidecar"        // mowgli-gps container (receiver plan/apply/factory reset)
    | "docker_host"         // Docker socket: container list/logs/restart (ROS restart)
    | "docker_admin"        // Docker-driven admin tools: host updater, rosbag recorder, remote access sidecar
    | "drive_tuning"        // drive / PID auto-tuning
    | "lidar"               // LiDAR settings, scan and map-anchor layers
    | "cameras"             // camera streams (perception page)
    | "dock_calibration"    // CalibrateDock wizard (charger contacts)
    | "fusion_graph"        // fusion_graph_node diagnostics and commands
    | "imu_yaw_calibration" // calibrate_imu_yaw_node
    | "status_leds"         // LED settings section
    | "lora_corrections"    // GNSS corrections over a LoRa base radio
    | "area_settings"       // per-area mowing settings served by the map server
    | "straight_driving"    // host-side angular trim / deadband / IMU heading hold (cmd_vel_slew)
    | "manual_blade_two_step"; // manual mode drives blade-off; blades start as a separate step

export const FEATURE_IDS: readonly FeatureId[] = [
    "stm32_firmware", "gnss_sidecar", "docker_host", "docker_admin", "drive_tuning", "lidar",
    "cameras", "dock_calibration", "fusion_graph", "imu_yaw_calibration",
    "status_leds", "lora_corrections", "area_settings", "straight_driving",
    "manual_blade_two_step",
];

/**
 * Feature set of a stock MowgliNext robot. Everything the GUI shows today is
 * on; the features that add UI no stock robot has are off.
 */
export const UPSTREAM_FEATURES: Readonly<Record<FeatureId, boolean>> = {
    stm32_firmware: true,
    gnss_sidecar: true,
    docker_host: true,
    docker_admin: true,
    drive_tuning: true,
    lidar: true,
    cameras: false,
    dock_calibration: true,
    fusion_graph: true,
    imu_yaw_calibration: true,
    status_leds: true,
    lora_corrections: false,
    area_settings: false,
    straight_driving: false,
    manual_blade_two_step: false,
};

export type PerceptionKind = "lidar" | "camera" | "none";
export type DockingKind = "charger_contacts" | "vision_marker";

export type RobotBattery = {
    /** Voltage thresholds; undefined = unknown for this robot. */
    fullV?: number;
    emptyV?: number;
    criticalV?: number;
    /** Trust the robot's reported percentage over a voltage estimate. */
    preferReportedPercent: boolean;
};

export type RobotCamera = {
    id: string;
    /** i18n key */
    label: string;
    topic: string;
    annotatedTopic?: string;
};

export type RobotTeleop = {maxLinear: number; maxAngular: number};

export type RobotProfile = MowerModel & {
    features: Readonly<Record<FeatureId, boolean>>;
    perception: PerceptionKind;
    docking: DockingKind;
    battery: RobotBattery;
    cameras: readonly RobotCamera[];
    /** Settings keys this robot does not use. A trailing `*` matches a prefix. */
    hiddenSettingKeys: readonly string[];
    teleop: RobotTeleop;
    /**
     * mowgli_interfaces/Status fields this robot has no sensor for. Its
     * hardware bridge leaves them at 0/false; the UI shows them as not
     * available instead of a fake reading.
     */
    unmeasuredStatusFields: readonly UnmeasuredStatusField[];
};

/** Status fields a robot may lack (see RobotProfile.unmeasuredStatusFields). */
export type UnmeasuredStatusField =
    | "mower_esc_temperature"
    | "mower_esc_current"
    | "mower_motor_temperature"
    | "raspberry_pi_power"
    | "ui_board_available"
    | "sound_module_available";

/** The profile-less fallback, and the id the schema defaults to. */
export const DEFAULT_ROBOT_PROFILE_ID = "YardForce500";
export const AIRSEEKERS_TRON_PROFILE_ID = "AirseekersTron";

/** Matches the caps hard-coded in pages/map/hooks/useManualMode.ts today. */
export const DEFAULT_TELEOP: RobotTeleop = {maxLinear: 0.25, maxAngular: 0.6};

/** What a profile may override on top of its preset. */
type ProfileOverlay = Partial<Omit<RobotProfile, keyof MowerModel | "features">> & {
    features?: Partial<Record<FeatureId, boolean>>;
};

function buildProfile(model: MowerModel, overlay: ProfileOverlay = {}): RobotProfile {
    const d = model.defaults;
    return {
        ...model,
        features: {...UPSTREAM_FEATURES, ...overlay.features},
        perception: overlay.perception ?? "lidar",
        docking: overlay.docking ?? "charger_contacts",
        battery: overlay.battery ?? {
            fullV: d.battery_full_voltage,
            emptyV: d.battery_empty_voltage,
            criticalV: d.battery_critical_voltage,
            preferReportedPercent: false,
        },
        cameras: overlay.cameras ?? [],
        hiddenSettingKeys: overlay.hiddenSettingKeys ?? [],
        teleop: overlay.teleop ?? DEFAULT_TELEOP,
        unmeasuredStatusFields: overlay.unmeasuredStatusFields ?? [],
    };
}

// Airseekers Tron (ROS 2 port: no STM32, Docker or LiDAR; three cameras, vision
// docking, LoRa RTK corrections). Its geometry lives once, in the AirseekersTron
// entry of MOWER_MODELS (placeholder URDF values, to be measured on the robot);
// this overlay only adds what the preset cannot express.
const PROFILE_OVERLAYS: Record<string, ProfileOverlay> = {
    [AIRSEEKERS_TRON_PROFILE_ID]: {
        features: {
            stm32_firmware: false,
            gnss_sidecar: false,
            docker_host: true,   // socket mounted: restart ROS 2, container list + logs
            docker_admin: false, // no host updater / rosbag / remote-access sidecar on Tron
            drive_tuning: false,
            lidar: false,
            cameras: true,
            dock_calibration: false,
            fusion_graph: false,
            imu_yaw_calibration: false,
            status_leds: false,
            lora_corrections: true,
            area_settings: true,  // map_server_node get/set_area_settings
            straight_driving: true, // mower_control cmd_vel_slew trim / heading hold
            // mower_mission manual_blade_requires_enable: manual drive starts blade-off,
            // the map page's Blades control sends mow_enabled (-> ~/manual_blade).
            manual_blade_two_step: true,
        },
        perception: "camera",
        docking: "vision_marker",
        battery: {preferReportedPercent: true},
        cameras: [
            // det_ros draws its detections per camera on /<ns>/image_annotated.
            {id: "left", label: "robotProfiles.cameras.left", topic: "/left_oa_camera/image_raw",
                annotatedTopic: "/left_oa_camera/image_annotated"},
            {id: "right", label: "robotProfiles.cameras.right", topic: "/right_oa_camera/image_raw",
                annotatedTopic: "/right_oa_camera/image_annotated"},
            {id: "rear", label: "robotProfiles.cameras.rear", topic: "/rear_camera/image_raw"},
            // Metoak front stereo (mower_cameras/stereo_cam, cameras.launch.py stereo:=true).
            {id: "front_left", label: "robotProfiles.cameras.front_left", topic: "/vio/left/image_raw"},
            {id: "front_right", label: "robotProfiles.cameras.front_right", topic: "/vio/right/image_raw"},
        ],
        // The cutter board reports one temperature per motor (mower_motor_temperature),
        // no separate ESC temperature; there is no Raspberry Pi, UI board or sound module.
        unmeasuredStatusFields: [
            "mower_esc_temperature", "raspberry_pi_power", "ui_board_available", "sound_module_available",
        ],
        // The motor MCU computes /odom itself; these only feed STM32 odometry.
        hiddenSettingKeys: [
            "ticks_per_meter", "deadband_pwm", "wheel_pid_*", "caster_*", "chassis_mass_kg",
        ],
    },
};

/** Every known profile: one per MowerModel preset, in picker order. */
export const ROBOT_PROFILES: readonly RobotProfile[] = [
    ...MOWER_MODELS.map((m) => buildProfile(m, PROFILE_OVERLAYS[m.value])),
];

export const DEFAULT_ROBOT_PROFILE: RobotProfile =
    ROBOT_PROFILES.find((p) => p.value === DEFAULT_ROBOT_PROFILE_ID)!;

/** Exact lookup; undefined for an unknown or empty id. */
export function findRobotProfile(id: string | null | undefined): RobotProfile | undefined {
    if (!id) return undefined;
    return ROBOT_PROFILES.find((p) => p.value === id);
}

/** Lookup that never fails: unknown ids get the stock (upstream) profile. */
export function getRobotProfile(id: string | null | undefined): RobotProfile {
    return findRobotProfile(id) ?? DEFAULT_ROBOT_PROFILE;
}

export function hasFeature(profile: RobotProfile, feature: FeatureId): boolean {
    return profile.features[feature] ?? UPSTREAM_FEATURES[feature];
}

export function isSettingKeyHidden(profile: RobotProfile, key: string): boolean {
    return profile.hiddenSettingKeys.some((pattern) =>
        pattern.endsWith("*") ? key.startsWith(pattern.slice(0, -1)) : key === pattern);
}
