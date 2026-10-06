import type {RobotBattery} from "../constants/robotProfiles.ts";

/** Default battery voltage thresholds — must match mower_config.schema.json */
export const BATTERY_DEFAULTS = {
    // YardForce500 SLA packs top out around 28.0 V at the dock; the old
    // 28.5 V default capped the displayed percent at 88.9 % even when full.
    fullVoltage: 28.0,
    emptyVoltage: 24.0,
    criticalVoltage: 23.0,
} as const;

/**
 * Compute battery percentage from voltage and settings.
 * Prefers highLevelStatus.battery_percent when available (> 0).
 * Falls back to linear interpolation between empty and full voltage.
 * Returns a rounded integer 0–100.
 */
export type BatteryLevel = "ok" | "warn" | "danger";

/** Thresholds shared by every battery badge/statistic on the GUI. */
export const BATTERY_WARN_PERCENT = 50;
export const BATTERY_DANGER_PERCENT = 20;

/**
 * Map a battery percentage to a severity level.
 * > 50 % → ok, 21–50 % → warn, ≤ 20 % → danger.
 */
export const getBatteryLevel = (percent: number): BatteryLevel => {
    if (percent <= BATTERY_DANGER_PERCENT) {
        return "danger";
    }
    if (percent <= BATTERY_WARN_PERCENT) {
        return "warn";
    }
    return "ok";
};

const finite = (v: unknown): number | undefined => {
    if (v === null || v === undefined || v === "") return undefined;
    const n = typeof v === "number" ? v : typeof v === "string" ? parseFloat(v) : NaN;
    return Number.isFinite(n) ? n : undefined;
};

export type BatteryThresholds = {fullV: number; emptyV: number; criticalV: number};

/**
 * Voltage window used for the percent estimate and the gauge: the yaml value
 * when set, else the robot profile's (profile.battery), else BATTERY_DEFAULTS.
 */
export const resolveBatteryThresholds = (
    settings: Record<string, any>,
    battery?: RobotBattery,
): BatteryThresholds => ({
    fullV: finite(settings["battery_full_voltage"]) ?? battery?.fullV ?? BATTERY_DEFAULTS.fullVoltage,
    emptyV: finite(settings["battery_empty_voltage"]) ?? battery?.emptyV ?? BATTERY_DEFAULTS.emptyVoltage,
    criticalV: finite(settings["battery_critical_voltage"]) ?? battery?.criticalV ?? BATTERY_DEFAULTS.criticalVoltage,
});

const percentFromVoltage = (voltage: number, full: number, empty: number): number => {
    const pct = ((voltage - empty) / (full - empty)) * 100;
    return Math.round(Math.max(0, Math.min(100, pct)));
};

/**
 * Whether the robot's reported percent can be trusted. A stock robot reports
 * 0 before its estimator runs, so 0 means "not reported"; a profile with
 * `preferReportedPercent` reports a real 0 % (the pack is empty).
 */
const reportedPercent = (
    batteryPercent: number | null | undefined,
    battery?: RobotBattery,
): number | undefined => {
    if (batteryPercent == null || !Number.isFinite(batteryPercent)) return undefined;
    if (battery?.preferReportedPercent ? batteryPercent >= 0 : batteryPercent > 0) {
        return Math.round(batteryPercent);
    }
    return undefined;
};

/**
 * Battery percent, or null when it cannot be known: a profile that prefers
 * the reported percent and has no voltage thresholds of its own does not
 * guess from voltage (the yaml's thresholds are another robot's defaults).
 */
const percentOrNull = (
    batteryPercent: number | null | undefined,
    voltage: number | undefined,
    settings: Record<string, any>,
    battery?: RobotBattery,
): number | null => {
    const reported = reportedPercent(batteryPercent, battery);
    if (reported !== undefined) return reported;
    if (battery?.preferReportedPercent) {
        if (!voltage || battery.fullV === undefined || battery.emptyV === undefined) return null;
        return percentFromVoltage(voltage, battery.fullV, battery.emptyV);
    }
    if (voltage) {
        const {fullV, emptyV} = resolveBatteryThresholds(settings, battery);
        return percentFromVoltage(voltage, fullV, emptyV);
    }
    return 0;
};

/**
 * @param battery the active robot profile's battery (profile.battery). When
 *   omitted, or for a stock profile, the behaviour is the historical one:
 *   reported percent when > 0, else a linear voltage estimate, else 0.
 */
export const computeBatteryPercent = (
    batteryPercent: number | null | undefined,
    voltage: number | undefined,
    settings: Record<string, any>,
    battery?: RobotBattery,
): number => percentOrNull(batteryPercent, voltage, settings, battery) ?? 0;

/**
 * Like computeBatteryPercent, but returns null until the high-level status
 * has arrived. Before that the voltage fallback is meaningless (e.g. 21.6 V
 * against a 28/24 V window reads 0 %), so the UI should show "--" instead.
 */
export const computeBatteryPercentOrNull = (
    highLevelStatus: { battery_percent?: number | null; state?: number; state_name?: string } | null | undefined,
    voltage: number | undefined,
    settings: Record<string, any>,
    battery?: RobotBattery,
): number | null => {
    if (!highLevelStatus || Object.keys(highLevelStatus).length === 0) {
        return null;
    }
    return percentOrNull(highLevelStatus.battery_percent, voltage, settings, battery);
};
