// Schedule page "Rules" card: the rain and low-battery behaviour that applies
// to every scheduled run, read from the robot settings instead of hard-coded
// "ON" chips (F25: they said "Rain-aware ON" with rain mode set to Ignore).

export interface ScheduleRule {
    id: "rain" | "battery";
    on: boolean;
    /** i18n key + params of the one-line hint. */
    hintKey: string;
    hintParams?: Record<string, string | number>;
}

/** settingsRain.* label keys per rain_mode (RainSection RAIN_MODES). */
const RAIN_MODE_LABEL: Record<number, string> = {
    0: "settingsRain.rainModeIgnoreLabel",
    1: "settingsRain.rainModeDockLabel",
    2: "settingsRain.rainModeDockUntilDryLabel",
    3: "settingsRain.rainModePauseAutoLabel",
};

const num = (v: unknown, fallback: number): number => {
    const n = typeof v === "number" ? v : typeof v === "string" && v.trim() !== "" ? Number(v) : NaN;
    return Number.isFinite(n) ? n : fallback;
};

/** Template defaults (asserts/mower_config.schema.json) when a key is absent. */
export function scheduleRules(settings: Record<string, unknown>): ScheduleRule[] {
    const rainMode = num(settings["rain_mode"], 2);
    const lowPct = Math.round(num(settings["battery_low_percent"], 20));
    const lowAction = settings["battery_low_action"] === "stop" ? "stop" : "dock";
    return [
        {id: "rain", on: rainMode !== 0, hintKey: RAIN_MODE_LABEL[rainMode] ?? RAIN_MODE_LABEL[2]},
        {
            id: "battery", on: lowAction === "dock",
            hintKey: lowAction === "dock" ? "schedulePage.ruleAutoDockLowHintPct" : "schedulePage.ruleStopLowHintPct",
            hintParams: {pct: lowPct},
        },
    ];
}
