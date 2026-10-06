// Lookups over the accumulated /diagnostics entries (hooks/useDiagnostics.ts)
// for cards that show one subsystem's health instead of the whole list.

/** Minimal shape of an accumulated /diagnostics entry. */
export type LookupStatus = {
    name?: string;
    hardware_id?: string;
    level: number;
    message?: string;
    values?: {key: string; value: string}[];
};

// hardware_id values / name fragments of the localisation chain: the state
// estimator (robot_localization's ekf_node / navsat_transform, fusion_graph),
// GNSS and IMU. Matched case-insensitively; names are "<node>: <what>".
const LOCALIZATION_HARDWARE_IDS = new Set(["localization", "gnss", "gps", "imu"]);
const LOCALIZATION_NAME = /\b(ekf|ukf|navsat|locali[sz]ation|gnss|gps|imu|fusion_graph)/i;

/** Entries that describe the localisation chain, in their original order. */
export function localizationDiagnostics<T extends LookupStatus>(statuses: readonly T[] | undefined): T[] {
    return (statuses ?? []).filter((s) =>
        LOCALIZATION_HARDWARE_IDS.has((s.hardware_id ?? "").toLowerCase()) ||
        LOCALIZATION_NAME.test(s.name ?? ""));
}

/** Value of `key` in the entry named exactly `name`, or undefined. */
export function diagnosticValue(
    statuses: readonly LookupStatus[] | undefined,
    name: string,
    key: string,
): string | undefined {
    const entry = (statuses ?? []).find((s) => s.name === name);
    return entry?.values?.find((kv) => kv.key === key)?.value;
}
