// Live deck (blade) height: mcu_node clamps to the vendor range 30-90 mm and
// latches the commanded value on /cutter/height_mm (topic key "cutterHeight").
export const BLADE_HEIGHT_MIN_MM = 30;
export const BLADE_HEIGHT_MAX_MM = 90;
export const BLADE_HEIGHT_STEP_MM = 5;

/** Clamp to 30-90 and snap to the 5 mm grid. */
export const snapBladeHeight = (mm: number): number => {
    const snapped = Math.round(mm / BLADE_HEIGHT_STEP_MM) * BLADE_HEIGHT_STEP_MM;
    return Math.max(BLADE_HEIGHT_MIN_MM, Math.min(BLADE_HEIGHT_MAX_MM, snapped));
};

/** One step up (+1) or down (-1) from the current height, clamped. */
export const stepBladeHeight = (mm: number, dir: 1 | -1): number =>
    snapBladeHeight(snapBladeHeight(mm) + dir * BLADE_HEIGHT_STEP_MM);

/** std_msgs/Int16 {data} -> mm, or undefined for anything else. */
export const parseCutterHeightMsg = (raw: unknown): number | undefined => {
    const d = (raw as { data?: unknown } | null | undefined)?.data;
    return typeof d === "number" && Number.isFinite(d) && d > 0 ? d : undefined;
};
