// Standard (default) mowing path width of the Airseekers Tron.
//
// Mirror of mower_map area_settings.derive_default_swath_width (ROS side is the
// authority; this is only for showing the result next to the settings):
//   default_swath_width_m                     when > 0 (explicit override)
//   blade_disc_mm / 1000 - swath_overlap_m    otherwise
// clamped to the swath_width_m range. Unknown disc -> 220 mm; overlap clamped
// to 0..0.15 m. An area's own swath_width_m always wins over this.

export const BLADE_DISCS_MM = [220, 330] as const;
export const SWATH_OVERLAP_RANGE = {min: 0, max: 0.15, step: 0.01} as const;
export const SWATH_WIDTH_RANGE = {min: 0.10, max: 0.40} as const;
export const PATH_WIDTH_FALLBACKS = {blade_disc_mm: 220, swath_overlap_m: 0.04, default_swath_width_m: 0} as const;

const num = (v: unknown, fallback: number): number =>
    typeof v === "number" && Number.isFinite(v) ? v : fallback;
const clamp = (v: number, lo: number, hi: number) => Math.min(hi, Math.max(lo, v));
const round4 = (v: number) => Math.round(v * 10000) / 10000;

export function deriveDefaultSwathWidth(
    bladeDiscMm: unknown, swathOverlapM: unknown, defaultSwathWidthM?: unknown,
): number {
    const explicit = num(defaultSwathWidthM, 0);
    if (explicit > 0) return round4(clamp(explicit, SWATH_WIDTH_RANGE.min, SWATH_WIDTH_RANGE.max));
    let disc = num(bladeDiscMm, PATH_WIDTH_FALLBACKS.blade_disc_mm);
    if (!(BLADE_DISCS_MM as readonly number[]).includes(Math.round(disc))) disc = PATH_WIDTH_FALLBACKS.blade_disc_mm;
    const overlap = clamp(num(swathOverlapM, PATH_WIDTH_FALLBACKS.swath_overlap_m),
        SWATH_OVERLAP_RANGE.min, SWATH_OVERLAP_RANGE.max);
    return round4(clamp(disc / 1000 - overlap, SWATH_WIDTH_RANGE.min, SWATH_WIDTH_RANGE.max));
}
