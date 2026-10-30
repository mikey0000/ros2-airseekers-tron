// RTK base station (vendor LoRa base) helpers for the Settings card: address
// parsing, regions, the vendor universal code and the driver's status JSON
// (/mower_gps_node/lora_status, topic key `rtkBaseStatus`).
import {parseStringMsgJson} from "./areaSettings.ts";

/** Highest LoRa address the board accepts (0xFFFF is not a base address). */
export const RTK_BASE_MAX_ADDR = 0xfffe;
export const RTK_BASE_MIN_CHANNEL = 1;
export const RTK_BASE_MAX_CHANNEL = 75;

/** The vendor app's cloud-less fallback ("universal code"). */
export const RTK_BASE_UNIVERSAL_CODE = {addr: 38183, channel: 40, area: "02"} as const;

/** Region codes the pairing takes, in picker order, with their i18n label keys. */
export const RTK_BASE_REGIONS = [
    {area: "02", labelKey: "rtkBase.region.europe"},
    {area: "01", labelKey: "rtkBase.region.northAmerica"},
    {area: "03", labelKey: "rtkBase.region.australia"},
] as const;

/** The board's $GLIST band -> i18n label key. */
const BAND_LABEL_KEYS: Record<string, string> = {
    "0": "rtkBase.band.europe",
    "1": "rtkBase.band.americas",
    "2": "rtkBase.band.australia",
    F: "rtkBase.band.all",
};

/** $GLIST band -> the region code that selects it. */
const AREA_FOR_BAND: Record<string, string> = {"0": "02", "1": "01", "2": "03"};

export interface RtkBasePairing {
    addr: number;
    addr_hex: string;
    channel: number;
    band?: string | null;
    band_name?: string | null;
}

export interface RtkBaseStatus {
    pairing: RtkBasePairing | null;
    pairing_enabled?: boolean;
    /** $GNSTA: FINE = corrections from the base arrive, FAIL = they do not. */
    ref_station: "FINE" | "FAIL" | null;
    /** The board is in network-RTK mode (NTRIP owns the receiver). */
    nrtk_on: boolean;
    station_id: string | null;
    diff_age_s: number | null;
    fix_type: string | null;
    correction_source?: string;
    rtcm_source?: string;
    connected?: boolean;
}

/** "38183", "0x9527" or "0X9527" -> 38183; undefined when not a valid address. */
export function parseRtkBaseAddr(text: string | number | null | undefined): number | undefined {
    if (typeof text === "number") {
        return Number.isInteger(text) && text >= 0 && text <= RTK_BASE_MAX_ADDR ? text : undefined;
    }
    const value = (text ?? "").trim();
    let addr: number;
    if (/^0x[0-9a-f]{1,4}$/i.test(value)) addr = parseInt(value.slice(2), 16);
    else if (/^\d{1,5}$/.test(value)) addr = parseInt(value, 10);
    else return undefined;
    return addr <= RTK_BASE_MAX_ADDR ? addr : undefined;
}

/** 38183 -> "38183 (0x9527)". */
export function formatRtkBaseAddr(addr: number): string {
    return `${addr} (0x${addr.toString(16).toUpperCase().padStart(4, "0")})`;
}

export function rtkBaseBandLabelKey(band: string | null | undefined): string | undefined {
    return band ? BAND_LABEL_KEYS[band.toUpperCase()] : undefined;
}

/** The region code that selects the stored band; undefined for "all bands" / unknown. */
export function rtkBaseAreaForBand(band: string | null | undefined): string | undefined {
    return band ? AREA_FOR_BAND[band] : undefined;
}

/** Multiplex payload (std_msgs/String {data}) -> status; undefined when malformed. */
export function selectRtkBaseStatus(raw: unknown): RtkBaseStatus | undefined {
    const d = parseStringMsgJson(raw);
    if (!d || !("pairing" in d)) return undefined;
    return d as unknown as RtkBaseStatus;
}

/**
 * True when the LoRa base cannot feed the receiver right now: the board is in
 * network-RTK mode, NTRIP is the active source, or Enable NTRIP is on.
 */
export function rtkBaseNtripOwns(status: RtkBaseStatus | null | undefined, ntripEnabled: boolean): boolean {
    return ntripEnabled || !!status?.nrtk_on || status?.correction_source === "ntrip";
}
