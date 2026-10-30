// RTK base station (vendor LoRa base): live status from the GNSS driver
// (topic key `rtkBaseStatus`) and the /rtk-base/* actions.
import {Api, ContentType} from "../api/Api.ts";
import {useTopic} from "./useTopic.ts";
import {selectRtkBaseStatus, type RtkBasePairing, type RtkBaseStatus} from "../utils/rtkBase.ts";

/** Latest /mower_gps_node/lora_status (1 Hz); null until the first message. */
export function useRtkBaseStatus(enabled = true): RtkBaseStatus | null {
    const {data} = useTopic<RtkBaseStatus | null>("rtkBaseStatus", null, {
        select: (raw) => selectRtkBaseStatus(raw),
        enabled,
    });
    return enabled ? data : null;
}

export interface RtkBaseActionResult {
    success: boolean;
    message?: string;
}

export interface RtkBasePairingResult {
    ok: boolean;
    pairing?: RtkBasePairing;
    raw?: string;
    message?: string;
}

interface HttpLike {error?: {error?: string; message?: string}; message?: string}

function errorText(e: unknown): string {
    const h = e as HttpLike;
    return h?.error?.error || h?.error?.message || h?.message || String(e);
}

async function call<T>(api: Api<unknown>, method: "GET" | "POST", path: string, body?: unknown): Promise<T> {
    try {
        const res = await api.request<T>({
            path, method, format: "json",
            ...(body === undefined ? {} : {body, type: ContentType.Json}),
        });
        return res.data;
    } catch (e) {
        throw new Error(errorText(e));
    }
}

/** Write a pairing; success=true only when the board stored it (read back). */
export function pairRtkBase(api: Api<unknown>, addr: number, channel: number, area: string) {
    return call<RtkBaseActionResult>(api, "POST", "/rtk-base/pair", {addr, channel, area});
}

/** Ask the board for its stored pairing. */
export function readRtkBasePairing(api: Api<unknown>) {
    return call<RtkBasePairingResult>(api, "GET", "/rtk-base/pairing");
}

/** $BRESET to the board (relayed to the base). */
export function resetRtkBase(api: Api<unknown>) {
    return call<RtkBaseActionResult>(api, "POST", "/rtk-base/reset");
}
