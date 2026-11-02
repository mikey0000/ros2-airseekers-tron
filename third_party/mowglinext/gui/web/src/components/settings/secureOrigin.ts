import {apiFetch} from "../../transport/index.ts";

/** GET /api/secure-origin (pkg/api/secure_origin.go). */
export interface SecureOriginStatus {
    enabled: boolean;
    https_urls: string[];
    hostname: string;
    ca_fingerprint_sha256?: string;
    ca_not_after?: string;
}

/** GET /api/secure-origin through the transport seam (also used by the Bluetooth guide). */
export async function fetchSecureOrigin(signal?: AbortSignal): Promise<SecureOriginStatus> {
    const res = await apiFetch("/api/secure-origin", {signal, cache: "no-store"});
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    return await res.json() as SecureOriginStatus;
}
