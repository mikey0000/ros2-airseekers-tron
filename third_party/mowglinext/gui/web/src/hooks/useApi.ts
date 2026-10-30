import {Api} from "../api/Api.ts";
import {apiFetch} from "../transport/index.ts";

/**
 * Backend path of a URL the generated client built: "/api/x?y" stays as is, a full URL
 * (or a protocol-relative "//host/api/x") keeps only path, query and hash.
 */
export function backendPath(url: string): string {
    if (url.startsWith("/") && !url.startsWith("//")) return url;
    const parsed = new URL(url, window.location.href);
    return `${parsed.pathname}${parsed.search}${parsed.hash}`;
}

/** The generated client's fetch: every request goes through the transport seam. */
const seamFetch: typeof fetch = (input, init) => {
    const url = typeof input === "string" ? input : input instanceof URL ? input.href : input.url;
    return apiFetch(backendPath(url), init);
};

// One shared Api instance for the whole app — previously `useApi` returned
// `new Api()` on every render, which allocated fetch config and broke any
// consumer that tried to memoize against the api reference.
const api = new Api({baseUrl: "/api", customFetch: seamFetch});

export const useApi = () => api;
