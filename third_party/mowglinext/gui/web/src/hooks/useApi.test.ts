import {afterEach, describe, expect, it, vi} from "vitest";
import {backendPath, useApi} from "./useApi.ts";

afterEach(() => {
    vi.unstubAllGlobals();
});

describe("useApi", () => {
    it("sends generated-client requests through the seam with /api paths", async () => {
        const fetchMock = vi.fn(() => Promise.resolve(new Response("{}", {headers: {"Content-Type": "application/json"}})));
        vi.stubGlobal("fetch", fetchMock);
        await useApi().request({path: "/settings/status", method: "GET", format: "json"});
        expect(fetchMock).toHaveBeenCalledTimes(1);
        expect((fetchMock.mock.calls[0] as unknown[])[0]).toBe("/api/settings/status");
    });

    it("strips the origin of full URLs", () => {
        expect(backendPath("/api/x?y=1")).toBe("/api/x?y=1");
        expect(backendPath("http://robot:4006/api/x?y=1")).toBe("/api/x?y=1");
        expect(backendPath("//localhost:4200/api/x")).toBe("/api/x");
    });
});
