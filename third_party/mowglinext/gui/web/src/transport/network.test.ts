import {afterEach, describe, expect, it, vi} from "vitest";

vi.mock("../utils/apiHost.ts", () => ({
    httpBase: () => "http://robot:4006",
    wsBase: () => "ws://robot:4006",
}));

import {NetworkTransport} from "./network.ts";
import type {Transport} from "./types.ts";

afterEach(() => {
    vi.unstubAllGlobals();
});

describe("NetworkTransport", () => {
    it("fetches the path against httpBase()", async () => {
        const fetchMock = vi.fn(() => Promise.resolve(new Response("ok")));
        vi.stubGlobal("fetch", fetchMock);
        const init = {method: "POST", body: "{}"};
        await new NetworkTransport().fetch("/api/settings/status", init);
        expect(fetchMock).toHaveBeenCalledWith("http://robot:4006/api/settings/status", init);
    });

    it("opens sockets against wsBase() and ignores the options", () => {
        const made: string[] = [];
        vi.stubGlobal("WebSocket", class {
            constructor(readonly url: string) { made.push(url); }
        });
        const t: Transport = new NetworkTransport();
        expect(t.kind).toBe("network");
        t.openSocket("/api/mowglinext/publish/joy", {prio: true});
        expect(made).toEqual(["ws://robot:4006/api/mowglinext/publish/joy"]);
    });
});
