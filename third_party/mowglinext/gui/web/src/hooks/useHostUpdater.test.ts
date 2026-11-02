import {act, renderHook} from "@testing-library/react";
import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {
    UPDATER_MAX_POLL_MS,
    UPDATER_POLL_MS,
    nextUpdaterPollDelay,
    useHostUpdater,
} from "./useHostUpdater.ts";

const mocks = vi.hoisted(() => ({
    apiFetch: vi.fn(),
    updaterVisible: true,
}));

vi.mock("../transport/index.ts", () => ({apiFetch: mocks.apiFetch}));
vi.mock("./useProfileGates.ts", () => ({
    useProfileGates: () => ({isVisibleStrict: (id: string) => id === "settings:updates" && mocks.updaterVisible}),
}));

const STATE = {api: 1, agent: {version: "1", revision: "r", platform: "p"}, trusted_repositories: [], state: {notices: []}};
const ok = () => new Response(JSON.stringify(STATE), {status: 200});
const unavailable = () => new Response(JSON.stringify({error: "host updater unavailable"}), {status: 503});

const flush = () => act(async () => { await Promise.resolve(); await Promise.resolve(); });

describe("nextUpdaterPollDelay", () => {
    it("keeps the base interval after a success", () => {
        expect(nextUpdaterPollDelay(40_000, true)).toBe(UPDATER_POLL_MS);
    });

    it("doubles after each failure up to the cap", () => {
        let d = UPDATER_POLL_MS;
        const seen: number[] = [];
        for (let i = 0; i < 6; i++) seen.push(d = nextUpdaterPollDelay(d, false));
        expect(seen).toEqual([10_000, 20_000, 40_000, 60_000, 60_000, 60_000]);
        expect(Math.max(...seen)).toBe(UPDATER_MAX_POLL_MS);
    });
});

describe("useHostUpdater polling", () => {
    beforeEach(() => {
        vi.useFakeTimers();
        mocks.apiFetch.mockReset();
        mocks.updaterVisible = true;
    });
    afterEach(() => vi.useRealTimers());

    it("never calls the updater on a robot without it (settings:updates gated)", async () => {
        mocks.updaterVisible = false;
        mocks.apiFetch.mockImplementation(() => Promise.resolve(ok()));
        renderHook(() => useHostUpdater());
        await flush();
        await act(async () => { await vi.advanceTimersByTimeAsync(60_000); });
        expect(mocks.apiFetch).not.toHaveBeenCalled();
    });

    it("polls every 5 s while the updater answers", async () => {
        mocks.apiFetch.mockImplementation(() => Promise.resolve(ok()));
        const {result} = renderHook(() => useHostUpdater());
        await flush();
        expect(mocks.apiFetch).toHaveBeenCalledTimes(1);
        expect(mocks.apiFetch).toHaveBeenCalledWith("/api/system/updater/state", {cache: "no-store"});
        await act(async () => { await vi.advanceTimersByTimeAsync(3 * UPDATER_POLL_MS); });
        expect(mocks.apiFetch).toHaveBeenCalledTimes(4);
        expect(result.current.data?.api).toBe(1);
    });

    it("backs off while the updater returns 503", async () => {
        mocks.apiFetch.mockImplementation(() => Promise.resolve(unavailable()));
        const {result, unmount} = renderHook(() => useHostUpdater());
        await flush();
        expect(result.current.error).toBe("host updater unavailable");
        // 5 + 10 + 20 + 40 ... would be 14 reads at a fixed 5 s over 70 s.
        await act(async () => { await vi.advanceTimersByTimeAsync(70_000); });
        expect(mocks.apiFetch).toHaveBeenCalledTimes(4); // t = 0, 10, 30, 70 s
        unmount();
        await act(async () => { await vi.advanceTimersByTimeAsync(120_000); });
        expect(mocks.apiFetch).toHaveBeenCalledTimes(4);
    });
});
