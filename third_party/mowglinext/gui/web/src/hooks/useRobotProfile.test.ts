import {act, renderHook, waitFor} from "@testing-library/react";
import {afterEach, describe, expect, it, vi} from "vitest";

const {request, api} = vi.hoisted(() => {
    const request = vi.fn();
    return {request, api: {request: (...args: unknown[]): unknown => request(...args)}};
});
vi.mock("./useApi.ts", () => ({useApi: () => api}));

import {
    refreshRobotProfile,
    resetRobotProfileCache,
    resolveRobotProfile,
    useFeature,
    useRobotProfile,
} from "./useRobotProfile.ts";

describe("resolveRobotProfile", () => {
    it("prefers an explicit override", () => {
        const r = resolveRobotProfile("Sabo", {id: "AirseekersTron", source: "env"});
        expect(r.profile.value).toBe("Sabo");
        expect(r.source).toBe("override");
    });

    it("uses the backend answer and its source", () => {
        const r = resolveRobotProfile(undefined, {id: "AirseekersTron", source: "env"});
        expect(r.profile.value).toBe("AirseekersTron");
        expect(r.source).toBe("env");
    });

    it("ignores an unknown override", () => {
        const r = resolveRobotProfile("NoSuchRobot", {id: "LUV1000RI", source: "settings"});
        expect(r.profile.value).toBe("LUV1000RI");
        expect(r.source).toBe("settings");
    });

    it("falls back to the stock profile", () => {
        expect(resolveRobotProfile(undefined, null)).toMatchObject({source: "fallback"});
        expect(resolveRobotProfile(undefined, {id: "Mystery", source: "env"}).profile.value)
            .toBe("YardForce500");
    });
});

describe("useRobotProfile", () => {
    afterEach(() => {
        request.mockReset();
        resetRobotProfileCache();
    });

    it("fetches /robot/profile once and shares the answer", async () => {
        request.mockResolvedValue({data: {id: "AirseekersTron", source: "env"}});
        const a = renderHook(() => useRobotProfile());
        const b = renderHook(() => useFeature("cameras"));
        expect(a.result.current.loading).toBe(true);
        expect(a.result.current.profile.value).toBe("YardForce500");

        await waitFor(() => expect(a.result.current.loading).toBe(false));
        expect(a.result.current.profile.value).toBe("AirseekersTron");
        expect(a.result.current.source).toBe("env");
        expect(b.result.current).toBe(true);
        expect(request).toHaveBeenCalledTimes(1);
        expect(request).toHaveBeenCalledWith(expect.objectContaining({path: "/robot/profile", method: "GET"}));
    });

    it("keeps the stock profile when the backend has no route", async () => {
        request.mockRejectedValue(new Error("404"));
        const {result} = renderHook(() => useRobotProfile());
        await waitFor(() => expect(result.current.loading).toBe(false));
        expect(result.current.profile.value).toBe("YardForce500");
        expect(result.current.source).toBe("fallback");
    });

    it("refetches after refreshRobotProfile", async () => {
        request.mockResolvedValueOnce({data: {id: "YardForce500", source: "default"}});
        const {result} = renderHook(() => useRobotProfile());
        await waitFor(() => expect(result.current.source).toBe("default"));

        request.mockResolvedValueOnce({data: {id: "Sabo", source: "settings"}});
        await act(() => refreshRobotProfile(api as never));
        expect(result.current.profile.value).toBe("Sabo");
        expect(request).toHaveBeenCalledTimes(2);
    });
});
