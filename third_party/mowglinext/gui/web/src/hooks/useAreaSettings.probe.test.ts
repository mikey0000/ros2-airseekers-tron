import {act, renderHook, waitFor} from "@testing-library/react";
import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";

const {request, api} = vi.hoisted(() => {
    const request = vi.fn();
    return {request, api: {request: (...args: unknown[]): unknown => request(...args)}};
});
vi.mock("./useApi.ts", () => ({useApi: () => api}));
vi.mock("./useProfileGates.ts", () => ({useGate: () => true}));

import {resetAreaSettingsProbe, useAreaSettingsSupport} from "./useAreaSettings.ts";

const ok = {data: {supported: true, settings: {cutter_height_mm: 90}}};

describe("useAreaSettingsSupport probe", () => {
    beforeEach(() => {
        vi.useFakeTimers({shouldAdvanceTime: true});
        resetAreaSettingsProbe(null);
        request.mockReset();
    });
    afterEach(() => {
        resetAreaSettingsProbe(null);
        vi.useRealTimers();
    });

    // Regression 2026-10-23: the page loaded while the stack restarted, the
    // defaults GET returned 500 (foxglove not connected), the probe latched
    // supported=false for the whole page life, and "Mow area" then sent
    // start_in_area directly instead of opening the StartMowSheet.
    it("a transient 500 is not cached as unsupported and is retried", async () => {
        request.mockRejectedValueOnce({status: 500, error: {error: "foxglove: not connected"}});
        request.mockResolvedValue(ok);
        const {result} = renderHook(() => useAreaSettingsSupport());
        await waitFor(() => expect(request).toHaveBeenCalledTimes(1));
        await act(async () => { await Promise.resolve(); });
        expect(result.current.supported).not.toBe(false);
        act(() => { vi.advanceTimersByTime(3000); });
        await waitFor(() => expect(result.current.supported).toBe(true));
        expect(request).toHaveBeenCalledTimes(2);
    });

    it("a 501 is a definite 'unsupported'", async () => {
        request.mockRejectedValue({status: 501});
        const {result} = renderHook(() => useAreaSettingsSupport());
        await waitFor(() => expect(result.current.supported).toBe(false));
        act(() => { vi.advanceTimersByTime(60000); });
        expect(request).toHaveBeenCalledTimes(1);
    });
});
