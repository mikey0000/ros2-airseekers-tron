import {act, renderHook} from "@testing-library/react";
import {describe, expect, it} from "vitest";
import {useStuckReset} from "./useStuckReset.ts";

describe("useStuckReset", () => {
    it("shares the Reset between the banner and the toolbar, and forgets it after the latch", () => {
        const banner = renderHook(({s}) => useStuckReset(s), {initialProps: {s: "STUCK_NEEDS_HELP"}});
        const toolbar = renderHook(({s}) => useStuckReset(s), {initialProps: {s: "STUCK_NEEDS_HELP"}});
        expect(toolbar.result.current[0]).toBe(false);
        act(() => banner.result.current[1]());
        expect(toolbar.result.current[0]).toBe(true);
        banner.rerender({s: "IDLE"});
        toolbar.rerender({s: "IDLE"});
        expect(toolbar.result.current[0]).toBe(false);
        banner.rerender({s: "STUCK_NEEDS_HELP"});
        toolbar.rerender({s: "STUCK_NEEDS_HELP"});
        expect(toolbar.result.current[0]).toBe(false);
    });
});
