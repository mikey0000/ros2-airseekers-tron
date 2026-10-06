import {renderHook} from "@testing-library/react";
import {beforeEach, describe, expect, it, vi} from "vitest";

const {request, api} = vi.hoisted(() => {
    const request = vi.fn();
    return {request, api: {request: (...args: unknown[]): unknown => request(...args)}};
});
vi.mock("./useApi.ts", () => ({useApi: () => api}));
vi.mock("react-i18next", () => ({useTranslation: () => ({t: (key: string) => `t:${key}`})}));

import {resetRobotProfileCache} from "./useRobotProfile.ts";
import {useRobotName} from "./useRobotName.ts";

describe("useRobotName", () => {
    beforeEach(() => {
        resetRobotProfileCache();
        request.mockReset();
        request.mockReturnValue(new Promise(() => {}));
    });

    it("is the translated label of the active profile (stock profile while loading)", () => {
        const {result} = renderHook(() => useRobotName());
        expect(result.current).toBe("t:mowerModels.YardForce500.label");
    });
});
