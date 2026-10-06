import {render, screen} from "@testing-library/react";
import {describe, expect, it, vi} from "vitest";
import {AreaSettingsDrawer} from "./AreaSettingsDrawer.tsx";

vi.mock("../../hooks/useIsMobile.ts", () => ({useIsMobile: () => true}));
const api = vi.hoisted(() => ({request: async () => ({data: {supported: true, settings: {}}})}));
vi.mock("../../hooks/useApi.ts", () => ({useApi: () => api}));
vi.mock("../../hooks/useTopic.ts", () => ({useTopic: (_: string, initial: unknown) => ({data: initial})}));
vi.mock("react-i18next", () => ({useTranslation: () => ({t: (k: string) => k})}));
vi.mock("../../theme/ThemeContext.tsx", () => ({useThemeMode: () => ({colors: {}})}));

describe("AreaSettingsDrawer (mobile)", () => {
    it("renders the settings form with path width in a bottom drawer", async () => {
        render(<AreaSettingsDrawer target={0} title="Area 1" onClose={() => {}}/>);
        expect(await screen.findByTestId("area-settings-form")).toBeTruthy();
        expect(screen.getByTestId("swath-width")).toBeTruthy();
        expect(document.querySelector(".ant-drawer-bottom")).not.toBeNull();
    });
});
