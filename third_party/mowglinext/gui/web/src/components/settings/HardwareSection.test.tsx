import { afterEach, describe, expect, it, vi } from "vitest";
import { fireEvent, render, screen } from "@testing-library/react";
import { ThemeProvider } from "../../theme/ThemeContext.tsx";
import { HardwareSection } from "./HardwareSection.tsx";
import { MOWER_MODELS } from "../../constants/mowerModels.ts";
import { AIRSEEKERS_TRON_PROFILE_ID, getRobotProfile } from "../../constants/robotProfiles.ts";

// Active robot profile for useProfileGates (stock unless a test switches it).
const active = vi.hoisted(() => ({ id: "YardForce500" }));
vi.mock("../../hooks/useRobotProfile.ts", async (importOriginal) => ({
    ...(await importOriginal<typeof import("../../hooks/useRobotProfile.ts")>()),
    useRobotProfile: () => ({ profile: getRobotProfile(active.id), source: "settings", loading: false }),
}));
afterEach(() => { active.id = "YardForce500"; });

function renderSection() {
    render(
        <ThemeProvider>
            <HardwareSection values={{}} onChange={vi.fn()} onBulkChange={vi.fn()} />
        </ThemeProvider>,
    );
}

describe("HardwareSection model images", () => {
    it("shows an image for every preset except CUSTOM", () => {
        renderSection();
        for (const m of MOWER_MODELS) {
            const img = screen.queryByTestId(`robot-image-${m.value}`);
            if (m.value === "CUSTOM") expect(img).toBeNull();
            else expect(img).not.toBeNull();
        }
    });

    it("tries the owner photo first, then the bundled svg, then hides the image", () => {
        renderSection();
        const id = "robot-image-AirseekersTron";
        expect(screen.getByTestId(id).getAttribute("src")).toBe("/robots/airseekers_tron.jpg");
        fireEvent.error(screen.getByTestId(id));
        expect(screen.getByTestId(id).getAttribute("src")).toBe("/robots/airseekers_tron.svg");
        fireEvent.error(screen.getByTestId(id));
        expect(screen.queryByTestId(id)).toBeNull();
    });
});

describe("HardwareSection profile gating", () => {
    it("offers every preset on a stock robot", () => {
        renderSection();
        expect(screen.getByTestId("robot-image-YardForce500")).toBeInTheDocument();
        expect(screen.getByText("Wheel Radius")).toBeInTheDocument();
    });

    it("on Tron offers only the Tron model and hides fields nothing reads", () => {
        active.id = AIRSEEKERS_TRON_PROFILE_ID;
        renderSection();
        expect(screen.getByTestId(`robot-image-${AIRSEEKERS_TRON_PROFILE_ID}`)).toBeInTheDocument();
        for (const m of MOWER_MODELS.filter((x) => x.value !== AIRSEEKERS_TRON_PROFILE_ID && x.value !== "CUSTOM")) {
            expect(screen.queryByTestId(`robot-image-${m.value}`), m.value).toBeNull();
        }
        expect(screen.queryByText("Wheel Radius")).toBeNull();
        expect(screen.queryByText("Chassis & Geometry")).toBeNull();
        expect(screen.queryByText("Not used by this robot")).toBeNull();
        expect(screen.getByText("Tool Width")).toBeInTheDocument();
    });
});
