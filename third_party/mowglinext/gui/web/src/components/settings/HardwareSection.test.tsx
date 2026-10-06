import { describe, expect, it, vi } from "vitest";
import { fireEvent, render, screen } from "@testing-library/react";
import { ThemeProvider } from "../../theme/ThemeContext.tsx";
import { HardwareSection } from "./HardwareSection.tsx";
import { MOWER_MODELS } from "../../constants/mowerModels.ts";

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
