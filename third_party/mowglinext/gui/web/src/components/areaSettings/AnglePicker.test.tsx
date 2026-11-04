import {describe, expect, it, vi} from "vitest";
import {render, screen} from "@testing-library/react";
import {AnglePicker} from "./AnglePicker.tsx";

/** Handle position relative to the 132 px picker's centre. */
function handleOffset(): {dx: number; dy: number} {
    const circles = screen.getByRole("slider").querySelectorAll("circle");
    const h = circles[circles.length - 2]; // the last one is the centre dot
    return {dx: Number(h.getAttribute("cx")) - 66, dy: Number(h.getAttribute("cy")) - 66};
}

describe("AnglePicker uses the planner's angle convention", () => {
    it("draws 0° pointing east and 90° pointing north", () => {
        const {rerender} = render(<AnglePicker value={0} onChange={vi.fn()}/>);
        let o = handleOffset();
        expect(o.dx).toBeGreaterThan(50);
        expect(Math.abs(o.dy)).toBeLessThan(1e-6);
        rerender(<AnglePicker value={90} onChange={vi.fn()}/>);
        o = handleOffset();
        expect(Math.abs(o.dx)).toBeLessThan(1e-6);
        expect(o.dy).toBeLessThan(-50);
    });
});
