import {describe, expect, it, vi} from "vitest";
import {fireEvent, render, screen} from "@testing-library/react";
import {PathSheet} from "./PathSheet.tsx";
import en from "../../../i18n/locales/en.json";

describe("PathSheet (F14)", () => {
    it("shows the path, its width and length, and no mow settings", () => {
        const onEdit = vi.fn();
        render(<PathSheet open title="Side gate" widthM={0.8} lengthM={12.34} onClose={vi.fn()} onEdit={onEdit}/>);
        expect(screen.getByTestId("path-name")).toHaveTextContent("Side gate");
        expect(screen.getByTestId("path-width")).toHaveTextContent("0.8 m");
        expect(screen.getByTestId("path-length")).toHaveTextContent("12.3 m");
        expect(screen.getByText(en.mapPath.sheetHint)).toBeInTheDocument();
        expect(screen.queryByText(en.mapPath.notMowedHint)).not.toBeInTheDocument();
        fireEvent.click(screen.getByTestId("path-edit"));
        expect(onEdit).toHaveBeenCalled();
    });

    it("a plain navigation polygon has no width, length or edit button", () => {
        render(<PathSheet open title="Navigation area 1" widthM={null} lengthM={null} onClose={vi.fn()}/>);
        expect(screen.queryByTestId("path-width")).not.toBeInTheDocument();
        expect(screen.queryByTestId("path-edit")).not.toBeInTheDocument();
    });
});
