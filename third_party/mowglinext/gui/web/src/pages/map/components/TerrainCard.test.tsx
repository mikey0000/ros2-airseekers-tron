import {App} from "antd";
import {fireEvent, render, screen} from "@testing-library/react";
import {describe, expect, it, vi} from "vitest";
import {TerrainCard, terrainKindText} from "./TerrainCard.tsx";
import type {TerrainSummary} from "../../../utils/terrain.ts";

const SUMMARY: TerrainSummary = {
    areas: [{
        area: "", area_index: 0, slope: null,
        clusters: [{id: 1, kinds: {follow_abort: 1, spin_drift: 2}, n: 3, weight: 1.4, x: 0, y: 0, status: "open",
            last_t: Date.now() / 1000 - 240}],
    }],
};

const renderCard = (onDismiss = vi.fn()) => {
    render(
        <App>
            <div data-testid="map">map</div>
            <TerrainCard summary={SUMMARY} selected={null} onSelect={vi.fn()} onAction={vi.fn()} onDismiss={onDismiss}/>
        </App>,
    );
    return onDismiss;
};

const tap = (el: Element, from = {x: 10, y: 10}, to = from) => {
    fireEvent.pointerDown(el, {clientX: from.x, clientY: from.y});
    fireEvent.pointerUp(el, {clientX: to.x, clientY: to.y});
};

describe("TerrainCard", () => {
    it("describes clusters in words", () => {
        renderCard();
        const row = screen.getByTestId("terrain-cluster");
        expect(row.textContent).toContain("Area 1 · 3 incidents · 4 min ago");
        expect(row.textContent).toContain("couldn't follow the path, wheels slipped while turning (2×)");
        expect(row.textContent).not.toMatch(/n=|w=/);
        expect(terrainKindText("some_new_kind")).toBe("some new kind");
    });

    it("closes from its ✕, on Escape and on a tap outside", () => {
        const onDismiss = renderCard();
        fireEvent.click(screen.getByRole("button", {name: "Hide terrain card"}));
        expect(onDismiss).toHaveBeenCalledTimes(1);

        fireEvent.keyDown(document, {key: "Escape"});
        expect(onDismiss).toHaveBeenCalledTimes(2);

        tap(screen.getByTestId("map"));
        expect(onDismiss).toHaveBeenCalledTimes(3);
    });

    it("stays open for taps inside it and for map pans", () => {
        const onDismiss = renderCard();
        tap(screen.getByTestId("terrain-cluster"));
        tap(screen.getByTestId("map"), {x: 10, y: 10}, {x: 80, y: 40});
        expect(onDismiss).not.toHaveBeenCalled();
    });
});
