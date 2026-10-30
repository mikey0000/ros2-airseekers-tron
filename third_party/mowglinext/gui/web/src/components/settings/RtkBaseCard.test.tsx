import { App } from "antd";
import { fireEvent, render, screen, waitFor } from "@testing-library/react";
import { beforeEach, describe, expect, it, vi } from "vitest";
import en from "../../i18n/locales/en.json";
import type { RtkBaseStatus } from "../../utils/rtkBase.ts";
import { RtkBaseCard } from "./RtkBaseCard.tsx";

const request = vi.fn();
let liveStatus: RtkBaseStatus | null = null;

vi.mock("../../hooks/useApi.ts", () => ({ useApi: () => ({ request }) }));
vi.mock("../../hooks/useTopic.ts", () => ({ useTopic: () => ({ data: liveStatus, lastMessageAt: null }) }));

const pairedStatus: RtkBaseStatus = {
    pairing: { addr: 38183, addr_hex: "9527", channel: 40, band: "2", band_name: "Australia/NZ 927 MHz" },
    ref_station: "FINE", nrtk_on: false, station_id: "0001", diff_age_s: 1.04,
    fix_type: "RTK_FIXED", correction_source: "lora",
};

const renderCard = (ntripEnabled = false) =>
    render(<App><RtkBaseCard ntripEnabled={ntripEnabled} /></App>);

describe("RtkBaseCard", () => {
    beforeEach(() => {
        request.mockReset();
        liveStatus = null;
    });

    it("shows the stored pairing and live status", () => {
        liveStatus = pairedStatus;
        renderCard();

        expect(screen.getByText(en.rtkBase.title)).toBeInTheDocument();
        expect(screen.getByTestId("rtk-base-stored")).toHaveTextContent(
            "address 38183 (0x9527) · channel 40 · Australia / NZ 927 MHz");
        expect(screen.getByText(en.rtkBase.refFine)).toBeInTheDocument();
        expect(screen.getByText("0001")).toBeInTheDocument();
        expect(screen.getByText("1.0 s")).toBeInTheDocument();
        expect(screen.getByText("RTK_FIXED")).toBeInTheDocument();
        expect(screen.queryByText(en.rtkBase.ntripOwns)).not.toBeInTheDocument();
        // The form is pre-filled from the stored pairing (band 2 -> region 03).
        expect(screen.getByPlaceholderText("38183 / 0x9527")).toHaveValue("38183");
    });

    it("warns when NTRIP owns the receiver", () => {
        liveStatus = { ...pairedStatus, nrtk_on: true, ref_station: "FAIL" };
        renderCard();
        expect(screen.getByText(en.rtkBase.ntripOwns)).toBeInTheDocument();
        expect(screen.getByText(en.rtkBase.refFail)).toBeInTheDocument();
    });

    it("warns when Enable NTRIP is on even before the driver reports", () => {
        renderCard(true);
        expect(screen.getByText(en.rtkBase.notReadYet)).toBeInTheDocument();
        expect(screen.getByText(en.rtkBase.ntripOwns)).toBeInTheDocument();
    });

    it("pairs the universal code after confirmation", async () => {
        request.mockResolvedValue({ data: { success: true } });
        renderCard();

        fireEvent.click(screen.getByText(en.rtkBase.useUniversal));
        expect(screen.getByPlaceholderText("38183 / 0x9527")).toHaveValue("38183");
        fireEvent.click(screen.getByRole("button", { name: en.rtkBase.pair }));

        expect((await screen.findAllByText(en.rtkBase.confirmTitle)).length).toBeGreaterThan(0);
        const buttons = screen.getAllByRole("button", { name: en.rtkBase.pair });
        fireEvent.click(buttons[buttons.length - 1]);

        await waitFor(() => expect(request).toHaveBeenCalledTimes(1));
        expect(request.mock.calls[0][0]).toMatchObject({
            path: "/rtk-base/pair", method: "POST", body: { addr: 38183, channel: 40, area: "02" },
        });
        expect(await screen.findByText(en.rtkBase.pairSuccess)).toBeInTheDocument();
    });

    it("accepts a hex address and does not call the API for an invalid one", async () => {
        renderCard();
        const addr = screen.getByPlaceholderText("38183 / 0x9527");
        fireEvent.change(addr, { target: { value: "0xFFFF" } });
        fireEvent.click(screen.getByRole("button", { name: en.rtkBase.pair }));
        expect(await screen.findByText(en.rtkBase.addressInvalid)).toBeInTheDocument();
        expect(screen.queryByText(en.rtkBase.confirmTitle)).not.toBeInTheDocument();
        expect(request).not.toHaveBeenCalled();
    });

    it("reads the pairing from the board", async () => {
        request.mockResolvedValue({ data: { ok: true, pairing: { addr: 1, addr_hex: "0001", channel: 5, band: "0" } } });
        renderCard();

        fireEvent.click(screen.getByText(en.rtkBase.readFromBoard));

        await waitFor(() => expect(request).toHaveBeenCalledTimes(1));
        expect(request.mock.calls[0][0]).toMatchObject({ path: "/rtk-base/pairing", method: "GET" });
        expect(await screen.findByTestId("rtk-base-stored")).toHaveTextContent(
            "address 1 (0x0001) · channel 5 · Europe 863 MHz");
    });

    it("reports a refused pairing", async () => {
        request.mockResolvedValue({ data: { success: false, message: "board did not store it" } });
        renderCard();
        fireEvent.change(screen.getByPlaceholderText("38183 / 0x9527"), { target: { value: "0x1234" } });
        fireEvent.change(screen.getByRole("spinbutton"), { target: { value: "12" } });
        fireEvent.click(screen.getByRole("button", { name: en.rtkBase.pair }));
        await screen.findAllByText(en.rtkBase.confirmTitle);
        const buttons = screen.getAllByRole("button", { name: en.rtkBase.pair });
        fireEvent.click(buttons[buttons.length - 1]);

        await waitFor(() => expect(request).toHaveBeenCalledTimes(1));
        expect((request.mock.calls[0][0] as { body: unknown }).body).toEqual({ addr: 0x1234, channel: 12, area: "02" });
        expect(await screen.findByText(/board did not store it/)).toBeInTheDocument();
    });
});
