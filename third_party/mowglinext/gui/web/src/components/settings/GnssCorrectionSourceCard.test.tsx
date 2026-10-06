import { App } from "antd";
import { fireEvent, render, screen, waitFor } from "@testing-library/react";
import { beforeEach, describe, expect, it, vi } from "vitest";
import en from "../../i18n/locales/en.json";
import type { GnssStatus } from "../../types/ros.ts";
import { deriveCorrectionSummary } from "../../utils/gpsStatus.ts";
import {
    loraWaitingLiveSample,
    noCorrectionsSample,
    ntripStreamingSample,
} from "../../test/gnssCorrectionSamples.ts";
import { GnssCorrectionSourceCard } from "./GnssCorrectionSourceCard.tsx";
import { NtripSection } from "./NtripSection.tsx";

const request = vi.fn();
let gnssStatus: GnssStatus = {};

vi.mock("../../hooks/useApi.ts", () => ({ useApi: () => ({ request }) }));
vi.mock("../../hooks/useGnssStatus.ts", () => ({ useGnssStatus: () => gnssStatus }));
vi.mock("../../hooks/useIsMobile.ts", () => ({ useIsMobile: () => false }));
vi.mock("../../theme/ThemeContext.tsx", () => ({
    useThemeMode: () => ({ colors: { text: "#000", textMuted: "#666", primary: "#1677ff" } }),
}));
vi.mock("./NtripStationMap.tsx", () => ({ NtripStationMap: () => null }));

describe("GnssCorrectionSourceCard", () => {
    beforeEach(() => {
        request.mockReset();
    });

    it("shows flow, age and the unobservable radio link for the live LoRa sample", () => {
        render(<App><GnssCorrectionSourceCard summary={deriveCorrectionSummary(loraWaitingLiveSample)} /></App>);

        expect(screen.getByText(en.corrections.lora.title)).toBeInTheDocument();
        expect(screen.getByText(en.corrections.state.waiting)).toBeInTheDocument();
        expect(screen.getByText(en.corrections.ageUnknown)).toBeInTheDocument();
        expect(screen.getByText(en.corrections.transport.radioNotObservable)).toBeInTheDocument();
    });

    it("posts the pairing form to the corrections API and reports acceptance", async () => {
        request.mockResolvedValue({ data: { success: true } });
        render(<App><GnssCorrectionSourceCard summary={deriveCorrectionSummary(loraWaitingLiveSample)} /></App>);

        fireEvent.click(screen.getByRole("button", { name: en.corrections.lora.pairButton }));
        fireEvent.change(await screen.findByLabelText(en.corrections.lora.sn), { target: { value: " 3001034903AB " } });
        fireEvent.change(screen.getByLabelText(en.corrections.lora.addr), { target: { value: "4660" } });
        fireEvent.change(screen.getByLabelText(en.corrections.lora.channel), { target: { value: "17" } });
        fireEvent.click(screen.getByRole("button", { name: en.corrections.lora.pairSubmit }));

        await waitFor(() => expect(request).toHaveBeenCalledTimes(1));
        expect(request.mock.calls[0][0]).toMatchObject({
            path: "/corrections/lora/pair",
            method: "POST",
            body: { sn: "3001034903AB", addr: 4660, channel: 17, area: "" },
        });
        expect(await screen.findByText(en.corrections.lora.pairSuccess)).toBeInTheDocument();
    });

    it("explains a refusal from the driver", async () => {
        request.mockResolvedValue({ data: { success: false } });
        render(<App><GnssCorrectionSourceCard summary={deriveCorrectionSummary(loraWaitingLiveSample)} /></App>);

        fireEvent.click(screen.getByRole("button", { name: en.corrections.lora.pairButton }));
        fireEvent.change(await screen.findByLabelText(en.corrections.lora.sn), { target: { value: "SN1" } });
        fireEvent.click(screen.getByRole("button", { name: en.corrections.lora.pairSubmit }));

        expect(await screen.findByText(en.corrections.lora.pairRejected)).toBeInTheDocument();
    });
});

describe("NtripSection correction-source gating", () => {
    beforeEach(() => {
        request.mockReset();
        request.mockResolvedValue({ data: { stations: [] } });
    });

    it("shows the LoRa card and hides the NTRIP editor when the live source is LoRa", () => {
        gnssStatus = loraWaitingLiveSample;
        render(<App><NtripSection values={{}} onChange={() => {}} /></App>);

        expect(screen.getByTestId("lora-correction-card")).toBeInTheDocument();
        expect(screen.getByText(en.corrections.lora.ntripHidden)).toBeInTheDocument();
        expect(screen.queryByText(en.settingsNtrip.title)).not.toBeInTheDocument();

        fireEvent.click(screen.getByRole("button", { name: en.corrections.lora.showNtrip }));
        expect(screen.getByText(en.settingsNtrip.title)).toBeInTheDocument();
    });

    it.each([
        ["ntrip", ntripStreamingSample],
        ["none", noCorrectionsSample],
        ["unreported", {} as GnssStatus],
    ])("keeps the stock NTRIP editor when the source is %s", (_name, sample) => {
        gnssStatus = sample;
        render(<App><NtripSection values={{}} onChange={() => {}} /></App>);

        expect(screen.getByText(en.settingsNtrip.title)).toBeInTheDocument();
        expect(screen.queryByTestId("lora-correction-card")).not.toBeInTheDocument();
    });
});
