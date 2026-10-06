import { App } from "antd";
import { render, screen } from "@testing-library/react";
import type { ComponentProps } from "react";
import { describe, expect, it } from "vitest";
import en from "../../i18n/locales/en.json";
import { gnssStatusSamples } from "../../test/mocks.tsx";
import { loraWaitingLiveSample, ntripStreamingSample } from "../../test/gnssCorrectionSamples.ts";
import { GnssLiveStatusSummaryCard } from "./GnssLiveStatusSummaryCard.tsx";

const renderCard = (
    gnssStatus: ComponentProps<typeof GnssLiveStatusSummaryCard>["gnssStatus"],
    selectedReceiverFamily: unknown = "auto",
) => render(
    <App>
        <GnssLiveStatusSummaryCard
            gnssStatus={gnssStatus}
            selectedReceiverFamily={selectedReceiverFamily}
        />
    </App>,
);

describe("GnssLiveStatusSummaryCard", () => {
    it("keeps Settings focused on a compact summary when runtime GNSS details are missing", () => {
        renderCard(undefined, "auto");

        expect(screen.getByText(en.settingsGnssLiveStatus.cardTitle)).toBeInTheDocument();
        expect(screen.getByText(en.gpsStatus.noGps)).toBeInTheDocument();
        expect(screen.getAllByText(en.settingsGnssLiveStatus.unknown).length).toBeGreaterThan(0);
        expect(screen.getByText("auto")).toBeInTheDocument();
        expect(screen.queryByText(en.settingsGnssLiveStatus.satelliteSectionTitle)).not.toBeInTheDocument();
        expect(screen.queryByText(en.settingsGnssLiveStatus.signalQualitySectionTitle)).not.toBeInTheDocument();
        expect(screen.queryByText(en.settingsGnssLiveStatus.correctionMsmSectionTitle)).not.toBeInTheDocument();
        expect(screen.queryByText(en.settingsGnssLiveStatus.receiverConfigSectionTitle)).not.toBeInTheDocument();
    });

    it("shows detected receiver, RTK, and correction stream without exposing the full diagnostics panel", () => {
        renderCard(gnssStatusSamples.correction_stream_active);

        expect(screen.getByText("u-blox F9P")).toBeInTheDocument();
        expect(screen.getAllByText(en.gpsStatus.rtkFloat).length).toBeGreaterThan(0);
        expect(screen.getAllByText(en.gpsStatus.correctionStreamActive).length).toBeGreaterThan(0);
        expect(screen.queryByText(en.settingsGnssLiveStatus.satelliteSectionTitle)).not.toBeInTheDocument();
        expect(screen.queryByText(en.settingsGnssLiveStatus.signalQualitySectionTitle)).not.toBeInTheDocument();
    });

    it("shows the LoRa source, waiting flow (amber), unobservable radio link and unknown age", () => {
        renderCard(loraWaitingLiveSample);

        expect(screen.getByText(en.corrections.label)).toBeInTheDocument();
        const tag = screen.getByTestId("gnss-corrections-tag");
        expect(tag).toHaveTextContent(`${en.corrections.source.lora} · ${en.corrections.state.waiting}`);
        expect(tag.className).toContain("warning");
        expect(screen.getByTestId("gnss-corrections-detail")).toHaveTextContent(
            `${en.corrections.transport.radioNotObservable} · ${en.corrections.age} ${en.corrections.ageUnknown}`,
        );
        expect(screen.queryByText(en.settingsGnssLiveStatus.correctionStream)).not.toBeInTheDocument();
        expect(screen.getByText(en.settingsGnssLiveStatus.rtkModeNone)).toBeInTheDocument();
    });

    it("shows NTRIP transport and age when streaming", () => {
        renderCard(ntripStreamingSample);

        const tag = screen.getByTestId("gnss-corrections-tag");
        expect(tag).toHaveTextContent(`${en.corrections.source.ntrip} · ${en.corrections.state.active}`);
        expect(tag.className).toContain("success");
        expect(screen.getByTestId("gnss-corrections-detail")).toHaveTextContent(
            `${en.corrections.transport.streaming} · ${en.corrections.age} 0.8 s`,
        );
    });
});
