import {describe, expect, it} from "vitest";
import en from "../i18n/locales/en.json";
import {GnssStatusConstants} from "../types/ros.ts";
import type {GnssStatus} from "../types/ros.ts";
import {
    correctionAgeLabel,
    correctionSummaryLabel,
    correctionTransportLabel,
    deriveCorrectionSummary,
    isLoraCorrectionsSupported,
} from "./gpsStatus.ts";
import {
    loraWaitingLiveSample,
    noCorrectionsSample,
    ntripStreamingSample,
} from "../test/gnssCorrectionSamples.ts";

const TYPED =
    GnssStatusConstants.CAP_CORRECTIONS_ACTIVE |
    GnssStatusConstants.CAP_CORRECTION_AGE |
    GnssStatusConstants.CAP_CORRECTION_TRANSPORT |
    GnssStatusConstants.CAP_CORRECTION_FLOW;

describe("deriveCorrectionSummary", () => {
    it("decodes the live LoRa sample: source lora, flow waiting, transport not observable, age unknown", () => {
        const summary = deriveCorrectionSummary(loraWaitingLiveSample);
        expect(summary).toEqual({
            origin: "typed",
            source: "lora",
            sourceRaw: "lora",
            state: "waiting",
            tone: "warning",
            streamStatus: undefined,
            flowStatus: GnssStatusConstants.CORRECTION_FLOW_STATUS_WAITING,
            transportStatus: undefined,
            transportUnobservable: true,
            active: false,
            ageS: undefined,
        });
        expect(correctionSummaryLabel(summary)).toBe(`${en.corrections.source.lora} · ${en.corrections.state.waiting}`);
        expect(correctionTransportLabel(summary)).toBe(en.corrections.transport.radioNotObservable);
        expect(correctionAgeLabel(summary)).toBe(en.corrections.ageUnknown);
        expect(isLoraCorrectionsSupported(loraWaitingLiveSample)).toBe(true);
    });

    it("ignores correction_stream_status when CAP_CORRECTION_STREAM is not set", () => {
        const summary = deriveCorrectionSummary({
            ...loraWaitingLiveSample,
            correction_stream_status: GnssStatusConstants.CORRECTION_STREAM_STATUS_ERROR,
        });
        expect(summary.state).toBe("waiting");
        expect(summary.streamStatus).toBeUndefined();
    });

    it("reports an NTRIP caster streaming valid RTCM as active with transport and age", () => {
        const summary = deriveCorrectionSummary(ntripStreamingSample);
        expect(summary).toMatchObject({
            origin: "typed",
            source: "ntrip",
            state: "active",
            tone: "success",
            transportStatus: GnssStatusConstants.CORRECTION_TRANSPORT_STATUS_STREAMING,
            transportUnobservable: false,
            active: true,
            ageS: 0.8,
        });
        expect(correctionSummaryLabel(summary)).toBe(`${en.corrections.source.ntrip} · ${en.corrections.state.active}`);
        expect(correctionTransportLabel(summary)).toBe(en.corrections.transport.streaming);
        expect(correctionAgeLabel(summary)).toBe("0.8 s");
        expect(isLoraCorrectionsSupported(ntripStreamingSample)).toBe(false);
    });

    it("reports a configured no-corrections source as disabled and neutral", () => {
        const summary = deriveCorrectionSummary(noCorrectionsSample);
        expect(summary).toMatchObject({origin: "typed", source: "none", state: "disabled", tone: "neutral"});
        expect(correctionSummaryLabel(summary)).toBe(en.corrections.noneConfigured);
        expect(isLoraCorrectionsSupported(noCorrectionsSample)).toBe(false);
    });

    it("reports nothing for a sample without any correction capability", () => {
        const summary = deriveCorrectionSummary({fix_type: GnssStatusConstants.FIX_TYPE_GPS_FIX, fix_valid: true});
        expect(summary).toMatchObject({origin: "none", source: "unknown", state: "unknown", tone: "neutral"});
        expect(correctionSummaryLabel(summary)).toBe(en.corrections.state.unknown);
        expect(deriveCorrectionSummary(undefined).origin).toBe("none");
    });

    it.each([
        [GnssStatusConstants.CORRECTION_FLOW_STATUS_IDLE, "idle", "warning"],
        [GnssStatusConstants.CORRECTION_FLOW_STATUS_STALE, "stale", "error"],
        [GnssStatusConstants.CORRECTION_FLOW_STATUS_INVALID, "invalid", "error"],
        [GnssStatusConstants.CORRECTION_FLOW_STATUS_UNKNOWN, "unknown", "neutral"],
    ] as const)("maps flow %s to %s/%s", (flow, state, tone) => {
        const summary = deriveCorrectionSummary({
            ...loraWaitingLiveSample,
            correction_flow_status: flow,
        });
        expect(summary.state).toBe(state);
        expect(summary.tone).toBe(tone);
    });

    it("shows the age in a stale LoRa label", () => {
        const summary = deriveCorrectionSummary({
            ...loraWaitingLiveSample,
            correction_flow_status: GnssStatusConstants.CORRECTION_FLOW_STATUS_STALE,
            correction_age_s: 12.4,
            value_flags: (loraWaitingLiveSample.value_flags ?? 0) | GnssStatusConstants.CAP_CORRECTION_AGE,
        });
        expect(summary.ageS).toBe(12.4);
        expect(correctionSummaryLabel(summary)).toBe(`${en.corrections.source.lora} · data stale (age 12 s)`);
    });

    it("lets a failed NTRIP transport override a waiting flow", () => {
        const summary = deriveCorrectionSummary({
            correction_source: "ntrip",
            correction_flow_status: GnssStatusConstants.CORRECTION_FLOW_STATUS_WAITING,
            correction_transport_status: GnssStatusConstants.CORRECTION_TRANSPORT_STATUS_FAILED,
            capability_flags: TYPED,
            value_flags: GnssStatusConstants.CAP_CORRECTION_FLOW | GnssStatusConstants.CAP_CORRECTION_TRANSPORT,
        });
        expect(summary.state).toBe("error");
        expect(summary.tone).toBe("error");
        expect(correctionTransportLabel(summary)).toBe(en.corrections.transport.failed);
    });

    it("falls back to corrections_active when flow has no value", () => {
        const base: GnssStatus = {
            correction_source: "ntrip",
            capability_flags: TYPED,
            value_flags: GnssStatusConstants.CAP_CORRECTIONS_ACTIVE,
        };
        expect(deriveCorrectionSummary({...base, corrections_active: true}).state).toBe("active");
        expect(deriveCorrectionSummary({...base, corrections_active: false}).state).toBe("inactive");
    });

    it("keeps an unrecognised source string as-is", () => {
        const summary = deriveCorrectionSummary({
            ...ntripStreamingSample,
            correction_source: "serial_radio",
        });
        expect(summary.source).toBe("other");
        expect(correctionSummaryLabel(summary)).toBe(`serial_radio · ${en.corrections.state.active}`);
    });

    it("uses the legacy stream status only when CAP_CORRECTION_STREAM is set", () => {
        const legacy: GnssStatus = {
            correction_stream_status: GnssStatusConstants.CORRECTION_STREAM_STATUS_UNAVAILABLE,
        };
        expect(deriveCorrectionSummary(legacy).origin).toBe("none");

        const summary = deriveCorrectionSummary({
            ...legacy,
            capability_flags: GnssStatusConstants.CAP_CORRECTION_STREAM,
            value_flags: GnssStatusConstants.CAP_CORRECTION_STREAM,
        });
        expect(summary).toMatchObject({origin: "stream", state: "unavailable", tone: "error"});
        expect(correctionSummaryLabel(summary)).toBe(en.gpsStatus.correctionStreamUnavailable);
    });

    it("falls back to the legacy stream when the typed fields carry no value", () => {
        const summary = deriveCorrectionSummary({
            correction_stream_status: GnssStatusConstants.CORRECTION_STREAM_STATUS_ACTIVE,
            capability_flags: TYPED | GnssStatusConstants.CAP_CORRECTION_STREAM,
            value_flags: GnssStatusConstants.CAP_CORRECTION_STREAM,
        });
        expect(summary).toMatchObject({origin: "typed", state: "active", tone: "success"});
    });
});
