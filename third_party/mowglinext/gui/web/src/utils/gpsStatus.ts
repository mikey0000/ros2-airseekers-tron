import {GnssStatus, GnssStatusConstants} from "../types/ros.ts";
import i18n from "../i18n";

export type GpsFixType = "RTK_FIX" | "RTK_FLOAT" | "GPS_FIX" | "NO_FIX";
export type OptionalGnssBooleanState = "unsupported" | "unknown" | "false" | "true";

export interface GpsStatus {
    fixType: GpsFixType;
    label: string;
    percent: number;
}

export type GnssRtkModeLabel = "Unknown" | "None" | "Float" | "Fixed";

const CORRECTION_STREAM_DIAGNOSTIC_NAMES = [
    "universal_gnss_ntrip/rtcm_forwarding",
    "universal_gnss/rtcm_forwarding",
];

const MSM_SUMMARY_DIAGNOSTIC_NAMES = [
    "universal_gnss_ntrip/rtcm_semantic/msm_summary",
    "universal_gnss/rtcm_semantic/msm_summary",
];

export interface DiagnosticStatusLike {
    name?: string;
    hardware_id?: string;
    message?: string;
    values?: { key?: string; value?: string }[];
}

export interface DiagnosticArrayLike {
    status?: DiagnosticStatusLike[];
}

function fromFixType(fixType: number | undefined | null): GpsStatus | null {
    switch (fixType) {
        case GnssStatusConstants.FIX_TYPE_RTK_FIXED:
            return {fixType: "RTK_FIX", label: i18n.t("gpsStatus.rtkFixed"), percent: 100};
        case GnssStatusConstants.FIX_TYPE_RTK_FLOAT:
            return {fixType: "RTK_FLOAT", label: i18n.t("gpsStatus.rtkFloat"), percent: 50};
        case GnssStatusConstants.FIX_TYPE_GPS_FIX:
            return {fixType: "GPS_FIX", label: i18n.t("gpsStatus.gpsFix"), percent: 25};
        case GnssStatusConstants.FIX_TYPE_DEAD_RECKONING:
            return {fixType: "NO_FIX", label: i18n.t("gpsStatus.deadReckoning"), percent: 10};
        case GnssStatusConstants.FIX_TYPE_NO_FIX:
            return {fixType: "NO_FIX", label: i18n.t("gpsStatus.noGps"), percent: 0};
        default:
            return null;
    }
}

// Source of truth: GnssStatus from /gps/status.
export function deriveGpsStatus(gnssStatus: GnssStatus | undefined | null): GpsStatus {
    if (gnssStatus?.fix_valid === false) {
        return {fixType: "NO_FIX", label: i18n.t("gpsStatus.noGps"), percent: 0};
    }

    if (gnssStatus?.rtk_mode === GnssStatusConstants.RTK_MODE_FIXED) {
        return {fixType: "RTK_FIX", label: i18n.t("gpsStatus.rtkFixed"), percent: 100};
    }

    if (gnssStatus?.rtk_mode === GnssStatusConstants.RTK_MODE_FLOAT) {
        return {fixType: "RTK_FLOAT", label: i18n.t("gpsStatus.rtkFloat"), percent: 50};
    }

    const fromTypedStatus = fromFixType(gnssStatus?.fix_type);
    if (fromTypedStatus && (fromTypedStatus.fixType !== "NO_FIX" || gnssStatus?.fix_valid !== true)) {
        return fromTypedStatus;
    }

    if (gnssStatus?.fix_valid === true) {
        return {fixType: "GPS_FIX", label: i18n.t("gpsStatus.gpsFix"), percent: 25};
    }

    return {fixType: "NO_FIX", label: i18n.t("gpsStatus.noGps"), percent: 0};
}

export function gnssRtkModeLabel(gnssStatus: GnssStatus | undefined | null): GnssRtkModeLabel | undefined {
    switch (gnssStatus?.rtk_mode) {
        case GnssStatusConstants.RTK_MODE_NONE:
            return "None";
        case GnssStatusConstants.RTK_MODE_FLOAT:
            return "Float";
        case GnssStatusConstants.RTK_MODE_FIXED:
            return "Fixed";
        case GnssStatusConstants.RTK_MODE_UNKNOWN:
            return "Unknown";
        default:
            return undefined;
    }
}

export function gnssBaselineSolutionStatusLabel(gnssStatus: GnssStatus | undefined | null): string | undefined {
    switch (gnssStatus?.baseline_solution_status) {
        case GnssStatusConstants.BASELINE_STATUS_UNKNOWN:
            return i18n.t("gpsStatus.baselineUnknown");
        case GnssStatusConstants.BASELINE_STATUS_COMPUTED:
            return i18n.t("gpsStatus.baselineComputed");
        case GnssStatusConstants.BASELINE_STATUS_NOT_SOLVED:
            return i18n.t("gpsStatus.baselineNotSolved");
        case GnssStatusConstants.BASELINE_STATUS_INSUFFICIENT_OBSERVATIONS:
            return i18n.t("gpsStatus.baselineInsufficientObservations");
        case GnssStatusConstants.BASELINE_STATUS_NO_CONVERGENCE:
            return i18n.t("gpsStatus.baselineNoConvergence");
        case GnssStatusConstants.BASELINE_STATUS_OUT_OF_TOLERANCE:
            return i18n.t("gpsStatus.baselineOutOfTolerance");
        case GnssStatusConstants.BASELINE_STATUS_COVARIANCE_TRACE_EXCEEDED:
            return i18n.t("gpsStatus.baselineCovarianceTraceExceeded");
        case GnssStatusConstants.BASELINE_STATUS_NOT_CONFIGURED:
            return i18n.t("gpsStatus.baselineNotConfigured");
        default:
            return undefined;
    }
}

export function gnssCorrectionStreamStatusLabel(gnssStatus: GnssStatus | undefined | null): string | undefined {
    return correctionStreamStatusLabel(gnssStatus?.correction_stream_status);
}

function correctionStreamStatusLabel(status: number | undefined): string | undefined {
    switch (status) {
        case GnssStatusConstants.CORRECTION_STREAM_STATUS_UNKNOWN:
            return i18n.t("gpsStatus.correctionStreamUnknown");
        case GnssStatusConstants.CORRECTION_STREAM_STATUS_IDLE:
            return i18n.t("gpsStatus.correctionStreamIdle");
        case GnssStatusConstants.CORRECTION_STREAM_STATUS_WAITING:
            return i18n.t("gpsStatus.correctionStreamWaiting");
        case GnssStatusConstants.CORRECTION_STREAM_STATUS_ACTIVE:
            return i18n.t("gpsStatus.correctionStreamActive");
        case GnssStatusConstants.CORRECTION_STREAM_STATUS_UNAVAILABLE:
            return i18n.t("gpsStatus.correctionStreamUnavailable");
        case GnssStatusConstants.CORRECTION_STREAM_STATUS_ERROR:
            return i18n.t("gpsStatus.correctionStreamError");
        default:
            return undefined;
    }
}

export function hasTypedGnssStatusSample(gnssStatus: GnssStatus | undefined | null): boolean {
    return gnssStatus?.fix_type !== undefined ||
        gnssStatus?.fix_valid !== undefined ||
        (gnssStatus?.backend?.trim().length ?? 0) > 0 ||
        (gnssStatus?.receiver_vendor?.trim().length ?? 0) > 0 ||
        (gnssStatus?.receiver_model?.trim().length ?? 0) > 0;
}

export function diagnosticsValueMap(entry: DiagnosticStatusLike | undefined): Record<string, string> {
    const out: Record<string, string> = {};
    for (const item of entry?.values ?? []) {
        const key = item.key?.trim();
        if (!key) {
            continue;
        }
        out[key] = item.value?.trim() ?? "";
    }
    return out;
}

export function parseDiagnosticBool(value: string | undefined): boolean | undefined {
    if (!value) {
        return undefined;
    }
    const normalized = value.trim().toLowerCase();
    if (normalized === "true") {
        return true;
    }
    if (normalized === "false") {
        return false;
    }
    return undefined;
}

function parseDiagnosticInt(value: string | undefined): number | undefined {
    if (!value) {
        return undefined;
    }
    const parsed = Number.parseInt(value, 10);
    return Number.isFinite(parsed) ? parsed : undefined;
}

function parseDiagnosticFloat(value: string | undefined): number | undefined {
    if (!value) {
        return undefined;
    }
    const parsed = Number.parseFloat(value);
    return Number.isFinite(parsed) ? parsed : undefined;
}

function correctionStreamStatusFromMessage(message: string | undefined): number | undefined {
    const normalized = message?.trim().toLowerCase() ?? "";
    if (!normalized) {
        return undefined;
    }
    if (normalized.includes("write error") || normalized.endsWith("error")) {
        return GnssStatusConstants.CORRECTION_STREAM_STATUS_ERROR;
    }
    if (normalized.includes("unavailable")) {
        return GnssStatusConstants.CORRECTION_STREAM_STATUS_UNAVAILABLE;
    }
    if (normalized.includes("waiting")) {
        return GnssStatusConstants.CORRECTION_STREAM_STATUS_WAITING;
    }
    if (normalized.includes("active")) {
        return GnssStatusConstants.CORRECTION_STREAM_STATUS_ACTIVE;
    }
    if (normalized.includes("idle")) {
        return GnssStatusConstants.CORRECTION_STREAM_STATUS_IDLE;
    }
    return undefined;
}

/**
 * Fix-type codes arrive over the wire as plain numbers, while
 * `GnssStatusConstants` is a const enum — comparing the two directly is
 * (correctly) rejected, since nothing proves the number is a member. Funnel
 * every such comparison through this one documented widening.
 */
export function isGnssFixType(
    value: number | undefined,
    fixType: GnssStatusConstants,
): boolean {
    return value === (fixType as number);
}

function navSatFixStatusToGnssFixType(fixStatus: number | undefined): number | undefined {
    switch (fixStatus) {
        case 2:
        case 1:
        case 0:
            return GnssStatusConstants.FIX_TYPE_GPS_FIX;
        default:
            return undefined;
    }
}

export function findDiagnosticStatusByName(
    diagnostics: DiagnosticArrayLike | undefined | null,
    name: string,
): DiagnosticStatusLike | undefined {
    return (diagnostics?.status ?? []).find((entry) => entry.name === name);
}

function findDiagnosticStatusByNames(
    diagnostics: DiagnosticArrayLike | undefined | null,
    names: string[],
): DiagnosticStatusLike | undefined {
    for (const name of names) {
        const match = findDiagnosticStatusByName(diagnostics, name);
        if (match) {
            return match;
        }
    }
    return undefined;
}

export function deriveGnssStatusFromDiagnostics(
    diagnostics: DiagnosticArrayLike | undefined | null,
): GnssStatus | undefined {
    const summary = findDiagnosticStatusByName(diagnostics, "universal_gnss/summary");
    const gps = findDiagnosticStatusByName(diagnostics, "GPS");
    const correctionStream = findDiagnosticStatusByNames(diagnostics, CORRECTION_STREAM_DIAGNOSTIC_NAMES);
    const msmSummary = findDiagnosticStatusByNames(diagnostics, MSM_SUMMARY_DIAGNOSTIC_NAMES);
    if (!summary && !gps && !correctionStream && !msmSummary) {
        return undefined;
    }

    const summaryValues = diagnosticsValueMap(summary);
    const gpsValues = diagnosticsValueMap(gps);
    const correctionStreamStatus = correctionStreamStatusFromMessage(correctionStream?.message);
    const msmSummaryValues = diagnosticsValueMap(msmSummary);
    const msmSummarySeen = parseDiagnosticBool(msmSummaryValues.seen);
    const msmSummaryDecoded = parseDiagnosticBool(msmSummaryValues.decoded);
    const msmSummaryValid = parseDiagnosticBool(msmSummaryValues.valid);
    const msmSummaryMessageType = parseDiagnosticInt(msmSummaryValues.message_type);
    const msmSummaryStationId = parseDiagnosticInt(msmSummaryValues.station_id);
    const msmSummarySatelliteCount = parseDiagnosticInt(msmSummaryValues.satellite_count);
    const msmSummarySignalCount = parseDiagnosticInt(msmSummaryValues.signal_count);
    const msmSummaryCellCount = parseDiagnosticInt(msmSummaryValues.cell_count);
    const msmSummaryAgeS = parseDiagnosticFloat(msmSummaryValues.age_s);
    const msmSummaryConstellationsSeen = msmSummaryValues.constellations_seen;
    const fixType = navSatFixStatusToGnssFixType(parseDiagnosticInt(gpsValues.fix_status));
    const fixValid = parseDiagnosticBool(summaryValues.fix_valid) ??
        (fixType !== undefined ? !isGnssFixType(fixType, GnssStatusConstants.FIX_TYPE_NO_FIX) : undefined);
    const capabilityFlags =
        (correctionStream ? GnssStatusConstants.CAP_CORRECTION_STREAM : 0) |
        (msmSummary ? GnssStatusConstants.CAP_MSM_SUMMARY : 0);
    const valueFlags =
        (correctionStreamStatus !== undefined ? GnssStatusConstants.CAP_CORRECTION_STREAM : 0) |
        ((msmSummarySeen !== undefined ||
            msmSummaryDecoded !== undefined ||
            msmSummaryValid !== undefined ||
            msmSummaryMessageType !== undefined ||
            msmSummaryStationId !== undefined ||
            msmSummaryConstellationsSeen !== undefined ||
            msmSummarySatelliteCount !== undefined ||
            msmSummarySignalCount !== undefined ||
            msmSummaryCellCount !== undefined ||
            msmSummaryAgeS !== undefined)
            ? GnssStatusConstants.CAP_MSM_SUMMARY
            : 0);

    const projected: GnssStatus = {
        backend: summary ? "universal" : undefined,
        fix_type: fixType ?? GnssStatusConstants.FIX_TYPE_NO_FIX,
        fix_valid: fixValid ?? false,
    };
    if (capabilityFlags) {
        projected.capability_flags = capabilityFlags;
    }
    if (valueFlags) {
        projected.value_flags = valueFlags;
    }
    if (correctionStream) {
        projected.correction_stream_status =
            correctionStreamStatus ?? GnssStatusConstants.CORRECTION_STREAM_STATUS_UNKNOWN;
    }
    if (msmSummary) {
        projected.msm_summary_seen = msmSummarySeen ?? false;
        projected.msm_summary_decoded = msmSummaryDecoded ?? false;
        projected.msm_summary_valid = msmSummaryValid ?? false;
        projected.msm_summary_message_type = msmSummaryMessageType ?? 0;
        projected.msm_summary_station_id = msmSummaryStationId ?? 0;
        projected.msm_summary_constellations_seen = msmSummaryConstellationsSeen;
        projected.msm_summary_satellite_count = msmSummarySatelliteCount ?? 0;
        projected.msm_summary_signal_count = msmSummarySignalCount ?? 0;
        projected.msm_summary_cell_count = msmSummaryCellCount ?? 0;
        projected.msm_summary_age_s = msmSummaryAgeS ?? 0;
    }
    return projected;
}

const CAPABILITY_GROUP_FIELDS: Array<{flag: number; fields: (keyof GnssStatus)[]}> = [
    {
        flag: GnssStatusConstants.CAP_CORRECTION_STREAM,
        fields: ["correction_stream_status"],
    },
    {
        flag: GnssStatusConstants.CAP_MSM_SUMMARY,
        fields: [
            "msm_summary_seen",
            "msm_summary_decoded",
            "msm_summary_valid",
            "msm_summary_message_type",
            "msm_summary_station_id",
            "msm_summary_constellations_seen",
            "msm_summary_satellite_count",
            "msm_summary_signal_count",
            "msm_summary_cell_count",
            "msm_summary_age_s",
        ],
    },
];

export function mergeGnssStatusDiagnosticProjection(
    gnssStatus: GnssStatus | undefined | null,
    diagnosticProjection: GnssStatus | undefined,
): GnssStatus {
    const base = gnssStatus ?? {};
    if (!diagnosticProjection) {
        return base;
    }
    if (!hasTypedGnssStatusSample(base)) {
        return diagnosticProjection;
    }

    const merged: GnssStatus = {
        ...diagnosticProjection,
        ...base,
        capability_flags: (base.capability_flags ?? 0) | (diagnosticProjection.capability_flags ?? 0),
        value_flags: (base.value_flags ?? 0) | (diagnosticProjection.value_flags ?? 0),
    };

    for (const group of CAPABILITY_GROUP_FIELDS) {
        if (((base.capability_flags ?? 0) & group.flag) !== 0 ||
            ((diagnosticProjection.capability_flags ?? 0) & group.flag) === 0) {
            continue;
        }
        for (const field of group.fields) {
            (merged as Record<string, unknown>)[field] = diagnosticProjection[field];
        }
    }

    return merged;
}

export function hasGnssCapability(gnssStatus: GnssStatus | undefined | null, flag: number): boolean {
    return ((gnssStatus?.capability_flags ?? 0) & flag) !== 0;
}

export function hasGnssValue(gnssStatus: GnssStatus | undefined | null, flag: number): boolean {
    return ((gnssStatus?.value_flags ?? 0) & flag) !== 0;
}

export function readGnssNumber(
    gnssStatus: GnssStatus | undefined | null,
    flag: number,
    value: number | undefined | null,
): number | undefined {
    return hasGnssValue(gnssStatus, flag) && value != null ? value : undefined;
}

export function displayHorizontalAccuracyM(
    gnssStatus: GnssStatus | undefined | null,
): number | undefined {
    return readGnssNumber(
        gnssStatus,
        GnssStatusConstants.CAP_HORIZONTAL_ACCURACY,
        gnssStatus?.horizontal_accuracy_m,
    );
}

export function readGnssBooleanState(
    gnssStatus: GnssStatus | undefined | null,
    flag: number,
    value: boolean | undefined | null,
): OptionalGnssBooleanState {
    if (!hasGnssCapability(gnssStatus, flag)) {
        return "unsupported";
    }
    if (!hasGnssValue(gnssStatus, flag) || value == null) {
        return "unknown";
    }
    return value ? "true" : "false";
}

export function gnssReceiverLabel(gnssStatus: GnssStatus | undefined | null): string {
    const vendor = gnssStatus?.receiver_vendor?.trim() ?? "";
    const model = gnssStatus?.receiver_model?.trim() ?? "";

    if (vendor && model) {
        return `${vendor} ${model}`;
    }
    if (model) {
        return model;
    }
    if (vendor) {
        return vendor;
    }

    return "GNSS";
}

// ── GNSS correction summary ─────────────────────────────────────────────
//
// GnssStatus carries two generations of correction telemetry:
//   * the source-owned fields (correction_source, correction_flow_status,
//     correction_transport_status, corrections_active, correction_age_s),
//     gated by CAP_CORRECTION_FLOW / CAP_CORRECTION_TRANSPORT /
//     CAP_CORRECTIONS_ACTIVE / CAP_CORRECTION_AGE;
//   * the legacy diagnostics-derived correction_stream_status, gated by
//     CAP_CORRECTION_STREAM.
// deriveCorrectionSummary folds both into one source-agnostic view (NTRIP,
// a radio/LoRa base, or no corrections at all) that the cards and the
// readiness check render. The source-owned fields win; the legacy stream
// status is consulted only when CAP_CORRECTION_STREAM is set.

/** Where RTCM corrections come from. "other" carries the raw string in `sourceRaw`. */
export type CorrectionSource = "ntrip" | "lora" | "none" | "other" | "unknown";

/** Coarse correction health, independent of the source. */
export type CorrectionState =
    | "active"
    | "waiting"
    | "idle"
    | "inactive"
    | "stale"
    | "invalid"
    | "unavailable"
    | "error"
    | "disabled"
    | "unknown";

export type CorrectionTone = "success" | "warning" | "error" | "neutral";

export interface CorrectionSummary {
    /** "typed": source-owned fields; "stream": legacy correction_stream_status; "none": nothing reported. */
    origin: "typed" | "stream" | "none";
    source: CorrectionSource;
    /** Trimmed correction_source as published (may be empty). */
    sourceRaw: string;
    state: CorrectionState;
    tone: CorrectionTone;
    /** correction_flow_status when the sample carries a value for it. */
    flowStatus?: number;
    /** Legacy correction_stream_status when CAP_CORRECTION_STREAM is set and valued. */
    streamStatus?: number;
    /** correction_transport_status when the sample carries a value for it. */
    transportStatus?: number;
    /**
     * The backend can report a transport but has no current value — e.g. a
     * LoRa radio link that the host cannot observe. Render "not observable",
     * not "unknown error".
     */
    transportUnobservable: boolean;
    /** corrections_active when known. */
    active?: boolean;
    /** Correction age in seconds when known. */
    ageS?: number;
}

const TYPED_CORRECTION_CAPABILITIES =
    GnssStatusConstants.CAP_CORRECTION_FLOW |
    GnssStatusConstants.CAP_CORRECTION_TRANSPORT |
    GnssStatusConstants.CAP_CORRECTIONS_ACTIVE;

function normalizeCorrectionSource(raw: string): CorrectionSource {
    switch (raw.toLowerCase()) {
        case "":
            return "unknown";
        case "ntrip":
            return "ntrip";
        case "lora":
            return "lora";
        case "none":
            return "none";
        default:
            return "other";
    }
}

function toneForCorrectionState(state: CorrectionState): CorrectionTone {
    switch (state) {
        case "active":
            return "success";
        case "waiting":
        case "idle":
        case "inactive":
            return "warning";
        case "stale":
        case "invalid":
        case "unavailable":
        case "error":
            return "error";
        default:
            return "neutral";
    }
}

// Wire values are plain numbers; lookup tables avoid comparing them against the const enum.
const CORRECTION_STATE_BY_FLOW: Record<number, CorrectionState> = {
    [GnssStatusConstants.CORRECTION_FLOW_STATUS_ACTIVE]: "active",
    [GnssStatusConstants.CORRECTION_FLOW_STATUS_WAITING]: "waiting",
    [GnssStatusConstants.CORRECTION_FLOW_STATUS_IDLE]: "idle",
    [GnssStatusConstants.CORRECTION_FLOW_STATUS_STALE]: "stale",
    [GnssStatusConstants.CORRECTION_FLOW_STATUS_INVALID]: "invalid",
};

const TRANSPORT_LABEL_KEYS: Record<number, string> = {
    [GnssStatusConstants.CORRECTION_TRANSPORT_STATUS_DISCONNECTED]: "corrections.transport.disconnected",
    [GnssStatusConstants.CORRECTION_TRANSPORT_STATUS_CONNECTING]: "corrections.transport.connecting",
    [GnssStatusConstants.CORRECTION_TRANSPORT_STATUS_CONNECTED]: "corrections.transport.connected",
    [GnssStatusConstants.CORRECTION_TRANSPORT_STATUS_STREAMING]: "corrections.transport.streaming",
    [GnssStatusConstants.CORRECTION_TRANSPORT_STATUS_RECONNECTING]: "corrections.transport.reconnecting",
    [GnssStatusConstants.CORRECTION_TRANSPORT_STATUS_FAILED]: "corrections.transport.failed",
};

function stateFromFlow(flow: number): CorrectionState {
    return CORRECTION_STATE_BY_FLOW[flow] ?? "unknown";
}

function stateFromStream(stream: number | undefined): CorrectionState {
    switch (stream) {
        case GnssStatusConstants.CORRECTION_STREAM_STATUS_ACTIVE:
            return "active";
        case GnssStatusConstants.CORRECTION_STREAM_STATUS_WAITING:
            return "waiting";
        case GnssStatusConstants.CORRECTION_STREAM_STATUS_IDLE:
            return "idle";
        case GnssStatusConstants.CORRECTION_STREAM_STATUS_UNAVAILABLE:
            return "unavailable";
        case GnssStatusConstants.CORRECTION_STREAM_STATUS_ERROR:
            return "error";
        default:
            return "unknown";
    }
}

/** Pure: no i18n. See CorrectionSummary for the field semantics. */
export function deriveCorrectionSummary(gnssStatus: GnssStatus | undefined | null): CorrectionSummary {
    const sourceRaw = gnssStatus?.correction_source?.trim() ?? "";
    const source = normalizeCorrectionSource(sourceRaw);
    const typed = ((gnssStatus?.capability_flags ?? 0) & TYPED_CORRECTION_CAPABILITIES) !== 0;
    const hasStream = hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_CORRECTION_STREAM);
    const streamStatus = hasStream && hasGnssValue(gnssStatus, GnssStatusConstants.CAP_CORRECTION_STREAM)
        ? gnssStatus?.correction_stream_status
        : undefined;
    const streamState = stateFromStream(streamStatus);

    const base: CorrectionSummary = {
        origin: "none",
        source,
        sourceRaw,
        state: "unknown",
        tone: "neutral",
        transportUnobservable: false,
    };

    if (!typed) {
        if (!hasStream) {
            return base;
        }
        return {
            ...base,
            origin: "stream",
            state: streamState,
            tone: toneForCorrectionState(streamState),
            streamStatus,
        };
    }

    const flowStatus = hasGnssValue(gnssStatus, GnssStatusConstants.CAP_CORRECTION_FLOW)
        ? gnssStatus?.correction_flow_status
        : undefined;
    const transportStatus = hasGnssValue(gnssStatus, GnssStatusConstants.CAP_CORRECTION_TRANSPORT)
        ? gnssStatus?.correction_transport_status
        : undefined;
    const active = hasGnssValue(gnssStatus, GnssStatusConstants.CAP_CORRECTIONS_ACTIVE)
        ? gnssStatus?.corrections_active
        : undefined;
    const rawAge = readGnssNumber(gnssStatus, GnssStatusConstants.CAP_CORRECTION_AGE, gnssStatus?.correction_age_s);
    const ageS = rawAge !== undefined && Number.isFinite(rawAge) && rawAge >= 0 ? rawAge : undefined;
    const transportUnobservable =
        hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_CORRECTION_TRANSPORT) && transportStatus === undefined;

    let state: CorrectionState;
    if (source === "none") {
        state = "disabled";
    } else if (flowStatus !== undefined) {
        state = stateFromFlow(flowStatus);
    } else if (active === true) {
        state = "active";
    } else if (active === false) {
        state = "inactive";
    } else {
        state = "unknown";
    }
    // A failed transport explains a missing flow better than "waiting".
    if (state !== "active" && state !== "disabled" &&
        transportStatus === GnssStatusConstants.CORRECTION_TRANSPORT_STATUS_FAILED) {
        state = "error";
    }
    // Typed fields present but silent: an older adapter may still mirror the legacy stream.
    if (state === "unknown" && streamState !== "unknown") {
        state = streamState;
    }

    return {
        ...base,
        origin: "typed",
        state,
        streamStatus,
        tone: toneForCorrectionState(state),
        flowStatus,
        transportStatus,
        transportUnobservable,
        active,
        ageS,
    };
}

/**
 * Whether the LoRa base-station card (status + pairing) applies.
 *
 * TODO(robot-profile): gate on `useRobotProfile().features.lora_corrections`
 * once the profile hook lands; until then the live source is the signal.
 */
export function isLoraCorrectionsSupported(gnssStatus: GnssStatus | undefined | null): boolean {
    return deriveCorrectionSummary(gnssStatus).source === "lora";
}

export function correctionSourceLabel(summary: CorrectionSummary): string | undefined {
    switch (summary.source) {
        case "ntrip":
            return i18n.t("corrections.source.ntrip");
        case "lora":
            return i18n.t("corrections.source.lora");
        case "none":
            return i18n.t("corrections.source.none");
        case "other":
            return summary.sourceRaw;
        default:
            return undefined;
    }
}

export function correctionAgeLabel(summary: CorrectionSummary): string {
    return summary.ageS !== undefined
        ? i18n.t("corrections.ageValue", {age: summary.ageS.toFixed(1)})
        : i18n.t("corrections.ageUnknown");
}

export function correctionStateLabel(summary: CorrectionSummary): string {
    if (summary.state === "stale" && summary.ageS !== undefined) {
        return i18n.t("corrections.state.staleWithAge", {age: summary.ageS.toFixed(0)});
    }
    return i18n.t(`corrections.state.${summary.state}`);
}

export function correctionTransportLabel(summary: CorrectionSummary): string | undefined {
    if (summary.transportStatus === undefined) {
        if (!summary.transportUnobservable) {
            return undefined;
        }
        return summary.source === "lora"
            ? i18n.t("corrections.transport.radioNotObservable")
            : i18n.t("corrections.transport.unknown");
    }
    return i18n.t(TRANSPORT_LABEL_KEYS[summary.transportStatus] ?? "corrections.transport.unknown");
}

/**
 * One-line headline, e.g. "LoRa base · waiting for data". Legacy-only samples
 * keep the established correction-stream wording.
 */
export function correctionSummaryLabel(summary: CorrectionSummary): string {
    if (summary.origin === "none") {
        return i18n.t("corrections.state.unknown");
    }
    if (summary.origin === "stream") {
        return correctionStreamStatusLabel(summary.streamStatus) ?? i18n.t("gpsStatus.correctionStreamUnknown");
    }
    if (summary.source === "none") {
        return i18n.t("corrections.noneConfigured");
    }
    const sourceLabel = correctionSourceLabel(summary);
    const stateLabel = correctionStateLabel(summary);
    return sourceLabel ? `${sourceLabel} · ${stateLabel}` : stateLabel;
}
