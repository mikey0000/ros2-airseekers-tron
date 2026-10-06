import type {GnssStatus} from "../types/ros.ts";
import {GnssStatusConstants} from "../types/ros.ts";

// Correction-telemetry fixtures for deriveCorrectionSummary and the cards.

/**
 * Live /gps/status sample decoded from a UM960 robot whose LoRa base is not
 * paired (ws …/api/mowglinext/subscribe/gnssStatus). The bridge advertises the
 * source-owned correction fields; flow has a value (WAITING), transport and
 * age are supported but currently unknown (the radio link is invisible to the
 * host), and the legacy correction_stream_status is not supported.
 * capability bits {0,1,3,4,7,8,10,11,12,25,26}, value bits {0,1,3,4,7,8,10,11,26}.
 */
export const loraWaitingLiveSample: GnssStatus = {
    backend: "um960_gps_driver",
    receiver_vendor: "Unicore",
    receiver_model: "UM960",
    fix_type: GnssStatusConstants.FIX_TYPE_GPS_FIX,
    fix_valid: true,
    rtk_mode: GnssStatusConstants.RTK_MODE_NONE,
    satellites_used: 37,
    satellites_visible: 37,
    hdop: 0.42,
    horizontal_accuracy_m: 0.45,
    differential_corrections: false,
    corrections_active: false,
    correction_source: "lora",
    correction_flow_status: GnssStatusConstants.CORRECTION_FLOW_STATUS_WAITING,
    correction_transport_status: GnssStatusConstants.CORRECTION_TRANSPORT_STATUS_UNKNOWN,
    correction_age_s: 0,
    correction_stream_status: GnssStatusConstants.CORRECTION_STREAM_STATUS_UNKNOWN,
    capability_flags: 100670875,
    value_flags: 67112347,
};

const TYPED_CORRECTION_CAPS =
    GnssStatusConstants.CAP_CORRECTIONS_ACTIVE |
    GnssStatusConstants.CAP_CORRECTION_AGE |
    GnssStatusConstants.CAP_CORRECTION_TRANSPORT |
    GnssStatusConstants.CAP_CORRECTION_FLOW;

/** NTRIP caster streaming valid RTCM, receiver in RTK fixed. */
export const ntripStreamingSample: GnssStatus = {
    backend: "um960_gps_driver",
    fix_type: GnssStatusConstants.FIX_TYPE_RTK_FIXED,
    fix_valid: true,
    rtk_mode: GnssStatusConstants.RTK_MODE_FIXED,
    corrections_active: true,
    correction_source: "ntrip",
    correction_flow_status: GnssStatusConstants.CORRECTION_FLOW_STATUS_ACTIVE,
    correction_transport_status: GnssStatusConstants.CORRECTION_TRANSPORT_STATUS_STREAMING,
    correction_response_accepted: true,
    correction_age_s: 0.8,
    capability_flags: GnssStatusConstants.CAP_RTK_MODE | TYPED_CORRECTION_CAPS,
    value_flags: GnssStatusConstants.CAP_RTK_MODE | TYPED_CORRECTION_CAPS,
};

/** Corrections deliberately off (correction_source "none"). */
export const noCorrectionsSample: GnssStatus = {
    backend: "um960_gps_driver",
    fix_type: GnssStatusConstants.FIX_TYPE_GPS_FIX,
    fix_valid: true,
    rtk_mode: GnssStatusConstants.RTK_MODE_NONE,
    corrections_active: false,
    correction_source: "none",
    correction_flow_status: GnssStatusConstants.CORRECTION_FLOW_STATUS_UNKNOWN,
    correction_transport_status: GnssStatusConstants.CORRECTION_TRANSPORT_STATUS_DISCONNECTED,
    capability_flags: GnssStatusConstants.CAP_RTK_MODE | TYPED_CORRECTION_CAPS,
    value_flags: GnssStatusConstants.CAP_RTK_MODE |
        GnssStatusConstants.CAP_CORRECTIONS_ACTIVE |
        GnssStatusConstants.CAP_CORRECTION_TRANSPORT,
};
