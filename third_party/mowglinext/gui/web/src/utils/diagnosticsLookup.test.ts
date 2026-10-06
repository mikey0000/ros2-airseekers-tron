import {describe, expect, it} from "vitest";
import {diagnosticValue, localizationDiagnostics} from "./diagnosticsLookup.ts";

const s = (name: string, hardware_id = "", values: {key: string; value: string}[] = []) =>
    ({name, hardware_id, level: 0, message: "", values});

describe("localizationDiagnostics", () => {
    it("keeps the estimator, GNSS and IMU entries in order", () => {
        const all = [
            s("twist_mux: Twist mux status", "none"),
            s("ekf_node: Filter diagnostic updater", "none"),
            s("tron: GNSS", "gnss"),
            s("tron: Battery", "battery"),
            s("IMU", "imu"),
            s("tron: Localization", "localization"),
            s("navsat_transform_node: datum"),
            s("/left_oa_camera/image_raw topic status", "left"),
        ];
        expect(localizationDiagnostics(all).map((x) => x.name)).toEqual([
            "ekf_node: Filter diagnostic updater",
            "tron: GNSS",
            "IMU",
            "tron: Localization",
            "navsat_transform_node: datum",
        ]);
    });

    it("does not match words that merely contain a fragment", () => {
        expect(localizationDiagnostics([s("tron: Cutter motor", "cutter_motor"), s("simulator")])).toEqual([]);
        expect(localizationDiagnostics(undefined)).toEqual([]);
    });
});

describe("diagnosticValue", () => {
    it("reads one key of an exactly named entry", () => {
        const all = [s("IMU", "imu", [{key: "Bias calibration", value: "CALIBRATED"}])];
        expect(diagnosticValue(all, "IMU", "Bias calibration")).toBe("CALIBRATED");
        expect(diagnosticValue(all, "IMU", "missing")).toBeUndefined();
        expect(diagnosticValue(all, "imu", "Bias calibration")).toBeUndefined();
        expect(diagnosticValue(undefined, "IMU", "x")).toBeUndefined();
    });
});
