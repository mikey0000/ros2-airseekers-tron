import {describe, expect, it} from "vitest";
import {
    RTK_BASE_UNIVERSAL_CODE,
    formatRtkBaseAddr,
    parseRtkBaseAddr,
    rtkBaseAreaForBand,
    rtkBaseBandLabelKey,
    rtkBaseNtripOwns,
    selectRtkBaseStatus,
    type RtkBaseStatus,
} from "./rtkBase.ts";

const status = (over: Partial<RtkBaseStatus> = {}): RtkBaseStatus => ({
    pairing: null, ref_station: null, nrtk_on: false, station_id: null,
    diff_age_s: null, fix_type: null, correction_source: "lora", ...over,
});

describe("rtkBase", () => {
    it("parses decimal and 0x hex addresses within 0..0xFFFE", () => {
        expect(parseRtkBaseAddr("38183")).toBe(38183);
        expect(parseRtkBaseAddr(" 0x9527 ")).toBe(38183);
        expect(parseRtkBaseAddr("0X9527")).toBe(38183);
        expect(parseRtkBaseAddr("0")).toBe(0);
        expect(parseRtkBaseAddr("65534")).toBe(65534);
        expect(parseRtkBaseAddr(38183)).toBe(38183);
        for (const bad of ["65535", "0xFFFF", "0x10000", "-1", "9527h", "", "1.5", "0x"]) {
            expect(parseRtkBaseAddr(bad)).toBeUndefined();
        }
        expect(parseRtkBaseAddr(undefined)).toBeUndefined();
        expect(parseRtkBaseAddr(1.5)).toBeUndefined();
    });

    it("formats the address in decimal and hex", () => {
        expect(formatRtkBaseAddr(38183)).toBe("38183 (0x9527)");
        expect(formatRtkBaseAddr(1)).toBe("1 (0x0001)");
    });

    it("keeps the vendor universal code", () => {
        expect(RTK_BASE_UNIVERSAL_CODE).toEqual({addr: 38183, channel: 40, area: "02"});
    });

    it("maps the board's band to its label and region", () => {
        expect(rtkBaseBandLabelKey("0")).toBe("rtkBase.band.europe");
        expect(rtkBaseBandLabelKey("2")).toBe("rtkBase.band.australia");
        expect(rtkBaseBandLabelKey("f")).toBe("rtkBase.band.all");
        expect(rtkBaseBandLabelKey(null)).toBeUndefined();
        expect(rtkBaseAreaForBand("0")).toBe("02");
        expect(rtkBaseAreaForBand("1")).toBe("01");
        expect(rtkBaseAreaForBand("2")).toBe("03");
        expect(rtkBaseAreaForBand("F")).toBeUndefined();
    });

    it("decodes the std_msgs/String status payload", () => {
        const inner = status({ref_station: "FINE"});
        expect(selectRtkBaseStatus({data: JSON.stringify(inner)})).toEqual(inner);
        expect(selectRtkBaseStatus({data: "not json"})).toBeUndefined();
        expect(selectRtkBaseStatus({data: "{}"})).toBeUndefined();
        expect(selectRtkBaseStatus(null)).toBeUndefined();
    });

    it("says NTRIP owns the receiver when enabled, active or the board is in network RTK", () => {
        expect(rtkBaseNtripOwns(status(), false)).toBe(false);
        expect(rtkBaseNtripOwns(null, false)).toBe(false);
        expect(rtkBaseNtripOwns(status(), true)).toBe(true);
        expect(rtkBaseNtripOwns(status({nrtk_on: true}), false)).toBe(true);
        expect(rtkBaseNtripOwns(status({correction_source: "ntrip"}), false)).toBe(true);
    });
});
