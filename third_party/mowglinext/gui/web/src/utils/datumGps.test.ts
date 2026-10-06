import { describe, expect, it, vi } from "vitest";
import { DatumParseError, datumErrorDetail, requestDatumFromGps } from "./datumGps.ts";

const apiWith = (res: unknown) =>
    ({ mowglinext: { callCreate: vi.fn().mockResolvedValue(res) } }) as any;

describe("requestDatumFromGps", () => {
    it("parses a lat,lon reply", async () => {
        const r = await requestDatumFromGps(apiWith({ data: { message: "48.1234567,-1.5" } }));
        expect(r).toEqual({ lat: 48.1234567, lon: -1.5 });
    });
    it("throws DatumParseError on an unparseable reply", async () => {
        await expect(requestDatumFromGps(apiWith({ data: { message: "datum set" } })))
            .rejects.toBeInstanceOf(DatumParseError);
        await expect(requestDatumFromGps(apiWith({ data: {} }))).rejects.toBeInstanceOf(DatumParseError);
    });
    it("throws with the service message when the call fails", async () => {
        await expect(requestDatumFromGps(apiWith({ error: { error: "no RTK fix" } })))
            .rejects.toThrow("no RTK fix");
    });
});

describe("datumErrorDetail", () => {
    it("includes the raw reply for parse errors and the message otherwise", () => {
        expect(datumErrorDetail(new DatumParseError("oops"), (m) => `bad: ${m}`)).toBe("bad: oops");
        expect(datumErrorDetail(new Error("no fix"), () => "x")).toBe("no fix");
        expect(datumErrorDetail("str", () => "x")).toBe("str");
    });
});
