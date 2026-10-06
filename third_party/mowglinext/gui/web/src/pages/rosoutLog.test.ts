import {describe, expect, it} from "vitest";
import {
    acceptRosoutRecord,
    decodeUtf8Binary,
    distinctNodes,
    formatLinesForCopy,
    parseRosoutFrame,
    rosLevelToSeverity,
    type RosoutRecord,
} from "./rosoutLog.ts";

const record = (seq: number, boot = 1): RosoutRecord => ({
    boot, seq, stamp_ms: 1747087353123, level: 'INFO', node: 'map_server_node', msg: `line ${seq}`,
});

describe("rosoutLog", () => {
    it("rebuilds UTF-8 from atob() output", () => {
        const binary = String.fromCharCode(...new TextEncoder().encode("dock → 0.5 m °"));
        expect(decodeUtf8Binary(binary)).toBe("dock → 0.5 m °");
        expect(decodeUtf8Binary("plain ascii")).toBe("plain ascii");
    });

    it("parses a record and rejects anything else", () => {
        expect(parseRosoutFrame(JSON.stringify(record(3)))).toEqual(record(3));
        expect(parseRosoutFrame("not json")).toBeNull();
        expect(parseRosoutFrame(JSON.stringify({seq: 1}))).toBeNull();
    });

    it("skips history replayed on reconnect but accepts a restarted backend", () => {
        let cursor = acceptRosoutRecord(null, record(5)).cursor;
        expect(acceptRosoutRecord(cursor, record(4)).accept).toBe(false);
        expect(acceptRosoutRecord(cursor, record(5)).accept).toBe(false);
        const next = acceptRosoutRecord(cursor, record(6));
        expect(next.accept).toBe(true);
        cursor = next.cursor;
        // New boot: seq restarted from 1.
        expect(acceptRosoutRecord(cursor, record(1, 2))).toEqual({accept: true, cursor: {boot: 2, seq: 1}});
    });

    it("maps ROS levels onto the page severities", () => {
        expect(rosLevelToSeverity('FATAL')).toBe('ERROR');
        expect(rosLevelToSeverity('ERROR')).toBe('ERROR');
        expect(rosLevelToSeverity('WARN')).toBe('WARN');
        expect(rosLevelToSeverity('INFO')).toBe('INFO');
        expect(rosLevelToSeverity('DEBUG')).toBe('DEBUG');
        expect(rosLevelToSeverity('UNSET')).toBe('OTHER');
    });

    it("lists distinct nodes sorted", () => {
        expect(distinctNodes([{node: 'b'}, {node: 'a'}, {}, {node: 'b'}])).toEqual(['a', 'b']);
    });

    it("formats lines for the clipboard", () => {
        const text = formatLinesForCopy([
            {tsMs: 0, severity: 'WARN', node: 'n', plain: 'careful'},
            {tsMs: 1, severity: 'OTHER', plain: 'raw'},
        ], (ms) => `t${ms}`);
        expect(text).toBe("t0 WARN [n] careful\nt1 raw");
    });
});
