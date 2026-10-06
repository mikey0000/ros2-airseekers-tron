// /rosout log stream helpers for the Logs page (pure, unit-tested).
//
// The backend (gui/pkg/api/rosout.go) sends one base64 text frame per ROS log
// line, each a JSON RosoutRecord. useWS already base64-decodes frames with
// atob(), which yields one char per byte, so the UTF-8 is rebuilt here.

export type Severity = 'ERROR' | 'WARN' | 'INFO' | 'DEBUG' | 'OTHER';

export type RosLogLevel = 'FATAL' | 'ERROR' | 'WARN' | 'INFO' | 'DEBUG' | 'UNSET';

/** Mirror of RosoutRecord in gui/pkg/api/rosout.go. */
export interface RosoutRecord {
    /** Backend hub instance (start time, epoch ms); seq restarts when it changes. */
    boot: number;
    seq: number;
    /** Producer timestamp, epoch ms. */
    stamp_ms: number;
    level: RosLogLevel;
    node: string;
    msg: string;
    file?: string;
    function?: string;
    line?: number;
}

/** Last record a client has seen; null before the first one. */
export type RosoutCursor = { boot: number; seq: number } | null;

/** Turn atob() output (one char per byte) back into the UTF-8 it encoded. */
export function decodeUtf8Binary(binary: string): string {
    // Pure ASCII needs no work and is the common case.
    if (!/[\u0080-\uffff]/.test(binary)) return binary;
    const bytes = Uint8Array.from(binary, (c) => c.charCodeAt(0));
    return new TextDecoder().decode(bytes);
}

function isRecord(value: unknown): value is RosoutRecord {
    const r = value as RosoutRecord | null;
    return !!r && typeof r === 'object'
        && typeof r.seq === 'number' && typeof r.boot === 'number'
        && typeof r.stamp_ms === 'number' && typeof r.level === 'string'
        && typeof r.node === 'string' && typeof r.msg === 'string';
}

/** Parse one decoded frame; null when it is not a RosoutRecord. */
export function parseRosoutFrame(frame: string): RosoutRecord | null {
    try {
        const parsed: unknown = JSON.parse(decodeUtf8Binary(frame));
        return isRecord(parsed) ? parsed : null;
    } catch {
        return null;
    }
}

/**
 * Whether to keep `record` given the last one seen. A reconnect replays the
 * backend's recent history, which must not be shown twice; a backend restart
 * (new boot) restarts seq, so everything from it is new.
 */
export function acceptRosoutRecord(
    cursor: RosoutCursor,
    record: RosoutRecord,
): { accept: boolean; cursor: RosoutCursor } {
    if (cursor && cursor.boot === record.boot && record.seq <= cursor.seq) {
        return {accept: false, cursor};
    }
    return {accept: true, cursor: {boot: record.boot, seq: record.seq}};
}

export function rosLevelToSeverity(level: RosLogLevel): Severity {
    switch (level) {
        case 'FATAL':
        case 'ERROR':
            return 'ERROR';
        case 'WARN':
            return 'WARN';
        case 'INFO':
            return 'INFO';
        case 'DEBUG':
            return 'DEBUG';
        default:
            return 'OTHER';
    }
}

/** Distinct node names, sorted, for the node filter. */
export function distinctNodes(lines: readonly { node?: string }[]): string[] {
    const set = new Set<string>();
    for (const l of lines) if (l.node) set.add(l.node);
    return [...set].sort((a, b) => a.localeCompare(b));
}

/** Plain-text rendering of lines for the clipboard, one per line. */
export function formatLinesForCopy(
    lines: readonly { tsMs: number; severity: Severity; node?: string; plain: string }[],
    formatTime: (ms: number) => string,
): string {
    return lines.map((l) => {
        const parts = [formatTime(l.tsMs)];
        if (l.severity !== 'OTHER') parts.push(l.severity);
        if (l.node) parts.push(`[${l.node}]`);
        parts.push(l.plain);
        return parts.join(' ');
    }).join('\n');
}
