import {useEffect, useState} from "react";
import type {DetectionSummary} from "../../hooks/useDetections.ts";

export interface FrameDetections {
    summary: DetectionSummary;
    /** Date.now() when the message arrived. */
    at: number;
}

export interface DetectionHistory {
    /** Latest message per source frame (camera). */
    byFrame: Record<string, FrameDetections>;
    /** Running count of detected objects per class since the page opened. */
    histogram: Record<string, number>;
    /** Messages seen since the page opened. */
    messages: number;
}

export const EMPTY_HISTORY: DetectionHistory = {byFrame: {}, histogram: {}, messages: 0};

/** Fold one detection message into the history (pure). */
export function addDetections(h: DetectionHistory, summary: DetectionSummary, at: number): DetectionHistory {
    const frame = (summary.frame_id ?? "").replace(/^\//, "") || "?";
    const histogram = {...h.histogram};
    const boxes = summary.boxes ?? [];
    if (boxes.length > 0) {
        for (const b of boxes) histogram[b.class] = (histogram[b.class] ?? 0) + 1;
    } else {
        // Older backend without boxes: count classes once per message.
        for (const c of summary.classes) histogram[c] = (histogram[c] ?? 0) + 1;
    }
    return {byFrame: {...h.byFrame, [frame]: {summary, at}}, histogram, messages: h.messages + 1};
}

/** Histogram entries, most frequent first. */
export function sortedHistogram(h: Record<string, number>): [string, number][] {
    return Object.entries(h).sort((a, b) => b[1] - a[1] || a[0].localeCompare(b[0]));
}

/** Accumulates every delivered detection message (keyed by `lastMessageAt`). */
export function useDetectionHistory(summary: DetectionSummary, lastMessageAt: number | null): DetectionHistory {
    const [history, setHistory] = useState<DetectionHistory>(EMPTY_HISTORY);
    useEffect(() => {
        if (lastMessageAt === null) return;
        setHistory((h) => addDetections(h, summary, lastMessageAt));
        // eslint-disable-next-line react-hooks/exhaustive-deps -- one fold per delivered message
    }, [lastMessageAt]);
    return history;
}
