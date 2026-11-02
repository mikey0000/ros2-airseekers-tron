// Plain-language presentation of the mission states that need the owner:
// STUCK_NEEDS_HELP, MOWING_INCOMPLETE, NAV_TO_DOCK_FAILED and the "heading not
// aligned" hint. The mission's sub_state strings are engineer text
// ("2 of 7 sub-paths not mowed: follow aborted: path blocked",
// "stuck: needs help at (-5.20, 0.40)"); these helpers turn them into i18n
// keys + params under `missionText`. Pure, so the banner, the map status line
// and the Home hero all say the same thing.

export interface Phrase {
    key: string;
    params?: Record<string, string | number>;
}

/** mower_mission fsm/common.py HEADING_NOT_ALIGNED (the sub_state prefix). */
const HEADING_PREFIX = "heading not aligned";

export const isHeadingHint = (subState?: string | null): boolean =>
    !!subState && subState.trim().toLowerCase().startsWith(HEADING_PREFIX);

/** Why STUCK_NEEDS_HELP latched, from the escape that gave up (stuck / tilt / boundary guards). */
export function stuckReason(subState?: string | null): Phrase {
    const s = (subState ?? "").toLowerCase();
    if (s.startsWith("steep slope")) return {key: "missionText.stuckSlope"};
    if (s.startsWith("boundary")) return {key: "missionText.stuckBoundary"};
    return {key: "missionText.stuckWheels"};
}

export interface IncompleteSummary {
    /** Sub-paths (strips) that were not mowed; null when the text has no count. */
    skippedStrips: number | null;
    plannedStrips: number | null;
    /** Areas the planner could not plan at all. */
    unplannedAreas: number;
    /** Stretches the end-of-area pass found blocked by obstacles. */
    blockedStretches: number;
    /** The last failure reason, cleaned ("path blocked"), or null. */
    reason: string | null;
}

/** Parse mower_mission outcome.py _not_mowed_text. */
export function parseIncomplete(subState?: string | null): IncompleteSummary {
    const s = subState ?? "";
    const strips = /(\d+) of (\d+) sub-paths not mowed/.exec(s);
    const areas = /(\d+) area\(s\) not planned/.exec(s);
    const blocked = /(\d+) stretch(?:es)? blocked by obstacles/.exec(s);
    // Everything after the first ": " is the last reason; keep its last clause
    // ("follow aborted: path blocked" -> "path blocked").
    const colon = s.indexOf(": ");
    const rawReason = colon >= 0 ? s.slice(colon + 2) : "";
    const reason = rawReason.split(": ").pop()?.trim() || null;
    return {
        skippedStrips: strips ? Number(strips[1]) : null,
        plannedStrips: strips ? Number(strips[2]) : null,
        unplannedAreas: areas ? Number(areas[1]) : 0,
        blockedStretches: blocked ? Number(blocked[1]) : 0,
        reason,
    };
}

/** Phrases for the MOWING_INCOMPLETE body, in display order. */
export function incompletePhrases(subState: string | null | undefined, skippedM?: number | null): Phrase[] {
    const p = parseIncomplete(subState);
    const out: Phrase[] = [];
    if (p.skippedStrips) out.push({key: "missionText.incompleteStrips", params: {count: p.skippedStrips}});
    if (skippedM != null && skippedM >= 0.5) out.push({key: "missionText.incompleteLength", params: {m: Math.round(skippedM)}});
    if (p.unplannedAreas) out.push({key: "missionText.incompleteAreas", params: {count: p.unplannedAreas}});
    if (p.blockedStretches) out.push({key: "missionText.incompleteBlocked", params: {count: p.blockedStretches}});
    if (p.reason) out.push({key: "missionText.incompleteReason", params: {reason: p.reason}});
    return out;
}

/** Why the drive to the dock failed (fsm/phases/docking.py _dock_failed reasons). */
export function dockFailReason(subState?: string | null): Phrase {
    const s = (subState ?? "").toLowerCase();
    if (/marker/.test(s)) return {key: "missionText.dockReasonMarker"};
    if (/no docking server|no dock_pose|dock pose/.test(s)) return {key: "missionText.dockReasonNoDock"};
    if (/off the route graph/.test(s)) return {key: "missionText.dockReasonRoute"};
    if (/approach pose/.test(s)) return {key: "missionText.dockReasonApproach"};
    if (/nav2 down|nav2/.test(s)) return {key: "missionText.dockReasonNav"};
    return {key: "missionText.dockReasonGeneric"};
}

/**
 * Short plain sub-state for the one-line status (map status line, Home hero),
 * or null to show the mission's own text.
 */
export function plainSubState(stateName?: string | null, subState?: string | null): Phrase | null {
    if (isHeadingHint(subState)) return {key: "missionText.headingShort"};
    switch (stateName) {
        case "STUCK_NEEDS_HELP": return stuckReason(subState);
        case "NAV_TO_DOCK_FAILED": return dockFailReason(subState);
        case "MOWING_INCOMPLETE": {
            const p = parseIncomplete(subState);
            return p.skippedStrips ? {key: "missionText.incompleteStrips", params: {count: p.skippedStrips}} : null;
        }
        default: return null;
    }
}

/** States (or hints) for which the Home hero must not say "idle, tap Play". */
export function attentionTitleKey(stateName?: string | null, subState?: string | null): string | null {
    switch (stateName) {
        case "STUCK_NEEDS_HELP": return "missionText.stuckTitle";
        case "MOWING_INCOMPLETE": return "missionText.incompleteTitle";
        case "NAV_TO_DOCK_FAILED": return "missionText.dockFailedTitle";
        case "BOUNDARY_EMERGENCY_STOP": return "utils.stateBoundaryAlert";
        default: return isHeadingHint(subState) ? "missionText.headingTitle" : null;
    }
}

/** Length (m) of the skipped stretches of the current plan. */
export function skippedLengthM(
    plan: {plan_id: string; subpaths: [number, number][][]} | null | undefined,
    progress: {plan_id: string | null; skipped: [number, number][]} | null | undefined,
): number | null {
    if (!plan || !progress || progress.plan_id !== plan.plan_id || progress.skipped.length === 0) return null;
    // Pose indices are absolute into the concatenated sub-paths; an edge
    // between the last pose of one sub-path and the first of the next is a
    // transit, not a mowed stretch.
    const pts = plan.subpaths.flat();
    const subOf = plan.subpaths.flatMap((sp, i) => sp.map(() => i));
    let total = 0;
    for (const [a, b] of progress.skipped) {
        for (let k = Math.max(0, a); k < Math.min(b, pts.length - 1); k++) {
            if (subOf[k] !== subOf[k + 1]) continue;
            total += Math.hypot(pts[k + 1][0] - pts[k][0], pts[k + 1][1] - pts[k][1]);
        }
    }
    return total;
}
