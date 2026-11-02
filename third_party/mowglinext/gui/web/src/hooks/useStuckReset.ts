import {useCallback, useEffect, useSyncExternalStore} from "react";

// STUCK_NEEDS_HELP: the owner moves the robot clear by hand and presses Reset
// before Resume is offered. The banner and the map toolbars are separate
// components; this tab-wide flag keeps them in step (the toolbar used to offer
// "Resume mowing" while the banner still asked for Reset).
let resetDone = false;
const listeners = new Set<() => void>();

const setResetDone = (v: boolean) => {
    if (resetDone === v) return;
    resetDone = v;
    listeners.forEach((l) => l());
};
const subscribe = (l: () => void) => {
    listeners.add(l);
    return () => { listeners.delete(l); };
};
const snapshot = () => resetDone;

/** [Reset pressed in the current STUCK_NEEDS_HELP latch, mark it pressed]. */
export function useStuckReset(stateName?: string | null): [boolean, () => void] {
    const done = useSyncExternalStore(subscribe, snapshot, snapshot);
    // Leaving the latch (or never being in it) forgets the reset.
    useEffect(() => {
        if (stateName !== "STUCK_NEEDS_HELP") setResetDone(false);
    }, [stateName]);
    const markDone = useCallback(() => setResetDone(true), []);
    return [stateName === "STUCK_NEEDS_HELP" && done, markDone];
}
