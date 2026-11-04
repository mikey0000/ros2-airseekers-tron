import {useTopic} from "./useTopic.ts";
import {selectDockMoved, type DockMovedResult} from "../utils/dockMoved.ts";

/** Latest latched /mower_docking/dock_moved result; null until one arrives. */
export function useDockMoved(enabled = true): DockMovedResult | null {
    const {data} = useTopic<DockMovedResult | null>("dockMoved", null, {
        select: (raw) => selectDockMoved(raw),
        enabled,
    });
    return enabled ? data : null;
}
