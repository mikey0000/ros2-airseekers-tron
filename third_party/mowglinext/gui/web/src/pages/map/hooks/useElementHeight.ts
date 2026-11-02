import {type RefObject, useEffect, useState} from "react";

/**
 * Live border-box height of an element (0 until it mounts). Floating map cards
 * use it to sit above a toolbar whose height depends on its wrapped rows.
 */
export function useElementHeight(ref: RefObject<HTMLElement | null>, enabled = true): number {
    const [height, setHeight] = useState(0);
    useEffect(() => {
        const el = ref.current;
        if (!enabled || !el) {
            setHeight(0);
            return;
        }
        const measure = () => setHeight(el.getBoundingClientRect().height);
        measure();
        if (typeof ResizeObserver === "undefined") return;
        const ro = new ResizeObserver(measure);
        ro.observe(el);
        return () => ro.disconnect();
    }, [ref, enabled]);
    return height;
}
