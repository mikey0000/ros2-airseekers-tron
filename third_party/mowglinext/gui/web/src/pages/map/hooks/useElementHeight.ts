import {useEffect, useState} from "react";

/**
 * Live border-box height of an element (0 while it is not mounted), via a
 * callback ref so it follows the element through remounts: MapPage renders a
 * loading branch first, and an object ref read once in an effect never saw
 * the toolbar mount later. Floating map cards use it to sit above a toolbar
 * whose height depends on its wrapped rows.
 */
export function useElementHeight(): [(el: HTMLElement | null) => void, number] {
    const [el, setEl] = useState<HTMLElement | null>(null);
    const [height, setHeight] = useState(0);
    useEffect(() => {
        if (!el) {
            setHeight(0);
            return;
        }
        const measure = () => setHeight(el.getBoundingClientRect().height);
        measure();
        if (typeof ResizeObserver === "undefined") return;
        const ro = new ResizeObserver(measure);
        ro.observe(el);
        return () => ro.disconnect();
    }, [el]);
    return [setEl, height];
}
