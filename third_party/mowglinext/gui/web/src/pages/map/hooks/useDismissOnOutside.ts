import {type RefObject, useEffect, useRef} from "react";

/** Pointer travel (px) above which a press counts as a map pan, not a tap. */
const TAP_SLOP_PX = 8;

/** Portalled antd overlays: a press inside one belongs to the card that opened it. */
const OVERLAY_SELECTOR = ".ant-modal-root, .ant-modal-wrap, .ant-popover, .ant-dropdown, .ant-select-dropdown, .ant-drawer, .ant-message, .ant-tooltip";

const modalOpen = (doc: Document) => Array.from(doc.querySelectorAll<HTMLElement>(".ant-modal-wrap"))
    .some((el) => el.style.display !== "none");

/**
 * Close a floating map card on Escape or on a tap outside it. A press that
 * moves more than a few pixels (panning the map) does not count, and neither
 * does a press inside an antd modal/popover the card itself opened (Clear
 * memory confirm). Escape is left to an open modal first.
 */
export function useDismissOnOutside(
    ref: RefObject<HTMLElement | null>,
    onDismiss: () => void,
    enabled = true,
): void {
    const dismissRef = useRef(onDismiss);
    dismissRef.current = onDismiss;

    useEffect(() => {
        if (!enabled) return;
        let down: {x: number; y: number; outside: boolean} | null = null;

        const isOutside = (target: EventTarget | null) => {
            const node = target instanceof Node ? target : null;
            if (!node || !ref.current || ref.current.contains(node)) return false;
            const el = node instanceof Element ? node : node.parentElement;
            return !el?.closest(OVERLAY_SELECTOR);
        };
        const onPointerDown = (e: PointerEvent) => {
            down = {x: e.clientX, y: e.clientY, outside: isOutside(e.target)};
        };
        const onPointerUp = (e: PointerEvent) => {
            const start = down;
            down = null;
            if (!start?.outside) return;
            if (Math.hypot(e.clientX - start.x, e.clientY - start.y) > TAP_SLOP_PX) return;
            dismissRef.current();
        };
        const onKeyDown = (e: KeyboardEvent) => {
            if (e.key !== "Escape" || e.defaultPrevented || modalOpen(document)) return;
            dismissRef.current();
        };
        document.addEventListener("pointerdown", onPointerDown, true);
        document.addEventListener("pointerup", onPointerUp, true);
        document.addEventListener("keydown", onKeyDown);
        return () => {
            document.removeEventListener("pointerdown", onPointerDown, true);
            document.removeEventListener("pointerup", onPointerUp, true);
            document.removeEventListener("keydown", onKeyDown);
        };
    }, [enabled, ref]);
}
