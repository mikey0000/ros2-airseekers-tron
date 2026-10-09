import {useEffect, useState} from "react";

/**
 * True while the browser tab is visible (Page Visibility API). Camera views use
 * it to drop their MJPEG/snapshot connections in a background tab: the robot
 * only subscribes to and encodes a camera while a client is connected to it.
 */
export function usePageVisible(): boolean {
    const [visible, setVisible] = useState(typeof document === "undefined" || !document.hidden);
    useEffect(() => {
        if (typeof document === "undefined") return;
        const h = () => setVisible(!document.hidden);
        h();
        document.addEventListener("visibilitychange", h);
        return () => document.removeEventListener("visibilitychange", h);
    }, []);
    return visible;
}
