import {type ImgHTMLAttributes, useEffect, useRef} from "react";

/** 1x1 transparent GIF: assigning it aborts the pending multipart request. */
const BLANK_IMG = "data:image/gif;base64,R0lGODlhAQABAAAAACH5BAEKAAEALAAAAAABAAEAAAICTAEAOw==";

type Props = ImgHTMLAttributes<HTMLImageElement> & {src: string};

/**
 * <img> for a never-ending MJPEG (multipart/x-mixed-replace) URL that closes
 * its HTTP connection as soon as it unmounts or the URL changes.
 *
 * Removing a streaming <img> from the DOM does not reliably abort its request
 * (browsers may keep a detached image loading until it is garbage collected),
 * which keeps web_video_server subscribed to the camera and encoding for
 * nobody. Pointing the element at a data: URL first ends the request at once,
 * so the robot drops the subscription (and the camera driver closes the
 * device) right away. A new URL mounts a new element (key), so the old
 * element is released rather than reused.
 */
export function MjpegImg(props: Props) {
    return <StreamImg key={props.src} {...props}/>;
}

function StreamImg(props: Props) {
    const ref = useRef<HTMLImageElement>(null);
    useEffect(() => {
        const el = ref.current;
        return () => releaseImg(el);
    }, []);
    return <img ref={ref} {...props}/>;
}

/** Abort an <img>'s pending/streaming request (no-op for null). */
function releaseImg(el: HTMLImageElement | null): void {
    if (el) el.src = BLANK_IMG;
}
