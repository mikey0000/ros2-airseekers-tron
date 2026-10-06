import {useEffect} from "react";
import {useMap} from "react-map-gl/mapbox";

/** Style-image id used as `fill-pattern` for the automatic dock corridor. */
export const CORRIDOR_HATCH_IMAGE = "corridor-hatch";

const SIZE = 12;

function hatchImage(color: string): ImageData | null {
    if (typeof document === "undefined") return null;
    const canvas = document.createElement("canvas");
    canvas.width = SIZE;
    canvas.height = SIZE;
    const ctx = canvas.getContext("2d");
    if (!ctx) return null;
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    // 45° diagonal, drawn three times so the tile repeats seamlessly.
    for (const o of [-SIZE, 0, SIZE]) {
        ctx.beginPath();
        ctx.moveTo(o, SIZE);
        ctx.lineTo(o + SIZE, 0);
        ctx.stroke();
    }
    return ctx.getImageData(0, 0, SIZE, SIZE);
}

/**
 * Registers the diagonal hatch used by the dock-corridor layer. Re-added on
 * `styleimagemissing`, because switching the base style (satellite / dark)
 * drops every runtime image. Renders nothing.
 */
export const CorridorHatchPattern = ({color}: { color: string }) => {
    const {current} = useMap();
    useEffect(() => {
        const map = current?.getMap();
        if (!map) return;
        const add = () => {
            if (map.hasImage(CORRIDOR_HATCH_IMAGE)) return;
            const img = hatchImage(color);
            if (img) map.addImage(CORRIDOR_HATCH_IMAGE, img);
        };
        const onMissing = (e: { id: string }) => {
            if (e.id === CORRIDOR_HATCH_IMAGE) add();
        };
        add();
        map.on("styleimagemissing", onMissing);
        return () => {
            map.off("styleimagemissing", onMissing);
        };
    }, [current, color]);
    return null;
};
