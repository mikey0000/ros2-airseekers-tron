import {useEffect} from "react";
import {Layer, Source, useMap} from "react-map-gl/mapbox";
import type {MarkerCorners} from "../../../utils/mapMarker.ts";

/**
 * A profile marker image pinned to four geographic corners (see
 * utils/mapMarker.ts). Linear resampling + no fade keeps it crisp and
 * flicker-free while the pose streams in.
 *
 * mapbox-gl-draw adds its area polygon layers imperatively after this layer
 * exists, which would put the robot under the areas. Every time the style's
 * layer list changes the raster layer is moved back to the top.
 */
export function MapImageMarker({id, src, corners}: {id: string; src: string; corners: MarkerCorners | null}) {
    const {current: map} = useMap();
    const layerId = `${id}-layer`;
    const active = !!corners;

    useEffect(() => {
        if (!map || !active) return;
        const m = map.getMap();
        let scheduled = false;
        const toTop = () => {
            if (scheduled) return;
            scheduled = true;
            // Defer: moveLayer inside a styledata handler re-fires styledata.
            requestAnimationFrame(() => {
                scheduled = false;
                try {
                    if (!m.getLayer(layerId)) return;
                    const layers = m.getStyle()?.layers ?? [];
                    if (layers.length && layers[layers.length - 1].id !== layerId) m.moveLayer(layerId);
                } catch { /* style not ready */ }
            });
        };
        toTop();
        m.on("styledata", toTop);
        return () => { m.off("styledata", toTop); };
    }, [map, layerId, active]);

    if (!corners) return null;
    return (
        <Source type={"image"} id={id} url={src} coordinates={corners}>
            <Layer type={"raster"} id={layerId} paint={{
                "raster-opacity": 1,
                "raster-fade-duration": 0,
                "raster-resampling": "linear",
            }}/>
        </Source>
    );
}
