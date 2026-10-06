import {useEffect, useMemo} from "react";
import {Layer, Source, useMap} from "react-map-gl/mapbox";
import {httpBase} from "../../../utils/apiHost.ts";
import {
    drawableOverlays,
    imageCornersMap,
    imageryImageUrl,
    imageryLayerId,
    imageryTileUrl,
    quadToLngLat,
    type ImageryOverlay,
    type ImageryPlacement,
} from "../../../utils/imagery.ts";

interface Props {
    overlays: ImageryOverlay[];
    offsetX: number;
    offsetY: number;
    datum: [number, number, number];
    /** Live (unsaved) placement of the overlay being aligned. */
    draft?: {name: string; placement: ImageryPlacement; opacity?: number} | null;
}

/**
 * Custom imagery raster layers, kept directly above the Mapbox base style and
 * below every robot/map layer, stacked in list order (index 0 = bottom).
 */
export function ImageryLayers({overlays, offsetX, offsetY, datum, draft}: Props) {
    const {current: map} = useMap();
    const list = useMemo(() => {
        const withDraft = overlays.map((o) => draft && o.name === draft.name
            ? {...o, placement: draft.placement, visible: true, opacity: draft.opacity ?? o.opacity}
            : o);
        return drawableOverlays(withDraft);
    }, [overlays, draft]);
    const order = list.map((o) => imageryLayerId(o.name)).join("|");

    // Keep the stack order and keep imagery under the app layers, which are
    // (re)added imperatively (mapbox-gl-draw) or after a style switch.
    useEffect(() => {
        if (!map || !order) return;
        const m = map.getMap();
        const ids = order.split("|");
        let scheduled = false;
        const fix = () => {
            if (scheduled) return;
            scheduled = true;
            requestAnimationFrame(() => {
                scheduled = false;
                try {
                    const layers = (m.getStyle()?.layers ?? []).map((l) => l.id);
                    const present = ids.filter((id) => layers.includes(id));
                    if (!present.length) return;
                    // First app layer above the base style = our anchor.
                    const anchor = layers.includes("custom-layer") ? nextAfter(layers, "custom-layer", present)
                        : layers.includes("mower") ? "mower" : undefined;
                    const want = present.join("|");
                    const firstIdx = layers.indexOf(present[0]);
                    const current = layers.slice(firstIdx, firstIdx + present.length).join("|");
                    const anchorOk = !anchor || layers.indexOf(anchor) === firstIdx + present.length;
                    if (current === want && anchorOk) return;
                    for (const id of present) m.moveLayer(id, anchor);
                } catch { /* style not ready */ }
            });
        };
        fix();
        m.on("styledata", fix);
        return () => { m.off("styledata", fix); };
    }, [map, order]);

    const base = httpBase();
    return (
        <>
            {list.map((o) => {
                const id = imageryLayerId(o.name);
                const paint = {"raster-opacity": o.opacity, "raster-fade-duration": 0};
                if (o.kind === "image") {
                    const corners = quadToLngLat(imageCornersMap(o.placement!, o.width!, o.height!), offsetX, offsetY, datum);
                    return (
                        <Source key={o.name} id={`imagery-${o.name}`} type="image" url={imageryImageUrl(o, base)} coordinates={corners}>
                            <Layer id={id} type="raster" paint={{...paint, "raster-resampling": "linear"}}/>
                        </Source>
                    );
                }
                return (
                    <Source key={o.name} id={`imagery-${o.name}`} type="raster" tileSize={256}
                            tiles={[imageryTileUrl(o, base)]}
                            minzoom={o.minZoom ?? 0} maxzoom={o.maxZoom ?? 22}
                            bounds={o.bounds}>
                        <Layer id={id} type="raster" paint={paint}/>
                    </Source>
                );
            })}
        </>
    );
}

/** The first layer after `after` that is not one of ours (null = top). */
function nextAfter(layers: string[], after: string, ours: string[]): string | undefined {
    for (let i = layers.indexOf(after) + 1; i < layers.length; i++) {
        if (!ours.includes(layers[i])) return layers[i];
    }
    return undefined;
}
