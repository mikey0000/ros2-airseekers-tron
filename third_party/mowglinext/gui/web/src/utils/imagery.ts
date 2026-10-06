/**
 * Custom map imagery (drone orthomosaics, XYZ / MBTiles sets, hand-aligned
 * photos). Pure helpers shared by the Map page layers, the Imagery panel and
 * the alignment tool — see docs/custom_imagery.md.
 *
 * Plain images are georeferenced in the ROBOT MAP FRAME (metres, x east,
 * y north, the frame /pose and the map areas use), not in lon/lat: a
 * similarity transform (centre, metres-per-pixel, rotation). They are drawn
 * through utils/map.tsx `transpose`, so they move with the map offset exactly
 * like the areas and the robot do.
 */
import {transpose} from "./map.tsx";

export type ImageryKind = "geotiff" | "image" | "xyz" | "mbtiles";

export interface ImageryPlacement {
    centerX: number;
    centerY: number;
    metersPerPixel: number;
    /** CCW rotation of the image x-axis from map +x (east), degrees. */
    rotationDeg: number;
}

export interface ImageryControlPoint {
    /** Normalised image coordinates, origin top-left. */
    u: number;
    v: number;
    /** Map-frame target in metres. */
    x: number;
    y: number;
}

export interface ImageryOverlay {
    name: string;
    label: string;
    kind: ImageryKind;
    status: "processing" | "ready" | "error";
    error?: string;
    progress: number;
    opacity: number;
    visible: boolean;
    minZoom?: number;
    maxZoom?: number;
    bounds?: [number, number, number, number];
    crs?: string;
    gsd?: number;
    tileExt?: string;
    width?: number;
    height?: number;
    imageExt?: string;
    placement?: ImageryPlacement;
    controlPoints?: ImageryControlPoint[];
    sizeBytes: number;
    createdAt: number;
}

export const IMAGERY_MAX_UPLOAD_BYTES = 300 * 1024 * 1024;
export const IMAGERY_ACCEPT = ".tif,.tiff,.png,.jpg,.jpeg,.webp,.zip,.mbtiles";
/** Default ground resolution for a freshly placed plain image (3 cm/px). */
export const DEFAULT_METERS_PER_PIXEL = 0.03;

export type XY = [number, number];
/** TL, TR, BR, BL — the order Mapbox image sources expect. */
export type Quad = [XY, XY, XY, XY];

const DEG = Math.PI / 180;

/** Normalised image (u,v) → map-frame metres. */
export function imageToMap(p: ImageryPlacement, w: number, h: number, u: number, v: number): XY {
    const dx = (u - 0.5) * w * p.metersPerPixel;
    const dy = -(v - 0.5) * h * p.metersPerPixel; // image y grows downwards
    const c = Math.cos(p.rotationDeg * DEG);
    const s = Math.sin(p.rotationDeg * DEG);
    return [p.centerX + dx * c - dy * s, p.centerY + dx * s + dy * c];
}

/** Map-frame metres → normalised image (u,v). Inverse of imageToMap. */
export function mapToImage(p: ImageryPlacement, w: number, h: number, x: number, y: number): XY {
    const c = Math.cos(p.rotationDeg * DEG);
    const s = Math.sin(p.rotationDeg * DEG);
    const ex = x - p.centerX;
    const ey = y - p.centerY;
    const dx = ex * c + ey * s;
    const dy = -ex * s + ey * c;
    return [dx / (w * p.metersPerPixel) + 0.5, -dy / (h * p.metersPerPixel) + 0.5];
}

export function imageCornersMap(p: ImageryPlacement, w: number, h: number): Quad {
    return [
        imageToMap(p, w, h, 0, 0),
        imageToMap(p, w, h, 1, 0),
        imageToMap(p, w, h, 1, 1),
        imageToMap(p, w, h, 0, 1),
    ];
}

/** Map-frame quad → [lon, lat] quad through the same projection as the areas. */
export function quadToLngLat(q: Quad, offsetX: number, offsetY: number, datum: [number, number, number]): Quad {
    return q.map(([x, y]) => transpose(offsetX, offsetY, datum, y, x)) as Quad;
}

/**
 * Solve the similarity transform (translation + uniform scale + rotation,
 * no shear/mirror) that maps two image control points onto their map-frame
 * targets exactly. Returns null when the points are degenerate (coincident
 * in the image or on the ground).
 */
export function solveTwoPointPlacement(a: ImageryControlPoint, b: ImageryControlPoint, w: number, h: number): ImageryPlacement | null {
    // Image points in a north-up pixel frame centred on the image.
    const pa: XY = [(a.u - 0.5) * w, -(a.v - 0.5) * h];
    const pb: XY = [(b.u - 0.5) * w, -(b.v - 0.5) * h];
    const dp: XY = [pb[0] - pa[0], pb[1] - pa[1]];
    const dq: XY = [b.x - a.x, b.y - a.y];
    const lp = Math.hypot(dp[0], dp[1]);
    const lq = Math.hypot(dq[0], dq[1]);
    if (lp < 1 || lq < 0.05) return null;
    const scale = lq / lp;
    const theta = Math.atan2(dq[1], dq[0]) - Math.atan2(dp[1], dp[0]);
    const c = Math.cos(theta);
    const s = Math.sin(theta);
    // centre = qa - s*R*pa
    const centerX = a.x - scale * (pa[0] * c - pa[1] * s);
    const centerY = a.y - scale * (pa[0] * s + pa[1] * c);
    return {centerX, centerY, metersPerPixel: scale, rotationDeg: normalizeDeg(theta / DEG)};
}

/** Wrap to (-180, 180]. */
export function normalizeDeg(d: number): number {
    let r = ((d + 180) % 360 + 360) % 360 - 180;
    if (r === -180) r = 180;
    return r;
}

/**
 * Freeform corner-handle drag: the top-right corner is dragged to `target`
 * (map frame); keep the centre fixed and derive the new scale + rotation.
 */
export function placementFromCornerDrag(p: ImageryPlacement, w: number, h: number, target: XY): ImageryPlacement {
    const vx = target[0] - p.centerX;
    const vy = target[1] - p.centerY;
    const len = Math.hypot(vx, vy);
    const half = Math.hypot(w / 2, h / 2);
    if (len < 0.05 || half === 0) return p;
    const base = Math.atan2(h / 2, w / 2); // TR corner angle in the north-up pixel frame
    return {
        ...p,
        metersPerPixel: len / half,
        rotationDeg: normalizeDeg((Math.atan2(vy, vx) - base) / DEG),
    };
}

export function defaultPlacement(center: XY): ImageryPlacement {
    return {centerX: center[0], centerY: center[1], metersPerPixel: DEFAULT_METERS_PER_PIXEL, rotationDeg: 0};
}

/** Tile URL template for Mapbox; createdAt busts the browser cache on re-upload. */
export function imageryTileUrl(o: Pick<ImageryOverlay, "name" | "createdAt" | "tileExt">, base = ""): string {
    const ext = o.tileExt || ".png";
    return `${base}/api/mowglinext/tiles/${encodeURIComponent(o.name)}/{z}/{x}/{y}${ext}?v=${o.createdAt}`;
}

export function imageryImageUrl(o: Pick<ImageryOverlay, "name" | "createdAt">, base = ""): string {
    return `${base}/api/mowglinext/imagery/${encodeURIComponent(o.name)}/image?v=${o.createdAt}`;
}

/** Overlays the map should draw (ready, visible, and placed if plain images). */
export function drawableOverlays(list: ImageryOverlay[]): ImageryOverlay[] {
    return list.filter((o) => o.status === "ready" && o.visible
        && (o.kind !== "image" || (!!o.placement && !!o.width && !!o.height)));
}

/** Move an overlay one step up (towards the top of the stack) or down. */
export function moveOverlay(names: string[], name: string, dir: "up" | "down"): string[] {
    const i = names.indexOf(name);
    if (i < 0) return names;
    const j = dir === "up" ? i + 1 : i - 1; // index 0 = bottom-most
    if (j < 0 || j >= names.length) return names;
    const out = names.slice();
    [out[i], out[j]] = [out[j], out[i]];
    return out;
}

export function clampOpacity(v: number): number {
    if (!Number.isFinite(v)) return 1;
    return Math.min(1, Math.max(0, v));
}

export function imageryLayerId(name: string): string {
    return `imagery-${name}-layer`;
}

export function validateUpload(file: {name: string; size: number}): string | null {
    const ext = file.name.toLowerCase().replace(/^.*(\.[^.]+)$/, "$1");
    if (!IMAGERY_ACCEPT.split(",").includes(ext)) return "type";
    if (file.size > IMAGERY_MAX_UPLOAD_BYTES) return "size";
    return null;
}
