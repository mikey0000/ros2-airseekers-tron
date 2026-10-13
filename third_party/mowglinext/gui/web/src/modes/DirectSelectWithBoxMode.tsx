import MapboxDraw from '@mapbox/mapbox-gl-draw';
import i18n from 'i18next';
const {
  createSupplementaryPoints,
  constrainFeatureMovement,
  doubleClickZoom,
  moveFeatures
} = MapboxDraw.lib;

const {
  noTarget,
  isOfMetaType,
  isActiveFeature,
  isInactiveFeature,
  isShiftDown,
} = MapboxDraw.lib.CommonSelectors;
const Constants = MapboxDraw.constants;

const isVertex = isOfMetaType(Constants.meta.VERTEX);
const isMidpoint = isOfMetaType(Constants.meta.MIDPOINT);

const DirectSelectWithBoxMode: any = {};

// INTERNAL FUNCTIONS

DirectSelectWithBoxMode.fireUpdate = function () {
  this.map.fire(Constants.events.UPDATE, {
    action: Constants.updateActions.CHANGE_COORDINATES,
    features: this.getSelected().map((f: any) => f.toGeoJSON())
  });
};

DirectSelectWithBoxMode.fireActionable = function (state: any) {
  this.setActionableState({
    combineFeatures: false,
    uncombineFeatures: false,
    trash: state.selectedCoordPaths.length > 0
  });
};

DirectSelectWithBoxMode.startDragging = function (state: any, e: any) {
  this.map.dragPan.disable();
  state.canDragMove = true;
  state.dragMoveLocation = e.lngLat;
};

DirectSelectWithBoxMode.stopDragging = function (state: any) {
  this.map.dragPan.enable();
  state.dragMoving = false;
  state.canDragMove = false;
  state.dragMoveLocation = null;
};

DirectSelectWithBoxMode.onVertex = function (state: any, e: any) {
  this.startDragging(state, e);
  const about = e.featureTarget.properties;
  const selectedIndex = state.selectedCoordPaths.indexOf(about.coord_path);
  if (!isShiftDown(e) && selectedIndex === -1) {
    state.selectedCoordPaths = [about.coord_path];
  } else if (isShiftDown(e) && selectedIndex === -1) {
    state.selectedCoordPaths.push(about.coord_path);
  }

  const selectedCoordinates = this.pathsToCoordinates(state.featureId, state.selectedCoordPaths);
  this.setSelectedCoordinates(selectedCoordinates);
};

DirectSelectWithBoxMode.onMidpoint = function (state: any, e: any) {
  this.startDragging(state, e);
  const about = e.featureTarget.properties;
  state.feature.addCoordinate(about.coord_path, about.lng, about.lat);
  this.fireUpdate();
  state.selectedCoordPaths = [about.coord_path];
};

DirectSelectWithBoxMode.pathsToCoordinates = function (featureId: any, paths: any[]) {
  return paths.map(coord_path => ({ feature_id: featureId, coord_path }));
};

DirectSelectWithBoxMode.onFeature = function (state: any, e: any) {
  // `TouchEvent` is undefined on desktop Safari/Firefox — referencing it bare
  // throws a ReferenceError. Guard with `typeof` before the instanceof check.
  const isTouch = typeof TouchEvent !== 'undefined' && e.originalEvent instanceof TouchEvent;
  // Require Ctrl/Cmd key to drag the entire area on desktop (prevents accidental moves).
  // On touch devices, allow dragging without modifier since there's no keyboard.
  if (state.selectedCoordPaths.length === 0 && (isTouch || e.originalEvent.ctrlKey || e.originalEvent.metaKey)) {
    this.startDragging(state, e);
  } else {
    this.stopDragging(state);
  }
};

DirectSelectWithBoxMode.dragFeature = function (state: any, e: any, delta: any) {
  moveFeatures(this.getSelected(), delta);
  state.dragMoveLocation = e.lngLat;
};

DirectSelectWithBoxMode.dragVertex = function (state: any, delta: any) {
  const selectedCoords = state.selectedCoordPaths.map((coord_path: any) => state.feature.getCoordinate(coord_path));
  const selectedCoordPoints = selectedCoords.map((coords: any) => ({
    type: Constants.geojsonTypes.FEATURE,
    properties: {},
    geometry: {
      type: Constants.geojsonTypes.POINT,
      coordinates: coords
    }
  }));

  const constrainedDelta = constrainFeatureMovement(selectedCoordPoints, delta);
  for (let i = 0; i < selectedCoords.length; i++) {
    const coord = selectedCoords[i];
    state.feature.updateCoordinate(state.selectedCoordPaths[i], coord[0] + constrainedDelta.lng, coord[1] + constrainedDelta.lat);
  }
};

DirectSelectWithBoxMode.clickNoTarget = function () {
  this.changeMode(Constants.modes.SIMPLE_SELECT, {});
};

DirectSelectWithBoxMode.clickInactive = function () {
  this.changeMode(Constants.modes.SIMPLE_SELECT, {});
};

DirectSelectWithBoxMode.clickActiveFeature = function (state: any, e: any) {
  if (e?.originalEvent?.detail >= 2) {
    // Double-click on active feature opens the edit modal
    this.map.fire('feature.open', { feature: state.feature });
    return;
  }
  state.selectedCoordPaths = [];
  this.clearSelectedCoordinates();
  state.feature.changed();
};

// EXTERNAL FUNCTIONS

DirectSelectWithBoxMode.onSetup = function (opts: any) {
  const featureId = opts.featureId;
  const feature = this.getFeature(featureId);

  if (!feature) {
    throw new Error('You must provide a featureId to enter direct_select mode');
  }

  if (feature.type === Constants.geojsonTypes.POINT) {
    throw new TypeError('direct_select mode doesn\'t handle point features');
  }

  const state = {
    featureId,
    feature,
    dragMoveLocation: opts.startPos || null,
    dragMoving: false,
    canDragMove: false,
    selectedCoordPaths: [], //opts.coordPath ? [opts.coordPath] : [],
    boxSelecting: false,
    boxStartLocation: null,
    boxSelectElement: undefined,
    rotating: false,
    rotationHandleElement: undefined,
    rotationPointerId: undefined,
    rotationOrigin: null,
    rotationCenter: null,
    rotationStartAngle: 0,
    rotationAngle: 0,
    handleAngle: undefined,
    rotationChildren: [] as any[],
    onSecondPointer: undefined as undefined | ((ev: PointerEvent) => void),
    onMapMove: undefined as undefined | (() => void),
    onDocKeyDown: undefined as undefined | ((ev: KeyboardEvent) => void),
  };

  this.setSelectedCoordinates(this.pathsToCoordinates(featureId, state.selectedCoordPaths));
  this.setSelected(featureId);
  doubleClickZoom.disable(this);

  this.setActionableState({
    trash: true
  });

  // Rotation handle: follow pan/zoom/rotate of the map.
  this.renderRotationHandle(state);
  state.onMapMove = () => this.updateRotationHandle(state);
  this.map.on('move', state.onMapMove);

  // Escape: cancel a rotation in progress, otherwise leave the mode (which
  // removes the handle in onStop). mapbox-gl-draw only sees keys while the
  // canvas has focus, so listen on the document, but ignore typing in inputs.
  state.onDocKeyDown = (ev: KeyboardEvent) => {
    if (ev.key !== 'Escape' || ev.defaultPrevented) return;
    const t = ev.target as HTMLElement | null;
    if (t && t !== document.body && !this.map.getContainer().contains(t)) return;
    if (state.rotating) {
      this.stopRotation(state, false);
      return;
    }
    this.changeMode(Constants.modes.SIMPLE_SELECT, {});
  };
  document.addEventListener('keydown', state.onDocKeyDown);

  return state;
};

DirectSelectWithBoxMode.onKeyDown = function (state: any, e: any) {
  if (e.key == "Enter")
    this.map.fire('feature.open', {
      feature: state.feature
    });
};

DirectSelectWithBoxMode.onStop = function (state: any) {
  doubleClickZoom.enable(this);
  this.clearSelectedCoordinates();
  if (state) {
    if (state.rotating) this.stopRotation(state);
    if (state.onMapMove) this.map.off('move', state.onMapMove);
    if (state.onDocKeyDown) document.removeEventListener('keydown', state.onDocKeyDown);
    state.onMapMove = undefined;
    state.onDocKeyDown = undefined;
    this.removeRotationHandle(state);
    if (state.boxSelectElement) {
      state.boxSelectElement.remove();
      state.boxSelectElement = undefined;
    }
  }
};

DirectSelectWithBoxMode.toDisplayFeatures = function (state: any, geojson: any, push: any) {
  if (state.featureId === geojson.properties.id) {
    geojson.properties.active = Constants.activeStates.ACTIVE;
    push(geojson);
    createSupplementaryPoints(geojson, {
      midpoints: true,
      selectedPaths: state.selectedCoordPaths
    }).forEach(push);
    // Geometry may have changed (vertex drag, delete, move): keep the handle
    // at a fixed screen distance from the shape.
    this.updateRotationHandle(state);
  } else {
    geojson.properties.active = Constants.activeStates.INACTIVE;
    push(geojson);
  }
  this.fireActionable(state);
};

DirectSelectWithBoxMode.onTrash = function (state: any) {
  state.selectedCoordPaths
    .sort((a: any, b: any) => b.localeCompare(a, 'en', { numeric: true }))
    .forEach((id: any) => state.feature.removeCoordinate(id));
  this.fireUpdate();
  state.selectedCoordPaths = [];
  this.clearSelectedCoordinates();
  this.fireActionable(state);
  if (state.feature.isValid() === false) {
    this.deleteFeature([state.featureId]);
    this.changeMode(Constants.modes.SIMPLE_SELECT, {});
  }
};

DirectSelectWithBoxMode.onMouseMove = function (state: any, e: any) {
  const isFeature = isActiveFeature(e);
  const onVertex = isVertex(e);
  const isMidPoint = isMidpoint(e);
  const noCoords = state.selectedCoordPaths.length === 0;
  // Only show move cursor for whole area when Ctrl/Cmd is held
  if (isFeature && noCoords && (e.originalEvent.ctrlKey || e.originalEvent.metaKey)) this.updateUIClasses({ mouse: Constants.cursors.MOVE });
  else if (isFeature && noCoords) this.updateUIClasses({ mouse: Constants.cursors.POINTER });
  else if (onVertex && !noCoords) this.updateUIClasses({ mouse: Constants.cursors.MOVE });
  else this.updateUIClasses({ mouse: Constants.cursors.NONE });

  const isDraggableItem = onVertex || isFeature || isMidPoint;
  if (isDraggableItem && state.dragMoving) this.fireUpdate();

  return true;
};

DirectSelectWithBoxMode.onMouseOut = function (state: any) {
  if (state.dragMoving) this.fireUpdate();

  return true;
};

DirectSelectWithBoxMode.onMouseDown = DirectSelectWithBoxMode.onTouchStart = function (state: any, e: any) {
  if (this.isRotationHandle(state, e)) {
    this.startRotation(state, e.point);
    return;
  }
  if (isVertex(e)) return this.onVertex(state, e);
  
  if (isShiftDown(e)) {
    this.map.dragPan.disable();
    state.boxSelecting = true;
    state.boxStartLocation = e.point;
    return;
  }

  if (isActiveFeature(e)) return this.onFeature(state, e);
  if (isMidpoint(e)) return this.onMidpoint(state, e);
};

DirectSelectWithBoxMode.onDrag = function (state: any, e: any) {
  if (state.rotating) {
    return this.updateRotation(state, e);
  }
  if (state.boxSelecting) {
    return this.whileBoxSelect(state, e)
  }

  if (state.canDragMove !== true) return;
  state.dragMoving = true;
  e.originalEvent.stopPropagation();

  const delta = {
    lng: e.lngLat.lng - state.dragMoveLocation.lng,
    lat: e.lngLat.lat - state.dragMoveLocation.lat
  };
  if (state.selectedCoordPaths.length > 0) this.dragVertex(state, delta);
  else this.dragFeature(state, e, delta);

  state.dragMoveLocation = e.lngLat;
};

DirectSelectWithBoxMode.whileBoxSelect = function (state:any, e: any) {
  const boxEndLocation = e.point;
  this.updateUIClasses({ mouse: Constants.cursors.ADD });

  const boxElement = state.boxSelectElement;
  if (!boxElement) {
    state.boxSelectElement = document.createElement('div');
    state.boxSelectElement.classList.add(Constants.classes.BOX_SELECT);
    this.map.getContainer().appendChild(state.boxSelectElement);
  }
  const minX = Math.min(state.boxStartLocation.x, boxEndLocation.x);
  const maxX = Math.max(state.boxStartLocation.x, boxEndLocation.x);
  const minY = Math.min(state.boxStartLocation.y, boxEndLocation.y);
  const maxY = Math.max(state.boxStartLocation.y, boxEndLocation.y);
  const translateValue = `translate(${minX}px, ${minY}px)`;
  state.boxSelectElement.style.transform = translateValue;
  state.boxSelectElement.style.WebkitTransform = translateValue;
  state.boxSelectElement.style.width = `${maxX - minX}px`;
  state.boxSelectElement.style.height = `${maxY - minY}px`;
};

DirectSelectWithBoxMode.onClick = function (state: any, e: any) {
  if (noTarget(e)) return this.clickNoTarget(state, e);
  if (isActiveFeature(e)) return this.clickActiveFeature(state, e);
  if (isInactiveFeature(e)) return this.clickInactive(state, e);
  if (isVertex(e)) {
    if (!isShiftDown(e)) {
      // Clear previous selection if shift is not pressed
      state.selectedCoordPaths = [];
    }
    return this.onVertex(state, e);
  }
  this.stopDragging(state);
};

DirectSelectWithBoxMode.onTap = function (state: any, e: any) {
  if (noTarget(e)) return this.clickNoTarget(state, e);
  if (isActiveFeature(e)) return this.clickActiveFeature(state, e);
  if (isInactiveFeature(e)) return this.clickInactive(state, e);
  if (isVertex(e)) return this.onVertex(state, e);
};

DirectSelectWithBoxMode.onMouseUp = DirectSelectWithBoxMode.onTouchEnd = function (state: any, e: any) {
  if (state.boxSelectElement) {
    this.map.getContainer().removeChild(state.boxSelectElement);
    state.boxSelectElement = undefined;
  }
  if (state.boxSelecting) {
    // End box selection
    state.boxSelecting = false;
    const boxEndLocation = e.point;


    const minX = Math.min(state.boxStartLocation.x, boxEndLocation.x);
    const maxX = Math.max(state.boxStartLocation.x, boxEndLocation.x);
    const minY = Math.min(state.boxStartLocation.y, boxEndLocation.y);
    const maxY = Math.max(state.boxStartLocation.y, boxEndLocation.y);

    const selectedVertices = this.getSelectedVerticesInBox(state.feature, minX, minY, maxX, maxY);


    state.selectedCoordPaths = [...new Set([...state.selectedCoordPaths, ...selectedVertices])];

    this.setSelectedCoordinates(this.pathsToCoordinates(state.featureId, state.selectedCoordPaths));
    state.boxStartLocation = null;

    this.map.dragPan.enable();

    // Trigger a re-render of the feature
    state.feature.changed();

    return;
  }

  if (state.rotating) {
    this.stopRotation(state);
  }

  if (state.dragMoving) {
    this.fireUpdate();
  }
  this.stopDragging(state);
};


DirectSelectWithBoxMode.getSelectedVerticesInBox = function (feature: any, minX: number, minY: number, maxX: number, maxY: number) {
  const selectedVertices: any[] = [];
  const coordinates = feature.getCoordinates();

  const checkCoordinate = (coord: any, path: any) => {
    const point = this.map.project(coord);
    if (point.x >= minX && point.x <= maxX && point.y >= minY && point.y <= maxY) {
      selectedVertices.push(path);
    }
  };

  const traverseCoordinates = (coords: any, basePath = '') => {
    coords.forEach((coord: any, index: any) => {
      const currentPath = basePath ? `${basePath}.${index}` : `${index}`;
      if (Array.isArray(coord[0])) {
        traverseCoordinates(coord, currentPath);
      } else {
        checkCoordinate(coord, currentPath);
      }
    });
  };

  traverseCoordinates(coordinates);
  return selectedVertices;
};

// ---------------------------------------------------------------------------
// Rotation handle (free rotate about the area centroid)
//
// The handle is a DOM element in the map container (a sibling of the canvas
// container). mapbox-gl only listens on the canvas container, so mouse/touch
// events on the handle never reach mapbox-gl-draw's onMouseDown/onTouchStart:
// the handle drives the rotation through its own pointer listeners (mouse,
// pen and touch alike, with pointer capture so the drag survives leaving the
// handle). The onMouseDown/onTouchStart hit test below is a fallback for
// events that do arrive through the map.
//
// The pure helpers are exposed as `DirectSelectWithBoxMode.rotationMath` for
// unit tests (function exports from a .tsx trip react-refresh lint).
// ---------------------------------------------------------------------------

export type LngLatPair = [number, number];
export type PixelPoint = {x: number; y: number};
export type RectLike = {left: number; top: number; right: number; bottom: number};

const ROTATION_HANDLE_ID = 'rotation-handle';
/** Visible circle of the handle. */
export const ROTATION_HANDLE_SIZE_PX = 28;
/** Hit target around it (touch guideline size). */
export const ROTATION_HANDLE_HIT_PX = 44;
/** Keep the handle this far inside the map container (toolbars, top banner). */
export const ROTATION_HANDLE_EDGE_PX = 56;
/** Screen gap between the farthest vertex and the handle. */
export const ROTATION_HANDLE_GAP_PX = 36;
/** Default handle direction: straight up on screen (y grows downwards). */
const HANDLE_UP = -Math.PI / 2;

/** Drop the GeoJSON closing vertex if present. */
function openRing(ring: LngLatPair[]): LngLatPair[] {
  if (ring.length > 1) {
    const a = ring[0], b = ring[ring.length - 1];
    if (a[0] === b[0] && a[1] === b[1]) return ring.slice(0, -1);
  }
  return ring;
}

/**
 * Area centroid of a polygon ring (closed or open), computed in a local
 * equirectangular frame so longitude and latitude are scaled alike. Falls back
 * to the vertex mean for a degenerate (zero-area) ring.
 */
function ringCentroid(ring: LngLatPair[]): LngLatPair {
  const pts = openRing(ring);
  const n = pts.length;
  if (n === 0) return [0, 0];
  let mx = 0, my = 0;
  for (const p of pts) { mx += p[0]; my += p[1]; }
  mx /= n; my /= n;
  const k = Math.cos((my * Math.PI) / 180);
  let a2 = 0, sx = 0, sy = 0;
  for (let i = 0; i < n; i++) {
    const x0 = (pts[i][0] - mx) * k, y0 = pts[i][1] - my;
    const j = (i + 1) % n;
    const x1 = (pts[j][0] - mx) * k, y1 = pts[j][1] - my;
    const cross = x0 * y1 - x1 * y0;
    a2 += cross;
    sx += (x0 + x1) * cross;
    sy += (y0 + y1) * cross;
  }
  if (Math.abs(a2) < 1e-18 || k === 0) return [mx, my];
  return [mx + sx / (3 * a2) / k, my + sy / (3 * a2)];
}

/**
 * Rotate every vertex of every ring by `angleRad` (counter-clockwise when
 * looking down on the ground, north up) about `center`. Works in a local
 * equirectangular frame so the ground shape is preserved at any latitude.
 * Identical input vertices map to identical outputs, so a closed ring
 * (last == first) stays closed and the vertex count never changes.
 */
function rotateRings(rings: LngLatPair[][], center: LngLatPair, angleRad: number): LngLatPair[][] {
  const [cx, cy] = center;
  const k = Math.cos((cy * Math.PI) / 180) || 1e-12;
  const cos = Math.cos(angleRad), sin = Math.sin(angleRad);
  return rings.map(ring => ring.map(([lng, lat]) => {
    const dx = (lng - cx) * k;
    const dy = lat - cy;
    return [cx + (dx * cos - dy * sin) / k, cy + dx * sin + dy * cos] as LngLatPair;
  }));
}

/**
 * Handle centre in container pixels: on a circle around the centroid pixel
 * whose radius is the farthest projected vertex + `gapPx`, in screen direction
 * `angleRad` (default: straight up). Pure pixel maths, so the handle stays the
 * same screen distance from the shape at every zoom and latitude.
 */
function rotationHandlePixel(
  centroidPx: PixelPoint,
  vertexPx: PixelPoint[],
  angleRad: number = HANDLE_UP,
  gapPx: number = ROTATION_HANDLE_GAP_PX,
): PixelPoint {
  let r = 0;
  for (const p of vertexPx) r = Math.max(r, Math.hypot(p.x - centroidPx.x, p.y - centroidPx.y));
  r += gapPx;
  return {x: centroidPx.x + r * Math.cos(angleRad), y: centroidPx.y + r * Math.sin(angleRad)};
}

/**
 * Hit test in CONTAINER pixels (what mapbox puts in `e.point`). The handle's
 * `getBoundingClientRect()` is in viewport pixels, so it is shifted by the
 * container's own viewport rect first.
 */
function isPointInHandle(point: PixelPoint, handleRect: RectLike, containerRect: RectLike, slopPx = 4): boolean {
  const left = handleRect.left - containerRect.left - slopPx;
  const right = handleRect.right - containerRect.left + slopPx;
  const top = handleRect.top - containerRect.top - slopPx;
  const bottom = handleRect.bottom - containerRect.top + slopPx;
  return point.x >= left && point.x <= right && point.y >= top && point.y <= bottom;
}

/**
 * Keep the handle on screen: if `ideal` lies outside the container shrunk by
 * `marginPx` on every side, move it back towards `center` along the same ray
 * to the last point inside. If even the centre is outside (shape mostly off
 * screen), clamp coordinate-wise to the nearest point inside.
 */
function clampHandlePixel(
  center: PixelPoint, ideal: PixelPoint, width: number, height: number, marginPx: number = ROTATION_HANDLE_EDGE_PX,
): PixelPoint {
  const lo = {x: Math.min(marginPx, width / 2), y: Math.min(marginPx, height / 2)};
  const hi = {x: Math.max(width - marginPx, width / 2), y: Math.max(height - marginPx, height / 2)};
  const inside = (p: PixelPoint) => p.x >= lo.x && p.x <= hi.x && p.y >= lo.y && p.y <= hi.y;
  if (inside(ideal)) return ideal;
  const clamp = (p: PixelPoint) => ({x: Math.min(hi.x, Math.max(lo.x, p.x)), y: Math.min(hi.y, Math.max(lo.y, p.y))});
  if (!inside(center)) return clamp(ideal);
  const dx = ideal.x - center.x, dy = ideal.y - center.y;
  let t = 1;
  if (dx > 0) t = Math.min(t, (hi.x - center.x) / dx);
  if (dx < 0) t = Math.min(t, (lo.x - center.x) / dx);
  if (dy > 0) t = Math.min(t, (hi.y - center.y) / dy);
  if (dy < 0) t = Math.min(t, (lo.y - center.y) / dy);
  return clamp({x: center.x + dx * t, y: center.y + dy * t});
}

/** Even-odd point-in-ring test in lng/lat (fine at garden scale). */
function pointInRing(pt: LngLatPair, ring: LngLatPair[]): boolean {
  const pts = openRing(ring);
  let inside = false;
  for (let i = 0, j = pts.length - 1; i < pts.length; j = i++) {
    const [xi, yi] = pts[i], [xj, yj] = pts[j];
    if ((yi > pt[1]) !== (yj > pt[1]) && pt[0] < ((xj - xi) * (pt[1] - yi)) / (yj - yi) + xi) inside = !inside;
  }
  return inside;
}

/** True when every vertex of `inner` lies inside `outer`. */
function ringInsideRing(inner: LngLatPair[], outer: LngLatPair[]): boolean {
  const pts = openRing(inner);
  return pts.length > 0 && pts.every(p => pointInRing(p, outer));
}

/** Screen-space angle (radians, y down) of `p` around `c`. */
function screenAngle(c: PixelPoint, p: PixelPoint): number {
  return Math.atan2(p.y - c.y, p.x - c.x);
}

/** Pixel position of a DOM pointer event inside the map container. */
function containerPoint(map: any, ev: {clientX: number; clientY: number}): PixelPoint {
  const r = map.getContainer().getBoundingClientRect();
  return {x: ev.clientX - r.left, y: ev.clientY - r.top};
}

// --- mode methods -----------------------------------------------------------

/**
 * Only plain polygons rotate. A navigation PATH carries its centreline in
 * `properties.channel` (map_server builds the route graph from it), and the
 * editor's draw.update handler (useMapEditing.onUpdate) copies geometry only,
 * so a rotated corridor would be saved with an unrotated centreline. Paths
 * are therefore not rotatable until that handler also takes the channel.
 */
DirectSelectWithBoxMode.canRotate = function (state: any): boolean {
  const f = state.feature;
  if (f?.type !== Constants.geojsonTypes.POLYGON) return false;
  const ch = f.properties?.channel;
  return !(Array.isArray(ch) && ch.length >= 2);
};

/**
 * Obstacles that belong to the selected work area: other draw features of
 * type obstacle whose outer ring lies inside the area's outer ring. The save
 * path (useMapFiles) re-parents obstacles by the same containment test, and
 * the editor's ObstacleFeature keeps its parent link, so rotating them with
 * the area keeps them inside it.
 */
DirectSelectWithBoxMode.getChildObstacles = function (state: any): any[] {
  if (state.feature?.properties?.feature_type !== 'workarea') return [];
  const outer: LngLatPair[] = state.feature.getCoordinates()[0];
  const all: any[] = this._ctx?.store?.getAll?.() ?? [];
  return all.filter(f =>
    f !== state.feature &&
    f.id !== state.featureId &&
    f.type === Constants.geojsonTypes.POLYGON &&
    f.properties?.feature_type === 'obstacle' &&
    ringInsideRing(f.getCoordinates()[0], outer));
};

/** Centroid of the outer ring (lng/lat). */
DirectSelectWithBoxMode.getCentroid = function (feature: any): LngLatPair {
  return ringCentroid(feature.getCoordinates()[0]);
};

/** Handle centre in container pixels. */
DirectSelectWithBoxMode.getRotationHandlePixel = function (state: any): PixelPoint {
  const ring: LngLatPair[] = openRing(state.feature.getCoordinates()[0]);
  const center = state.rotating && state.rotationCenter ? state.rotationCenter : this.getCentroid(state.feature);
  const c = this.map.project(center);
  const verts = ring.map(v => this.map.project(v));
  const centerPx = {x: c.x, y: c.y};
  const ideal = rotationHandlePixel(centerPx, verts.map((p: any) => ({x: p.x, y: p.y})), state.handleAngle ?? HANDLE_UP);
  const el = this.map.getContainer();
  const w = el.clientWidth, h = el.clientHeight;
  // An unlaid-out container (0x0, e.g. jsdom) has nothing to clamp to.
  return w > 0 && h > 0 ? clampHandlePixel(centerPx, ideal, w, h) : ideal;
};

DirectSelectWithBoxMode.renderRotationHandle = function (state: any) {
  if (state.rotationHandleElement || !this.canRotate(state)) return;
  const size = ROTATION_HANDLE_SIZE_PX;
  const hit = ROTATION_HANDLE_HIT_PX;
  // Outer element = transparent 44 px hit target; inner = 28 px visible circle.
  const el = document.createElement('div');
  el.id = ROTATION_HANDLE_ID;
  el.className = 'mapboxgl-draw-rotation-handle';
  const label = i18n.t('map.rotateHandle');
  el.title = label;
  el.setAttribute('role', 'button');
  el.setAttribute('aria-label', label);
  Object.assign(el.style, {
    position: 'absolute',
    top: '0',
    left: '0',
    width: `${hit}px`,
    height: `${hit}px`,
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    cursor: 'grab',
    userSelect: 'none',
    touchAction: 'none',
    zIndex: '1000',
  } as Partial<CSSStyleDeclaration>);
  const dot = document.createElement('div');
  dot.className = 'mapboxgl-draw-rotation-handle-dot';
  dot.textContent = '↻';
  Object.assign(dot.style, {
    width: `${size}px`,
    height: `${size}px`,
    boxSizing: 'border-box',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    borderRadius: '50%',
    background: 'rgba(255, 73, 79, 0.95)',
    border: '2px solid white',
    color: 'white',
    fontSize: '18px',
    fontWeight: '700',
    lineHeight: '1',
    pointerEvents: 'none',
    boxShadow: '0 2px 6px rgba(0,0,0,0.4)',
  } as Partial<CSSStyleDeclaration>);
  el.appendChild(dot);

  el.addEventListener('pointerdown', (ev: PointerEvent) => {
    if (ev.button !== undefined && ev.button !== 0) return;
    ev.preventDefault();
    ev.stopPropagation();
    try { el.setPointerCapture(ev.pointerId); } catch { /* jsdom / old browsers */ }
    state.rotationPointerId = ev.pointerId;
    this.startRotation(state, containerPoint(this.map, ev));
  });
  el.addEventListener('pointermove', (ev: PointerEvent) => {
    if (!state.rotating || ev.pointerId !== state.rotationPointerId) return;
    ev.preventDefault();
    ev.stopPropagation();
    this.updateRotation(state, {point: containerPoint(this.map, ev)});
  });
  const end = (ev: PointerEvent) => {
    if (!state.rotating || ev.pointerId !== state.rotationPointerId) return;
    ev.stopPropagation();
    this.stopRotation(state);
  };
  el.addEventListener('pointerup', end);
  el.addEventListener('pointercancel', end);
  el.addEventListener('lostpointercapture', end);

  state.rotationHandleElement = el;
  this.map.getContainer().appendChild(el);
  this.updateRotationHandle(state);
};

DirectSelectWithBoxMode.updateRotationHandle = function (state: any) {
  const el = state.rotationHandleElement;
  if (!el) return;
  const p = this.getRotationHandlePixel(state);
  const half = ROTATION_HANDLE_HIT_PX / 2;
  el.style.transform = `translate(${p.x - half}px, ${p.y - half}px)`;
  el.style.cursor = state.rotating ? 'grabbing' : 'grab';
};

DirectSelectWithBoxMode.removeRotationHandle = function (state: any) {
  if (state.rotationHandleElement) {
    state.rotationHandleElement.remove();
    state.rotationHandleElement = undefined;
  }
};

/** Fallback hit test for events delivered through the map (`e.point` = container px). */
DirectSelectWithBoxMode.isRotationHandle = function (state: any, e: any): boolean {
  if (!state.rotationHandleElement || !e?.point) return false;
  return isPointInHandle(
    e.point,
    state.rotationHandleElement.getBoundingClientRect(),
    this.map.getContainer().getBoundingClientRect(),
  );
};

/**
 * Set the feature to `rotationOrigin` rotated by `angleRad` (ground CCW).
 * Always rotates from the snapshot taken at drag start, so per-frame error
 * never accumulates. mapbox-gl-draw's Polygon stores OPEN rings
 * (`getCoordinates()` appends the closing vertex, `setCoordinates()` does not
 * strip it), so the closing vertex is removed before writing back.
 */
DirectSelectWithBoxMode.rotateFeature = function (state: any, angleRad: number) {
  const rotated = rotateRings(state.rotationOrigin, state.rotationCenter, angleRad);
  state.feature.setCoordinates(rotated.map(openRing));
  for (const child of state.rotationChildren ?? []) {
    child.feature.setCoordinates(rotateRings(child.origin, state.rotationCenter, angleRad).map(openRing));
  }
};

DirectSelectWithBoxMode.startRotation = function (state: any, point: PixelPoint) {
  if (!this.canRotate(state)) return;
  this.stopDragging(state);
  state.rotating = true;
  state.rotationOrigin = state.feature.getCoordinates().map((r: LngLatPair[]) => r.map(c => [c[0], c[1]]));
  state.rotationCenter = this.getCentroid(state.feature);
  state.rotationChildren = this.getChildObstacles(state).map((f: any) => ({
    feature: f,
    origin: f.getCoordinates().map((r: LngLatPair[]) => r.map(c => [c[0], c[1]])),
  }));
  const c = this.map.project(state.rotationCenter);
  state.rotationStartAngle = screenAngle(c, point);
  state.rotationAngle = 0;
  this.map.dragPan.disable();
  // Touch has no Escape: a second finger anywhere cancels the rotation.
  state.onSecondPointer = (ev: PointerEvent) => {
    if (state.rotating && ev.pointerId !== state.rotationPointerId && ev.pointerType === 'touch') {
      this.stopRotation(state, false);
    }
  };
  document.addEventListener('pointerdown', state.onSecondPointer, true);
  this.updateRotationHandle(state);
};

DirectSelectWithBoxMode.updateRotation = function (state: any, e: {point: PixelPoint}) {
  if (!state.rotating) return;
  const c = this.map.project(state.rotationCenter);
  const a = screenAngle(c, e.point);
  // Screen y points down, so a clockwise screen drag is a clockwise ground
  // rotation, i.e. a negative (CCW-positive) angle.
  state.rotationAngle = a - state.rotationStartAngle;
  state.handleAngle = a;
  this.rotateFeature(state, -state.rotationAngle);
  this._ctx.store.render();
  this.updateRotationHandle(state);
};

/** End a rotation. `commit=false` restores the original shape (Escape). */
DirectSelectWithBoxMode.stopRotation = function (state: any, commit = true) {
  if (!state.rotating) return;
  state.rotating = false;
  const el = state.rotationHandleElement;
  if (el && state.rotationPointerId !== undefined) {
    try { if (el.hasPointerCapture?.(state.rotationPointerId)) el.releasePointerCapture(state.rotationPointerId); } catch { /* ignore */ }
  }
  state.rotationPointerId = undefined;
  if (state.onSecondPointer) document.removeEventListener('pointerdown', state.onSecondPointer, true);
  state.onSecondPointer = undefined;
  const children: any[] = state.rotationChildren ?? [];
  if (!commit) {
    state.feature.setCoordinates(state.rotationOrigin.map(openRing));
    for (const child of children) child.feature.setCoordinates(child.origin.map(openRing));
    state.handleAngle = undefined;
    this._ctx.store.render();
  } else if (state.rotationAngle) {
    // One draw.update per gesture (area + its obstacles), so it is saved once.
    this.map.fire(Constants.events.UPDATE, {
      action: Constants.updateActions.CHANGE_COORDINATES,
      features: [
        ...this.getSelected().map((f: any) => f.toGeoJSON()),
        ...children.map(c => c.feature.toGeoJSON()),
      ],
    });
  }
  state.rotationChildren = [];
  state.rotationOrigin = null;
  state.rotationCenter = null;
  state.rotationAngle = 0;
  this.map.dragPan.enable();
  this.updateRotationHandle(state);
};


DirectSelectWithBoxMode.rotationMath = {ringCentroid, rotateRings, rotationHandlePixel, isPointInHandle, clampHandlePixel, ringInsideRing};

export default DirectSelectWithBoxMode;
