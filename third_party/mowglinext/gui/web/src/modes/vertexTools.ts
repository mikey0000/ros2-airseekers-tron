/**
 * Vertex editing additions to direct_select (DirectSelectWithBoxMode):
 * drag-merge onto a neighbour, a ✕ chip on the one selected vertex, the
 * Delete key, and a right-click / long-press menu (Delete point / Insert
 * point here). Every change goes through the feature + fireUpdate, so the
 * editor records it in its undo history like any vertex drag.
 *
 * Functions take the mode context (`ctx`, the mode's `this`) and its state.
 */
import i18n from 'i18next';
import {
  SNAP_PX_DESKTOP,
  SNAP_PX_PHONE,
  deleteVertex,
  findSnapNeighbour,
  hitVertexOrEdge,
  type VertexHit,
} from '../pages/map/utils/vertexEdit';

const LONG_PRESS_MS = 550;
const LONG_PRESS_SLOP_PX = 8;
const CHIP_SIZE_PX = 28;

/* eslint-disable @typescript-eslint/no-explicit-any */
type Ctx = any;
type State = any;

export const snapPx = (): number =>
  typeof window !== 'undefined' && window.matchMedia?.('(pointer: coarse)').matches ? SNAP_PX_PHONE : SNAP_PX_DESKTOP;

const isPolygon = (state: State) => state.feature?.type === 'Polygon';

/** Open vertex list of ring `r` in the draw feature (draw stores rings open). */
const ringOf = (state: State, r: number): [number, number][] => state.feature.coordinates[r] ?? [];

const parsePath = (path: string): [number, number] => {
  const [r, i] = path.split('.').map(Number);
  return [r, i];
};

const toPx = (ctx: Ctx, ring: [number, number][]): [number, number][] =>
  ring.map((c) => { const p = ctx.map.project(c); return [p.x, p.y]; });

/** Would deleting vertex `i` of ring `r` keep a valid, non-crossing ring? */
export function canDeleteVertex(state: State, r: number, i: number): boolean {
  if (!isPolygon(state)) return false;
  return deleteVertex(ringOf(state, r), i, true).ok;
}

function clearVertexSelection(ctx: Ctx, state: State) {
  state.selectedCoordPaths = [];
  ctx.clearSelectedCoordinates();
  ctx.fireActionable(state);
}

export function deleteVertexAt(ctx: Ctx, state: State, r: number, i: number): boolean {
  if (!canDeleteVertex(state, r, i)) return false;
  state.feature.removeCoordinate(`${r}.${i}`);
  ctx.fireUpdate();
  clearVertexSelection(ctx, state);
  return true;
}

export function insertMidpointAt(ctx: Ctx, state: State, r: number, edgeStart: number) {
  if (!isPolygon(state)) return;
  const ring = ringOf(state, r);
  if (ring.length < 2) return;
  const a = ring[edgeStart], b = ring[(edgeStart + 1) % ring.length];
  state.feature.addCoordinate(`${r}.${edgeStart + 1}`, (a[0] + b[0]) / 2, (a[1] + b[1]) / 2);
  ctx.fireUpdate();
  clearVertexSelection(ctx, state);
}

/**
 * After a single-vertex drag: if the vertex was dropped within the snap
 * radius of a ring neighbour, remove it (the neighbour keeps its position).
 */
export function mergeOnDrop(ctx: Ctx, state: State): boolean {
  if (!isPolygon(state) || state.selectedCoordPaths.length !== 1) return false;
  const [r, i] = parsePath(state.selectedCoordPaths[0]);
  const ring = ringOf(state, r);
  const target = findSnapNeighbour(toPx(ctx, ring), i, true, snapPx());
  if (target < 0 || !canDeleteVertex(state, r, i)) return false;
  state.feature.removeCoordinate(`${r}.${i}`);
  clearVertexSelection(ctx, state);
  return true;
}

/** Block a trash that would leave a ring with < 3 points or a crossing outline. */
export function trashAllowed(state: State): boolean {
  if (!isPolygon(state)) return true;
  const byRing = new Map<number, Set<number>>();
  for (const p of state.selectedCoordPaths as string[]) {
    const [r, i] = parsePath(p);
    if (!byRing.has(r)) byRing.set(r, new Set());
    byRing.get(r)!.add(i);
  }
  for (const [r, idx] of byRing) {
    if (ringOf(state, r).length - idx.size < 3) return false;
  }
  return true;
}

// --- ✕ chip -------------------------------------------------------------------

export function updateDeleteChip(ctx: Ctx, state: State) {
  const single = isPolygon(state) && state.selectedCoordPaths.length === 1 && !state.dragMoving && !state.rotating;
  const [r, i] = single ? parsePath(state.selectedCoordPaths[0]) : [0, 0];
  if (!single || !canDeleteVertex(state, r, i)) {
    removeDeleteChip(state);
    return;
  }
  let el: HTMLElement | undefined = state.deleteChipElement;
  if (!el) {
    el = document.createElement('button');
    const label = i18n.t('mapVertex.deletePoint');
    el.title = label;
    el.setAttribute('aria-label', label);
    el.setAttribute('data-testid', 'vertex-delete-chip');
    el.textContent = '✕';
    Object.assign(el.style, {
      position: 'absolute', top: '0', left: '0', zIndex: '1000',
      width: `${CHIP_SIZE_PX}px`, height: `${CHIP_SIZE_PX}px`, padding: '0',
      borderRadius: '50%', border: '2px solid white', background: 'rgba(255, 73, 79, 0.95)',
      color: 'white', fontSize: '14px', fontWeight: '700', lineHeight: '1', cursor: 'pointer',
      boxShadow: '0 2px 6px rgba(0,0,0,0.4)', touchAction: 'none',
    } as Partial<CSSStyleDeclaration>);
    const stop = (ev: Event) => { ev.stopPropagation(); };
    el.addEventListener('pointerdown', stop);
    el.addEventListener('mousedown', stop);
    el.addEventListener('touchstart', stop);
    el.addEventListener('click', (ev) => {
      ev.preventDefault();
      ev.stopPropagation();
      const path = state.selectedCoordPaths[0];
      if (!path) return;
      const [rr, ii] = parsePath(path);
      deleteVertexAt(ctx, state, rr, ii);
      updateDeleteChip(ctx, state);
    });
    state.deleteChipElement = el;
    ctx.map.getContainer().appendChild(el);
  }
  const p = ctx.map.project(ringOf(state, r)[i]);
  el.style.transform = `translate(${p.x + 10}px, ${p.y - 10 - CHIP_SIZE_PX}px)`;
}

export function removeDeleteChip(state: State) {
  state.deleteChipElement?.remove();
  state.deleteChipElement = undefined;
}

// --- context menu (right-click / long-press) -----------------------------------

export function closeVertexMenu(state: State) {
  state.vertexMenuElement?.remove();
  state.vertexMenuElement = undefined;
}

export function openVertexMenu(ctx: Ctx, state: State, point: {x: number; y: number}) {
  closeVertexMenu(state);
  if (!isPolygon(state)) return;
  const rings = (state.feature.coordinates as [number, number][][]).map((ring) => toPx(ctx, ring));
  const hit: VertexHit = hitVertexOrEdge(rings, [point.x, point.y], Math.max(snapPx(), 16));
  if (!hit) return;
  const menu = document.createElement('div');
  menu.setAttribute('role', 'menu');
  menu.setAttribute('data-testid', 'vertex-menu');
  Object.assign(menu.style, {
    position: 'absolute', top: '0', left: '0', zIndex: '1001', minWidth: '160px',
    transform: `translate(${point.x + 4}px, ${point.y + 4}px)`,
    background: 'rgba(32, 32, 36, 0.96)', color: 'white', borderRadius: '8px', padding: '4px',
    boxShadow: '0 4px 14px rgba(0,0,0,0.45)', fontSize: '14px',
  } as Partial<CSSStyleDeclaration>);
  const item = (label: string, testId: string, enabled: boolean, run: () => void) => {
    const b = document.createElement('button');
    b.setAttribute('role', 'menuitem');
    b.setAttribute('data-testid', testId);
    b.textContent = label;
    b.disabled = !enabled;
    Object.assign(b.style, {
      display: 'block', width: '100%', textAlign: 'left', padding: '10px 12px', minHeight: '40px',
      background: 'transparent', border: 'none', color: enabled ? 'white' : 'rgba(255,255,255,0.4)',
      cursor: enabled ? 'pointer' : 'not-allowed', borderRadius: '6px', font: 'inherit',
    } as Partial<CSSStyleDeclaration>);
    b.addEventListener('click', (ev) => {
      ev.preventDefault();
      ev.stopPropagation();
      closeVertexMenu(state);
      if (enabled) run();
      updateDeleteChip(ctx, state);
    });
    menu.appendChild(b);
  };
  const h = hit as Exclude<VertexHit, null>;
  item(i18n.t('mapVertex.deletePoint'), 'vertex-menu-delete',
    h.kind === 'vertex' && canDeleteVertex(state, h.ring, h.index),
    () => { if (h.kind === 'vertex') deleteVertexAt(ctx, state, h.ring, h.index); });
  item(i18n.t('mapVertex.insertPoint'), 'vertex-menu-insert', h.kind === 'edge',
    () => { if (h.kind === 'edge') insertMidpointAt(ctx, state, h.ring, h.edgeStart); });
  for (const ev of ['pointerdown', 'mousedown', 'touchstart', 'contextmenu']) {
    menu.addEventListener(ev, (e) => e.stopPropagation());
  }
  state.vertexMenuElement = menu;
  ctx.map.getContainer().appendChild(menu);
}

/** Register right-click and long-press; returns the cleanup. */
export function attachVertexMenu(ctx: Ctx, state: State): () => void {
  const onContextMenu = (e: any) => {
    e.originalEvent?.preventDefault?.();
    openVertexMenu(ctx, state, e.point);
  };
  ctx.map.on('contextmenu', onContextMenu);

  const canvas: HTMLElement = ctx.map.getCanvasContainer();
  let timer: ReturnType<typeof setTimeout> | undefined;
  let start: {x: number; y: number} | null = null;
  const cancel = () => { clearTimeout(timer); timer = undefined; start = null; };
  const local = (t: Touch) => {
    const rect = ctx.map.getContainer().getBoundingClientRect();
    return {x: t.clientX - rect.left, y: t.clientY - rect.top};
  };
  const onTouchStart = (ev: TouchEvent) => {
    cancel();
    if (ev.touches.length !== 1) return;
    start = local(ev.touches[0]);
    const at = start;
    timer = setTimeout(() => {
      timer = undefined;
      ctx.stopDragging(state);
      openVertexMenu(ctx, state, at);
    }, LONG_PRESS_MS);
  };
  const onTouchMove = (ev: TouchEvent) => {
    if (!start || ev.touches.length !== 1) return cancel();
    const p = local(ev.touches[0]);
    if (Math.hypot(p.x - start.x, p.y - start.y) > LONG_PRESS_SLOP_PX) cancel();
  };
  const onDocPointerDown = (ev: Event) => {
    const menu: HTMLElement | undefined = state.vertexMenuElement;
    if (menu && !menu.contains(ev.target as Node)) closeVertexMenu(state);
  };
  canvas.addEventListener('touchstart', onTouchStart, {passive: true});
  canvas.addEventListener('touchmove', onTouchMove, {passive: true});
  canvas.addEventListener('touchend', cancel);
  canvas.addEventListener('touchcancel', cancel);
  document.addEventListener('pointerdown', onDocPointerDown, true);
  return () => {
    cancel();
    ctx.map.off('contextmenu', onContextMenu);
    canvas.removeEventListener('touchstart', onTouchStart);
    canvas.removeEventListener('touchmove', onTouchMove);
    canvas.removeEventListener('touchend', cancel);
    canvas.removeEventListener('touchcancel', cancel);
    document.removeEventListener('pointerdown', onDocPointerDown, true);
    closeVertexMenu(state);
    removeDeleteChip(state);
  };
}
