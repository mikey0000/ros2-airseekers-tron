import {act, renderHook} from '@testing-library/react';
import {useState} from 'react';
import {describe, expect, it, vi} from 'vitest';
import type {Feature, Polygon} from 'geojson';
import {useMapEditing, type UseMapEditingOptions} from './useMapEditing';
import {MowingAreaFeature, NavigationFeature, ObstacleFeature, type MowingFeature} from '../../../types/map';

vi.mock('antd', () => ({App: {useApp: () => ({modal: {confirm: vi.fn()}})}}));
vi.mock('react-i18next', () => ({useTranslation: () => ({t: (key: string) => key})}));

const ring = (cx: number, cy: number, half: number) =>
    [[cx - half, cy - half], [cx + half, cy - half], [cx + half, cy + half], [cx - half, cy + half], [cx - half, cy - half]];

function lawn(id = 'area-0-area-0', cx = 0, cy = 0, half = 0.001) {
    const f = new MowingAreaFeature(id, 1);
    f.setGeometry({type: 'Polygon', coordinates: [ring(cx, cy, half)]});
    return f;
}

function drawn(coords: number[][], id = 'draw-1'): Feature<Polygon> {
    return {type: 'Feature', id, properties: {}, geometry: {type: 'Polygon', coordinates: [coords]}};
}

function setup(initial: Record<string, MowingFeature>) {
    const draw = {delete: vi.fn(), getMode: () => 'simple_select', getSelectedIds: () => [], trash: vi.fn()};
    const notification = {info: vi.fn(), warning: vi.fn(), error: vi.fn(), success: vi.fn()};
    const hook = renderHook(() => {
        const [features, setFeatures] = useState<Record<string, MowingFeature>>(initial);
        const editing = useMapEditing({
            features, setFeatures, editMap: true, mowingAreas: [],
            drawRef: {current: draw}, notification, mapInstanceRef: {current: null},
        } as unknown as UseMapEditingOptions);
        return {features, setFeatures, editing};
    });
    return {hook, draw, notification};
}

const obstacles = (features: Record<string, MowingFeature>) =>
    Object.values(features).filter((f): f is ObstacleFeature => f instanceof ObstacleFeature);

describe('adding a manual obstacle', () => {
    it('draw -> pick Obstacle -> OK adds an ObstacleFeature parented to the containing area', () => {
        const area = lawn();
        const {hook} = setup({[area.id]: area});
        act(() => hook.result.current.editing.onCreate({features: [drawn(ring(0, 0, 0.0001))]}));
        act(() => hook.result.current.editing.setNewAreaType('obstacle'));
        act(() => hook.result.current.editing.handleSaveNewArea());
        const obs = obstacles(hook.result.current.features);
        expect(obs).toHaveLength(1);
        expect(obs[0].getMowingArea().id).toBe(area.id);
        expect(obs[0].getInfo().source).toBe(0);
        expect(hook.result.current.editing.modalOpen).toBe(false);
    });

    it('uses the LATEST features, not the ones captured when the type was picked (stale closure)', () => {
        const {hook} = setup({});
        act(() => hook.result.current.editing.onCreate({features: [drawn(ring(0.01, 0.01, 0.0001))]}));
        act(() => hook.result.current.editing.setNewAreaType('obstacle'));
        // The map's areas arrive after the type was chosen (map load / undo / stream).
        const area = lawn('area-0-area-0', 0.01, 0.01);
        act(() => hook.result.current.setFeatures({[area.id]: area}));
        act(() => hook.result.current.editing.handleSaveNewArea());
        expect(obstacles(hook.result.current.features)).toHaveLength(1);
    });

    it('parents an obstacle straddling the boundary (centroid outside) to the area it overlaps', () => {
        const area = lawn();
        const {hook} = setup({[area.id]: area});
        // L-shaped ring hugging the area corner: two vertices inside, centroid outside.
        const straddle = [[0.0009, 0.0009], [0.0013, 0.0009], [0.0013, 0.0013], [0.0009, 0.0013], [0.0009, 0.0009]];
        act(() => hook.result.current.editing.onCreate({features: [drawn(straddle)]}));
        act(() => hook.result.current.editing.setNewAreaType('obstacle'));
        act(() => hook.result.current.editing.handleSaveNewArea());
        expect(obstacles(hook.result.current.features)).toHaveLength(1);
    });

    it('a failed add removes the drawn shape and says why (it never looks added)', () => {
        const area = lawn();
        const nav = new NavigationFeature('navigation-0-area-0');
        nav.setGeometry({type: 'Polygon', coordinates: [ring(0.01, 0.01, 0.001)]});
        const {hook, draw, notification} = setup({[area.id]: area, [nav.id]: nav});
        act(() => hook.result.current.editing.onCreate({features: [drawn(ring(0.01, 0.01, 0.0001), 'draw-9')]}));
        act(() => hook.result.current.editing.setNewAreaType('obstacle'));
        act(() => hook.result.current.editing.handleSaveNewArea());
        expect(obstacles(hook.result.current.features)).toHaveLength(0);
        expect(draw.delete).toHaveBeenCalledWith('draw-9');
        expect(notification.warning).toHaveBeenCalledTimes(1);
    });

    it('Cancel removes the drawn shape from the draw layer', () => {
        const area = lawn();
        const {hook, draw} = setup({[area.id]: area});
        act(() => hook.result.current.editing.onCreate({features: [drawn(ring(0, 0, 0.0001), 'draw-2')]}));
        act(() => hook.result.current.editing.setNewAreaType('obstacle'));
        act(() => hook.result.current.editing.deleteFeature());
        expect(draw.delete).toHaveBeenCalledWith('draw-2');
        expect(obstacles(hook.result.current.features)).toHaveLength(0);
        expect(hook.result.current.editing.modalOpen).toBe(false);
    });
});

describe('saving obstacles', () => {
    it('sends obstacle_info index-aligned with obstacles (manual stays user, auto keeps its source)', async () => {
        const {useMapFiles} = await import('./useMapFiles');
        const {featuresFromJSON, serializeFeatures} = await import('../../../types/map');
        const area = lawn();
        const manual = new ObstacleFeature('area-0-obstacle-0', area).setInfo(undefined);
        manual.setGeometry({type: 'Polygon', coordinates: [ring(0, 0, 0.0001)]});
        const proposal = new ObstacleFeature('area-0-obstacle-1', area)
            .setInfo({name: 'dig', source: 2, pending: true, id: 7});
        proposal.setGeometry({type: 'Polygon', coordinates: [ring(0.0005, 0.0005, 0.0001)]});
        // Undo/redo snapshots round-trip the provenance too.
        const features = featuresFromJSON(serializeFeatures({[area.id]: area, [manual.id]: manual, [proposal.id]: proposal}));
        const put = vi.fn().mockResolvedValue({});
        const notification = {info: vi.fn(), warning: vi.fn(), error: vi.fn(), success: vi.fn()};
        const hook = renderHook(() => useMapFiles({
            features, setFeatures: vi.fn(), map: undefined, setMap: vi.fn(), editMap: true,
            setEditMap: vi.fn(), setHasUnsavedChanges: vi.fn(), offsetX: 0, offsetY: 0,
            datum: [0, 0, 0], notification, guiApi: {mowglinext: {putMowglinext: put}},
            dockDirty: false, setDockDirty: vi.fn(), buildFeaturesFromMap: vi.fn(),
        } as never));
        await act(() => hook.result.current.handleSaveMap());
        expect(notification.error).not.toHaveBeenCalled();
        const sent = put.mock.calls[0][0].areas[0].area;
        expect(sent.obstacles).toHaveLength(2);
        expect(sent.obstacle_info).toEqual([
            {name: '', source: 0, pending: false, id: 0},
            {name: 'dig', source: 2, pending: true, id: 7},
        ]);
    });
});
