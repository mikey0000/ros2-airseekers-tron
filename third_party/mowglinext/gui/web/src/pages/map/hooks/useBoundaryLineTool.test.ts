import {describe, expect, it, vi} from 'vitest';
import {act, renderHook} from '@testing-library/react';
import {useBoundaryLineTool} from './useBoundaryLineTool.ts';
import {transpose} from '../../../utils/map.tsx';

const datum: [number, number, number] = [48.0, 2.0, 0];
const ll = (x: number, y: number): [number, number] => transpose(0, 0, datum, y, x);

function setup() {
    const onFinish = vi.fn();
    const draw = {changeMode: vi.fn(), getMode: () => 'simple_select'};
    const hook = renderHook(() => useBoundaryLineTool({
        drawRef: {current: draw as never}, mapInstanceRef: {current: null}, onFinish,
    }));
    return {hook, onFinish, draw};
}

describe('useBoundaryLineTool', () => {
    it('draws a line: start, add vertices, finish hands the points over', () => {
        const {hook, onFinish, draw} = setup();
        act(() => hook.result.current.start());
        expect(draw.changeMode).toHaveBeenCalledWith('simple_select', {featureIds: []});
        expect(hook.result.current.drawing).toBe(true);
        act(() => hook.result.current.addVertex(ll(0, 0)));
        act(() => hook.result.current.finish());
        expect(onFinish).not.toHaveBeenCalled();
        act(() => hook.result.current.addVertex(ll(5, 0)));
        expect(hook.result.current.canFinish).toBe(true);
        expect(hook.result.current.previewCollection.features).toHaveLength(3);
        act(() => hook.result.current.finish());
        expect(onFinish).toHaveBeenCalledWith([ll(0, 0), ll(5, 0)]);
        expect(hook.result.current.drawing).toBe(false);
    });

    it('drops a vertex closer than the minimum spacing (double-tap jitter)', () => {
        const {hook} = setup();
        act(() => hook.result.current.start());
        act(() => hook.result.current.addVertex(ll(0, 0), 0.1));
        act(() => hook.result.current.addVertex(ll(0.05, 0), 0.1));
        expect(hook.result.current.pointCount).toBe(1);
    });

    it('Escape cancels, Enter finishes', () => {
        const {hook, onFinish} = setup();
        act(() => hook.result.current.start());
        act(() => window.dispatchEvent(new KeyboardEvent('keydown', {key: 'Escape'})));
        expect(hook.result.current.drawing).toBe(false);
        act(() => hook.result.current.start());
        act(() => hook.result.current.addVertex(ll(0, 0)));
        act(() => hook.result.current.addVertex(ll(0, 3)));
        act(() => window.dispatchEvent(new KeyboardEvent('keydown', {key: 'Enter'})));
        expect(onFinish).toHaveBeenCalledTimes(1);
    });
});
