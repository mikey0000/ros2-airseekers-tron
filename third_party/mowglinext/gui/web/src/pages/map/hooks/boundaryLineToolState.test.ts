import {describe, expect, it} from 'vitest';
import {
    boundaryLineToolReducer,
    canFinishBoundaryLine,
    initialBoundaryLineToolState,
    type BoundaryLineToolState,
} from './boundaryLineToolState.ts';

const start = () => boundaryLineToolReducer(initialBoundaryLineToolState(), {type: 'start'});

describe('boundaryLineToolReducer', () => {
    it('adds vertices while drawing and ignores clicks when idle', () => {
        expect(boundaryLineToolReducer(initialBoundaryLineToolState(), {type: 'add', point: [1, 1]}).points).toEqual([]);
        let s: BoundaryLineToolState = start();
        s = boundaryLineToolReducer(s, {type: 'add', point: [1, 1]});
        s = boundaryLineToolReducer(s, {type: 'add', point: [2, 2]});
        expect(s).toMatchObject({phase: 'drawing', points: [[1, 1], [2, 2]]});
    });

    it('collapses the two clicks of a double-click into one vertex', () => {
        let s = boundaryLineToolReducer(start(), {type: 'add', point: [1, 1]});
        s = boundaryLineToolReducer(s, {type: 'add', point: [1, 1]});
        expect(s.points).toHaveLength(1);
    });

    it('undoes the last vertex, and undo on no vertices is a no-op', () => {
        let s = boundaryLineToolReducer(start(), {type: 'add', point: [1, 1]});
        s = boundaryLineToolReducer(s, {type: 'add', point: [2, 2]});
        s = boundaryLineToolReducer(s, {type: 'undo'});
        expect(s.points).toEqual([[1, 1]]);
        s = boundaryLineToolReducer(boundaryLineToolReducer(s, {type: 'undo'}), {type: 'undo'});
        expect(s).toMatchObject({phase: 'drawing', points: []});
    });

    it('finishes only with at least two points', () => {
        let s = boundaryLineToolReducer(start(), {type: 'add', point: [1, 1]});
        expect(canFinishBoundaryLine(s)).toBe(false);
        s = boundaryLineToolReducer(s, {type: 'finish'});
        expect(s).toMatchObject({phase: 'drawing', points: [[1, 1]]});
        s = boundaryLineToolReducer(s, {type: 'add', point: [3, 1]});
        expect(canFinishBoundaryLine(s)).toBe(true);
        s = boundaryLineToolReducer(s, {type: 'finish'});
        expect(s).toEqual(initialBoundaryLineToolState());
    });

    it('cancels back to idle and drops the vertices', () => {
        let s = boundaryLineToolReducer(start(), {type: 'add', point: [1, 1]});
        s = boundaryLineToolReducer(s, {type: 'cursor', point: [5, 5]});
        expect(s.cursor).toEqual([5, 5]);
        s = boundaryLineToolReducer(s, {type: 'cancel'});
        expect(s).toEqual(initialBoundaryLineToolState());
    });
});
