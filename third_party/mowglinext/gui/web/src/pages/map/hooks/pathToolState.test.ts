import {describe, it, expect} from 'vitest';
import {clampWidth, initialPathToolState, pathToolReducer, type PathToolState} from './pathToolState.ts';

const line = [[0, 0], [1, 1]];

describe('pathToolReducer', () => {
    it('draws a new path: idle -> drawing -> editing -> saved', () => {
        let s: PathToolState = initialPathToolState();
        expect(s).toMatchObject({phase: 'idle', width: 0.7});
        s = pathToolReducer(s, {type: 'start'});
        expect(s.phase).toBe('drawing');
        s = pathToolReducer(s, {type: 'coords', coords: line});
        expect(s.coords).toEqual(line);
        s = pathToolReducer(s, {type: 'lineFinished', lineId: 'L', coords: line, defaultName: 'Path 3'});
        expect(s).toMatchObject({phase: 'editing', lineId: 'L', name: 'Path 3', editFeatureId: null});
        s = pathToolReducer(s, {type: 'width', width: 1.2});
        s = pathToolReducer(s, {type: 'saved'});
        expect(s).toMatchObject({phase: 'idle', lineId: null, coords: null, width: 1.2});
    });

    it('starts at the dock and aborts back to idle', () => {
        let s = pathToolReducer(initialPathToolState(), {type: 'start', fromDock: true, lineId: 'seed'});
        expect(s).toMatchObject({phase: 'drawing', fromDock: true, lineId: 'seed'});
        s = pathToolReducer(s, {type: 'aborted'});
        expect(s).toMatchObject({phase: 'idle', lineId: null, fromDock: false});
    });

    it('re-edits an existing path and keeps its name/width', () => {
        const s = pathToolReducer(initialPathToolState(), {
            type: 'editExisting', featureId: 'navigation-1-area-0', lineId: 'L2', coords: line, name: 'Dock path', width: 9,
        });
        expect(s).toMatchObject({phase: 'editing', editFeatureId: 'navigation-1-area-0', name: 'Dock path', width: 3});
        expect(pathToolReducer(s, {type: 'cancel'}).phase).toBe('idle');
    });

    it('ignores stray events', () => {
        const idle = initialPathToolState();
        expect(pathToolReducer(idle, {type: 'coords', coords: line})).toBe(idle);
        expect(pathToolReducer(idle, {type: 'lineFinished', lineId: 'x', coords: line, defaultName: 'n'})).toBe(idle);
        expect(pathToolReducer(idle, {type: 'aborted'})).toBe(idle);
    });

    it('clamps the width to 0.5..3.0 m', () => {
        expect(clampWidth(0.1)).toBe(0.5);
        expect(clampWidth(5)).toBe(3);
        expect(clampWidth(NaN)).toBe(0.7);
    });
});
