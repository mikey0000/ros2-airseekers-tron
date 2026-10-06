import {describe, it, expect} from 'vitest';
import {appendTrack, formatDuration, parseMissionPlan, parseMissionProgress, planStretches} from './missionProgress.ts';

const plan = parseMissionPlan({data: JSON.stringify({
    plan_id: 'p1', area: 0,
    subpaths: [[[0, 0], [1, 0], [2, 0], [3, 0]], [[0, 1], [1, 1], [2, 1], [3, 1], [4, 1]], [[0, 2], [1, 2], [2, 2]]],
})})!;

const prog = (over: Record<string, unknown> = {}) => parseMissionProgress({data: JSON.stringify({
    plan_id: 'p1', area: 0, state: 'MOWING', sub_state: 'mowing', sub_path: 1, sub_paths: 3,
    pose_index: 6, total_poses: 12, mowed_segments: [[0, 6]], current_segment: [6, 8], skipped: [],
    blade_on: true, percent: 55.6, mowed_m: 5, remaining_m: 4, elapsed_s: 120, eta_s: 120,
    why: {obstacle: null}, ...over,
})})!;

describe('parseMissionPlan / parseMissionProgress', () => {
    it('decodes both and rejects garbage', () => {
        expect(plan.subpaths.map((s) => s.length)).toEqual([4, 5, 3]);
        const p = prog();
        expect(p.current_segment).toEqual([6, 8]);
        expect(p.mowed_segments).toEqual([[0, 6]]);
        expect(parseMissionPlan({data: '{"x":1}'})).toBeNull();
        expect(parseMissionProgress({data: 'nope'})).toBeNull();
        expect(parseMissionProgress({data: JSON.stringify({state: 'IDLE', mowed_segments: [[1], 'x']})})!.mowed_segments).toEqual([]);
    });
});

describe('planStretches', () => {
    it('colours mowed / current / remaining per sub-path, never joining sub-paths', () => {
        const s = planStretches(plan, prog());
        expect(s.map((x) => x.kind)).toEqual(['mowed', 'mowed', 'current', 'remaining']);
        expect(s[0].points).toEqual([[0, 0], [1, 0], [2, 0], [3, 0]]);
        expect(s[1].points).toEqual([[0, 1], [1, 1], [2, 1]]);   // abs 4..6
        expect(s[2].points).toEqual([[2, 1], [3, 1], [4, 1]]);   // abs 6..8
        expect(s[3].points).toEqual([[0, 2], [1, 2], [2, 2]]);
    });
    it('skipped wins over mowed', () => {
        const s = planStretches(plan, prog({skipped: [[1, 2]]}));
        expect(s.slice(0, 3).map((x) => x.kind)).toEqual(['mowed', 'skipped', 'mowed']);
        expect(s[1].points).toEqual([[1, 0], [2, 0]]);
    });
    it('draws nothing for another plan', () => {
        expect(planStretches(plan, prog({plan_id: 'other'}))).toEqual([]);
        expect(planStretches(null, prog())).toEqual([]);
    });
});

describe('formatDuration', () => {
    it.each([[null, '–'], [45, '45 s'], [125, '2 min'], [3900, '1 h 05 min']])('%s -> %s', (s, out) => {
        expect(formatDuration(s as number | null)).toBe(out);
    });
});

describe('appendTrack', () => {
    it('drops tiny steps and points older than the window', () => {
        let tr = appendTrack([], 0, 0, 0);
        tr = appendTrack(tr, 0.01, 0, 1000);
        expect(tr).toHaveLength(1);
        tr = appendTrack(tr, 1, 0, 2000);
        expect(tr).toHaveLength(2);
        tr = appendTrack(tr, 2, 0, 5 * 60_000 + 1500, 5 * 60_000);
        expect(tr.map((p) => p.x)).toEqual([1, 2]);
        expect(appendTrack(tr, NaN, 0, 0)).toBe(tr);
    });
});
