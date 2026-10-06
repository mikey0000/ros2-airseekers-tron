import {describe, it, expect, vi} from 'vitest';
import {render, screen} from '@testing-library/react';
import en from "../../../i18n/locales/en.json";
import {hasLiveProgress, MowProgressCard} from './MowProgressCard.tsx';
import {WhyStoppedPanel} from './WhyStoppedPanel.tsx';
import {whyLabelKey} from './MissionStatusLine.tsx';
import {parseMissionProgress} from '../../../utils/missionProgress.ts';

vi.mock('../../../theme/ThemeContext.tsx', () => ({useThemeMode: () => ({colors: {}})}));

const colors = {mowed: '#167A48', current: '#0ff', remaining: '#45D688', skipped: '#FF7A1A', track: '#fff'};
const p = parseMissionProgress({data: JSON.stringify({
    plan_id: 'p', area: 2, state: 'MOWING', sub_state: '', sub_path: 2, sub_paths: 12, pose_index: 40,
    total_poses: 300, mowed_segments: [[0, 39]], current_segment: [40, 60], skipped: [[10, 12]],
    blade_on: true, percent: 41.6, mowed_m: 50, remaining_m: 70.4, elapsed_s: 1300, eta_s: 1500,
    why: {},
})})!;

describe('MowProgressCard', () => {
    it('shows percent, sub-path n/m, ETA and length left on desktop', () => {
        render(<MowProgressCard progress={p} colors={colors} areaName="Front"/>);
        expect(screen.getByText('Mowing Front')).toBeInTheDocument();
        expect(screen.getByTestId('mow-progress-sub').textContent).toBe('3/12');
        expect(screen.getByTestId('mow-progress-eta').textContent).toBe('25 min');
        expect(screen.getByText('70 m')).toBeInTheDocument();
        expect(screen.getByText('21 min')).toBeInTheDocument();          // elapsed
        expect(screen.getByText(en.mowProgress.legendSkipped)).toBeInTheDocument();
    });
    it('renders a compact bar on a phone', () => {
        render(<MowProgressCard compact progress={p} colors={colors}/>);
        expect(screen.getByTestId('mow-progress-bar')).toBeInTheDocument();
        expect(screen.getByText('42 %')).toBeInTheDocument();
        expect(screen.getByText('3/12 · 25 min left · 70 m to go')).toBeInTheDocument();
    });
    it('hasLiveProgress needs a planned area', () => {
        expect(hasLiveProgress(p)).toBe(true);
        expect(hasLiveProgress({...p, area: -1})).toBe(false);
        expect(hasLiveProgress(null)).toBe(false);
    });
});

describe('WhyStoppedPanel', () => {
    it('explains obstacle, camera, costmap and safety inputs', () => {
        render(<WhyStoppedPanel stateName="MOWING" subState="waiting for the obstacle to clear"
            why={{obstacle: {kind: 'dynamic', class: 'person', distance_m: 1.24, bearing_deg: -12, wall_time: 100},
                  stereo_stale_s: 6, lift: true}}
            detections={{count: 1, classes: ['person'], max_score: 0.8, frame_id: 'front_camera'}}
            detectionsAt={99_000} now={100_000}/>);
        expect(screen.getByText('waiting for the obstacle to clear')).toBeInTheDocument();
        expect(screen.getByText('person · moving (waits) · 1.2 m · -12° · 0 s ago')).toBeInTheDocument();
        expect(screen.getByText('front_camera: person · 1 s ago')).toBeInTheDocument();
        expect(screen.getByText('stereo depth stale for 6 s')).toBeInTheDocument();
        expect(screen.getByText('lifted')).toBeInTheDocument();
    });
    it('says all clear without data', () => {
        render(<WhyStoppedPanel stateName="IDLE" subState="" why={null}/>);
        expect(screen.getByText(en.whyStopped.noReason)).toBeInTheDocument();
        expect(screen.getByText(en.whyStopped.allClear)).toBeInTheDocument();
    });
    it('labels the button by state', () => {
        expect(whyLabelKey('MOWING', 'sub-path 3/12')).toBe('whyStopped.details');
        expect(whyLabelKey('MOWING', 'waiting for obstacle')).toBe('whyStopped.button');
        expect(whyLabelKey('IDLE', '')).toBe('whyStopped.button');
    });
});
