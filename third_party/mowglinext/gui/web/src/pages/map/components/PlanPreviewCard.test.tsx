import type React from 'react';
import {describe, it, expect, vi} from 'vitest';
import {render, screen} from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import {PlanPreviewCard} from './PlanPreviewCard.tsx';
import {parsePlanPreview, canPreviewPlan} from '../../../hooks/usePlanPreview.ts';
import en from "../../../i18n/locales/en.json";

vi.mock('../../../components/AsyncButton.tsx', () => ({
    default: ({children, onAsyncClick}: {children?: React.ReactNode; onAsyncClick?: () => void}) => (
        <button onClick={onAsyncClick}>{children}</button>
    ),
}));

const colors = {ring: '#f3a85c', swath: '#45d688', transit: '#a78bfa'};

const okMsg = {data: JSON.stringify({
    id: 3, status: 'ok', area: 1, message: '',
    areas: [{index: 1, name: 'Front', path_mode: 'zigzag', headland_rings: 2, rings: 2,
             swaths: 14, length_m: 123.4, sub_paths: 2, inset_m: 0.29}],
    rings: 2, swaths: 14, length_m: 123.4, sub_paths: 2, inset_m: 0.29,
    segments: [{type: 'ring', points: [[0, 0], [1, 0]]}, {type: 'swath', points: [[0, 1], [1, 1]]},
               {type: 'bogus', points: []}],
    transits: [[[1, 0], [0, 1]]],
})};

describe('parsePlanPreview', () => {
    it('decodes the summary and drops malformed segments', () => {
        const s = parsePlanPreview(okMsg)!;
        expect(s.status).toBe('ok');
        expect(s.segments.map((g) => g.type)).toEqual(['ring', 'swath']);
        expect(s.transits).toHaveLength(1);
        expect(s.areas[0].name).toBe('Front');
    });
    it('rejects garbage', () => {
        expect(parsePlanPreview({data: 'nope'})).toBeNull();
        expect(parsePlanPreview({data: '{"status":"weird"}'})).toBeNull();
    });
    it('allows previews only at rest', () => {
        expect(canPreviewPlan('IDLE_DOCKED')).toBe(true);
        expect(canPreviewPlan('MOWING')).toBe(false);
        expect(canPreviewPlan(undefined)).toBe(false);
    });
});

const crossMsg = {data: JSON.stringify({
    id: 4, status: 'ok', area: 1, message: '',
    areas: [{index: 1, name: 'Front', path_mode: 'cross', headland_rings: 1, rings: 2,
             swaths: 4, length_m: 44, sub_paths: 4, inset_m: 0.29, run_line_angles_deg: [105, 15]}],
    rings: 2, swaths: 4, length_m: 44, sub_paths: 4, inset_m: 0.29,
    segments: [{type: 'ring', points: [[0, 0], [1, 0]], run: 0},
               {type: 'swath', points: [[0, 1], [1, 1]], run: 0},
               {type: 'swath', points: [[0, 2], [1, 2]], run: 0},
               {type: 'ring', points: [[0, 0], [1, 0]], run: 1},
               {type: 'swath', points: [[0, 1], [0, 2]], run: 1},
               {type: 'swath', points: [[1, 1], [1, 2]], run: 1}],
    transits: [],
})};

describe('PlanPreviewCard cross preview', () => {
    it('keeps each segment run (absent = first pass)', () => {
        expect(parsePlanPreview(crossMsg)!.segments.map((g) => g.run)).toEqual([0, 0, 0, 1, 1, 1]);
        expect(parsePlanPreview(okMsg)!.segments.map((g) => g.run)).toEqual([0, 0]);
        expect(parsePlanPreview(crossMsg)!.areas[0].run_line_angles_deg).toEqual([105, 15]);
    });
    it('lists the first and the cross pass with their row angles', () => {
        render(<PlanPreviewCard summary={parsePlanPreview(crossMsg)!} colors={{...colors, swathCross: '#45d6e8'}}
                                onClear={vi.fn()} onDismiss={vi.fn()}/>);
        expect(screen.getByTestId('plan-preview-pass-first')).toHaveTextContent(`${en.planPreview.swathsFirst} (105°)2`);
        expect(screen.getByTestId('plan-preview-pass-cross')).toHaveTextContent(`${en.planPreview.swathsCross} (15°)2`);
        expect(screen.queryByTestId('plan-preview-swaths')).toBeNull();
    });
});

describe('PlanPreviewCard', () => {
    it('shows counts, length and boundary inset', () => {
        render(<PlanPreviewCard summary={parsePlanPreview(okMsg)!} colors={colors}
                                onClear={vi.fn()} onDismiss={vi.fn()}/>);
        expect(screen.getByText('Plan preview: Front')).toBeInTheDocument();
        expect(screen.getByTestId('plan-preview-rings')).toHaveTextContent('2');
        expect(screen.getByTestId('plan-preview-swaths')).toHaveTextContent('14');
        expect(screen.getByTestId('plan-preview-length')).toHaveTextContent('123.4 m');
        expect(screen.getByTestId('plan-preview-inset')).toHaveTextContent('0.29 m');
        expect(screen.getByText(en.planPreview.inset)).toBeInTheDocument();
    });

    it('clear and hide call back', async () => {
        const onClear = vi.fn().mockResolvedValue(undefined);
        const onDismiss = vi.fn();
        render(<PlanPreviewCard summary={parsePlanPreview(okMsg)!} colors={colors}
                                onClear={onClear} onDismiss={onDismiss}/>);
        await userEvent.click(screen.getByText(en.planPreview.clear));
        expect(onClear).toHaveBeenCalled();
        await userEvent.click(screen.getByLabelText(en.planPreview.hide));
        expect(onDismiss).toHaveBeenCalled();
    });

    it('shows planning and failure states', () => {
        const planning = parsePlanPreview({data: JSON.stringify({id: 4, status: 'planning', area: -1})})!;
        const {rerender} = render(<PlanPreviewCard summary={planning} colors={colors}
                                                   onClear={vi.fn()} onDismiss={vi.fn()}/>);
        expect(screen.getByText(en.planPreview.titleAll)).toBeInTheDocument();
        expect(screen.getByText(en.planPreview.planning)).toBeInTheDocument();
        expect(screen.queryByText(en.planPreview.clear)).not.toBeInTheDocument();
        const failed = parsePlanPreview({data: JSON.stringify({id: 4, status: 'failed', area: -1,
            message: 'refused: busy'})})!;
        rerender(<PlanPreviewCard summary={failed} colors={colors} onClear={vi.fn()} onDismiss={vi.fn()}/>);
        expect(screen.getByText(/refused: busy/)).toBeInTheDocument();
    });
});
