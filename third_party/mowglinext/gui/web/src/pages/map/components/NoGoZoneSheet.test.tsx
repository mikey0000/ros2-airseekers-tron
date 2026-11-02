import {describe, expect, it, vi} from 'vitest';
import {render, screen} from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import {NoGoZoneSheet, type NoGoZoneSheetProps} from './NoGoZoneSheet.tsx';
import {OBSTACLE_SOURCE_TRACKER, OBSTACLE_SOURCE_USER} from '../../../types/map.ts';
import en from '../../../i18n/locales/en.json';

const props = (over: Partial<NoGoZoneSheetProps> = {}): NoGoZoneSheetProps => ({
    open: true,
    title: 'Obstacle 2',
    areaM2: 12.345,
    source: OBSTACLE_SOURCE_USER,
    pending: false,
    parentName: 'Front lawn (1)',
    onClose: vi.fn(),
    onDelete: vi.fn(),
    onEditVertices: vi.fn(),
    onMowArea: vi.fn(),
    ...over,
});

describe('NoGoZoneSheet', () => {
    it('shows the title, the area in m² and the containing area, no provenance for a user zone', () => {
        render(<NoGoZoneSheet {...props()}/>);
        expect(screen.getByText(en.mapNoGo.title)).toBeInTheDocument();
        expect(screen.getByTestId('nogo-name')).toHaveTextContent('Obstacle 2');
        expect(screen.getByTestId('nogo-area')).toHaveTextContent('12.3 m²');
        expect(screen.getByText('Front lawn (1)')).toBeInTheDocument();
        expect(screen.queryByText(en.mapNoGo.sourceTracker)).not.toBeInTheDocument();
    });

    it('shows the provenance of a pending tracker proposal', () => {
        render(<NoGoZoneSheet {...props({source: OBSTACLE_SOURCE_TRACKER, pending: true})}/>);
        expect(screen.getByText(en.mapNoGo.sourceTracker)).toBeInTheDocument();
        expect(screen.getByText(en.mapNoGo.pending)).toBeInTheDocument();
    });

    it('deletes only after the confirm', async () => {
        const p = props();
        render(<NoGoZoneSheet {...p}/>);
        await userEvent.click(screen.getByTestId('nogo-delete'));
        expect(p.onDelete).not.toHaveBeenCalled();
        expect(await screen.findByText(en.mapNoGo.deleteConfirmTitle)).toBeInTheDocument();
        await userEvent.click(screen.getByRole('button', {name: en.mapNoGo.deleteConfirmOk}));
        expect(p.onDelete).toHaveBeenCalledTimes(1);
    });

    it('edits vertices and mows the containing area', async () => {
        const p = props();
        render(<NoGoZoneSheet {...p}/>);
        await userEvent.click(screen.getByTestId('nogo-edit-vertices'));
        expect(p.onEditVertices).toHaveBeenCalledTimes(1);
        await userEvent.click(screen.getByRole('button', {name: /Mow Front lawn \(1\)/}));
        expect(p.onMowArea).toHaveBeenCalledTimes(1);
    });

    it('hides the mow link without area settings', () => {
        render(<NoGoZoneSheet {...props({onMowArea: undefined})}/>);
        expect(screen.queryByTestId('nogo-mow-area')).not.toBeInTheDocument();
    });
});
