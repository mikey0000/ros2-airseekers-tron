import {describe, expect, it, vi} from 'vitest';
import {render, screen} from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import {BoundaryLineSheet, type BoundaryLineSheetProps} from './BoundaryLineSheet.tsx';
import {BoundaryLineDrawPanel} from './BoundaryLineDrawPanel.tsx';
import en from '../../../i18n/locales/en.json';

const props = (over: Partial<BoundaryLineSheetProps> = {}): BoundaryLineSheetProps => ({
    open: true,
    lineKey: 'boundary-bl-1',
    name: '',
    fallbackName: 'Boundary line 1',
    widthM: 0,
    onApply: vi.fn(),
    onDelete: vi.fn(),
    onClose: vi.fn(),
    ...over,
});

describe('BoundaryLineSheet', () => {
    it('shows the fallback name for an unnamed line; Apply is disabled until something changes', () => {
        render(<BoundaryLineSheet {...props()}/>);
        expect(screen.getByText(en.mapBoundary.sheetTitle)).toBeInTheDocument();
        expect(screen.getByLabelText(en.mapBoundary.name)).toHaveAttribute('placeholder', 'Boundary line 1');
        expect(screen.getByTestId('boundary-apply')).toBeDisabled();
    });

    it('applies the edited name and width', async () => {
        const p = props();
        render(<BoundaryLineSheet {...p}/>);
        await userEvent.type(screen.getByLabelText(en.mapBoundary.name), '  Pond  ');
        const width = screen.getByLabelText(en.mapBoundary.widthLabel);
        await userEvent.clear(width);
        await userEvent.type(width, '0.35');
        await userEvent.click(screen.getByTestId('boundary-apply'));
        expect(p.onApply).toHaveBeenCalledWith('Pond', 0.35);
    });

    it('clamps the width to 2 m', async () => {
        const p = props({name: 'Wall'});
        render(<BoundaryLineSheet {...p}/>);
        const width = screen.getByLabelText(en.mapBoundary.widthLabel);
        await userEvent.clear(width);
        await userEvent.type(width, '5');
        await userEvent.click(screen.getByTestId('boundary-apply'));
        expect(p.onApply).toHaveBeenCalledWith('Wall', 2);
    });

    it('deletes only after the confirm', async () => {
        const p = props();
        render(<BoundaryLineSheet {...p}/>);
        await userEvent.click(screen.getByTestId('boundary-delete'));
        expect(p.onDelete).not.toHaveBeenCalled();
        await userEvent.click(await screen.findByRole('button', {name: en.mapBoundary.deleteConfirmOk}));
        expect(p.onDelete).toHaveBeenCalledTimes(1);
    });
});

describe('BoundaryLineDrawPanel', () => {
    it('enables Undo with a point and Finish with two', async () => {
        const handlers = {onUndo: vi.fn(), onFinish: vi.fn(), onCancel: vi.fn()};
        const {rerender} = render(<BoundaryLineDrawPanel open pointCount={0} canFinish={false} {...handlers}/>);
        expect(screen.getByTestId('boundary-undo')).toBeDisabled();
        expect(screen.getByTestId('boundary-finish')).toBeDisabled();
        rerender(<BoundaryLineDrawPanel open pointCount={2} canFinish {...handlers}/>);
        await userEvent.click(screen.getByTestId('boundary-undo'));
        await userEvent.click(screen.getByTestId('boundary-finish'));
        await userEvent.click(screen.getByTestId('boundary-cancel'));
        expect(handlers.onUndo).toHaveBeenCalledTimes(1);
        expect(handlers.onFinish).toHaveBeenCalledTimes(1);
        expect(handlers.onCancel).toHaveBeenCalledTimes(1);
        expect(screen.getByTestId('boundary-point-count')).toHaveTextContent('Points: 2');
    });

    it('renders nothing when closed', () => {
        render(<BoundaryLineDrawPanel open={false} pointCount={0} canFinish={false}
                                      onUndo={vi.fn()} onFinish={vi.fn()} onCancel={vi.fn()}/>);
        expect(screen.queryByTestId('boundary-draw-panel')).not.toBeInTheDocument();
    });
});
