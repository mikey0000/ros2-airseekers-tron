import {describe, it, expect, vi} from 'vitest';
import {render, screen} from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import {PathModal} from './PathModal.tsx';
import en from "../../../i18n/locales/en.json";

describe('PathModal', () => {
    const props = () => ({
        open: true,
        name: 'Path 1',
        width: 0.8,
        dockAvailable: true,
        snapToDock: true,
        onNameChange: vi.fn(),
        onWidthChange: vi.fn(),
        onSnapToDockChange: vi.fn(),
        onSave: vi.fn(),
        onCancel: vi.fn(),
    });

    it('shows the name, width and dock snap controls', () => {
        render(<PathModal {...props()}/>);
        expect(screen.getByText(en.mapPath.title)).toBeInTheDocument();
        expect(screen.getByLabelText(en.mapPath.name)).toHaveValue('Path 1');
        expect(screen.getByText('Width: 0.8 m')).toBeInTheDocument();
        expect(screen.getByRole('switch', {name: en.mapPath.snapToDock})).toBeChecked();
    });

    it('hides the dock snap toggle when no dock pose is known', () => {
        render(<PathModal {...props()} dockAvailable={false}/>);
        expect(screen.queryByRole('switch')).not.toBeInTheDocument();
    });

    it('reports name edits and snap toggles', async () => {
        const p = props();
        render(<PathModal {...p}/>);
        await userEvent.type(screen.getByLabelText(en.mapPath.name), 'x');
        expect(p.onNameChange).toHaveBeenLastCalledWith('Path 1x');
        await userEvent.click(screen.getByRole('switch', {name: en.mapPath.snapToDock}));
        expect(p.onSnapToDockChange).toHaveBeenCalledWith(false, expect.anything());
    });

    it('saves via the OK button and Enter, cancels via Cancel', async () => {
        const p = props();
        render(<PathModal {...p}/>);
        await userEvent.click(screen.getByRole('button', {name: en.mapPath.save}));
        expect(p.onSave).toHaveBeenCalledTimes(1);
        await userEvent.type(screen.getByLabelText(en.mapPath.name), '{enter}');
        expect(p.onSave).toHaveBeenCalledTimes(2);
        await userEvent.click(screen.getByRole('button', {name: en.mapPath.cancel}));
        expect(p.onCancel).toHaveBeenCalled();
    });

    it('blocks saving a degenerate path', async () => {
        const p = props();
        render(<PathModal {...p} invalid/>);
        expect(screen.getByText(en.mapPath.invalid)).toBeInTheDocument();
        expect(screen.getByRole('button', {name: en.mapPath.save})).toBeDisabled();
        await userEvent.type(screen.getByLabelText(en.mapPath.name), '{enter}');
        expect(p.onSave).not.toHaveBeenCalled();
    });
});
