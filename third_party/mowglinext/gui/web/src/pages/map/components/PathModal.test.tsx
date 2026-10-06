import {describe, it, expect, vi} from 'vitest';
import {render, screen} from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import {PathModal} from './PathModal.tsx';
import en from "../../../i18n/locales/en.json";

describe('PathModal', () => {
    const props = () => ({
        open: true,
        name: 'Path 1',
        width: 0.7,
        start: 'area' as const,
        end: 'dock' as const,
        extendToDock: false,
        onNameChange: vi.fn(),
        onWidthChange: vi.fn(),
        onExtendToDockChange: vi.fn(),
        onSave: vi.fn(),
        onCancel: vi.fn(),
        onDelete: vi.fn(),
    });

    it('shows the name, width, end connections and the dock option', () => {
        render(<PathModal {...props()}/>);
        expect(screen.getByText(en.mapPath.title)).toBeInTheDocument();
        expect(screen.getByLabelText(en.mapPath.name)).toHaveValue('Path 1');
        expect(screen.getByText('Width: 0.7 m')).toBeInTheDocument();
        expect(screen.getByText(`${en.mapPath.endLabel}: ${en.mapPath.end.dock}`)).toBeInTheDocument();
        expect(screen.getByText(`${en.mapPath.startLabel}: ${en.mapPath.end.area}`)).toBeInTheDocument();
        expect(screen.getByRole('switch', {name: en.mapPath.extendToDock})).not.toBeChecked();
        expect(screen.queryByText(en.mapPath.notConnected)).not.toBeInTheDocument();
        expect(screen.queryByRole('button', {name: en.mapPath.delete})).not.toBeInTheDocument();
    });

    it('renders nothing when closed', () => {
        render(<PathModal {...props()} open={false}/>);
        expect(screen.queryByText(en.mapPath.title)).not.toBeInTheDocument();
    });

    it('hints when an end is not connected and hides the dock option without the dock', () => {
        render(<PathModal {...props()} end="none"/>);
        expect(screen.getByText(en.mapPath.notConnected)).toBeInTheDocument();
        expect(screen.queryByRole('switch')).not.toBeInTheDocument();
    });

    it('reports name edits and the dock toggle', async () => {
        const p = props();
        render(<PathModal {...p}/>);
        await userEvent.type(screen.getByLabelText(en.mapPath.name), 'x');
        expect(p.onNameChange).toHaveBeenLastCalledWith('Path 1x');
        await userEvent.click(screen.getByRole('switch', {name: en.mapPath.extendToDock}));
        expect(p.onExtendToDockChange).toHaveBeenCalledWith(true, expect.anything());
    });

    it('saves via the button and Enter, cancels via Cancel', async () => {
        const p = props();
        render(<PathModal {...p}/>);
        await userEvent.click(screen.getByRole('button', {name: en.mapPath.save}));
        expect(p.onSave).toHaveBeenCalledTimes(1);
        await userEvent.type(screen.getByLabelText(en.mapPath.name), '{enter}');
        expect(p.onSave).toHaveBeenCalledTimes(2);
        await userEvent.click(screen.getByRole('button', {name: en.mapPath.cancel}));
        expect(p.onCancel).toHaveBeenCalled();
    });

    it('offers Delete when editing a saved path', async () => {
        const p = props();
        render(<PathModal {...p} editing/>);
        expect(screen.getByText(en.mapPath.editTitle)).toBeInTheDocument();
        await userEvent.click(screen.getByRole('button', {name: en.mapPath.delete}));
        expect(p.onDelete).toHaveBeenCalled();
    });

    it('blocks saving a degenerate path', async () => {
        const p = props();
        render(<PathModal {...p} invalid/>);
        expect(screen.getByText(en.mapPath.invalid)).toBeInTheDocument();
        expect(screen.getByRole('button', {name: en.mapPath.save})).toBeDisabled();
        await userEvent.type(screen.getByLabelText(en.mapPath.name), '{enter}');
        expect(p.onSave).not.toHaveBeenCalled();
    });

    it('renders as a bottom sheet on a phone, without stealing focus', () => {
        render(<PathModal {...props()} mobile/>);
        const sheet = screen.getByRole('dialog', {name: en.mapPath.title});
        expect(sheet).toHaveAttribute('data-layout', 'sheet');
        expect(sheet).toHaveStyle({position: 'fixed', bottom: '0px'});
        expect(screen.getByLabelText(en.mapPath.name)).not.toHaveFocus();
        expect(screen.getByRole('button', {name: en.mapPath.save})).toBeInTheDocument();
    });

    it('stays a side panel on desktop', () => {
        render(<PathModal {...props()}/>);
        expect(screen.getByRole('dialog', {name: en.mapPath.title})).toHaveAttribute('data-layout', 'panel');
    });
});
