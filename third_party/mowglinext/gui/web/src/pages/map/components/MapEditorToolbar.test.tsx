import {describe, expect, it, vi} from 'vitest';
import {render, screen} from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import {MapEditorToolbar} from './MapEditorToolbar.tsx';
import en from '../../../i18n/locales/en.json';

vi.mock('../../../theme/ThemeContext.tsx', () => ({
    useThemeMode: () => ({colors: {text: '#fff', muted: '#888', danger: '#f00', success: '#0f0', bgElevated: '#222'}}),
}));
vi.mock('./ShapePickerDropdown.tsx', () => ({ShapePickerDropdown: ({children}: {children: React.ReactNode}) => children}));

const base = {
    hasUnsavedChanges: false, historyIndex: 0, editHistoryLength: 1, selectedFeatureCount: 0,
    onSaveMap: vi.fn().mockResolvedValue(undefined), onCancel: vi.fn(), onUndo: vi.fn(), onRedo: vi.fn(),
};

describe('MapEditorToolbar boundary line tool', () => {
    it('starts the tool, and shows the active tooltip label while drawing', async () => {
        const onDrawBoundaryLine = vi.fn();
        const {rerender} = render(<MapEditorToolbar {...base} onDrawBoundaryLine={onDrawBoundaryLine}/>);
        await userEvent.click(screen.getByRole('button', {name: en.mapBoundary.tool}));
        expect(onDrawBoundaryLine).toHaveBeenCalledOnce();
        rerender(<MapEditorToolbar {...base} onDrawBoundaryLine={onDrawBoundaryLine} boundaryDrawing/>);
        expect(screen.getByTestId('tool-boundary-line')).toHaveAttribute('aria-label', en.mapBoundary.toolActive);
    });
});
