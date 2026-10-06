import {describe, it, expect, vi} from 'vitest';
import {render, screen, fireEvent} from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import {DockHeadingPanel, compassPoint, headingDegToRad, headingRadToDeg} from './DockHeadingPanel.tsx';
import en from "../../../i18n/locales/en.json";
import fr from "../../../i18n/locales/fr.json";

describe('DockHeadingPanel', () => {
    it('converts ENU headings and names the compass point', () => {
        expect(headingRadToDeg(Math.PI / 2)).toBe(90);
        expect(headingRadToDeg(-Math.PI / 2)).toBe(270);
        expect(headingDegToRad(270)).toBeCloseTo(-Math.PI / 2);
        expect(compassPoint(0)).toBe('E');
        expect(compassPoint(90)).toBe('N');
        expect(compassPoint(350)).toBe('E');
        expect(compassPoint(225)).toBe('SW');
    });

    it('desktop: typing a heading rotates the marker and Apply stores it', async () => {
        const user = userEvent.setup();
        const onChange = vi.fn();
        const onApply = vi.fn().mockResolvedValue(undefined);
        render(<DockHeadingPanel heading={0} onChange={onChange} onApply={onApply}/>);
        const input = screen.getByLabelText(en.dockHeading.label);
        expect(input).toHaveValue('0.0');
        fireEvent.change(input, {target: {value: '90'}});
        expect(onChange).toHaveBeenLastCalledWith(expect.closeTo(Math.PI / 2, 6));
        await user.click(screen.getByRole('button', {name: en.dockHeading.rotateRight}));
        expect(onChange).toHaveBeenLastCalledWith(expect.closeTo(85 * Math.PI / 180, 6));
        await user.click(screen.getByRole('button', {name: en.dockHeading.apply}));
        expect(onApply).toHaveBeenCalledOnce();
    });

    it('mobile: collapsed button expands to the heading field', async () => {
        const user = userEvent.setup();
        render(<DockHeadingPanel heading={Math.PI} onChange={vi.fn()} onApply={vi.fn()} mobile/>);
        expect(screen.queryByLabelText(en.dockHeading.label)).toBeNull();
        await user.click(screen.getByRole('button', {name: en.dockHeading.title}));
        expect(screen.getByLabelText(en.dockHeading.label)).toHaveValue('180.0');
        expect(screen.getByRole('button', {name: en.dockHeading.apply})).toBeInTheDocument();
    });

    it('has French strings for every key', () => {
        expect(Object.keys(fr.dockHeading).sort()).toEqual(Object.keys(en.dockHeading).sort());
        expect(fr.mapToolbarMobile.drawPathToDock).toBeTruthy();
        expect(fr.mapToolbarMobile.editPath).toBeTruthy();
    });
});
