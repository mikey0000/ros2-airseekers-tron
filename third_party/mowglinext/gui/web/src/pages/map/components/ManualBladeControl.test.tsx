import {describe, it, expect, vi, beforeEach, afterEach} from 'vitest';
import {render, screen, fireEvent, act} from '@testing-library/react';
import {ManualBladeControl, BLADE_HOLD_MS} from './ManualBladeControl.tsx';
import en from "../../../i18n/locales/en.json";

const status = vi.hoisted(() => ({rpm: 0}));
vi.mock('../../../hooks/useStatus.ts', () => ({useStatus: () => ({mower_motor_rpm: status.rpm})}));

describe('ManualBladeControl', () => {
    beforeEach(() => {
        vi.useFakeTimers();
        status.rpm = 0;
    });
    afterEach(() => vi.useRealTimers());

    it('starts the blades only after a full hold', async () => {
        const onStart = vi.fn().mockResolvedValue(undefined);
        render(<ManualBladeControl bladeOn={false} canStart={true} onStart={onStart} onStop={vi.fn()}/>);
        const btn = screen.getByRole('button', {name: en.manualBlade.startHint});
        fireEvent.pointerDown(btn);
        act(() => vi.advanceTimersByTime(BLADE_HOLD_MS - 100));
        fireEvent.pointerUp(btn);
        act(() => vi.advanceTimersByTime(500));
        expect(onStart).not.toHaveBeenCalled();

        fireEvent.pointerDown(btn);
        await act(async () => { await vi.advanceTimersByTimeAsync(BLADE_HOLD_MS); });
        expect(onStart).toHaveBeenCalledTimes(1);
    });

    it('does not start when manual mode is not active', () => {
        const onStart = vi.fn().mockResolvedValue(undefined);
        render(<ManualBladeControl bladeOn={false} canStart={false} onStart={onStart} onStop={vi.fn()}/>);
        fireEvent.pointerDown(screen.getByRole('button', {name: en.manualBlade.startHint}));
        act(() => vi.advanceTimersByTime(BLADE_HOLD_MS * 2));
        expect(onStart).not.toHaveBeenCalled();
        expect(screen.getByText(en.manualBlade.off)).toBeTruthy();
    });

    it('shows a red stop button and rpm while the blades run; stop is immediate', () => {
        status.rpm = 2850;
        const onStop = vi.fn().mockResolvedValue(undefined);
        render(<ManualBladeControl bladeOn={true} canStart={false} onStart={vi.fn()} onStop={onStop}/>);
        expect(screen.getByText(`${en.manualBlade.on} · 2850 rpm`)).toBeTruthy();
        fireEvent.click(screen.getByText(en.manualBlade.stop));
        expect(onStop).toHaveBeenCalledTimes(1);
    });
});
