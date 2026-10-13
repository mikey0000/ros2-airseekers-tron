import {describe, it, expect, vi, beforeEach} from 'vitest';
import {render, screen} from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import {App as AntApp} from 'antd';
import en from "../i18n/locales/en.json";

const callCreate = vi.fn();
let resumeAvailable = true;
vi.mock('./MowerActions.tsx', () => ({
    useMowerAction: () => (command: string, args: Record<string, unknown> = {}) => async () => {
        callCreate(command, args);
    },
}));
vi.mock('../hooks/useStatus.ts', () => ({useStatus: () => ({mow_enabled: false})}));
vi.mock('../hooks/useCoverageResumeAvailable.ts', () => ({useCoverageResumeAvailable: () => resumeAvailable}));
let isMobile = false;
vi.mock('../hooks/useIsMobile.ts', () => ({useIsMobile: () => isMobile}));

vi.mock('./BladeHeightControl.tsx', () => ({BladeHeightControl: () => <div data-testid="blade-height"/>}));

import {MissionStopControls} from './MissionStopControls.tsx';

const renderAt = (stateName: string, state = 1, onStart = vi.fn()) =>
    render(<AntApp><MissionStopControls state={state} stateName={stateName} subStateName="why" onStart={onStart}/></AntApp>);

describe('MissionStopControls Resume', () => {
    beforeEach(() => { callCreate.mockClear(); resumeAvailable = true; });

    it.each(['IDLE', 'IDLE_DOCKED', 'MOWING_INCOMPLETE', 'NAV_TO_DOCK_FAILED'])(
        'shows Resume at rest in %s and sends START (no sheet)', async (name) => {
            const onStart = vi.fn();
            renderAt(name, 1, onStart);
            await userEvent.click(screen.getByTestId('mission-resume'));
            expect(callCreate).toHaveBeenCalledWith('high_level_control', {Command: 1});
            expect(onStart).not.toHaveBeenCalled();
            expect(screen.getByText(en.missionStop.resumeMowing)).toBeInTheDocument();
            expect(screen.getByTestId('mission-start-fresh')).toBeInTheDocument();
        });

    it.each(['MOWING', 'TRANSIT', 'RETURNING_HOME', 'EMERGENCY', 'BOUNDARY_EMERGENCY_STOP'])(
        'hides Resume in %s', (name) => {
            renderAt(name, name.startsWith('MOW') || name === 'TRANSIT' || name === 'RETURNING_HOME' ? 2 : 1);
            expect(screen.queryByTestId('mission-resume')).toBeNull();
        });

    it('hides Resume without a resume cursor', () => {
        resumeAvailable = false;
        renderAt('IDLE');
        expect(screen.queryByTestId('mission-resume')).toBeNull();
        resumeAvailable = false;
        renderAt('MOWING_INCOMPLETE');
        expect(screen.queryByTestId('mission-resume')).toBeNull();
    });

    it('STUCK_NEEDS_HELP: Resume only after Reset', async () => {
        renderAt('STUCK_NEEDS_HELP');
        expect(screen.queryByTestId('mission-resume')).toBeNull();
        expect(screen.getByTestId('mission-stuck-hint')).toBeInTheDocument();
        await userEvent.click(screen.getByTestId('mission-reset'));
        expect(callCreate).toHaveBeenCalledWith('high_level_control', {Command: 254});
        await userEvent.click(await screen.findByTestId('mission-resume'));
        expect(callCreate).toHaveBeenLastCalledWith('high_level_control', {Command: 1});
    });

    it('fault banner shows Resume inline next to Reset for NAV_TO_DOCK_FAILED', () => {
        renderAt('NAV_TO_DOCK_FAILED');
        expect(screen.getByTestId('mission-reset')).toBeInTheDocument();
        expect(screen.getByTestId('mission-resume')).toBeInTheDocument();
    });

    it('Start fresh clears the cursor then sends START', async () => {
        renderAt('IDLE');
        await userEvent.click(screen.getByTestId('mission-start-fresh'));
        expect(callCreate.mock.calls).toEqual([
            ['coverage_clear_resume', {}], ['high_level_control', {Command: 1}]]);
    });

    it('idle with cursor offers Why stopped?', () => {
        renderAt('IDLE');
        expect(screen.getByTestId('mission-why-stopped')).toBeInTheDocument();
    });
});

describe('MissionStopControls blade height', () => {
    it('offers the live blade height while mowing only', () => {
        const {unmount} = renderAt('MOWING', 2);
        expect(screen.getByTestId('blade-height')).toBeInTheDocument();
        unmount();
        renderAt('RETURNING_HOME', 2);
        expect(screen.queryByTestId('blade-height')).toBeNull();
    });
});

describe('MissionStopControls narrow layout', () => {
    beforeEach(() => { resumeAvailable = true; });

    it('desktop keeps the buttons in the side action column', () => {
        isMobile = false;
        const {container} = renderAt('IDLE');
        expect(screen.queryByTestId('mission-alert-stacked')).toBeNull();
        expect(container.querySelector('.ant-alert-action')).toContainElement(screen.getByTestId('mission-resume'));
    });

    it('phone stacks the buttons below the text (no side action column)', () => {
        isMobile = true;
        const {container} = renderAt('IDLE');
        const stacked = screen.getByTestId('mission-alert-stacked');
        expect(container.querySelector('.ant-alert-action')).toBeNull();
        expect(stacked).toHaveTextContent(en.missionStop.interruptedHint);
        expect(stacked).toContainElement(screen.getByTestId('mission-resume'));
        expect(stacked).toContainElement(screen.getByTestId('mission-start-fresh'));
        isMobile = false;
    });
});

describe('MissionStopControls on the map (one banner)', () => {
    beforeEach(() => { resumeAvailable = true; isMobile = false; });
    const renderMap = (stateName: string, props: {compact?: boolean; showFollowUp?: boolean}, state = 1) =>
        render(<AntApp><MissionStopControls state={state} stateName={stateName} subStateName="why" onStart={vi.fn()} {...props}/></AntApp>);

    it('compact: interrupted follow-up is a slim row, not a card, and keeps Resume', () => {
        const {container} = renderMap('IDLE', {compact: true});
        expect(container.querySelector('.ant-alert')).toBeNull();
        expect(screen.getByTestId('mission-stopped')).toHaveTextContent(en.missionStop.interruptedTitle);
        expect(screen.getByTestId('mission-resume')).toBeInTheDocument();
        expect(screen.getByTestId('mission-start-fresh')).toBeInTheDocument();
        // "Why stopped?" comes from the status line around it, not repeated here.
        expect(screen.queryByTestId('mission-why-stopped')).toBeNull();
    });

    it('showFollowUp=false (edit mode) hides the interrupted card', () => {
        renderMap('IDLE', {showFollowUp: false});
        expect(screen.queryByTestId('mission-stopped')).toBeNull();
        expect(screen.queryByTestId('mission-resume')).toBeNull();
    });

    it('showFollowUp=false still shows fault banners and Stop', () => {
        renderMap('NAV_TO_DOCK_FAILED', {showFollowUp: false});
        expect(screen.getByTestId('mission-fault')).toBeInTheDocument();
    });
});
