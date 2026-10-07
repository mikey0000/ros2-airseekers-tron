import React, {useEffect, useState} from "react";
import {Alert, App, Button, Popover, Space} from "antd";
import {PlayCircleOutlined, QuestionCircleOutlined, RedoOutlined, StepForwardOutlined, StopOutlined} from "@ant-design/icons";
import {DockIcon} from "./DockIcon.tsx";
import {BladeHeightControl} from "./BladeHeightControl.tsx";
import {useTranslation} from "react-i18next";
import {useMowerAction} from "./MowerActions.tsx";
import {useStatus} from "../hooks/useStatus.ts";
import {useCoverageResumeAvailable} from "../hooks/useCoverageResumeAvailable.ts";
import {
    canHome, canReset, canResume, canStop, CMD_HOME, CMD_START, CMD_RESET_EMERGENCY, CMD_STOP, isLatchedFault, isMissionActive,
    isNotice, MISSION_PHASES, stopNeedsConfirm,
} from "../utils/missionStates.ts";

// Phases of a mow (incl. paused / planning) where the live blade height is offered.
const BLADE_HEIGHT_PHASES = new Set<string>([...MISSION_PHASES, "MANUAL_MOWING"]);

interface Props {
    state?: number;
    stateName?: string;
    subStateName?: string;
    /** Opens the Start sheet (or sends START on robots without one). */
    onStart: () => void;
    /** "Start fresh" (discard the resume cursor). Defaults to clear_resume + START. */
    onStartFresh?: () => void;
    style?: React.CSSProperties;
}

/**
 * Always-available "Stop mowing" (high_level_control STOP=8), a fault banner
 * with Reset (254) for latched faults, and a post-stop follow-up offering
 * Return to dock / Start-or-Resume. State gating lives in utils/missionStates.
 */
export const MissionStopControls: React.FC<Props> = ({state, stateName, subStateName, onStart, onStartFresh, style}) => {
    const {t} = useTranslation();
    const {modal, notification} = App.useApp();
    const mowerAction = useMowerAction();
    const status = useStatus();
    const resumeAvailable = useCoverageResumeAvailable();
    const [busy, setBusy] = useState<string | null>(null);
    const [stopped, setStopped] = useState(false);
    // STUCK_NEEDS_HELP: the owner moves the robot by hand first; Resume appears only
    // once Reset was pressed in this latch (it clears the stuck guard history).
    const [stuckReset, setStuckReset] = useState(false);

    const active = isMissionActive(state, stateName);
    const fault = isLatchedFault(stateName);
    const showStop = canStop(state, stateName);

    // A new mission (or a fault) supersedes the post-stop follow-up.
    useEffect(() => {
        if (active || fault) setStopped(false);
    }, [active, fault]);
    useEffect(() => {
        if (stateName !== "STUCK_NEEDS_HELP") setStuckReset(false);
    }, [stateName]);

    const run = (key: string, command: number, after?: () => void) => async () => {
        setBusy(key);
        try {
            await mowerAction("high_level_control", {Command: command})();
            after?.();
        } catch (e: unknown) {
            notification.error({
                message: t('missionStop.failed'),
                description: e instanceof Error ? e.message : undefined,
            });
        } finally {
            setBusy(null);
        }
    };

    const resumable = canResume(state, stateName, resumeAvailable) &&
        (stateName !== "STUCK_NEEDS_HELP" || stuckReset);
    // Resume = START with a cursor present: the mission continues the same area at
    // the saved sub-path / pose (mission_fsm._start resume path), no Start sheet.
    const resumeButton = resumable ? (
        <Button type="primary" size="large" icon={<StepForwardOutlined/>} loading={busy === "resume"}
                onClick={run("resume", CMD_START, () => setStopped(false))} data-testid="mission-resume">
            {t('missionStop.resumeMowing')}
        </Button>
    ) : null;
    const startFresh = async () => {
        setStopped(false);
        if (onStartFresh) { onStartFresh(); return; }
        setBusy("fresh");
        try {
            await mowerAction("coverage_clear_resume", {})();
            await mowerAction("high_level_control", {Command: CMD_START})();
        } catch (e: unknown) {
            notification.error({message: t('missionStop.failed'), description: e instanceof Error ? e.message : undefined});
        } finally {
            setBusy(null);
        }
    };
    const startFreshButton = resumable ? (
        <Button size="small" type="link" icon={<PlayCircleOutlined/>} loading={busy === "fresh"}
                onClick={() => void startFresh()} data-testid="mission-start-fresh">
            {t('missionStop.startFresh')}
        </Button>
    ) : null;
    const whyStopped = (
        <Popover content={subStateName || t('missionStop.faultNoReason')} trigger="click">
            <Button size="small" type="link" icon={<QuestionCircleOutlined/>} data-testid="mission-why-stopped">
                {t('missionStop.whyStopped')}
            </Button>
        </Popover>
    );

    const sendStop = run("stop", CMD_STOP, () => setStopped(true));
    const onStopClick = () => {
        if (stopNeedsConfirm(stateName, status.mow_enabled)) {
            modal.confirm({
                title: t('missionStop.confirmTitle'),
                content: t('missionStop.confirmBody'),
                okText: t('missionStop.stop'),
                okType: "danger",
                cancelText: t('missionStop.cancel'),
                onOk: () => sendStop(),
            });
            return;
        }
        void sendStop();
    };

    // "Stop mowing" only while the blade may be cutting; a transit, return to
    // dock, undock or recording is just "Stop".
    const mowingNow = stateName === "MOWING" || stateName === "MANUAL_MOWING";
    const stopButton = showStop ? (
        <Button danger type="primary" size="large" icon={<StopOutlined/>}
                loading={busy === "stop"} onClick={onStopClick} data-testid="mission-stop">
            {mowingNow ? t('missionStop.stop') : t('missionStop.stopPlain')}
        </Button>
    ) : null;

    if (fault) {
        return (
            <Alert
                style={style}
                type="error"
                showIcon
                data-testid="mission-fault"
                message={t('missionStop.faultTitle', {state: stateName})}
                description={<>
                    {subStateName || t('missionStop.faultNoReason')}
                    {stateName === "STUCK_NEEDS_HELP" && resumeAvailable && !stuckReset && (
                        <div data-testid="mission-stuck-hint">{t('missionStop.stuckResetFirst')}</div>
                    )}
                </>}
                action={
                    <Space direction="vertical">
                        {stopButton}
                        {canReset(stateName) && (
                            <Button icon={<RedoOutlined/>} loading={busy === "reset"}
                                    onClick={run("reset", CMD_RESET_EMERGENCY,
                                        () => { if (stateName === "STUCK_NEEDS_HELP") setStuckReset(true); })}
                                    data-testid="mission-reset">
                                {t('missionStop.reset')}
                            </Button>
                        )}
                        {resumeButton}
                        {startFreshButton}
                    </Space>
                }
            />
        );
    }

    if (isNotice(stateName)) {
        // MOWING_INCOMPLETE: stopped in place, sub_state says what was not mowed.
        return (
            <Alert
                style={style}
                type="warning"
                showIcon
                data-testid="mission-notice"
                message={t('missionStop.incompleteTitle')}
                description={subStateName || t('missionStop.faultNoReason')}
                action={
                    <Space direction="vertical">
                        <Button icon={<DockIcon/>} loading={busy === "home"}
                                onClick={run("home", CMD_HOME)}>
                            {t('missionStop.returnToDock')}
                        </Button>
                        {resumeButton ?? (
                            <Button type="primary" icon={<PlayCircleOutlined/>} onClick={onStart}>
                                {t('missionStop.start')}
                            </Button>
                        )}
                        {startFreshButton}
                    </Space>
                }
            />
        );
    }

    if (showStop) {
        const bladeHeight = !!stateName && BLADE_HEIGHT_PHASES.has(stateName);
        return (
            <div style={{display: "flex", flexDirection: "column", alignItems: "center", gap: 8, ...style}}>
                {stopButton}
                {bladeHeight && <BladeHeightControl/>}
            </div>
        );
    }

    if (stopped || resumable) {
        return (
            <Alert
                style={style}
                type="info"
                showIcon
                closable={!resumable}
                onClose={() => setStopped(false)}
                data-testid="mission-stopped"
                message={resumable && !stopped ? t('missionStop.interruptedTitle') : t('missionStop.stoppedTitle')}
                description={resumable ? <>{t('missionStop.interruptedHint')} {whyStopped}</> : undefined}
                action={
                    <Space direction="vertical">
                        {resumeButton}
                        {canHome(state, stateName) && (
                            <Button icon={<DockIcon/>} loading={busy === "home"}
                                    onClick={run("home", CMD_HOME, () => setStopped(false))}>
                                {t('missionStop.returnToDock')}
                            </Button>
                        )}
                        {resumable ? startFreshButton : (
                            <Button type="primary" icon={<PlayCircleOutlined/>}
                                    onClick={() => { setStopped(false); onStart(); }}>
                                {t('missionStop.start')}
                            </Button>
                        )}
                    </Space>
                }
            />
        );
    }
    return null;
};
