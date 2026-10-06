import React, {useEffect, useState} from "react";
import {Alert, App, Button, Space} from "antd";
import {PlayCircleOutlined, RedoOutlined, StopOutlined} from "@ant-design/icons";
import {DockIcon} from "./DockIcon.tsx";
import {useTranslation} from "react-i18next";
import {useMowerAction} from "./MowerActions.tsx";
import {useStatus} from "../hooks/useStatus.ts";
import {useCoverageResumeAvailable} from "../hooks/useCoverageResumeAvailable.ts";
import {
    canReset, canStop, CMD_HOME, CMD_RESET_EMERGENCY, CMD_STOP, isLatchedFault, isMissionActive,
    isNotice, stopNeedsConfirm,
} from "../utils/missionStates.ts";

interface Props {
    state?: number;
    stateName?: string;
    subStateName?: string;
    /** Opens the Start sheet (or sends START on robots without one). */
    onStart: () => void;
    style?: React.CSSProperties;
}

/**
 * Always-available "Stop mowing" (high_level_control STOP=8), a fault banner
 * with Reset (254) for latched faults, and a post-stop follow-up offering
 * Return to dock / Start-or-Resume. State gating lives in utils/missionStates.
 */
export const MissionStopControls: React.FC<Props> = ({state, stateName, subStateName, onStart, style}) => {
    const {t} = useTranslation();
    const {modal, notification} = App.useApp();
    const mowerAction = useMowerAction();
    const status = useStatus();
    const resumeAvailable = useCoverageResumeAvailable();
    const [busy, setBusy] = useState<string | null>(null);
    const [stopped, setStopped] = useState(false);

    const active = isMissionActive(state, stateName);
    const fault = isLatchedFault(stateName);
    const showStop = canStop(state, stateName);

    // A new mission (or a fault) supersedes the post-stop follow-up.
    useEffect(() => {
        if (active || fault) setStopped(false);
    }, [active, fault]);

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
                description={subStateName || t('missionStop.faultNoReason')}
                action={
                    <Space direction="vertical">
                        {stopButton}
                        {canReset(stateName) && (
                            <Button icon={<RedoOutlined/>} loading={busy === "reset"}
                                    onClick={run("reset", CMD_RESET_EMERGENCY)} data-testid="mission-reset">
                                {t('missionStop.reset')}
                            </Button>
                        )}
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
                        <Button type="primary" icon={<PlayCircleOutlined/>} onClick={onStart}>
                            {resumeAvailable ? t('missionStop.resumeOrFresh') : t('missionStop.start')}
                        </Button>
                    </Space>
                }
            />
        );
    }

    if (showStop) {
        return <div style={{display: "flex", justifyContent: "center", ...style}}>{stopButton}</div>;
    }

    if (stopped) {
        return (
            <Alert
                style={style}
                type="info"
                showIcon
                closable
                onClose={() => setStopped(false)}
                message={t('missionStop.stoppedTitle')}
                description={resumeAvailable ? t('missionStop.stoppedResumeHint') : undefined}
                action={
                    <Space direction="vertical">
                        <Button icon={<DockIcon/>} loading={busy === "home"}
                                onClick={run("home", CMD_HOME, () => setStopped(false))}>
                            {t('missionStop.returnToDock')}
                        </Button>
                        <Button type="primary" icon={<PlayCircleOutlined/>}
                                onClick={() => { setStopped(false); onStart(); }}>
                            {resumeAvailable ? t('missionStop.resumeOrFresh') : t('missionStop.start')}
                        </Button>
                    </Space>
                }
            />
        );
    }
    return null;
};
