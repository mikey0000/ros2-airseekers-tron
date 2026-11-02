import React, {useEffect, useState} from "react";
import {Alert, App, Button, Popover, Space} from "antd";
import {CloseOutlined, ControlOutlined, PlayCircleOutlined, QuestionCircleOutlined, RedoOutlined, StepForwardOutlined, StopOutlined} from "@ant-design/icons";
import {DockIcon} from "./DockIcon.tsx";
import {BladeHeightControl} from "./BladeHeightControl.tsx";
import {useTranslation} from "react-i18next";
import {useMowerAction} from "./MowerActions.tsx";
import {useStatus} from "../hooks/useStatus.ts";
import {useCoverageResumeAvailable} from "../hooks/useCoverageResumeAvailable.ts";
import {useIsMobile} from "../hooks/useIsMobile.ts";
import {useStuckReset} from "../hooks/useStuckReset.ts";
import {stateRenderer} from "./utils.tsx";
import {dockFailReason, incompletePhrases, isHeadingHint, stuckReason, type Phrase} from "../utils/missionPresentation.ts";
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
    /** Map page: render the post-stop / interrupted follow-up as one slim button
     *  row (it sits inside the status line, which already has "Why stopped?")
     *  instead of a second full-width card over the map. */
    compact?: boolean;
    /** false hides the post-stop / interrupted follow-up (Map edit mode); the
     *  Stop button and fault / notice banners are always shown. */
    showFollowUp?: boolean;
    /** Opens manual drive (Map page): "Drive home manually" after a failed
     *  dock drive and "Drive manually" for the heading hint. */
    onManualDrive?: () => void;
    /** MOWING_INCOMPLETE: length of the skipped stretches of the plan (m). */
    skippedM?: number | null;
    /** MOWING_INCOMPLETE: current return_home_on_incomplete value (undefined = unknown). */
    returnHomeOnIncomplete?: boolean;
    /** Opens Settings at the return-home-on-incomplete switch. */
    onOpenReturnHomeSetting?: () => void;
}

/**
 * Always-available "Stop mowing" (high_level_control STOP=8), a fault banner
 * with Reset (254) for latched faults, and a post-stop follow-up offering
 * Return to dock / Start-or-Resume. State gating lives in utils/missionStates.
 */
export const MissionStopControls: React.FC<Props> = ({state, stateName, subStateName, onStart, onStartFresh, style,
                                                        compact = false, showFollowUp = true, onManualDrive,
                                                        skippedM, returnHomeOnIncomplete, onOpenReturnHomeSetting}) => {
    const {t} = useTranslation();
    const {modal, notification} = App.useApp();
    const mowerAction = useMowerAction();
    const status = useStatus();
    const resumeAvailable = useCoverageResumeAvailable();
    const isMobile = useIsMobile();
    const [busy, setBusy] = useState<string | null>(null);
    const [stopped, setStopped] = useState(false);
    // STUCK_NEEDS_HELP: the owner moves the robot by hand first; Resume appears only
    // once Reset was pressed in this latch (it clears the stuck guard history).
    // Shared with the map toolbars, which gate their Resume on the same flag.
    const [stuckReset, markStuckReset] = useStuckReset(stateName);
    const phrase = (p: Phrase) => t(p.key, p.params);

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

    // antd puts `action` beside the text; on a phone the buttons then squeeze the
    // text column to ~1 character per line. Narrow screens stack them below it.
    const alertBody = (description: React.ReactNode, action: React.ReactNode) => isMobile
        ? {
            description: (
                <div style={{flex: "1 1 auto", minWidth: 0}} data-testid="mission-alert-stacked">
                    {description}
                    <div style={{marginTop: 8}}>{action}</div>
                </div>
            ),
            action: undefined,
        }
        : {description, action};

    if (fault) {
        const stuck = stateName === "STUCK_NEEDS_HELP";
        const dockFailed = stateName === "NAV_TO_DOCK_FAILED";
        const title = stuck ? t('missionText.stuckTitle')
            : dockFailed ? t('missionText.dockFailedTitle')
                : stateRenderer(stateName);
        const body = stuck
            ? <>
                <div>{phrase(stuckReason(subStateName))}</div>
                <div data-testid="mission-stuck-hint">
                    {stuckReset ? t('missionText.stuckAfterReset') : t('missionText.stuckBody')}
                </div>
            </>
            : dockFailed
                ? <div data-testid="mission-dock-reason">{phrase(dockFailReason(subStateName))}</div>
                : (subStateName || t('missionStop.faultNoReason'));
        return (
            <Alert
                style={style}
                type="error"
                showIcon
                data-testid="mission-fault"
                message={<span data-testid="mission-fault-title">{title}</span>}
                {...alertBody(body,
                    <Space direction="vertical">
                        {stopButton}
                        {dockFailed && (
                            <Button type="primary" icon={<DockIcon/>} loading={busy === "home"}
                                    onClick={run("home", CMD_HOME)} data-testid="mission-dock-retry">
                                {t('missionText.tryAgain')}
                            </Button>
                        )}
                        {dockFailed && onManualDrive && (
                            <Button icon={<ControlOutlined/>} onClick={onManualDrive} data-testid="mission-drive-home">
                                {t('missionText.driveHome')}
                            </Button>
                        )}
                        {canReset(stateName) && !(stuck && stuckReset) && (
                            <Button type={stuck ? "primary" : "default"} icon={<RedoOutlined/>} loading={busy === "reset"}
                                    onClick={run("reset", CMD_RESET_EMERGENCY, () => { if (stuck) markStuckReset(); })}
                                    data-testid="mission-reset">
                                {t('missionStop.reset')}
                            </Button>
                        )}
                        {!dockFailed && resumeButton}
                        {!dockFailed && startFreshButton}
                    </Space>
                )}
            />
        );
    }

    if (isNotice(stateName)) {
        // MOWING_INCOMPLETE: stopped in place, sub_state says what was not mowed.
        // One action name everywhere: "Resume mowing" (START with the kept cursor).
        const parts = incompletePhrases(subStateName, skippedM).map(phrase);
        return (
            <Alert
                style={style}
                type="warning"
                showIcon
                data-testid="mission-notice"
                message={t('missionText.incompleteTitle')}
                {...alertBody(<>
                    <div data-testid="mission-incomplete-summary">
                        {parts.length > 0 ? parts.join(" · ") : (subStateName || t('missionStop.faultNoReason'))}
                    </div>
                    {resumable && <div>{t('missionText.incompleteHint')}</div>}
                    {returnHomeOnIncomplete !== undefined && (
                        <div data-testid="mission-return-home-setting">
                            {returnHomeOnIncomplete ? t('missionText.returnHomeOn') : t('missionText.returnHomeOff')}
                            {onOpenReturnHomeSetting && (
                                <Button size="small" type="link" onClick={onOpenReturnHomeSetting}
                                        data-testid="mission-return-home-link">
                                    {t('missionText.returnHomeChange')}
                                </Button>
                            )}
                        </div>
                    )}
                </>,
                    <Space direction="vertical">
                        {resumeButton ?? (
                            <Button type="primary" icon={<PlayCircleOutlined/>} onClick={onStart}>
                                {t('missionStop.start')}
                            </Button>
                        )}
                        <Button icon={<DockIcon/>} loading={busy === "home"}
                                onClick={run("home", CMD_HOME)}>
                            {t('missionStop.returnToDock')}
                        </Button>
                        {startFreshButton}
                    </Space>
                )}
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

    // "heading not aligned" (IDLE): the next START will be refused, so the hint
    // and its Drive action come first, in full (it used to be cut off on a phone).
    // Not in the map editor (showFollowUp false): it would cover the tools.
    const headingHint = showFollowUp && isHeadingHint(subStateName) ? (
        <Alert
            type="info"
            showIcon
            data-testid="mission-heading-hint"
            style={compact ? {width: "100%"} : style}
            message={t('missionText.headingTitle')}
            {...alertBody(<>
                {t('missionText.headingBody')}
                {!onManualDrive && <div>{t('missionText.headingMapHint')}</div>}
            </>, onManualDrive ? (
                <Button type="primary" icon={<ControlOutlined/>} onClick={onManualDrive} data-testid="mission-heading-drive">
                    {t('missionText.headingDrive')}
                </Button>
            ) : undefined)}
        />
    ) : null;

    if ((stopped || resumable) && showFollowUp) {
        const title = resumable && !stopped ? t('missionStop.interruptedTitle') : t('missionStop.stoppedTitle');
        const followUpButtons = <>
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
        </>;
        if (compact) {
            return (<>
                {headingHint}
                <div data-testid="mission-stopped" style={{display: "flex", flexWrap: "wrap", alignItems: "center",
                    gap: 6, minWidth: 0, ...style}}>
                    <span style={{fontWeight: 600, flex: "1 1 auto", minWidth: 0}}>{title}</span>
                    {followUpButtons}
                    {!resumable && (
                        <Button type="text" size="small" icon={<CloseOutlined/>} aria-label={t('nav.close')}
                                onClick={() => setStopped(false)}/>
                    )}
                </div>
            </>);
        }
        return (<>
            {headingHint}
            <Alert
                style={style}
                type="info"
                showIcon
                closable={!resumable}
                onClose={() => setStopped(false)}
                data-testid="mission-stopped"
                message={title}
                {...alertBody(resumable ? <>{t('missionStop.interruptedHint')} {whyStopped}</> : undefined,
                    <Space direction="vertical">{followUpButtons}</Space>
                )}
            />
        </>);
    }
    return headingHint;
};
