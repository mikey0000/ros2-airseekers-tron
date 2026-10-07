import {App, Button, Dropdown, Space} from "antd";
import type {MenuProps} from "antd";
import type {MenuItemType} from "antd/es/menu/interface";
import {
    GlobalOutlined,
    EllipsisOutlined,
    EditOutlined,
    DatabaseOutlined,
    DownloadOutlined,
    ControlOutlined,
    PlayCircleOutlined,
    StepForwardOutlined,
    StopOutlined,
    WarningOutlined,
    ScissorOutlined,
    AimOutlined,
    ForwardOutlined,
    CaretRightOutlined,
    PauseOutlined,
    ThunderboltOutlined,
    CheckOutlined,
    CloseOutlined,
    ImportOutlined,
    DeleteOutlined,
    EyeOutlined,
    LogoutOutlined,
    SettingOutlined,
    NodeIndexOutlined,
} from "@ant-design/icons";
import type {MenuInfo} from "rc-menu/lib/interface";
import {useTranslation} from "react-i18next";
import AsyncButton from "../../../components/AsyncButton.tsx";
import AsyncDropDownButton from "../../../components/AsyncDropDownButton.tsx";
import type {Feature} from "geojson";
import {canPreviewPlan} from "../../../hooks/usePlanPreview.ts";
import {canHome, canUndock} from "../../../utils/missionStates.ts";
import {confirmUndock} from "./undockConfirm.ts";
import {confirmAction, dockNeedsConfirm} from "./confirmAction.ts";
import {DockIcon} from "../../../components/DockIcon.tsx";

/** Menu key of the "all areas" entry of the Preview plan dropdown. */
export const PREVIEW_ALL_KEY = "__all__";

interface MowingAreaItem extends MenuItemType {
    feat: Feature;
}

interface MapToolbarProps {
    manualMode: boolean;
    useSatellite: boolean;
    mowingAreas: MowingAreaItem[];
    stateName?: string;
    highLevelState?: number;
    emergency?: boolean;
    onEditMap: () => void;
    onToggleSatellite: () => void;
    onManualMode: () => Promise<void>;
    onStopManualMode: () => Promise<void>;
    onBackupMap: () => void;
    onRestoreMap: () => void;
    onDownloadGeoJSON: () => void;
    onImportOpenMower: () => void;
    onResetMowingProgress: () => void;
    onMowArea: (key: string) => Promise<void>;
    /** Plan preview of one area (its menu key) or PREVIEW_ALL_KEY; no mowing. */
    onPreviewPlan?: (key: string) => Promise<void>;
    /** Areas whose mow settings can be opened; empty/undefined hides the button. */
    settingsAreas?: {key: string; label: string}[];
    onAreaSettings?: (key: string) => void;
    pitched?: boolean;
    onTogglePitch?: () => void;
    onStart?: () => Promise<void>;
    /** Set when a resume cursor exists and the mission is at rest (canResume): shows the
     *  primary "Resume mowing" (START from the cursor, no sheet) and moves Start to
     *  "Start fresh" in More. */
    onResume?: () => Promise<void>;
    /** "Start fresh" (More menu, only with onResume): opens the start flow in fresh mode. */
    onStartFresh?: () => Promise<void>;
    onHome?: () => Promise<void>;
    /** Standalone undock (shown only while docked: IDLE_DOCKED / CHARGING). */
    onUndock?: () => Promise<void>;
    onEmergencyOn?: () => Promise<void>;
    onEmergencyOff?: () => Promise<void>;
    onAreaRecording?: () => Promise<void>;
    /** Record an open path (navigation band) by driving with the joystick. */
    onPathRecording?: () => Promise<void>;
    onMowNextArea?: () => Promise<void>;
    onContinueOrPause?: () => Promise<void>;
    onBladeForward?: () => Promise<void>;
    onBladeBackward?: () => Promise<void>;
    onBladeOff?: () => Promise<void>;
    onRecordFinish?: () => Promise<void>;
    onRecordCancel?: () => Promise<void>;
}

export const MapToolbar = ({
    manualMode, useSatellite, mowingAreas, stateName, highLevelState, emergency,
    onEditMap, onToggleSatellite,
    onManualMode, onStopManualMode,
    onBackupMap, onRestoreMap, onDownloadGeoJSON, onImportOpenMower, onResetMowingProgress,
    onMowArea, onPreviewPlan, settingsAreas, onAreaSettings, pitched, onTogglePitch,
    onStart, onResume, onStartFresh, onHome, onUndock, onEmergencyOn, onEmergencyOff,
    onAreaRecording, onPathRecording, onMowNextArea, onContinueOrPause,
    onBladeForward, onBladeBackward, onBladeOff,
    onRecordFinish, onRecordCancel,
}: MapToolbarProps) => {
    const {notification, modal} = App.useApp();
    const {t} = useTranslation();
    // DIG_OBSTRUCTION is a held robot (numeric state IDLE, wheels hard-stopped
    // by firmware): the exits are Play after lifting it clear, or Home — so
    // offer Continue, not Pause.
    const isIdle = stateName === "IDLE" || stateName === "IDLE_DOCKED" || stateName === "DIG_OBSTRUCTION";
    const isRecording = stateName === "RECORDING";
    // Numeric state is the authoritative signal. States 2 and above are
    // autonomous, recording, manual mowing, or a future active mode; clearing
    // persisted progress during any of them could race an active mission.
    // Fail closed until the first status frame arrives.
    const resetDisabled = highLevelState === undefined || highLevelState >= 2;

    const safeCall = (fn?: () => Promise<void>) => {
        fn?.().catch((e: Error) => {
            console.error(e);
            notification.error({
                message: t("mapToolbar.actionFailed"),
                description: e.message,
            });
        });
    };

    const confirmBlade = (fn?: () => Promise<void>) => confirmAction(modal, {
        title: t("mapToolbar.bladeConfirmTitle"),
        content: t("mapToolbar.bladeConfirmContent"),
        okText: t("mapToolbar.bladeConfirmOk"),
        cancelText: t("mapToolbar.undockCancel"),
        danger: true,
    }, () => fn?.() ?? Promise.resolve());
    const onDock = () => dockNeedsConfirm(stateName)
        ? confirmAction(modal, {
            title: t("mapToolbar.dockConfirmTitle"),
            content: t("mapToolbar.dockConfirmContent"),
            okText: t("mapToolbar.home"),
            cancelText: t("mapToolbar.undockCancel"),
        }, onHome!)
        : onHome!();

    const moreMenuItems: MenuProps["items"] = [
        {key: "satellite", icon: <GlobalOutlined />, label: useSatellite ? t("mapToolbar.darkMap") : t("mapToolbar.satellite")},
        ...(onTogglePitch
            ? [{key: "pitch", icon: <GlobalOutlined />, label: pitched ? t("mapToolbar.flattenMap") : t("mapToolbar.tilt3dView")} satisfies NonNullable<MenuProps["items"]>[number]]
            : []),
        {type: "divider"},
        {key: "areaRecording", icon: <AimOutlined />, label: t("mapToolbar.recordArea")},
        ...(onPathRecording ? [{key: "pathRecording", icon: <NodeIndexOutlined />, label: t("mapToolbar.recordPath")}] : []),
        {key: "mowNext", icon: <ForwardOutlined />, label: t("mapToolbar.mowNextArea")},
        ...(onResume && onStartFresh
            ? [{key: "startFresh", icon: <PlayCircleOutlined />, label: t("missionStop.startFresh")} satisfies NonNullable<MenuProps["items"]>[number]]
            : []),
        {key: "continueOrPause", icon: isIdle ? <CaretRightOutlined /> : <PauseOutlined />, label: isIdle ? t("mapToolbar.continue") : t("mapToolbar.pause")},
        {type: "divider"},
        ...(manualMode
            ? [{key: "stopManual", icon: <StopOutlined />, label: t("mapToolbar.stopManualMowing"), danger: true} satisfies NonNullable<MenuProps["items"]>[number]]
            : [{key: "manual", icon: <ControlOutlined />, label: t("mapToolbar.manualMowing")} satisfies NonNullable<MenuProps["items"]>[number]]
        ),
        {type: "divider"},
        {key: "bladeForward", icon: <ThunderboltOutlined />, label: t("mapToolbar.bladeForward")},
        {key: "bladeBackward", icon: <ThunderboltOutlined />, label: t("mapToolbar.bladeBackward")},
        {key: "bladeOff", icon: <ThunderboltOutlined />, label: t("mapToolbar.bladeOff"), danger: true},
        {type: "divider"},
        {key: "backup", icon: <DatabaseOutlined />, label: t("mapToolbar.backupMap")},
        {key: "restore", icon: <DatabaseOutlined />, label: t("mapToolbar.restoreMap")},
        {key: "importOpenMower", icon: <ImportOutlined />, label: t("mapToolbar.importFromOpenMower")},
        {
            key: "resetMowingProgress",
            icon: <DeleteOutlined />,
            label: t("resetMowingProgress.action"),
            danger: true,
            disabled: resetDisabled,
        },
        {type: "divider"},
        {key: "download", icon: <DownloadOutlined />, label: t("mapToolbar.downloadGeojson")},
    ];

    const handleMoreClick: MenuProps["onClick"] = ({key}: MenuInfo) => {
        switch (key) {
            case "satellite": onToggleSatellite(); break;
            case "pitch": onTogglePitch?.(); break;
            case "manual": safeCall(() => onManualMode()); break;
            case "stopManual": safeCall(() => onStopManualMode()); break;
            case "areaRecording": safeCall(onAreaRecording); break;
            case "pathRecording": safeCall(onPathRecording); break;
            case "mowNext": safeCall(onMowNextArea); break;
            case "startFresh": safeCall(onStartFresh); break;
            case "continueOrPause": safeCall(onContinueOrPause); break;
            case "bladeForward": safeCall(() => confirmBlade(onBladeForward)); break;
            case "bladeBackward": safeCall(() => confirmBlade(onBladeBackward)); break;
            case "bladeOff": safeCall(onBladeOff); break;
            case "backup": onBackupMap(); break;
            case "restore": onRestoreMap(); break;
            case "importOpenMower": onImportOpenMower(); break;
            case "resetMowingProgress": onResetMowingProgress(); break;
            case "download": onDownloadGeoJSON(); break;
        }
    };

    return (
        <Space size="small" wrap>
            <Button
                type="primary"
                icon={<EditOutlined />}
                onClick={onEditMap}
            >
                {t("mapToolbar.editMap")}
            </Button>

            {isRecording ? (
                <>
                    <AsyncButton
                        type="primary"
                        icon={<CheckOutlined />}
                        onAsyncClick={onRecordFinish!}
                    >
                        {t("mapToolbar.finishRecording")}
                    </AsyncButton>
                    <AsyncButton
                        danger
                        icon={<CloseOutlined />}
                        onAsyncClick={onRecordCancel!}
                    >
                        {t("mapToolbar.cancelRecording")}
                    </AsyncButton>
                </>
            ) : (
                <>
                    {onResume && (
                        <AsyncButton
                            type="primary"
                            icon={<StepForwardOutlined />}
                            onAsyncClick={onResume}
                            data-testid="toolbar-resume"
                        >
                            {t("missionStop.resumeMowing")}
                        </AsyncButton>
                    )}
                    {isIdle && !onResume && (
                        <AsyncButton
                            type="primary"
                            icon={<PlayCircleOutlined />}
                            onAsyncClick={onStart!}
                        >
                            {t("mapToolbar.start")}
                        </AsyncButton>
                    )}
                    {onUndock && canUndock(stateName) && (
                        <AsyncButton
                            icon={<LogoutOutlined />}
                            onAsyncClick={() => confirmUndock(modal, t, onUndock)}
                        >
                            {t("mapToolbar.undock")}
                        </AsyncButton>
                    )}
                    {/* Dock (return to dock) whenever it is allowed, idle off-dock
                        included; hidden when already docked (canHome). */}
                    {onHome && canHome(highLevelState, stateName) && (
                        <AsyncButton
                            type={isIdle ? "default" : "primary"}
                            icon={<DockIcon />}
                            onAsyncClick={onDock}
                        >
                            {t("mapToolbar.home")}
                        </AsyncButton>
                    )}
                </>
            )}

            {!emergency ? (
                <AsyncButton
                    danger
                    icon={<WarningOutlined />}
                    onAsyncClick={onEmergencyOn!}
                >
                    {t("mapToolbar.emergencyOn")}
                </AsyncButton>
            ) : (
                <AsyncButton
                    danger
                    icon={<WarningOutlined />}
                    onAsyncClick={onEmergencyOff!}
                >
                    {t("mapToolbar.emergencyOff")}
                </AsyncButton>
            )}

            <AsyncDropDownButton
                icon={<ScissorOutlined />}
                menu={{
                    items: mowingAreas,
                    onAsyncClick: (e: MenuInfo) => onMowArea(e.key),
                }}
            >
                {t("mapToolbar.mowArea")}
            </AsyncDropDownButton>

            {onPreviewPlan && canPreviewPlan(stateName) && (
                <AsyncDropDownButton
                    icon={<EyeOutlined />}
                    menu={{
                        items: [
                            {key: PREVIEW_ALL_KEY, label: t("planPreview.allAreas")},
                            ...mowingAreas.map(({key, label}) => ({key, label})),
                        ],
                        onAsyncClick: (e: MenuInfo) => onPreviewPlan(e.key),
                    }}
                >
                    {t("planPreview.button")}
                </AsyncDropDownButton>
            )}

            {onAreaSettings && settingsAreas && settingsAreas.length > 0 && (
                <Dropdown
                    menu={{
                        items: settingsAreas.map(({key, label}) => ({key, label})),
                        onClick: ({key}: MenuInfo) => onAreaSettings(key),
                    }}
                    trigger={["click"]}
                >
                    <Button icon={<SettingOutlined />}>{t("mapToolbarMobile.areaSettings")}</Button>
                </Dropdown>
            )}

            <AsyncButton
                danger={manualMode}
                icon={manualMode ? <StopOutlined /> : <ControlOutlined />}
                onAsyncClick={manualMode ? onStopManualMode : onManualMode}
            >
                {manualMode ? t("mapToolbar.stopManual") : t("mapToolbar.manualMow")}
            </AsyncButton>

            <Dropdown
                menu={{items: moreMenuItems, onClick: handleMoreClick}}
                trigger={["click"]}
            >
                <Button icon={<EllipsisOutlined />}>{t("mapToolbar.more")}</Button>
            </Dropdown>
        </Space>
    );
};
