import {useState} from "react";
import {useTranslation} from "react-i18next";
import {useThemeMode} from "../../../theme/ThemeContext.tsx";
import {App, Button, Dropdown, Space} from "antd";
import type {MenuProps} from "antd";
import {
    UndoOutlined,
    RedoOutlined,
    GlobalOutlined,
    PictureOutlined,
    EllipsisOutlined,
    SaveOutlined,
    EditOutlined,
    FormOutlined,
    NodeIndexOutlined,
    ApiOutlined,
    CloseOutlined,
    DeleteOutlined,
    MergeCellsOutlined,
    DatabaseOutlined,
    DownloadOutlined,
    UploadOutlined,
    ScissorOutlined,
    ControlOutlined,
    SplitCellsOutlined,
    MinusSquareOutlined,
    PlayCircleOutlined,
    WarningOutlined,
    PlusOutlined,
    BorderOutlined,
    AimOutlined,
    ForwardOutlined,
    CaretRightOutlined,
    PauseOutlined,
    ThunderboltOutlined,
    ImportOutlined,
    EyeOutlined,
    SettingOutlined,
    LogoutOutlined,
    HighlightOutlined,
    StopOutlined,
} from "@ant-design/icons";
import {DockIcon} from "../../../components/DockIcon.tsx";
import {confirmAction, dockNeedsConfirm} from "./confirmAction.ts";
import {canPreviewPlan} from "../../../hooks/usePlanPreview.ts";
import {PREVIEW_ALL_KEY} from "./MapToolbar.tsx";
import {confirmUndock} from "./undockConfirm.ts";
import {canHome, canUndock} from "../../../utils/missionStates.ts";
import type {MenuInfo} from "rc-menu/lib/interface";
import AsyncButton from "../../../components/AsyncButton.tsx";
import type {Feature} from "geojson";
import type {MenuItemType} from "antd/es/menu/interface";
import {ShapePickerDropdown} from "./ShapePickerDropdown.tsx";
import type {ShapeType} from "../hooks/useMapEditing.ts";

interface MowingAreaItem extends MenuItemType {
    feat: Feature;
}

interface MapToolbarMobileProps {
    editMap: boolean;
    hasUnsavedChanges: boolean;
    manualMode: boolean;
    useSatellite: boolean;
    historyIndex: number;
    editHistoryLength: number;
    mowingAreas: MowingAreaItem[];
    onEditMap: () => void;
    onSaveMap: () => Promise<void>;
    onUndo: () => void;
    onRedo: () => void;
    onToggleSatellite: () => void;
    /** Open the custom imagery sheet. */
    onImagery?: () => void;
    onManualMode: () => Promise<void>;
    onStopManualMode: () => Promise<void>;
    onBackupMap: () => void;
    onRestoreMap: () => void;
    onDownloadGeoJSON: () => void;
    onUploadGeoJSON: () => void;
    onImportOpenMower: () => void;
    onResetMowingProgress: () => void;
    onMowArea: (key: string) => Promise<void>;
    /** Plan preview of one area (its menu key) or PREVIEW_ALL_KEY; no mowing. */
    onPreviewPlan?: (key: string) => Promise<void>;
    /** Areas whose mow settings can be opened (view mode); empty/undefined hides the button. */
    settingsAreas?: {key: string; label: string}[];
    onAreaSettings?: (key: string) => void;
    selectedFeatureCount?: number;
    onEditSelectedFeature?: () => void;
    onDrawPolygon?: () => void;
    onDrawShape?: (shape: ShapeType, sizeMeters: number) => void;
    onDrawEmoji?: (emoji: string, sizeMeters: number) => void;
    onTrash?: () => void;
    onCombine?: () => void;
    onSubtract?: () => void;
    onSplit?: () => void;
    onPlaceDock?: () => void;
    dockPlacementMode?: boolean;
    onDrawPath?: () => void;
    onConnectDock?: () => void;
    onDrawPathToDock?: () => void;
    onEditPath?: () => void;
    editPathEnabled?: boolean;
    dockAvailable?: boolean;
    stateName?: string;
    highLevelState?: number;
    emergency?: boolean;
    onStart?: () => Promise<void>;
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

export const MapToolbarMobile = ({
    editMap, hasUnsavedChanges, manualMode, useSatellite,
    historyIndex, editHistoryLength, mowingAreas,
    onEditMap, onSaveMap, onUndo, onRedo, onToggleSatellite, onImagery,
    onManualMode, onStopManualMode,
    onBackupMap, onRestoreMap, onDownloadGeoJSON, onUploadGeoJSON, onImportOpenMower, onResetMowingProgress,
    onMowArea, onPreviewPlan, settingsAreas, onAreaSettings, selectedFeatureCount = 0, onEditSelectedFeature,
    onDrawPolygon, onDrawShape, onDrawEmoji, onTrash, onCombine, onSubtract, onSplit,
    onPlaceDock, dockPlacementMode, onDrawPath, onConnectDock, dockAvailable,
    onDrawPathToDock, onEditPath, editPathEnabled,
    stateName, highLevelState, emergency,
    onStart, onHome, onUndock, onEmergencyOn, onEmergencyOff,
    onAreaRecording, onPathRecording, onMowNextArea, onContinueOrPause,
    onBladeForward, onBladeBackward, onBladeOff,
}: MapToolbarMobileProps) => {
    const {colors, displayMode} = useThemeMode();
    const {notification, modal} = App.useApp();
    const {t} = useTranslation();
    const [mowLoading, setMowLoading] = useState(false);
    const [previewLoading, setPreviewLoading] = useState(false);

    // 44px minimum touch target on every control in the cluster (Apple/WCAG
    // thumb-reach guideline). Applied via a shared style so size="large" AntD
    // buttons never fall below the floor.
    const touchTarget: React.CSSProperties = {minWidth: 44, minHeight: 44};

    const toolbarStyle: React.CSSProperties = {
        // Anchor to the VIEWPORT (fixed), not the map container — that container
        // bleeds past the viewport (height: 100% + 122px, negative bottom margin),
        // and iOS Safari positions an absolute child relative to that off-screen
        // bottom, hiding the toolbar entirely. Fixed keeps it just above the nav.
        position: "fixed",
        // Sit just above the floating bottom-nav (≈85px tall + safe-area). Derive
        // the offset from the safe-area inset so it tracks the nav height on
        // notched phones, with a comfortable gap above the nav pill.
        bottom: "calc(env(safe-area-inset-bottom, 0px) + 100px)",
        // Leave room on the right for the pinned STOP button so the scrolling
        // cluster never slides under it.
        left: 12,
        right: 80,
        // Above the bottom-nav (zIndex 50) so the nav never paints over it.
        zIndex: 55,
        display: "flex",
        gap: 8,
        // Horizontally scrollable cluster — never force a multi-row wrap that
        // would push controls under the bottom-nav on short phones.
        flexWrap: "nowrap",
        overflowX: "auto",
        overflowY: "hidden",
        WebkitOverflowScrolling: "touch",
        alignItems: "center",
        background: colors.glassBackground,
        backdropFilter: displayMode === 'visual' ? 'blur(22px) saturate(140%)' : undefined,
        WebkitBackdropFilter: displayMode === 'visual' ? 'blur(22px) saturate(140%)' : undefined,
        borderRadius: 18,
        border: colors.glassBorder,
        boxShadow: colors.glassShadow,
        padding: "8px 12px",
    };

    // The emergency / STOP button is deliberately separated from the tool
    // cluster and pinned to its own fixed corner so a panicking beginner finds
    // it instantly. Larger than every other control, filled rose.
    const stopButtonStyle: React.CSSProperties = {
        position: "fixed",
        bottom: "calc(env(safe-area-inset-bottom, 0px) + 100px)",
        right: 12,
        zIndex: 56,
        width: 60,
        height: 60,
        borderRadius: 18,
        fontWeight: 700,
        background: emergency ? colors.bgElevated : colors.danger,
        borderColor: colors.danger,
        color: emergency ? colors.danger : colors.text,
        boxShadow: colors.glassShadow,
    };

    // DIG_OBSTRUCTION is a held robot (numeric state IDLE, wheels hard-stopped
    // by firmware): the exits are Play after lifting it clear, or Home — so
    // offer Continue, not Pause.
    const isIdle = stateName === "IDLE" || stateName === "IDLE_DOCKED" || stateName === "DIG_OBSTRUCTION";
    const isRecording = stateName === "RECORDING";
    const resetDisabled = highLevelState === undefined || highLevelState >= 2;

    const safeCall = (fn?: () => Promise<void>) => {
        fn?.().catch((e: Error) => {
            console.error(e);
            notification.error({
                message: t("mapToolbarMobile.actionFailed"),
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
            okText: t("mapToolbarMobile.home"),
            cancelText: t("mapToolbar.undockCancel"),
        }, onHome!)
        : onHome!();

    const dataMenuItems: MenuProps["items"] = [
        {key: "satellite", icon: <GlobalOutlined />, label: useSatellite ? t("mapToolbarMobile.darkMap") : t("mapToolbarMobile.satellite")},
        ...(onImagery ? [{key: "imagery", icon: <PictureOutlined />, label: t("imagery.title")}] : []),
        {type: "divider"},
        {key: "areaRecording", icon: <AimOutlined />, label: t("mapToolbarMobile.recordArea")},
        ...(onPathRecording ? [{key: "pathRecording", icon: <NodeIndexOutlined />, label: t("mapToolbarMobile.recordPath")}] : []),
        {key: "mowNext", icon: <ForwardOutlined />, label: t("mapToolbarMobile.mowNextArea")},
        {key: "continueOrPause", icon: isIdle ? <CaretRightOutlined /> : <PauseOutlined />, label: isIdle ? t("mapToolbarMobile.continue") : t("mapToolbarMobile.pause")},
        {type: "divider"},
        {key: "bladeForward", icon: <ThunderboltOutlined />, label: t("mapToolbarMobile.bladeForward")},
        {key: "bladeBackward", icon: <ThunderboltOutlined />, label: t("mapToolbarMobile.bladeBackward")},
        {key: "bladeOff", icon: <ThunderboltOutlined />, label: t("mapToolbarMobile.bladeOff"), danger: true},
        {type: "divider"},
        {key: "backup", icon: <DatabaseOutlined />, label: t("mapToolbarMobile.backupMap")},
        {key: "restore", icon: <DatabaseOutlined />, label: t("mapToolbarMobile.restoreMap")},
        {key: "importOpenMower", icon: <ImportOutlined />, label: t("mapToolbarMobile.importFromOpenMower")},
        {
            key: "resetMowingProgress",
            icon: <DeleteOutlined />,
            label: t("resetMowingProgress.action"),
            danger: true,
            disabled: resetDisabled,
        },
        {type: "divider"},
        {key: "download", icon: <DownloadOutlined />, label: t("mapToolbarMobile.downloadGeojson")},
        ...(editMap
            ? [{key: "upload", icon: <UploadOutlined />, label: t("mapToolbarMobile.uploadGeojson")} satisfies NonNullable<MenuProps["items"]>[number]]
            : []),
    ];

    const handleMoreClick: MenuProps["onClick"] = ({key}: MenuInfo) => {
        switch (key) {
            case "satellite": onToggleSatellite(); break;
            case "imagery": onImagery?.(); break;
            case "areaRecording": safeCall(onAreaRecording); break;
            case "pathRecording": safeCall(onPathRecording); break;
            case "mowNext": safeCall(onMowNextArea); break;
            case "continueOrPause": safeCall(onContinueOrPause); break;
            case "bladeForward": safeCall(() => confirmBlade(onBladeForward)); break;
            case "bladeBackward": safeCall(() => confirmBlade(onBladeBackward)); break;
            case "bladeOff": safeCall(onBladeOff); break;
            case "backup": onBackupMap(); break;
            case "restore": onRestoreMap(); break;
            case "importOpenMower": onImportOpenMower(); break;
            case "resetMowingProgress": onResetMowingProgress(); break;
            case "download": onDownloadGeoJSON(); break;
            case "upload": onUploadGeoJSON(); break;
        }
    };

    const handleMowClick: MenuProps["onClick"] = ({key}: MenuInfo) => {
        setMowLoading(true);
        onMowArea(key).finally(() => setMowLoading(false));
    };

    const handlePreviewClick: MenuProps["onClick"] = ({key}: MenuInfo) => {
        if (!onPreviewPlan) return;
        setPreviewLoading(true);
        safeCall(() => onPreviewPlan(key).finally(() => setPreviewLoading(false)));
    };

    const editMenuItems: MenuProps["items"] = [
        // Draw path / Path to dock / Edit path are icon buttons in the cluster.
        ...(onConnectDock ? [{key: "connectDock", icon: <ApiOutlined />, label: t("mapToolbarMobile.connectDock"), disabled: !dockAvailable}] : []),
        {key: "editProps", icon: <FormOutlined />, label: t("mapToolbarMobile.editProperties"), disabled: selectedFeatureCount !== 1},
        {key: "combine", icon: <MergeCellsOutlined />, label: t("mapToolbarMobile.combine"), disabled: selectedFeatureCount < 2},
        {key: "subtract", icon: <MinusSquareOutlined />, label: t("mapToolbarMobile.subtract"), disabled: selectedFeatureCount !== 2},
        {key: "split", icon: <SplitCellsOutlined />, label: t("mapToolbarMobile.splitDrawLine"), disabled: selectedFeatureCount !== 1},
        {type: "divider"},
        ...dataMenuItems,
    ];

    const handleEditMenuClick: MenuProps["onClick"] = ({key}: MenuInfo) => {
        switch (key) {
            case "connectDock": onConnectDock?.(); break;
            case "editProps": onEditSelectedFeature?.(); break;
            case "combine": onCombine?.(); break;
            case "subtract": onSubtract?.(); break;
            case "split": onSplit?.(); break;
            default: handleMoreClick({key} as MenuInfo); break;
        }
    };

    // Always-present, dominant, separated emergency control. Rendered outside
    // the scrolling cluster in its own fixed corner. Keeps the exact emergency
    // on/off commands — only the styling/label/placement changed.
    const stopButton = (
        <AsyncButton
            danger
            type={emergency ? "default" : "primary"}
            icon={<WarningOutlined />}
            onAsyncClick={(emergency ? onEmergencyOff : onEmergencyOn)!}
            aria-label={emergency ? t("mapToolbarMobile.emergencyOff") : t("mapToolbarMobile.emergencyOn")}
            style={stopButtonStyle}
        >
            {t("mapToolbarMobile.stop")}
        </AsyncButton>
    );

    if (editMap) {
        return (
            <>
                <div style={toolbarStyle}>
                    {/* Save / Cancel */}
                    <Space.Compact size="large">
                        <AsyncButton
                            type="primary"
                            size="large"
                            danger={hasUnsavedChanges}
                            icon={<SaveOutlined />}
                            onAsyncClick={onSaveMap}
                            aria-label={t("mapToolbarMobile.save")}
                            style={touchTarget}
                        />
                        <Button
                            size="large"
                            icon={<CloseOutlined />}
                            onClick={onEditMap}
                            aria-label={t("mapToolbarMobile.cancel")}
                            style={touchTarget}
                        />
                    </Space.Compact>

                    {/* Undo / Redo */}
                    <Space.Compact size="large">
                        <Button
                            size="large"
                            icon={<UndoOutlined />}
                            onClick={onUndo}
                            disabled={historyIndex <= 0}
                            aria-label={t("mapToolbarMobile.undo")}
                            style={touchTarget}
                        />
                        <Button
                            size="large"
                            icon={<RedoOutlined />}
                            onClick={onRedo}
                            disabled={historyIndex >= editHistoryLength - 1}
                            aria-label={t("mapToolbarMobile.redo")}
                            style={touchTarget}
                        />
                    </Space.Compact>

                    {/* Draw / Add shape / Delete */}
                    <Space.Compact size="large">
                        <Button
                            size="large"
                            icon={<BorderOutlined />}
                            onClick={onDrawPolygon}
                            aria-label={t("mapToolbarMobile.drawPolygon")}
                            style={touchTarget}
                        />
                        <ShapePickerDropdown
                            onDrawShape={onDrawShape}
                            onDrawEmoji={onDrawEmoji}
                            placement="top"
                        >
                            <Button size="large" icon={<PlusOutlined />} aria-label={t("mapToolbarMobile.addShape")} style={touchTarget} />
                        </ShapePickerDropdown>
                        <Button
                            size="large"
                            icon={<DeleteOutlined />}
                            disabled={selectedFeatureCount === 0}
                            onClick={onTrash}
                            aria-label={t("mapToolbarMobile.delete")}
                            style={touchTarget}
                        />
                    </Space.Compact>

                    <Button
                        size="large"
                        icon={<AimOutlined />}
                        type={dockPlacementMode ? "primary" : "default"}
                        onClick={onPlaceDock}
                        aria-label={t("mapToolbarMobile.placeDock")}
                        style={touchTarget}
                    />

                    {/* Path tools (drive-only corridors), same handlers as the
                        desktop MapEditorToolbar. */}
                    {(onDrawPath || onDrawPathToDock || onEditPath) && (
                        <Space.Compact size="large">
                            {onDrawPath && (
                                <Button
                                    size="large"
                                    icon={<NodeIndexOutlined />}
                                    onClick={onDrawPath}
                                    aria-label={t("mapToolbarMobile.drawPath")}
                                    style={touchTarget}
                                />
                            )}
                            {onDrawPathToDock && (
                                <Button
                                    size="large"
                                    icon={<DockIcon />}
                                    onClick={onDrawPathToDock}
                                    disabled={!dockAvailable}
                                    aria-label={t("mapToolbarMobile.drawPathToDock")}
                                    style={touchTarget}
                                />
                            )}
                            {onEditPath && (
                                <Button
                                    size="large"
                                    icon={<HighlightOutlined />}
                                    onClick={onEditPath}
                                    disabled={!editPathEnabled}
                                    aria-label={t("mapToolbarMobile.editPath")}
                                    style={touchTarget}
                                />
                            )}
                        </Space.Compact>
                    )}

                    {/* Combine/Subtract/Split now live inside this More menu
                        (editMenuItems) to keep the top row uncluttered. */}
                    <Dropdown
                        menu={{items: editMenuItems, onClick: handleEditMenuClick}}
                        trigger={["click"]}
                        placement="topRight"
                    >
                        <Button size="large" icon={<EllipsisOutlined />} aria-label={t("mapToolbarMobile.more")} style={touchTarget} />
                    </Dropdown>
                </div>
                {stopButton}
            </>
        );
    }

    // View mode — top row stays ≤5 items:
    //   [primary action] · Edit Map · Mow · Manual · More
    // The emergency control is pulled out into the pinned STOP corner.
    // While RECORDING the toolbar suppresses its own primary action AND the
    // Finish/Cancel record buttons — the JoystickOverlay owns the single
    // Finish/Cancel/Home set so they aren't duplicated.
    return (
        <>
            <div style={toolbarStyle}>
                {/* Primary mission controls first, labelled (the owner could not
                    find an icon-only Home); editing tools sit at the end next to More. */}
                {!isRecording && isIdle && (
                    <AsyncButton
                        type="primary"
                        size="large"
                        icon={<PlayCircleOutlined />}
                        onAsyncClick={onStart!}
                        aria-label={t("mapToolbarMobile.start")}
                        style={touchTarget}
                    >
                        {t("mapToolbarMobile.start")}
                    </AsyncButton>
                )}
                {/* Dock (return to dock) whenever it is allowed, idle included: an
                    idle robot off the dock must be able to go back. */}
                {!isRecording && onHome && canHome(highLevelState, stateName) && (
                    <AsyncButton
                        type={isIdle ? "default" : "primary"}
                        size="large"
                        icon={<DockIcon />}
                        onAsyncClick={onDock}
                        aria-label={t("mapToolbarMobile.home")}
                        style={touchTarget}
                    >
                        {t("mapToolbarMobile.home")}
                    </AsyncButton>
                )}

                {!isRecording && onUndock && canUndock(stateName) && (
                    <AsyncButton
                        size="large"
                        icon={<LogoutOutlined />}
                        onAsyncClick={() => confirmUndock(modal, t, onUndock)}
                        aria-label={t("mapToolbarMobile.undock")}
                        style={touchTarget}
                    />
                )}

                <Dropdown
                    menu={{items: mowingAreas, onClick: handleMowClick}}
                    trigger={["click"]}
                    placement="topLeft"
                >
                    <Button
                        size="large"
                        icon={<ScissorOutlined />}
                        loading={mowLoading}
                        aria-label={t("mapToolbarMobile.mowArea")}
                        style={touchTarget}
                    >
                        {t("mapToolbarMobile.mow")}
                    </Button>
                </Dropdown>

                {onAreaSettings && settingsAreas && settingsAreas.length > 0 && (
                    <Dropdown
                        menu={{
                            items: settingsAreas.map(({key, label}) => ({key, label})),
                            onClick: ({key}: MenuInfo) => onAreaSettings(key),
                        }}
                        trigger={["click"]}
                        placement="topLeft"
                    >
                        <Button
                            size="large"
                            icon={<SettingOutlined />}
                            aria-label={t("mapToolbarMobile.areaSettings")}
                            style={touchTarget}
                        >
                            {t("mapToolbarMobile.areaSettingsShort")}
                        </Button>
                    </Dropdown>
                )}

                {onPreviewPlan && canPreviewPlan(stateName) && (
                    <Dropdown
                        menu={{
                            items: [
                                {key: PREVIEW_ALL_KEY, label: t("planPreview.allAreas")},
                                ...mowingAreas.map(({key, label}) => ({key, label})),
                            ],
                            onClick: handlePreviewClick,
                        }}
                        trigger={["click"]}
                        placement="topLeft"
                    >
                        <Button
                            size="large"
                            icon={<EyeOutlined />}
                            loading={previewLoading}
                            aria-label={t("planPreview.button")}
                            style={touchTarget}
                        />
                    </Dropdown>
                )}

                <AsyncButton
                    size="large"
                    danger={manualMode}
                    icon={manualMode ? <StopOutlined /> : <ControlOutlined />}
                    onAsyncClick={manualMode ? onStopManualMode : onManualMode}
                    aria-label={manualMode ? t("mapToolbarMobile.stopManualMowing") : t("mapToolbarMobile.manualMowing")}
                    style={touchTarget}
                />

                <Button
                    size="large"
                    icon={<EditOutlined />}
                    onClick={onEditMap}
                    aria-label={t("mapToolbarMobile.editMap")}
                    style={touchTarget}
                />

                <Dropdown
                    menu={{items: dataMenuItems, onClick: handleMoreClick}}
                    trigger={["click"]}
                    placement="topRight"
                >
                    <Button size="large" icon={<EllipsisOutlined />} aria-label={t("mapToolbarMobile.more")} style={touchTarget} />
                </Dropdown>
            </div>
            {stopButton}
        </>
    );
};
