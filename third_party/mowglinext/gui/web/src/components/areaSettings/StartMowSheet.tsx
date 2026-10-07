import React, {useEffect, useMemo, useState} from "react";
import {Alert, App, Button, Drawer, Modal, Radio, Select, Skeleton, Slider, Space, Typography} from "antd";
import {useTranslation} from "react-i18next";
import {useApi} from "../../hooks/useApi.ts";
import {useIsMobile} from "../../hooks/useIsMobile.ts";
import {useCoverageResumeAvailable} from "../../hooks/useCoverageResumeAvailable.ts";
import {type AreaSettingsPatch, type AreaSettingsTarget, saveAreaSettings, useAreaSettings} from "../../hooks/useAreaSettings.ts";
import {AREA_SETTINGS_RANGES, effectiveAreaSettings, overriddenKeys} from "../../utils/areaSettings.ts";
import type {MowingAreaChoice} from "../../utils/mapAreaIndex.ts";
import {AreaSettingsSummary} from "./AreaSettingsSummary.tsx";

export type StartSelection = "all" | number;

type Props = {
    open: boolean;
    onClose: () => void;
    areas: MowingAreaChoice[];
    initialSelection?: StartSelection;
    /** Pre-selected Resume / Start fresh choice ("Start fresh" entry points pass fresh). */
    initialMode?: "resume" | "fresh";
    /** Open the full editor for this target (closes the sheet first). */
    onEdit?: (target: AreaSettingsTarget) => void;
};

/**
 * Start sheet: choose all areas or one, review the effective settings, adjust
 * blade height / repeat for this run (persisted to the chosen area — or the
 * defaults for "All areas" — before START), pick Resume vs Start fresh, then
 * fire the existing START / start_in_area flow.
 */
export const StartMowSheet: React.FC<Props> = ({open, onClose, areas, initialSelection = "all", initialMode = "resume", onEdit}) => {
    const {t} = useTranslation();
    const api = useApi();
    const isMobile = useIsMobile();
    const {notification} = App.useApp();
    const resumeAvailable = useCoverageResumeAvailable();
    const [selection, setSelection] = useState<StartSelection>(initialSelection);
    const [resumeMode, setResumeMode] = useState<"resume" | "fresh">(initialMode);
    const target: AreaSettingsTarget = selection === "all" ? "defaults" : selection;
    const {loading, supported, defaults, effective: stored} = useAreaSettings(open ? target : null);
    const effective = useMemo(() => effectiveAreaSettings(defaults, stored), [defaults, stored]);
    const usesDefaults = useMemo(
        () => selection !== "all" && overriddenKeys(effective, effectiveAreaSettings(defaults)).size === 0,
        [selection, effective, defaults],
    );
    const [height, setHeight] = useState(effective.cutter_height_mm);
    const [repeat, setRepeat] = useState(effective.repeat);
    const [busy, setBusy] = useState(false);

    useEffect(() => {
        if (open) {
            setSelection(initialSelection);
            setResumeMode(initialMode);
        }
    }, [open, initialSelection, initialMode]);
    useEffect(() => {
        setHeight(effective.cutter_height_mm);
        setRepeat(effective.repeat);
    }, [effective]);

    const call = async (command: string, args: Record<string, unknown>) => {
        const res = await api.mowglinext.callCreate(command, args);
        if (res.error) throw new Error(res.error.error);
    };

    const confirm = async () => {
        setBusy(true);
        try {
            if (supported && (height !== effective.cutter_height_mm || repeat !== effective.repeat)) {
                // The map server merges: send only what this run changes.
                const patch: AreaSettingsPatch = {};
                if (height !== effective.cutter_height_mm) patch.cutter_height_mm = height;
                if (repeat !== effective.repeat) patch.repeat = repeat;
                await saveAreaSettings(api, target, patch);
            }
            if (resumeAvailable && resumeMode === "fresh") {
                await call("coverage_clear_resume", {});
            }
            if (selection === "all") {
                await call("high_level_control", {Command: 1});
            } else {
                await call("start_in_area", {area: selection});
            }
            onClose();
        } catch (e) {
            notification.error({
                message: t("startSheet.failed"),
                description: e instanceof Error ? e.message : undefined,
            });
        } finally {
            setBusy(false);
        }
    };

    const body = (
        <div data-testid="start-sheet">
            <Typography.Text strong>{t("startSheet.what")}</Typography.Text>
            <Radio.Group style={{display: "flex", flexDirection: "column", gap: 8, margin: "8px 0 16px"}}
                         value={selection === "all" ? "all" : "one"}
                         onChange={(e) => setSelection(e.target.value === "all" ? "all" : (areas[0]?.index ?? "all"))}>
                <Radio value="all" data-testid="start-all">{t("startSheet.allAreas")}</Radio>
                <Radio value="one" disabled={areas.length === 0} data-testid="start-one">{t("startSheet.oneArea")}</Radio>
            </Radio.Group>
            {selection !== "all" && (
                <Select style={{width: "100%", marginBottom: 16}} value={selection}
                        data-testid="start-area-select"
                        options={areas.map((a) => ({value: a.index, label: a.name}))}
                        onChange={(v: number) => setSelection(v)}/>
            )}

            <div style={{display: "flex", justifyContent: "space-between", alignItems: "baseline"}}>
                <Typography.Text strong>{t("startSheet.settings")}</Typography.Text>
                {onEdit && supported && (
                    <Button type="link" size="small" style={{padding: 0}}
                            onClick={() => {
                                onClose();
                                onEdit(target);
                            }}>{t("startSheet.edit")}</Button>
                )}
            </div>
            {loading ? <Skeleton active paragraph={{rows: 2}}/> : !supported ? (
                <Alert type="info" showIcon message={t("areaSettings.notSupported")} style={{margin: "8px 0 16px"}}/>
            ) : (
                <>
                    <AreaSettingsSummary settings={effective} usesDefaults={usesDefaults}/>
                    <div style={{marginTop: 16}}>
                        <Typography.Text>{t("startSheet.heightOverride")}</Typography.Text>
                        <span style={{float: "right", fontWeight: 600}}>{t("areaSettings.mm", {value: height})}</span>
                        <Slider min={AREA_SETTINGS_RANGES.cutter_height_mm.min} max={AREA_SETTINGS_RANGES.cutter_height_mm.max}
                                step={AREA_SETTINGS_RANGES.cutter_height_mm.step} value={height} onChange={setHeight}/>
                        <Typography.Text>{t("startSheet.repeatOverride")}</Typography.Text>
                        <span style={{float: "right", fontWeight: 600}}>{t("areaSettings.times", {count: repeat})}</span>
                        <Slider min={1} max={5} step={1} dots value={repeat} onChange={setRepeat}/>
                        {(height !== effective.cutter_height_mm || repeat !== effective.repeat) && (
                            <div style={{fontSize: 11, opacity: 0.65}}>
                                {selection === "all" ? t("startSheet.overrideSavedDefaults") : t("startSheet.overrideSavedArea")}
                            </div>
                        )}
                    </div>
                </>
            )}

            {resumeAvailable && (
                <div style={{marginTop: 16}}>
                    <Typography.Text strong>{t("startSheet.progress")}</Typography.Text>
                    <Radio.Group style={{display: "flex", flexDirection: "column", gap: 8, marginTop: 8}}
                                 value={resumeMode} onChange={(e) => setResumeMode(e.target.value)}>
                        <Radio value="resume" data-testid="start-resume">{t("mowerActions.resume")}</Radio>
                        <Radio value="fresh" data-testid="start-fresh">{t("mowerActions.startFresh")}</Radio>
                    </Radio.Group>
                </div>
            )}
        </div>
    );

    const footer = (
        <Space style={{width: "100%", justifyContent: "flex-end"}}>
            <Button onClick={onClose}>{t("startSheet.cancel")}</Button>
            <Button type="primary" loading={busy} onClick={confirm} data-testid="start-confirm">
                {resumeAvailable && resumeMode === "resume" ? t("mowerActions.resume") : t("startSheet.start")}
            </Button>
        </Space>
    );

    return isMobile ? (
        <Drawer open={open} onClose={onClose} placement="bottom" height="auto" title={t("startSheet.title")}
                footer={footer} styles={{body: {maxHeight: "70vh", overflowY: "auto"}}} destroyOnHidden>
            {body}
        </Drawer>
    ) : (
        <Modal open={open} onCancel={onClose} title={t("startSheet.title")} footer={footer} destroyOnHidden>
            {body}
        </Modal>
    );
};
