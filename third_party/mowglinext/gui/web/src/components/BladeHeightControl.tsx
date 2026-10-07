import React, {useEffect, useState} from "react";
import {App, Button, Space, Typography} from "antd";
import {MinusOutlined, PlusOutlined, SaveOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import {ContentType} from "../api/Api.ts";
import {useApi} from "../hooks/useApi.ts";
import {useCutterHeight} from "../hooks/useCutterHeight.ts";
import {useEmergency} from "../hooks/useEmergency.ts";
import {saveAreaSettings, useActiveAreaSettings} from "../hooks/useAreaSettings.ts";
import {BLADE_HEIGHT_MAX_MM, BLADE_HEIGHT_MIN_MM, stepBladeHeight} from "../utils/bladeHeight.ts";

type HeightResult = {ok?: boolean; message?: string; height_mm?: number};
type HttpLike = {status?: number; error?: (HeightResult & {error?: string}) | null};

/**
 * Live blade height during a mow: -/+ 5 mm (30-90) via POST
 * /mowglinext/cutter/height (mcu_node /cutter/set_height: height only, blade
 * state untouched). The mission keeps the new height for the rest of this
 * mow; "Save to area" also stores it in the running area's settings.
 */
export const BladeHeightControl: React.FC<{style?: React.CSSProperties}> = ({style}) => {
    const {t} = useTranslation();
    const {message} = App.useApp();
    const api = useApi();
    const current = useCutterHeight();
    const emergency = useEmergency();
    const active = useActiveAreaSettings();
    const latched = !!(emergency.latched_emergency || emergency.active_emergency);
    const [busy, setBusy] = useState<string | null>(null);
    const [pending, setPending] = useState<number | null>(null);
    useEffect(() => { setPending(null); }, [current]);
    const shown = pending ?? current;

    const apply = async (dir: 1 | -1) => {
        if (shown == null) return;
        const target = stepBladeHeight(shown, dir);
        if (target === shown) return;
        setBusy(dir > 0 ? "up" : "down");
        setPending(target);
        try {
            const res = await api.request<HeightResult>({
                path: "/mowglinext/cutter/height", method: "POST", format: "json",
                type: ContentType.Json, body: {height_mm: target},
            });
            void message.success(res.data?.message || t('bladeHeight.applied', {mm: target}));
        } catch (e) {
            setPending(null);
            const h = e as HttpLike;
            void message.error(h?.error?.message || h?.error?.error ||
                (e instanceof Error ? e.message : t('bladeHeight.failed')));
        } finally {
            setBusy(null);
        }
    };

    const saveToArea = async () => {
        if (current == null || active?.areaIndex == null) return;
        setBusy("save");
        try {
            await saveAreaSettings(api, active.areaIndex, {cutter_height_mm: current});
            void message.success(t('bladeHeight.saved', {mm: current, area: active.area ?? `#${active.areaIndex}`}));
        } catch (e) {
            void message.error(e instanceof Error ? e.message : t('bladeHeight.failed'));
        } finally {
            setBusy(null);
        }
    };

    const savedHeight = active?.cutter_height_mm;
    const canSave = current != null && active?.areaIndex != null && !latched;
    return (
        <Space size={6} wrap align="center" style={style} data-testid="blade-height">
            <Typography.Text style={{fontSize: 13}}>{t('bladeHeight.label')}</Typography.Text>
            <Button size="small" icon={<MinusOutlined/>} aria-label={t('bladeHeight.lower')}
                    disabled={latched || shown == null || shown <= BLADE_HEIGHT_MIN_MM || busy !== null}
                    loading={busy === "down"} onClick={() => void apply(-1)} data-testid="blade-height-down"/>
            <Typography.Text strong style={{minWidth: 52, textAlign: "center", display: "inline-block"}}
                             data-testid="blade-height-value">
                {shown == null ? "—" : t('bladeHeight.mm', {mm: shown})}
            </Typography.Text>
            <Button size="small" icon={<PlusOutlined/>} aria-label={t('bladeHeight.raise')}
                    disabled={latched || shown == null || shown >= BLADE_HEIGHT_MAX_MM || busy !== null}
                    loading={busy === "up"} onClick={() => void apply(1)} data-testid="blade-height-up"/>
            {canSave && (
                <Button size="small" type="link" icon={<SaveOutlined/>} loading={busy === "save"}
                        disabled={busy !== null} onClick={() => void saveToArea()}
                        title={savedHeight != null ? t('bladeHeight.saveHint') : undefined}
                        data-testid="blade-height-save">
                    {t('bladeHeight.saveToArea')}
                </Button>
            )}
            {latched && <Typography.Text type="secondary" style={{fontSize: 12}}>{t('bladeHeight.emergency')}</Typography.Text>}
        </Space>
    );
};
