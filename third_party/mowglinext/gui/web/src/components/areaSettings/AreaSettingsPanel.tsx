import React, {useEffect, useMemo, useState} from "react";
import {Alert, App, Button, Skeleton, Switch, Typography} from "antd";
import {useTranslation} from "react-i18next";
import {useApi} from "../../hooks/useApi.ts";
import {
    AreaSettingsUnsupportedError,
    type AreaSettingsTarget,
    saveAreaSettings,
    useAreaSettings,
    useAreaSettingsTopic,
} from "../../hooks/useAreaSettings.ts";
import {
    type AreaSettings, buildAreaPatch, effectiveAreaSettings, invalidAreaSettingKey, overriddenKeys,
} from "../../utils/areaSettings.ts";
import {AreaSettingsForm} from "./AreaSettingsForm.tsx";

/**
 * Load / edit / save the mow settings of one area (or the robot defaults).
 * The map server merges what we send: for an area we send the keys that
 * differ from the defaults and null for the rest (so they keep following the
 * defaults); "Use defaults" sends {} which clears every override.
 */
export const AreaSettingsPanel: React.FC<{
    target: AreaSettingsTarget;
    /** Area name, to find its override list in the latched topic. */
    areaName?: string;
    onSaved?: () => void;
}> = ({target, areaName, onSaved}) => {
    const {t} = useTranslation();
    const api = useApi();
    const {notification} = App.useApp();
    const {loading, supported, effective, defaults, derived, error, reload} = useAreaSettings(target);
    const topic = useAreaSettingsTopic(target !== "defaults");
    const isDefaults = target === "defaults";
    const defaultsFull = useMemo(() => effectiveAreaSettings(defaults), [defaults]);
    const effectiveFull = useMemo(() => effectiveAreaSettings(defaults, effective), [defaults, effective]);
    const topicOverrides = areaName ? topic?.areas[areaName] : undefined;
    const overridden = useMemo(
        () => overriddenKeys(effectiveFull, defaultsFull, topicOverrides),
        [effectiveFull, defaultsFull, topicOverrides],
    );
    const [useDefaults, setUseDefaults] = useState(false);
    const [draft, setDraft] = useState<AreaSettings>(effectiveFull);
    const [saving, setSaving] = useState(false);
    const [dirty, setDirty] = useState(false);

    useEffect(() => {
        if (loading) return;
        setDraft(effectiveFull);
        setDirty(false);
    }, [loading, effectiveFull]);
    useEffect(() => {
        if (!loading && !dirty) setUseDefaults(!isDefaults && overridden.size === 0);
    }, [loading, dirty, isDefaults, overridden]);

    if (loading) return <Skeleton active paragraph={{rows: 6}}/>;
    if (!supported) {
        return <Alert type="info" showIcon message={t("areaSettings.notSupported")}
                      description={t("areaSettings.notSupportedHint")} data-testid="area-settings-unsupported"/>;
    }
    if (error) {
        return <Alert type="error" showIcon message={t("areaSettings.loadFailed")} description={error}
                      action={<Button size="small" onClick={reload}>{t("areaSettings.retry")}</Button>}/>;
    }

    const save = async () => {
        const bad = invalidAreaSettingKey(draft);
        if (!useDefaults && bad) {
            notification.error({message: t("areaSettings.invalid", {key: bad})});
            return;
        }
        setSaving(true);
        try {
            const body = isDefaults ? draft : useDefaults ? null : buildAreaPatch(draft, defaultsFull);
            await saveAreaSettings(api, target, body);
            notification.success({message: t("areaSettings.saved")});
            setDirty(false);
            reload();
            onSaved?.();
        } catch (e) {
            notification.error({
                message: e instanceof AreaSettingsUnsupportedError ? t("areaSettings.notSupported") : t("areaSettings.saveFailed"),
                description: e instanceof Error ? e.message : undefined,
            });
        } finally {
            setSaving(false);
        }
    };

    return (
        <div>
            {!isDefaults && (
                <div style={{display: "flex", alignItems: "center", gap: 10, marginBottom: 16}}>
                    <Switch checked={useDefaults} data-testid="use-defaults"
                            onChange={(on) => {
                                setUseDefaults(on);
                                if (on) setDraft(defaultsFull);
                                setDirty(true);
                            }}/>
                    <div>
                        <Typography.Text strong>{t("areaSettings.useDefaults")}</Typography.Text>
                        <div style={{fontSize: 11, opacity: 0.65}}>{t("areaSettings.useDefaultsHint")}</div>
                    </div>
                </div>
            )}
            <AreaSettingsForm value={draft} disabled={!isDefaults && useDefaults}
                              slopeDerived={isDefaults ? null : derived}
                              overridden={isDefaults || useDefaults ? undefined
                                  : dirty ? overriddenKeys(draft, defaultsFull) : overridden}
                              onChange={(next) => {
                                  setDraft(next);
                                  setDirty(true);
                              }}/>
            <Button type="primary" block loading={saving} disabled={!dirty} onClick={save} data-testid="area-settings-save">
                {t("areaSettings.save")}
            </Button>
        </div>
    );
};
