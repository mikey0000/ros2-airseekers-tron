import React from "react";
import {Card, Typography} from "antd";
import {useTranslation} from "react-i18next";
import {useGate} from "../../hooks/useProfileGates.ts";
import {AreaSettingsPanel} from "./AreaSettingsPanel.tsx";

/** Settings -> Mowing: robot-wide defaults (index 255) every area inherits. */
export const AreaDefaultsCard: React.FC = () => {
    const {t} = useTranslation();
    const enabled = useGate("feature:area_settings");
    if (!enabled) return null;
    return (
        <Card title={t("areaSettings.defaultsTitle")} size="small" style={{marginTop: 16}} data-testid="area-defaults-card">
            <Typography.Paragraph type="secondary" style={{fontSize: 12}}>
                {t("areaSettings.defaultsHint")}
            </Typography.Paragraph>
            <AreaSettingsPanel target="defaults"/>
        </Card>
    );
};
