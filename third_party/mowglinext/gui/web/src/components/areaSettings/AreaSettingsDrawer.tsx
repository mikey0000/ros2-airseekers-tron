import React from "react";
import {Alert, Drawer} from "antd";
import {useTranslation} from "react-i18next";
import {useIsMobile} from "../../hooks/useIsMobile.ts";
import type {AreaSettingsTarget} from "../../hooks/useAreaSettings.ts";
import {AreaSettingsPanel} from "./AreaSettingsPanel.tsx";

/** "Mow settings" for one area: right-hand panel on desktop, bottom drawer on mobile. */
export const AreaSettingsDrawer: React.FC<{
    target: AreaSettingsTarget | null;
    title?: string;
    /** Area name as stored by the map server (topic key). */
    areaName?: string;
    /**
     * Shown instead of the settings form when the selected map area has no
     * mow settings (e.g. a navigation area / path, which is never mowed).
     */
    disabledHint?: string;
    onClose: () => void;
}> = ({target, title, areaName, disabledHint, onClose}) => {
    const {t} = useTranslation();
    const isMobile = useIsMobile();
    return (
        <Drawer
            open={target !== null || !!disabledHint}
            onClose={onClose}
            placement={isMobile ? "bottom" : "right"}
            width={isMobile ? undefined : 380}
            height={isMobile ? "85vh" : undefined}
            title={<div>
                <div>{t("areaSettings.title")}</div>
                {title && <div style={{fontSize: 12, fontWeight: 400, opacity: 0.65}}>{title}</div>}
            </div>}
            destroyOnHidden
        >
            {disabledHint ? (
                <div aria-disabled="true">
                    <Alert type="info" showIcon message={disabledHint}/>
                </div>
            ) : target !== null && <AreaSettingsPanel key={String(target)} target={target} areaName={areaName}/>}
        </Drawer>
    );
};
