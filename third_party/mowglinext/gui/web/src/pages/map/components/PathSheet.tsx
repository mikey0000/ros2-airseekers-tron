import React from "react";
import {Alert, Button, Descriptions, Space} from "antd";
import {EditOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import {MapItemDrawer} from "./MapItemDrawer.tsx";

export interface PathSheetProps {
    open: boolean;
    /** Path name, or the "Navigation area N" fallback label. */
    title: string;
    /** Band width (m) for a path drawn with the Path tool; null for a plain navigation polygon. */
    widthM: number | null;
    /** Centreline length (m) for a path; null for a plain navigation polygon. */
    lengthM: number | null;
    onClose: () => void;
    /** Open the editor on this path (only for paths with a centreline). */
    onEdit?: () => void;
}

/**
 * Sheet for a path / navigation area tapped on the map (F14: it opened the
 * "Mow settings" drawer, which then said the path has no settings). Paths are
 * drive-only, so the sheet shows what the path is and offers Edit path.
 */
export const PathSheet: React.FC<PathSheetProps> = ({open, title, widthM, lengthM, onClose, onEdit}) => {
    const {t} = useTranslation();
    return (
        <MapItemDrawer
            open={open}
            onClose={onClose}
            testId="path-sheet"
            title={<div>
                <div>{t("mapPath.sheetTitle")}</div>
                <div style={{fontSize: 12, fontWeight: 400, opacity: 0.65}} data-testid="path-name">{title}</div>
            </div>}
        >
            {(widthM !== null || lengthM !== null) && (
                <Descriptions column={1} size="small" style={{marginBottom: 12}}>
                    {widthM !== null && (
                        <Descriptions.Item label={t("mapPath.widthShort")}>
                            <span data-testid="path-width">{t("mapPath.metres", {value: widthM.toFixed(1)})}</span>
                        </Descriptions.Item>
                    )}
                    {lengthM !== null && (
                        <Descriptions.Item label={t("mapPath.lengthLabel")}>
                            <span data-testid="path-length">{t("mapPath.metres", {value: lengthM.toFixed(1)})}</span>
                        </Descriptions.Item>
                    )}
                </Descriptions>
            )}
            <Alert type="info" showIcon message={t("mapPath.sheetHint")} style={{marginBottom: 12}}/>
            {onEdit && (
                <Space direction="vertical" style={{width: "100%"}}>
                    <Button block icon={<EditOutlined/>} onClick={onEdit} data-testid="path-edit" style={{minHeight: 44}}>
                        {t("mapPath.editTitle")}
                    </Button>
                </Space>
            )}
        </MapItemDrawer>
    );
};
