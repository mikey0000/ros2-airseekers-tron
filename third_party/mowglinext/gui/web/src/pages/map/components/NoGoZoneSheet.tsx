import React from "react";
import {Alert, Button, Descriptions, Popconfirm, Space, Tag} from "antd";
import {DeleteOutlined, EditOutlined, PlayCircleOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import {OBSTACLE_SOURCE_DIG, OBSTACLE_SOURCE_TRACKER, OBSTACLE_SOURCE_USER} from "../../../types/map.ts";
import {MapItemDrawer} from "./MapItemDrawer.tsx";

export interface NoGoZoneSheetProps {
    open: boolean;
    /** Obstacle name, or the "Obstacle N" fallback label. */
    title: string;
    areaM2: number;
    /** MapObstacleInfo.source (OBSTACLE_SOURCE_*). */
    source: number;
    pending: boolean;
    /** Label of the mowing area the zone belongs to. */
    parentName?: string;
    onClose: () => void;
    /** Called after the operator confirmed the delete. */
    onDelete: () => void;
    onEditVertices: () => void;
    /** Open the containing area's mow sheet (only when area settings are available). */
    onMowArea?: () => void;
}

/** Sheet for a no-go zone clicked on the map: details, delete, edit vertices, mow the containing area. */
export const NoGoZoneSheet: React.FC<NoGoZoneSheetProps> = ({
    open, title, areaM2, source, pending, parentName, onClose, onDelete, onEditVertices, onMowArea,
}) => {
    const {t} = useTranslation();
    const sourceLabel = source === OBSTACLE_SOURCE_TRACKER ? t("mapNoGo.sourceTracker")
        : source === OBSTACLE_SOURCE_DIG ? t("mapNoGo.sourceDig") : t("mapNoGo.sourceUser");
    return (
        <MapItemDrawer
            open={open}
            onClose={onClose}
            testId="nogo-sheet"
            title={<div>
                <div>{t("mapNoGo.title")}</div>
                <div style={{fontSize: 12, fontWeight: 400, opacity: 0.65}} data-testid="nogo-name">{title}</div>
            </div>}
        >
            <Descriptions column={1} size="small" style={{marginBottom: 12}}>
                <Descriptions.Item label={t("mapNoGo.area")}>
                    <span data-testid="nogo-area">{t("mapNoGo.areaValue", {area: areaM2.toFixed(1)})}</span>
                </Descriptions.Item>
                {parentName && (
                    <Descriptions.Item label={t("mapNoGo.inArea")}>{parentName}</Descriptions.Item>
                )}
                {(source !== OBSTACLE_SOURCE_USER || pending) && (
                    <Descriptions.Item label={t("mapNoGo.source")}>
                        <Space size={4} wrap>
                            <Tag>{sourceLabel}</Tag>
                            {pending && <Tag color="warning">{t("mapNoGo.pending")}</Tag>}
                        </Space>
                    </Descriptions.Item>
                )}
            </Descriptions>
            <Alert type="info" showIcon message={t("mapNoGo.hint")} style={{marginBottom: 12}}/>
            <Space direction="vertical" style={{width: "100%"}}>
                <Button block icon={<EditOutlined/>} onClick={onEditVertices} data-testid="nogo-edit-vertices"
                        style={{minHeight: 44}}>
                    {t("mapNoGo.editVertices")}
                </Button>
                <Popconfirm
                    title={t("mapNoGo.deleteConfirmTitle")}
                    description={t("mapNoGo.deleteConfirmBody")}
                    okText={t("mapNoGo.deleteConfirmOk")}
                    okButtonProps={{danger: true}}
                    cancelText={t("mapNoGo.deleteConfirmCancel")}
                    onConfirm={onDelete}
                >
                    <Button block danger icon={<DeleteOutlined/>} data-testid="nogo-delete" style={{minHeight: 44}}>
                        {t("mapNoGo.delete")}
                    </Button>
                </Popconfirm>
                {onMowArea && parentName && (
                    <Button type="link" block icon={<PlayCircleOutlined/>} onClick={onMowArea}
                            data-testid="nogo-mow-area">
                        {t("mapNoGo.mowArea", {name: parentName})}
                    </Button>
                )}
            </Space>
        </MapItemDrawer>
    );
};
