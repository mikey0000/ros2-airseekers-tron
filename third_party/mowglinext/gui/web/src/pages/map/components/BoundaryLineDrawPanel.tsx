import React from "react";
import {Button, Card, Space, Typography} from "antd";
import {CheckOutlined, CloseOutlined, UndoOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";

interface BoundaryLineDrawPanelProps {
    open: boolean;
    pointCount: number;
    canFinish: boolean;
    onUndo: () => void;
    onFinish: () => void;
    onCancel: () => void;
    /** Phone layout: full-width bottom sheet above the toolbar. */
    mobile?: boolean;
}

/** Floating controls while a boundary line is being drawn: Undo point / Cancel / Finish. */
export const BoundaryLineDrawPanel: React.FC<BoundaryLineDrawPanelProps> = ({
    open, pointCount, canFinish, onUndo, onFinish, onCancel, mobile,
}) => {
    const {t} = useTranslation();
    if (!open) return null;
    const touch = mobile ? {minHeight: 44} : undefined;
    return (
        <Card
            size="small"
            title={t("mapBoundary.drawTitle")}
            role="dialog"
            aria-label={t("mapBoundary.drawTitle")}
            data-testid="boundary-draw-panel"
            style={mobile
                ? {position: 'fixed', left: 0, right: 0, bottom: 0, zIndex: 60, width: 'auto',
                    borderRadius: '16px 16px 0 0', paddingBottom: 'env(safe-area-inset-bottom, 0px)'}
                : {position: 'absolute', top: 12, right: 272, zIndex: 20, width: 320, maxWidth: 'calc(100vw - 32px)'}}
        >
            <Typography.Paragraph style={{marginBottom: 8}}>
                {t(mobile ? "mapBoundary.drawHintTouch" : "mapBoundary.drawHint")}
            </Typography.Paragraph>
            <Typography.Text type="secondary" data-testid="boundary-point-count">
                {t("mapBoundary.points", {count: pointCount})}
            </Typography.Text>
            <Space style={{marginTop: 12, width: '100%', justifyContent: 'flex-end'}} wrap>
                <Button icon={<UndoOutlined/>} onClick={onUndo} disabled={pointCount === 0}
                        data-testid="boundary-undo" style={touch}>{t("mapBoundary.undoPoint")}</Button>
                <Button icon={<CloseOutlined/>} onClick={onCancel} data-testid="boundary-cancel"
                        style={touch}>{t("mapBoundary.cancel")}</Button>
                <Button type="primary" icon={<CheckOutlined/>} onClick={onFinish} disabled={!canFinish}
                        data-testid="boundary-finish" style={touch}>{t("mapBoundary.finish")}</Button>
            </Space>
        </Card>
    );
};
