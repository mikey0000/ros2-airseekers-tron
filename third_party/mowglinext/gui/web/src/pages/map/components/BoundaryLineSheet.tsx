import React, {useState} from "react";
import {Alert, Button, Form, Input, InputNumber, Popconfirm, Slider, Space} from "antd";
import {DeleteOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import {BOUNDARY_LINE_MAX_WIDTH_M, BOUNDARY_LINE_NAME_MAX} from "../../../types/map.ts";
import {MapItemDrawer} from "./MapItemDrawer.tsx";

export const BOUNDARY_LINE_WIDTH_STEP_M = 0.05;

export interface BoundaryLineSheetProps {
    open: boolean;
    /** Feature id of the line; the form resets when it changes. */
    lineKey: string;
    name: string;
    /** "Boundary line N", shown when the name is empty. */
    fallbackName: string;
    widthM: number;
    /** Commit the edited name / width (the sheet closes afterwards). */
    onApply: (name: string, widthM: number) => void;
    /** Called after the operator confirmed the delete. */
    onDelete: () => void;
    onClose: () => void;
}

const clampWidth = (w: number | null | undefined) =>
    Math.min(BOUNDARY_LINE_MAX_WIDTH_M, Math.max(0, Number.isFinite(w) ? Number(w) : 0));

const BoundaryLineForm: React.FC<Omit<BoundaryLineSheetProps, "open" | "onClose" | "lineKey">> = ({
    name, fallbackName, widthM, onApply, onDelete,
}) => {
    const {t} = useTranslation();
    const [draftName, setDraftName] = useState(name);
    const [draftWidth, setDraftWidth] = useState(clampWidth(widthM));
    const changed = draftName !== name || draftWidth !== clampWidth(widthM);
    return (
        <Form layout="vertical">
            <Form.Item label={t("mapBoundary.name")} style={{marginBottom: 8}}>
                <Input
                    value={draftName}
                    placeholder={fallbackName}
                    maxLength={BOUNDARY_LINE_NAME_MAX}
                    onChange={(e) => setDraftName(e.target.value)}
                    onPressEnter={() => onApply(draftName.trim(), draftWidth)}
                    aria-label={t("mapBoundary.name")}
                    data-testid="boundary-name"
                />
            </Form.Item>
            <Form.Item label={t("mapBoundary.width", {width: draftWidth.toFixed(2)})}
                       extra={t("mapBoundary.widthHint")} style={{marginBottom: 8}}>
                <div style={{display: "flex", width: "100%", alignItems: "center", gap: 12}}>
                    <Slider
                        style={{flex: 1}}
                        min={0}
                        max={BOUNDARY_LINE_MAX_WIDTH_M}
                        step={BOUNDARY_LINE_WIDTH_STEP_M}
                        value={draftWidth}
                        onChange={(v) => setDraftWidth(clampWidth(v))}
                        tooltip={{formatter: (v) => `${(v ?? 0).toFixed(2)} m`}}
                    />
                    <InputNumber
                        min={0}
                        max={BOUNDARY_LINE_MAX_WIDTH_M}
                        step={BOUNDARY_LINE_WIDTH_STEP_M}
                        precision={2}
                        value={draftWidth}
                        onChange={(v) => setDraftWidth(clampWidth(v))}
                        aria-label={t("mapBoundary.widthLabel")}
                        data-testid="boundary-width"
                        style={{width: 90}}
                    />
                </div>
            </Form.Item>
            <Alert type="info" showIcon message={t("mapBoundary.saveHint")} style={{marginBottom: 12}}/>
            <Space style={{width: "100%", justifyContent: "space-between"}} wrap>
                <Popconfirm
                    title={t("mapBoundary.deleteConfirmTitle")}
                    okText={t("mapBoundary.deleteConfirmOk")}
                    okButtonProps={{danger: true}}
                    cancelText={t("mapBoundary.deleteConfirmCancel")}
                    onConfirm={onDelete}
                >
                    <Button danger icon={<DeleteOutlined/>} data-testid="boundary-delete" style={{minHeight: 44}}>
                        {t("mapBoundary.delete")}
                    </Button>
                </Popconfirm>
                <Button type="primary" disabled={!changed} onClick={() => onApply(draftName.trim(), draftWidth)}
                        data-testid="boundary-apply" style={{minHeight: 44}}>
                    {t("mapBoundary.apply")}
                </Button>
            </Space>
        </Form>
    );
};

/** Sheet for a boundary line: name, keep-out width, delete. */
export const BoundaryLineSheet: React.FC<BoundaryLineSheetProps> = ({open, lineKey, onClose, ...form}) => {
    const {t} = useTranslation();
    return (
        <MapItemDrawer
            open={open}
            onClose={onClose}
            testId="boundary-sheet"
            title={<div>
                <div>{t("mapBoundary.sheetTitle")}</div>
                <div style={{fontSize: 12, fontWeight: 400, opacity: 0.65}}>{form.name || form.fallbackName}</div>
            </div>}
        >
            <BoundaryLineForm key={lineKey} {...form}/>
        </MapItemDrawer>
    );
};
