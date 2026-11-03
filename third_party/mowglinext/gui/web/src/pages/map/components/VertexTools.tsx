import {Alert, Button, Card, InputNumber, Modal, Space, Typography} from "antd";
import {useTranslation} from "react-i18next";
import {TOLERANCE_MAX_M, TOLERANCE_MIN_M, type VertexTool, type VertexToolPreview} from "../hooks/vertexToolState.ts";

interface VertexActionsProps {
    vertexCount: number;
    onMerge: () => void;
    onSimplify: () => void;
}

/** Point count + "Merge nearby points" / "Simplify" buttons (panel body and boundary line sheet). */
export const VertexActions = ({vertexCount, onMerge, onSimplify, hideHint}: VertexActionsProps & {hideHint?: boolean}) => {
    const {t} = useTranslation();
    return (
        <div>
            <Typography.Text data-testid="vertex-count" style={{display: "block", marginBottom: 6}}>
                {t("mapVertex.points", {count: vertexCount})}
            </Typography.Text>
            <Space wrap size={6}>
                <Button size="small" onClick={onMerge} data-testid="vertex-merge-open" style={{minHeight: 36}}>
                    {t("mapVertex.mergeNearby")}
                </Button>
                <Button size="small" onClick={onSimplify} data-testid="vertex-simplify-open" style={{minHeight: 36}}>
                    {t("mapVertex.simplify")}
                </Button>
            </Space>
            {!hideHint && (
                <Typography.Text type="secondary" style={{display: "block", fontSize: 11, marginTop: 6}}>
                    {t("mapVertex.hint")}
                </Typography.Text>
            )}
        </div>
    );
};

interface VertexToolsPanelProps extends VertexActionsProps {
    mobile?: boolean;
}

/** Edit mode, one polygon selected: floating card with the vertex tools. */
export const VertexToolsPanel = ({mobile, ...actions}: VertexToolsPanelProps) => {
    const {t} = useTranslation();
    return (
        <Card
            size="small"
            title={t("mapVertex.panelTitle")}
            data-testid="vertex-panel"
            style={mobile
                // Phone: top-left, clear of the bottom edit toolbar and the top-right dock button.
                ? {position: "absolute", left: 12, top: 12, zIndex: 20, maxWidth: 240}
                : {position: "absolute", left: 76, bottom: 16, zIndex: 20, width: 260}}
            styles={{body: {padding: 8}}}
        >
            <VertexActions {...actions} hideHint={mobile}/>
        </Card>
    );
};

interface VertexToolDialogProps {
    tool: VertexTool | null;
    tolerance: number;
    preview: VertexToolPreview | null;
    onTolerance: (v: number | null) => void;
    onApply: () => void;
    onCancel: () => void;
}

/** Merge / Simplify dialog: tolerance, "N → M points", reject message. Non-modal so the ghost stays visible. */
export const VertexToolDialog = ({tool, tolerance, preview, onTolerance, onApply, onCancel}: VertexToolDialogProps) => {
    const {t} = useTranslation();
    const changes = !!preview?.ok && preview.after < preview.before;
    return (
        <Modal
            open={tool !== null}
            title={t(tool === "merge" ? "mapVertex.mergeTitle" : "mapVertex.simplifyTitle")}
            mask={false}
            style={{top: 12}}
            width={360}
            onCancel={onCancel}
            onOk={onApply}
            okText={t("mapVertex.apply")}
            cancelText={t("mapVertex.cancel")}
            okButtonProps={{disabled: !changes, "data-testid": "vertex-apply"} as React.ComponentProps<typeof Button>}
            destroyOnHidden
        >
            <Typography.Paragraph type="secondary" style={{fontSize: 12}}>
                {t(tool === "merge" ? "mapVertex.mergeHelp" : "mapVertex.simplifyHelp")}
            </Typography.Paragraph>
            <Space align="center" style={{marginBottom: 12}}>
                <span>{t(tool === "merge" ? "mapVertex.distance" : "mapVertex.tolerance")}</span>
                <InputNumber
                    min={TOLERANCE_MIN_M}
                    max={TOLERANCE_MAX_M}
                    step={0.01}
                    precision={2}
                    value={tolerance}
                    onChange={(v) => onTolerance(typeof v === "number" ? v : null)}
                    suffix="m"
                    inputMode="decimal"
                    data-testid="vertex-tolerance"
                    style={{width: 110}}
                />
            </Space>
            {preview && (preview.ok ? (
                <Typography.Text strong data-testid="vertex-preview-count" style={{display: "block"}}>
                    {changes
                        ? t("mapVertex.change", {before: preview.before, after: preview.after})
                        : t("mapVertex.noChange")}
                </Typography.Text>
            ) : (
                <Alert type="warning" showIcon data-testid="vertex-preview-error"
                       message={t(preview.reason === "tooFew" ? "mapVertex.tooFew" : "mapVertex.selfIntersection")}/>
            ))}
        </Modal>
    );
};
