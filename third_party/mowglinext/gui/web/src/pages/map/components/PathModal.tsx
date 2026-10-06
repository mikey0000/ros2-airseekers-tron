import {Alert, Button, Card, Form, Input, Slider, Space, Switch, Tag} from "antd";
import {useTranslation} from "react-i18next";
import {MAX_PATH_WIDTH_M, MIN_PATH_WIDTH_M, type PathEndSnap} from "../../../utils/corridor.ts";

interface PathModalProps {
    open: boolean;
    /** Re-editing a saved path (title "Edit path", Delete offered). */
    editing?: boolean;
    name: string;
    width: number;
    /** How each end is connected (after snapping). */
    start: PathEndSnap;
    end: PathEndSnap;
    extendToDock: boolean;
    /** Buffered corridor is empty (degenerate polyline) — Save is disabled. */
    invalid?: boolean;
    onNameChange: (v: string) => void;
    onWidthChange: (v: number) => void;
    onExtendToDockChange: (v: boolean) => void;
    onSave: () => void;
    onCancel: () => void;
    onDelete?: () => void;
    /** Phone layout: full-width bottom sheet above the toolbar, no autofocus. */
    mobile?: boolean;
}

const EndTag = ({snap, label}: { snap: PathEndSnap; label: string }) => {
    const {t} = useTranslation();
    const color = snap === "none" ? "warning" : "success";
    return <Tag color={color}>{label}: {t(`mapPath.end.${snap}`)}</Tag>;
};

/**
 * Non-modal panel shown while a path is being edited: the line stays on the
 * map with draggable vertices and the band preview follows the vertices, the
 * width and the dock option live. Save adds (or replaces) the navigation area.
 */
export const PathModal = ({
    open, editing, name, width, start, end, extendToDock, invalid,
    onNameChange, onWidthChange, onExtendToDockChange, onSave, onCancel, onDelete, mobile,
}: PathModalProps) => {
    const {t} = useTranslation();
    if (!open) return null;
    const touchesDock = start === "dock" || end === "dock";
    const connected = start !== "none" && end !== "none";
    return (
        <Card
            size="small"
            title={editing ? t('mapPath.editTitle') : t('mapPath.title')}
            role="dialog"
            aria-label={editing ? t('mapPath.editTitle') : t('mapPath.title')}
            data-layout={mobile ? 'sheet' : 'panel'}
            style={mobile
                // Bottom sheet: fixed to the viewport (the map container bleeds
                // past it on phones), above the mobile toolbar (z 55/56) so
                // Save/Cancel stay reachable; capped so the line stays visible.
                ? {position: 'fixed', left: 0, right: 0, bottom: 0, zIndex: 60, width: 'auto',
                    maxHeight: '55vh', overflowY: 'auto', borderRadius: '16px 16px 0 0',
                    paddingBottom: 'env(safe-area-inset-bottom, 0px)'}
                : {position: 'absolute', top: 12, right: 12, zIndex: 20, width: 320, maxWidth: 'calc(100vw - 32px)'}}
        >
            <Form layout="vertical">
                <Form.Item label={t('mapPath.name')} style={{marginBottom: 8}}>
                    <Input
                        value={name}
                        onChange={(e) => onNameChange(e.target.value)}
                        onPressEnter={() => { if (!invalid) onSave(); }}
                        aria-label={t('mapPath.name')}
                        autoFocus={!mobile}
                    />
                </Form.Item>
                <Form.Item label={t('mapPath.width', {width: width.toFixed(1)})} style={{marginBottom: 8}}>
                    <Slider
                        min={MIN_PATH_WIDTH_M}
                        max={MAX_PATH_WIDTH_M}
                        step={0.1}
                        value={width}
                        onChange={onWidthChange}
                        tooltip={{formatter: (v) => `${(v ?? 0).toFixed(1)} m`}}
                        aria-label={t('mapPath.widthLabel')}
                    />
                </Form.Item>
                <Space size={4} wrap style={{marginBottom: 8}}>
                    <EndTag snap={start} label={t('mapPath.startLabel')}/>
                    <EndTag snap={end} label={t('mapPath.endLabel')}/>
                </Space>
                {touchesDock && (
                    <Form.Item label={t('mapPath.extendToDock')} extra={t('mapPath.extendToDockHint')} style={{marginBottom: 8}}>
                        <Switch checked={extendToDock} onChange={onExtendToDockChange}
                                aria-label={t('mapPath.extendToDock')}/>
                    </Form.Item>
                )}
                {!connected && !invalid && (
                    <Alert style={{marginBottom: 8}} type="warning" showIcon message={t('mapPath.notConnected')}/>
                )}
                {invalid && <Alert style={{marginBottom: 8}} type="warning" showIcon message={t('mapPath.invalid')}/>}
                <Alert type="info" showIcon message={t('mapPath.editHint')}/>
                <Space style={{marginTop: 12, width: '100%', justifyContent: 'flex-end'}} wrap>
                    {editing && onDelete && (
                        <Button danger onClick={onDelete}>{t('mapPath.delete')}</Button>
                    )}
                    <Button onClick={onCancel} style={mobile ? {minHeight: 44} : undefined}>{t('mapPath.cancel')}</Button>
                    <Button type="primary" onClick={onSave} disabled={invalid}
                            style={mobile ? {minHeight: 44} : undefined}>{t('mapPath.save')}</Button>
                </Space>
            </Form>
        </Card>
    );
};
