import {Alert, Form, Input, Modal, Slider, Switch} from "antd";
import {useTranslation} from "react-i18next";
import {MAX_PATH_WIDTH_M, MIN_PATH_WIDTH_M} from "../../../utils/corridor.ts";

interface PathModalProps {
    open: boolean;
    name: string;
    width: number;
    /** Dock pose is known, so "snap end to dock" can be offered. */
    dockAvailable: boolean;
    snapToDock: boolean;
    /** Buffered corridor is empty (degenerate polyline) — Save is disabled. */
    invalid?: boolean;
    onNameChange: (v: string) => void;
    onWidthChange: (v: number) => void;
    onSnapToDockChange: (v: boolean) => void;
    onSave: () => void;
    onCancel: () => void;
}

/**
 * Dialog shown after a path polyline is finished. The corridor preview on
 * the map follows the width / snap controls live; Save adds it as a
 * navigation area (drive-only, never mowed) to the unsaved map edit.
 */
export const PathModal = ({
    open, name, width, dockAvailable, snapToDock, invalid,
    onNameChange, onWidthChange, onSnapToDockChange, onSave, onCancel,
}: PathModalProps) => {
    const {t} = useTranslation();
    return (
        <Modal
            open={open}
            title={t('mapPath.title')}
            okText={t('mapPath.save')}
            cancelText={t('mapPath.cancel')}
            onOk={onSave}
            okButtonProps={{disabled: invalid}}
            onCancel={onCancel}
            mask={false}
            destroyOnClose
        >
            <Form layout="vertical" style={{marginTop: 16}}>
                <Form.Item label={t('mapPath.name')}>
                    <Input
                        value={name}
                        onChange={(e) => onNameChange(e.target.value)}
                        onPressEnter={() => { if (!invalid) onSave(); }}
                        aria-label={t('mapPath.name')}
                        autoFocus
                    />
                </Form.Item>
                <Form.Item label={t('mapPath.width', {width: width.toFixed(1)})}>
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
                {dockAvailable && (
                    <Form.Item label={t('mapPath.snapToDock')} extra={t('mapPath.snapToDockHint')}>
                        <Switch checked={snapToDock} onChange={onSnapToDockChange}
                                aria-label={t('mapPath.snapToDock')}/>
                    </Form.Item>
                )}
                <Alert type="info" showIcon message={t('mapPath.navigationHint')}/>
                {invalid && <Alert style={{marginTop: 8}} type="warning" showIcon message={t('mapPath.invalid')}/>}
            </Form>
        </Modal>
    );
};
