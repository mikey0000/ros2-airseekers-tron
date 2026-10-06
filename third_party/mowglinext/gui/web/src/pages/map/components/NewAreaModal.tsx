import {Form, Input, Modal, Select} from "antd";
import {useTranslation} from "react-i18next";

interface NewAreaModalProps {
    open: boolean;
    areaType: 'workarea' | 'navigation' | 'obstacle';
    areaName: string;
    onAreaTypeChange: (v: 'workarea' | 'navigation' | 'obstacle') => void;
    onAreaNameChange: (v: string) => void;
    onSave: () => void;
    onCancel: () => void;
}

export const NewAreaModal = ({open, areaType, areaName, onAreaTypeChange, onAreaNameChange, onSave, onCancel}: NewAreaModalProps) => {
    const {t} = useTranslation();
    return (
        <Modal
            open={open}
            title={t('mapNewArea.title')}
            okText={t('mapNewArea.addArea')}
            cancelText={t('mapNewArea.cancel')}
            onOk={onSave}
            onCancel={onCancel}
            destroyOnClose
        >
            <Form layout="vertical" style={{marginTop: 16}}>
                {/* Area types, available on every robot profile (no gating):
                    - workarea: a mowing area (map server working area).
                    - navigation: a drive-only corridor saved with
                      is_navigation_area=true; vendor apps call this a
                      "channel" / "path" between lawns.
                    - obstacle: a keep-out polygon, parented to the work area
                      that contains it (useMapEditing.addObstacle).
                    The dock is placed with the toolbar's dock tool. */}
                <Form.Item label={t('mapNewArea.areaType')}>
                    <Select
                        value={areaType}
                        onChange={onAreaTypeChange}
                        options={[
                            {value: 'workarea', label: t('mapNewArea.areaTypeWorking')},
                            {value: 'navigation', label: t('mapNewArea.areaTypeNavigation')},
                            {value: 'obstacle', label: t('mapNewArea.areaTypeObstacle')},
                        ]}
                    />
                </Form.Item>
                {areaType === 'workarea' && (
                    <Form.Item label={t('mapNewArea.areaNameOptional')}>
                        <Input
                            placeholder={t('mapNewArea.areaNamePlaceholder')}
                            value={areaName}
                            onChange={(e) => onAreaNameChange(e.target.value)}
                            onPressEnter={onSave}
                            autoFocus
                        />
                    </Form.Item>
                )}
            </Form>
        </Modal>
    );
};
