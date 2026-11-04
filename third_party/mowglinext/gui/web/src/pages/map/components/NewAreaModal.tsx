import {Button, Card, Form, Input, Select, Space} from "antd";
import {useTranslation} from "react-i18next";

type NewAreaType = 'workarea' | 'navigation' | 'obstacle';

interface NewAreaModalProps {
    open: boolean;
    areaType: NewAreaType;
    areaName: string;
    onAreaTypeChange: (v: NewAreaType) => void;
    onAreaNameChange: (v: string) => void;
    onSave: () => void;
    onCancel: () => void;
    /** Phone layout: full-width bottom sheet above the toolbar, no autofocus. */
    mobile?: boolean;
}

const NAME_PLACEHOLDER_KEYS: Record<NewAreaType, string> = {
    workarea: 'mapNewArea.areaNamePlaceholder',
    navigation: 'mapNewArea.navigationNamePlaceholder',
    obstacle: 'mapNewArea.noGoNamePlaceholder',
};

/**
 * Panel shown after a polygon is drawn: pick what it is and name it. A
 * non-modal card in a corner (bottom sheet on phones) like the path panel,
 * so the polygon just drawn stays visible (F12: a centred modal covered it).
 */
export const NewAreaModal = ({open, areaType, areaName, onAreaTypeChange, onAreaNameChange, onSave, onCancel, mobile}: NewAreaModalProps) => {
    const {t} = useTranslation();
    if (!open) return null;
    return (
        <Card
            size="small"
            title={t('mapNewArea.title')}
            role="dialog"
            aria-label={t('mapNewArea.title')}
            data-layout={mobile ? 'sheet' : 'panel'}
            style={mobile
                ? {position: 'fixed', left: 0, right: 0, bottom: 0, zIndex: 60, width: 'auto',
                    maxHeight: '55vh', overflowY: 'auto', borderRadius: '16px 16px 0 0',
                    paddingBottom: 'env(safe-area-inset-bottom, 0px)'}
                : {position: 'absolute', top: 12, right: 12, zIndex: 20, width: 320, maxWidth: 'calc(100vw - 32px)'}}
        >
            <Form layout="vertical">
                {/* Area types, available on every robot profile (no gating):
                    - workarea: a mowing area (map server working area).
                    - navigation: a drive-only corridor saved with
                      is_navigation_area=true; vendor apps call these
                      "channels". The toolbar's Path tool draws them as a
                      polyline buffered to a width (utils/corridor.ts),
                      optionally snapped to the dock approach.
                    - obstacle: a no-go zone polygon, parented to the work area
                      that contains it (useMapEditing.addObstacle).
                    Every type takes a name (F12: the no-go zone lost its name
                    field). The dock is placed with the toolbar's dock tool. */}
                <Form.Item label={t('mapNewArea.areaType')} style={{marginBottom: 8}}>
                    <Select
                        value={areaType}
                        onChange={onAreaTypeChange}
                        aria-label={t('mapNewArea.areaType')}
                        options={[
                            {value: 'workarea', label: t('mapNewArea.areaTypeWorking')},
                            {value: 'navigation', label: t('mapNewArea.areaTypeNavigation')},
                            {value: 'obstacle', label: t('mapNewArea.areaTypeObstacle')},
                        ]}
                    />
                </Form.Item>
                <Form.Item label={t('mapNewArea.nameOptional')} style={{marginBottom: 8}}>
                    <Input
                        placeholder={t(NAME_PLACEHOLDER_KEYS[areaType])}
                        value={areaName}
                        onChange={(e) => onAreaNameChange(e.target.value)}
                        onPressEnter={onSave}
                        aria-label={t('mapNewArea.nameOptional')}
                        autoFocus={!mobile}
                    />
                </Form.Item>
                <Space style={{width: '100%', justifyContent: 'flex-end'}} wrap>
                    <Button onClick={onCancel} style={mobile ? {minHeight: 44} : undefined}>{t('mapNewArea.cancel')}</Button>
                    <Button type="primary" onClick={onSave}
                            style={mobile ? {minHeight: 44} : undefined}>{t('mapNewArea.addArea')}</Button>
                </Space>
            </Form>
        </Card>
    );
};
