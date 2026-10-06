import React, {useState} from "react";
import {Button, Popover} from "antd";
import {QuestionCircleOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import {stateRenderer} from "../../../components/utils.tsx";
import {useThemeMode} from "../../../theme/ThemeContext.tsx";
import {useDetections} from "../../../hooks/useDetections.ts";
import {isLatchedFault} from "../../../utils/missionStates.ts";
import type {WhyStopped} from "../../../utils/missionProgress.ts";
import {WhyStoppedPanel} from "./WhyStoppedPanel.tsx";

/** States in which the robot is expected to be moving on its own. */
const MOVING = new Set(["MOWING", "TRANSIT", "UNDOCKING", "RETURNING_HOME", "LOW_BATTERY_DOCKING",
    "RAIN_DETECTED_DOCKING", "COVERAGE_FAILED_DOCKING", "PLANNING", "RECORDING", "MANUAL_MOWING"]);

/** Label of the explain button: "Why stopped?" unless the robot is driving. */
export const whyLabelKey = (stateName?: string, subState?: string): string =>
    stateName && MOVING.has(stateName) && !/wait|paus|block|obstacle|retry|stale/i.test(subState ?? "")
        ? "whyStopped.details" : "whyStopped.button";

interface Props {
    stateName?: string;
    subStateName?: string;
    why?: WhyStopped | null;
    /** Rendered under the state line (the phone's compact progress bar). */
    children?: React.ReactNode;
    style?: React.CSSProperties;
}

/**
 * Always-visible "STATE · sub_state" line (phone and desktop) with a
 * "Why stopped?" explainer. A latched fault is shown by MissionStopControls,
 * so the line then only keeps the explainer.
 */
export const MissionStatusLine: React.FC<Props> = ({stateName, subStateName, why, children, style}) => {
    const {t} = useTranslation();
    const {colors} = useThemeMode();
    const [open, setOpen] = useState(false);
    // Detections are unthrottled: subscribe only while the explainer is open.
    const det = useDetections(open);
    const sub = subStateName?.trim();
    const label = stateRenderer(stateName);
    const fault = isLatchedFault(stateName);
    return (
        <div data-testid="map-status-line" style={{
            display: 'flex', flexDirection: 'column', gap: 4, minWidth: 0, maxWidth: '100%',
            padding: '4px 6px 4px 12px', borderRadius: 14, fontSize: 12, lineHeight: '18px',
            background: colors.glassBackground, border: colors.glassBorder, color: colors.text,
            boxShadow: colors.glassShadow, pointerEvents: 'auto', ...style,
        }}>
            <div style={{display: 'flex', alignItems: 'center', gap: 6, minWidth: 0}}>
                {!fault && <span style={{fontWeight: 600, flexShrink: 0}}>{label}</span>}
                {!fault && sub && <span style={{color: colors.textSecondary, flexShrink: 0}}>·</span>}
                {!fault && sub && (
                    <span title={sub} style={{overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap', minWidth: 0, flex: 1}}>{sub}</span>
                )}
                <Popover
                    open={open}
                    onOpenChange={setOpen}
                    trigger="click"
                    placement="bottom"
                    title={t('whyStopped.title')}
                    content={<WhyStoppedPanel stateName={stateName} subState={subStateName} why={why}
                                              detections={det.data} detectionsAt={det.lastMessageAt}/>}
                >
                    <Button size="small" type="text" icon={<QuestionCircleOutlined/>}
                            style={{flexShrink: 0, marginLeft: 'auto', minHeight: 32}}
                            data-testid="why-stopped-button">
                        {t(whyLabelKey(stateName, subStateName))}
                    </Button>
                </Popover>
            </div>
            {children}
        </div>
    );
};
