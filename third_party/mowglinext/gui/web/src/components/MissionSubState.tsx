import React from "react";
import {stateRenderer} from "./utils.tsx";
import {isLatchedFault} from "../utils/missionStates.ts";
import {useThemeMode} from "../theme/ThemeContext.tsx";

type Props = {
    stateName?: string;
    /** Free-text reason from the mission (why the robot is waiting/stopped). */
    subStateName?: string;
    style?: React.CSSProperties;
};

/**
 * Compact one-line "STATE · sub_state" pill. Renders nothing when sub_state is
 * empty or when a latched fault is shown by MissionStopControls instead.
 */
export const MissionSubState: React.FC<Props> = ({stateName, subStateName, style}) => {
    const {colors} = useThemeMode();
    const sub = subStateName?.trim();
    if (!sub || isLatchedFault(stateName)) return null;
    const label = stateRenderer(stateName);
    const full = `${label} · ${sub}`;
    return (
        <div data-testid="map-sub-state" title={full} style={{
            display: 'flex', alignItems: 'center', gap: 6, minWidth: 0, maxWidth: '100%',
            padding: '4px 10px', borderRadius: 999, fontSize: 12, lineHeight: '18px',
            background: colors.glassBackground, border: colors.glassBorder, color: colors.text,
            boxShadow: colors.glassShadow, pointerEvents: 'auto', ...style,
        }}>
            <span style={{fontWeight: 600, flexShrink: 0}}>{label}</span>
            <span style={{color: colors.textSecondary, flexShrink: 0}}>·</span>
            <span style={{overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap', minWidth: 0}}>{sub}</span>
        </div>
    );
};
