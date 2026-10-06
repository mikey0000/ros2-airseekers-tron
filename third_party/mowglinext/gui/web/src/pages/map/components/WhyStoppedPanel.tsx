import React from "react";
import {useTranslation} from "react-i18next";
import {stateRenderer} from "../../../components/utils.tsx";
import {ageSeconds, WhyStopped} from "../../../utils/missionProgress.ts";
import type {DetectionSummary} from "../../../hooks/useDetections.ts";

interface Props {
    stateName?: string;
    subState?: string;
    why?: WhyStopped | null;
    detections?: DetectionSummary | null;
    /** Date.now() of the last detection message, if any. */
    detectionsAt?: number | null;
    now?: number;
}

/** Explain why the robot is stopped / waiting: mission reason, last obstacle
 *  policy, the camera that saw it, costmap and safety inputs. */
export const WhyStoppedPanel: React.FC<Props> = ({stateName, subState, why, detections, detectionsAt, now = Date.now()}) => {
    const {t} = useTranslation();
    const lines: {label: string; value: string}[] = [];
    lines.push({label: t('whyStopped.state'), value: stateRenderer(stateName)});
    lines.push({label: t('whyStopped.reason'), value: subState?.trim() || t('whyStopped.noReason')});
    const ob = why?.obstacle;
    if (ob && ob.kind && ob.kind !== 'none') {
        const age = ageSeconds(ob.wall_time, now);
        const parts = [ob.class || t('whyStopped.unknownClass'), t(`whyStopped.kind_${ob.kind}`, {defaultValue: ob.kind})];
        if (typeof ob.distance_m === 'number') parts.push(`${ob.distance_m.toFixed(1)} m`);
        if (typeof ob.bearing_deg === 'number') parts.push(`${Math.round(ob.bearing_deg)}°`);
        if (age !== null) parts.push(t('whyStopped.ago', {s: Math.round(age)}));
        lines.push({label: t('whyStopped.obstacle'), value: parts.join(' · ')});
    } else {
        lines.push({label: t('whyStopped.obstacle'), value: t('whyStopped.none')});
    }
    if (detections && detections.count > 0) {
        const age = detectionsAt ? Math.round((now - detectionsAt) / 1000) : null;
        lines.push({
            label: t('whyStopped.camera'),
            value: `${detections.frame_id || '?'}: ${detections.classes.join(', ')}${age !== null ? ' · ' + t('whyStopped.ago', {s: age}) : ''}`,
        });
    } else if (detections) {
        lines.push({label: t('whyStopped.camera'), value: t('whyStopped.nothingSeen')});
    }
    if (typeof why?.stereo_stale_s === 'number') {
        lines.push({label: t('whyStopped.costmap'), value: t('whyStopped.stereoStale', {s: Math.round(why.stereo_stale_s)})});
    }
    const flags: string[] = [];
    if (why?.emergency) flags.push(t('whyStopped.flagEmergency'));
    if (why?.lift) flags.push(t('whyStopped.flagLift'));
    if (why?.stop_button) flags.push(t('whyStopped.flagStopButton'));
    if (why?.boundary_violation) flags.push(t('whyStopped.flagBoundary'));
    if (why?.rain) flags.push(t('whyStopped.flagRain'));
    if (why?.critical_nodes_down) flags.push(t('whyStopped.flagNodes', {nodes: why.critical_nodes_down}));
    lines.push({label: t('whyStopped.safety'), value: flags.length ? flags.join(' · ') : t('whyStopped.allClear')});
    return (
        <div data-testid="why-stopped" style={{fontSize: 12, maxWidth: 320, display: 'grid', gridTemplateColumns: 'auto 1fr', gap: '4px 10px'}}>
            {lines.map((l) => (
                <React.Fragment key={l.label}>
                    <span style={{opacity: 0.7}}>{l.label}</span>
                    <span style={{fontWeight: 500, wordBreak: 'break-word'}}>{l.value}</span>
                </React.Fragment>
            ))}
        </div>
    );
};
