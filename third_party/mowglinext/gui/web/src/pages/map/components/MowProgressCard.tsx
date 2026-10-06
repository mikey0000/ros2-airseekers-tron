import React from "react";
import {Progress} from "antd";
import {useTranslation} from "react-i18next";
import {formatDuration, MissionProgress} from "../../../utils/missionProgress.ts";

export interface MowProgressColors {
    mowed: string;
    current: string;
    remaining: string;
    skipped: string;
    track: string;
}

interface Props {
    progress: MissionProgress;
    colors: MowProgressColors;
    /** Phone: one compact bar line instead of the card. */
    compact?: boolean;
    areaName?: string;
}

/** True when the progress describes a planned area worth showing. */
export const hasLiveProgress = (p: MissionProgress | null | undefined): p is MissionProgress =>
    !!p && p.area >= 0 && p.total_poses > 0;

const Swatch: React.FC<{color: string; label: string; dashed?: boolean}> = ({color, label, dashed}) => (
    <span style={{display: 'inline-flex', alignItems: 'center', gap: 4, whiteSpace: 'nowrap'}}>
        <span style={{width: 14, height: 0, borderTop: `3px ${dashed ? 'dotted' : 'solid'} ${color}`}}/>
        {label}
    </span>
);

/**
 * Live mow progress: percent, sub-path n/m, elapsed, ETA and length left
 * (desktop card), or a compact bar line (phone).
 */
export const MowProgressCard: React.FC<Props> = ({progress: p, colors, compact, areaName}) => {
    const {t} = useTranslation();
    const sub = p.sub_paths > 0 ? `${Math.min(p.sub_path + 1, p.sub_paths)}/${p.sub_paths}` : "–";
    const eta = formatDuration(p.eta_s);
    const left = `${Math.round(p.remaining_m)} m`;
    const pct = Math.round(p.percent);
    if (compact) {
        return (
            <div data-testid="mow-progress-bar" style={{display: 'flex', alignItems: 'center', gap: 8, fontSize: 12, minWidth: 0}}>
                <Progress percent={pct} size="small" showInfo={false} strokeColor={colors.mowed}
                          style={{flex: 1, margin: 0, minWidth: 60}}/>
                <span style={{whiteSpace: 'nowrap', fontWeight: 600}}>{pct} %</span>
                <span style={{whiteSpace: 'nowrap', overflow: 'hidden', textOverflow: 'ellipsis'}}>
                    {t('mowProgress.compact', {sub, eta, left})}
                </span>
            </div>
        );
    }
    const row = (label: string, value: string, testid?: string) => (
        <div style={{display: 'flex', justifyContent: 'space-between', gap: 12}}>
            <span style={{opacity: 0.75}}>{label}</span>
            <span style={{fontWeight: 600}} data-testid={testid}>{value}</span>
        </div>
    );
    return (
        <div data-testid="mow-progress-card" style={{fontSize: 12, minWidth: 220}}>
            <div style={{fontWeight: 600, marginBottom: 4}}>
                {areaName ? t('mowProgress.titleArea', {name: areaName}) : t('mowProgress.title')}
            </div>
            <Progress percent={pct} size="small" strokeColor={colors.mowed} style={{margin: '0 0 6px'}}/>
            {row(t('mowProgress.subPath'), sub, 'mow-progress-sub')}
            {row(t('mowProgress.elapsed'), formatDuration(p.elapsed_s))}
            {row(t('mowProgress.eta'), eta, 'mow-progress-eta')}
            {row(t('mowProgress.left'), left)}
            {p.skipped.length > 0 && row(t('mowProgress.skipped'), String(p.skipped.length))}
            <div style={{display: 'flex', flexWrap: 'wrap', gap: '4px 10px', marginTop: 6, opacity: 0.85}}>
                <Swatch color={colors.mowed} label={t('mowProgress.legendMowed')}/>
                <Swatch color={colors.current} label={t('mowProgress.legendCurrent')}/>
                <Swatch color={colors.remaining} label={t('mowProgress.legendRemaining')}/>
                <Swatch color={colors.skipped} label={t('mowProgress.legendSkipped')}/>
                <Swatch color={colors.track} label={t('mowProgress.legendTrack')} dashed/>
            </div>
        </div>
    );
};
