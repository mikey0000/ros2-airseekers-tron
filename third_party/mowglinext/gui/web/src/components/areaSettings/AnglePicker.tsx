import React, {useRef} from "react";
import {useTranslation} from "react-i18next";
import {useThemeMode} from "../../theme/ThemeContext.tsx";
import {angleFromPoint, normaliseAngle} from "../../utils/areaSettings.ts";

const SIZE = 132;
const C = SIZE / 2;
const R = 52;

/**
 * Compass-style picker for the swath direction (0° = north, clockwise). The
 * line is drawn through the centre because a swath at θ and θ+180 is the same
 * set of stripes. Click or drag to set; arrow keys nudge by 5°.
 */
export const AnglePicker: React.FC<{
    value: number;
    onChange: (deg: number) => void;
    disabled?: boolean;
}> = ({value, onChange, disabled}) => {
    const {t} = useTranslation();
    const {colors} = useThemeMode();
    const ref = useRef<SVGSVGElement>(null);
    const dragging = useRef(false);

    const setFromEvent = (e: React.PointerEvent) => {
        const box = ref.current?.getBoundingClientRect();
        if (!box) return;
        const dx = e.clientX - (box.left + box.width / 2);
        const dy = e.clientY - (box.top + box.height / 2);
        if (dx === 0 && dy === 0) return;
        onChange(angleFromPoint(dx, dy));
    };

    const rad = (value * Math.PI) / 180;
    const x = Math.sin(rad) * R;
    const y = -Math.cos(rad) * R;

    return (
        <svg
            ref={ref}
            width={SIZE} height={SIZE} viewBox={`0 0 ${SIZE} ${SIZE}`}
            role="slider" tabIndex={disabled ? -1 : 0}
            aria-label={t("areaSettings.mowAngle")}
            aria-valuemin={0} aria-valuemax={359} aria-valuenow={value}
            aria-disabled={disabled}
            style={{touchAction: "none", cursor: disabled ? "not-allowed" : "pointer", opacity: disabled ? 0.45 : 1}}
            onPointerDown={(e) => {
                if (disabled) return;
                dragging.current = true;
                (e.target as Element).setPointerCapture?.(e.pointerId);
                setFromEvent(e);
            }}
            onPointerMove={(e) => dragging.current && !disabled && setFromEvent(e)}
            onPointerUp={() => (dragging.current = false)}
            onKeyDown={(e) => {
                if (disabled) return;
                if (e.key === "ArrowRight" || e.key === "ArrowUp") onChange(normaliseAngle(value + 5));
                if (e.key === "ArrowLeft" || e.key === "ArrowDown") onChange(normaliseAngle(value - 5));
            }}
        >
            <circle cx={C} cy={C} r={R + 8} fill={colors.bgSubtle} stroke={colors.borderSubtle}/>
            {Array.from({length: 24}, (_, i) => {
                const a = (i * 15 * Math.PI) / 180;
                const long = i % 6 === 0;
                const r1 = R + 8 - (long ? 9 : 4);
                return <line key={i}
                             x1={C + Math.sin(a) * r1} y1={C - Math.cos(a) * r1}
                             x2={C + Math.sin(a) * (R + 7)} y2={C - Math.cos(a) * (R + 7)}
                             stroke={colors.muted} strokeWidth={long ? 1.5 : 1}/>;
            })}
            {(["N", "E", "S", "W"] as const).map((l, i) => {
                const a = (i * 90 * Math.PI) / 180;
                return <text key={l} x={C + Math.sin(a) * (R - 10)} y={C - Math.cos(a) * (R - 10) + 4}
                             textAnchor="middle" fontSize="10" fontWeight={700} fill={colors.muted}>{l}</text>;
            })}
            <line x1={C - x} y1={C - y} x2={C + x} y2={C + y}
                  stroke={colors.primary} strokeWidth={3} strokeLinecap="round" opacity={0.35}/>
            <line x1={C} y1={C} x2={C + x} y2={C + y}
                  stroke={colors.primary} strokeWidth={3} strokeLinecap="round"/>
            <circle cx={C + x} cy={C + y} r={6} fill={colors.primary}/>
            <circle cx={C} cy={C} r={3} fill={colors.text}/>
        </svg>
    );
};
