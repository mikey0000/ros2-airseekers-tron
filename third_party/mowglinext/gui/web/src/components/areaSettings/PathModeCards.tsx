import React from "react";
import {useTranslation} from "react-i18next";
import {useThemeMode} from "../../theme/ThemeContext.tsx";
import {PATH_MODES, type PathMode} from "../../utils/areaSettings.ts";

// Small schematic of each coverage pattern inside a rounded field.
const PATHS: Record<PathMode, React.ReactNode> = {
    zigzag: <path d="M8 8 H32 V14 H8 V20 H32 V26 H8 V32 H32"/>,
    cross: <>
        <path d="M8 9 H32 M8 16 H32 M8 23 H32 M8 30 H32"/>
        <path d="M11 6 V34 M18 6 V34 M25 6 V34 M32 6 V34" opacity="0.55"/>
    </>,
    alternate: <>
        <path d="M8 9 H32 M8 16 H32 M8 23 H32 M8 30 H32"/>
        <path d="M8 34 L20 6" opacity="0.4" strokeDasharray="2 2"/>
    </>,
    spiral: <path d="M20 20 h4 v-4 h-8 v8 h12 v-12 h-16 v16 h20 v-20 h-24"/>,
    contour_only: <>
        <rect x="6" y="6" width="28" height="28" rx="4"/>
        <rect x="11" y="11" width="18" height="18" rx="3" opacity="0.6"/>
    </>,
};

export const PathModeCards: React.FC<{
    value: PathMode;
    onChange: (v: PathMode) => void;
    disabled?: boolean;
}> = ({value, onChange, disabled}) => {
    const {t} = useTranslation();
    const {colors} = useThemeMode();
    return (
        <div role="radiogroup" aria-label={t("areaSettings.pathMode")}
             style={{display: "grid", gridTemplateColumns: "repeat(auto-fill, minmax(84px, 1fr))", gap: 8}}>
            {PATH_MODES.map((m) => {
                const active = m === value;
                return (
                    <button
                        key={m}
                        type="button"
                        role="radio"
                        aria-checked={active}
                        data-testid={`path-mode-${m}`}
                        disabled={disabled}
                        onClick={() => onChange(m)}
                        style={{
                            display: "flex", flexDirection: "column", alignItems: "center", gap: 4,
                            padding: "8px 4px", borderRadius: 12, cursor: disabled ? "not-allowed" : "pointer",
                            background: active ? colors.bgElevated : "transparent",
                            border: `1.5px solid ${active ? colors.primary : colors.borderSubtle}`,
                            color: active ? colors.primary : colors.text,
                            opacity: disabled ? 0.5 : 1,
                        }}
                    >
                        <svg width="40" height="40" viewBox="0 0 40 40" fill="none" stroke="currentColor"
                             strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden>
                            {PATHS[m]}
                        </svg>
                        <span style={{fontSize: 11, fontWeight: 600, textAlign: "center"}}>
                            {t(`areaSettings.pathModes.${m}`)}
                        </span>
                    </button>
                );
            })}
        </div>
    );
};
