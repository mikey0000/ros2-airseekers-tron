import {Joystick} from "react-joystick-component";
import {IJoystickUpdateEvent} from "react-joystick-component/build/lib/Joystick";
import {CheckOutlined, CloseOutlined} from "@ant-design/icons";
import {DockIcon} from "../../../components/DockIcon.tsx";
import {useTranslation} from "react-i18next";
import AsyncButton from "../../../components/AsyncButton.tsx";
import {useThemeMode} from "../../../theme/ThemeContext.tsx";
import {limeAlpha} from "../../../theme/colors.ts";

interface JoystickOverlayProps {
    visible: boolean;
    isRecording?: boolean;
    /** What is being recorded (shown above Finish / Cancel). */
    recordKind?: "area" | "path" | "obstacle" | null;
    mobile?: boolean;
    onMove: (event: IJoystickUpdateEvent) => void;
    onStop: () => void;
    onFinishRecording?: () => Promise<void>;
    onCancelRecording?: () => Promise<void>;
    onHome?: () => Promise<void>;
    /** Extra controls beside the stick (two-step manual: the Blades control). */
    sideControls?: React.ReactNode;
    /** Joystick socket is down and reconnecting: stick input is dropped, show it. */
    linkDown?: boolean;
}

export const JoystickOverlay = ({
    visible, isRecording, recordKind, mobile,
    onMove, onStop, onFinishRecording, onCancelRecording, onHome, sideControls, linkDown,
}: JoystickOverlayProps) => {
    const {colors, displayMode} = useThemeMode();
    const {t} = useTranslation();
    if (!visible) return null;

    const size = mobile ? 132 : 110;

    // The default react-joystick-component look is a flat grey puck. We dress
    // it up with a glassy base ring + a glowing brand-green stick, sized up for
    // thumbs. Tokenised from the palette (lime/mint/emerald + deep bg).
    const baseColor = `radial-gradient(circle at 50% 40%, ${limeAlpha(0.10)}, ${colors.bgBase} 70%)`;
    const stickColor = `radial-gradient(circle at 38% 32%, ${colors.primaryLight}, ${colors.mint} 55%, ${colors.emeraldDeep})`;

    // Mobile: bottom-left (natural left thumb), lifted clear of the centred
    // toolbar + bottom-nav. The toolbar pins itself at safe-area + 100px and is
    // ~60px tall, so the control stack starts above that band rather than at a
    // magic offset tuned by hand. Desktop: bottom-right corner.
    const TOOLBAR_BAND_PX = 100 + 60 + 12; // toolbar bottom + height + gap
    const anchor: React.CSSProperties = mobile
        ? {left: 18, bottom: `calc(env(safe-area-inset-bottom, 0px) + ${TOOLBAR_BAND_PX}px)`}
        : {right: 30, bottom: 30};

    // A single flex "control stack" so the Finish/Cancel/Home buttons derive
    // their position from the joystick row instead of their own magic offsets.
    const actionButtonStyle: React.CSSProperties = {height: 44, minWidth: 44, borderRadius: 10, fontWeight: 600};

    return (
        <div style={{
            // Fixed to the viewport (not the bleeding map container) so it lands
            // reliably above the nav on iOS Safari — same reason as the toolbar.
            position: "fixed",
            ...anchor,
            zIndex: 60,
            display: "flex",
            alignItems: "flex-end",
            gap: 12,
            // Don't let dragging the stick pan/scroll the map underneath.
            touchAction: "none",
        }}>
            <div style={{
                position: "relative",
                padding: 6,
                borderRadius: "50%",
                background: colors.glassBackground,
                border: `1px solid ${limeAlpha(0.28)}`,
                boxShadow: `0 10px 30px -10px rgba(0,0,0,0.6), inset 0 0 24px ${limeAlpha(0.06)}`,
                backdropFilter: displayMode === 'visual' ? 'blur(8px)' : undefined,
                WebkitBackdropFilter: displayMode === 'visual' ? 'blur(8px)' : undefined,
                opacity: linkDown ? 0.55 : 1,
            }}>
                {linkDown && (
                    <div role="status" aria-live="polite" data-testid="joystick-reconnecting" style={{
                        position: "absolute", bottom: "100%", left: "50%", transform: "translate(-50%, -6px)",
                        whiteSpace: "nowrap", padding: "3px 10px", borderRadius: 999,
                        background: "rgba(250, 173, 20, 0.95)", color: "#1f1f1f",
                        fontSize: 12, fontWeight: 600, pointerEvents: "none",
                    }}>
                        {t('connection.joystickReconnecting', 'Joystick reconnecting…')}
                    </div>
                )}
                <Joystick
                    size={size}
                    baseColor={baseColor}
                    stickColor={stickColor}
                    stickSize={Math.round(size * 0.42)}
                    move={onMove}
                    stop={onStop}
                    throttle={50}
                />
            </div>

            {!isRecording && sideControls}

            {isRecording && (
                <div style={{display: "flex", flexDirection: "column", gap: 8, marginBottom: 4}}>
                    <div data-testid="recording-kind" style={{
                        alignSelf: "flex-start", padding: "3px 10px", borderRadius: 999, fontSize: 12,
                        fontWeight: 600, background: colors.glassBackground, color: colors.text,
                        border: `1px solid ${limeAlpha(0.28)}`,
                    }}>
                        {recordKind === "path" ? t('mapJoystick.recordingPath')
                            : recordKind === "obstacle" ? t('mapJoystick.recordingObstacle')
                                : t('mapJoystick.recordingArea')}
                    </div>
                    <AsyncButton
                        type="primary"
                        icon={<CheckOutlined/>}
                        onAsyncClick={onFinishRecording!}
                        style={actionButtonStyle}
                    >
                        {t('mapJoystick.finish')}
                    </AsyncButton>
                    <AsyncButton
                        danger
                        icon={<CloseOutlined/>}
                        onAsyncClick={onCancelRecording!}
                        style={actionButtonStyle}
                    >
                        {t('mapJoystick.cancel')}
                    </AsyncButton>
                    <AsyncButton
                        icon={<DockIcon/>}
                        onAsyncClick={onHome!}
                        style={actionButtonStyle}
                    >
                        {t('mapJoystick.home')}
                    </AsyncButton>
                </div>
            )}
        </div>
    );
};
