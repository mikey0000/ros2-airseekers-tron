import {useEffect, useRef, useState} from "react";
import {App, Button, Tag} from "antd";
import {ScissorOutlined, StopOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import {useThemeMode} from "../../../theme/ThemeContext.tsx";
import {useStatus} from "../../../hooks/useStatus.ts";

/** How long "Start blades" must be held before mow_enabled=1 is sent. */
export const BLADE_HOLD_MS = 1000;
/** No "blade on" sub_state this long after a start request: tell the operator it was refused. */
export const BLADE_CONFIRM_TIMEOUT_MS = 4000;

interface ManualBladeControlProps {
    /** Mission reports MANUAL_MOWING with the blade on (sub_state). */
    bladeOn: boolean;
    /** MANUAL_MOWING reached and blade off: a start request makes sense. */
    canStart: boolean;
    onStart: () => Promise<void>;
    onStop: () => Promise<void>;
}

/**
 * Two-step manual mowing (profile feature manual_blade_two_step): the joystick
 * only drives; the blades start from this separate press-and-hold control and
 * stop immediately from the red button.
 */
export const ManualBladeControl = ({bladeOn, canStart, onStart, onStop}: ManualBladeControlProps) => {
    // Subscribed here (only while the control is shown), not in MapPage.
    const rpm = useStatus().mower_motor_rpm;
    const {t} = useTranslation();
    const {notification} = App.useApp();
    const {colors} = useThemeMode();
    const [holding, setHolding] = useState(false);
    const [pending, setPending] = useState(false);
    const holdTimer = useRef<ReturnType<typeof setTimeout> | undefined>(undefined);
    const confirmTimer = useRef<ReturnType<typeof setTimeout> | undefined>(undefined);

    useEffect(() => () => {
        clearTimeout(holdTimer.current);
        clearTimeout(confirmTimer.current);
    }, []);

    // The mission confirmed the blade (or manual ended): stop waiting.
    useEffect(() => {
        if (bladeOn || !canStart) {
            clearTimeout(confirmTimer.current);
            setPending(false);
        }
    }, [bladeOn, canStart]);

    const fail = (description: string) => notification.error({message: t("manualBlade.startFailed"), description});

    const fire = () => {
        setHolding(false);
        setPending(true);
        onStart().then(() => {
            clearTimeout(confirmTimer.current);
            confirmTimer.current = setTimeout(() => {
                setPending(false);
                fail(t("manualBlade.refused"));
            }, BLADE_CONFIRM_TIMEOUT_MS);
        }).catch((e: Error) => {
            setPending(false);
            fail(e.message);
        });
    };

    const beginHold = () => {
        if (!canStart || pending) return;
        setHolding(true);
        clearTimeout(holdTimer.current);
        holdTimer.current = setTimeout(fire, BLADE_HOLD_MS);
    };
    const cancelHold = () => {
        clearTimeout(holdTimer.current);
        holdTimer.current = undefined;
        setHolding(false);
    };

    const stop = () => {
        cancelHold();
        clearTimeout(confirmTimer.current);
        setPending(false);
        onStop().catch((e: Error) => notification.error({message: t("manualBlade.stopFailed"), description: e.message}));
    };

    const style: React.CSSProperties = {height: 44, minWidth: 150, borderRadius: 10, fontWeight: 600};

    return (
        <div style={{display: "flex", flexDirection: "column", gap: 8, marginBottom: 4, alignItems: "stretch"}}
             data-testid="manual-blade-control">
            <Tag color={bladeOn ? "red" : "default"} style={{margin: 0, textAlign: "center", fontWeight: 600}}>
                {bladeOn ? t("manualBlade.on") : t("manualBlade.off")}
                {bladeOn && rpm !== undefined && rpm > 0 ? ` · ${Math.round(rpm)} rpm` : ""}
            </Tag>
            {bladeOn ? (
                <Button danger type="primary" icon={<StopOutlined/>} style={style} onClick={stop}>
                    {t("manualBlade.stop")}
                </Button>
            ) : (
                <Button
                    icon={<ScissorOutlined/>}
                    disabled={!canStart}
                    loading={pending}
                    style={{
                        ...style,
                        // Fill grows while held so the operator sees the hold progress.
                        backgroundImage: canStart
                            ? `linear-gradient(90deg, ${colors.danger}, ${colors.danger})`
                            : undefined,
                        backgroundRepeat: "no-repeat",
                        backgroundPosition: "left center",
                        backgroundSize: holding ? "100% 100%" : "0% 100%",
                        transition: holding ? `background-size ${BLADE_HOLD_MS}ms linear` : "none",
                        touchAction: "none",
                        userSelect: "none",
                    }}
                    onPointerDown={beginHold}
                    onPointerUp={cancelHold}
                    onPointerLeave={cancelHold}
                    onPointerCancel={cancelHold}
                    onKeyDown={(e) => { if ((e.key === " " || e.key === "Enter") && !e.repeat) beginHold(); }}
                    onKeyUp={cancelHold}
                    aria-label={t("manualBlade.startHint")}
                    title={t("manualBlade.startHint")}
                >
                    {pending ? t("manualBlade.starting") : holding ? t("manualBlade.holding") : t("manualBlade.start")}
                </Button>
            )}
        </div>
    );
};
