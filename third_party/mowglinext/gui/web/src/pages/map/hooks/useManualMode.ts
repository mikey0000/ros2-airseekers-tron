import {useCallback, useEffect, useRef, useState} from "react";
import type {TwistStamped} from "../../../types/ros.ts";
import type {IJoystickUpdateEvent} from "react-joystick-component/build/lib/Joystick";
import {useTeleopLimits} from "./useTeleopLimits.ts";

const JOY_SEND_INTERVAL_MS = 100;
// How long a sustained non-MANUAL_MOWING state must persist before we tear
// down the manual UI. A single stray guard frame (EMERGENCY/battery/boundary
// blip emits one non-MANUAL tick) must NOT collapse manual mode and kill the
// joystick socket mid-drive — only a genuine, sustained exit should.
const MANUAL_EXIT_DEBOUNCE_MS = 1200;
// After the operator presses Manual, the robot may take a while to report
// MANUAL_MOWING (slow state tick): keep the UI and joystick up this long
// before a still-non-manual state may tear it down.
const MANUAL_ENTRY_GRACE_MS = 5000;
// Two-step manual mowing (profile feature manual_blade_two_step): the mission
// publishes the blade state in HighLevelStatus.sub_state_name.
export const MANUAL_SUB_BLADE_ON = "joystick, blade on";
export const MANUAL_SUB_BLADE_OFF = "joystick, blade off";
// Teleop velocity caps — raw joystick values are in [-1, 1] (normalized by
// react-joystick-component) and multiplied at this layer (before twist_mux),
// so Nav2 autonomous speeds are unaffected. The caps come from the robot
// profile (`profile.teleop`, stock 0.25 m/s / 0.6 rad/s: at 1.0 m/s the robot
// was too twitchy on grass), lowered to a robot-side relay clamp when one is
// reported (see useTeleopLimits.ts).

interface UseManualModeOptions {
    mowerAction: (action: string, params: Record<string, unknown>) => () => Promise<void>;
    joyStream: { sendJsonMessage: (msg: unknown) => void; start: (uri: string) => void };
    stateName?: string;
    subStateName?: string;
    /** Profile feature manual_blade_two_step: entering manual never starts the blade. */
    bladeTwoStep?: boolean;
}

export function useManualMode({mowerAction, joyStream, stateName, subStateName, bladeTwoStep}: UseManualModeOptions) {
    const [manualMode, setManualMode] = useState(() => stateName === "MANUAL_MOWING");
    const limits = useTeleopLimits();
    const limitsRef = useRef(limits);
    useEffect(() => {
        limitsRef.current = {maxLinear: limits.maxLinear, maxAngular: limits.maxAngular};
    }, [limits.maxLinear, limits.maxAngular]);
    const exitTimerRef = useRef<ReturnType<typeof setTimeout> | undefined>(undefined);
    const enteredAtRef = useRef<number>(-Infinity);

    // LATCH + DEBOUNCE manual mode. Entering MANUAL_MOWING latches it ON
    // immediately; leaving it only tears the UI down after a sustained
    // (debounced) window of non-MANUAL frames. This keeps a single stray guard
    // frame (EMERGENCY/battery/boundary blip ahead of MainLogic) from collapsing
    // the manual UI and killing the joystick socket mid-drive. The explicit Stop
    // button flips manualMode directly, so it doesn't depend on this debounce.
    useEffect(() => {
        if (stateName === "MANUAL_MOWING") {
            clearTimeout(exitTimerRef.current);
            exitTimerRef.current = undefined;
            setManualMode(true);
            return;
        }
        // Non-MANUAL frame while latched: arm (or keep) the debounce timer.
        if (manualMode && exitTimerRef.current === undefined) {
            const grace = enteredAtRef.current + MANUAL_ENTRY_GRACE_MS - Date.now();
            exitTimerRef.current = setTimeout(() => {
                exitTimerRef.current = undefined;
                setManualMode(false);
            }, Math.max(MANUAL_EXIT_DEBOUNCE_MS, grace));
        }
    }, [stateName, manualMode]);

    const lastTwistRef = useRef<TwistStamped | null>(null);
    const joyIntervalRef = useRef<ReturnType<typeof setInterval> | undefined>(undefined);

    const startJoyInterval = useCallback(() => {
        clearInterval(joyIntervalRef.current);
        joyIntervalRef.current = setInterval(() => {
            if (lastTwistRef.current) {
                joyStream.sendJsonMessage(lastTwistRef.current);
            }
        }, JOY_SEND_INTERVAL_MS);
    }, [joyStream]);

    const stopJoyInterval = useCallback(() => {
        clearInterval(joyIntervalRef.current);
        joyIntervalRef.current = undefined;
    }, []);

    // Cleanup on unmount — stop joy interval and any pending exit debounce
    useEffect(() => {
        return () => {
            clearInterval(joyIntervalRef.current);
            clearTimeout(exitTimerRef.current);
        };
    }, []);

    const handleManualMode = async () => {
        // Joy stream is auto-started by useMapStreams when state becomes MANUAL_MOWING.
        // Send the command first — the BT will transition to MANUAL_MOWING state.
        await mowerAction("high_level_control", {Command: 7})();
        // Open the joy socket now instead of waiting for the next state frame
        // (useMapStreams also starts it on MANUAL_MOWING).
        enteredAtRef.current = Date.now();
        clearTimeout(exitTimerRef.current);
        exitTimerRef.current = undefined;
        joyStream.start("/api/mowglinext/publish/joy");
        // The BT owns the blade: once state=4 (MANUAL_MOWING) it re-ticks
        // SetMowerEnabled(true) ~10 Hz. We deliberately do NOT send mow_enabled=1
        // from the client here — that call races the firmware, which zeroes the
        // blade while the HL mode is still IDLE (this immediate send + a 10 s
        // keepalive was documented as REMOVED in main_tree.xml). Latch the UI.
        setManualMode(true);
    };

    const handleStopManualMode = async () => {
        // STOP (COMMAND_STOP=8 → StopHoldSequence): from MANUAL_MOWING (state 4)
        // this halts in place and turns the mower off, exiting manual — no dock
        // drive.
        await mowerAction("high_level_control", {Command: 8})();
        // Explicit Stop: drop the manual UI immediately and cancel any pending
        // debounce so a lingering timer can't re-toggle it.
        clearTimeout(exitTimerRef.current);
        exitTimerRef.current = undefined;
        stopJoyInterval();
        lastTwistRef.current = null;
        setManualMode(false);
        await mowerAction("mow_enabled", {mow_enabled: 0, mow_direction: 0})();
    };

    const handleJoyMove = useCallback((event: IJoystickUpdateEvent) => {
        const {maxLinear, maxAngular} = limitsRef.current;
        const linear = (event.y ?? 0) * maxLinear;
        const angular = (event.x ?? 0) * -1 * maxAngular;
        const msg: TwistStamped = {
            header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""},
            twist: {linear: {x: linear, y: 0, z: 0}, angular: {z: angular, x: 0, y: 0}},
        };
        lastTwistRef.current = msg;
        joyStream.sendJsonMessage(msg);
        if (!joyIntervalRef.current) {
            startJoyInterval();
        }
    }, [joyStream, startJoyInterval]);

    const handleJoyStop = useCallback(() => {
        const msg: TwistStamped = {
            header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""},
            twist: {linear: {x: 0, y: 0, z: 0}, angular: {z: 0, x: 0, y: 0}},
        };
        lastTwistRef.current = null;
        stopJoyInterval();
        joyStream.sendJsonMessage(msg);
    }, [joyStream, stopJoyInterval]);

    // Two-step blade control (only meaningful with bladeTwoStep): the mission
    // forwards mow_enabled to ~/manual_blade while MANUAL_MOWING and reports
    // the result in sub_state_name.
    const bladeOn = stateName === "MANUAL_MOWING" && subStateName === MANUAL_SUB_BLADE_ON;
    const canStartBlade = !!bladeTwoStep && stateName === "MANUAL_MOWING" && !bladeOn;

    const handleBladeStart = useCallback(async () => {
        await mowerAction("mow_enabled", {mow_enabled: 1, mow_direction: 0})();
    }, [mowerAction]);

    const handleBladeStop = useCallback(async () => {
        await mowerAction("mow_enabled", {mow_enabled: 0, mow_direction: 0})();
    }, [mowerAction]);

    return {
        manualMode, handleManualMode, handleStopManualMode, handleJoyMove, handleJoyStop,
        bladeOn, canStartBlade, handleBladeStart, handleBladeStop,
    };
}
