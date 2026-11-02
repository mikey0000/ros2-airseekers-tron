import {useEffect, useRef} from "react";
import {useTranslation} from "react-i18next";
import {getMultiplexedSocket} from "./multiplexedSocket.ts";
import {stateNotificationKey, useNotificationCenter, type NotificationLevel} from "./useNotificationCenter.tsx";

// /mower_alerts/events (backend key "alerts"): std_msgs/String whose data is
// the alert node's JSON (src/mower_alerts README "Wire format"). Until now it
// only fed push/MQTT; the bell and the Health check never saw it (F07).

export interface AlertEvent {
    id: string;
    incident: string;
    kind: string;
    message: string;
    priority: number;
    text: string;
    params: Record<string, string>;
    /** Wall time (s) on the mower. */
    stamp: number | null;
}

/** Parse one frame; null for heartbeats and anything malformed. */
export function parseAlertEvent(raw: unknown): AlertEvent | null {
    let d: unknown = raw;
    if (d && typeof d === "object" && typeof (d as {data?: unknown}).data === "string") {
        try {
            d = JSON.parse((d as {data: string}).data);
        } catch {
            return null;
        }
    }
    if (!d || typeof d !== "object") return null;
    const r = d as Record<string, unknown>;
    if (r.type !== "alert" || typeof r.message !== "string" || !r.message) return null;
    const str = (v: unknown) => (typeof v === "string" ? v : "");
    const params: Record<string, string> = {};
    if (r.params && typeof r.params === "object") {
        for (const [k, v] of Object.entries(r.params as Record<string, unknown>)) params[k] = String(v);
    }
    return {
        id: str(r.id) || `${str(r.incident)}-${String(r.stamp)}`,
        incident: str(r.incident),
        kind: str(r.kind),
        message: r.message,
        priority: typeof r.priority === "number" ? r.priority : 3,
        text: str(r.text),
        params,
        stamp: typeof r.stamp === "number" ? r.stamp : null,
    };
}

/** Alert message ids that report a mission state the GUI also pushes on entry. */
const ALERT_STATE: Record<string, string> = {
    stuck: "STUCK_NEEDS_HELP",
    mowIncomplete: "MOWING_INCOMPLETE",
    dockFailed: "NAV_TO_DOCK_FAILED",
    undockFailed: "UNDOCK_FAILED",
};

/** Notification dedupe key of an alert: shared with the state push when it reports one. */
export const alertNotificationKey = (a: AlertEvent): string =>
    ALERT_STATE[a.message] ? stateNotificationKey(ALERT_STATE[a.message]) : `alert:${a.id}`;

export const alertLevel = (priority: number): NotificationLevel =>
    priority >= 5 ? "error" : priority >= 4 ? "warning" : "info";

/** Alerts older than this are not news (the backend replays the last one on subscribe). */
export const ALERT_MAX_AGE_S = 15 * 60;

/**
 * Feeds mower_alerts events into the notification center. Mounted once in the
 * app shell. Subscribes on the socket directly rather than through useTopic so
 * two alerts in one frame are never coalesced into one render.
 */
export function useAlertNotifications(enabled = true): void {
    const {t, i18n} = useTranslation();
    const {push} = useNotificationCenter();
    const seen = useRef(new Set<string>());
    const pushRef = useRef(push);
    pushRef.current = push;
    const tRef = useRef({t, i18n});
    tRef.current = {t, i18n};

    useEffect(() => {
        if (!enabled) return;
        return getMultiplexedSocket().subscribe("alerts", (raw) => {
            const a = parseAlertEvent(raw);
            if (!a || seen.current.has(a.id)) return;
            seen.current.add(a.id);
            if (a.stamp !== null && Date.now() / 1000 - a.stamp > ALERT_MAX_AGE_S) return;
            const {t: tr, i18n: i} = tRef.current;
            const titleKey = `alertEvents.${a.message}`;
            pushRef.current({
                level: alertLevel(a.priority),
                title: i.exists(titleKey) ? tr(titleKey, a.params) : (a.text || a.message),
                body: a.text || undefined,
                key: alertNotificationKey(a),
                source: "alert",
                href: a.params.map ? undefined : "/#/map",
            });
        });
    }, [enabled]);
}
