import {
    createContext, useCallback, useContext, useEffect, useMemo, useRef, useState,
    type ReactNode,
} from "react";
import {useTranslation} from "react-i18next";
import {updaterRequest, useHostUpdater} from './useHostUpdater';
import {plainSubState} from '../utils/missionPresentation.ts';

/** Mission states that need the owner, pushed on entry (F07). */
const STATE_NOTIFICATIONS: Record<string, {level: NotificationLevel; titleKey: string}> = {
    STUCK_NEEDS_HELP: {level: 'error', titleKey: 'missionText.stuckTitle'},
    MOWING_INCOMPLETE: {level: 'warning', titleKey: 'missionText.incompleteTitle'},
    NAV_TO_DOCK_FAILED: {level: 'error', titleKey: 'missionText.dockFailedTitle'},
    UNDOCK_FAILED: {level: 'error', titleKey: 'utils.stateStartFailed'},
    AREA_UNREACHABLE: {level: 'warning', titleKey: 'utils.stateAreaUnreachable'},
};

/** Shared dedupe key of a mission state and the mower_alerts event for it. */
export const stateNotificationKey = (stateName: string): string => `state:${stateName}`;

/**
 * App-scoped notification center.
 *
 * Buffers persistent notifications across pages so operators can come back to
 * them, and exposes a bell + drawer UI. Independent from AntD's transient
 * `notification.error()` toasts; for now items only enter the buffer by an
 * explicit `push()` call. Future work: auto-mirror critical AntD toasts here,
 * and auto-generate items from BT state transitions (boundary recovery,
 * coverage failed, etc.).
 */

export type NotificationLevel = 'info' | 'warning' | 'error' | 'success';

export interface NotificationItem {
    id: string;
    level: NotificationLevel;
    title: string;
    body?: string;
    timestamp: number;
    read: boolean;
    href?: string;
    /** Dedupe key: one event seen twice (the mission state change and the
     *  mower_alerts event for it, or a replayed alert) is pushed once. */
    key?: string;
    /** Where it came from: 'alert' = /mower_alerts/events, 'state' = a mission state change. */
    source?: 'alert' | 'state';
}

interface NotificationCenterValue {
    items: NotificationItem[];
    unread: number;
    push: (n: Omit<NotificationItem, 'id' | 'timestamp' | 'read'>) => void;
    markRead: (id: string) => void;
    markAllRead: () => void;
    dismiss: (id: string) => void;
    clear: () => void;
}

const NotificationCenterContext = createContext<NotificationCenterValue>({
    items: [],
    unread: 0,
    push: () => { /* no-op */ },
    markRead: () => { /* no-op */ },
    markAllRead: () => { /* no-op */ },
    dismiss: () => { /* no-op */ },
    clear: () => { /* no-op */ },
});

const MAX_ITEMS = 100;
/** A second push with the same key within this window is dropped. */
export const NOTIFICATION_DEDUPE_MS = 10 * 60 * 1000;

export function NotificationCenterProvider({children}: {children: ReactNode}) {
    const {t} = useTranslation();
    const host = useHostUpdater();
    const [items, setItems] = useState<NotificationItem[]>([]);
    const counterRef = useRef(0);

    useEffect(() => {
        if (!host.data) return;
        const notices: NotificationItem[] = host.data.state.notices.filter(n => !n.dismissed).map(n => ({
            id: `update:${n.id}`, level: 'info', title: t(n.kind === 'updater' ? 'hostUpdater.agentNotice' : n.kind === 'available' ? 'hostUpdater.notice' : 'hostUpdater.reviewNotice'),
            body: n.deployment, timestamp: new Date(n.created_at).getTime(), read: n.read, href: '/#/settings?section=updates',
        }));
        setItems(previous => [...notices, ...previous.filter(n => !n.id.startsWith('update:'))].slice(0, MAX_ITEMS));
    }, [host.data, t]);

    const acknowledgeUpdate = useCallback((id: string, dismiss: boolean) => {
        if (id.startsWith('update:')) void updaterRequest('notice', {id: id.slice(7), dismiss}).then(host.refresh).catch(() => { /* Next cached status read restores unacknowledged notices. */ });
    }, [host.refresh]);

    const push = useCallback((n: Omit<NotificationItem, 'id' | 'timestamp' | 'read'>) => {
        setItems(prev => {
            const now = Date.now();
            if (n.key && prev.some(it => it.key === n.key && now - it.timestamp < NOTIFICATION_DEDUPE_MS)) return prev;
            const next: NotificationItem = {
                ...n,
                id: `n-${Date.now()}-${counterRef.current++}`,
                timestamp: Date.now(),
                read: false,
            };
            const trimmed = [next, ...prev];
            return trimmed.length > MAX_ITEMS ? trimmed.slice(0, MAX_ITEMS) : trimmed;
        });
    }, []);

    const markRead = useCallback((id: string) => {
        acknowledgeUpdate(id, false);
        setItems(prev => prev.map(it => it.id === id ? {...it, read: true} : it));
    }, [acknowledgeUpdate]);

    const markAllRead = useCallback(() => {
        items.filter(it => !it.read).forEach(it => acknowledgeUpdate(it.id, false));
        setItems(prev => prev.map(it => it.read ? it : {...it, read: true}));
    }, [items, acknowledgeUpdate]);

    const dismiss = useCallback((id: string) => {
        acknowledgeUpdate(id, true);
        setItems(prev => prev.filter(it => it.id !== id));
    }, [acknowledgeUpdate]);

    const clear = useCallback(() => {items.forEach(it => acknowledgeUpdate(it.id, true)); setItems([]);}, [items, acknowledgeUpdate]);

    const unread = items.reduce((acc, it) => acc + (it.read ? 0 : 1), 0);

    const value = useMemo<NotificationCenterValue>(() => ({
        items, unread, push, markRead, markAllRead, dismiss, clear,
    }), [items, unread, push, markRead, markAllRead, dismiss, clear]);

    return (
        <NotificationCenterContext.Provider value={value}>
            {children}
        </NotificationCenterContext.Provider>
    );
}

export function useNotificationCenter(): NotificationCenterValue {
    return useContext(NotificationCenterContext);
}

/**
 * Hook variant that subscribes to selected app events and auto-pushes
 * notifications. Mounted once at app root so the drawer is populated even
 * when the operator is on a different page.
 */
export function useAutoNotifications(opts: {
    emergencyActive: boolean;
    emergencyLatched: boolean;
    rainDetected: boolean;
    state: string | undefined;
    /** sub_state_name: the plain reason in the stuck / unfinished / dock-failed items. */
    subState?: string;
}) {
    const {t} = useTranslation();
    const {push} = useNotificationCenter();
    const prevEmergencyRef = useRef(opts.emergencyActive);
    const prevRainRef = useRef(opts.rainDetected);
    const prevStateRef = useRef(opts.state);
    // Read when the state changes; not a trigger of its own.
    const subStateRef = useRef(opts.subState);
    subStateRef.current = opts.subState;

    useEffect(() => {
        if (opts.emergencyActive && !prevEmergencyRef.current) {
            push({level: 'error', title: t('notificationCenter.emergencyTitle'),
                  body: t('notificationCenter.emergencyBody')});
        }
        prevEmergencyRef.current = opts.emergencyActive;
    }, [opts.emergencyActive, push, t]);

    useEffect(() => {
        if (opts.rainDetected && !prevRainRef.current) {
            push({level: 'warning', title: t('notificationCenter.rainTitle'),
                  body: t('notificationCenter.rainBody')});
        }
        prevRainRef.current = opts.rainDetected;
    }, [opts.rainDetected, push, t]);

    useEffect(() => {
        if (opts.state === 'MOWING_COMPLETE' && prevStateRef.current !== 'MOWING_COMPLETE') {
            push({level: 'success', title: t('notificationCenter.mowingCompleteTitle'),
                  body: t('notificationCenter.mowingCompleteBody')});
        }
        if (opts.state === 'BOUNDARY_EMERGENCY_STOP' && prevStateRef.current !== 'BOUNDARY_EMERGENCY_STOP') {
            push({level: 'error', title: t('notificationCenter.boundaryTitle'),
                  body: t('notificationCenter.boundaryBody'), key: stateNotificationKey('BOUNDARY_EMERGENCY_STOP'),
                  source: 'state', href: '/#/map'});
        }
        const attention = opts.state ? STATE_NOTIFICATIONS[opts.state] : undefined;
        if (attention && prevStateRef.current !== opts.state) {
            const plain = plainSubState(opts.state, subStateRef.current);
            push({level: attention.level, title: t(attention.titleKey),
                  body: plain ? t(plain.key, plain.params) : undefined,
                  key: stateNotificationKey(opts.state!), source: 'state', href: '/#/map'});
        }
        prevStateRef.current = opts.state;
    }, [opts.state, push, t]);
}
