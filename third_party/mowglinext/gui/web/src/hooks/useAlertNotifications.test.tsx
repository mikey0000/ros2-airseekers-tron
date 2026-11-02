import {act, render, screen} from "@testing-library/react";
import {beforeEach, describe, expect, it, vi} from "vitest";
import type {ReactNode} from "react";

type Listener = (raw: unknown) => void;
const listeners = new Map<string, Set<Listener>>();
vi.mock("./multiplexedSocket.ts", () => ({
    getMultiplexedSocket: () => ({
        subscribe: (topic: string, cb: Listener) => {
            const set = listeners.get(topic) ?? new Set<Listener>();
            set.add(cb);
            listeners.set(topic, set);
            return () => set.delete(cb);
        },
    }),
}));
vi.mock("./useHostUpdater", () => ({useHostUpdater: () => ({data: null, refresh: vi.fn()}), updaterRequest: vi.fn()}));
vi.mock("react-i18next", () => ({
    useTranslation: () => ({
        t: (k: string) => k,
        i18n: {exists: (k: string) => k.startsWith("alertEvents.") && k !== "alertEvents.someNewAlert"},
    }),
}));

import {alertNotificationKey, parseAlertEvent, useAlertNotifications} from "./useAlertNotifications.ts";
import {NotificationCenterProvider, useAutoNotifications, useNotificationCenter} from "./useNotificationCenter.tsx";

const alert = (over: Record<string, unknown> = {}) => ({
    data: JSON.stringify({
        type: "alert", id: "stuck-1", incident: "stuck", kind: "blocked", message: "stuck", priority: 4,
        text: "Stuck: the robot could not free itself and needs help.", params: {state: "STUCK_NEEDS_HELP"},
        stamp: Date.now() / 1000, ...over,
    }),
});
const deliver = (raw: unknown) => act(() => { listeners.get("alerts")?.forEach((cb) => cb(raw)); });

function Bell() {
    const {items, unread, markAllRead} = useNotificationCenter();
    return <div>
        <span data-testid="unread">{unread}</span>
        {items.map((i) => <div key={i.id} data-testid="item" data-level={i.level}>{i.title}|{i.body}</div>)}
        <button onClick={markAllRead}>read</button>
    </div>;
}
function Harness({state}: {state?: string}) {
    useAlertNotifications();
    useAutoNotifications({emergencyActive: false, emergencyLatched: false, rainDetected: false, state,
        subState: "stuck: needs help at (1.00, 2.00)"});
    return <Bell/>;
}
const wrapper = ({children}: {children: ReactNode}) => <NotificationCenterProvider>{children}</NotificationCenterProvider>;

describe("mower_alerts -> notification bell", () => {
    beforeEach(() => listeners.clear());

    it("parses the std_msgs/String envelope and skips heartbeats", () => {
        expect(parseAlertEvent(alert())?.message).toBe("stuck");
        expect(parseAlertEvent({data: JSON.stringify({type: "heartbeat", active: []})})).toBeNull();
        expect(parseAlertEvent({data: "not json"})).toBeNull();
        expect(alertNotificationKey(parseAlertEvent(alert())!)).toBe("state:STUCK_NEEDS_HELP");
        expect(alertNotificationKey(parseAlertEvent(alert({id: "t-1", message: "theftLift"}))!)).toBe("alert:t-1");
    });

    it("pushes an alert once (replays and the matching state change are deduped), then marks it read", () => {
        const {rerender} = render(<Harness state="MOWING"/>, {wrapper});
        deliver(alert());
        deliver(alert()); // backend replays the last alert on (re)subscribe
        rerender(<Harness state="STUCK_NEEDS_HELP"/>); // the mission state for the same event
        expect(screen.getAllByTestId("item")).toHaveLength(1);
        expect(screen.getByTestId("item")).toHaveTextContent("alertEvents.stuck|Stuck: the robot could not free itself");
        expect(screen.getByTestId("item").dataset.level).toBe("warning");
        expect(screen.getByTestId("unread")).toHaveTextContent("1");
        act(() => screen.getByText("read").click());
        expect(screen.getByTestId("unread")).toHaveTextContent("0");
    });

    it("pushes stuck / unfinished / dock-failed state changes without an alert node", () => {
        const {rerender} = render(<Harness state="MOWING"/>, {wrapper});
        rerender(<Harness state="STUCK_NEEDS_HELP"/>);
        rerender(<Harness state="MOWING_INCOMPLETE"/>);
        rerender(<Harness state="NAV_TO_DOCK_FAILED"/>);
        const titles = screen.getAllByTestId("item").map((i) => i.textContent);
        expect(titles).toEqual([
            expect.stringContaining("missionText.dockFailedTitle"),
            expect.stringContaining("missionText.incompleteTitle"),
            "missionText.stuckTitle|missionText.stuckWheels",
        ]);
    });

    it("ignores stale alerts and falls back to the sender text for unknown ids", () => {
        render(<Harness/>, {wrapper});
        deliver(alert({id: "old-1", stamp: Date.now() / 1000 - 3600}));
        deliver(alert({id: "n-1", message: "someNewAlert", text: "Something new", priority: 5}));
        const items = screen.getAllByTestId("item");
        expect(items).toHaveLength(1);
        expect(items[0]).toHaveTextContent("Something new|Something new");
        expect(items[0].dataset.level).toBe("error");
    });
});
