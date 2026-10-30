import {App} from "antd";
import {act, render, screen, waitFor} from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import {beforeEach, describe, expect, it, vi} from "vitest";
import type {ConnectionState} from "../../transport/connectionManager.ts";
import {ConnectionSection} from "./ConnectionSection.tsx";

const baseState: ConnectionState = {
    mode: "wifi",
    ble: {link: "idle", session: "idle", deviceId: null, deviceName: null, host: null, rttMs: null},
    bleSupport: "ok",
    networkAlive: true,
    lastNetworkAliveAt: 0,
    token: null,
    deviceId: null,
    notice: null,
};

let state: ConnectionState = baseState;
const listeners = new Set<() => void>();
const fakeManager = {
    switchTo: vi.fn(() => Promise.resolve()),
    pairNew: vi.fn(() => Promise.resolve()),
    forgetDevice: vi.fn(() => Promise.resolve()),
    setToken: vi.fn(),
    getBleCounters: vi.fn(() => ({bytesIn: 2048, bytesOut: 100})),
    dismissNotice: vi.fn(),
};

vi.mock("../../transport/connectionManager.ts", async () => {
    const react = await import("react");
    return {
        getConnectionManager: () => fakeManager,
        useConnectionState: () => react.useSyncExternalStore(
            (cb: () => void) => {
                listeners.add(cb);
                return () => listeners.delete(cb);
            },
            () => state,
        ),
    };
});

function setState(patch: Partial<ConnectionState>) {
    state = {...state, ...patch};
    act(() => listeners.forEach((l) => l()));
}

function renderSection() {
    return render(<App><ConnectionSection/></App>);
}

beforeEach(() => {
    state = baseState;
    listeners.clear();
    vi.clearAllMocks();
});

describe("ConnectionSection", () => {
    it("explains that Bluetooth needs a secure origin", () => {
        state = {...baseState, bleSupport: "insecure"};
        renderSection();
        expect(screen.getByTestId("connection-unsupported")).toHaveTextContent("Open the app from its HTTPS address (Settings → Remote access → Secure origin).");
        expect(screen.queryByText("Connect a Bluetooth mower")).toBeNull();
    });

    it("explains which browsers support Bluetooth", () => {
        state = {...baseState, bleSupport: "unsupported"};
        renderSection();
        expect(screen.getByTestId("connection-unsupported")).toHaveTextContent("iPhone is not supported");
    });

    it("pairs a new mower from the button", async () => {
        renderSection();
        await userEvent.click(screen.getByRole("button", {name: /Connect a Bluetooth mower/}));
        expect(fakeManager.pairNew).toHaveBeenCalledTimes(1);
    });

    it("switches the link with the segmented control", async () => {
        renderSection();
        await userEvent.click(screen.getByText("Bluetooth", {selector: ".ant-segmented-item-label *"}));
        expect(fakeManager.switchTo).toHaveBeenCalledWith("ble");
    });

    it("shows link details and counters", () => {
        state = {
            ...baseState,
            mode: "ble",
            deviceId: "dev-1",
            ble: {link: "connected", session: "ready", deviceId: "dev-1", deviceName: "AIRSEEKERS_ABC123", host: "mower", rttMs: 81},
        };
        renderSection();
        expect(screen.getByText("AIRSEEKERS_ABC123")).toBeInTheDocument();
        expect(screen.getByTestId("connection-link-state")).toHaveTextContent("Connected");
        expect(screen.getByText("81 ms")).toBeInTheDocument();
        expect(screen.getByText("2.0 KB")).toBeInTheDocument();
        expect(screen.getByRole("button", {name: /Forget device/})).toBeInTheDocument();
    });

    it("offers a reconnect when the remembered mower is out of range", async () => {
        state = {...baseState, mode: "ble", deviceId: "dev-1"};
        renderSection();
        expect(screen.getByText(/not in range/)).toBeInTheDocument();
        await userEvent.click(screen.getByRole("button", {name: /Reconnect/}));
        expect(fakeManager.switchTo).toHaveBeenCalledWith("ble");
    });

    it("saves a pasted token and follows a fetched one", async () => {
        renderSection();
        expect(screen.getByText(/No token yet/)).toBeInTheDocument();
        await userEvent.type(screen.getByTestId("connection-token"), "PASTEDTOKEN");
        await userEvent.click(screen.getByRole("button", {name: /Save token/}));
        expect(fakeManager.setToken).toHaveBeenCalledWith("PASTEDTOKEN");
        setState({token: "FETCHED"});
        await waitFor(() => expect(screen.getByTestId("connection-token")).toHaveValue("FETCHED"));
    });

    it("warns when the mower rejected the token", () => {
        state = {...baseState, token: "OLD", ble: {...baseState.ble, session: "unauthenticated"}};
        renderSection();
        expect(screen.getByText(/rejected this token/)).toBeInTheDocument();
    });
});
