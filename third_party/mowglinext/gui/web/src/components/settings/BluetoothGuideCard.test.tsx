import {act, render, screen, waitFor} from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import {MemoryRouter, useLocation} from "react-router-dom";
import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import type {ConnectionState} from "../../transport/connectionManager.ts";
import {BluetoothGuideCard} from "./BluetoothGuideCard.tsx";
import type {SecureOriginStatus} from "./secureOrigin.ts";

const readyState: ConnectionState = {
    mode: "wifi",
    ble: {link: "idle", session: "idle", deviceId: null, deviceName: null, host: null, rttMs: null},
    bleSupport: "ok",
    networkAlive: true,
    lastNetworkAliveAt: 0,
    token: "TESTTOKEN",
    deviceId: "dev-1",
    notice: null,
};

let state: ConnectionState = readyState;
const listeners = new Set<() => void>();

vi.mock("../../transport/connectionManager.ts", async () => {
    const react = await import("react");
    return {
        useConnectionState: () => react.useSyncExternalStore(
            (cb: () => void) => {
                listeners.add(cb);
                return () => listeners.delete(cb);
            },
            () => state,
        ),
    };
});

const secureOrigin: SecureOriginStatus = {
    enabled: true,
    https_urls: ["https://test-mower.local:4443", "https://192.0.2.10:4443"],
    hostname: "test-mower",
};

let permittedDevices: {id: string; name?: string}[] = [];

function setSecureContext(value: boolean) {
    Object.defineProperty(window, "isSecureContext", {configurable: true, value});
}

/** `getDevices: false` models a browser with Web Bluetooth but without the permissions backend. */
function setBluetooth(present: boolean, {getDevices = true} = {}) {
    const bluetooth = present
        ? {getDevices: getDevices ? () => Promise.resolve(permittedDevices) : undefined}
        : undefined;
    Object.defineProperty(navigator, "bluetooth", {configurable: true, value: bluetooth});
}

function mockSecureOrigin(status: SecureOriginStatus | null) {
    const fetchMock = vi.fn((input: string) => {
        expect(input).toBe("/api/secure-origin");
        return Promise.resolve(status
            ? new Response(JSON.stringify(status), {status: 200, headers: {"Content-Type": "application/json"}})
            : new Response("boom", {status: 500}));
    });
    vi.stubGlobal("fetch", fetchMock);
    return fetchMock;
}

function LocationProbe() {
    const location = useLocation();
    return <span data-testid="location">{location.search}</span>;
}

function renderGuide() {
    return render(
        <MemoryRouter initialEntries={["/settings?section=connection"]}>
            <BluetoothGuideCard/>
            <LocationProbe/>
        </MemoryRouter>,
    );
}

function setState(patch: Partial<ConnectionState>) {
    state = {...state, ...patch};
    act(() => listeners.forEach((l) => l()));
}

const result = (testId: string) => screen.getByTestId(testId).getAttribute("data-result");

beforeEach(() => {
    state = readyState;
    listeners.clear();
    permittedDevices = [{id: "dev-1", name: "AIRSEEKERS_TEST"}];
    setSecureContext(true);
    setBluetooth(true);
    mockSecureOrigin(secureOrigin);
});

afterEach(() => {
    vi.unstubAllGlobals();
    Reflect.deleteProperty(navigator, "bluetooth");
});

describe("BluetoothGuideCard", () => {
    it("collapses to one line when every prerequisite passes", async () => {
        renderGuide();
        expect(await screen.findByTestId("ble-guide-ready")).toHaveTextContent("Bluetooth ready");
        expect(screen.queryByTestId("ble-guide")).toBeNull();
        expect(fetch).not.toHaveBeenCalled();
    });

    it("names supported browsers when Web Bluetooth is missing on a secure origin", async () => {
        setBluetooth(false);
        renderGuide();
        await waitFor(() => expect(result("ble-guide-browser")).toBe("fail"));
        expect(screen.getByTestId("ble-guide-browser")).toHaveTextContent(/Chrome or Edge.*iPhone/);
        expect(result("ble-guide-origin")).toBe("ok");
    });

    it("links the HTTPS addresses and Remote access when the origin is not secure", async () => {
        setSecureContext(false);
        setBluetooth(false);
        renderGuide();

        const urls = await screen.findByTestId("ble-guide-https-urls");
        expect(Array.from(urls.querySelectorAll("a[href]")).map((a) => a.getAttribute("href")))
            .toEqual(["https://test-mower.local:4443/", "https://192.0.2.10:4443/"]);
        expect(result("ble-guide-origin")).toBe("fail");
        // Chrome hides navigator.bluetooth on plain HTTP: not a verdict on the browser.
        expect(result("ble-guide-browser")).toBe("unknown");

        await userEvent.click(screen.getByRole("button", {name: "Open Remote access settings"}));
        expect(screen.getByTestId("location")).toHaveTextContent("section=remote_access");
    });

    it("still points to Remote access when the HTTPS status cannot be read", async () => {
        setSecureContext(false);
        mockSecureOrigin(null);
        renderGuide();
        await waitFor(() => expect(fetch).toHaveBeenCalledTimes(1));
        expect(screen.queryByTestId("ble-guide-https-urls")).toBeNull();
        expect(screen.getByRole("button", {name: "Open Remote access settings"})).toBeInTheDocument();
    });

    it("asks for the token when none is stored", async () => {
        state = {...readyState, token: null};
        renderGuide();
        await waitFor(() => expect(result("ble-guide-token")).toBe("fail"));
        expect(screen.getByTestId("ble-guide-token")).toHaveTextContent(/paste it under Bluetooth token/);
        expect(result("ble-guide-paired")).toBe("ok");
    });

    it("flags a stored token the mower rejected", async () => {
        state = {...readyState, ble: {...readyState.ble, session: "unauthenticated"}};
        renderGuide();
        await waitFor(() => expect(result("ble-guide-token")).toBe("fail"));
        expect(screen.getByTestId("ble-guide-token")).toHaveTextContent(/rejected the stored token/);
    });

    it("asks to pair when the browser holds no mower permission", async () => {
        permittedDevices = [];
        renderGuide();
        await waitFor(() => expect(result("ble-guide-paired")).toBe("fail"));
        expect(screen.getByTestId("ble-guide-paired")).toHaveTextContent(/Connect a Bluetooth mower/);
    });

    it("falls back to the remembered device when the browser cannot list permissions", async () => {
        setBluetooth(true, {getDevices: false});
        state = {...readyState, deviceId: null};
        renderGuide();
        await waitFor(() => expect(result("ble-guide-paired")).toBe("fail"));

        setState({deviceId: "dev-1"});
        expect(await screen.findByTestId("ble-guide-ready")).toBeInTheDocument();
    });

    it("re-checks when the window regains focus", async () => {
        permittedDevices = [];
        renderGuide();
        await waitFor(() => expect(result("ble-guide-paired")).toBe("fail"));

        permittedDevices = [{id: "dev-2", name: "AIRSEEKERS_OTHER"}];
        act(() => {
            window.dispatchEvent(new Event("focus"));
        });
        expect(await screen.findByTestId("ble-guide-ready")).toBeInTheDocument();
    });
});
