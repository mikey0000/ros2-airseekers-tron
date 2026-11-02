import {App} from "antd";
import {render, screen, waitFor, within} from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import {MemoryRouter} from "react-router-dom";
import {beforeEach, describe, expect, it, vi} from "vitest";
import {NetworkSection} from "./NetworkSection.tsx";
import {groupNetworks, signalBars, type WifiAccessPoint, type WifiHostStatus} from "./networkWifi.ts";

// Placeholder identifiers only (documentation-range address, locally administered MACs).
const savedHome = {
    uuid: "uuid-home", id: "ExampleHome", ssid: "ExampleHome", security: "wpa2", band: "",
    powersave: "enabled", autoconnect: true, hidden: false, active: true,
} as const;
const savedOld = {...savedHome, uuid: "uuid-old", id: "ExampleOld", ssid: "ExampleOld", band: "bg", active: false} as const;

const hostStatus: WifiHostStatus = {
    available: true, interface: "wlan0", state: "activated", connected: true, ssid: "ExampleHome",
    bssid: "02:00:00:00:00:02", freq_mhz: 5180, strength: 55, ip4: ["192.0.2.10/24"],
    connection: savedHome, saved: [savedHome, savedOld],
};

const aps: WifiAccessPoint[] = [
    {ssid: "ExampleHome", bssid: "02:00:00:00:00:01", freq_mhz: 2437, strength: 70, security: "wpa2", active: false, saved: true},
    {ssid: "ExampleHome", bssid: "02:00:00:00:00:02", freq_mhz: 5180, strength: 55, security: "wpa2", active: true, saved: true},
    {ssid: "ExampleCafe", bssid: "02:00:00:00:00:03", freq_mhz: 2412, strength: 80, security: "wpa2-wpa3", active: false, saved: false},
    {ssid: "ExampleCorp", bssid: "02:00:00:00:00:05", freq_mhz: 5240, strength: 30, security: "enterprise", active: false, saved: false},
    {ssid: "", bssid: "02:00:00:00:00:06", freq_mhz: 2462, strength: 90, security: "wpa2", active: false, saved: false},
];

type Call = {path: string; method: string; headers: Headers; body: unknown};
const calls: Call[] = [];
let status: WifiHostStatus = hostStatus;
let rejectToken = false;

const apiFetch = vi.fn((path: string, init?: RequestInit): Promise<Response> => {
    const headers = new Headers(init?.headers);
    const method = init?.method ?? "GET";
    calls.push({path, method, headers, body: typeof init?.body === "string" ? JSON.parse(init.body) as unknown : undefined});
    const json = (body: unknown, code = 200) => Promise.resolve(
        new Response(JSON.stringify(body), {status: code, headers: {"content-type": "application/json"}}));
    if (path === "/api/network/wifi/status") return json(status);
    if (path === "/api/network/wifi/scan") return json({scanned: method === "POST", access_points: aps});
    if (path === "/api/ble/token") return json({token: "FRESHTOKEN"});
    if (rejectToken) return json({error: "missing or wrong X-Mower-Token"}, 401);
    if (path === "/api/network/wifi/connect") return json({uuid: "uuid-new", created: true});
    return json(savedHome);
});

vi.mock("../../transport/index.ts", () => ({apiFetch: (path: string, init?: RequestInit) => apiFetch(path, init)}));

const manager = {
    state: {mode: "wifi", token: "STOREDTOKEN"} as {mode: string; token: string | null},
    getState() {
        return this.state;
    },
    setToken: vi.fn((token: string) => {
        manager.state = {...manager.state, token};
    }),
};
vi.mock("../../transport/connectionManager.ts", () => ({getConnectionManager: () => manager}));

let linkMessage: unknown = null;
vi.mock("../../hooks/useTopic.ts", () => ({
    useTopic: (_key: string, initial: unknown, opts?: {select?: (raw: unknown) => unknown}) => ({
        data: linkMessage && opts?.select ? opts.select(linkMessage) ?? initial : initial,
    }),
}));

let wifiGate = true;
vi.mock("../../hooks/useProfileGates.ts", () => ({
    useProfileGates: () => ({loading: false, isVisible: (id: string) => id !== "feature:wifi_setup" || wifiGate}),
}));

function renderSection() {
    return render(<MemoryRouter><App><NetworkSection/></App></MemoryRouter>);
}

const posts = () => calls.filter((c) => c.method === "POST" && c.path !== "/api/network/wifi/scan");

beforeEach(() => {
    calls.length = 0;
    status = hostStatus;
    rejectToken = false;
    wifiGate = true;
    linkMessage = null;
    manager.state = {mode: "wifi", token: "STOREDTOKEN"};
    vi.clearAllMocks();
});

describe("networkWifi helpers", () => {
    it("groups the scan by SSID, drops hidden ones, keeps the connected network first", () => {
        const groups = groupNetworks(aps);
        expect(groups.map((g) => g.ssid)).toEqual(["ExampleHome", "ExampleCafe", "ExampleCorp"]);
        expect(groups[0]).toMatchObject({strength: 70, bands: ["2.4", "5"], accessPoints: 2, active: true, saved: true});
    });

    it("maps strength to bars", () => {
        expect([0, 20, 40, 60, 90].map(signalBars)).toEqual([0, 1, 2, 3, 4]);
    });
});

describe("NetworkSection", () => {
    it("shows the link, the drop warning, the networks and the saved list", async () => {
        linkMessage = {connected: true, ssid: "ExampleHome", bssid: "02:00:00:00:00:02", freq_mhz: 5180, signal_dbm: -63,
            tx_bitrate_mbps: 72, rx_bitrate_mbps: 65, roam_count: 3};
        renderSection();
        expect(await screen.findByTestId("wifi-drop-warning")).toBeInTheDocument();
        const card = screen.getByTestId("wifi-link-card");
        expect(within(card).getByTestId("wifi-link-ssid")).toHaveTextContent("ExampleHome");
        expect(within(card).getByText("-63 dBm")).toBeInTheDocument();
        expect(within(card).getByText("36 (5 GHz, 5180 MHz)")).toBeInTheDocument();
        expect(within(card).getByText("192.0.2.10/24")).toBeInTheDocument();

        const networks = await screen.findAllByTestId("wifi-network");
        expect(networks).toHaveLength(3);
        expect(within(networks[1]).getByText("WPA2/WPA3")).toBeInTheDocument();
        expect(within(networks[2]).getByRole("button", {name: "Join"})).toBeDisabled();
        expect(screen.getAllByTestId("wifi-saved-item")).toHaveLength(2);
        expect(calls.every((c) => !c.headers.has("X-Mower-Token")), "reads carry no token").toBe(true);
    });

    it("says Wi-Fi setup is unavailable when NetworkManager is not reachable", async () => {
        status = {available: false, reason: "system D-Bus: no socket", connected: false, saved: []};
        renderSection();
        expect(await screen.findByTestId("wifi-unavailable")).toHaveTextContent("Wi-Fi setup isn't available on this host");
        expect(screen.getByText("system D-Bus: no socket")).toBeInTheDocument();
        expect(screen.queryByTestId("wifi-networks")).not.toBeInTheDocument();
    });

    it("renders nothing of Wi-Fi setup without the profile feature", () => {
        wifiGate = false;
        renderSection();
        expect(screen.getByTestId("wifi-not-on-robot")).toBeInTheDocument();
        expect(apiFetch).not.toHaveBeenCalled();
    });

    it("joins a new network with the stored token, password and remember flag", async () => {
        const user = userEvent.setup();
        renderSection();
        const networks = await screen.findAllByTestId("wifi-network");
        await user.click(within(networks[1]).getByRole("button", {name: "Join"}));
        const dialog = await screen.findByRole("dialog");
        const ok = within(dialog).getByRole("button", {name: "Join"});
        expect(ok).toBeDisabled();
        const password = within(dialog).getByPlaceholderText("Password");
        await user.type(password, "short");
        expect(ok).toBeDisabled();
        await user.type(password, "-but-now-long");
        expect(password).toHaveAttribute("type", "password");
        await user.click(within(dialog).getByRole("img", {name: /eye/i}));
        expect(password).toHaveAttribute("type", "text");
        await user.click(within(dialog).getByTestId("wifi-remember"));
        await user.click(ok);

        await waitFor(() => expect(posts()).toHaveLength(1));
        const [call] = posts();
        expect(call.path).toBe("/api/network/wifi/connect");
        expect(call.headers.get("X-Mower-Token")).toBe("STOREDTOKEN");
        expect(call.body).toEqual({ssid: "ExampleCafe", password: "short-but-now-long", remember: false});
    });

    it("fetches the token over Wi-Fi when none is stored, and stores it", async () => {
        manager.state = {mode: "wifi", token: null};
        const user = userEvent.setup();
        renderSection();
        const saved = await screen.findAllByTestId("wifi-saved-item");
        await user.click(within(saved[1]).getByRole("button", {name: "Connect"}));
        await waitFor(() => expect(posts()).toHaveLength(1));
        expect(calls.some((c) => c.path === "/api/ble/token")).toBe(true);
        expect(manager.setToken).toHaveBeenCalledWith("FRESHTOKEN");
        expect(posts()[0].headers.get("X-Mower-Token")).toBe("FRESHTOKEN");
        expect(posts()[0].body).toEqual({uuid: "uuid-old"});
    });

    it("reports a rejected token", async () => {
        rejectToken = true;
        const user = userEvent.setup();
        renderSection();
        const saved = await screen.findAllByTestId("wifi-saved-item");
        await user.click(within(saved[1]).getByRole("button", {name: "Connect"}));
        expect(await screen.findByText(/refused the Bluetooth token/)).toBeInTheDocument();
        // One retry with a freshly fetched token, then give up.
        expect(posts()).toHaveLength(2);
    });

    it("forgets a saved network after confirmation", async () => {
        const user = userEvent.setup();
        renderSection();
        const saved = await screen.findAllByTestId("wifi-saved-item");
        await user.click(within(saved[1]).getByRole("button", {name: "Forget"}));
        const buttons = await waitFor(() => {
            const el = document.querySelector<HTMLElement>(".ant-popconfirm-buttons");
            expect(el).not.toBeNull();
            return el!;
        });
        await user.click(within(buttons).getByRole("button", {name: "Forget"}));
        await waitFor(() => expect(posts().map((c) => c.path)).toContain("/api/network/wifi/forget"));
        expect(posts()[0].body).toEqual({uuid: "uuid-old"});
    });

    it("locks the band of the active network after confirmation", async () => {
        const user = userEvent.setup();
        renderSection();
        await user.click(within(await screen.findByTestId("wifi-band")).getByText("2.4 GHz"));
        await user.click(await screen.findByRole("button", {name: "Apply"}));
        await waitFor(() => expect(posts()).toHaveLength(1));
        expect(posts()[0]).toMatchObject({path: "/api/network/wifi/band", body: {uuid: "uuid-home", band: "bg"}});
        expect(screen.getByText(/2.4 GHz reaches further/)).toBeInTheDocument();
    });

    it("switches power save off", async () => {
        const user = userEvent.setup();
        renderSection();
        const sw = await screen.findByTestId("wifi-powersave");
        expect(sw).toBeChecked();
        await user.click(sw);
        await user.click(await screen.findByRole("button", {name: "Apply"}));
        await waitFor(() => expect(posts()).toHaveLength(1));
        expect(posts()[0]).toMatchObject({path: "/api/network/wifi/powersave", body: {uuid: "uuid-home", enabled: false}});
    });

    it("scans on demand", async () => {
        const user = userEvent.setup();
        renderSection();
        await user.click(await screen.findByTestId("wifi-scan"));
        await waitFor(() => expect(calls.some((c) => c.path === "/api/network/wifi/scan" && c.method === "POST")).toBe(true));
    });
});
