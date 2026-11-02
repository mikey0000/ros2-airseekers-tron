// End-to-end over the real useTopic: a raw /wifi/status frame delivered on the
// "wifiStatus" multiplex key must render the chip. Guards the 2026-11-02 bug
// where the backend allowlist dropped the topic and the chip never appeared.
import {act, render, screen} from "@testing-library/react";
import {beforeEach, describe, expect, it, vi} from "vitest";
import {WifiChip} from "./WifiChip.tsx";

type Listener = (raw: unknown) => void;
const listeners = new Map<string, Set<Listener>>();

vi.mock("../hooks/multiplexedSocket.ts", () => ({
    getMultiplexedSocket: () => ({
        subscribe: (topic: string, cb: Listener) => {
            const set = listeners.get(topic) ?? new Set<Listener>();
            set.add(cb);
            listeners.set(topic, set);
            return () => set.delete(cb);
        },
    }),
}));
vi.mock("react-i18next", () => ({
    useTranslation: () => ({t: (k: string) => k}),
}));

const deliver = (topic: string, raw: unknown) => act(() => {
    listeners.get(topic)?.forEach((cb) => cb(raw));
});

describe("WifiChip over the multiplex stream", () => {
    beforeEach(() => listeners.clear());

    it("subscribes to the wifiStatus key and renders on the first frame", () => {
        render(<WifiChip/>);
        expect(listeners.get("wifiStatus")?.size).toBe(1);
        expect(screen.queryByTestId("wifi-chip")).toBeNull();

        deliver("wifiStatus", {
            connected: false, ssid: "", bssid: "", freq_mhz: 0, signal_dbm: 0,
            tx_bitrate_mbps: 0, rx_bitrate_mbps: 0, tx_retries: 0, tx_failed: 0, roam_count: 0,
        });
        const chip = screen.getByTestId("wifi-chip");
        expect(chip.dataset.level).toBe("disconnected");
        expect(chip.textContent).toContain("wifiChip.noSignal");
    });
});
