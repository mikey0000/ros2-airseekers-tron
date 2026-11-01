import {describe, expect, it, vi} from "vitest";
import {render, screen} from "@testing-library/react";
import {freqToChannel, selectWifiStatus, wifiLevel, WifiChip, type WifiStatusView} from "./WifiChip.tsx";

let payload: WifiStatusView | null = null;
vi.mock("../hooks/useTopic.ts", () => ({useTopic: () => ({data: payload})}));
vi.mock("react-i18next", () => ({
    useTranslation: () => ({t: (k: string, o?: Record<string, unknown>) => (o ? `${k} ${JSON.stringify(o)}` : k)}),
}));

const RAW = {
    connected: true, ssid: "example-net", bssid: "02:00:5e:10:20:30", freq_mhz: 5180,
    signal_dbm: -63, tx_bitrate_mbps: 433.3, rx_bitrate_mbps: 390, tx_retries: 12,
    tx_failed: 1, roam_count: 3, link_state: "ok",
};

describe("WifiChip helpers", () => {
    it("selects the status payload", () => {
        const v = selectWifiStatus(RAW)!;
        expect(v.signalDbm).toBe(-63);
        expect(v.roamCount).toBe(3);
        expect(selectWifiStatus({data: "x"})).toBeUndefined();
        expect(selectWifiStatus({...RAW, connected: false, signal_dbm: NaN})!.signalDbm).toBeNull();
    });

    it("maps signal to four levels", () => {
        const at = (s: number) => wifiLevel({...selectWifiStatus(RAW)!, signalDbm: s});
        expect(at(-55)).toBe("good");
        expect(at(-60)).toBe("ok");
        expect(at(-70)).toBe("ok");
        expect(at(-75)).toBe("weak");
        expect(at(-79)).toBe("bad");
        expect(wifiLevel(selectWifiStatus({...RAW, connected: false})!)).toBe("bad");
    });

    it("converts frequency to channel", () => {
        expect(freqToChannel(2437)).toBe(6);
        expect(freqToChannel(5180)).toBe(36);
        expect(freqToChannel(0)).toBe(0);
    });
});

describe("WifiChip", () => {
    it("renders nothing before the first message", () => {
        payload = null;
        const {container} = render(<WifiChip/>);
        expect(container.firstChild).toBeNull();
    });

    it("shows dBm, level and tooltip", () => {
        payload = selectWifiStatus(RAW)!;
        render(<WifiChip/>);
        const chip = screen.getByTestId("wifi-chip");
        expect(chip.textContent).toContain("-63 dBm");
        expect(chip.dataset.level).toBe("ok");
        expect(chip.title).toContain("02:00:5e:10:20:30");
        expect(chip.title).toContain('"channel":36');
        expect(chip.title).toContain('"count":3');
    });

    it("shows disconnected", () => {
        payload = selectWifiStatus({...RAW, connected: false})!;
        render(<WifiChip/>);
        expect(screen.getByTestId("wifi-chip").dataset.level).toBe("disconnected");
    });
});
