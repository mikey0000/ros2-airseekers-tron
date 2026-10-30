import {BleTransport} from "./bleTransport.ts";
import {GattLink, webBluetoothSupport} from "./gattLink.ts";

let instance: BleTransport | null = null;

/** The app's single BLE transport, created on first use. */
export function getBleTransport(): BleTransport {
    if (!instance) {
        const bluetooth = webBluetoothSupport() === "ok" ? navigator.bluetooth : undefined;
        const buildId = (import.meta.env.VITE_WEB_BUILD_ID as string | undefined) ?? "dev";
        instance = new BleTransport(new GattLink(bluetooth), `mowglinext-web/${buildId}`);
    }
    return instance;
}
