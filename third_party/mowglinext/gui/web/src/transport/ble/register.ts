/**
 * Side-effect module imported once from main.tsx: registers the `ble` transport with the seam
 * and starts the connection manager (token fetch, liveness probe, the one automatic switch).
 */
import {registerTransport} from "../registry.ts";
import {getConnectionManager} from "../connectionManager.ts";
import {getBleTransport} from "./instance.ts";

registerTransport("ble", getBleTransport);
getConnectionManager().start();
