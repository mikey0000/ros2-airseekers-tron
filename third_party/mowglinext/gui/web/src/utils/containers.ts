import { Api, ApiContainer } from "../api/Api.ts";

type GuiApi = Api<unknown>;

type ContainerMatch = {
    /** Match by container name substring (checked against all names in the names array) */
    name?: string;
    /** Match by Docker label key=value */
    label?: { key: string; value: string };
};

/**
 * Find a running container by name or label and execute a command on it.
 * Throws on failure so callers can catch and show notifications.
 */
export const containerAction = async (
    api: GuiApi,
    match: ContainerMatch,
    action: "restart" | "start" | "stop",
): Promise<void> => {
    const res = await api.containers.containersList();
    if (res.error) throw new Error(res.error.error);

    const container = res.data.containers?.find((c: any) => {
        if (match.name && c.names?.some((n: string) => n.includes(match.name!))) return true;
        if (match.label && c.labels?.[match.label.key] === match.label.value) return true;
        return false;
    });

    if (!container?.id) {
        throw new Error(`Container not found (match: ${match.name ?? match.label?.value})`);
    }

    const cmdRes = await api.containers.containersCreate(container.id, action);
    if (cmdRes.error) throw new Error(cmdRes.error.error);
};

const bare = (n: string) => n.replace(/^\//, "");

const findByName = (containers: ApiContainer[], name?: string) =>
    name ? containers.find((c) => !!c.id && (c.names ?? []).some((n) => bare(n) === name)) : undefined;

/** Restart a stack container whose configured name the backend reports. */
const restartNamed = async (api: GuiApi, which: "ros" | "gps" | "gui"): Promise<void> => {
    const res = await api.containers.containersList();
    if (res.error) throw new Error(res.error.error);
    const name = res.data.names?.[which];
    if (!name) throw new Error(`No ${which} container is configured on this robot`);
    const container = findByName(res.data.containers ?? [], name);
    if (!container?.id) throw new Error(`Container not found (${name})`);
    const cmdRes = await api.containers.containersCreate(container.id, "restart");
    if (cmdRes.error) throw new Error(cmdRes.error.error);
};

/** Restart the ROS2 container (name from the backend: ROS_CONTAINER_NAME) */
export const restartRos2 = (api: GuiApi) => restartNamed(api, "ros");

/** Restart the GUI container (GUI_CONTAINER_NAME) */
export const restartGui = (api: GuiApi) => restartNamed(api, "gui");

/**
 * Restart the whole stack: the configured ROS, GPS (if any) and GUI
 * containers, names supplied by the backend (ROS_/GPS_/GUI_CONTAINER_NAME).
 *
 * The GUI container is restarted LAST and fire-and-forget: restarting it
 * kills the backend serving this very request, so the response never
 * returns. The caller reloads the browser once the GUI is back.
 */
export const restartMowgliStack = async (api: GuiApi): Promise<void> => {
    const res = await api.containers.containersList();
    if (res.error) throw new Error(res.error.error);

    const containers = res.data.containers ?? [];
    const names = res.data.names ?? {};
    const others = [names.ros, names.gps]
        .map((n) => findByName(containers, n))
        .filter((c): c is ApiContainer => !!c);
    const gui = findByName(containers, names.gui);
    if (others.length === 0 && !gui) throw new Error("No stack containers found");

    await Promise.all(others.map((c) => api.containers.containersCreate(c.id!, "restart")));
    if (gui?.id) void api.containers.containersCreate(gui.id, "restart");
};

/** Restart the GNSS receiver container (picks up new NTRIP / serial config) */
export const restartGps = (api: GuiApi) => restartNamed(api, "gps");

/**
 * Settings keys whose values are consumed directly by the GNSS receiver
 * container on a plain restart. The vendor-neutral profile/signal-profile
 * keys are intentionally excluded: a container restart only re-launches the
 * driver with new serial/NTRIP transport — it never re-flashes the receiver.
 * The signal profile is a receiver-flash setting that only reaches the
 * receiver through the Expert-mode Plan & Apply flow (POST /settings/gnss/apply,
 * which runs gnss_config_apply --signal-profile). Saving from the basic view
 * persists intent for that next apply; only serial/NTRIP transport changes
 * require an immediate mowgli-gps restart.
 */
export const GPS_RESTART_KEYS = new Set<string>([
    "gnss_receiver_family",
    "gnss_serial_device",
    "gnss_serial_baud",
    "ntrip_enabled",
    "ntrip_host",
    "ntrip_port",
    "ntrip_user",
    "ntrip_password",
    "ntrip_mountpoint",
]);

/** True if any dirty key affects the GPS container. */
export const dirtyKeysRequireGpsRestart = (dirty: Iterable<string>): boolean => {
    for (const k of dirty) if (GPS_RESTART_KEYS.has(k)) return true;
    return false;
};
