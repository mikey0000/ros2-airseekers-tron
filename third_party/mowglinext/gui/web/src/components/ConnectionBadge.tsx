import {useTranslation} from "react-i18next";
import {useMultiplexReconnecting} from "../hooks/useWS.ts";

/**
 * "Reconnecting…" pill shown while the shared status WebSocket is down after
 * having been connected (phone roaming at the edge of the Wi-Fi). The last
 * known robot state stays on screen; the socket reconnects on its own within
 * ~1 s of the link coming back and re-delivers the latest state.
 */
export function ConnectionBadge({label}: {label?: string}) {
    const {t} = useTranslation();
    const reconnecting = useMultiplexReconnecting();
    if (!reconnecting && !label) return null;
    return (
        <div role="status" aria-live="polite" data-testid="connection-badge" style={{
            position: "fixed",
            top: "calc(env(safe-area-inset-top, 0px) + 8px)",
            left: "50%",
            transform: "translateX(-50%)",
            zIndex: 2000,
            padding: "6px 14px",
            borderRadius: 999,
            background: "rgba(250, 173, 20, 0.95)",
            color: "#1f1f1f",
            fontSize: 13,
            fontWeight: 600,
            boxShadow: "0 2px 8px rgba(0,0,0,0.3)",
            pointerEvents: "none",
        }}>
            {label ?? t("connection.reconnecting", "Reconnecting…")}
        </div>
    );
}
