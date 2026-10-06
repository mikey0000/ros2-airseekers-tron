import {useCallback, useEffect, useRef, useState} from "react";
import {wsBase} from "../utils/apiHost";
import {
    getMultiplexedSocket,
    isMultiplexableSubscribeUri,
    topicFromSubscribeUri,
    type MultiplexStatus,
} from "./multiplexedSocket.ts";
import {PublishSocket, type PublishSocketStatus} from "./reconnect.ts";

/**
 * useMultiplexStatus — live connection status of the shared multiplex
 * WebSocket. Lets the app shell render a truthful offline indicator instead
 * of assuming the stream is up. "closed" covers both idle (no subscribers)
 * and a dropped connection awaiting reconnect.
 */
export const useMultiplexStatus = (): MultiplexStatus => {
    const [status, setStatus] = useState<MultiplexStatus>(() => getMultiplexedSocket().getStatus());
    useEffect(() => getMultiplexedSocket().onStatusChange(setStatus), []);
    return status;
};

/**
 * useMultiplexReconnecting — true while the shared status socket is down after
 * having been connected (drives the "Reconnecting…" badge). False before the
 * first connection so a cold page load does not flash the badge.
 */
export const useMultiplexReconnecting = (): boolean => {
    const status = useMultiplexStatus();
    return status !== "open" && getMultiplexedSocket().wasEverOpen();
};

/**
 * useWS — start/stop a stream of payloads from the GUI backend.
 *
 * Subscribe URIs (`/api/mowglinext/subscribe/<topic>`) are multiplexed
 * over a single shared WebSocket via {@link getMultiplexedSocket}.
 * Other URIs (e.g. `/api/mowglinext/publish/joy`) keep their dedicated
 * connection because they need bidirectional traffic that the
 * multiplex protocol does not handle yet.
 *
 * The signature is unchanged from before #177 so every existing hook
 * keeps working without modification.
 */
export const useWS = <T>(
    onError: (e: Error) => void,
    onInfo: (msg: string) => void,
    onData: (data: T, first?: boolean) => void,
) => {
    // Refs to always invoke the latest callbacks, avoiding stale closures.
    const onDataRef = useRef(onData);
    onDataRef.current = onData;
    const onErrorRef = useRef(onError);
    onErrorRef.current = onError;
    const onInfoRef = useRef(onInfo);
    onInfoRef.current = onInfo;

    // Active multiplex unsubscribe + status-listener unregister (set when
    // start() targets a subscribe URI).
    const muxUnsubscribeRef = useRef<(() => void) | null>(null);
    const muxStatusUnsubRef = useRef<(() => void) | null>(null);

    // Publish-side socket (only used for non-subscribe URIs, i.e. the
    // joystick). See PublishSocket: fast reconnect, liveness watchdog, and
    // send() drops (never queues) while disconnected.
    const pubRef = useRef<PublishSocket | null>(null);
    const pubStatusUnsubRef = useRef<(() => void) | null>(null);
    const [pubStatus, setPubStatus] = useState<PublishSocketStatus>("idle");
    useEffect(() => () => {
        pubStatusUnsubRef.current?.();
        pubRef.current?.stop();
    }, []);
    const sendJsonMessage = useCallback((msg: unknown) => {
        pubRef.current?.send(msg);
    }, []);

    const teardown = () => {
        if (muxUnsubscribeRef.current) {
            muxUnsubscribeRef.current();
            muxUnsubscribeRef.current = null;
        }
        if (muxStatusUnsubRef.current) {
            muxStatusUnsubRef.current();
            muxStatusUnsubRef.current = null;
        }
        if (pubRef.current) {
            pubStatusUnsubRef.current?.();
            pubStatusUnsubRef.current = null;
            pubRef.current.stop();
            pubRef.current = null;
            setPubStatus("idle");
        }
    };

    const start = (uri: string) => {
        // Re-starting the same publish stream (every RECORDING/MANUAL_MOWING
        // state frame calls start) must keep the live socket, not reconnect.
        if (pubRef.current && pubRef.current.url === `${wsBase()}${uri}`) return;
        teardown();

        if (isMultiplexableSubscribeUri(uri)) {
            const topic = topicFromSubscribeUri(uri);
            let firstReported = false;
            muxUnsubscribeRef.current = getMultiplexedSocket().subscribe(
                topic,
                (data, isFirst) => {
                    if (isFirst && !firstReported) {
                        firstReported = true;
                        onInfoRef.current("Stream connected");
                    }
                    onDataRef.current(data as T, isFirst);
                },
            );
            // Surface shared-socket drops/reconnects through the caller's
            // handlers so they are no longer dead code on the multiplex path.
            let sawClose = false;
            muxStatusUnsubRef.current = getMultiplexedSocket().onStatusChange((status) => {
                if (status === "closed") {
                    sawClose = true;
                    onErrorRef.current(new Error("Stream closed"));
                } else if (status === "open" && sawClose) {
                    sawClose = false;
                    onInfoRef.current("Stream connected");
                }
            });
            return;
        }

        // Publish path: dedicated socket with automatic reconnect.
        const sock = new PublishSocket(`${wsBase()}${uri}`);
        pubRef.current = sock;
        pubStatusUnsubRef.current = sock.onStatusChange((status) => {
            setPubStatus(status);
            if (status === "open") onInfoRef.current("Stream connected");
            else if (status === "reconnecting") onErrorRef.current(new Error("Stream closed"));
        });
        sock.start();
    };

    const stop = () => {
        teardown();
    };

    return {start, stop, sendJsonMessage, status: pubStatus};
};
