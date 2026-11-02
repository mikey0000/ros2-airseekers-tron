import {type ReactNode, useEffect, useState} from "react";
import {Button, Card, Space, Typography} from "antd";
import {CircleCheck, CircleDashed, CircleX} from "lucide-react";
import {useTranslation} from "react-i18next";
import {useSearchParams} from "react-router-dom";
import {useThemeMode} from "../../theme/ThemeContext.tsx";
import {hasPermittedMower} from "../../transport/ble/gattLink.ts";
import {useConnectionState} from "../../transport/connectionManager.ts";
import {fetchSecureOrigin, type SecureOriginStatus} from "./secureOrigin.ts";

const {Text, Link} = Typography;

type CheckResult = "ok" | "fail" | "unknown";

/** Bumps on window focus and when the tab becomes visible, so the checks below re-run. */
function useRecheckTick(): number {
    const [tick, setTick] = useState(0);
    useEffect(() => {
        const bump = () => setTick((n) => n + 1);
        const onVisibility = () => {
            if (document.visibilityState === "visible") bump();
        };
        window.addEventListener("focus", bump);
        document.addEventListener("visibilitychange", onVisibility);
        return () => {
            window.removeEventListener("focus", bump);
            document.removeEventListener("visibilitychange", onVisibility);
        };
    }, []);
    return tick;
}

/** The mower's HTTPS addresses, fetched only while the page is not on a secure origin. */
function useSecureOriginStatus(secure: boolean, tick: number): SecureOriginStatus | null {
    const [status, setStatus] = useState<SecureOriginStatus | null>(null);
    useEffect(() => {
        if (secure) return;
        const controller = new AbortController();
        fetchSecureOrigin(controller.signal)
            .then(setStatus)
            .catch(() => {
                // The row still points to Remote access; the addresses are a shortcut.
            });
        return () => controller.abort();
    }, [secure, tick]);
    return secure ? null : status;
}

/** The HTTPS address with the current in-app route kept, so the user lands back on this screen. */
function httpsLink(base: string): string {
    try {
        const url = new URL(base);
        url.hash = window.location.hash;
        return url.toString();
    } catch {
        return base;
    }
}

interface GuideRowProps {
    result: CheckResult;
    title: string;
    hint: string | null;
    testId: string;
    children?: ReactNode;
}

function GuideRow({result, title, hint, testId, children}: GuideRowProps) {
    const {colors} = useThemeMode();
    const icon = result === "ok"
        ? <CircleCheck size={18} color={colors.success} aria-label="ok"/>
        : result === "fail"
            ? <CircleX size={18} color={colors.danger} aria-label="missing"/>
            : <CircleDashed size={18} color={colors.textSecondary} aria-label="unknown"/>;
    return (
        <div data-testid={testId} data-result={result} style={{display: "flex", gap: 10, alignItems: "flex-start"}}>
            <span style={{flex: "none", lineHeight: 0, paddingTop: 2}}>{icon}</span>
            <div style={{minWidth: 0}}>
                <Text strong>{title}</Text>
                {result !== "ok" && hint && (
                    <Text type="secondary" style={{display: "block", fontSize: 12}}>{hint}</Text>
                )}
                {result !== "ok" && children}
            </div>
        </div>
    );
}

/**
 * Settings → Connection, first card: the four things Bluetooth needs (a browser with Web
 * Bluetooth, a secure origin, the BLE token, a paired mower), each re-checked live with the
 * next step. Collapses to one line once all pass.
 */
export function BluetoothGuideCard() {
    const {t} = useTranslation();
    const {colors} = useThemeMode();
    const state = useConnectionState();
    const [, setSearchParams] = useSearchParams();
    const tick = useRecheckTick();

    const secure = window.isSecureContext;
    const hasBluetooth = typeof navigator !== "undefined" && navigator.bluetooth !== undefined;
    const secureOrigin = useSecureOriginStatus(secure, tick);

    // null: the browser cannot list permitted devices; fall back to the transport's own record.
    const [permitted, setPermitted] = useState<boolean | null>(null);
    useEffect(() => {
        let cancelled = false;
        void hasPermittedMower(state.deviceId).then((value) => {
            if (!cancelled) setPermitted(value);
        });
        return () => {
            cancelled = true;
        };
    }, [state.deviceId, state.ble.link, hasBluetooth, tick]);

    // Chrome hides navigator.bluetooth on an insecure origin, so its absence there says nothing.
    const browser: CheckResult = hasBluetooth ? "ok" : secure ? "fail" : "unknown";
    const origin: CheckResult = secure ? "ok" : "fail";
    const tokenRejected = state.token !== null && state.ble.session === "unauthenticated";
    const token: CheckResult = state.token !== null && !tokenRejected ? "ok" : "fail";
    const paired: CheckResult = state.ble.link === "connected" || (permitted ?? state.deviceId !== null) ? "ok" : "fail";

    if ([browser, origin, token, paired].every((r) => r === "ok")) {
        return (
            <div data-testid="ble-guide-ready" style={{display: "flex", gap: 8, alignItems: "center", marginBottom: 16}}>
                <CircleCheck size={16} color={colors.success} aria-hidden/>
                <Text>{t("bleGuide.ready")}</Text>
            </div>
        );
    }

    const openRemoteAccess = () => setSearchParams((current) => {
        const next = new URLSearchParams(current);
        next.set("section", "remote_access");
        return next;
    }, {replace: true});
    const httpsUrls = secureOrigin?.enabled ? secureOrigin.https_urls : [];

    return (
        <Card
            size="small"
            title={t("bleGuide.title")}
            style={{marginBottom: 16, borderColor: colors.warning}}
            data-testid="ble-guide"
        >
            <Space direction="vertical" size={12} style={{width: "100%"}}>
                <GuideRow
                    testId="ble-guide-browser"
                    result={browser}
                    title={t("bleGuide.browser.title")}
                    hint={t(browser === "unknown" ? "bleGuide.browser.unknown" : "bleGuide.browser.fail")}
                />
                <GuideRow testId="ble-guide-origin" result={origin} title={t("bleGuide.origin.title")} hint={t("bleGuide.origin.fail")}>
                    {httpsUrls.length > 0 && (
                        <Space direction="vertical" size={0} style={{marginTop: 4}} data-testid="ble-guide-https-urls">
                            {httpsUrls.map((url) => (
                                <Link key={url} href={httpsLink(url)} style={{fontSize: 12, wordBreak: "break-all"}}>{url}</Link>
                            ))}
                        </Space>
                    )}
                    <Button size="small" type="link" style={{padding: 0, height: "auto", fontSize: 12, display: "block"}} onClick={openRemoteAccess}>
                        {t("bleGuide.origin.openRemoteAccess")}
                    </Button>
                </GuideRow>
                <GuideRow
                    testId="ble-guide-token"
                    result={token}
                    title={t("bleGuide.token.title")}
                    hint={t(tokenRejected ? "bleGuide.token.rejected" : "bleGuide.token.fail")}
                />
                <GuideRow testId="ble-guide-paired" result={paired} title={t("bleGuide.paired.title")} hint={t("bleGuide.paired.fail")}/>
            </Space>
        </Card>
    );
}
