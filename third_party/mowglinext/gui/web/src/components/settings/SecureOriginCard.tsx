import {useEffect, useState} from "react";
import {Alert, Button, Card, Collapse, Space, Spin, Tag, Typography} from "antd";
import {CheckCircleOutlined, DownloadOutlined, LockOutlined, WarningOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import {apiFetch, networkHttpBase} from "../../transport/index.ts";

const {Text, Paragraph, Link} = Typography;

/** GET /api/secure-origin (pkg/api/secure_origin.go). */
export interface SecureOriginStatus {
    enabled: boolean;
    https_urls: string[];
    hostname: string;
    ca_fingerprint_sha256?: string;
    ca_not_after?: string;
}

const SECURE_ORIGIN_CA_PATH = "/api/secure-origin/ca.crt";

/** The card's only backend call, kept in one place for the transport seam. */
async function fetchSecureOrigin(signal?: AbortSignal): Promise<SecureOriginStatus> {
    const res = await apiFetch("/api/secure-origin", {signal, cache: "no-store"});
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    return await res.json() as SecureOriginStatus;
}

const INSTALL_GUIDES = [
    {key: "android", steps: ["step1", "step2", "step3"]},
    {key: "desktop", steps: ["step1", "step2", "step3"]},
] as const;

/**
 * How to reach the GUI on a secure origin: the mower serves HTTPS with a
 * certificate from its own CA, which each phone installs once.
 */
export function SecureOriginCard() {
    const {t} = useTranslation();
    const [status, setStatus] = useState<SecureOriginStatus | null>(null);
    const [error, setError] = useState<string | null>(null);
    const secure = window.isSecureContext;

    useEffect(() => {
        const controller = new AbortController();
        fetchSecureOrigin(controller.signal)
            .then(setStatus)
            .catch((e: unknown) => {
                if (!controller.signal.aborted) setError(e instanceof Error ? e.message : String(e));
            });
        return () => controller.abort();
    }, []);

    return (
        <Card
            size="small"
            style={{marginBottom: 16}}
            title={<span><LockOutlined style={{marginRight: 6}}/>{t("secureOrigin.title")}</span>}
            data-testid="secure-origin-card"
        >
            <Paragraph type="secondary" style={{margin: "0 0 8px"}}>{t("secureOrigin.why")}</Paragraph>
            <Tag
                color={secure ? "success" : "warning"}
                icon={secure ? <CheckCircleOutlined/> : <WarningOutlined/>}
                style={{marginBottom: 8, whiteSpace: "normal"}}
                data-testid="secure-origin-verdict"
            >
                {secure ? t("secureOrigin.onSecureOrigin") : t("secureOrigin.notOnSecureOrigin")}
            </Tag>

            {error && <Alert type="warning" showIcon style={{marginBottom: 8}} message={t("secureOrigin.loadFailed")} description={error}/>}
            {!status && !error && <Spin size="small" style={{display: "block", margin: "8px 0"}}/>}
            {status && !status.enabled && (
                <Alert type="warning" showIcon style={{marginBottom: 8}} message={t("secureOrigin.disabled")}/>
            )}

            {status?.enabled && (
                <>
                    <Paragraph style={{margin: "0 0 4px"}}>{t("secureOrigin.openAt")}</Paragraph>
                    <Space direction="vertical" size={2} style={{marginBottom: 8}} data-testid="secure-origin-urls">
                        {status.https_urls.map((url, i) => (
                            <Link key={url} href={url} target="_blank" rel="noopener noreferrer" strong={i === 0} copyable>
                                {url}
                            </Link>
                        ))}
                    </Space>
                    <Space wrap align="center" style={{display: "flex", marginBottom: 8}}>
                        <Button
                            icon={<DownloadOutlined/>}
                            href={`${networkHttpBase()}${SECURE_ORIGIN_CA_PATH}`}
                            download="mower-ca.crt"
                            data-testid="secure-origin-download"
                        >
                            {t("secureOrigin.downloadCa")}
                        </Button>
                        {status.ca_not_after && (
                            <Text type="secondary" style={{fontSize: 12}}>
                                {t("secureOrigin.caExpires", {date: new Date(status.ca_not_after).toLocaleDateString()})}
                            </Text>
                        )}
                    </Space>
                    {status.ca_fingerprint_sha256 && (
                        <Paragraph style={{margin: "0 0 8px", fontSize: 12}}>
                            <Text type="secondary">{t("secureOrigin.fingerprint")} </Text>
                            <Text code copyable style={{wordBreak: "break-all"}}>{status.ca_fingerprint_sha256}</Text>
                        </Paragraph>
                    )}
                    <Collapse
                        size="small"
                        items={[
                            ...INSTALL_GUIDES.map(({key, steps}) => ({
                                key,
                                label: t(`secureOrigin.${key}.title`),
                                children: (
                                    <ol style={{margin: 0, paddingLeft: 18}}>
                                        {steps.map((step) => <li key={step}>{t(`secureOrigin.${key}.${step}`)}</li>)}
                                    </ol>
                                ),
                            })),
                            {key: "ios", label: t("secureOrigin.ios.title"), children: <span>{t("secureOrigin.ios.note")}</span>},
                        ]}
                    />
                </>
            )}
        </Card>
    );
}
