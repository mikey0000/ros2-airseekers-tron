import {useEffect, useState} from "react";
import {Button, Card, InputNumber, Space, Typography} from "antd";
import {useTranslation} from "react-i18next";
import AsyncButton from "../../../components/AsyncButton.tsx";

/** ENU degrees in [0, 360) from a heading in radians (0 = East, CCW positive). */
export const headingRadToDeg = (rad: number): number => {
    const d = (rad * 180) / Math.PI;
    return Math.round((((d % 360) + 360) % 360) * 10) / 10;
};

/** Heading in radians, wrapped to (-pi, pi], from ENU degrees. */
export const headingDegToRad = (deg: number): number => {
    const r = (deg * Math.PI) / 180;
    return Math.atan2(Math.sin(r), Math.cos(r));
};

/** Desktop: clear of the 240 px Areas side panel (right 16) plus a 12 px gap. */
export const DESKTOP_SIDE_PANEL_CLEARANCE = 16 + 240 + 12;

const COMPASS = ["E", "NE", "N", "NW", "W", "SW", "S", "SE"] as const;

/** Nearest compass point of an ENU heading (0 = East, counter-clockwise). */
export const compassPoint = (deg: number): typeof COMPASS[number] =>
    COMPASS[Math.round((((deg % 360) + 360) % 360) / 45) % 8];

interface DockHeadingPanelProps {
    /** Dock marker heading (rad, ENU). */
    heading: number;
    /** Live edit of the marker heading (rad). */
    onChange: (headingRad: number) => void;
    /** Store the dock pose now (set_docking_point, yaw_source REQUEST, position kept). */
    onApply: () => Promise<void>;
    mobile?: boolean;
}

/**
 * Edit Map: numeric dock heading. Typing rotates the dock marker; Apply calls
 * set_docking_point with the marker position unchanged and this heading.
 */
export const DockHeadingPanel = ({heading, onChange, onApply, mobile}: DockHeadingPanelProps) => {
    const {t} = useTranslation();
    const [value, setValue] = useState<number>(headingRadToDeg(heading));
    // Starts collapsed to one small button (opened on demand) so entering Edit
    // Map never pops a card over the map or the side panel.
    const [expanded, setExpanded] = useState<boolean>(false);
    const anchor: React.CSSProperties = mobile
        ? {position: "absolute", top: 64, right: 12, zIndex: 20}
        : {position: "absolute", bottom: 16, right: DESKTOP_SIDE_PANEL_CLEARANCE, zIndex: 20};
    useEffect(() => { setValue(headingRadToDeg(heading)); }, [heading]);

    const commit = (v: number | null) => {
        if (v === null || !Number.isFinite(v)) return;
        setValue(v);
        onChange(headingDegToRad(v));
    };
    const step = (delta: number) => commit(headingRadToDeg(headingDegToRad(value + delta)));

    if (!expanded) {
        return (
            <Button
                aria-label={t("dockHeading.title")}
                onClick={() => setExpanded(true)}
                style={{...anchor, minHeight: mobile ? 44 : undefined}}
            >
                {t("dockHeading.short", {deg: value.toFixed(0)})}
            </Button>
        );
    }

    return (
        <Card
            size="small"
            title={t("dockHeading.title")}
            extra={
                <Button type="text" size="small" aria-label={t("dockHeading.collapse")}
                        onClick={() => setExpanded(false)}>✕</Button>
            }
            role="group"
            aria-label={t("dockHeading.title")}
            style={{...anchor, width: mobile ? 220 : 260}}
            styles={{body: {padding: 8}}}
        >
            <Space.Compact style={{width: "100%"}}>
                <Button aria-label={t("dockHeading.rotateLeft")} onClick={() => step(5)}>↺</Button>
                <InputNumber
                    aria-label={t("dockHeading.label")}
                    min={0}
                    max={359.9}
                    step={1}
                    precision={1}
                    value={value}
                    onChange={(v) => commit(typeof v === "number" ? v : null)}
                    suffix="°"
                    inputMode="decimal"
                    style={{flex: 1}}
                />
                <Button aria-label={t("dockHeading.rotateRight")} onClick={() => step(-5)}>↻</Button>
            </Space.Compact>
            <Typography.Text type="secondary" style={{display: "block", fontSize: 12, marginTop: 4}}>
                {t("dockHeading.compassHint", {point: t(`dockHeading.compass.${compassPoint(value)}`)})}
            </Typography.Text>
            <Typography.Text type="secondary" style={{display: "block", fontSize: 12, marginTop: 4}}>
                {t("dockHeading.alignHint")}
            </Typography.Text>
            <AsyncButton type="primary" block style={{marginTop: 8, minHeight: mobile ? 44 : undefined}}
                         onAsyncClick={onApply}>
                {t("dockHeading.apply")}
            </AsyncButton>
        </Card>
    );
};
