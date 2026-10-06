import React, {useState} from "react";
import {useNavigate} from "react-router-dom";
import {useTranslation} from "react-i18next";
import {App, Button, Card, Descriptions, Popconfirm, Space, Tag, Typography} from "antd";
import {AimOutlined, EnvironmentOutlined, HomeOutlined} from "@ant-design/icons";
import {useThemeMode} from "../../theme/ThemeContext.tsx";
import {useApi} from "../../hooks/useApi.ts";
import {type CalibrationStatus, useCalibrationStatus} from "../../hooks/useCalibrationStatus.ts";
import {type DockPose, resolveDockPose} from "./visionDockPose.ts";

const {Text, Paragraph} = Typography;

/** SetDockingPoint yaw_source PRESERVE: keep the stored dock heading. */
const YAW_SOURCE_PRESERVE = 0;

const round = (value: number, digits: number) => {
    const f = 10 ** digits;
    return Math.round(value * f) / f;
};

type Props = {
    /** Settings values (dock_pose_x/y/yaw); used until the live status loads. */
    values?: Record<string, unknown>;
    /** When given, the stored pose is written back into the settings form so a
     *  later Save cannot overwrite it with a stale copy. */
    onChange?: (key: string, value: unknown) => void;
};

/**
 * VisionDockCard — dock pose for robots that find their dock with a camera
 * marker (profile.docking === "vision_marker") instead of the CalibrateDock
 * drive onto charger contacts. The robot only needs to know roughly where the
 * dock is; the marker does the final alignment. Two ways to set it, both
 * through map_server's set_docking_point (POST /mowglinext/map/docking):
 *  - place and rotate the dock marker on the map (map editor), or
 *  - with the robot sitting on its dock, take its current position.
 */
export const VisionDockCard: React.FC<Props> = ({values, onChange}) => {
    const {t} = useTranslation();
    const {colors} = useThemeMode();
    const {notification} = App.useApp();
    const navigate = useNavigate();
    const guiApi = useApi();
    const {status, refresh} = useCalibrationStatus();
    const [setting, setSetting] = useState(false);

    const pose = resolveDockPose(status?.dock, values);

    const setFromRobotPose = async () => {
        setSetting(true);
        try {
            const res = await guiApi.mowglinext.mapDockingCreate({
                // Ignored for the position (use_gps_position) and the heading
                // (PRESERVE); the request shape needs a pose anyway.
                docking_pose: {
                    orientation: {x: 0, y: 0, z: 0, w: 1},
                    position: {x: 0, y: 0, z: 0},
                },
                // map_server averages the robot's recent localized position.
                use_gps_position: true,
                // A standing robot gives no trustworthy heading; keep the
                // stored one (set it on the map).
                yaw_source: YAW_SOURCE_PRESERVE,
            });
            if (res.error) throw new Error(res.error.error);
            // Read back what map_server actually stored (it may differ from
            // anything we could compute here).
            let stored: DockPose | null = null;
            try {
                const fresh = await guiApi.request<CalibrationStatus>({
                    path: "/calibration/status",
                    method: "GET",
                    format: "json",
                });
                stored = resolveDockPose(fresh.data?.dock, undefined);
            } catch {
                // The pose is persisted regardless; the card's poll catches up.
            }
            void refresh();
            if (stored && onChange) {
                onChange("dock_pose_x", round(stored.x, 3));
                onChange("dock_pose_y", round(stored.y, 3));
                onChange("dock_pose_yaw", round(stored.yawRad, 4));
            }
            notification.success({
                message: t("visionDock.setFromRobotDone"),
                description: stored
                    ? t("visionDock.storedPose", {x: stored.x.toFixed(2), y: stored.y.toFixed(2)})
                    : undefined,
            });
        } catch (e: unknown) {
            notification.error({
                message: t("visionDock.setFromRobotFailed"),
                description: e instanceof Error ? e.message : String(e),
            });
        } finally {
            setSetting(false);
        }
    };

    return (
        <Card size="small" style={{marginBottom: 16}} data-testid="vision-dock-card">
            <Space direction="vertical" size={12} style={{width: "100%"}}>
                <div>
                    <Text strong className="mn-display" style={{fontSize: 14, color: colors.text}}>
                        <HomeOutlined style={{marginRight: 6, color: colors.primary}}/>
                        {t("visionDock.title")}
                    </Text>
                    <Paragraph type="secondary" style={{margin: "4px 0 0"}}>
                        {t("visionDock.description")}
                    </Paragraph>
                </div>

                {pose ? (
                    <Descriptions size="small" column={3} bordered>
                        <Descriptions.Item label="X">{pose.x.toFixed(2)} m</Descriptions.Item>
                        <Descriptions.Item label="Y">{pose.y.toFixed(2)} m</Descriptions.Item>
                        <Descriptions.Item label={t("visionDock.heading")}>
                            {(pose.yawRad * 180 / Math.PI).toFixed(1)}°
                        </Descriptions.Item>
                    </Descriptions>
                ) : (
                    <Tag color="warning" style={{alignSelf: "flex-start"}}>{t("visionDock.notSet")}</Tag>
                )}

                <Space wrap>
                    <Button icon={<EnvironmentOutlined/>} onClick={() => void navigate("/map")}>
                        {t("visionDock.placeOnMap")}
                    </Button>
                    <Popconfirm
                        title={t("visionDock.setFromRobotConfirmTitle")}
                        description={t("visionDock.setFromRobotConfirm")}
                        okText={t("visionDock.setFromRobot")}
                        onConfirm={() => { void setFromRobotPose(); }}
                    >
                        <Button type="primary" icon={<AimOutlined/>} loading={setting}>
                            {t("visionDock.setFromRobot")}
                        </Button>
                    </Popconfirm>
                </Space>
                <Text type="secondary" style={{fontSize: 12}}>
                    {t("visionDock.placeOnMapHint")}
                </Text>
            </Space>
        </Card>
    );
};

export default VisionDockCard;
