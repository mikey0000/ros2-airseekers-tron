import React from "react";
import { Badge, Button, Space, Tag, Tooltip } from "antd";
import { UndoOutlined } from "@ant-design/icons";
import { useTranslation } from "react-i18next";
import { useRobotProfile } from "../../hooks/useRobotProfile.ts";
import { isKeyUsedByRobot } from "../../constants/paramBindings.ts";

type Props = {
    /** The parameter key this label is for (e.g. "mowing_speed"). */
    settingKey: string;
    /** The visible field label. */
    label: React.ReactNode;
    /** True when the current value differs from its schema default. */
    overridden?: boolean;
    /** True when a schema default exists for this key (reset is meaningful). */
    canReset?: boolean;
    /** Reset the field to its schema default. */
    onReset?: (key: string) => void;
};

/**
 * SettingFieldLabel wraps a Form.Item label with two default-awareness
 * affordances used by the sparse-config settings UI:
 *   - a small dot before the label when the value is operator-overridden
 *     (differs from the schema default), and
 *   - a subtle "reset to default" undo button after the label, shown only when
 *     the value is overridden AND a default exists.
 * When no default is known (canReset === false) it renders the label plainly,
 * so it is safe to use everywhere regardless of whether the key has a default.
 *
 * On a robot profile with a settings -> ROS parameter binding table
 * (constants/paramBindings.ts) a key that nothing on the robot reads gets a
 * "not used by this robot" tag; stock profiles never show it.
 */
export const SettingFieldLabel: React.FC<Props> = ({
    settingKey,
    label,
    overridden = false,
    canReset = false,
    onReset,
}) => {
    const { t } = useTranslation();
    const { profile } = useRobotProfile();
    const unused = !isKeyUsedByRobot(profile, settingKey);
    const showReset = canReset && overridden && !!onReset;
    const overriddenText = t("settingsReset.overridden", "Overridden — differs from default");
    return (
        <Space size={4} align="center">
            {overridden ? (
                <Tooltip title={overriddenText}>
                    <Badge
                        color="gold"
                        // The dot alone is a color-only signal; role="img" +
                        // aria-label give screen readers the same information
                        // the sighted hover tooltip provides.
                        role="img"
                        aria-label={overriddenText}
                    />
                </Tooltip>
            ) : null}
            <span>{label}</span>
            {unused ? (
                <Tooltip title={t("paramBindings.notUsedTooltip")}>
                    <Tag style={{ marginInlineEnd: 0, fontSize: 11, lineHeight: "16px" }}>
                        {t("paramBindings.notUsed")}
                    </Tag>
                </Tooltip>
            ) : null}
            {showReset ? (
                <Tooltip title={t("settingsReset.resetToDefault", "Reset to default")}>
                    <Button
                        type="text"
                        size="small"
                        aria-label={t("settingsReset.resetToDefault", "Reset to default")}
                        icon={<UndoOutlined style={{ fontSize: 11 }} />}
                        onClick={(e) => {
                            e.preventDefault();
                            onReset?.(settingKey);
                        }}
                        style={{ height: 18, width: 18, minWidth: 18, padding: 0, opacity: 0.65 }}
                    />
                </Tooltip>
            ) : null}
        </Space>
    );
};
