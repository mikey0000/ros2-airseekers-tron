// Dock pose shown by VisionDockCard (pure, unit-tested).

export type DockPose = {x: number; y: number; yawRad: number};

function finite(value: unknown): number | undefined {
    return typeof value === "number" && Number.isFinite(value) ? value : undefined;
}

/**
 * The dock pose to show: the one persisted in mowgli_robot.yaml (read live via
 * /calibration/status), else the settings form's copy, else none. A pose of
 * exactly (0, 0) is the "never set" placeholder.
 */
export function resolveDockPose(
    status: {present: boolean; dock_pose_x?: number; dock_pose_y?: number; dock_pose_yaw_rad?: number} | undefined,
    values: Record<string, unknown> | undefined,
): DockPose | null {
    const fromStatus = status?.present
        ? {x: finite(status.dock_pose_x) ?? 0, y: finite(status.dock_pose_y) ?? 0, yawRad: finite(status.dock_pose_yaw_rad) ?? 0}
        : null;
    const x = finite(values?.dock_pose_x);
    const y = finite(values?.dock_pose_y);
    const fromValues = x !== undefined && y !== undefined
        ? {x, y, yawRad: finite(values?.dock_pose_yaw) ?? 0}
        : null;
    const pose = fromStatus ?? fromValues;
    if (!pose || (pose.x === 0 && pose.y === 0)) return null;
    return pose;
}
