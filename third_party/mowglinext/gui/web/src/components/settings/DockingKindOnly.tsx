import React from "react";
import { useRobotProfile } from "../../hooks/useRobotProfile.ts";
import type { DockingKind } from "../../constants/robotProfiles.ts";

/**
 * Renders its children only on a robot that docks the given way. Settings
 * that only make sense for charger-contact docking (charger current
 * detection, contact overshoot) disappear on a vision-marker robot instead of
 * offering a knob nothing reads.
 */
export const DockingKindOnly: React.FC<{ kind: DockingKind; children: React.ReactNode }> = ({ kind, children }) => {
    const { profile } = useRobotProfile();
    return profile.docking === kind ? <>{children}</> : null;
};
