import {useCallback} from "react";
import {useRobotProfile} from "./useRobotProfile.ts";
import {
    type GateId,
    isGateVisible,
    isProfileSettingKeyHidden,
} from "../constants/profileGates.ts";
import type {RobotProfile} from "../constants/robotProfiles.ts";

export type ProfileGates = {
    profile: RobotProfile;
    /** True until GET /api/robot/profile has answered. */
    loading: boolean;
    /**
     * Whether the gated UI should render. While the profile is loading the
     * stock profile answers, so stock robots never see their UI blink.
     */
    isVisible: (id: GateId) => boolean;
    /**
     * Like isVisible, but false until the profile has loaded. For things that
     * must not even start on a robot without the feature: nav entries, route
     * redirects, stream subscriptions.
     */
    isVisibleStrict: (id: GateId) => boolean;
    /** Settings key this robot does not use (its own list + missing features). */
    isSettingKeyHidden: (key: string) => boolean;
};

/** Profile gates of the active robot. See constants/profileGates.ts. */
export function useProfileGates(): ProfileGates {
    const {profile, loading} = useRobotProfile();
    const isVisible = useCallback((id: GateId) => isGateVisible(profile, id), [profile]);
    const isVisibleStrict = useCallback(
        (id: GateId) => !loading && isGateVisible(profile, id),
        [profile, loading],
    );
    const isSettingKeyHidden = useCallback(
        (key: string) => isProfileSettingKeyHidden(profile, key),
        [profile],
    );
    return {profile, loading, isVisible, isVisibleStrict, isSettingKeyHidden};
}

/** `true` when gate `id` is visible for the active robot (stock while loading). */
export function useGate(id: GateId): boolean {
    return useProfileGates().isVisible(id);
}
