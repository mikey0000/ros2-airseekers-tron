import type {ReactNode} from "react";
import {Spin} from "antd";
import {useProfileGates} from "../hooks/useProfileGates.ts";
import type {GateId} from "../constants/profileGates.ts";
import {NotFoundPage} from "./RouteFallbacks.tsx";

/**
 * Route element wrapper for a profile-gated page: a robot without the feature
 * gets the shell's 404 (as if the route did not exist) instead of a page that
 * drives hardware it does not have. Waits for the profile before deciding.
 */
export function ProfileRoute({gate, children}: {gate: GateId; children: ReactNode}) {
    const {loading, isVisible} = useProfileGates();
    if (loading) {
        return <div style={{display: "flex", justifyContent: "center", padding: 48}}><Spin size="large"/></div>;
    }
    return isVisible(gate) ? <>{children}</> : <NotFoundPage/>;
}
