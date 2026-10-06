import {describe, expect, it} from "vitest";
import {fireEvent, render, screen} from "@testing-library/react";
import {ThemeProvider} from "../theme/ThemeContext.tsx";
import {AIRSEEKERS_TRON_PROFILE_ID, getRobotProfile} from "../constants/robotProfiles.ts";
import {RobotAnatomy, anatomyParts, type AnatomyInputs} from "./RobotAnatomy.tsx";

const inputs: AnatomyInputs = {
    batteryPct: 80, vBattery: 27, motorTempC: 30, escTempC: 30,
    gpsLabel: "RTK Fixed", gpsOk: true, imuYawDeg: 0, imuOk: true,
    wheelLeftRpm: 0, wheelRightRpm: 0, bladeOn: false, rain: false, dockCharging: true,
};

describe("RobotAnatomy per robot profile", () => {
    it("stock robots keep the LiDAR part", () => {
        expect(anatomyParts("lidar")).toContain("lidar");
        render(<ThemeProvider><RobotAnatomy inputs={inputs}/></ThemeProvider>);
        expect(screen.getByText("LiDAR")).toBeInTheDocument();
        expect(screen.queryByTestId("anatomy-cameras")).not.toBeInTheDocument();
    });

    it("camera perception shows Cameras (3) and no LiDAR", () => {
        const tron = getRobotProfile(AIRSEEKERS_TRON_PROFILE_ID);
        render(
            <ThemeProvider>
                <RobotAnatomy inputs={{...inputs, cameras: {total: 3, live: 2, reporting: 3}}} profile={tron}/>
            </ThemeProvider>,
        );
        expect(screen.queryByText("LiDAR")).not.toBeInTheDocument();
        fireEvent.mouseEnter(screen.getByTestId("anatomy-cameras"));
        expect(screen.getAllByText("Cameras (3)").length).toBeGreaterThan(0);
        expect(screen.getByText("2/3 streaming")).toBeInTheDocument();
    });

    it("shows unknown when no camera reports freshness", () => {
        const tron = getRobotProfile(AIRSEEKERS_TRON_PROFILE_ID);
        render(<ThemeProvider><RobotAnatomy inputs={inputs} profile={tron}/></ThemeProvider>);
        fireEvent.mouseEnter(screen.getByTestId("anatomy-cameras"));
        expect(screen.getByText("Unknown")).toBeInTheDocument();
    });
});
