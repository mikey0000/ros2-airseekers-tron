package api

import (
	"os"
	"strings"
)

// Container names of the robot stack, overridable per deployment so robots
// whose compose project names things differently (e.g. mower_humble) work.
const (
	defaultROSContainerName = "mowgli-ros2"
	defaultGPSContainerName = "mowgli-gps"
	defaultGUIContainerName = "mowgli-gui"
)

func envOr(key, def string) string {
	if v, ok := os.LookupEnv(key); ok {
		return strings.TrimSpace(v)
	}
	return def
}

// ROSContainerName is the ROS 2 container (env ROS_CONTAINER_NAME).
func ROSContainerName() string { return envOr("ROS_CONTAINER_NAME", defaultROSContainerName) }

// GPSContainerName is the GNSS sidecar (env GPS_CONTAINER_NAME); set the
// variable empty for robots without a GPS sidecar.
func GPSContainerName() string { return envOr("GPS_CONTAINER_NAME", defaultGPSContainerName) }

// GUIContainerName is this GUI's own container (env GUI_CONTAINER_NAME).
func GUIContainerName() string { return envOr("GUI_CONTAINER_NAME", defaultGUIContainerName) }

// ContainerNames is returned with the container list so the frontend never
// hard-codes names.
type ContainerNames struct {
	ROS string `json:"ros"`
	GPS string `json:"gps"`
	GUI string `json:"gui"`
}

func CurrentContainerNames() ContainerNames {
	return ContainerNames{ROS: ROSContainerName(), GPS: GPSContainerName(), GUI: GUIContainerName()}
}
