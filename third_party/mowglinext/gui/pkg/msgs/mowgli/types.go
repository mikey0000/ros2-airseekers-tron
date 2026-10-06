package mowgli

// Types in this file are GUI-internal (not generated from ROS2 .msg files).
// For ROS2 message types, see types_generated.go.

// Map is the internal map structure sent to the frontend via the virtual "map" topic.
// It is assembled by pollMap() from get_mowing_area service calls.
type Map struct {
	WorkingAreaIndices []uint32  `json:"working_area_indices"`
	MapWidth           float64   `json:"map_width"`
	MapHeight          float64   `json:"map_height"`
	MapCenterX         float64   `json:"map_center_x"`
	MapCenterY         float64   `json:"map_center_y"`
	NavigationAreas    []MapArea `json:"navigation_areas"`
	WorkingArea        []MapArea `json:"working_area"`
	// Nil means no pose received; an explicitly received zero is a valid dock.
	DockX       *float64 `json:"dock_x,omitempty"`
	DockY       *float64 `json:"dock_y,omitempty"`
	DockHeading *float64 `json:"dock_heading,omitempty"`
	// Path (channel) metadata, parallel to NavigationAreas: nil entries are
	// plain polygon navigation areas. Omitted when the map server does not
	// serve get_area_channel.
	NavigationChannels []*AreaChannel `json:"navigation_channels,omitempty"`
}

// AreaChannel is the centreline (map metres) + band width of a navigation
// area drawn with the path tool. The area polygon stays the authoritative
// shape; this only lets the editor re-open the path as a line.
type AreaChannel struct {
	Points [][2]float64 `json:"points"`
	WidthM float64      `json:"width_m"`
}

// Map server path-metadata services (mower_map; they reuse the
// mower_interfaces area-settings service types, payload = AreaChannel JSON).
const (
	SetAreaChannelService = "/map_server_node/set_area_channel"
	GetAreaChannelService = "/map_server_node/get_area_channel"
	SetAreaChannelType    = "mower_interfaces/srv/SetAreaSettings"
	GetAreaChannelType    = "mower_interfaces/srv/GetAreaSettings"
)

type SetAreaChannelReq struct {
	AreaIndex    uint8  `json:"area_index"`
	SettingsJSON string `json:"settings_json"`
}

type SetAreaChannelRes struct {
	Success bool   `json:"success"`
	Message string `json:"message"`
}

type GetAreaChannelReq struct {
	AreaIndex uint8 `json:"area_index"`
}

type GetAreaChannelRes struct {
	Success      bool   `json:"success"`
	SettingsJSON string `json:"settings_json"`
}

// DockingSensor - placeholder, may not exist in ROS2 mowgli
type DockingSensor struct {
	DockPresent  bool    `json:"dock_present"`
	DockDistance float32 `json:"dock_distance"`
}
