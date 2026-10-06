package api

// Settings -> ROS parameter bindings, per robot profile.
//
// Stock MowgliNext robots consume mowgli_robot.yaml directly: the ROS2 launch
// deep-merges it into every node's parameters, so a saved setting takes effect
// on the next container restart and nothing else is needed. A robot whose
// stack does NOT read that file (e.g. the Airseekers Tron port) declares a
// binding table instead: each GUI settings key is mapped to the ROS parameter
// that implements it. After a successful POST /settings/yaml the bound keys of
// the active profile are pushed to the running nodes through the foxglove
// `parameters` capability, and the save response reports a per-key status.
// The robot's own launch files read the same table's keys from the yaml at
// boot so the values survive a restart.
//
// A profile without a table (every stock profile) keeps today's behaviour
// exactly: nothing is pushed, nothing is flagged, the response is unchanged.
//
// The frontend mirrors the key lists in web/src/constants/paramBindings.ts so
// forms can flag "not used by this robot" without a round trip;
// TestParamBindingsFrontendParity keeps the two in sync.

import (
	"context"
	"fmt"
	"math"
	"net/http"
	"sort"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
)

// ROS parameter types a binding can produce. They must match the type the
// node declared: rclpy and rclcpp refuse a set that changes the type.
const (
	ParamTypeDouble  = "double"
	ParamTypeInteger = "integer"
	ParamTypeBool    = "bool"
	ParamTypeString  = "string"
)

// ParamBinding maps one GUI settings key to one ROS parameter.
type ParamBinding struct {
	// Key is the flat settings key (mowgli_robot.yaml / settings form).
	Key string `json:"key"`
	// Node is the fully-qualified ROS node name, e.g. "/coverage_server".
	Node string `json:"node"`
	// Param is the parameter name on that node; nested Nav2 plugin params use
	// dots ("FollowPath.desired_linear_vel").
	Param string `json:"param"`
	// Type is the ROS type the node declared (ParamType*).
	Type string `json:"type"`
	// Live is true when the node re-reads the parameter at runtime. A
	// non-live binding is not pushed (a node that read the value once at
	// startup would report the new value but keep using the old one); it
	// takes effect at the next restart through the robot's launch files.
	Live bool `json:"live"`
	// Offset is added to numeric values (e.g. cut width -> swath spacing).
	Offset float64 `json:"offset,omitempty"`
	// ValueMap translates GUI values (formatted with %v: "true", "2") to ROS
	// values when the two enums differ. A GUI value missing from the map is
	// rejected rather than guessed.
	ValueMap map[string]any `json:"value_map,omitempty"`
}

// FullName is the parameter name as the foxglove bridge reports it.
func (b ParamBinding) FullName() string {
	return b.Node + "." + b.Param
}

// ParamBindingTable describes how one robot profile consumes the settings.
type ParamBindingTable struct {
	Bindings []ParamBinding `json:"bindings"`
	// GuiKeys are settings that matter on this robot without a ROS parameter
	// binding: read by the GUI itself (map datum, model) or by a node that
	// reads mowgli_robot.yaml directly (dock pose via map_server). Every other
	// unbound key is "not used by this robot".
	GuiKeys []string `json:"gui_keys"`
}

// Bound reports whether key has a binding in this table.
func (t ParamBindingTable) Bound(key string) bool {
	for _, b := range t.Bindings {
		if b.Key == key {
			return true
		}
	}
	return false
}

// Used reports whether key has any effect on this robot.
func (t ParamBindingTable) Used(key string) bool {
	if t.Bound(key) {
		return true
	}
	for _, k := range t.GuiKeys {
		if k == key {
			return true
		}
	}
	return false
}

// tronSwathOverlapM is mower_bringup's swath_overlap_m launch default: the
// coverage bridge plans with operation_width = cut width - overlap, so the
// live binding of tool_width applies the same offset.
const tronSwathOverlapM = 0.02

// paramBindingTables is keyed by robot profile id (mower_model). Order within
// a table is the order parameters are pushed; correction_source comes after
// the NTRIP caster fields so a node that reconfigures on the source change
// already has the new caster.
var paramBindingTables = map[string]ParamBindingTable{
	"AirseekersTron": {
		Bindings: []ParamBinding{
			// Coverage planner and Nav2 controllers (src/mower_navigation/config/nav2_params.yaml).
			{Key: "tool_width", Node: "/coverage_server", Param: "operation_width", Type: ParamTypeDouble, Live: true, Offset: -tronSwathOverlapM},
			{Key: "mowing_speed", Node: "/controller_server", Param: "FollowCoveragePath.desired_linear_vel", Type: ParamTypeDouble, Live: true},
			{Key: "transit_speed", Node: "/controller_server", Param: "FollowPath.desired_linear_vel", Type: ParamTypeDouble, Live: true},
			// Mission layer (src/mower_mission/config/mission.yaml). The mission
			// node reads its parameters once at startup.
			{Key: "undock_distance", Node: "/behavior_tree_node", Param: "undock_distance_m", Type: ParamTypeDouble},
			{Key: "undock_speed", Node: "/behavior_tree_node", Param: "undock_speed_mps", Type: ParamTypeDouble},
			{Key: "battery_low_percent", Node: "/behavior_tree_node", Param: "battery_low_percent", Type: ParamTypeDouble},
			{Key: "battery_full_percent", Node: "/behavior_tree_node", Param: "battery_full_percent", Type: ParamTypeDouble},
			// GUI 0 ignore / 1 dock / 2 dock until dry / 3 pause; Tron 0 ignore / 1 dock and wait.
			{Key: "rain_mode", Node: "/behavior_tree_node", Param: "rain_mode", Type: ParamTypeInteger,
				ValueMap: map[string]any{"0": 0, "1": 1, "2": 1, "3": 1}},
			{Key: "rain_delay_minutes", Node: "/behavior_tree_node", Param: "rain_delay_minutes", Type: ParamTypeDouble},
			{Key: "rain_debounce_sec", Node: "/behavior_tree_node", Param: "rain_debounce_s", Type: ParamTypeDouble},
			{Key: "mow_angle_deg", Node: "/behavior_tree_node", Param: "mow_angle_deg", Type: ParamTypeDouble},
			// Vision docking (src/mower_docking/config/docking.yaml), read per goal.
			{Key: "dock_approach_distance", Node: "/mower_docking", Param: "approach_distance", Type: ParamTypeDouble, Live: true},
			{Key: "dock_max_retries", Node: "/mower_docking", Param: "max_retries", Type: ParamTypeInteger, Live: true},
			// UM960 corrections (src/um960_gps_driver): a change reconnects the client.
			{Key: "ntrip_host", Node: "/um960_gps_driver", Param: "ntrip_host", Type: ParamTypeString, Live: true},
			{Key: "ntrip_port", Node: "/um960_gps_driver", Param: "ntrip_port", Type: ParamTypeInteger, Live: true},
			{Key: "ntrip_mountpoint", Node: "/um960_gps_driver", Param: "ntrip_mountpoint", Type: ParamTypeString, Live: true},
			{Key: "ntrip_user", Node: "/um960_gps_driver", Param: "ntrip_user", Type: ParamTypeString, Live: true},
			{Key: "ntrip_password", Node: "/um960_gps_driver", Param: "ntrip_password", Type: ParamTypeString, Live: true},
			// NTRIP off means the vendor LoRa base on this robot.
			{Key: "ntrip_enabled", Node: "/um960_gps_driver", Param: "correction_source", Type: ParamTypeString, Live: true,
				ValueMap: map[string]any{"true": "ntrip", "false": "lora"}},
			// Straight-line driving (src/mower_control cmd_vel_slew, the last stage
			// before the MCU): trim / joystick deadband / IMU heading hold.
			{Key: "angular_trim_radps", Node: "/cmd_vel_slew", Param: "angular_trim_radps", Type: ParamTypeDouble, Live: true},
			{Key: "angular_deadband_radps", Node: "/cmd_vel_slew", Param: "angular_deadband_radps", Type: ParamTypeDouble, Live: true},
			{Key: "heading_hold", Node: "/cmd_vel_slew", Param: "heading_hold", Type: ParamTypeBool, Live: true},
			{Key: "heading_hold_kp", Node: "/cmd_vel_slew", Param: "heading_hold_kp", Type: ParamTypeDouble, Live: true},
			{Key: "heading_hold_kd", Node: "/cmd_vel_slew", Param: "heading_hold_kd", Type: ParamTypeDouble, Live: true},
			{Key: "heading_hold_max_radps", Node: "/cmd_vel_slew", Param: "heading_hold_max_radps", Type: ParamTypeDouble, Live: true},
		},
		GuiKeys: []string{
			"mower_model", "datum_lat", "datum_lon", "datum_alt",
			"dock_pose_x", "dock_pose_y", "dock_pose_yaw",
			"camera_stream_base_url",
		},
	},
}

// paramBindingTableFor returns the profile's table; ok is false for a profile
// that consumes mowgli_robot.yaml directly (every stock profile).
func paramBindingTableFor(profileID string) (ParamBindingTable, bool) {
	t, ok := paramBindingTables[profileID]
	return t, ok
}

// rosValueForBinding converts a settings value to the value the node expects.
func rosValueForBinding(b ParamBinding, value any) (any, error) {
	if b.ValueMap != nil {
		mapped, ok := b.ValueMap[fmt.Sprintf("%v", value)]
		if !ok {
			return nil, fmt.Errorf("value %v has no equivalent on this robot", value)
		}
		value = mapped
	}
	switch b.Type {
	case ParamTypeDouble:
		f, ok := asFloat64(value)
		if !ok || math.IsNaN(f) || math.IsInf(f, 0) {
			return nil, fmt.Errorf("value %v is not a number", value)
		}
		return f + b.Offset, nil
	case ParamTypeInteger:
		f, ok := asFloat64(value)
		if !ok || f != math.Trunc(f) {
			return nil, fmt.Errorf("value %v is not an integer", value)
		}
		return int64(f + b.Offset), nil
	case ParamTypeBool:
		switch v := value.(type) {
		case bool:
			return v, nil
		case string:
			parsed, err := strconv.ParseBool(strings.TrimSpace(v))
			if err != nil {
				return nil, fmt.Errorf("value %v is not a boolean", value)
			}
			return parsed, nil
		}
		return nil, fmt.Errorf("value %v is not a boolean", value)
	case ParamTypeString:
		if value == nil {
			return "", nil
		}
		return fmt.Sprintf("%v", value), nil
	}
	return nil, fmt.Errorf("unknown binding type %q", b.Type)
}

// foxgloveType is the foxglove `type` hint. The bridge infers integer for a
// JSON number without a fraction, which a node that declared a double
// rejects; "float64" forces a double.
func foxgloveType(b ParamBinding) string {
	if b.Type == ParamTypeDouble {
		return "float64"
	}
	return ""
}

// Per-key apply status returned by POST /settings/yaml.
const (
	// The node accepted the value and reports it back.
	ParamApplyApplied = "applied"
	// The node kept a different value (validation refused it).
	ParamApplyRejected = "rejected"
	// The node does not re-read this parameter: saved, effective after restart.
	ParamApplyRestart = "restart_required"
	// The key was removed from the yaml: the package default returns at restart.
	ParamApplyReset = "reset"
	// The value cannot be expressed on this robot (bad type, unmapped enum).
	ParamApplyInvalid = "invalid"
	// The bridge did not answer, or the node is not running.
	ParamApplyUnavailable = "unavailable"
)

// ParamApplyResult is the outcome for one bound key of a save.
type ParamApplyResult struct {
	Key     string `json:"key"`
	Param   string `json:"param"`
	Status  string `json:"status"`
	Value   any    `json:"value,omitempty"`
	Message string `json:"message,omitempty"`
}

// SettingsSaveResponse is the body of POST /settings/yaml. ParamBindings is
// only present for a profile with a binding table and a payload that touched
// one of its keys, so a stock robot gets the same `{}` as before.
type SettingsSaveResponse struct {
	OkResponse
	ParamBindings []ParamApplyResult `json:"param_bindings,omitempty"`
}

// applyParamBindings pushes the bound keys of payload to the running nodes.
// payload is the save request (only the keys the user changed; nil = reset).
func applyParamBindings(ctx context.Context, rosProvider types.IRosProvider, table ParamBindingTable, payload map[string]any) []ParamApplyResult {
	var results []ParamApplyResult
	var toSet []types.RosParameter
	pending := map[string]int{} // full param name -> index in results
	for _, b := range table.Bindings {
		value, touched := payload[b.Key]
		if !touched {
			continue
		}
		res := ParamApplyResult{Key: b.Key, Param: b.FullName()}
		if value == nil {
			res.Status = ParamApplyReset
			results = append(results, res)
			continue
		}
		rosValue, err := rosValueForBinding(b, value)
		if err != nil {
			res.Status = ParamApplyInvalid
			res.Message = err.Error()
			results = append(results, res)
			continue
		}
		res.Value = rosValue
		if !b.Live {
			res.Status = ParamApplyRestart
			results = append(results, res)
			continue
		}
		pending[res.Param] = len(results)
		results = append(results, res)
		toSet = append(toSet, types.RosParameter{Name: res.Param, Value: rosValue, Type: foxgloveType(b)})
	}
	if len(toSet) == 0 {
		return results
	}
	if rosProvider == nil {
		markUnavailable(results, pending, "no ROS connection")
		return results
	}
	echoed, err := rosProvider.SetParameters(ctx, toSet)
	if err != nil {
		markUnavailable(results, pending, err.Error())
		return results
	}
	reported := map[string]any{}
	for _, p := range echoed {
		reported[p.Name] = p.Value
	}
	for name, i := range pending {
		got, ok := reported[name]
		switch {
		case !ok:
			results[i].Status = ParamApplyUnavailable
			results[i].Message = "the node did not report the parameter (not running?)"
		case valuesEqual(got, results[i].Value):
			results[i].Status = ParamApplyApplied
		default:
			results[i].Status = ParamApplyRejected
			results[i].Message = fmt.Sprintf("the node kept %v", got)
		}
	}
	return results
}

func markUnavailable(results []ParamApplyResult, pending map[string]int, msg string) {
	for _, i := range pending {
		results[i].Status = ParamApplyUnavailable
		results[i].Message = msg
	}
}

// pinnedBoundValues returns the bound keys the payload sets explicitly. The
// save keeps them in the yaml even when they equal the schema default: the
// schema default describes a stock robot, while a robot with a binding table
// falls back to its own package default when the key is absent.
func pinnedBoundValues(table ParamBindingTable, payload map[string]any) map[string]any {
	pinned := map[string]any{}
	for _, b := range table.Bindings {
		if v, ok := payload[b.Key]; ok && v != nil {
			pinned[b.Key] = v
		}
	}
	return pinned
}

// paramBindingsState is filled by ParamBindingRoutes; PostSettingsYAML only
// applies bindings once a ROS provider has been registered.
var paramBindingsState struct {
	sync.RWMutex
	ros types.IRosProvider
}

func setParamBindingsRosProvider(ros types.IRosProvider) {
	paramBindingsState.Lock()
	paramBindingsState.ros = ros
	paramBindingsState.Unlock()
}

func paramBindingsRosProvider() types.IRosProvider {
	paramBindingsState.RLock()
	defer paramBindingsState.RUnlock()
	return paramBindingsState.ros
}

// saveProfileID is the profile a save applies to: a mower_model in the same
// payload wins over the one on disk.
func saveProfileID(dbProvider types.IDBProvider, payload map[string]any) string {
	if m, ok := payload["mower_model"].(string); ok && m != "" {
		return m
	}
	return resolveRobotProfile(dbProvider).ID
}

const paramBindingsApplyTimeout = 12 * time.Second

// applyParamBindingsAfterSave is called by PostSettingsYAML after the yaml was
// written. It returns nil for a stock profile.
func applyParamBindingsAfterSave(c *gin.Context, profileID string, payload map[string]any) []ParamApplyResult {
	table, ok := paramBindingTableFor(profileID)
	if !ok {
		return nil
	}
	ctx, cancel := context.WithTimeout(c.Request.Context(), paramBindingsApplyTimeout)
	defer cancel()
	return applyParamBindings(ctx, paramBindingsRosProvider(), table, payload)
}

// ParamBindingsResponse is the body of GET /settings/bindings.
type ParamBindingsResponse struct {
	Profile string `json:"profile"`
	// HasBindings is false for a profile that consumes mowgli_robot.yaml
	// directly; the key lists are then empty and nothing is "unused".
	HasBindings bool           `json:"has_bindings"`
	Bindings    []ParamBinding `json:"bindings"`
	GuiKeys     []string       `json:"gui_keys"`
	// BoundKeys / UnboundKeys partition the schema keys (sorted). UnboundKeys
	// excludes GuiKeys: it is exactly the "not used by this robot" set.
	BoundKeys   []string `json:"bound_keys"`
	UnboundKeys []string `json:"unbound_keys"`
}

// ParamBindingRoutes registers GET /settings/bindings and enables the
// post-save parameter push of POST /settings/yaml.
func ParamBindingRoutes(r *gin.RouterGroup, dbProvider types.IDBProvider, rosProvider types.IRosProvider) {
	setParamBindingsRosProvider(rosProvider)
	GetSettingsBindings(r, dbProvider)
}

// GetSettingsBindings lists the settings keys bound to ROS parameters
//
// @Summary lists the settings -> ROS parameter bindings of a robot profile
// @Description bound and unbound settings keys for the active profile (or ?profile=). A profile without a table consumes mowgli_robot.yaml directly and reports has_bindings=false.
// @Tags settings
// @Produce json
// @Param profile query string false "robot profile id (default: the active profile)"
// @Success 200 {object} ParamBindingsResponse
// @Router /settings/bindings [get]
func GetSettingsBindings(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.GET("/settings/bindings", func(c *gin.Context) {
		profileID := c.Query("profile")
		if profileID == "" {
			profileID = resolveRobotProfile(dbProvider).ID
		}
		resp := ParamBindingsResponse{
			Profile:     profileID,
			Bindings:    []ParamBinding{},
			GuiKeys:     []string{},
			BoundKeys:   []string{},
			UnboundKeys: []string{},
		}
		table, ok := paramBindingTableFor(profileID)
		if ok {
			resp.HasBindings = true
			resp.Bindings = table.Bindings
			resp.GuiKeys = table.GuiKeys
			keys := map[string]bool{}
			if schema, err := getSchema(dbProvider); err == nil {
				extractAllKeys(schema, keys)
			}
			for _, b := range table.Bindings {
				keys[b.Key] = true
			}
			for key := range keys {
				switch {
				case table.Bound(key):
					resp.BoundKeys = append(resp.BoundKeys, key)
				case !table.Used(key):
					resp.UnboundKeys = append(resp.UnboundKeys, key)
				}
			}
			sort.Strings(resp.BoundKeys)
			sort.Strings(resp.UnboundKeys)
		}
		c.JSON(http.StatusOK, resp)
	})
}
