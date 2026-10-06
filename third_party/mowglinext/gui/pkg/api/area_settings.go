package api

import (
	"context"
	"encoding/json"
	"fmt"
	"math"
	"net/http"
	"strconv"
	"strings"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
)

// Per-area mowing settings ("area settings") are owned by the map server:
// it persists a defaults object plus optional per-area overrides and exposes
// them over two services. The GUI only validates and relays; it never caches.
const (
	areaSettingsSetService = "/map_server_node/set_area_settings"
	areaSettingsGetService = "/map_server_node/get_area_settings"
	areaSettingsSetType    = "mower_interfaces/srv/SetAreaSettings"
	areaSettingsGetType    = "mower_interfaces/srv/GetAreaSettings"
	// AreaSettingsDefaultsIndex addresses the robot-wide defaults.
	AreaSettingsDefaultsIndex = 255
)

type setAreaSettingsReq struct {
	AreaIndex    uint8  `json:"area_index"`
	SettingsJSON string `json:"settings_json"`
}

type setAreaSettingsRes struct {
	Success bool   `json:"success"`
	Message string `json:"message"`
}

type getAreaSettingsReq struct {
	AreaIndex uint8 `json:"area_index"`
}

type getAreaSettingsRes struct {
	Success      bool   `json:"success"`
	SettingsJSON string `json:"settings_json"`
}

// AreaSettingsResponse is the body of GET/PUT /mowglinext/areas/:index/settings.
// Settings is the fully merged (effective) settings for that index.
type AreaSettingsResponse struct {
	Supported bool           `json:"supported"`
	Index     int            `json:"index"`
	Settings  map[string]any `json:"settings"`
	Message   string         `json:"message,omitempty"`
}

// AreaSettingsPutRequest merges into the stored settings for one index (a key
// set to null resets it to the default). UseDefaults=true clears every
// per-area override (sent as "{}").
type AreaSettingsPutRequest struct {
	Settings    map[string]any `json:"settings"`
	UseDefaults bool           `json:"use_defaults"`
}

var areaPathModes = map[string]bool{
	"zigzag": true, "cross": true, "alternate": true, "spiral": true, "contour_only": true,
}

var areaRouteOrders = map[string]bool{"boustrophedon": true, "snake": true, "spiral": true, "racetrack": true}

var areaTurnTypes = map[string]bool{"auto": true, "loop": true, "reverse": true, "pivot": true}

var areaSlopeModes = map[string]bool{"off": true, "auto": true, "contour": true, "updown": true}

// areaDerivedKeys are read-only values the map server adds to a GET response
// (derived from terrain memory). They are dropped from a PUT so a client that
// echoes the effective settings back does not get rejected.
var areaDerivedKeys = []string{"slope_mow_angle_deg", "slope_angle_why"}

func stripDerivedAreaKeys(s map[string]any) {
	for _, k := range areaDerivedKeys {
		delete(s, k)
	}
}

var areaBladePolicies = map[string]bool{"continuous": true, "conservative": true}

var areaObstacleDetections = map[string]bool{"none": true, "standard": true, "sensitive": true}

type numRange struct {
	min, max float64
	integer  bool
}

var areaNumericKeys = map[string]numRange{
	"cutter_height_mm":           {30, 90, true},
	"perimeter_laps":             {0, 4, true},
	"mow_angle_deg":              {-1, 360, false},
	"cut_speed_mps":              {0.1, 0.5, false},
	"swath_overlap_m":            {0, 0.1, false},
	"swath_width_m":              {0.10, 0.40, false}, // planner path spacing (m)
	"edge_margin_m":              {0, 0.5, false},     // blade edge inside the boundary (m)
	"repeat":                     {1, 5, true},
	"alternate_angle_offset_deg": {0, 180, false},
	"route_spiral_size":          {2, 20, true},
	"min_turn_radius_m":          {0, 2, false},
	"slope_contour_above_deg":    {2, 30, false},
}

// validateAreaSettings checks every key against the contract. Unknown keys
// are rejected so a typo cannot silently persist.
func validateAreaSettings(s map[string]any) error {
	for k, v := range s {
		if v == nil {
			// null resets the key to its default (map server merge semantics).
			if _, known := areaNumericKeys[k]; known || k == "path_mode" || k == "edge_first" ||
				k == "route_order" || k == "turn_type" || k == "obstacle_detection" || k == "blade_policy" || k == "slope_mode" {
				continue
			}
			return fmt.Errorf("unknown setting %q", k)
		}
		if r, ok := areaNumericKeys[k]; ok {
			f, ok := v.(float64)
			if !ok || math.IsNaN(f) || math.IsInf(f, 0) {
				return fmt.Errorf("%s must be a number", k)
			}
			if r.integer && f != math.Trunc(f) {
				return fmt.Errorf("%s must be an integer", k)
			}
			if k == "mow_angle_deg" && f < 0 && f != -1 {
				return fmt.Errorf("mow_angle_deg must be -1 (auto) or between 0 and 360")
			}
			if f < r.min || f > r.max {
				return fmt.Errorf("%s must be between %g and %g", k, r.min, r.max)
			}
			continue
		}
		switch k {
		case "path_mode":
			str, ok := v.(string)
			if !ok || !areaPathModes[str] {
				return fmt.Errorf("path_mode must be one of zigzag, cross, alternate, spiral, contour_only")
			}
		case "route_order":
			str, ok := v.(string)
			if !ok || !areaRouteOrders[str] {
				return fmt.Errorf("route_order must be one of boustrophedon, snake, spiral, racetrack")
			}
		case "turn_type":
			str, ok := v.(string)
			if !ok || !areaTurnTypes[str] {
				return fmt.Errorf("turn_type must be one of auto, loop, reverse, pivot")
			}
		case "obstacle_detection":
			str, ok := v.(string)
			if !ok || !areaObstacleDetections[str] {
				return fmt.Errorf("obstacle_detection must be one of none, standard, sensitive")
			}
		case "blade_policy":
			str, ok := v.(string)
			if !ok || !areaBladePolicies[str] {
				return fmt.Errorf("blade_policy must be one of continuous, conservative")
			}
		case "slope_mode":
			str, ok := v.(string)
			if !ok || !areaSlopeModes[str] {
				return fmt.Errorf("slope_mode must be one of off, auto, contour, updown")
			}
		case "edge_first":
			if _, ok := v.(bool); !ok {
				return fmt.Errorf("edge_first must be a boolean")
			}
		default:
			return fmt.Errorf("unknown setting %q", k)
		}
	}
	return nil
}

func isServiceNotAdvertised(err error) bool {
	return err != nil && strings.Contains(err.Error(), "not advertised")
}

func parseAreaIndex(raw string) (int, error) {
	if raw == "defaults" {
		return AreaSettingsDefaultsIndex, nil
	}
	n, err := strconv.Atoi(raw)
	if err != nil || n < 0 || n >= AreaSettingsDefaultsIndex {
		return 0, fmt.Errorf("invalid area index %q", raw)
	}
	return n, nil
}

func notSupported(c *gin.Context, index int) {
	c.JSON(http.StatusNotImplemented, AreaSettingsResponse{
		Supported: false, Index: index, Settings: map[string]any{},
		Message: "area settings not supported by this robot",
	})
}

func getAreaSettings(ctx context.Context, provider types.IRosProvider, index int) (map[string]any, error) {
	var res getAreaSettingsRes
	err := provider.CallService(ctx, areaSettingsGetService,
		&getAreaSettingsReq{AreaIndex: uint8(index)}, &res, areaSettingsGetType)
	if err != nil {
		return nil, err
	}
	if !res.Success {
		return nil, fmt.Errorf("map server rejected get_area_settings for index %d", index)
	}
	settings := map[string]any{}
	if strings.TrimSpace(res.SettingsJSON) != "" {
		if err := json.Unmarshal([]byte(res.SettingsJSON), &settings); err != nil {
			return nil, fmt.Errorf("map server returned invalid settings JSON: %w", err)
		}
	}
	return settings, nil
}

// AreaSettingsRoutes registers GET/PUT /mowglinext/areas/:index/settings
// (":index" is a 0-based area index or "defaults" = 255).
//
// @Summary get or replace per-area mowing settings
// @Tags mowglinext
// @Produce json
// @Success 200 {object} AreaSettingsResponse
// @Failure 501 {object} AreaSettingsResponse
// @Router /mowglinext/areas/{index}/settings [get]
// @Router /mowglinext/areas/{index}/settings [put]
func AreaSettingsRoutes(group *gin.RouterGroup, provider types.IRosProvider) {
	group.GET("/areas/:index/settings", func(c *gin.Context) {
		index, err := parseAreaIndex(c.Param("index"))
		if err != nil {
			c.JSON(400, ErrorResponse{Error: err.Error()})
			return
		}
		ctx, cancel := context.WithTimeout(c.Request.Context(), 5*time.Second)
		defer cancel()
		settings, err := getAreaSettings(ctx, provider, index)
		if isServiceNotAdvertised(err) {
			notSupported(c, index)
			return
		}
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(200, AreaSettingsResponse{Supported: true, Index: index, Settings: settings})
	})

	group.PUT("/areas/:index/settings", func(c *gin.Context) {
		index, err := parseAreaIndex(c.Param("index"))
		if err != nil {
			c.JSON(400, ErrorResponse{Error: err.Error()})
			return
		}
		var body AreaSettingsPutRequest
		if err := json.NewDecoder(c.Request.Body).Decode(&body); err != nil {
			c.JSON(400, ErrorResponse{Error: "invalid JSON body: " + err.Error()})
			return
		}
		settings := body.Settings
		if body.UseDefaults {
			if index == AreaSettingsDefaultsIndex {
				c.JSON(400, ErrorResponse{Error: "use_defaults is not valid for the defaults index"})
				return
			}
			settings = map[string]any{}
		}
		if settings == nil {
			c.JSON(400, ErrorResponse{Error: "settings is required"})
			return
		}
		stripDerivedAreaKeys(settings)
		if err := validateAreaSettings(settings); err != nil {
			c.JSON(400, ErrorResponse{Error: err.Error()})
			return
		}
		encoded, _ := json.Marshal(settings)
		ctx, cancel := context.WithTimeout(c.Request.Context(), 5*time.Second)
		defer cancel()
		var res setAreaSettingsRes
		err = provider.CallService(ctx, areaSettingsSetService,
			&setAreaSettingsReq{AreaIndex: uint8(index), SettingsJSON: string(encoded)},
			&res, areaSettingsSetType)
		if isServiceNotAdvertised(err) {
			notSupported(c, index)
			return
		}
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		if !res.Success {
			c.JSON(400, ErrorResponse{Error: res.Message})
			return
		}
		// On success the map server answers with the effective settings JSON.
		effective := map[string]any{}
		if json.Unmarshal([]byte(res.Message), &effective) != nil {
			effective = settings
		}
		c.JSON(200, AreaSettingsResponse{Supported: true, Index: index, Settings: effective})
	})
}
