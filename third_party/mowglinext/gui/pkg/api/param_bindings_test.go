package api

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"net/http"
	"net/http/httptest"
	"os"
	"regexp"
	"sort"
	"strings"
	"testing"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"gopkg.in/yaml.v3"
)

func tronTable(t *testing.T) ParamBindingTable {
	t.Helper()
	table, ok := paramBindingTableFor("AirseekersTron")
	require.True(t, ok)
	return table
}

func bindingFor(t *testing.T, table ParamBindingTable, key string) ParamBinding {
	t.Helper()
	for _, b := range table.Bindings {
		if b.Key == key {
			return b
		}
	}
	t.Fatalf("no binding for %s", key)
	return ParamBinding{}
}

func TestParamBindings_StockProfilesHaveNoTable(t *testing.T) {
	for _, id := range []string{"YardForce500", "Sabo", "", "unknown"} {
		_, ok := paramBindingTableFor(id)
		assert.False(t, ok, id)
	}
}

func TestParamBindings_TableIsWellFormed(t *testing.T) {
	for id, table := range paramBindingTables {
		seenKey := map[string]bool{}
		seenParam := map[string]bool{}
		for _, b := range table.Bindings {
			assert.False(t, seenKey[b.Key], "%s: %s bound twice", id, b.Key)
			assert.False(t, seenParam[b.FullName()], "%s: %s targeted twice", id, b.FullName())
			seenKey[b.Key], seenParam[b.FullName()] = true, true
			assert.True(t, strings.HasPrefix(b.Node, "/"), "%s: node %q must be fully qualified", id, b.Node)
			assert.Contains(t, []string{ParamTypeDouble, ParamTypeInteger, ParamTypeBool, ParamTypeString}, b.Type)
		}
		for _, k := range table.GuiKeys {
			assert.False(t, seenKey[k], "%s: %s is both bound and a GUI key", id, k)
		}
	}
}

func TestParamBindings_BoundKeysExistInSchema(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)
	schema, err := getSchema(types.NewMockDBProvider())
	require.NoError(t, err)
	keys := map[string]bool{}
	extractAllKeys(schema, keys)
	// The NTRIP caster fields are GUI-only keys (NtripSection), not schema properties.
	guiOnly := map[string]bool{"ntrip_host": true, "ntrip_port": true, "ntrip_mountpoint": true,
		"ntrip_user": true, "ntrip_password": true}
	for _, b := range tronTable(t).Bindings {
		if !guiOnly[b.Key] {
			assert.True(t, keys[b.Key], "bound key %s is not in the settings schema", b.Key)
		}
	}
}

func TestRosValueForBinding(t *testing.T) {
	table := tronTable(t)
	cases := []struct {
		key   string
		in    any
		want  any
		isErr bool
	}{
		{"mowing_speed", 0.3, 0.3, false},
		{"mowing_speed", float64(1), float64(1), false}, // stays a double
		{"mowing_speed", "0.25", 0.25, false},
		{"mowing_speed", "fast", nil, true},
		{"tool_width", 0.22, 0.22 - tronSwathOverlapM, false},
		{"rain_mode", float64(0), int64(0), false},
		{"rain_mode", float64(2), int64(1), false},
		{"rain_mode", float64(7), nil, true},
		{"dock_max_retries", float64(4), int64(4), false},
		{"dock_max_retries", 4.5, nil, true},
		{"ntrip_port", float64(2101), int64(2101), false},
		{"ntrip_host", "caster.example", "caster.example", false},
		{"ntrip_enabled", true, "ntrip", false},
		{"ntrip_enabled", false, "lora", false},
	}
	for _, tc := range cases {
		got, err := rosValueForBinding(bindingFor(t, table, tc.key), tc.in)
		if tc.isErr {
			assert.Error(t, err, "%s=%v", tc.key, tc.in)
			continue
		}
		require.NoError(t, err, "%s=%v", tc.key, tc.in)
		if f, ok := tc.want.(float64); ok {
			assert.InDelta(t, f, got, 1e-12, tc.key)
			assert.IsType(t, float64(0), got, tc.key)
		} else {
			assert.Equal(t, tc.want, got, "%s=%v", tc.key, tc.in)
		}
	}
}

func TestApplyParamBindings_PushesLiveKeysOnly(t *testing.T) {
	ros := types.NewMockRosProvider()
	results := applyParamBindings(t.Context(), ros, tronTable(t), map[string]any{
		"mowing_speed":        0.3,
		"battery_low_percent": 25.0, // mission node: not live
		"ntrip_enabled":       true, // um960: live
		"rain_mode":           9.0,  // invalid enum
		"undock_distance":     nil,  // reset
		"wheel_track":         0.4,  // unbound: ignored
		"ntrip_host":          "h",  // live
	})

	byKey := map[string]ParamApplyResult{}
	for _, r := range results {
		byKey[r.Key] = r
	}
	require.Len(t, byKey, 6)
	assert.Equal(t, ParamApplyApplied, byKey["mowing_speed"].Status)
	assert.Equal(t, "/controller_server.FollowCoveragePath.desired_linear_vel", byKey["mowing_speed"].Param)
	assert.Equal(t, ParamApplyRestart, byKey["battery_low_percent"].Status)
	assert.Equal(t, 25.0, byKey["battery_low_percent"].Value)
	assert.Equal(t, ParamApplyApplied, byKey["ntrip_enabled"].Status)
	assert.Equal(t, ParamApplyApplied, byKey["ntrip_host"].Status)
	assert.Equal(t, ParamApplyInvalid, byKey["rain_mode"].Status)
	assert.Equal(t, ParamApplyReset, byKey["undock_distance"].Status)

	require.Len(t, ros.SetParams, 1, "one batched set")
	sent := ros.SetParams[0]
	require.Len(t, sent, 3)
	names := []string{}
	for _, p := range sent {
		names = append(names, p.Name)
	}
	// Table order: caster fields before the source switch.
	assert.Equal(t, []string{
		"/controller_server.FollowCoveragePath.desired_linear_vel",
		"/um960_gps_driver.ntrip_host",
		"/um960_gps_driver.correction_source",
	}, names)
	assert.Equal(t, "float64", sent[0].Type, "doubles carry the foxglove float64 hint")
	assert.Equal(t, "ntrip", sent[2].Value)
}

// rosWithEcho answers every set with a fixed echo (what the nodes kept).
type rosWithEcho struct {
	*types.MockRosProvider
	echo []types.RosParameter
	err  error
}

func (r *rosWithEcho) SetParameters(_ context.Context, _ []types.RosParameter) ([]types.RosParameter, error) {
	return r.echo, r.err
}

func TestApplyParamBindings_RejectedAndMissingAndOffline(t *testing.T) {
	table := tronTable(t)
	payload := map[string]any{"mowing_speed": 0.3, "transit_speed": 0.4}

	ros := &rosWithEcho{MockRosProvider: types.NewMockRosProvider(), echo: []types.RosParameter{
		{Name: "/controller_server.FollowCoveragePath.desired_linear_vel", Value: 0.25},
	}}
	results := applyParamBindings(t.Context(), ros, table, payload)
	require.Len(t, results, 2)
	assert.Equal(t, ParamApplyRejected, results[0].Status)
	assert.Contains(t, results[0].Message, "0.25")
	assert.Equal(t, ParamApplyUnavailable, results[1].Status)

	offline := &rosWithEcho{MockRosProvider: types.NewMockRosProvider(), err: errors.New("foxglove: SetParameters: not connected")}
	results = applyParamBindings(t.Context(), offline, table, payload)
	for _, r := range results {
		assert.Equal(t, ParamApplyUnavailable, r.Status)
		assert.Contains(t, r.Message, "not connected")
	}

	results = applyParamBindings(t.Context(), nil, table, payload)
	assert.Equal(t, ParamApplyUnavailable, results[0].Status)
}

func postSettingsYAML(t *testing.T, db types.IDBProvider, ros types.IRosProvider, payload map[string]any) (int, SettingsSaveResponse, []byte) {
	t.Helper()
	gin.SetMode(gin.TestMode)
	r := gin.New()
	g := r.Group("/api")
	SettingsRoutes(g, db)
	ParamBindingRoutes(g, db, ros)
	t.Cleanup(func() { setParamBindingsRosProvider(nil) })
	body, _ := json.Marshal(payload)
	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/yaml", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	r.ServeHTTP(w, req)
	var resp SettingsSaveResponse
	_ = json.Unmarshal(w.Body.Bytes(), &resp)
	return w.Code, resp, w.Body.Bytes()
}

func readFlatYAML(t *testing.T, path string) map[string]any {
	t.Helper()
	raw, err := os.ReadFile(path)
	require.NoError(t, err)
	nested := map[string]any{}
	require.NoError(t, yaml.Unmarshal(raw, &nested))
	return flattenROS2YAML(nested)
}

func TestPostSettingsYAML_StockProfileResponseUnchanged(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)
	yamlFile := createTempYAMLFile(t, "mowgli:\n  ros__parameters:\n    mower_model: YardForce500\n")
	db := types.NewMockDBProvider()
	db.Set("system.mower.yamlConfigFile", []byte(yamlFile))
	ros := types.NewMockRosProvider()

	code, _, body := postSettingsYAML(t, db, ros, map[string]any{"mowing_speed": 0.3})
	require.Equal(t, http.StatusOK, code)
	assert.JSONEq(t, `{}`, string(body))
	assert.Empty(t, ros.SetParams, "a stock robot reads the yaml itself")
	// Stock sparsity is untouched: a value at the schema default is pruned.
	code, _, _ = postSettingsYAML(t, db, ros, map[string]any{"mowing_speed": 0.2})
	require.Equal(t, http.StatusOK, code)
	_, present := readFlatYAML(t, yamlFile)["mowing_speed"]
	assert.False(t, present)
}

func TestPostSettingsYAML_TronAppliesBindingsAndPinsBoundKeys(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)
	yamlFile := createTempYAMLFile(t, "mowgli:\n  ros__parameters:\n    mower_model: AirseekersTron\n")
	db := types.NewMockDBProvider()
	db.Set("system.mower.yamlConfigFile", []byte(yamlFile))
	ros := types.NewMockRosProvider()

	// 0.2 is the schema default of mowing_speed; Tron's package default is
	// 0.25, so the explicit value must survive in the yaml.
	code, resp, _ := postSettingsYAML(t, db, ros, map[string]any{
		"mowing_speed":        0.2,
		"battery_low_percent": 30.0,
		"wheel_track":         0.5,
	})
	require.Equal(t, http.StatusOK, code)
	require.Len(t, resp.ParamBindings, 2)
	statuses := map[string]string{}
	for _, r := range resp.ParamBindings {
		statuses[r.Key] = r.Status
	}
	assert.Equal(t, map[string]string{"mowing_speed": ParamApplyApplied, "battery_low_percent": ParamApplyRestart}, statuses)
	require.Len(t, ros.SetParams, 1)
	assert.Equal(t, "/controller_server.FollowCoveragePath.desired_linear_vel", ros.SetParams[0][0].Name)

	flat := readFlatYAML(t, yamlFile)
	assert.True(t, valuesEqual(0.2, flat["mowing_speed"]), "bound key pinned at the schema default")
	assert.True(t, valuesEqual(30.0, flat["battery_low_percent"]))
	assert.True(t, valuesEqual(0.5, flat["wheel_track"]))

	// null = reset: the key leaves the yaml, the package default returns at restart.
	code, resp, _ = postSettingsYAML(t, db, ros, map[string]any{"mowing_speed": nil})
	require.Equal(t, http.StatusOK, code)
	require.Len(t, resp.ParamBindings, 1)
	assert.Equal(t, ParamApplyReset, resp.ParamBindings[0].Status)
	_, present := readFlatYAML(t, yamlFile)["mowing_speed"]
	assert.False(t, present)
}

func TestPostSettingsYAML_ModelInPayloadSelectsTheTable(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)
	yamlFile := createTempYAMLFile(t, "")
	db := types.NewMockDBProvider()
	db.Set("system.mower.yamlConfigFile", []byte(yamlFile))
	ros := types.NewMockRosProvider()

	code, resp, _ := postSettingsYAML(t, db, ros, map[string]any{
		"mower_model": "AirseekersTron", "transit_speed": 0.35,
	})
	require.Equal(t, http.StatusOK, code)
	require.Len(t, resp.ParamBindings, 1)
	assert.Equal(t, "/controller_server.FollowPath.desired_linear_vel", resp.ParamBindings[0].Param)
}

func getBindings(t *testing.T, db types.IDBProvider, query string) ParamBindingsResponse {
	t.Helper()
	gin.SetMode(gin.TestMode)
	r := gin.New()
	ParamBindingRoutes(r.Group("/api"), db, nil)
	w := httptest.NewRecorder()
	req, _ := http.NewRequest("GET", "/api/settings/bindings"+query, nil)
	r.ServeHTTP(w, req)
	require.Equal(t, http.StatusOK, w.Code)
	var resp ParamBindingsResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &resp))
	return resp
}

func TestGetSettingsBindings(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)
	yamlFile := createTempYAMLFile(t, "mowgli:\n  ros__parameters:\n    mower_model: AirseekersTron\n")
	db := types.NewMockDBProvider()
	db.Set("system.mower.yamlConfigFile", []byte(yamlFile))

	tron := getBindings(t, db, "")
	assert.Equal(t, "AirseekersTron", tron.Profile)
	assert.True(t, tron.HasBindings)
	assert.Contains(t, tron.BoundKeys, "mowing_speed")
	assert.Contains(t, tron.BoundKeys, "ntrip_host")
	assert.Contains(t, tron.UnboundKeys, "wheel_track")
	assert.Contains(t, tron.UnboundKeys, "dock_charging_threshold")
	assert.NotContains(t, tron.UnboundKeys, "datum_lat", "GUI keys are used")
	assert.NotContains(t, tron.UnboundKeys, "mowing_speed")
	assert.True(t, sort.StringsAreSorted(tron.UnboundKeys))

	stock := getBindings(t, db, "?profile=YardForce500")
	assert.False(t, stock.HasBindings)
	assert.Empty(t, stock.BoundKeys)
	assert.Empty(t, stock.UnboundKeys)
}

// The frontend mirror (web/src/constants/paramBindings.ts) must list the
// same bound and GUI keys per profile as the backend table.
func TestParamBindingsFrontendParity(t *testing.T) {
	chdirToGuiRoot(t)
	src, err := os.ReadFile("web/src/constants/paramBindings.ts")
	require.NoError(t, err)
	text := string(src)

	start := strings.Index(text, "PARAM_BINDING_PROFILES")
	require.GreaterOrEqual(t, start, 0)
	profileRe := regexp.MustCompile(`(?m)^    ([A-Za-z0-9_]+): \{`)
	listRe := regexp.MustCompile(`(?s)(boundKeys|guiKeys): \[(.*?)\]`)
	quoted := regexp.MustCompile(`"([^"]+)"`)

	body := text[start:]
	locs := profileRe.FindAllStringSubmatchIndex(body, -1)
	frontend := map[string]map[string][]string{}
	for i, loc := range locs {
		end := len(body)
		if i+1 < len(locs) {
			end = locs[i+1][0]
		}
		id := body[loc[2]:loc[3]]
		frontend[id] = map[string][]string{}
		for _, m := range listRe.FindAllStringSubmatch(body[loc[1]:end], -1) {
			var keys []string
			for _, q := range quoted.FindAllStringSubmatch(m[2], -1) {
				keys = append(keys, q[1])
			}
			frontend[id][m[1]] = keys
		}
	}

	require.Len(t, frontend, len(paramBindingTables), "profiles: %v", frontend)
	for id, table := range paramBindingTables {
		fe, ok := frontend[id]
		require.True(t, ok, "profile %s missing from paramBindings.ts", id)
		var bound []string
		for _, b := range table.Bindings {
			bound = append(bound, b.Key)
		}
		assert.Equal(t, bound, fe["boundKeys"], "%s boundKeys", id)
		assert.Equal(t, table.GuiKeys, fe["guiKeys"], "%s guiKeys", id)
	}
}
