package api

import (
	"bytes"
	"encoding/json"
	"errors"
	"net/http"
	"net/http/httptest"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func doAreaSettings(t *testing.T, mock *types.MockRosProvider, method, path string, body any) (*httptest.ResponseRecorder, map[string]any) {
	t.Helper()
	router := setupMowgliNextRouter(mock)
	var rd *bytes.Reader
	if body != nil {
		b, _ := json.Marshal(body)
		rd = bytes.NewReader(b)
	} else {
		rd = bytes.NewReader(nil)
	}
	req, _ := http.NewRequest(method, path, rd)
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	router.ServeHTTP(w, req)
	out := map[string]any{}
	_ = json.Unmarshal(w.Body.Bytes(), &out)
	return w, out
}

func TestAreaSettings_GetArea(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceResponder = func(service string, req any, res any) {
		r := res.(*getAreaSettingsRes)
		r.Success = true
		r.SettingsJSON = `{"cutter_height_mm":60,"path_mode":"spiral"}`
	}
	w, out := doAreaSettings(t, mock, "GET", "/api/mowglinext/areas/2/settings", nil)
	require.Equal(t, 200, w.Code)
	assert.Equal(t, true, out["supported"])
	assert.Equal(t, float64(2), out["index"])
	assert.Equal(t, float64(60), out["settings"].(map[string]any)["cutter_height_mm"])
	require.Len(t, mock.ServiceCalls, 1)
	assert.Equal(t, areaSettingsGetService, mock.ServiceCalls[0].Service)
	assert.Equal(t, uint8(2), mock.ServiceCalls[0].Req.(*getAreaSettingsReq).AreaIndex)
}

func TestAreaSettings_GetDefaultsUses255(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceResponder = func(_ string, _ any, res any) {
		r := res.(*getAreaSettingsRes)
		r.Success = true
		r.SettingsJSON = `{"cutter_height_mm":50}`
	}
	w, out := doAreaSettings(t, mock, "GET", "/api/mowglinext/areas/defaults/settings", nil)
	require.Equal(t, 200, w.Code)
	assert.Equal(t, float64(255), out["index"])
	assert.Equal(t, uint8(255), mock.ServiceCalls[0].Req.(*getAreaSettingsReq).AreaIndex)
}

func TestAreaSettings_NotAdvertisedIs501(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceErr = errors.New("foxglove: CallService /map_server_node/get_area_settings: service not advertised")
	w, out := doAreaSettings(t, mock, "GET", "/api/mowglinext/areas/0/settings", nil)
	assert.Equal(t, http.StatusNotImplemented, w.Code)
	assert.Equal(t, false, out["supported"])

	mock.ServiceErr = errors.New("foxglove: CallService /map_server_node/set_area_settings: service not advertised")
	w, out = doAreaSettings(t, mock, "PUT", "/api/mowglinext/areas/0/settings", map[string]any{"settings": map[string]any{"repeat": 2}})
	assert.Equal(t, http.StatusNotImplemented, w.Code)
	assert.Equal(t, false, out["supported"])
}

func TestAreaSettings_OtherErrorIs500(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceErr = errors.New("foxglove: CallService x: not connected")
	w, _ := doAreaSettings(t, mock, "GET", "/api/mowglinext/areas/0/settings", nil)
	assert.Equal(t, 500, w.Code)
}

func TestAreaSettings_PutValid(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceResponder = func(_ string, _ any, res any) {
		res.(*setAreaSettingsRes).Success = true
	}
	settings := map[string]any{
		"cutter_height_mm": 45, "perimeter_laps": 3, "path_mode": "alternate",
		"mow_angle_deg": -1, "cut_speed_mps": 0.25, "swath_overlap_m": 0.05, "swath_width_m": 0.18,
		"edge_first": false, "repeat": 2, "alternate_angle_offset_deg": 90,
		"obstacle_detection": "sensitive",
	}
	w, out := doAreaSettings(t, mock, "PUT", "/api/mowglinext/areas/1/settings", map[string]any{"settings": settings})
	require.Equal(t, 200, w.Code, w.Body.String())
	assert.Equal(t, true, out["supported"])
	require.Len(t, mock.ServiceCalls, 1)
	call := mock.ServiceCalls[0]
	assert.Equal(t, areaSettingsSetService, call.Service)
	req := call.Req.(*setAreaSettingsReq)
	assert.Equal(t, uint8(1), req.AreaIndex)
	var sent map[string]any
	require.NoError(t, json.Unmarshal([]byte(req.SettingsJSON), &sent))
	assert.Equal(t, "alternate", sent["path_mode"])
	assert.Equal(t, float64(45), sent["cutter_height_mm"])
	assert.Equal(t, "sensitive", sent["obstacle_detection"])
}

func TestAreaSettings_PutUseDefaultsSendsEmpty(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceResponder = func(_ string, _ any, res any) { res.(*setAreaSettingsRes).Success = true }
	w, _ := doAreaSettings(t, mock, "PUT", "/api/mowglinext/areas/3/settings", map[string]any{"use_defaults": true, "settings": map[string]any{"repeat": 9}})
	require.Equal(t, 200, w.Code)
	assert.Equal(t, "{}", mock.ServiceCalls[0].Req.(*setAreaSettingsReq).SettingsJSON)

	w, _ = doAreaSettings(t, mock, "PUT", "/api/mowglinext/areas/defaults/settings", map[string]any{"use_defaults": true})
	assert.Equal(t, 400, w.Code)
}

func TestAreaSettings_PutServiceFailureIs400(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceResponder = func(_ string, _ any, res any) {
		r := res.(*setAreaSettingsRes)
		r.Success = false
		r.Message = "unknown area"
	}
	w, out := doAreaSettings(t, mock, "PUT", "/api/mowglinext/areas/9/settings", map[string]any{"settings": map[string]any{"repeat": 1}})
	assert.Equal(t, 400, w.Code)
	assert.Equal(t, "unknown area", out["error"])
}

func TestAreaSettings_PutRejectsOutOfRange(t *testing.T) {
	cases := []map[string]any{
		{"cutter_height_mm": 20},
		{"cutter_height_mm": 95},
		{"cutter_height_mm": 50.5},
		{"perimeter_laps": 5},
		{"path_mode": "random"},
		{"mow_angle_deg": -5},
		{"mow_angle_deg": -0.5},
		{"cut_speed_mps": 0.6},
		{"swath_overlap_m": 0.2},
		{"swath_width_m": 0.05},
		{"swath_width_m": 0.41},
		{"swath_width_m": 0},
		{"edge_margin_m": 0.6},
		{"edge_margin_m": -0.1},
		{"edge_first": "yes"},
		{"repeat": 0},
		{"repeat": 6},
		{"obstacle_detection": "high"},
		{"obstacle_detection": true},
		{"bogus": 1},
	}
	for _, s := range cases {
		mock := types.NewMockRosProvider()
		w, _ := doAreaSettings(t, mock, "PUT", "/api/mowglinext/areas/0/settings", map[string]any{"settings": s})
		assert.Equalf(t, 400, w.Code, "%v should be rejected", s)
		assert.Emptyf(t, mock.ServiceCalls, "%v must not reach ROS", s)
	}
}

func TestAreaSettings_BadIndex(t *testing.T) {
	for _, idx := range []string{"-1", "255", "abc"} {
		w, _ := doAreaSettings(t, types.NewMockRosProvider(), "GET", "/api/mowglinext/areas/"+idx+"/settings", nil)
		assert.Equalf(t, 400, w.Code, idx)
	}
}

func TestAreaSettings_PutNullResetsAndReturnsEffective(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceResponder = func(_ string, _ any, res any) {
		r := res.(*setAreaSettingsRes)
		r.Success = true
		r.Message = `{"cutter_height_mm":50,"repeat":1}`
	}
	w, out := doAreaSettings(t, mock, "PUT", "/api/mowglinext/areas/0/settings", map[string]any{"settings": map[string]any{"cutter_height_mm": nil}})
	require.Equal(t, 200, w.Code, w.Body.String())
	assert.Equal(t, `{"cutter_height_mm":null}`, mock.ServiceCalls[0].Req.(*setAreaSettingsReq).SettingsJSON)
	assert.Equal(t, float64(50), out["settings"].(map[string]any)["cutter_height_mm"])
	w, _ = doAreaSettings(t, types.NewMockRosProvider(), "PUT", "/api/mowglinext/areas/0/settings", map[string]any{"settings": map[string]any{"bogus": nil}})
	assert.Equal(t, 400, w.Code)
}
