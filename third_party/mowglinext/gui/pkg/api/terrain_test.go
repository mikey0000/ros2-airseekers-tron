package api

import (
	"encoding/json"
	"errors"
	"net/http"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

const terrainPath = "/api/mowglinext/terrain/action"

func TestTerrainAction_Keepout(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceResponder = func(_ string, _ any, res any) {
		r := res.(*setAreaSettingsRes)
		r.Success = true
		r.Message = "ok"
	}
	w, out := doAreaSettings(t, mock, "POST", terrainPath, map[string]any{"area_index": 2, "action": "keepout", "cluster_id": 7})
	require.Equal(t, 200, w.Code)
	assert.Equal(t, true, out["success"])
	require.Len(t, mock.ServiceCalls, 1)
	call := mock.ServiceCalls[0]
	assert.Equal(t, terrainActionService, call.Service)
	req := call.Req.(*setAreaSettingsReq)
	assert.Equal(t, uint8(2), req.AreaIndex)
	sent := map[string]any{}
	require.NoError(t, json.Unmarshal([]byte(req.SettingsJSON), &sent))
	assert.Equal(t, map[string]any{"action": "keepout", "cluster_id": float64(7)}, sent)
}

func TestTerrainAction_ClearDefaultsTo255(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceResponder = func(_ string, _ any, res any) { res.(*setAreaSettingsRes).Success = true }
	w, _ := doAreaSettings(t, mock, "POST", terrainPath, map[string]any{"action": "clear"})
	require.Equal(t, 200, w.Code)
	req := mock.ServiceCalls[0].Req.(*setAreaSettingsReq)
	assert.Equal(t, uint8(255), req.AreaIndex)
	assert.Equal(t, `{"action":"clear"}`, req.SettingsJSON)
}

func TestTerrainAction_Validation(t *testing.T) {
	for _, b := range []map[string]any{
		{"action": "bogus", "cluster_id": 1},
		{"action": "confirm"},
		{"action": "dismiss", "area_index": 300, "cluster_id": 1},
	} {
		mock := types.NewMockRosProvider()
		w, _ := doAreaSettings(t, mock, "POST", terrainPath, b)
		assert.Equal(t, 400, w.Code, "%v", b)
		assert.Len(t, mock.ServiceCalls, 0)
	}
}

func TestTerrainAction_NotAdvertisedIs501(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceErr = errors.New("foxglove: CallService /map_server_node/terrain_action: service not advertised")
	w, out := doAreaSettings(t, mock, "POST", terrainPath, map[string]any{"action": "clear"})
	assert.Equal(t, http.StatusNotImplemented, w.Code)
	assert.Equal(t, false, out["supported"])
}

func TestTerrainAction_FailureIs400(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceResponder = func(_ string, _ any, res any) {
		r := res.(*setAreaSettingsRes)
		r.Success = false
		r.Message = "unknown cluster 9"
	}
	w, out := doAreaSettings(t, mock, "POST", terrainPath, map[string]any{"action": "confirm", "cluster_id": 9})
	assert.Equal(t, 400, w.Code)
	assert.Equal(t, "unknown cluster 9", out["error"])
}
