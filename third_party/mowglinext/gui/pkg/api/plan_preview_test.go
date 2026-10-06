package api

import (
	"errors"
	"net/http"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestPlanPreview_AreaIndex(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceResponder = func(_ string, _ any, res any) {
		r := res.(*setAreaSettingsRes)
		r.Success = true
		r.Message = "planning preview of area 2"
	}
	w, out := doAreaSettings(t, mock, "POST", "/api/mowglinext/plan/preview?area=2", nil)
	require.Equal(t, 200, w.Code)
	assert.Equal(t, true, out["accepted"])
	assert.Equal(t, float64(2), out["area"])
	require.Len(t, mock.ServiceCalls, 1)
	assert.Equal(t, planPreviewService, mock.ServiceCalls[0].Service)
	assert.Equal(t, uint8(2), mock.ServiceCalls[0].Req.(*setAreaSettingsReq).AreaIndex)
}

func TestPlanPreview_AllAreasAndClear(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceResponder = func(_ string, _ any, res any) {
		res.(*setAreaSettingsRes).Success = true
	}
	w, out := doAreaSettings(t, mock, "POST", "/api/mowglinext/plan/preview", nil)
	require.Equal(t, 200, w.Code)
	assert.Equal(t, float64(-1), out["area"])
	w, _ = doAreaSettings(t, mock, "POST", "/api/mowglinext/plan/preview?area=-1", nil)
	require.Equal(t, 200, w.Code)
	w, _ = doAreaSettings(t, mock, "DELETE", "/api/mowglinext/plan/preview", nil)
	require.Equal(t, 200, w.Code)
	require.Len(t, mock.ServiceCalls, 3)
	assert.Equal(t, uint8(255), mock.ServiceCalls[0].Req.(*setAreaSettingsReq).AreaIndex)
	assert.Equal(t, uint8(255), mock.ServiceCalls[1].Req.(*setAreaSettingsReq).AreaIndex)
	assert.Equal(t, uint8(254), mock.ServiceCalls[2].Req.(*setAreaSettingsReq).AreaIndex)
}

func TestPlanPreview_RefusedIs409(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceResponder = func(_ string, _ any, res any) {
		r := res.(*setAreaSettingsRes)
		r.Success = false
		r.Message = "refused: mowing session running"
	}
	w, out := doAreaSettings(t, mock, "POST", "/api/mowglinext/plan/preview?area=0", nil)
	assert.Equal(t, http.StatusConflict, w.Code)
	assert.Equal(t, false, out["accepted"])
	assert.Contains(t, out["message"], "mowing session running")
}

func TestPlanPreview_BadAreaAndUnsupported(t *testing.T) {
	mock := types.NewMockRosProvider()
	w, _ := doAreaSettings(t, mock, "POST", "/api/mowglinext/plan/preview?area=x", nil)
	assert.Equal(t, http.StatusBadRequest, w.Code)
	w, _ = doAreaSettings(t, mock, "POST", "/api/mowglinext/plan/preview?area=254", nil)
	assert.Equal(t, http.StatusBadRequest, w.Code)
	assert.Empty(t, mock.ServiceCalls)

	mock.ServiceErr = errors.New("foxglove: CallService /behavior_tree_node/preview_plan: service not advertised")
	w, out := doAreaSettings(t, mock, "POST", "/api/mowglinext/plan/preview?area=0", nil)
	assert.Equal(t, http.StatusNotImplemented, w.Code)
	assert.Equal(t, false, out["supported"])
}
