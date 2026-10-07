package api

import (
	"errors"
	"net/http"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestCutterHeight_CallsDriverService(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceResponder = func(_ string, _ any, res any) {
		r := res.(*setCutterHeightRes)
		r.Ok, r.Message, r.HeightMm = true, "height 65 mm (blade on)", 65
	}
	w, out := doAreaSettings(t, mock, "POST", "/api/mowglinext/cutter/height", map[string]any{"height_mm": 65})
	require.Equal(t, 200, w.Code)
	assert.Equal(t, true, out["ok"])
	assert.Equal(t, float64(65), out["height_mm"])
	assert.Equal(t, "height 65 mm (blade on)", out["message"])
	require.Len(t, mock.ServiceCalls, 1)
	assert.Equal(t, cutterSetHeightService, mock.ServiceCalls[0].Service)
	assert.Equal(t, int16(65), mock.ServiceCalls[0].Req.(*setCutterHeightReq).HeightMm)
}

func TestCutterHeight_RefusedIs409WithMessage(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceResponder = func(_ string, _ any, res any) {
		r := res.(*setCutterHeightRes)
		r.Ok, r.Message, r.HeightMm = false, "refused: interlock / e-stop latched", 50
	}
	w, out := doAreaSettings(t, mock, "POST", "/api/mowglinext/cutter/height", map[string]any{"height_mm": 70})
	assert.Equal(t, http.StatusConflict, w.Code)
	assert.Equal(t, false, out["ok"])
	assert.Equal(t, "refused: interlock / e-stop latched", out["message"])
}

func TestCutterHeight_BadBodyAndNotAdvertised(t *testing.T) {
	mock := types.NewMockRosProvider()
	w, _ := doAreaSettings(t, mock, "POST", "/api/mowglinext/cutter/height", map[string]any{})
	assert.Equal(t, 400, w.Code)
	assert.Empty(t, mock.ServiceCalls)

	mock.ServiceErr = errors.New("foxglove: CallService /cutter/set_height: service not advertised")
	w, _ = doAreaSettings(t, mock, "POST", "/api/mowglinext/cutter/height", map[string]any{"height_mm": 50})
	assert.Equal(t, http.StatusNotImplemented, w.Code)
}
