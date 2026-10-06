package api

import (
	"bytes"
	"encoding/json"
	"errors"
	"net/http"
	"net/http/httptest"
	"testing"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func setupCorrectionsRouter(provider types.IRosProvider) *gin.Engine {
	gin.SetMode(gin.TestMode)
	r := gin.New()
	CorrectionsRoutes(r.Group("/api"), provider)
	return r
}

func postLoRaPairJSON(t *testing.T, router *gin.Engine, body any) *httptest.ResponseRecorder {
	t.Helper()
	raw, err := json.Marshal(body)
	require.NoError(t, err)
	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/corrections/lora/pair", bytes.NewReader(raw))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)
	return w
}

func TestLoRaPair_ForwardsToDriverService(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceResponder = func(_ string, _ any, res any) {
		res.(*setLoRaRes).Result = true
	}
	router := setupCorrectionsRouter(mock)

	w := postLoRaPairJSON(t, router, map[string]any{"sn": " 3001034903AB ", "addr": 4660, "channel": 17, "area": ""})

	assert.Equal(t, http.StatusOK, w.Code)
	var resp LoRaPairResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &resp))
	assert.True(t, resp.Success)
	require.Len(t, mock.ServiceCalls, 1)
	assert.Equal(t, LoRaPairService, mock.ServiceCalls[0].Service)
	assert.Equal(t, &setLoRaReq{SN: "3001034903AB", Addr: 4660, Channel: 17, Area: ""}, mock.ServiceCalls[0].Req)

	// Field names must match the .srv for CDR encoding.
	encoded, _ := json.Marshal(mock.ServiceCalls[0].Req)
	assert.JSONEq(t, `{"sn":"3001034903AB","addr":4660,"channel":17,"area":""}`, string(encoded))
}

func TestLoRaPair_DriverRefusalIsReportedNotAnError(t *testing.T) {
	mock := types.NewMockRosProvider() // responder leaves result=false
	router := setupCorrectionsRouter(mock)

	w := postLoRaPairJSON(t, router, map[string]any{"sn": "SN1", "addr": 1, "channel": 1})

	assert.Equal(t, http.StatusOK, w.Code)
	var resp LoRaPairResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &resp))
	assert.False(t, resp.Success)
	assert.NotEmpty(t, resp.Message)
}

func TestLoRaPair_ServiceNotOfferedIs503(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceErr = errors.New("foxglove: CallService /mower_gps_node/set_lora: service not advertised")
	router := setupCorrectionsRouter(mock)

	w := postLoRaPairJSON(t, router, map[string]any{"sn": "SN1", "addr": 1, "channel": 1})

	assert.Equal(t, http.StatusServiceUnavailable, w.Code)
	assert.Contains(t, w.Body.String(), "not available")
}

func TestLoRaPair_TransportErrorIs500(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceErr = errors.New("foxglove: CallService: no connection")
	router := setupCorrectionsRouter(mock)

	w := postLoRaPairJSON(t, router, map[string]any{"sn": "SN1", "addr": 1, "channel": 1})

	assert.Equal(t, http.StatusInternalServerError, w.Code)
}

func TestLoRaPair_ValidatesInput(t *testing.T) {
	cases := map[string]map[string]any{
		"missing sn":      {"addr": 1, "channel": 1},
		"blank sn":        {"sn": "   ", "addr": 1, "channel": 1},
		"addr too large":  {"sn": "SN1", "addr": 70000, "channel": 1},
		"negative addr":   {"sn": "SN1", "addr": -1, "channel": 1},
		"channel zero":    {"sn": "SN1", "addr": 1, "channel": 0},
		"channel too big": {"sn": "SN1", "addr": 1, "channel": 76},
		"area too long":   {"sn": "SN1", "addr": 1, "channel": 1, "area": "ABCDEFGHIJ"},
	}
	for name, body := range cases {
		t.Run(name, func(t *testing.T) {
			mock := types.NewMockRosProvider()
			router := setupCorrectionsRouter(mock)
			w := postLoRaPairJSON(t, router, body)
			assert.Equal(t, http.StatusBadRequest, w.Code)
			assert.Empty(t, mock.ServiceCalls)
		})
	}

	t.Run("malformed json", func(t *testing.T) {
		mock := types.NewMockRosProvider()
		router := setupCorrectionsRouter(mock)
		w := httptest.NewRecorder()
		req, _ := http.NewRequest("POST", "/api/corrections/lora/pair", bytes.NewReader([]byte("{")))
		req.Header.Set("Content-Type", "application/json")
		router.ServeHTTP(w, req)
		assert.Equal(t, http.StatusBadRequest, w.Code)
		assert.Empty(t, mock.ServiceCalls)
	})
}
