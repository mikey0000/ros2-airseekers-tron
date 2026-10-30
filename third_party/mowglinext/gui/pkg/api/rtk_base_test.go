package api

import (
	"bytes"
	"encoding/json"
	"errors"
	"net/http"
	"net/http/httptest"
	"testing"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func setupRtkBaseRouter(provider types.IRosProvider) *gin.Engine {
	gin.SetMode(gin.TestMode)
	r := gin.New()
	RtkBaseRoutes(r.Group("/api"), provider)
	return r
}

func rtkBaseRequest(t *testing.T, router *gin.Engine, method, path string, body any) *httptest.ResponseRecorder {
	t.Helper()
	var reader *bytes.Reader
	if body != nil {
		raw, err := json.Marshal(body)
		require.NoError(t, err)
		reader = bytes.NewReader(raw)
	} else {
		reader = bytes.NewReader(nil)
	}
	w := httptest.NewRecorder()
	req, _ := http.NewRequest(method, path, reader)
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)
	return w
}

func TestRtkBasePair_UniversalCodeHexAddr(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceResponder = func(_ string, _ any, res any) {
		res.(*setLoRaRes).Result = true
	}
	router := setupRtkBaseRouter(mock)

	w := rtkBaseRequest(t, router, "POST", "/api/rtk-base/pair", map[string]any{"addr": "0x9527", "channel": 40, "area": "02"})

	assert.Equal(t, http.StatusOK, w.Code)
	var resp RtkBaseActionResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &resp))
	assert.True(t, resp.Success)
	assert.Contains(t, resp.Message, "38183 (0x9527)")
	require.Len(t, mock.ServiceCalls, 1)
	assert.Equal(t, LoRaPairService, mock.ServiceCalls[0].Service)
	// Field names must match SetLoRa.srv for CDR encoding; sn stays empty (band from area).
	encoded, _ := json.Marshal(mock.ServiceCalls[0].Req)
	assert.JSONEq(t, `{"sn":"","addr":38183,"channel":40,"area":"02"}`, string(encoded))
}

func TestRtkBasePair_AcceptsDecimalNumberAndString(t *testing.T) {
	for _, addr := range []any{38183, "38183", " 0X9527 "} {
		mock := types.NewMockRosProvider()
		router := setupRtkBaseRouter(mock)
		w := rtkBaseRequest(t, router, "POST", "/api/rtk-base/pair", map[string]any{"addr": addr, "channel": 40, "area": "3"})
		require.Equal(t, http.StatusOK, w.Code, "addr %v: %s", addr, w.Body.String())
		require.Len(t, mock.ServiceCalls, 1)
		assert.Equal(t, &setLoRaReq{Addr: 38183, Channel: 40, Area: "03"}, mock.ServiceCalls[0].Req)
	}
}

func TestRtkBasePair_Validation(t *testing.T) {
	cases := []map[string]any{
		{"addr": 65535, "channel": 40, "area": "02"}, // 0xFFFF is not a base address
		{"addr": "0x10000", "channel": 40, "area": "02"},
		{"addr": -1, "channel": 40, "area": "02"},
		{"addr": "zz", "channel": 40, "area": "02"},
		{"addr": 1, "channel": 0, "area": "02"},
		{"addr": 1, "channel": 76, "area": "02"},
		{"addr": 1, "channel": 40, "area": "04"},
		{"addr": 1, "channel": 40, "area": ""},
	}
	for _, body := range cases {
		mock := types.NewMockRosProvider()
		router := setupRtkBaseRouter(mock)
		w := rtkBaseRequest(t, router, "POST", "/api/rtk-base/pair", body)
		assert.Equal(t, http.StatusBadRequest, w.Code, "%v", body)
		assert.Empty(t, mock.ServiceCalls, "%v", body)
	}
}

func TestRtkBasePair_RefusalIsReportedNotAnError(t *testing.T) {
	mock := types.NewMockRosProvider() // result=false: NTRIP owns the receiver or no read-back
	router := setupRtkBaseRouter(mock)

	w := rtkBaseRequest(t, router, "POST", "/api/rtk-base/pair", map[string]any{"addr": 1, "channel": 1, "area": "F"})

	assert.Equal(t, http.StatusOK, w.Code)
	var resp RtkBaseActionResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &resp))
	assert.False(t, resp.Success)
	assert.NotEmpty(t, resp.Message)
}

func TestRtkBase_ServiceNotOfferedIs503(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceErr = errors.New("foxglove: CallService: service not advertised")
	router := setupRtkBaseRouter(mock)

	for _, call := range []struct{ method, path string }{
		{"POST", "/api/rtk-base/pair"}, {"GET", "/api/rtk-base/pairing"}, {"POST", "/api/rtk-base/reset"},
	} {
		w := rtkBaseRequest(t, router, call.method, call.path, map[string]any{"addr": 1, "channel": 1, "area": "02"})
		assert.Equal(t, http.StatusServiceUnavailable, w.Code, call.path)
		assert.Contains(t, w.Body.String(), "not available", call.path)
	}
}

func TestRtkBase_OtherServiceErrorIs500(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceErr = errors.New("timeout")
	router := setupRtkBaseRouter(mock)

	w := rtkBaseRequest(t, router, "GET", "/api/rtk-base/pairing", nil)
	assert.Equal(t, http.StatusInternalServerError, w.Code)
}

func TestRtkBasePairing_ReadsTheBoard(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceResponder = func(service string, _ any, res any) {
		require.Equal(t, RtkBaseGetService, service)
		*res.(*getLoRaRes) = getLoRaRes{OK: true, Addr: 0x9527, Channel: 40, Band: "2",
			Raw: "$GNCON,9527,28*48\n$GLIST,2*5B"}
	}
	router := setupRtkBaseRouter(mock)

	w := rtkBaseRequest(t, router, "GET", "/api/rtk-base/pairing", nil)

	assert.Equal(t, http.StatusOK, w.Code)
	var resp RtkBasePairingResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &resp))
	assert.True(t, resp.OK)
	assert.Equal(t, &RtkBasePairing{Addr: 38183, AddrHex: "9527", Channel: 40, Band: "2",
		BandName: "Australia/NZ 927 MHz"}, resp.Pairing)
	assert.Contains(t, resp.Raw, "$GLIST,2")
}

func TestRtkBasePairing_NoBoardAnswer(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceResponder = func(_ string, _ any, res any) {
		*res.(*getLoRaRes) = getLoRaRes{OK: false, Message: "no $GNCON reply"}
	}
	router := setupRtkBaseRouter(mock)

	w := rtkBaseRequest(t, router, "GET", "/api/rtk-base/pairing", nil)

	assert.Equal(t, http.StatusOK, w.Code)
	assert.JSONEq(t, `{"ok":false,"message":"no $GNCON reply"}`, w.Body.String())
}

func TestRtkBaseReset_ForwardsTrigger(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceResponder = func(_ string, _ any, res any) {
		*res.(*triggerRes) = triggerRes{Success: true, Message: "sent $BRESET*17"}
	}
	router := setupRtkBaseRouter(mock)

	w := rtkBaseRequest(t, router, "POST", "/api/rtk-base/reset", nil)

	assert.Equal(t, http.StatusOK, w.Code)
	assert.JSONEq(t, `{"success":true,"message":"sent $BRESET*17"}`, w.Body.String())
	require.Len(t, mock.ServiceCalls, 1)
	assert.Equal(t, RtkBaseResetService, mock.ServiceCalls[0].Service)
}

func TestRtkBaseStatus_LatestTopicMessage(t *testing.T) {
	old := rtkBaseStatusWait
	rtkBaseStatusWait = 50 * time.Millisecond
	defer func() { rtkBaseStatusWait = old }()
	mock := types.NewMockRosProvider()
	router := setupRtkBaseRouter(mock)

	// Before the driver has published anything: 503 (and the subscription is now open).
	w := rtkBaseRequest(t, router, "GET", "/api/rtk-base/status", nil)
	assert.Equal(t, http.StatusServiceUnavailable, w.Code)

	inner := `{"pairing":{"addr":38183,"addr_hex":"9527","channel":40,"band":"0","band_name":"Europe 863 MHz"},"ref_station":"FINE","nrtk_on":false}`
	envelope, _ := json.Marshal(map[string]string{"data": inner})
	mock.Dispatch(rtkBaseStatusTopic, envelope)

	w = rtkBaseRequest(t, router, "GET", "/api/rtk-base/status", nil)
	assert.Equal(t, http.StatusOK, w.Code)
	assert.JSONEq(t, inner, w.Body.String())

	mock.Dispatch(rtkBaseStatusTopic, []byte(`{"data":"not json"}`)) // ignored
	w = rtkBaseRequest(t, router, "GET", "/api/rtk-base/status", nil)
	assert.JSONEq(t, inner, w.Body.String())
}
