package api

import (
	"context"
	"encoding/json"
	"fmt"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/providers"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// stubWifi records what the routes asked for.
type stubWifi struct {
	calls     []string
	connect   providers.WifiConnectRequest
	band      string
	powersave *bool
	err       error
}

func (s *stubWifi) Status(context.Context) providers.WifiStatus {
	s.calls = append(s.calls, "status")
	return providers.WifiStatus{Available: true, Interface: "wlan0", SSID: "ExampleHome", Saved: []providers.WifiSavedNetwork{}}
}

func (s *stubWifi) Scan(_ context.Context, rescan bool) (providers.WifiScanResult, error) {
	s.calls = append(s.calls, fmt.Sprintf("scan:%v", rescan))
	return providers.WifiScanResult{Scanned: rescan, AccessPoints: []providers.WifiAccessPoint{{SSID: "ExampleHome"}}}, s.err
}

func (s *stubWifi) Connect(_ context.Context, req providers.WifiConnectRequest) (providers.WifiConnectResult, error) {
	s.calls = append(s.calls, "connect")
	s.connect = req
	return providers.WifiConnectResult{UUID: "uuid-new", Created: true}, s.err
}

func (s *stubWifi) Forget(_ context.Context, uuid string) error {
	s.calls = append(s.calls, "forget:"+uuid)
	return s.err
}

func (s *stubWifi) SetBand(_ context.Context, uuid, band string) (providers.WifiSavedNetwork, error) {
	s.calls = append(s.calls, "band:"+uuid)
	s.band = band
	return providers.WifiSavedNetwork{UUID: uuid, Band: band}, s.err
}

func (s *stubWifi) SetPowersave(_ context.Context, uuid string, enabled bool) (providers.WifiSavedNetwork, error) {
	s.calls = append(s.calls, "powersave:"+uuid)
	s.powersave = &enabled
	return providers.WifiSavedNetwork{UUID: uuid}, s.err
}

func setupWifiRouter(t *testing.T) (*gin.Engine, *stubWifi, string) {
	t.Helper()
	gin.SetMode(gin.TestMode)
	db := types.NewMockDBProvider()
	tokens := newBleTokens(db)
	token, err := tokens.get()
	require.NoError(t, err)
	wifi := &stubWifi{}
	r := gin.New()
	NetworkWifiRoutes(r.Group("/api"), wifi, tokens)
	return r, wifi, token
}

func wifiRequest(r *gin.Engine, method, path, token, body string) *httptest.ResponseRecorder {
	req := httptest.NewRequest(method, path, strings.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	if token != "" {
		req.Header.Set(MowerTokenHeader, token)
	}
	w := httptest.NewRecorder()
	r.ServeHTTP(w, req)
	return w
}

func TestWifiStatusAndScanNeedNoToken(t *testing.T) {
	r, wifi, _ := setupWifiRouter(t)
	w := wifiRequest(r, http.MethodGet, "/api/network/wifi/status", "", "")
	require.Equal(t, http.StatusOK, w.Code)
	var st providers.WifiStatus
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &st))
	assert.True(t, st.Available)
	assert.Equal(t, "no-store", w.Header().Get("Cache-Control"))

	assert.Equal(t, http.StatusOK, wifiRequest(r, http.MethodGet, "/api/network/wifi/scan", "", "").Code)
	assert.Equal(t, http.StatusOK, wifiRequest(r, http.MethodPost, "/api/network/wifi/scan", "", "").Code)
	assert.Equal(t, []string{"status", "scan:false", "scan:true"}, wifi.calls)
}

func TestWifiMutatingRoutesRejectMissingOrWrongToken(t *testing.T) {
	r, wifi, token := setupWifiRouter(t)
	routes := map[string]string{
		"/api/network/wifi/connect":   `{"ssid":"ExampleHome","password":"password1"}`,
		"/api/network/wifi/forget":    `{"uuid":"u"}`,
		"/api/network/wifi/band":      `{"band":"bg"}`,
		"/api/network/wifi/powersave": `{"enabled":false}`,
	}
	for path, body := range routes {
		for _, bad := range []string{"", "WRONG", token + "X", strings.ToLower(token)} {
			w := wifiRequest(r, http.MethodPost, path, bad, body)
			assert.Equal(t, http.StatusUnauthorized, w.Code, "%s with token %q", path, bad)
		}
	}
	assert.Empty(t, wifi.calls, "nothing reaches NetworkManager without the token")
}

func TestWifiMutatingRoutesWithToken(t *testing.T) {
	r, wifi, token := setupWifiRouter(t)

	w := wifiRequest(r, http.MethodPost, "/api/network/wifi/connect", token, `{"ssid":"ExampleHome","password":"password1","remember":false}`)
	require.Equal(t, http.StatusOK, w.Code, w.Body.String())
	assert.Equal(t, "ExampleHome", wifi.connect.SSID)
	assert.Equal(t, "password1", wifi.connect.Password)
	require.NotNil(t, wifi.connect.Remember)
	assert.False(t, *wifi.connect.Remember)

	assert.Equal(t, http.StatusOK, wifiRequest(r, http.MethodPost, "/api/network/wifi/forget", token, `{"uuid":"u1"}`).Code)
	assert.Equal(t, http.StatusOK, wifiRequest(r, http.MethodPost, "/api/network/wifi/band", token, `{"uuid":"u2","band":"a"}`).Code)
	assert.Equal(t, "a", wifi.band)
	assert.Equal(t, http.StatusOK, wifiRequest(r, http.MethodPost, "/api/network/wifi/powersave", token, `{"enabled":false}`).Code)
	require.NotNil(t, wifi.powersave)
	assert.False(t, *wifi.powersave)
	assert.Equal(t, []string{"connect", "forget:u1", "band:u2", "powersave:"}, wifi.calls)
}

func TestWifiTokenFollowsRotation(t *testing.T) {
	gin.SetMode(gin.TestMode)
	db := types.NewMockDBProvider()
	tokens := newBleTokens(db)
	r := gin.New()
	bleTokenRoutes(r.Group("/api"), tokens)
	NetworkWifiRoutes(r.Group("/api"), &stubWifi{}, tokens)

	old, err := tokens.get()
	require.NoError(t, err)
	w := wifiRequest(r, http.MethodPost, "/api/ble/token/rotate", "", "")
	require.Equal(t, http.StatusOK, w.Code)
	var fresh BleTokenResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &fresh))

	assert.Equal(t, http.StatusUnauthorized, wifiRequest(r, http.MethodPost, "/api/network/wifi/forget", old, `{"uuid":"u"}`).Code)
	assert.Equal(t, http.StatusOK, wifiRequest(r, http.MethodPost, "/api/network/wifi/forget", fresh.Token, `{"uuid":"u"}`).Code)
}

func TestWifiErrorStatuses(t *testing.T) {
	cases := map[error]int{
		fmt.Errorf("%w: no socket", providers.ErrWifiUnavailable): http.StatusServiceUnavailable,
		fmt.Errorf("%w: bad", providers.ErrWifiInvalid):           http.StatusBadRequest,
		fmt.Errorf("%w: gone", providers.ErrWifiNotFound):         http.StatusNotFound,
		fmt.Errorf("nm said no"):                                  http.StatusBadGateway,
	}
	for err, want := range cases {
		r, wifi, token := setupWifiRouter(t)
		wifi.err = err
		w := wifiRequest(r, http.MethodPost, "/api/network/wifi/forget", token, `{"uuid":"u"}`)
		assert.Equal(t, want, w.Code, err.Error())
		var body ErrorResponse
		require.NoError(t, json.Unmarshal(w.Body.Bytes(), &body))
		assert.Equal(t, err.Error(), body.Error)
	}
}

func TestWifiPowersaveNeedsEnabled(t *testing.T) {
	r, wifi, token := setupWifiRouter(t)
	assert.Equal(t, http.StatusBadRequest, wifiRequest(r, http.MethodPost, "/api/network/wifi/powersave", token, `{"uuid":"u"}`).Code)
	assert.Equal(t, http.StatusBadRequest, wifiRequest(r, http.MethodPost, "/api/network/wifi/connect", token, `not json`).Code)
	assert.Empty(t, wifi.calls)
}
