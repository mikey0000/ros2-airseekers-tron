package api

import (
	"context"
	"crypto/subtle"
	"errors"
	"net/http"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/providers"
)

// MowerTokenHeader carries the BLE tunnel token on the mutating Wi-Fi routes.
// The LAN API has no auth layer; these routes can cut the mower off the
// network, so they at least demand the token a paired phone already holds.
// A custom header also forces a CORS preflight, which the gin CORS config
// does not allow for it, so a foreign web page cannot send one.
const MowerTokenHeader = "X-Mower-Token"

// wifiService is what the routes need from providers.NetworkWifiProvider.
type wifiService interface {
	Status(ctx context.Context) providers.WifiStatus
	Scan(ctx context.Context, rescan bool) (providers.WifiScanResult, error)
	Connect(ctx context.Context, req providers.WifiConnectRequest) (providers.WifiConnectResult, error)
	Forget(ctx context.Context, uuid string) error
	SetBand(ctx context.Context, uuid, band string) (providers.WifiSavedNetwork, error)
	SetPowersave(ctx context.Context, uuid string, enabled bool) (providers.WifiSavedNetwork, error)
}

// WifiForgetRequest names the saved network to delete.
type WifiForgetRequest struct {
	UUID string `json:"uuid"`
}

// WifiBandRequest locks a saved network ("" uuid = the active one) to a band:
// "bg" = 2.4 GHz, "a" = 5 GHz, "" = automatic.
type WifiBandRequest struct {
	UUID string `json:"uuid"`
	Band string `json:"band"`
}

// WifiPowersaveRequest turns Wi-Fi power save on or off for a saved network
// ("" uuid = the active one).
type WifiPowersaveRequest struct {
	UUID    string `json:"uuid"`
	Enabled *bool  `json:"enabled"`
}

// NetworkWifiRoutes registers /network/wifi/*. status and scan are open; the
// routes that change the network need the X-Mower-Token header.
func NetworkWifiRoutes(r *gin.RouterGroup, wifi wifiService, tokens *bleTokens) {
	group := r.Group("/network/wifi")
	group.GET("/status", getWifiStatus(wifi))
	group.GET("/scan", getWifiScan(wifi))
	group.POST("/scan", postWifiScan(wifi))
	guarded := group.Group("", requireMowerToken(tokens))
	guarded.POST("/connect", postWifiConnect(wifi))
	guarded.POST("/forget", postWifiForget(wifi))
	guarded.POST("/band", postWifiBand(wifi))
	guarded.POST("/powersave", postWifiPowersave(wifi))
}

// requireMowerToken rejects a request whose X-Mower-Token is not the BLE token.
func requireMowerToken(tokens *bleTokens) gin.HandlerFunc {
	return func(c *gin.Context) {
		got := c.GetHeader(MowerTokenHeader)
		want, err := tokens.get()
		if err != nil {
			c.AbortWithStatusJSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
			return
		}
		if got == "" || subtle.ConstantTimeCompare([]byte(got), []byte(want)) != 1 {
			c.AbortWithStatusJSON(http.StatusUnauthorized, ErrorResponse{Error: "missing or wrong " + MowerTokenHeader + " (the Bluetooth token, Settings > Connection)"})
			return
		}
		c.Next()
	}
}

func wifiError(c *gin.Context, err error) {
	status := http.StatusBadGateway
	switch {
	case errors.Is(err, providers.ErrWifiUnavailable):
		status = http.StatusServiceUnavailable
	case errors.Is(err, providers.ErrWifiInvalid):
		status = http.StatusBadRequest
	case errors.Is(err, providers.ErrWifiNotFound):
		status = http.StatusNotFound
	case errors.Is(err, context.Canceled), errors.Is(err, context.DeadlineExceeded):
		status = http.StatusGatewayTimeout
	}
	c.JSON(status, ErrorResponse{Error: err.Error()})
}

// getWifiStatus returns the host Wi-Fi link and saved networks
//
// @Summary Wi-Fi status
// @Description NetworkManager's view of the Wi-Fi device: state, SSID, BSSID, frequency, strength (percent), IPv4 and the saved Wi-Fi networks. available=false (with a reason) when the host D-Bus socket or NetworkManager is missing.
// @Tags network
// @Produce json
// @Success 200 {object} providers.WifiStatus
// @Router /network/wifi/status [get]
func getWifiStatus(wifi wifiService) gin.HandlerFunc {
	return func(c *gin.Context) {
		c.Header("Cache-Control", "no-store")
		c.JSON(http.StatusOK, wifi.Status(c.Request.Context()))
	}
}

// getWifiScan returns the cached scan list
//
// @Summary Wi-Fi scan list (cached)
// @Tags network
// @Produce json
// @Success 200 {object} providers.WifiScanResult
// @Failure 503 {object} ErrorResponse
// @Router /network/wifi/scan [get]
func getWifiScan(wifi wifiService) gin.HandlerFunc {
	return func(c *gin.Context) {
		respondWifiScan(c, wifi, false)
	}
}

// postWifiScan triggers a scan and returns the list
//
// @Summary Wi-Fi scan
// @Description Asks NetworkManager for a fresh scan, waits up to ~6 s for it, then returns the access points (strongest first). A scan the driver refuses still returns the cached list (scanned=false).
// @Tags network
// @Produce json
// @Success 200 {object} providers.WifiScanResult
// @Failure 503 {object} ErrorResponse
// @Router /network/wifi/scan [post]
func postWifiScan(wifi wifiService) gin.HandlerFunc {
	return func(c *gin.Context) {
		respondWifiScan(c, wifi, true)
	}
}

func respondWifiScan(c *gin.Context, wifi wifiService, rescan bool) {
	res, err := wifi.Scan(c.Request.Context(), rescan)
	if err != nil {
		wifiError(c, err)
		return
	}
	c.Header("Cache-Control", "no-store")
	c.JSON(http.StatusOK, res)
}

// postWifiConnect joins a Wi-Fi network
//
// @Summary join a Wi-Fi network
// @Description Activates a saved network (uuid, or the saved profile for ssid) or creates one (WPA/WPA2/WPA3-personal or open). A password replaces the stored key. remember=false keeps a new profile in memory only. Needs the X-Mower-Token header.
// @Tags network
// @Accept json
// @Produce json
// @Param X-Mower-Token header string true "BLE tunnel token"
// @Param request body providers.WifiConnectRequest true "network"
// @Success 200 {object} providers.WifiConnectResult
// @Failure 400 {object} ErrorResponse
// @Failure 401 {object} ErrorResponse
// @Failure 404 {object} ErrorResponse
// @Failure 503 {object} ErrorResponse
// @Router /network/wifi/connect [post]
func postWifiConnect(wifi wifiService) gin.HandlerFunc {
	return func(c *gin.Context) {
		var req providers.WifiConnectRequest
		if err := c.ShouldBindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}
		res, err := wifi.Connect(c.Request.Context(), req)
		if err != nil {
			wifiError(c, err)
			return
		}
		c.JSON(http.StatusOK, res)
	}
}

// postWifiForget deletes a saved Wi-Fi network
//
// @Summary forget a Wi-Fi network
// @Tags network
// @Accept json
// @Produce json
// @Param X-Mower-Token header string true "BLE tunnel token"
// @Param request body WifiForgetRequest true "saved network"
// @Success 200 {object} OkResponse
// @Failure 400 {object} ErrorResponse
// @Failure 401 {object} ErrorResponse
// @Failure 404 {object} ErrorResponse
// @Router /network/wifi/forget [post]
func postWifiForget(wifi wifiService) gin.HandlerFunc {
	return func(c *gin.Context) {
		var req WifiForgetRequest
		if err := c.ShouldBindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}
		if err := wifi.Forget(c.Request.Context(), req.UUID); err != nil {
			wifiError(c, err)
			return
		}
		c.JSON(http.StatusOK, OkResponse{Ok: "forgotten"})
	}
}

// postWifiBand locks a saved network to a band
//
// @Summary Wi-Fi band lock
// @Description Sets 802-11-wireless.band ("bg" 2.4 GHz, "a" 5 GHz, "" automatic) and re-activates the network when it is the active one.
// @Tags network
// @Accept json
// @Produce json
// @Param X-Mower-Token header string true "BLE tunnel token"
// @Param request body WifiBandRequest true "band"
// @Success 200 {object} providers.WifiSavedNetwork
// @Failure 400 {object} ErrorResponse
// @Failure 401 {object} ErrorResponse
// @Failure 404 {object} ErrorResponse
// @Router /network/wifi/band [post]
func postWifiBand(wifi wifiService) gin.HandlerFunc {
	return func(c *gin.Context) {
		var req WifiBandRequest
		if err := c.ShouldBindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}
		net, err := wifi.SetBand(c.Request.Context(), req.UUID, req.Band)
		if err != nil {
			wifiError(c, err)
			return
		}
		c.JSON(http.StatusOK, net)
	}
}

// postWifiPowersave switches Wi-Fi power save
//
// @Summary Wi-Fi power save
// @Description Sets 802-11-wireless.powersave (3 enable / 2 disable) and re-activates the network when it is the active one.
// @Tags network
// @Accept json
// @Produce json
// @Param X-Mower-Token header string true "BLE tunnel token"
// @Param request body WifiPowersaveRequest true "power save"
// @Success 200 {object} providers.WifiSavedNetwork
// @Failure 400 {object} ErrorResponse
// @Failure 401 {object} ErrorResponse
// @Failure 404 {object} ErrorResponse
// @Router /network/wifi/powersave [post]
func postWifiPowersave(wifi wifiService) gin.HandlerFunc {
	return func(c *gin.Context) {
		var req WifiPowersaveRequest
		if err := c.ShouldBindJSON(&req); err != nil || req.Enabled == nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "body must be {\"uuid\": string, \"enabled\": bool}"})
			return
		}
		net, err := wifi.SetPowersave(c.Request.Context(), req.UUID, *req.Enabled)
		if err != nil {
			wifiError(c, err)
			return
		}
		c.JSON(http.StatusOK, net)
	}
}
