package api

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"net/http"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
)

// ---------------------------------------------------------------------------
// RTK base station (vendor LoRa base, Airseekers rtk_rover board)
// ---------------------------------------------------------------------------
//
// Pairing tells the mower's RTK board the base's LoRa address, channel and
// radio band; the base itself is never configured. The GNSS driver
// (um960_gps_driver) writes the vendor sentences ($GLIST, $GNCON) and reads the
// stored pairing back. These routes only forward to its services, so they are
// safe on any robot: without the services they answer 503.

const (
	// RtkBaseGetService reads the pairing stored on the RTK board.
	RtkBaseGetService     = "/mower_gps_node/get_lora"
	RtkBaseGetServiceType = "mower_interfaces/srv/GetLoRa"
	// RtkBaseResetService sends $BRESET to the RTK board (relayed to the base).
	RtkBaseResetService     = "/mower_gps_node/reset_base"
	RtkBaseResetServiceType = "std_srvs/srv/Trigger"
	// rtkBaseStatusTopic is the topicMap key of /mower_gps_node/lora_status.
	rtkBaseStatusTopic = "rtkBaseStatus"

	rtkBaseMaxAddr     = 0xFFFE
	rtkBaseCallTimeout = 10 * time.Second
)

// rtkBaseStatusWait is how long GET /rtk-base/status waits for the first
// status message after subscribing (a var so tests can shorten it).
var rtkBaseStatusWait = 2 * time.Second

// rtkBaseAreas are the region codes the board accepts (vendor setBand).
var rtkBaseAreas = map[string]bool{"00": true, "01": true, "02": true, "03": true, "F": true}

// rtkBaseBandNames maps the board's $GLIST band to the radio band.
var rtkBaseBandNames = map[string]string{
	"0": "Europe 863 MHz",
	"1": "Americas 902 MHz",
	"2": "Australia/NZ 927 MHz",
	"F": "all bands",
}

// RtkBaseAddr is a LoRa address given as a JSON number or as a decimal or
// 0x-prefixed hex string.
type RtkBaseAddr int

// UnmarshalJSON accepts 38183, "38183" and "0x9527".
func (a *RtkBaseAddr) UnmarshalJSON(raw []byte) error {
	var n int
	if err := json.Unmarshal(raw, &n); err == nil {
		*a = RtkBaseAddr(n)
		return nil
	}
	var s string
	if err := json.Unmarshal(raw, &s); err != nil {
		return errors.New("addr must be a number or a decimal / 0x hex string")
	}
	v, err := parseRtkBaseAddr(s)
	if err != nil {
		return err
	}
	*a = RtkBaseAddr(v)
	return nil
}

func parseRtkBaseAddr(s string) (int, error) {
	s = strings.TrimSpace(s)
	lower := strings.ToLower(s)
	var v uint64
	var err error
	if strings.HasPrefix(lower, "0x") {
		v, err = strconv.ParseUint(lower[2:], 16, 32)
	} else {
		v, err = strconv.ParseUint(s, 10, 32)
	}
	if err != nil {
		return 0, fmt.Errorf("addr %q is not a decimal or 0x hex number", s)
	}
	return int(v), nil
}

// RtkBasePairRequest is the body of POST /rtk-base/pair.
type RtkBasePairRequest struct {
	// Addr is the base's LoRa address, 0-65534 (decimal or "0x" hex string).
	Addr RtkBaseAddr `json:"addr" swaggertype:"string" example:"0x9527"`
	// Channel is the LoRa channel, 1-75.
	Channel int `json:"channel" example:"40"`
	// Area is the region code: 02 Europe/UK, 01 North America, 03 Australia/NZ
	// (00 = Europe too, F = all bands).
	Area string `json:"area" example:"02"`
}

// RtkBasePairing is a pairing stored on the RTK board.
type RtkBasePairing struct {
	Addr     int    `json:"addr"`
	AddrHex  string `json:"addr_hex"`
	Channel  int    `json:"channel"`
	Band     string `json:"band,omitempty"`
	BandName string `json:"band_name,omitempty"`
}

// RtkBasePairingResponse is the answer of GET /rtk-base/pairing.
type RtkBasePairingResponse struct {
	OK      bool            `json:"ok"`
	Pairing *RtkBasePairing `json:"pairing,omitempty"`
	Raw     string          `json:"raw,omitempty"`
	Message string          `json:"message,omitempty"`
}

// RtkBaseActionResponse is the answer of the pair and reset actions.
type RtkBaseActionResponse struct {
	Success bool   `json:"success"`
	Message string `json:"message,omitempty"`
}

// getLoRaRes mirrors mower_interfaces/srv/GetLoRa's response.
type getLoRaRes struct {
	OK      bool   `json:"ok"`
	Addr    int    `json:"addr"`
	Channel int    `json:"channel"`
	Band    string `json:"band"`
	Raw     string `json:"raw"`
	Message string `json:"message"`
}

// rtkBaseStatusCache holds the latest /mower_gps_node/lora_status payload. It
// subscribes on first use and keeps the subscription (one 1 Hz String topic).
type rtkBaseStatusCache struct {
	mu         sync.Mutex
	subscribed bool
	latest     json.RawMessage
	ready      chan struct{}
}

func newRtkBaseStatusCache() *rtkBaseStatusCache {
	return &rtkBaseStatusCache{ready: make(chan struct{})}
}

func (s *rtkBaseStatusCache) store(msg []byte) {
	var env struct {
		Data string `json:"data"`
	}
	payload := msg
	if err := json.Unmarshal(msg, &env); err == nil && env.Data != "" {
		payload = []byte(env.Data)
	}
	if !json.Valid(payload) {
		return
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	first := s.latest == nil
	s.latest = append(json.RawMessage(nil), payload...)
	if first {
		close(s.ready)
	}
}

// get subscribes on first use and waits up to rtkBaseStatusWait for a message.
func (s *rtkBaseStatusCache) get(ctx context.Context, rosProvider types.IRosProvider) (json.RawMessage, error) {
	s.mu.Lock()
	if !s.subscribed {
		if err := rosProvider.Subscribe(rtkBaseStatusTopic, "api-rtk-base-status", 0, s.store); err != nil {
			s.mu.Unlock()
			return nil, err
		}
		s.subscribed = true
	}
	s.mu.Unlock()
	select {
	case <-s.ready:
	case <-time.After(rtkBaseStatusWait):
	case <-ctx.Done():
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.latest, nil
}

// RtkBaseRoutes registers the RTK base station endpoints.
func RtkBaseRoutes(r *gin.RouterGroup, rosProvider types.IRosProvider) {
	status := newRtkBaseStatusCache()
	group := r.Group("/rtk-base")
	group.GET("/status", getRtkBaseStatus(rosProvider, status))
	group.GET("/pairing", getRtkBasePairing(rosProvider))
	group.POST("/pair", postRtkBasePair(rosProvider))
	group.POST("/reset", postRtkBaseReset(rosProvider))
}

func normalizeRtkBaseArea(area string) string {
	area = strings.ToUpper(strings.TrimSpace(area))
	if len(area) == 1 && area >= "0" && area <= "3" {
		area = "0" + area
	}
	return area
}

func validateRtkBasePair(body *RtkBasePairRequest) string {
	body.Area = normalizeRtkBaseArea(body.Area)
	switch {
	case body.Addr < 0 || body.Addr > rtkBaseMaxAddr:
		return "addr must be between 0 and 65534 (0x0000-0xFFFE)"
	case body.Channel < loRaMinChannel || body.Channel > loRaMaxChannel:
		return "channel must be between 1 and 75"
	case !rtkBaseAreas[body.Area]:
		return "area must be one of 00, 01, 02, 03, F"
	}
	return ""
}

// rtkBaseServiceError answers a failed service call: 503 when the GNSS driver
// does not offer the service, 500 otherwise.
func rtkBaseServiceError(c *gin.Context, service string, err error) {
	if strings.Contains(err.Error(), "not advertised") {
		c.JSON(http.StatusServiceUnavailable, ErrorResponse{
			Error: "RTK base pairing is not available on this robot (" + service + " is not offered)",
		})
		return
	}
	c.JSON(http.StatusInternalServerError, ErrorResponse{Error: "Failed to call " + service + ": " + err.Error()})
}

// getRtkBaseStatus returns the GNSS driver's LoRa base status.
//
// @Summary RTK base station status
// @Description Latest /mower_gps_node/lora_status JSON: stored pairing {addr, addr_hex, channel, band, band_name} or null, ref_station FINE|FAIL|null, nrtk_on, station_id, diff_age_s, fix_type, correction_source.
// @Tags rtk-base
// @Produce json
// @Success 200 {object} map[string]interface{}
// @Failure 503 {object} ErrorResponse
// @Router /rtk-base/status [get]
func getRtkBaseStatus(rosProvider types.IRosProvider, cache *rtkBaseStatusCache) gin.HandlerFunc {
	return func(c *gin.Context) {
		latest, err := cache.get(c.Request.Context(), rosProvider)
		if err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: "Failed to subscribe to the RTK base status: " + err.Error()})
			return
		}
		if latest == nil {
			c.JSON(http.StatusServiceUnavailable, ErrorResponse{Error: "no RTK base status from the GNSS driver yet"})
			return
		}
		c.Data(http.StatusOK, "application/json; charset=utf-8", latest)
	}
}

// getRtkBasePairing reads the pairing stored on the RTK board.
//
// @Summary read the stored RTK base pairing
// @Description Asks the RTK board for its stored LoRa address, channel and band ($GNCON,0000,FF). ok=false when the board did not answer.
// @Tags rtk-base
// @Produce json
// @Success 200 {object} RtkBasePairingResponse
// @Failure 503 {object} ErrorResponse
// @Failure 500 {object} ErrorResponse
// @Router /rtk-base/pairing [get]
func getRtkBasePairing(rosProvider types.IRosProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		ctx, cancel := context.WithTimeout(c.Request.Context(), rtkBaseCallTimeout)
		defer cancel()
		var res getLoRaRes
		if err := rosProvider.CallService(ctx, RtkBaseGetService, &struct{}{}, &res, RtkBaseGetServiceType); err != nil {
			rtkBaseServiceError(c, RtkBaseGetService, err)
			return
		}
		resp := RtkBasePairingResponse{OK: res.OK, Raw: res.Raw, Message: res.Message}
		if res.OK {
			resp.Pairing = &RtkBasePairing{
				Addr:     res.Addr,
				AddrHex:  fmt.Sprintf("%04X", res.Addr),
				Channel:  res.Channel,
				Band:     res.Band,
				BandName: rtkBaseBandNames[res.Band],
			}
		}
		c.JSON(http.StatusOK, resp)
	}
}

// postRtkBasePair writes a pairing to the RTK board.
//
// @Summary pair the RTK base station
// @Description Writes the band for the region and the base's LoRa address/channel to the RTK board, then reads them back. success=true only when the stored pairing matches. Refused (success=false) while NTRIP owns the receiver.
// @Tags rtk-base
// @Accept json
// @Produce json
// @Param body body RtkBasePairRequest true "pairing"
// @Success 200 {object} RtkBaseActionResponse
// @Failure 400 {object} ErrorResponse
// @Failure 503 {object} ErrorResponse
// @Failure 500 {object} ErrorResponse
// @Router /rtk-base/pair [post]
func postRtkBasePair(rosProvider types.IRosProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		var body RtkBasePairRequest
		if err := c.ShouldBindJSON(&body); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "Invalid request body: " + err.Error()})
			return
		}
		if msg := validateRtkBasePair(&body); msg != "" {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: msg})
			return
		}
		ctx, cancel := context.WithTimeout(c.Request.Context(), rtkBaseCallTimeout)
		defer cancel()
		req := setLoRaReq{Addr: uint16(body.Addr), Channel: uint8(body.Channel), Area: body.Area}
		var res setLoRaRes
		if err := rosProvider.CallService(ctx, LoRaPairService, &req, &res, LoRaPairServiceType); err != nil {
			rtkBaseServiceError(c, LoRaPairService, err)
			return
		}
		resp := RtkBaseActionResponse{Success: res.Result}
		if res.Result {
			resp.Message = fmt.Sprintf("stored address %d (0x%04X), channel %d, area %s", int(body.Addr), int(body.Addr), body.Channel, body.Area)
		} else {
			resp.Message = "the RTK board did not store the pairing (NTRIP may own the receiver, or the board did not answer)"
		}
		c.JSON(http.StatusOK, resp)
	}
}

// postRtkBaseReset asks the RTK board to reset (relayed to the base).
//
// @Summary reboot the RTK base station
// @Description Sends $BRESET to the RTK board, which relays a reset to the base. Only while the LoRa base is the correction source.
// @Tags rtk-base
// @Produce json
// @Success 200 {object} RtkBaseActionResponse
// @Failure 503 {object} ErrorResponse
// @Failure 500 {object} ErrorResponse
// @Router /rtk-base/reset [post]
func postRtkBaseReset(rosProvider types.IRosProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		ctx, cancel := context.WithTimeout(c.Request.Context(), rtkBaseCallTimeout)
		defer cancel()
		var res triggerRes
		if err := rosProvider.CallService(ctx, RtkBaseResetService, &struct{}{}, &res, RtkBaseResetServiceType); err != nil {
			rtkBaseServiceError(c, RtkBaseResetService, err)
			return
		}
		c.JSON(http.StatusOK, RtkBaseActionResponse{Success: res.Success, Message: res.Message})
	}
}
