package api

import (
	"context"
	"net/http"
	"strings"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
)

// ---------------------------------------------------------------------------
// GNSS correction-source actions
// ---------------------------------------------------------------------------
//
// Correction *status* reaches the GUI through GnssStatus (/gps/status). This
// file only carries actions that change the correction source. Today that is
// pairing a LoRa base station for receivers whose RTCM arrives over a radio
// link instead of NTRIP. The service is optional: robots that do not offer it
// get a 503 with a clear message, so the endpoint is safe on any robot.

const (
	// LoRaPairService is the GNSS driver's LoRa base pairing service.
	LoRaPairService = "/mower_gps_node/set_lora"
	// LoRaPairServiceType is its ROS interface (sn, addr, channel, area -> result).
	LoRaPairServiceType = "mower_interfaces/srv/SetLoRa"

	loRaMinChannel   = 1
	loRaMaxChannel   = 75
	loRaMaxSNLength  = 64
	loRaMaxAreaChars = 8
	loRaPairTimeout  = 10 * time.Second
)

// LoRaPairRequest is the JSON body for POST /corrections/lora/pair.
type LoRaPairRequest struct {
	// SN is the base station serial number.
	SN string `json:"sn"`
	// Addr is the radio address (0-65535).
	Addr int `json:"addr"`
	// Channel is the radio channel (1-75).
	Channel int `json:"channel"`
	// Area is the frequency-region code; empty lets the driver derive it from SN.
	Area string `json:"area"`
}

// LoRaPairResponse reports whether the driver accepted the pairing request.
type LoRaPairResponse struct {
	Success bool   `json:"success"`
	Message string `json:"message,omitempty"`
}

// setLoRaReq mirrors the SetLoRa request fields by name for CDR encoding.
type setLoRaReq struct {
	SN      string `json:"sn"`
	Addr    uint16 `json:"addr"`
	Channel uint8  `json:"channel"`
	Area    string `json:"area"`
}

type setLoRaRes struct {
	Result bool `json:"result"`
}

// CorrectionsRoutes registers GNSS correction-source endpoints.
func CorrectionsRoutes(r *gin.RouterGroup, rosProvider types.IRosProvider) {
	group := r.Group("/corrections")
	group.POST("/lora/pair", postLoRaPair(rosProvider))
}

func validateLoRaPair(body *LoRaPairRequest) string {
	body.SN = strings.TrimSpace(body.SN)
	body.Area = strings.TrimSpace(body.Area)
	switch {
	case body.SN == "":
		return "sn is required"
	case len(body.SN) > loRaMaxSNLength:
		return "sn is too long"
	case body.Addr < 0 || body.Addr > 0xFFFF:
		return "addr must be between 0 and 65535"
	case body.Channel < loRaMinChannel || body.Channel > loRaMaxChannel:
		return "channel must be between 1 and 75"
	case len(body.Area) > loRaMaxAreaChars:
		return "area is too long"
	}
	return ""
}

// postLoRaPair forwards a pairing request to the GNSS driver.
//
// @Summary pair a LoRa RTK base station
// @Description Sends the base serial number, radio address, channel and region to the GNSS driver's LoRa pairing service. success=false means the driver refused (pairing disabled, or LoRa is not the active correction source).
// @Tags corrections
// @Accept json
// @Produce json
// @Param body body LoRaPairRequest true "pairing parameters"
// @Success 200 {object} LoRaPairResponse
// @Failure 400 {object} ErrorResponse
// @Failure 503 {object} ErrorResponse
// @Failure 500 {object} ErrorResponse
// @Router /corrections/lora/pair [post]
func postLoRaPair(rosProvider types.IRosProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		var body LoRaPairRequest
		if err := c.ShouldBindJSON(&body); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "Invalid request body: " + err.Error()})
			return
		}
		if msg := validateLoRaPair(&body); msg != "" {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: msg})
			return
		}

		ctx, cancel := context.WithTimeout(c.Request.Context(), loRaPairTimeout)
		defer cancel()

		req := setLoRaReq{
			SN:      body.SN,
			Addr:    uint16(body.Addr),
			Channel: uint8(body.Channel),
			Area:    body.Area,
		}
		var res setLoRaRes
		if err := rosProvider.CallService(ctx, LoRaPairService, &req, &res, LoRaPairServiceType); err != nil {
			if strings.Contains(err.Error(), "not advertised") {
				c.JSON(http.StatusServiceUnavailable, ErrorResponse{
					Error: "LoRa pairing is not available on this robot (" + LoRaPairService + " is not offered)",
				})
				return
			}
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: "Failed to call " + LoRaPairService + ": " + err.Error()})
			return
		}
		resp := LoRaPairResponse{Success: res.Result}
		if !res.Result {
			resp.Message = "the GNSS driver refused the pairing request"
		}
		c.JSON(http.StatusOK, resp)
	}
}
