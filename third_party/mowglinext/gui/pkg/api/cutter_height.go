package api

import (
	"context"
	"encoding/json"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
)

// Live deck (blade) height. mcu_node owns it: /cutter/set_height changes only
// the height (blade state untouched, refused while an interlock / e-stop is
// latched) and latches the commanded value on /cutter/height_mm (topic key
// "cutterHeight"). The GUI only relays.
const (
	cutterSetHeightService = "/cutter/set_height"
	cutterSetHeightType    = "mower_interfaces/srv/SetCutterHeight"
)

type setCutterHeightReq struct {
	HeightMm int16 `json:"height_mm"`
}

type setCutterHeightRes struct {
	Ok       bool   `json:"ok"`
	Message  string `json:"message"`
	HeightMm int16  `json:"height_mm"`
}

// CutterHeightRequest is the body of POST /mowglinext/cutter/height.
type CutterHeightRequest struct {
	HeightMm *int `json:"height_mm"`
}

// CutterHeightResponse is the result of POST /mowglinext/cutter/height.
type CutterHeightResponse struct {
	Ok       bool   `json:"ok"`
	Message  string `json:"message"`
	HeightMm int    `json:"height_mm"`
}

// CutterHeightRoutes registers POST /mowglinext/cutter/height.
//
// @Summary change the deck (blade) height without touching the blade state
// @Tags mowglinext
// @Accept json
// @Produce json
// @Param body body CutterHeightRequest true "height in mm (clamped 30-90 by the driver)"
// @Success 200 {object} CutterHeightResponse
// @Failure 400 {object} ErrorResponse
// @Failure 409 {object} CutterHeightResponse
// @Failure 501 {object} ErrorResponse
// @Router /mowglinext/cutter/height [post]
func CutterHeightRoutes(group *gin.RouterGroup, provider types.IRosProvider) {
	group.POST("/cutter/height", func(c *gin.Context) {
		var body CutterHeightRequest
		if err := json.NewDecoder(c.Request.Body).Decode(&body); err != nil || body.HeightMm == nil {
			c.JSON(400, ErrorResponse{Error: "height_mm is required"})
			return
		}
		mm := *body.HeightMm
		if mm < 0 || mm > 200 {
			c.JSON(400, ErrorResponse{Error: "height_mm out of range"})
			return
		}
		ctx, cancel := context.WithTimeout(c.Request.Context(), 5*time.Second)
		defer cancel()
		var res setCutterHeightRes
		err := provider.CallService(ctx, cutterSetHeightService,
			&setCutterHeightReq{HeightMm: int16(mm)}, &res, cutterSetHeightType)
		if isServiceNotAdvertised(err) {
			c.JSON(501, ErrorResponse{Error: "blade height change not supported by this robot"})
			return
		}
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		out := CutterHeightResponse{Ok: res.Ok, Message: res.Message, HeightMm: int(res.HeightMm)}
		if !res.Ok {
			c.JSON(409, out)
			return
		}
		c.JSON(200, out)
	})
}
