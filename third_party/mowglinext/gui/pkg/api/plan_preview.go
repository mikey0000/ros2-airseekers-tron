package api

import (
	"context"
	"fmt"
	"net/http"
	"strconv"
	"strings"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
)

// Plan preview: the mission (behavior_tree_node) plans an area with its current
// per-area settings exactly as a mission's PLANNING step would, WITHOUT starting a
// mission or moving the robot. The service only accepts the request; the plan
// arrives asynchronously on /coverage/full_plan (topic "path") and the latched
// JSON summary on /behavior_tree_node/preview_summary (topic "previewSummary").
// The service reuses mower_interfaces/srv/SetAreaSettings: area_index is the area
// (255 = all areas, 254 = clear the preview); settings_json is reserved.
const (
	planPreviewService = "/behavior_tree_node/preview_plan"
	planPreviewType    = "mower_interfaces/srv/SetAreaSettings"
	planPreviewAll     = 255
	planPreviewClear   = 254
)

// PlanPreviewResponse is the body of POST/DELETE /mowglinext/plan/preview.
type PlanPreviewResponse struct {
	Supported bool   `json:"supported"`
	Accepted  bool   `json:"accepted"`
	Area      int    `json:"area"`
	Message   string `json:"message"`
}

func callPlanPreview(ctx context.Context, provider types.IRosProvider, index int) (setAreaSettingsRes, error) {
	var res setAreaSettingsRes
	err := provider.CallService(ctx, planPreviewService,
		&setAreaSettingsReq{AreaIndex: uint8(index)}, &res, planPreviewType)
	return res, err
}

func planPreviewReply(c *gin.Context, provider types.IRosProvider, index, area int) {
	ctx, cancel := context.WithTimeout(c.Request.Context(), 10*time.Second)
	defer cancel()
	res, err := callPlanPreview(ctx, provider, index)
	if err != nil {
		if strings.Contains(err.Error(), "not advertised") {
			c.JSON(http.StatusNotImplemented, PlanPreviewResponse{
				Supported: false, Area: area, Message: "plan preview not supported by this robot"})
			return
		}
		c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
		return
	}
	if !res.Success {
		// Refused (mission running, emergency, ...): 409 with the mission's reason.
		c.JSON(http.StatusConflict, PlanPreviewResponse{
			Supported: true, Accepted: false, Area: area, Message: res.Message})
		return
	}
	c.JSON(http.StatusOK, PlanPreviewResponse{
		Supported: true, Accepted: true, Area: area, Message: res.Message})
}

func parsePreviewArea(raw string) (int, error) {
	if raw == "" || raw == "-1" || raw == "all" {
		return -1, nil
	}
	n, err := strconv.Atoi(raw)
	if err != nil || n < 0 || n >= planPreviewClear {
		return 0, fmt.Errorf("invalid area %q", raw)
	}
	return n, nil
}

// PlanPreviewRoutes registers the plan preview routes.
//
// @Summary preview the coverage plan of an area (or all areas) without mowing
// @Tags mowglinext
// @Produce json
// @Param area query int false "0-based area index; omitted or -1 = all areas"
// @Success 200 {object} PlanPreviewResponse
// @Failure 409 {object} PlanPreviewResponse
// @Router /mowglinext/plan/preview [post]
func PlanPreviewRoutes(group *gin.RouterGroup, provider types.IRosProvider) {
	group.POST("/plan/preview", func(c *gin.Context) {
		area, err := parsePreviewArea(c.Query("area"))
		if err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}
		index := area
		if area < 0 {
			index = planPreviewAll
		}
		planPreviewReply(c, provider, index, area)
	})
	// DELETE clears the drawn preview (refused while a mission runs).
	group.DELETE("/plan/preview", func(c *gin.Context) {
		planPreviewReply(c, provider, planPreviewClear, -1)
	})
}
