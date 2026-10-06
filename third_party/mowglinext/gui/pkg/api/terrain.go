package api

import (
	"context"
	"encoding/json"
	"fmt"
	"net/http"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
)

// Terrain memory actions (map server). Reuses the SetAreaSettings service type:
// area_index = area (255 = all areas), settings_json = the action object.
const terrainActionService = "/map_server_node/terrain_action"

var terrainActions = map[string]bool{"keepout": true, "confirm": true, "dismiss": true, "clear": true}

// TerrainActionRequest is the body of POST /mowglinext/terrain/action.
type TerrainActionRequest struct {
	AreaIndex *int   `json:"area_index"`
	Action    string `json:"action"`
	ClusterID *int   `json:"cluster_id"`
}

// TerrainActionResponse is the reply of POST /mowglinext/terrain/action.
type TerrainActionResponse struct {
	Supported bool   `json:"supported"`
	Success   bool   `json:"success"`
	Message   string `json:"message,omitempty"`
}

// TerrainRoutes registers POST /mowglinext/terrain/action.
//
// @Summary act on a terrain-memory incident cluster (keepout/confirm/dismiss) or clear the memory
// @Tags mowglinext
// @Accept json
// @Produce json
// @Param body body TerrainActionRequest true "action"
// @Success 200 {object} TerrainActionResponse
// @Router /mowglinext/terrain/action [post]
func TerrainRoutes(group *gin.RouterGroup, provider types.IRosProvider) {
	group.POST("/terrain/action", func(c *gin.Context) {
		var body TerrainActionRequest
		if err := json.NewDecoder(c.Request.Body).Decode(&body); err != nil {
			c.JSON(400, ErrorResponse{Error: "invalid JSON body: " + err.Error()})
			return
		}
		if !terrainActions[body.Action] {
			c.JSON(400, ErrorResponse{Error: "action must be one of keepout, confirm, dismiss, clear"})
			return
		}
		index := AreaSettingsDefaultsIndex
		if body.AreaIndex != nil {
			index = *body.AreaIndex
		}
		if index < 0 || index > 255 {
			c.JSON(400, ErrorResponse{Error: fmt.Sprintf("invalid area_index %d", index)})
			return
		}
		payload := map[string]any{"action": body.Action}
		if body.Action != "clear" {
			if body.ClusterID == nil {
				c.JSON(400, ErrorResponse{Error: "cluster_id is required for " + body.Action})
				return
			}
			payload["cluster_id"] = *body.ClusterID
		}
		encoded, _ := json.Marshal(payload)
		ctx, cancel := context.WithTimeout(c.Request.Context(), 5*time.Second)
		defer cancel()
		var res setAreaSettingsRes
		err := provider.CallService(ctx, terrainActionService,
			&setAreaSettingsReq{AreaIndex: uint8(index), SettingsJSON: string(encoded)},
			&res, areaSettingsSetType)
		if isServiceNotAdvertised(err) {
			c.JSON(http.StatusNotImplemented, TerrainActionResponse{
				Supported: false, Message: "terrain memory not supported by this robot",
			})
			return
		}
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		if !res.Success {
			c.JSON(400, ErrorResponse{Error: res.Message})
			return
		}
		c.JSON(200, TerrainActionResponse{Supported: true, Success: true, Message: res.Message})
	})
}
