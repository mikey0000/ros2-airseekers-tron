package api

// Per-vertex RTK quality of recorded boundaries (mower_mission, recording phase):
// after each area / no-go zone / path recording the mission node writes
// <recording_quality_dir>/<name>.json with the fix type and hdop of every saved
// vertex. The GUI container mounts the same maps volume at /ros2_ws/maps, so the
// files are read straight from disk and the map colours the vertices.

import (
	"encoding/json"
	"net/http"
	"os"
	"path/filepath"
	"sort"
	"strings"

	"github.com/gin-gonic/gin"
)

// recordingQualityDir is mission.yaml recording_quality_dir as seen from the GUI
// container. A var (not const) so tests can point it at a temp dir.
var recordingQualityDir = "/ros2_ws/maps/record_quality"

// recordingQualityMaxFileBytes skips anything that is not a recording sidecar.
const recordingQualityMaxFileBytes = 1 << 20

// RecordingQualityVertex is one saved vertex: map-frame x, y (m), GNSS fix type
// (GnssStatus FIX_TYPE_*: 3 = RTK fixed, 2 = RTK float; null = no sample) and hdop.
type RecordingQualityVertex struct {
	X       float64  `json:"x"`
	Y       float64  `json:"y"`
	FixType *int     `json:"fix_type"`
	Hdop    *float64 `json:"hdop"`
	Fixed   bool     `json:"fixed"`
}

// RecordingQuality is one <name>.json sidecar.
type RecordingQuality struct {
	Name         string                   `json:"name"`
	Kind         string                   `json:"kind"` // area | obstacle | path
	Saved        float64                  `json:"saved"`
	RequireFixed bool                     `json:"require_fixed"`
	NotFixed     int                      `json:"not_fixed"`
	Vertices     []RecordingQualityVertex `json:"vertices"`
}

// RecordingQualityResponse lists every readable sidecar, newest first.
type RecordingQualityResponse struct {
	Recordings []RecordingQuality `json:"recordings"`
}

// readRecordingQuality reads every *.json in dir; unreadable or malformed files
// are skipped. A missing dir is an empty list (no recording made yet).
func readRecordingQuality(dir string) []RecordingQuality {
	entries, err := os.ReadDir(dir)
	if err != nil {
		return []RecordingQuality{}
	}
	out := []RecordingQuality{}
	for _, e := range entries {
		if e.IsDir() || !strings.HasSuffix(e.Name(), ".json") {
			continue
		}
		info, err := e.Info()
		if err != nil || info.Size() > recordingQualityMaxFileBytes {
			continue
		}
		raw, err := os.ReadFile(filepath.Join(dir, e.Name()))
		if err != nil {
			continue
		}
		var rq RecordingQuality
		if err := json.Unmarshal(raw, &rq); err != nil || rq.Name == "" {
			continue
		}
		if rq.Vertices == nil {
			rq.Vertices = []RecordingQualityVertex{}
		}
		out = append(out, rq)
	}
	sort.SliceStable(out, func(i, j int) bool { return out[i].Saved > out[j].Saved })
	return out
}

// RecordingQualityRoute serves the recording sidecars.
//
// @Summary RTK quality of recorded boundary vertices
// @Description fix type and hdop per saved vertex of each recorded area, no-go zone or path (mower_mission recording_quality_dir)
// @Tags mowglinext
// @Produce json
// @Success 200 {object} RecordingQualityResponse
// @Router /mowglinext/recording-quality [get]
func RecordingQualityRoute(group *gin.RouterGroup) {
	group.GET("/recording-quality", func(c *gin.Context) {
		c.Header("Cache-Control", "no-cache")
		c.JSON(http.StatusOK, RecordingQualityResponse{Recordings: readRecordingQuality(recordingQualityDir)})
	})
}
