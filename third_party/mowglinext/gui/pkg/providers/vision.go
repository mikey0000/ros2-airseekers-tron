package providers

import (
	"encoding/json"
	"sort"

	"github.com/mowglinext/mowglinext/pkg/msgs/geometry"
	"github.com/mowglinext/mowglinext/pkg/msgs/vision"
)

// DetectionSummary is the reduced form of a vision_msgs/Detection2DArray that
// the "detections" topic key delivers to the GUI. Poses, covariances and the
// non-top hypotheses are dropped server-side (a full Detection2DArray at
// camera rate would be mostly unused JSON on the websocket); each detection
// keeps its top class, score and axis-aligned box so the perception page can
// draw them over a raw camera stream.
type DetectionSummary struct {
	// Count is the number of detections in the message.
	Count int `json:"count"`
	// Classes is the sorted, de-duplicated set of top-hypothesis class ids.
	Classes []string `json:"classes"`
	// MaxScore is the best hypothesis score over all detections (0 if none).
	MaxScore float64 `json:"max_score"`
	// Stamp is the message header stamp.
	Stamp geometry.Stamp `json:"stamp"`
	// FrameId is the header frame (the source camera on multi-camera
	// detectors such as det_ros).
	FrameId string `json:"frame_id"`
	// Boxes are the detections (top hypothesis each), at most maxSummaryBoxes,
	// best score first.
	Boxes []DetectionBox `json:"boxes"`
}

// DetectionBox is one detection: top-left corner and size in source image
// pixels (det_ros: the camera image it ran on).
type DetectionBox struct {
	Class string  `json:"class"`
	Score float64 `json:"score"`
	X     float64 `json:"x"`
	Y     float64 `json:"y"`
	W     float64 `json:"w"`
	H     float64 `json:"h"`
}

const maxSummaryBoxes = 50

// SummarizeDetections reduces a Detection2DArray to a DetectionSummary. For
// each detection the highest-scoring hypothesis is taken as its class.
func SummarizeDetections(msg vision.Detection2DArray) DetectionSummary {
	out := DetectionSummary{
		Count:   len(msg.Detections),
		Classes: []string{},
		Boxes:   []DetectionBox{},
		Stamp:   msg.Header.Stamp,
		FrameId: msg.Header.FrameId,
	}
	seen := map[string]bool{}
	for _, det := range msg.Detections {
		best := -1
		for i, r := range det.Results {
			if best < 0 || r.Hypothesis.Score > det.Results[best].Hypothesis.Score {
				best = i
			}
		}
		if best < 0 {
			continue
		}
		h := det.Results[best].Hypothesis
		b := det.Bbox
		out.Boxes = append(out.Boxes, DetectionBox{
			Class: h.ClassId, Score: h.Score,
			X: b.Center.Position.X - b.SizeX/2, Y: b.Center.Position.Y - b.SizeY/2,
			W: b.SizeX, H: b.SizeY,
		})
		if h.Score > out.MaxScore {
			out.MaxScore = h.Score
		}
		if h.ClassId != "" && !seen[h.ClassId] {
			seen[h.ClassId] = true
			out.Classes = append(out.Classes, h.ClassId)
		}
	}
	sort.Strings(out.Classes)
	sort.SliceStable(out.Boxes, func(i, j int) bool { return out.Boxes[i].Score > out.Boxes[j].Score })
	if len(out.Boxes) > maxSummaryBoxes {
		out.Boxes = out.Boxes[:maxSummaryBoxes]
	}
	return out
}

// adaptDetections is the foxglove adapter for the "detections" key: snake_case
// Detection2DArray JSON in, DetectionSummary JSON out.
func adaptDetections(raw []byte) ([]byte, error) {
	var msg vision.Detection2DArray
	if err := json.Unmarshal(raw, &msg); err != nil {
		return nil, err
	}
	return json.Marshal(SummarizeDetections(msg))
}
