package providers

import (
	"encoding/json"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// Snake_case JSON exactly as gui/pkg/foxglove DeserializeCDR emits a
// vision_msgs/msg/Detection2DArray (det_ros puts the class *name* in class_id).
const detectionArrayJSON = `{
  "header": {"stamp": {"sec": 1700000000, "nanosec": 5}, "frame_id": "left_oa_camera"},
  "detections": [
    {"header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""},
     "results": [
       {"hypothesis": {"class_id": "person", "score": 0.81}, "pose": {"pose": {"position": {"x":0,"y":0,"z":0}, "orientation": {"x":0,"y":0,"z":0,"w":1}}, "covariance": []}},
       {"hypothesis": {"class_id": "dog", "score": 0.10}, "pose": {"pose": {"position": {"x":0,"y":0,"z":0}, "orientation": {"x":0,"y":0,"z":0,"w":1}}, "covariance": []}}
     ],
     "bbox": {"center": {"position": {"x": 10, "y": 20}, "theta": 0}, "size_x": 5, "size_y": 6},
     "id": ""},
    {"header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""},
     "results": [{"hypothesis": {"class_id": "chair", "score": 0.55}, "pose": {"pose": {"position": {"x":0,"y":0,"z":0}, "orientation": {"x":0,"y":0,"z":0,"w":1}}, "covariance": []}}],
     "bbox": {"center": {"position": {"x": 1, "y": 2}, "theta": 0}, "size_x": 3, "size_y": 4},
     "id": ""},
    {"header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""},
     "results": [{"hypothesis": {"class_id": "person", "score": 0.40}, "pose": {"pose": {"position": {"x":0,"y":0,"z":0}, "orientation": {"x":0,"y":0,"z":0,"w":1}}, "covariance": []}}],
     "bbox": {"center": {"position": {"x": 1, "y": 2}, "theta": 0}, "size_x": 3, "size_y": 4},
     "id": ""},
    {"header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""}, "results": [],
     "bbox": {"center": {"position": {"x": 0, "y": 0}, "theta": 0}, "size_x": 0, "size_y": 0}, "id": ""}
  ]
}`

func TestAdaptDetections_ReducesToSummary(t *testing.T) {
	out, err := adaptDetections([]byte(detectionArrayJSON))
	require.NoError(t, err)

	var got map[string]any
	require.NoError(t, json.Unmarshal(out, &got))
	assert.ElementsMatch(t, []string{"count", "classes", "max_score", "stamp", "frame_id"}, keys(got),
		"only the reduced fields may reach the browser")
	assert.EqualValues(t, 4, got["count"])
	assert.Equal(t, []any{"chair", "person"}, got["classes"], "sorted, de-duplicated top-1 classes; 'dog' is not a top hypothesis")
	assert.InDelta(t, 0.81, got["max_score"], 1e-9)
	assert.Equal(t, "left_oa_camera", got["frame_id"])
	stamp := got["stamp"].(map[string]any)
	assert.EqualValues(t, 1700000000, stamp["sec"])
	assert.EqualValues(t, 5, stamp["nanosec"])
}

func TestAdaptDetections_Empty(t *testing.T) {
	out, err := adaptDetections([]byte(`{"header":{"stamp":{"sec":1,"nanosec":2},"frame_id":"x"},"detections":[]}`))
	require.NoError(t, err)
	assert.JSONEq(t, `{"count":0,"classes":[],"max_score":0,"stamp":{"sec":1,"nanosec":2},"frame_id":"x"}`, string(out))
}

func TestAdaptDetections_BadJSON(t *testing.T) {
	_, err := adaptDetections([]byte(`not json`))
	assert.Error(t, err)
}

func TestVisionTopicsRegistered(t *testing.T) {
	assert.Equal(t, topicDef{"/vision/obstacle_close", "std_msgs/msg/Bool"}, topicMap["visionObstacleClose"])
	assert.Equal(t, topicDef{"/ai/det/detections", "vision_msgs/msg/Detection2DArray"}, topicMap["detections"])
	assert.NotNil(t, foxgloveAdapters["detections"])
}

func keys(m map[string]any) []string {
	out := make([]string, 0, len(m))
	for k := range m {
		out = append(out, k)
	}
	return out
}
