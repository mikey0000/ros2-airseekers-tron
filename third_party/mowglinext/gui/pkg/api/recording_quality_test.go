package api

import (
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestRecordingQualityRoute(t *testing.T) {
	orig := recordingQualityDir
	recordingQualityDir = t.TempDir()
	defer func() { recordingQualityDir = orig }()

	// The mission node's format (mission_node._save_quality).
	write := func(name, body string) {
		require.NoError(t, os.WriteFile(filepath.Join(recordingQualityDir, name), []byte(body), 0o644))
	}
	write("Front_lawn.json", `{"name": "Front lawn", "kind": "area", "saved": 200.0, "require_fixed": true,
		"not_fixed": 1, "vertices": [{"x": 1.0, "y": 2.0, "fix_type": 3, "hdop": 0.6, "fixed": true},
		{"x": 3.0, "y": 4.0, "fix_type": 2, "hdop": 1.4, "fixed": false}]}`)
	write("old.json", `{"name": "old", "kind": "obstacle", "saved": 100.0, "vertices": [{"x": 0, "y": 0, "fix_type": null, "hdop": null, "fixed": false}]}`)
	write("broken.json", `{not json`)
	write("notes.txt", `ignored`)

	router := setupMowgliNextRouter(types.NewMockRosProvider())
	w := httptest.NewRecorder()
	router.ServeHTTP(w, httptest.NewRequest("GET", "/api/mowglinext/recording-quality", nil))
	require.Equal(t, http.StatusOK, w.Code)

	var res RecordingQualityResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &res))
	require.Len(t, res.Recordings, 2)
	assert.Equal(t, "Front lawn", res.Recordings[0].Name, "newest first")
	require.Len(t, res.Recordings[0].Vertices, 2)
	assert.Equal(t, 2, *res.Recordings[0].Vertices[1].FixType)
	assert.False(t, res.Recordings[0].Vertices[1].Fixed)
	assert.Nil(t, res.Recordings[1].Vertices[0].FixType)
}

func TestRecordingQualityMissingDirIsEmpty(t *testing.T) {
	assert.Empty(t, readRecordingQuality(filepath.Join(t.TempDir(), "absent")))
}
