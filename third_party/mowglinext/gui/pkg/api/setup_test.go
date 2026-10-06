package api

import (
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"testing"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func getRobotProfile(t *testing.T, db types.IDBProvider) RobotProfileResponse {
	t.Helper()
	gin.SetMode(gin.TestMode)
	r := gin.New()
	RobotProfileRoutes(r.Group("/api"), db)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("GET", "/api/robot/profile", nil)
	r.ServeHTTP(w, req)
	require.Equal(t, http.StatusOK, w.Code)

	var resp RobotProfileResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &resp))
	return resp
}

func TestGetRobotProfile_SchemaDefaultBeforeAnySettings(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)

	db := types.NewMockDBProvider()
	db.Set("system.mower.yamlConfigFile", []byte(t.TempDir()+"/missing.yaml"))

	assert.Equal(t, RobotProfileResponse{ID: "YardForce500", Source: "default"}, getRobotProfile(t, db))
}

func TestGetRobotProfile_EnvFallbackBeatsSchemaDefault(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)

	// The yaml exists but does not name a model: the env must still win,
	// even though GET /settings/yaml would report the schema default.
	yamlFile := createTempYAMLFile(t, "mowgli:\n  ros__parameters:\n    tool_width: 0.2\n")
	db := types.NewMockDBProvider()
	db.Set("system.mower.yamlConfigFile", []byte(yamlFile))
	db.Set(robotProfileDBKey, []byte("AirseekersTron"))

	assert.Equal(t, RobotProfileResponse{ID: "AirseekersTron", Source: "env"}, getRobotProfile(t, db))
}

func TestGetRobotProfile_ExplicitYAMLModelBeatsEnv(t *testing.T) {
	yamlFile := createTempYAMLFile(t, "mowgli:\n  ros__parameters:\n    mower_model: Sabo\n")
	db := types.NewMockDBProvider()
	db.Set("system.mower.yamlConfigFile", []byte(yamlFile))
	db.Set(robotProfileDBKey, []byte("AirseekersTron"))

	assert.Equal(t, RobotProfileResponse{ID: "Sabo", Source: "settings"}, getRobotProfile(t, db))
}

func TestGetRobotProfile_UnreadableYAMLFallsThrough(t *testing.T) {
	yamlFile := createTempYAMLFile(t, "mowgli: [not: a map\n")
	db := types.NewMockDBProvider()
	db.Set("system.mower.yamlConfigFile", []byte(yamlFile))
	db.Set(robotProfileDBKey, []byte("AirseekersTron"))

	assert.Equal(t, RobotProfileResponse{ID: "AirseekersTron", Source: "env"}, getRobotProfile(t, db))
}
