package api

import (
	"bufio"
	"io"
	"os"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
	"gopkg.in/yaml.v3"
)

func SetupRoutes(r *gin.RouterGroup, provider types.IFirmwareProvider) {
	group := r.Group("/setup")
	FlashBoard(group, provider)
}

// FlashBoard flash the mower board with the given config
//
// @Summary flash the mower board with the given config
// @Description flash the mower board with the given config
// @Tags setup
// @Accept  json
// @Produce  text/event-stream
// @Param settings body types.FirmwareConfig true "config"
// @Success 200 {object} OkResponse
// @Failure 500 {object} ErrorResponse
// @Router /setup/flashBoard [post]
func FlashBoard(r *gin.RouterGroup, provider types.IFirmwareProvider) gin.IRoutes {
	return r.POST("/flashBoard", func(c *gin.Context) {
		var config types.FirmwareConfig
		var err error
		err = c.BindJSON(&config)
		if err != nil {
			c.JSON(500, ErrorResponse{
				Error: err.Error(),
			})
			return
		}
		reader, writer := io.Pipe()
		rd := bufio.NewReader(reader)
		go func() {
			err = provider.FlashFirmware(writer, config)
			if err != nil {
				writer.CloseWithError(err)
			} else {
				writer.Close()
			}
		}()
		c.Stream(func(w io.Writer) bool {
			line, _, err2 := rd.ReadLine()
			if err2 != nil {
				if err2 == io.EOF {
					c.SSEvent("end", "end")
					return false
				}
				c.SSEvent("error", err2.Error())
				return false
			}
			c.SSEvent("message", string(line))
			return true
		})
	})
}

// RobotProfileResponse names the robot profile the GUI should use. Source is
// "settings" (mower_model set in mowgli_robot.yaml), "env" (the ROBOT_PROFILE
// fallback, see providers.EnvFallbacks) or "default" (the schema default).
type RobotProfileResponse struct {
	ID     string `json:"id"`
	Source string `json:"source"`
}

// robotProfileDBKey is only ever filled from the ROBOT_PROFILE env fallback; it
// lets a deployment pick its profile before mowgli_robot.yaml names a model.
const robotProfileDBKey = "robot.profile"

func RobotProfileRoutes(r *gin.RouterGroup, dbProvider types.IDBProvider) {
	GetRobotProfile(r, dbProvider)
}

// resolveRobotProfile applies yaml > ROBOT_PROFILE > schema default. Only a
// mower_model written in the yaml counts as "settings": GET /settings/yaml
// fills absent keys with schema defaults, which would always mask the env.
func resolveRobotProfile(dbProvider types.IDBProvider) RobotProfileResponse {
	if path, err := dbProvider.Get("system.mower.yamlConfigFile"); err == nil {
		if file, err := os.ReadFile(string(path)); err == nil {
			existing := map[string]any{}
			if yaml.Unmarshal(file, &existing) == nil {
				if model, ok := flattenROS2YAML(existing)["mower_model"].(string); ok && model != "" {
					return RobotProfileResponse{ID: model, Source: "settings"}
				}
			}
		}
	}
	if env, err := dbProvider.Get(robotProfileDBKey); err == nil && len(env) > 0 {
		return RobotProfileResponse{ID: string(env), Source: "env"}
	}
	model, _ := loadSchemaDefaults(dbProvider)["mower_model"].(string)
	return RobotProfileResponse{ID: model, Source: "default"}
}

// GetRobotProfile returns the active robot profile id
//
// @Summary returns the active robot profile id
// @Description the mower_model from mowgli_robot.yaml, else the ROBOT_PROFILE env fallback, else the schema default. Answers before any settings exist.
// @Tags setup
// @Produce json
// @Success 200 {object} RobotProfileResponse
// @Router /robot/profile [get]
func GetRobotProfile(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.GET("/robot/profile", func(c *gin.Context) {
		c.JSON(200, resolveRobotProfile(dbProvider))
	})
}
