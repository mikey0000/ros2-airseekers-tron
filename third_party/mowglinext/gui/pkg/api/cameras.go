package api

// Camera registry and MJPEG proxy.
//
// The GUI never touches image topics itself: a robot that has cameras runs
// web_video_server (ROS 2 package, MJPEG over HTTP, default :8080) next to the
// stack, and this file reverse-proxies its /stream and /snapshot endpoints
// under /api/cameras so the browser only ever talks to :4006 (one origin, no
// CORS, no extra port to open). Frames are relayed part by part and flushed
// immediately — nothing is buffered beyond the current JPEG.
//
// Which cameras exist is configuration, not code. Resolution order
// (resolveCameraConfig):
//
//  1. DB key "system.cameras" or env CAMERAS — a JSON array of Camera
//     (operator override);
//  2. ProfileCameras — the active robot profile's `cameras` list (hook for the
//     robot-profile descriptor; nil until that lands);
//  3. defaultCameras — the built-in list.
//
// The web_video_server base URL is the settings key
// perception_settings.camera_stream_base_url (mowgli_robot.yaml), then env
// CAMERA_STREAM_BASE_URL, then the schema default.

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"math"
	"mime"
	"mime/multipart"
	"net"
	"net/http"
	"net/url"
	"os"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
	"gopkg.in/yaml.v3"
)

// Camera describes one image source the GUI can show.
type Camera struct {
	// ID is the URL-safe identifier used in /api/cameras/:id/...
	ID string `json:"id"`
	// Label is the human-readable name.
	Label string `json:"label"`
	// Topic is the raw sensor_msgs/Image topic.
	Topic string `json:"topic"`
	// AnnotatedTopic optionally names an overlay image (detections drawn in)
	// for the same camera; selected with ?variant=annotated.
	AnnotatedTopic string `json:"annotatedTopic,omitempty"`
	// Width/Height optionally set the default stream size (web_video_server
	// resizes before JPEG encoding, which is most of its CPU cost). Both or
	// neither: web_video_server does not preserve the aspect ratio when only
	// one is given.
	Width  int `json:"width,omitempty"`
	Height int `json:"height,omitempty"`
	// SourceWidth/SourceHeight optionally give the published image size the
	// detector's pixel boxes refer to (for the client-side box overlay on a
	// resized stream). 0 = assume the box coordinates match the stream size.
	SourceWidth  int `json:"sourceWidth,omitempty"`
	SourceHeight int `json:"sourceHeight,omitempty"`
	// QoSProfile is web_video_server's subscription QoS preset
	// (default | system_default | sensor_data | services_default). Empty =
	// sensor_data: a best-effort subscriber matches both best-effort camera
	// drivers and reliable publishers, whereas web_video_server's own default
	// (reliable) silently receives nothing from a best-effort camera.
	QoSProfile string `json:"qosProfile,omitempty"`
}

// CameraInfo is one entry of GET /api/cameras.
type CameraInfo struct {
	Camera
	StreamURL   string `json:"streamUrl"`
	SnapshotURL string `json:"snapshotUrl"`
	HealthURL   string `json:"healthUrl"`
	// Status says whether the camera's topics exist right now.
	Status CameraStatus `json:"status"`
}

// CameraStatus is the per-camera publisher state of GET /api/cameras.
//
// Listed comes from web_video_server's topic index (it lists image topics that
// exist in the ROS graph); Raw/Annotated health come from the frames this
// proxy relayed recently. State folds both into one word for the UI:
//
//	"publishing"      a frame was relayed within cameraStaleAfter
//	"stale"           frames were relayed before, but none recently
//	"listed"          the topic exists but nothing was streamed yet
//	"not_publishing"  the topic is not in the graph
//	"unknown"         web_video_server did not answer
type CameraStatus struct {
	State           string       `json:"state"`
	Listed          bool         `json:"listed"`
	AnnotatedListed bool         `json:"annotatedListed"`
	Raw             *TopicHealth `json:"raw,omitempty"`
	Annotated       *TopicHealth `json:"annotated,omitempty"`
}

// TopicHealth is the frame rate and last-frame age measured by the proxy on
// one topic (as delivered by web_video_server, before the per-viewer fps cap).
type TopicHealth struct {
	Topic string `json:"topic"`
	// FPS over the last cameraHealthWindow (0 = fewer than two frames).
	FPS float64 `json:"fps"`
	// LastFrameAgeMs is the age of the newest frame; nil = never streamed.
	LastFrameAgeMs *int64 `json:"lastFrameAgeMs"`
	// Viewers is the number of open proxied streams on this topic.
	Viewers int `json:"viewers"`
}

// CameraHealthResponse is the body of GET /api/cameras/:id/health.
type CameraHealthResponse struct {
	ID     string       `json:"id"`
	Status CameraStatus `json:"status"`
}

// CamerasResponse is the body of GET /api/cameras.
type CamerasResponse struct {
	Cameras []CameraInfo `json:"cameras"`
	// Source says where the list came from: "config" (DB/env override),
	// "profile" (robot profile) or "default".
	Source string `json:"source"`
	// Available is true when web_video_server answered a quick probe.
	Available bool `json:"available"`
	// Hint explains how to fix an unavailable stream server.
	Hint string `json:"hint,omitempty"`
	// Defaults are the stream parameters used when the client passes none.
	Defaults CameraStreamDefaults `json:"defaults"`
}

// CameraStreamDefaults are the per-request defaults and limits.
type CameraStreamDefaults struct {
	Quality int     `json:"quality"`
	FPS     float64 `json:"fps"`
	MaxFPS  float64 `json:"maxFps"`
}

// CameraErrorResponse is returned with 4xx/5xx from the camera routes.
type CameraErrorResponse struct {
	Error string `json:"error"`
	Hint  string `json:"hint,omitempty"`
}

const (
	cameraSettingsKeyBaseURL = "camera_stream_base_url"
	cameraDBKey              = "system.cameras"
	cameraEnvList            = "CAMERAS"
	cameraEnvBaseURL         = "CAMERA_STREAM_BASE_URL"
	// cameraFallbackBaseURL is only used when the schema cannot be read
	// (the schema default is the real source).
	cameraFallbackBaseURL = "http://localhost:8080"

	// Conservative defaults for small boards (4 GB RK3588S): JPEG quality 50
	// at 5 fps per viewer.
	cameraDefaultQuality = 50
	cameraDefaultFPS     = 5.0
	cameraMaxFPS         = 15.0
	cameraMaxDimension   = 1920
	cameraMaxFrameBytes  = 16 << 20
	cameraDefaultQoS     = "sensor_data"
)

// defaultCameras is the built-in list used when neither an operator override
// nor a robot profile provides one. It matches the Airseekers Tron camera
// stack (ros2_stack/launch/cameras.launch.py + det_ros); robots without these
// topics simply see black tiles, and should set CAMERAS / a profile instead.
//
// The OA cameras carry det_ros's per-camera overlay (/<ns>/image_annotated,
// boxes + labels drawn on the 960x540 input); the front stereo eyes come from
// mower_cameras/stereo_cam (cameras.launch.py stereo:=true, 640x480 bgr8). The rear
// webcam runs its 4:3 640x480 MJPG mode (cameras.launch.py rear_width/height).
var defaultCameras = []Camera{
	{ID: "left_oa", Label: "Left obstacle camera", Topic: "/left_oa_camera/image_raw",
		AnnotatedTopic: "/left_oa_camera/image_annotated", Width: 640, Height: 360, SourceWidth: 960, SourceHeight: 540},
	{ID: "right_oa", Label: "Right obstacle camera", Topic: "/right_oa_camera/image_raw",
		AnnotatedTopic: "/right_oa_camera/image_annotated", Width: 640, Height: 360, SourceWidth: 960, SourceHeight: 540},
	{ID: "rear", Label: "Rear camera", Topic: "/rear_camera/image_raw", Width: 640, Height: 480},
	{ID: "front_left", Label: "Front stereo left", Topic: "/vio/left/image_color"},
	{ID: "front_right", Label: "Front stereo right", Topic: "/vio/right/image_color"},
}

// ProfileCameras returns the active robot profile's camera list (nil/empty =
// the profile does not define one). It is the integration point for the
// robot-profile descriptor (`cameras?: {id,label,topic,annotatedTopic}[]`);
// left nil until profiles are served by the backend.
var ProfileCameras func(dbProvider types.IDBProvider) []Camera

// cameraConfig is the resolved registry for one request.
type cameraConfig struct {
	BaseURL string
	Cameras []Camera
	Source  string
}

func (c cameraConfig) find(id string) (Camera, bool) {
	for _, cam := range c.Cameras {
		if cam.ID == id {
			return cam, true
		}
	}
	return Camera{}, false
}

// parseCameraList decodes and validates a JSON camera list.
func parseCameraList(raw []byte) ([]Camera, error) {
	var cams []Camera
	if err := json.Unmarshal(raw, &cams); err != nil {
		return nil, err
	}
	seen := map[string]bool{}
	for i, cam := range cams {
		if cam.ID == "" || cam.Topic == "" {
			return nil, fmt.Errorf("camera %d: id and topic are required", i)
		}
		if strings.ContainsAny(cam.ID, "/?#% ") {
			return nil, fmt.Errorf("camera %q: id must be URL-safe", cam.ID)
		}
		if seen[cam.ID] {
			return nil, fmt.Errorf("camera %q: duplicate id", cam.ID)
		}
		seen[cam.ID] = true
		if cams[i].Label == "" {
			cams[i].Label = cam.ID
		}
	}
	return cams, nil
}

func resolveCameraList(dbProvider types.IDBProvider) ([]Camera, string) {
	var override []byte
	if v, err := dbProvider.Get(cameraDBKey); err == nil && len(strings.TrimSpace(string(v))) > 0 {
		override = v
	} else if env := strings.TrimSpace(os.Getenv(cameraEnvList)); env != "" {
		override = []byte(env)
	}
	if override != nil {
		if cams, err := parseCameraList(override); err == nil {
			return cams, "config"
		}
		// A malformed override must not take the page down: fall through.
	}
	if ProfileCameras != nil {
		if cams := ProfileCameras(dbProvider); len(cams) > 0 {
			return cams, "profile"
		}
	}
	return append([]Camera(nil), defaultCameras...), "default"
}

// resolveCameraBaseURL reads perception_settings.camera_stream_base_url from
// the installed mowgli_robot.yaml, then env, then the schema default.
func resolveCameraBaseURL(dbProvider types.IDBProvider) string {
	if path, err := dbProvider.Get("system.mower.yamlConfigFile"); err == nil {
		if data, err := os.ReadFile(string(path)); err == nil {
			var y map[string]any
			if yaml.Unmarshal(data, &y) == nil {
				if v := strings.TrimSpace(stringValue(flattenROS2YAML(y)[cameraSettingsKeyBaseURL], "")); v != "" {
					return strings.TrimRight(v, "/")
				}
			}
		}
	}
	if v := strings.TrimSpace(os.Getenv(cameraEnvBaseURL)); v != "" {
		return strings.TrimRight(v, "/")
	}
	if v := strings.TrimSpace(stringValue(loadSchemaDefaults(dbProvider)[cameraSettingsKeyBaseURL], "")); v != "" {
		return strings.TrimRight(v, "/")
	}
	return cameraFallbackBaseURL
}

func resolveCameraConfig(dbProvider types.IDBProvider) cameraConfig {
	cams, source := resolveCameraList(dbProvider)
	return cameraConfig{BaseURL: resolveCameraBaseURL(dbProvider), Cameras: cams, Source: source}
}

func cameraUnavailableHint(base string) string {
	return fmt.Sprintf("web_video_server is not reachable at %s. Start it on the robot "+
		"(e.g. `ros2 run web_video_server web_video_server`, or the camera launch with "+
		"web_video_server:=true) or set Perception > Camera stream base URL "+
		"(camera_stream_base_url).", base)
}

// HTTP clients. Streams have no overall timeout (they are long-lived and end
// when the browser disconnects); dialing and the response header are bounded
// so a dead server fails fast with 503.
var (
	cameraDialer       = &net.Dialer{Timeout: 2 * time.Second}
	cameraStreamClient = &http.Client{Transport: &http.Transport{
		DialContext:           cameraDialer.DialContext,
		ResponseHeaderTimeout: 5 * time.Second,
		DisableCompression:    true,
		// web_video_server closes every connection after one response;
		// pooling would only hand out dead sockets (EOF).
		DisableKeepAlives: true,
	}}
	cameraSnapshotClient = &http.Client{Timeout: 6 * time.Second, Transport: &http.Transport{
		DialContext:        cameraDialer.DialContext,
		DisableCompression: true,
		DisableKeepAlives:  true,
	}}
	cameraProbeTimeout = 700 * time.Millisecond
)

// probeCameraServer fetches web_video_server's index page. ok is false when
// the server did not answer; topics is the set of image topics it lists (nil
// when the page could not be parsed, e.g. a non-web_video_server answer).
func probeCameraServer(ctx context.Context, base string) (ok bool, topics map[string]bool) {
	ctx, cancel := context.WithTimeout(ctx, cameraProbeTimeout)
	defer cancel()
	req, err := http.NewRequestWithContext(ctx, http.MethodGet, base+"/", nil)
	if err != nil {
		return false, nil
	}
	resp, err := cameraSnapshotClient.Do(req)
	if err != nil {
		return false, nil
	}
	defer resp.Body.Close()
	body, _ := io.ReadAll(io.LimitReader(resp.Body, 1<<20))
	return true, parseVideoServerTopics(string(body))
}

// parseVideoServerTopics extracts the topic names of web_video_server's
// "Available ROS Topics" index (`<li>/topic<ul>...`). nil = not that page.
func parseVideoServerTopics(body string) map[string]bool {
	if !strings.Contains(body, "Available ROS Topics") {
		return nil
	}
	out := map[string]bool{}
	for _, chunk := range strings.Split(body, "<li>")[1:] {
		if i := strings.Index(chunk, "<ul>"); i > 0 && strings.HasPrefix(chunk, "/") {
			out[chunk[:i]] = true
		}
	}
	return out
}

// cameraStatus combines the topic index and the proxy's frame statistics.
func cameraStatus(cam Camera, reachable bool, topics map[string]bool, now time.Time) CameraStatus {
	st := CameraStatus{Raw: cameraStats.health(cam.Topic, now)}
	if cam.AnnotatedTopic != "" {
		st.Annotated = cameraStats.health(cam.AnnotatedTopic, now)
	}
	if topics != nil {
		st.Listed = topics[cam.Topic]
		st.AnnotatedListed = cam.AnnotatedTopic != "" && topics[cam.AnnotatedTopic]
	}
	fresh := func(h *TopicHealth) bool {
		return h != nil && h.LastFrameAgeMs != nil && *h.LastFrameAgeMs < cameraStaleAfter.Milliseconds()
	}
	seen := func(h *TopicHealth) bool { return h != nil && h.LastFrameAgeMs != nil }
	switch {
	case fresh(st.Raw) || fresh(st.Annotated):
		st.State = "publishing"
	case !reachable:
		st.State = "unknown"
	case topics != nil && !st.Listed:
		st.State = "not_publishing"
	case seen(st.Raw) || seen(st.Annotated):
		st.State = "stale"
	case topics == nil:
		st.State = "unknown"
	default:
		st.State = "listed"
	}
	return st
}

// CameraRoutes registers the camera registry and proxy routes.
func CameraRoutes(r *gin.RouterGroup, dbProvider types.IDBProvider) {
	group := r.Group("/cameras")
	GetCameras(group, dbProvider)
	GetCameraStream(group, dbProvider)
	GetCameraSnapshot(group, dbProvider)
	GetCameraHealth(group, dbProvider)
}

// GetCameraHealth reports the measured frame rate and last-frame age.
//
// @Summary camera stream health
// @Description fps and last-frame age measured by the MJPEG proxy (raw and annotated topic), plus whether web_video_server lists the topics
// @Tags cameras
// @Produce  json
// @Param id path string true "camera id"
// @Success 200 {object} CameraHealthResponse
// @Failure 404 {object} CameraErrorResponse
// @Router /cameras/{id}/health [get]
func GetCameraHealth(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.GET("/:id/health", func(c *gin.Context) {
		cfg := resolveCameraConfig(dbProvider)
		cam, ok := cfg.find(c.Param("id"))
		if !ok {
			c.JSON(http.StatusNotFound, CameraErrorResponse{Error: fmt.Sprintf("unknown camera %q", c.Param("id"))})
			return
		}
		reachable, topics := probeCameraServer(c.Request.Context(), cfg.BaseURL)
		setNoCacheHeaders(c)
		c.JSON(http.StatusOK, CameraHealthResponse{ID: cam.ID, Status: cameraStatus(cam, reachable, topics, time.Now())})
	})
}

// GetCameras lists the configured cameras.
//
// @Summary list cameras
// @Description returns the camera registry (from config, robot profile or built-in default) and whether the MJPEG server is reachable
// @Tags cameras
// @Produce  json
// @Success 200 {object} CamerasResponse
// @Router /cameras [get]
func GetCameras(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.GET("", func(c *gin.Context) {
		cfg := resolveCameraConfig(dbProvider)
		out := CamerasResponse{
			Cameras: make([]CameraInfo, 0, len(cfg.Cameras)),
			Source:  cfg.Source,
			Defaults: CameraStreamDefaults{
				Quality: cameraDefaultQuality, FPS: cameraDefaultFPS, MaxFPS: cameraMaxFPS,
			},
		}
		reachable, topics := probeCameraServer(c.Request.Context(), cfg.BaseURL)
		now := time.Now()
		for _, cam := range cfg.Cameras {
			esc := url.PathEscape(cam.ID)
			out.Cameras = append(out.Cameras, CameraInfo{
				Camera:      cam,
				StreamURL:   "/api/cameras/" + esc + "/stream",
				SnapshotURL: "/api/cameras/" + esc + "/snapshot",
				HealthURL:   "/api/cameras/" + esc + "/health",
				Status:      cameraStatus(cam, reachable, topics, now),
			})
		}
		out.Available = reachable
		if !out.Available {
			out.Hint = cameraUnavailableHint(cfg.BaseURL)
		}
		c.JSON(http.StatusOK, out)
	})
}

// cameraRequest is the parsed per-request stream/snapshot parameters.
type cameraRequest struct {
	cfg     cameraConfig
	cam     Camera
	topic   string
	quality int
	fps     float64
	width   int
	height  int
}

// parseCameraRequest resolves the camera and validates query parameters.
// On failure it has already written the error response.
func parseCameraRequest(c *gin.Context, dbProvider types.IDBProvider) (cameraRequest, bool) {
	cfg := resolveCameraConfig(dbProvider)
	cam, ok := cfg.find(c.Param("id"))
	if !ok {
		c.JSON(http.StatusNotFound, CameraErrorResponse{Error: fmt.Sprintf("unknown camera %q", c.Param("id"))})
		return cameraRequest{}, false
	}
	req := cameraRequest{cfg: cfg, cam: cam, topic: cam.Topic,
		quality: cameraDefaultQuality, fps: cameraDefaultFPS, width: cam.Width, height: cam.Height}

	switch c.DefaultQuery("variant", "raw") {
	case "raw":
	case "annotated":
		if cam.AnnotatedTopic == "" {
			c.JSON(http.StatusBadRequest, CameraErrorResponse{Error: fmt.Sprintf("camera %q has no annotated stream", cam.ID)})
			return cameraRequest{}, false
		}
		req.topic = cam.AnnotatedTopic
	default:
		c.JSON(http.StatusBadRequest, CameraErrorResponse{Error: "variant must be raw or annotated"})
		return cameraRequest{}, false
	}

	bad := func(msg string) (cameraRequest, bool) {
		c.JSON(http.StatusBadRequest, CameraErrorResponse{Error: msg})
		return cameraRequest{}, false
	}
	if v := c.Query("quality"); v != "" {
		q, err := strconv.Atoi(v)
		if err != nil || q < 1 || q > 100 {
			return bad("quality must be an integer in 1..100")
		}
		req.quality = q
	}
	if v := c.Query("fps"); v != "" {
		f, err := strconv.ParseFloat(v, 64)
		if err != nil || f <= 0 {
			return bad("fps must be a positive number")
		}
		req.fps = min(f, cameraMaxFPS)
	}
	w, h := c.Query("width"), c.Query("height")
	if (w == "") != (h == "") {
		return bad("width and height must be given together")
	}
	if w != "" {
		wi, err1 := strconv.Atoi(w)
		hi, err2 := strconv.Atoi(h)
		if err1 != nil || err2 != nil || wi < 0 || hi < 0 || wi > cameraMaxDimension || hi > cameraMaxDimension {
			return bad(fmt.Sprintf("width/height must be integers in 0..%d (0 = native size)", cameraMaxDimension))
		}
		req.width, req.height = wi, hi
	}
	return req, true
}

// upstreamURL builds the web_video_server URL for endpoint "stream" or
// "snapshot".
func (r cameraRequest) upstreamURL(endpoint string) string {
	qos := r.cam.QoSProfile
	if qos == "" {
		qos = cameraDefaultQoS
	}
	// web_video_server matches the topic query value literally (no
	// percent-decoding), so "/" must stay unescaped: "%2Frear_camera%2F..."
	// silently streams nothing.
	params := [][2]string{{"topic", r.topic}}
	if endpoint == "stream" {
		params = append(params, [2]string{"type", "mjpeg"})
	}
	params = append(params,
		[2]string{"quality", strconv.Itoa(r.quality)},
		[2]string{"qos_profile", qos})
	if r.width > 0 && r.height > 0 {
		params = append(params,
			[2]string{"width", strconv.Itoa(r.width)},
			[2]string{"height", strconv.Itoa(r.height)})
	}
	parts := make([]string, 0, len(params))
	for _, p := range params {
		parts = append(parts, p[0]+"="+strings.ReplaceAll(url.QueryEscape(p[1]), "%2F", "/"))
	}
	return r.cfg.BaseURL + "/" + endpoint + "?" + strings.Join(parts, "&")
}

// doCameraUpstream performs the upstream request and maps transport failures
// to 503 and non-200 answers to 502. On failure it has already written the
// error response.
func doCameraUpstream(c *gin.Context, client *http.Client, req cameraRequest, endpoint string) (*http.Response, bool) {
	upReq, err := http.NewRequestWithContext(c.Request.Context(), http.MethodGet, req.upstreamURL(endpoint), nil)
	if err != nil {
		c.JSON(http.StatusInternalServerError, CameraErrorResponse{Error: err.Error()})
		return nil, false
	}
	resp, err := client.Do(upReq)
	if err != nil {
		if errors.Is(c.Request.Context().Err(), context.Canceled) {
			return nil, false // browser went away; nothing to answer
		}
		c.JSON(http.StatusServiceUnavailable, CameraErrorResponse{
			Error: "camera stream server unavailable: " + err.Error(),
			Hint:  cameraUnavailableHint(req.cfg.BaseURL),
		})
		return nil, false
	}
	if resp.StatusCode != http.StatusOK {
		body, _ := io.ReadAll(io.LimitReader(resp.Body, 512))
		_ = resp.Body.Close()
		c.JSON(http.StatusBadGateway, CameraErrorResponse{
			Error: fmt.Sprintf("camera stream server answered %d: %s", resp.StatusCode, strings.TrimSpace(string(body))),
		})
		return nil, false
	}
	return resp, true
}

func setNoCacheHeaders(c *gin.Context) {
	c.Header("Cache-Control", "no-cache, no-store, must-revalidate")
	c.Header("Pragma", "no-cache")
	c.Header("X-Accel-Buffering", "no") // reverse proxies: do not buffer
}

// GetCameraStream proxies a live MJPEG stream.
//
// @Summary live MJPEG stream of a camera
// @Description multipart/x-mixed-replace MJPEG relayed from web_video_server. 503 (with a hint) when the stream server is down.
// @Tags cameras
// @Produce  multipart/x-mixed-replace
// @Param id path string true "camera id"
// @Param variant query string false "raw (default) or annotated"
// @Param quality query int false "JPEG quality 1..100 (default 50)"
// @Param fps query number false "max frames per second delivered (default 5, max 15)"
// @Param width query int false "resize width (with height; 0 = native)"
// @Param height query int false "resize height (with width; 0 = native)"
// @Success 200
// @Failure 400 {object} CameraErrorResponse
// @Failure 404 {object} CameraErrorResponse
// @Failure 502 {object} CameraErrorResponse
// @Failure 503 {object} CameraErrorResponse
// @Router /cameras/{id}/stream [get]
func GetCameraStream(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.GET("/:id/stream", func(c *gin.Context) {
		req, ok := parseCameraRequest(c, dbProvider)
		if !ok {
			return
		}
		resp, ok := doCameraUpstream(c, cameraStreamClient, req, "stream")
		if !ok {
			return
		}
		defer resp.Body.Close()

		ct := resp.Header.Get("Content-Type")
		c.Header("Content-Type", ct)
		setNoCacheHeaders(c)
		c.Status(http.StatusOK)
		c.Writer.WriteHeaderNow()
		c.Writer.Flush()

		boundary := ""
		if mt, params, err := mime.ParseMediaType(ct); err == nil && strings.HasPrefix(mt, "multipart/") {
			boundary = params["boundary"]
		}
		if boundary == "" {
			// Not multipart (unexpected): relay bytes verbatim.
			_ = copyFlushing(c.Writer, resp.Body)
			return
		}
		done := cameraStats.open(req.topic)
		defer done()
		_ = relayMJPEG(c.Writer, resp.Body, boundary, req.fps, time.Now, func(t time.Time) {
			cameraStats.frame(req.topic, t)
		})
	})
}

// GetCameraSnapshot proxies a single JPEG frame.
//
// @Summary single JPEG frame of a camera
// @Description one JPEG from web_video_server /snapshot. 503 (with a hint) when the stream server is down.
// @Tags cameras
// @Produce  image/jpeg
// @Param id path string true "camera id"
// @Param variant query string false "raw (default) or annotated"
// @Param quality query int false "JPEG quality 1..100 (default 50)"
// @Param width query int false "resize width (with height; 0 = native)"
// @Param height query int false "resize height (with width; 0 = native)"
// @Success 200
// @Failure 404 {object} CameraErrorResponse
// @Failure 502 {object} CameraErrorResponse
// @Failure 503 {object} CameraErrorResponse
// @Router /cameras/{id}/snapshot [get]
func GetCameraSnapshot(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.GET("/:id/snapshot", func(c *gin.Context) {
		req, ok := parseCameraRequest(c, dbProvider)
		if !ok {
			return
		}
		resp, ok := doCameraUpstream(c, cameraSnapshotClient, req, "snapshot")
		if !ok {
			return
		}
		defer resp.Body.Close()
		ct := resp.Header.Get("Content-Type")
		if ct == "" {
			ct = "image/jpeg"
		}
		// Snapshot-polling tiles feed the same health statistics as streams.
		cameraStats.frame(req.topic, time.Now())
		setNoCacheHeaders(c)
		c.DataFromReader(http.StatusOK, resp.ContentLength, ct, io.LimitReader(resp.Body, cameraMaxFrameBytes), nil)
	})
}

type flushWriter interface {
	io.Writer
	http.Flusher
}

// copyFlushing copies src to w, flushing after every read so nothing sits in
// a buffer.
func copyFlushing(w flushWriter, src io.Reader) error {
	buf := make([]byte, 32<<10)
	for {
		n, err := src.Read(buf)
		if n > 0 {
			if _, werr := w.Write(buf[:n]); werr != nil {
				return werr
			}
			w.Flush()
		}
		if err != nil {
			if err == io.EOF {
				return nil
			}
			return err
		}
	}
}

// relayMJPEG re-emits the parts of a multipart MJPEG stream, dropping frames
// that arrive sooner than 1/fps after the last forwarded one (fps <= 0 keeps
// every frame). Each forwarded frame is written and flushed whole. It returns
// when the upstream ends or a write fails (browser closed the tab).
//
// onFrame (optional) is called for every upstream frame, forwarded or not, so
// the health statistics measure the source rate rather than the viewer cap.
func relayMJPEG(w flushWriter, src io.Reader, boundary string, fps float64, now func() time.Time, onFrame func(time.Time)) error {
	var minGap time.Duration
	if fps > 0 {
		// 10% slack so a 15 fps source capped at 5 fps keeps every 3rd frame
		// instead of drifting to every 4th on jitter.
		minGap = time.Duration(float64(time.Second) / fps * 0.9)
	}
	mr := multipart.NewReader(src, boundary)
	var last time.Time
	for {
		part, err := mr.NextPart()
		if err != nil {
			if err == io.EOF {
				return nil
			}
			return err
		}
		frame, err := io.ReadAll(io.LimitReader(part, cameraMaxFrameBytes))
		_ = part.Close()
		if err != nil {
			return err
		}
		t := now()
		if onFrame != nil {
			onFrame(t)
		}
		if minGap > 0 && !last.IsZero() && t.Sub(last) < minGap {
			continue
		}
		last = t
		ct := part.Header.Get("Content-Type")
		if ct == "" {
			ct = "image/jpeg"
		}
		header := fmt.Sprintf("--%s\r\nContent-Type: %s\r\nContent-Length: %d\r\n", boundary, ct, len(frame))
		if ts := part.Header.Get("X-Timestamp"); ts != "" {
			header += "X-Timestamp: " + ts + "\r\n"
		}
		if _, err := io.WriteString(w, header+"\r\n"); err != nil {
			return err
		}
		if _, err := w.Write(frame); err != nil {
			return err
		}
		if _, err := io.WriteString(w, "\r\n"); err != nil {
			return err
		}
		w.Flush()
	}
}

// ---- proxy frame statistics ----

const (
	cameraHealthWindow = 3 * time.Second
	cameraStaleAfter   = 3 * time.Second
)

type topicFrames struct {
	times   []time.Time // within cameraHealthWindow of the newest
	last    time.Time
	viewers int
}

type cameraFrameStats struct {
	mu     sync.Mutex
	topics map[string]*topicFrames
}

var cameraStats = &cameraFrameStats{topics: map[string]*topicFrames{}}

func (s *cameraFrameStats) get(topic string) *topicFrames {
	tf := s.topics[topic]
	if tf == nil {
		tf = &topicFrames{}
		s.topics[topic] = tf
	}
	return tf
}

// open registers a viewer; the returned func unregisters it.
func (s *cameraFrameStats) open(topic string) func() {
	s.mu.Lock()
	s.get(topic).viewers++
	s.mu.Unlock()
	return func() {
		s.mu.Lock()
		s.get(topic).viewers--
		s.mu.Unlock()
	}
}

func (s *cameraFrameStats) frame(topic string, t time.Time) {
	s.mu.Lock()
	defer s.mu.Unlock()
	tf := s.get(topic)
	// Several viewers relay the same frames: count a frame once (frames of one
	// camera are >= ~30 ms apart).
	if !tf.last.IsZero() && t.Sub(tf.last) < 15*time.Millisecond && tf.viewers > 1 {
		return
	}
	tf.last = t
	tf.times = append(tf.times, t)
	cut := 0
	for cut < len(tf.times) && t.Sub(tf.times[cut]) > cameraHealthWindow {
		cut++
	}
	tf.times = tf.times[cut:]
}

// health returns the statistics of one topic (age nil = never streamed).
func (s *cameraFrameStats) health(topic string, now time.Time) *TopicHealth {
	s.mu.Lock()
	defer s.mu.Unlock()
	h := &TopicHealth{Topic: topic}
	tf := s.topics[topic]
	if tf == nil {
		return h
	}
	h.Viewers = tf.viewers
	if tf.last.IsZero() {
		return h
	}
	age := now.Sub(tf.last).Milliseconds()
	h.LastFrameAgeMs = &age
	n := 0
	var first time.Time
	for _, t := range tf.times {
		if now.Sub(t) <= cameraHealthWindow {
			if n == 0 {
				first = t
			}
			n++
		}
	}
	if n >= 2 {
		if span := tf.last.Sub(first).Seconds(); span > 0 {
			h.FPS = math.Round(float64(n-1)/span*10) / 10
		}
	}
	return h
}
