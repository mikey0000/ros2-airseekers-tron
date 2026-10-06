package api

import (
	"bufio"
	"bytes"
	"encoding/json"
	"fmt"
	"io"
	"mime"
	"mime/multipart"
	"net"
	"net/http"
	"net/http/httptest"
	"net/url"
	"os"
	"path/filepath"
	"strings"
	"sync"
	"testing"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

const testBoundary = "boundarydonotcross"

// fakeVideoServer mimics web_video_server: /stream emits an endless
// multipart/x-mixed-replace MJPEG stream (one part every `period`), /snapshot
// one JPEG, / an HTML index. It records the query of every request.
type fakeVideoServer struct {
	*httptest.Server
	period time.Duration

	mu         sync.Mutex
	queries    []url.Values
	rawQueries []string
	streamed   chan struct{} // closed once the first frame has been flushed
	once       sync.Once
}

func newFakeVideoServer(t *testing.T, period time.Duration) *fakeVideoServer {
	f := &fakeVideoServer{period: period, streamed: make(chan struct{})}
	mux := http.NewServeMux()
	mux.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		_, _ = io.WriteString(w, "<html>web_video_server</html>")
	})
	mux.HandleFunc("/snapshot", func(w http.ResponseWriter, r *http.Request) {
		f.record(r)
		w.Header().Set("Content-Type", "image/jpeg")
		_, _ = w.Write(fakeJPEG(0))
	})
	mux.HandleFunc("/stream", func(w http.ResponseWriter, r *http.Request) {
		f.record(r)
		w.Header().Set("Content-Type", "multipart/x-mixed-replace;boundary="+testBoundary)
		w.WriteHeader(http.StatusOK)
		fl := w.(http.Flusher)
		_, _ = io.WriteString(w, "--"+testBoundary+"\r\n")
		for i := 0; ; i++ {
			jpg := fakeJPEG(i)
			_, err := fmt.Fprintf(w, "Content-type: image/jpeg\r\nX-Timestamp: %d.000000000\r\nContent-Length: %d\r\n\r\n", i, len(jpg))
			if err == nil {
				_, err = w.Write(jpg)
			}
			if err == nil {
				_, err = io.WriteString(w, "\r\n--"+testBoundary+"\r\n")
			}
			if err != nil {
				return
			}
			fl.Flush()
			f.once.Do(func() { close(f.streamed) })
			select {
			case <-r.Context().Done():
				return
			case <-time.After(f.period):
			}
		}
	})
	f.Server = httptest.NewServer(mux)
	t.Cleanup(f.Close)
	return f
}

func (f *fakeVideoServer) record(r *http.Request) {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.queries = append(f.queries, r.URL.Query())
	f.rawQueries = append(f.rawQueries, r.URL.RawQuery)
}

func (f *fakeVideoServer) lastRawQuery() string {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.rawQueries[len(f.rawQueries)-1]
}

func (f *fakeVideoServer) lastQuery() url.Values {
	f.mu.Lock()
	defer f.mu.Unlock()
	if len(f.queries) == 0 {
		return nil
	}
	return f.queries[len(f.queries)-1]
}

// fakeJPEG is a tiny SOI..EOI payload with the frame index inside (and a CRLF
// plus a boundary-like line to prove parts are length/boundary-safe).
func fakeJPEG(i int) []byte {
	return []byte(fmt.Sprintf("\xff\xd8frame-%d\r\n--notaboundary\xff\xd9", i))
}

// cameraTestRouter builds a router whose camera config points at base via the
// installed-yaml settings key (the production path).
func cameraTestRouter(t *testing.T, base string) (*gin.Engine, *types.MockDBProvider) {
	t.Helper()
	gin.SetMode(gin.TestMode)
	t.Setenv(cameraEnvList, "")
	t.Setenv(cameraEnvBaseURL, "")
	dir := t.TempDir()
	yamlPath := filepath.Join(dir, "mowgli_robot.yaml")
	require.NoError(t, os.WriteFile(yamlPath,
		[]byte("mowgli:\n  ros__parameters:\n    camera_stream_base_url: \""+base+"/\"\n"), 0o644))
	db := types.NewMockDBProvider()
	require.NoError(t, db.Set("system.mower.yamlConfigFile", []byte(yamlPath)))
	r := gin.New()
	CameraRoutes(r.Group("/api"), db)
	return r, db
}

func TestCameras_ListDefault(t *testing.T) {
	fake := newFakeVideoServer(t, 10*time.Millisecond)
	router, _ := cameraTestRouter(t, fake.URL)

	w := httptest.NewRecorder()
	router.ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/api/cameras", nil))
	require.Equal(t, http.StatusOK, w.Code)

	var resp CamerasResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &resp))
	assert.Equal(t, "default", resp.Source)
	assert.True(t, resp.Available)
	assert.Empty(t, resp.Hint)
	assert.Equal(t, CameraStreamDefaults{Quality: 50, FPS: 5, MaxFPS: 15}, resp.Defaults)
	require.Len(t, resp.Cameras, 4)
	topics := []string{}
	for _, c := range resp.Cameras {
		topics = append(topics, c.Topic)
	}
	assert.Equal(t, []string{"/left_oa_camera/image_raw", "/right_oa_camera/image_raw",
		"/rear_camera/image_raw", "/ai/det/image_annotated"}, topics)
	assert.Equal(t, "/api/cameras/rear/stream", resp.Cameras[2].StreamURL)
	assert.Equal(t, "/api/cameras/rear/snapshot", resp.Cameras[2].SnapshotURL)
}

func TestCameras_ListOverrideEnvAndProfile(t *testing.T) {
	fake := newFakeVideoServer(t, 10*time.Millisecond)
	router, db := cameraTestRouter(t, fake.URL)

	list := func() CamerasResponse {
		w := httptest.NewRecorder()
		router.ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/api/cameras", nil))
		require.Equal(t, http.StatusOK, w.Code)
		var resp CamerasResponse
		require.NoError(t, json.Unmarshal(w.Body.Bytes(), &resp))
		return resp
	}

	// Robot profile hook.
	ProfileCameras = func(types.IDBProvider) []Camera {
		return []Camera{{ID: "front", Label: "Front", Topic: "/front/image_raw"}}
	}
	t.Cleanup(func() { ProfileCameras = nil })
	resp := list()
	assert.Equal(t, "profile", resp.Source)
	require.Len(t, resp.Cameras, 1)
	assert.Equal(t, "/front/image_raw", resp.Cameras[0].Topic)

	// Env override beats the profile.
	t.Setenv(cameraEnvList, `[{"id":"a","topic":"/a/image","annotatedTopic":"/a/ann"}]`)
	resp = list()
	assert.Equal(t, "config", resp.Source)
	require.Len(t, resp.Cameras, 1)
	assert.Equal(t, "a", resp.Cameras[0].Label, "label defaults to id")
	assert.Equal(t, "/a/ann", resp.Cameras[0].AnnotatedTopic)

	// DB key beats env.
	require.NoError(t, db.Set(cameraDBKey, []byte(`[{"id":"b","label":"B","topic":"/b/image"}]`)))
	resp = list()
	assert.Equal(t, "B", resp.Cameras[0].Label)

	// Malformed override falls through instead of breaking the page.
	require.NoError(t, db.Set(cameraDBKey, []byte(`[{"id":"has space","topic":"/x"}]`)))
	t.Setenv(cameraEnvList, `not json`)
	resp = list()
	assert.Equal(t, "profile", resp.Source)
}

func TestCameras_StreamProxiesMJPEGUnbuffered(t *testing.T) {
	fake := newFakeVideoServer(t, 10*time.Millisecond)
	router, _ := cameraTestRouter(t, fake.URL)
	srv := httptest.NewServer(router)
	defer srv.Close()

	resp, err := http.Get(srv.URL + "/api/cameras/rear/stream?fps=15")
	require.NoError(t, err)
	defer resp.Body.Close()
	require.Equal(t, http.StatusOK, resp.StatusCode)

	mt, params, err := mime.ParseMediaType(resp.Header.Get("Content-Type"))
	require.NoError(t, err)
	assert.Equal(t, "multipart/x-mixed-replace", mt)
	assert.Equal(t, testBoundary, params["boundary"])
	assert.Contains(t, resp.Header.Get("Cache-Control"), "no-cache")

	// The upstream never ends, so reading three whole frames proves the proxy
	// streams instead of buffering the response.
	mr := multipart.NewReader(resp.Body, params["boundary"])
	prev := -1
	for n := 0; n < 3; n++ {
		part, err := mr.NextPart()
		require.NoError(t, err)
		assert.Equal(t, "image/jpeg", part.Header.Get("Content-Type"))
		body, err := io.ReadAll(part)
		require.NoError(t, err)
		var idx int
		_, err = fmt.Sscanf(strings.TrimPrefix(string(body), "\xff\xd8"), "frame-%d", &idx)
		require.NoError(t, err, "frame payload must be intact: %q", body)
		assert.Equal(t, fakeJPEG(idx), body)
		assert.Greater(t, idx, prev)
		prev = idx
	}

	q := fake.lastQuery()
	assert.Equal(t, "/rear_camera/image_raw", q.Get("topic"))
	assert.Equal(t, "mjpeg", q.Get("type"))
	assert.Equal(t, "50", q.Get("quality"))
	assert.Equal(t, "640", q.Get("width"))
	assert.Equal(t, "360", q.Get("height"))
	assert.Equal(t, "sensor_data", q.Get("qos_profile"), "best-effort cameras need a best-effort subscription")
	// web_video_server does not percent-decode: slashes must go out raw.
	assert.True(t, strings.HasPrefix(fake.lastRawQuery(), "topic=/rear_camera/image_raw&"), fake.lastRawQuery())
}

func TestCameras_StreamQueryParams(t *testing.T) {
	fake := newFakeVideoServer(t, 5*time.Millisecond)
	router, _ := cameraTestRouter(t, fake.URL)
	t.Setenv(cameraEnvList, `[{"id":"cam","topic":"/cam/raw","annotatedTopic":"/cam/ann","qosProfile":"default"}]`)
	srv := httptest.NewServer(router)
	defer srv.Close()

	resp, err := http.Get(srv.URL + "/api/cameras/cam/stream?variant=annotated&quality=30&width=320&height=240")
	require.NoError(t, err)
	// Read one frame then hang up.
	_, err = bufio.NewReader(resp.Body).ReadString('\n')
	require.NoError(t, err)
	resp.Body.Close()

	q := fake.lastQuery()
	assert.Equal(t, "/cam/ann", q.Get("topic"))
	assert.Equal(t, "30", q.Get("quality"))
	assert.Equal(t, "320", q.Get("width"))
	assert.Equal(t, "240", q.Get("height"))
	assert.Equal(t, "default", q.Get("qos_profile"))

	for path, want := range map[string]int{
		"/api/cameras/nope/stream":                    http.StatusNotFound,
		"/api/cameras/cam/stream?variant=thermal":     http.StatusBadRequest,
		"/api/cameras/cam/stream?quality=0":           http.StatusBadRequest,
		"/api/cameras/cam/stream?fps=-1":              http.StatusBadRequest,
		"/api/cameras/cam/stream?width=320":           http.StatusBadRequest,
		"/api/cameras/cam/stream?width=9999&height=1": http.StatusBadRequest,
	} {
		w := httptest.NewRecorder()
		router.ServeHTTP(w, httptest.NewRequest(http.MethodGet, path, nil))
		assert.Equal(t, want, w.Code, path)
	}

	// A camera without an annotated topic rejects variant=annotated.
	t.Setenv(cameraEnvList, `[{"id":"cam","topic":"/cam/raw"}]`)
	w := httptest.NewRecorder()
	router.ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/api/cameras/cam/stream?variant=annotated", nil))
	assert.Equal(t, http.StatusBadRequest, w.Code)
}

func TestCameras_ServerDownIs503WithHint(t *testing.T) {
	// A port that is guaranteed closed.
	ln, err := net.Listen("tcp", "127.0.0.1:0")
	require.NoError(t, err)
	dead := "http://" + ln.Addr().String()
	require.NoError(t, ln.Close())

	router, _ := cameraTestRouter(t, dead)
	for _, path := range []string{"/api/cameras/rear/stream", "/api/cameras/rear/snapshot"} {
		w := httptest.NewRecorder()
		router.ServeHTTP(w, httptest.NewRequest(http.MethodGet, path, nil))
		assert.Equal(t, http.StatusServiceUnavailable, w.Code, path)
		var e CameraErrorResponse
		require.NoError(t, json.Unmarshal(w.Body.Bytes(), &e))
		assert.Contains(t, e.Hint, "web_video_server")
		assert.Contains(t, e.Hint, dead)
	}

	w := httptest.NewRecorder()
	router.ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/api/cameras", nil))
	require.Equal(t, http.StatusOK, w.Code)
	var resp CamerasResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &resp))
	assert.False(t, resp.Available)
	assert.Contains(t, resp.Hint, "camera_stream_base_url")
}

func TestCameras_Snapshot(t *testing.T) {
	fake := newFakeVideoServer(t, 10*time.Millisecond)
	router, _ := cameraTestRouter(t, fake.URL)

	w := httptest.NewRecorder()
	router.ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/api/cameras/left_oa/snapshot?quality=80", nil))
	require.Equal(t, http.StatusOK, w.Code)
	assert.Equal(t, "image/jpeg", w.Header().Get("Content-Type"))
	assert.Equal(t, fakeJPEG(0), w.Body.Bytes())
	q := fake.lastQuery()
	assert.Equal(t, "/left_oa_camera/image_raw", q.Get("topic"))
	assert.Equal(t, "80", q.Get("quality"))
	assert.Empty(t, q.Get("type"))
}

func TestCameras_UpstreamErrorIs502(t *testing.T) {
	up := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		http.Error(w, "no such topic", http.StatusNotFound)
	}))
	defer up.Close()
	router, _ := cameraTestRouter(t, up.URL)
	w := httptest.NewRecorder()
	router.ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/api/cameras/rear/snapshot", nil))
	assert.Equal(t, http.StatusBadGateway, w.Code)
	assert.Contains(t, w.Body.String(), "no such topic")
}

func TestCameras_BaseURLResolution(t *testing.T) {
	chdirToGuiRoot(t) // schema default lives in asserts/
	resetSchemaCache()
	t.Cleanup(resetSchemaCache) // do not leak the real schema into later tests
	t.Setenv(cameraEnvBaseURL, "")
	db := types.NewMockDBProvider()
	require.NoError(t, db.Set("system.mower.yamlConfigFile", []byte(filepath.Join(t.TempDir(), "missing.yaml"))))
	assert.Equal(t, "http://localhost:8080", resolveCameraBaseURL(db), "schema default")

	t.Setenv(cameraEnvBaseURL, "http://robot:9090/")
	assert.Equal(t, "http://robot:9090", resolveCameraBaseURL(db), "env beats schema default")
}

// recFlusher is an in-memory flushWriter counting flushes.
type recFlusher struct {
	bytes.Buffer
	flushes int
}

func (r *recFlusher) Flush() { r.flushes++ }

func TestRelayMJPEG_DecimatesToFPS(t *testing.T) {
	// 30 frames, 1/30 s apart on a fake clock, capped at 5 fps -> every 6th
	// frame is forwarded (0, 6, 12, 18, 24).
	var src bytes.Buffer
	src.WriteString("--" + testBoundary + "\r\n")
	for i := 0; i < 30; i++ {
		jpg := fakeJPEG(i)
		fmt.Fprintf(&src, "Content-type: image/jpeg\r\nContent-Length: %d\r\n\r\n", len(jpg))
		src.Write(jpg)
		src.WriteString("\r\n--" + testBoundary + "\r\n")
	}
	clock := time.Unix(0, 0)
	now := func() time.Time {
		t := clock
		clock = clock.Add(time.Second / 30)
		return t
	}
	var out recFlusher
	require.NoError(t, relayMJPEG(&out, &src, testBoundary, 5, now))

	// The relay never writes a closing delimiter (the stream is endless);
	// add one so the parser sees a clean end.
	out.WriteString("--" + testBoundary + "--\r\n")
	mr := multipart.NewReader(bytes.NewReader(out.Bytes()), testBoundary)
	var got []string
	for {
		p, err := mr.NextPart()
		if err == io.EOF {
			break
		}
		require.NoError(t, err)
		b, _ := io.ReadAll(p)
		got = append(got, string(b))
	}
	want := []string{}
	for _, i := range []int{0, 6, 12, 18, 24} {
		want = append(want, string(fakeJPEG(i)))
	}
	assert.Equal(t, want, got)
	assert.Equal(t, 5, out.flushes, "one flush per forwarded frame")
}
