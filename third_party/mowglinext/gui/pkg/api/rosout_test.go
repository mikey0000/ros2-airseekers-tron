package api

import (
	"context"
	"encoding/base64"
	"encoding/json"
	"errors"
	"fmt"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"

	docker "github.com/docker/docker/api/types"
	dockerclient "github.com/docker/docker/client"
	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
	pkgtypes "github.com/mowglinext/mowglinext/pkg/types"
)

// rosLog builds the JSON the foxglove CDR decoder produces for an
// rcl_interfaces/msg/Log.
func rosLog(level int, node, msg string) []byte {
	return []byte(fmt.Sprintf(
		`{"stamp":{"sec":1747087353,"nanosec":123456789},"level":%d,"name":%q,"msg":%q,"file":"node.cpp","function":"tick","line":42}`,
		level, node, msg))
}

func decodeFrame(t *testing.T, frame []byte) RosoutRecord {
	t.Helper()
	raw, err := base64.StdEncoding.DecodeString(string(frame))
	if err != nil {
		t.Fatalf("frame is not base64: %v", err)
	}
	var rec RosoutRecord
	if err := json.Unmarshal(raw, &rec); err != nil {
		t.Fatalf("frame is not a RosoutRecord: %v", err)
	}
	return rec
}

func TestRosLogLevelName(t *testing.T) {
	cases := map[int]string{0: "UNSET", 10: "DEBUG", 20: "INFO", 25: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL", 60: "FATAL"}
	for level, want := range cases {
		if got := RosLogLevelName(level); got != want {
			t.Errorf("RosLogLevelName(%d) = %q, want %q", level, got, want)
		}
	}
}

func TestParseRosoutMessage(t *testing.T) {
	rec, err := ParseRosoutMessage(rosLog(30, "map_server_node", "dock pose unset"))
	if err != nil {
		t.Fatal(err)
	}
	if rec.StampMs != 1747087353123 {
		t.Errorf("StampMs = %d, want 1747087353123", rec.StampMs)
	}
	if rec.Level != "WARN" || rec.Node != "map_server_node" || rec.Msg != "dock pose unset" {
		t.Errorf("unexpected record %+v", rec)
	}
	if rec.File != "node.cpp" || rec.Function != "tick" || rec.Line != 42 {
		t.Errorf("source location lost: %+v", rec)
	}
	if _, err := ParseRosoutMessage([]byte("not json")); err == nil {
		t.Error("expected an error for undecodable input")
	}
}

func TestRosoutHubSubscribesLazilyAndKeepsHistory(t *testing.T) {
	ros := pkgtypes.NewMockRosProvider()
	hub := NewRosoutHub(ros)
	hub.historySize = 3

	// Nothing is subscribed before a client attaches.
	ros.Dispatch(rosoutTopic, rosLog(20, "early", "lost"))

	client, history, err := hub.attach()
	if err != nil {
		t.Fatal(err)
	}
	if len(history) != 0 {
		t.Fatalf("history before any message = %d, want 0", len(history))
	}
	for i := 0; i < 5; i++ {
		ros.Dispatch(rosoutTopic, rosLog(20, "n", fmt.Sprintf("line %d", i)))
	}
	// The live client sees every record in order with increasing seq.
	for i := 0; i < 5; i++ {
		rec := decodeFrame(t, <-client.ch)
		if rec.Msg != fmt.Sprintf("line %d", i) || rec.Seq != uint64(i+1) || rec.Boot != hub.boot {
			t.Fatalf("record %d = %+v", i, rec)
		}
	}

	// A second client gets the capped history first, oldest first.
	second, history, err := hub.attach()
	if err != nil {
		t.Fatal(err)
	}
	if len(history) != 3 {
		t.Fatalf("history = %d records, want 3", len(history))
	}
	if first := decodeFrame(t, history[0]); first.Msg != "line 2" {
		t.Errorf("oldest kept record = %q, want line 2", first.Msg)
	}

	// The upstream subscription is dropped with the last client only.
	hub.detach(client)
	ros.Dispatch(rosoutTopic, rosLog(40, "n", "still live"))
	if rec := decodeFrame(t, <-second.ch); rec.Msg != "still live" || rec.Level != "ERROR" {
		t.Fatalf("second client got %+v", rec)
	}
	hub.detach(second)
	ros.Dispatch(rosoutTopic, rosLog(20, "n", "after detach"))
	if hub.seq != 6 {
		t.Errorf("hub ingested after the last client left: seq = %d, want 6", hub.seq)
	}
}

func TestRosoutHubSlowClientDropsOldest(t *testing.T) {
	ros := pkgtypes.NewMockRosProvider()
	hub := NewRosoutHub(ros)
	hub.clientBuffer = 2
	client, _, err := hub.attach()
	if err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 4; i++ {
		ros.Dispatch(rosoutTopic, rosLog(20, "n", fmt.Sprintf("line %d", i)))
	}
	// Never blocked; the newest two survive.
	if got := decodeFrame(t, <-client.ch).Msg; got != "line 2" {
		t.Errorf("first pending = %q, want line 2", got)
	}
	if got := decodeFrame(t, <-client.ch).Msg; got != "line 3" {
		t.Errorf("second pending = %q, want line 3", got)
	}
}

func TestRosoutHubSubscribeError(t *testing.T) {
	ros := pkgtypes.NewMockRosProvider()
	ros.SubscribeErr = errors.New("bridge down")
	if _, _, err := NewRosoutHub(ros).attach(); err == nil {
		t.Fatal("expected the subscribe error to surface")
	}
}

func TestRosoutStreamRouteSendsHistoryThenLive(t *testing.T) {
	gin.SetMode(gin.TestMode)
	ros := pkgtypes.NewMockRosProvider()
	router := gin.New()
	RosoutRoutes(router.Group("/api"), ros)
	server := httptest.NewServer(router)
	defer server.Close()

	wsURL := "ws" + strings.TrimPrefix(server.URL, "http") + "/api/rosout/stream"
	conn, _, err := websocket.DefaultDialer.Dial(wsURL, nil)
	if err != nil {
		t.Fatal(err)
	}
	defer conn.Close()

	// The hub subscribes on attach; dispatch once it has.
	deadline := time.Now().Add(2 * time.Second)
	for {
		ros.Dispatch(rosoutTopic, rosLog(30, "behavior_tree_node", "hello"))
		_ = conn.SetReadDeadline(time.Now().Add(100 * time.Millisecond))
		_, frame, err := conn.ReadMessage()
		if err == nil {
			rec := decodeFrame(t, frame)
			if rec.Node != "behavior_tree_node" || rec.Level != "WARN" || rec.Msg != "hello" {
				t.Fatalf("unexpected record %+v", rec)
			}
			return
		}
		if time.Now().After(deadline) {
			t.Fatalf("no record received: %v", err)
		}
		// A read deadline poisons gorilla's conn; redial.
		conn.Close()
		conn, _, err = websocket.DefaultDialer.Dial(wsURL, nil)
		if err != nil {
			t.Fatal(err)
		}
	}
}

// listDocker is an IDockerProvider whose ContainerList is scripted.
type listDocker struct {
	pkgtypes.IDockerProvider
	containers []docker.Container
	err        error
}

func (d *listDocker) ContainerList(context.Context) ([]docker.Container, error) {
	return d.containers, d.err
}

func getContainers(t *testing.T, provider pkgtypes.IDockerProvider) (int, map[string]any) {
	t.Helper()
	gin.SetMode(gin.TestMode)
	router := gin.New()
	ContainersRoutes(router.Group("/api"), provider)
	w := httptest.NewRecorder()
	router.ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/api/containers/", nil))
	var body map[string]any
	if err := json.Unmarshal(w.Body.Bytes(), &body); err != nil {
		t.Fatalf("body %q: %v", w.Body.String(), err)
	}
	return w.Code, body
}

func TestContainerListWithoutDockerDaemonIsNotAnError(t *testing.T) {
	code, body := getContainers(t, &listDocker{err: dockerclient.ErrorConnectionFailed("unix:///var/run/docker.sock")})
	if code != http.StatusOK {
		t.Fatalf("status = %d, want 200 (body %v)", code, body)
	}
	if body["available"] != false {
		t.Errorf("available = %v, want false", body["available"])
	}
	if list, ok := body["containers"].([]any); !ok || len(list) != 0 {
		t.Errorf("containers = %v, want an empty list", body["containers"])
	}
}

func TestContainerListReportsAvailableAndRealErrors(t *testing.T) {
	code, body := getContainers(t, &listDocker{containers: []docker.Container{{ID: "abc", Names: []string{"/mowgli-ros2"}, State: "running"}}})
	if code != http.StatusOK || body["available"] != true {
		t.Fatalf("status %d body %v, want 200 and available=true", code, body)
	}

	code, body = getContainers(t, &listDocker{err: errors.New("permission denied")})
	if code != http.StatusInternalServerError || body["error"] != "permission denied" {
		t.Fatalf("status %d body %v, want 500 with the error", code, body)
	}
}
