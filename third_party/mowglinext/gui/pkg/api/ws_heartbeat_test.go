package api

import (
	"errors"
	"net"
	"net/http/httptest"
	"strings"
	"testing"
	"time"

	"github.com/gorilla/websocket"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/require"
	"github.com/vmihailenco/msgpack/v5"
)

func dialTestWS(t *testing.T, path string) *websocket.Conn {
	t.Helper()
	srv := httptest.NewServer(setupMowgliNextRouter(types.NewMockRosProvider()))
	t.Cleanup(srv.Close)
	url := "ws" + strings.TrimPrefix(srv.URL, "http") + path
	conn, _, err := websocket.DefaultDialer.Dial(url, nil)
	require.NoError(t, err)
	t.Cleanup(func() { _ = conn.Close() })
	return conn
}

// The joystick socket gets an "hb" text frame every second so the browser can
// tell a live link from a half-open one.
func TestPublisherRouteSendsHeartbeat(t *testing.T) {
	conn := dialTestWS(t, "/api/mowglinext/publish/joy")
	_ = conn.SetReadDeadline(time.Now().Add(3 * time.Second))
	typ, msg, err := conn.ReadMessage()
	require.NoError(t, err)
	require.Equal(t, websocket.TextMessage, typ)
	require.Equal(t, PublishHeartbeat, string(msg))
}

// The multiplex socket's heartbeat is a msgpack frame on the "__hb" topic.
func TestMultiplexRouteSendsHeartbeat(t *testing.T) {
	conn := dialTestWS(t, "/api/mowglinext/multiplex")
	_ = conn.SetReadDeadline(time.Now().Add(3 * time.Second))
	typ, msg, err := conn.ReadMessage()
	require.NoError(t, err)
	require.Equal(t, websocket.BinaryMessage, typ)
	var frame map[string]interface{}
	require.NoError(t, msgpack.Unmarshal(msg, &frame))
	require.Equal(t, HeartbeatTopic, frame["topic"])
}

// A peer that never answers pings (no reads => gorilla never sends pongs) is
// dropped after wsReadTimeout instead of holding the route forever.
func TestPublisherRouteDropsSilentPeer(t *testing.T) {
	if testing.Short() {
		t.Skip("waits for wsReadTimeout")
	}
	conn := dialTestWS(t, "/api/mowglinext/publish/joy")
	time.Sleep(wsReadTimeout + 1500*time.Millisecond)
	_ = conn.SetReadDeadline(time.Now().Add(2 * time.Second))
	var err error
	for err == nil {
		_, _, err = conn.ReadMessage() // drain queued heartbeats, then the close
	}
	// Any failure except our own read timeout means the server dropped us.
	var ne net.Error
	if errors.As(err, &ne) && ne.Timeout() {
		t.Fatalf("server kept the silent connection open: %v", err)
	}
}
