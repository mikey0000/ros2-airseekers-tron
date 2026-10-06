package api

import (
	"sync"
	"time"

	"github.com/gorilla/websocket"
)

// Browser WebSocket liveness (phone roaming at the edge of the Wi-Fi).
//
// A phone that loses the AP mid-session leaves a half-open TCP connection: the
// browser keeps reporting OPEN and the server's read blocks forever. Both ends
// therefore exchange traffic every wsHeartbeatInterval:
//   - the server sends a WebSocket ping (the browser answers with a pong
//     automatically) plus a small application heartbeat DATA frame, because
//     browsers do not expose control frames to JavaScript;
//   - the server drops a connection that sent nothing (no message, no pong) for
//     wsReadTimeout, releasing its subscriptions;
//   - the web client drops and reconnects a socket that received nothing for
//     its own liveness timeout (see web/src/hooks/reconnect.ts).
const (
	wsHeartbeatInterval = time.Second
	wsReadTimeout       = 5 * time.Second
)

// HeartbeatTopic is the multiplex pseudo-topic of the heartbeat frame. No
// client subscribes to it, so older clients drop it as an unknown topic.
const HeartbeatTopic = "__hb"

// PublishHeartbeat is the text frame sent on publish (joystick) sockets.
const PublishHeartbeat = "hb"

// armReadDeadline makes conn's reads fail after wsReadTimeout of silence; every
// pong extends it. Call extendReadDeadline after each received message.
func armReadDeadline(conn *websocket.Conn) {
	_ = conn.SetReadDeadline(time.Now().Add(wsReadTimeout))
	conn.SetPongHandler(func(string) error {
		return conn.SetReadDeadline(time.Now().Add(wsReadTimeout))
	})
}

func extendReadDeadline(conn *websocket.Conn) {
	_ = conn.SetReadDeadline(time.Now().Add(wsReadTimeout))
}

// startHeartbeat pings conn and writes the heartbeat data frame every
// wsHeartbeatInterval until the returned stop function is called or a write
// fails (which closes conn so the read loop unblocks). writeMu serialises with
// the route's other writers.
func startHeartbeat(conn *websocket.Conn, writeMu *sync.Mutex, msgType int, frame []byte) (stop func()) {
	done := make(chan struct{})
	go func() {
		t := time.NewTicker(wsHeartbeatInterval)
		defer t.Stop()
		for {
			select {
			case <-done:
				return
			case <-t.C:
			}
			writeMu.Lock()
			deadline := time.Now().Add(wsWriteTimeout)
			err := conn.WriteControl(websocket.PingMessage, nil, deadline)
			if err == nil {
				_ = conn.SetWriteDeadline(deadline)
				err = conn.WriteMessage(msgType, frame)
			}
			writeMu.Unlock()
			if err != nil {
				_ = conn.Close()
				return
			}
		}
	}()
	var once sync.Once
	return func() { once.Do(func() { close(done) }) }
}
