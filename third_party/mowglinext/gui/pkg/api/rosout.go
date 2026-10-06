package api

// ROS log stream (/rosout) for the Logs page.
//
// Container logs need the Docker socket, which a robot that runs the stack
// natively (no Docker host) does not have. Every ROS 2 node also publishes its
// log lines on /rosout (rcl_interfaces/msg/Log), so this route streams those
// instead: one WebSocket per browser tab, each frame a base64-encoded JSON
// RosoutRecord (base64 like the container log stream, so the dedicated-socket
// path of useWS decodes both the same way).
//
// A single hub holds the upstream "rosout" subscription while at least one
// client is connected and keeps the last rosoutHistorySize records, which a new
// client receives first. Every record carries the hub's boot id and a
// monotonically increasing seq so a client that reconnects can skip the
// history it already has.

import (
	"encoding/base64"
	"encoding/json"
	"fmt"
	"log"
	"sync"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
	"github.com/mowglinext/mowglinext/pkg/types"
)

const (
	// rosoutHistorySize is how many recent records a newly connected client
	// receives before the live tail.
	rosoutHistorySize = 1000
	// rosoutClientBuffer is the per-client backlog. A client that falls this
	// far behind loses its oldest pending records instead of stalling the hub.
	rosoutClientBuffer = 2000
	// rosoutTopic is the logical topic key in providers.topicMap.
	rosoutTopic = "rosout"
)

// RosoutRecord is one /rosout log line as sent to the browser.
type RosoutRecord struct {
	// Boot identifies the hub instance (its start time, epoch ms); seq
	// restarts when it changes.
	Boot int64  `json:"boot"`
	Seq  uint64 `json:"seq"`
	// StampMs is the producer's timestamp (header stamp of the Log message),
	// epoch milliseconds.
	StampMs  int64  `json:"stamp_ms"`
	Level    string `json:"level"`
	Node     string `json:"node"`
	Msg      string `json:"msg"`
	File     string `json:"file,omitempty"`
	Function string `json:"function,omitempty"`
	Line     uint32 `json:"line,omitempty"`
}

// rawRosLog is rcl_interfaces/msg/Log as produced by the foxglove CDR decoder.
type rawRosLog struct {
	Stamp struct {
		Sec     int64 `json:"sec"`
		Nanosec int64 `json:"nanosec"`
	} `json:"stamp"`
	Level    int    `json:"level"`
	Name     string `json:"name"`
	Msg      string `json:"msg"`
	File     string `json:"file"`
	Function string `json:"function"`
	Line     uint32 `json:"line"`
}

// RosLogLevelName maps an rcl_interfaces/msg/Log level to its name. Values in
// between the named levels round down; anything below DEBUG is "UNSET".
func RosLogLevelName(level int) string {
	switch {
	case level >= 50:
		return "FATAL"
	case level >= 40:
		return "ERROR"
	case level >= 30:
		return "WARN"
	case level >= 20:
		return "INFO"
	case level >= 10:
		return "DEBUG"
	default:
		return "UNSET"
	}
}

// ParseRosoutMessage converts one decoded rcl_interfaces/msg/Log into the
// browser record, without the hub's boot/seq.
func ParseRosoutMessage(msg []byte) (RosoutRecord, error) {
	var raw rawRosLog
	if err := json.Unmarshal(msg, &raw); err != nil {
		return RosoutRecord{}, err
	}
	return RosoutRecord{
		StampMs:  raw.Stamp.Sec*1000 + raw.Stamp.Nanosec/1_000_000,
		Level:    RosLogLevelName(raw.Level),
		Node:     raw.Name,
		Msg:      raw.Msg,
		File:     raw.File,
		Function: raw.Function,
		Line:     raw.Line,
	}, nil
}

// rosoutClient is one connected browser.
type rosoutClient struct {
	ch chan []byte
}

// RosoutHub fans /rosout out to WebSocket clients and keeps recent history.
type RosoutHub struct {
	provider types.IRosProvider
	boot     int64

	mtx      sync.Mutex
	seq      uint64
	history  [][]byte // encoded frames, oldest first, at most historySize
	clients  map[*rosoutClient]struct{}
	upstream bool

	historySize  int
	clientBuffer int
}

// NewRosoutHub builds a hub on provider. It subscribes upstream lazily, when
// the first client connects.
func NewRosoutHub(provider types.IRosProvider) *RosoutHub {
	return &RosoutHub{
		provider:     provider,
		boot:         time.Now().UnixMilli(),
		clients:      make(map[*rosoutClient]struct{}),
		historySize:  rosoutHistorySize,
		clientBuffer: rosoutClientBuffer,
	}
}

// subscriberID is the hub's id on the provider's "rosout" key.
func (h *RosoutHub) subscriberID() string {
	return fmt.Sprintf("rosout-hub-%d", h.boot)
}

// attach registers a client and returns it with a snapshot of the history,
// which the caller must send before reading from client.ch.
func (h *RosoutHub) attach() (*rosoutClient, [][]byte, error) {
	h.mtx.Lock()
	defer h.mtx.Unlock()
	if !h.upstream {
		if err := h.provider.Subscribe(rosoutTopic, h.subscriberID(), 0, h.ingest); err != nil {
			return nil, nil, err
		}
		h.upstream = true
	}
	client := &rosoutClient{ch: make(chan []byte, h.clientBuffer)}
	h.clients[client] = struct{}{}
	snapshot := append([][]byte(nil), h.history...)
	return client, snapshot, nil
}

// detach removes a client; the upstream subscription is dropped (and the
// history kept) when it was the last one.
func (h *RosoutHub) detach(client *rosoutClient) {
	h.mtx.Lock()
	defer h.mtx.Unlock()
	delete(h.clients, client)
	if len(h.clients) == 0 && h.upstream {
		h.upstream = false
		// Safe under h.mtx: the provider never holds its own lock while
		// calling ingest (delivery runs on the subscriber's goroutine).
		h.provider.UnSubscribe(rosoutTopic, h.subscriberID())
	}
}

// ingest handles one upstream message (called from the provider's delivery
// goroutine, one message at a time).
func (h *RosoutHub) ingest(msg []byte) {
	record, err := ParseRosoutMessage(msg)
	if err != nil {
		log.Printf("rosout: dropping undecodable message: %v", err)
		return
	}

	h.mtx.Lock()
	defer h.mtx.Unlock()
	h.seq++
	record.Boot = h.boot
	record.Seq = h.seq
	encoded, err := json.Marshal(record)
	if err != nil {
		return
	}
	frame := []byte(base64.StdEncoding.EncodeToString(encoded))

	if len(h.history) >= h.historySize {
		h.history = append(h.history[:0], h.history[len(h.history)-h.historySize+1:]...)
	}
	h.history = append(h.history, frame)

	for client := range h.clients {
		select {
		case client.ch <- frame:
		default:
			// Slow client: drop its oldest pending frame to make room so the
			// tail stays live; it never blocks the hub.
			select {
			case <-client.ch:
			default:
			}
			select {
			case client.ch <- frame:
			default:
			}
		}
	}
}

// RosoutRoutes registers GET /rosout/stream.
//
// @Summary stream ROS log lines (/rosout)
// @Description WebSocket; each frame is a base64-encoded JSON RosoutRecord. Recent history is sent first.
// @Tags logs
// @Router /rosout/stream [get]
func RosoutRoutes(r *gin.RouterGroup, provider types.IRosProvider) {
	hub := NewRosoutHub(provider)
	r.GET("/rosout/stream", func(c *gin.Context) {
		conn, err := upgrader.Upgrade(c.Writer, c.Request, nil)
		if err != nil {
			return
		}
		defer func() { _ = conn.Close() }()
		serveRosoutClient(hub, conn)
	})
}

// serveRosoutClient streams history then live records to conn until the client
// goes away.
func serveRosoutClient(hub *RosoutHub, conn *websocket.Conn) {
	client, history, err := hub.attach()
	if err != nil {
		log.Printf("rosout: subscribe failed: %v", err)
		return
	}
	defer hub.detach(client)

	// The read loop only exists to notice the client closing the socket.
	gone := make(chan struct{})
	go func() {
		defer close(gone)
		for {
			if _, _, err := conn.ReadMessage(); err != nil {
				return
			}
		}
	}()

	for _, frame := range history {
		if err := conn.WriteMessage(websocket.TextMessage, frame); err != nil {
			return
		}
	}
	for {
		select {
		case <-gone:
			return
		case frame := <-client.ch:
			if err := conn.WriteMessage(websocket.TextMessage, frame); err != nil {
				return
			}
		}
	}
}
