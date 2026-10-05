// Package foxglove implements a WebSocket client for the Foxglove WebSocket
// protocol (https://github.com/foxglove/ws-protocol). It connects to a
// foxglove_bridge (ROS2) to subscribe to topics, call services, and publish
// messages using a binary CDR encoding for topic data and JSON for services.
package foxglove

import "encoding/json"

// ---------------------------------------------------------------------------
// Server-to-client binary opcodes
// ---------------------------------------------------------------------------

const (
	// serverBinMessageData is the opcode for topic subscription data.
	serverBinMessageData byte = 0x01
	// serverBinTime is the opcode for server time broadcasts.
	serverBinTime byte = 0x02
	// serverBinServiceCallResponse is the opcode for service call responses.
	serverBinServiceCallResponse byte = 0x03
)

// ---------------------------------------------------------------------------
// Client-to-server binary opcodes
// ---------------------------------------------------------------------------

const (
	// clientBinMessageData is the opcode for publishing topic data.
	clientBinMessageData byte = 0x01
	// clientBinServiceCallRequest is the opcode for service call requests.
	clientBinServiceCallRequest byte = 0x02
)

// ---------------------------------------------------------------------------
// Server-to-client JSON messages
// ---------------------------------------------------------------------------

// serverInfo is sent by the server immediately after connection.
type serverInfo struct {
	Op                  string   `json:"op"`
	Name                string   `json:"name"`
	Capabilities        []string `json:"capabilities"`
	SupportedEncodings  []string `json:"supportedEncodings"`
	SessionID           string   `json:"sessionId"`
}

// channelDef describes a single advertised channel (topic).
type channelDef struct {
	ID             uint32 `json:"id"`
	Topic          string `json:"topic"`
	Encoding       string `json:"encoding"`
	SchemaName     string `json:"schemaName"`
	Schema         string `json:"schema"`
	SchemaEncoding string `json:"schemaEncoding"`
}

// serverAdvertise is sent by the server to announce available channels.
type serverAdvertise struct {
	Op       string       `json:"op"`
	Channels []channelDef `json:"channels"`
}

// serverUnadvertise is sent when channels are removed.
type serverUnadvertise struct {
	Op         string   `json:"op"`
	ChannelIDs []uint32 `json:"channelIds"`
}

// serviceEndpointDef describes the request or response side of a service.
type serviceEndpointDef struct {
	Encoding       string `json:"encoding"`
	SchemaName     string `json:"schemaName"`
	SchemaEncoding string `json:"schemaEncoding"`
	Schema         string `json:"schema"`
}

// serviceDef describes an advertised service.
type serviceDef struct {
	ID       uint32             `json:"id"`
	Name     string             `json:"name"`
	Type     string             `json:"type"`
	Request  serviceEndpointDef `json:"request"`
	Response serviceEndpointDef `json:"response"`
}

// serverAdvertiseServices announces available services.
type serverAdvertiseServices struct {
	Op       string       `json:"op"`
	Services []serviceDef `json:"services"`
}

// serverUnadvertiseServices is sent when services are removed.
type serverUnadvertiseServices struct {
	Op         string   `json:"op"`
	ServiceIDs []uint32 `json:"serviceIds"`
}

// serverStatus carries status/error messages from the server.
type serverStatus struct {
	Op      string `json:"op"`
	Level   int    `json:"level"`
	Message string `json:"message"`
}

// ---------------------------------------------------------------------------
// Client-to-server JSON messages
// ---------------------------------------------------------------------------

// subscriptionDef pairs a client-chosen subscription ID with a server channel.
type subscriptionDef struct {
	ID        uint32 `json:"id"`
	ChannelID uint32 `json:"channelId"`
}

// clientSubscribe requests topic subscriptions.
type clientSubscribe struct {
	Op            string            `json:"op"`
	Subscriptions []subscriptionDef `json:"subscriptions"`
}

// clientUnsubscribe cancels topic subscriptions.
type clientUnsubscribe struct {
	Op              string   `json:"op"`
	SubscriptionIDs []uint32 `json:"subscriptionIds"`
}

// clientChannelDef describes a channel that the client wants to publish on.
type clientChannelDef struct {
	ID         uint32 `json:"id"`
	Topic      string `json:"topic"`
	Encoding   string `json:"encoding"`
	SchemaName string `json:"schemaName"`
}

// clientAdvertise declares client-publish channels.
type clientAdvertise struct {
	Op       string             `json:"op"`
	Channels []clientChannelDef `json:"channels"`
}

// clientUnadvertise removes client-publish channels.
type clientUnadvertise struct {
	Op         string   `json:"op"`
	ChannelIDs []uint32 `json:"channelIds"`
}

// ---------------------------------------------------------------------------
// Generic JSON envelope for dispatch
// ---------------------------------------------------------------------------

// jsonEnvelope extracts just the "op" field for routing.
type jsonEnvelope struct {
	Op string `json:"op"`
}

// rawJSON is a convenience alias.
type rawJSON = json.RawMessage
