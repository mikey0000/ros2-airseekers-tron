package foxglove

import "testing"

// subscribeTopic must not send a second subscribe op for a channel that
// already holds a live subscription id (it would double every message).
func TestSubscribeTopicSkipsAlreadySubscribedChannel(t *testing.T) {
	c := NewClient("ws://127.0.0.1:1")
	c.connected.Store(true) // writeJSON fails (no conn) but must not be reached
	c.channels["/t"] = &channelState{def: channelDef{ID: 7, Topic: "/t"}, subscriptionID: 42}
	before := c.subIDCounter.Load()
	c.subscribeTopic("/t")
	if c.subIDCounter.Load() != before {
		t.Fatal("allocated a new subscription id for an already-subscribed channel")
	}
	if got := c.channels["/t"].subscriptionID; got != 42 {
		t.Fatalf("subscriptionID = %d, want 42", got)
	}
	c.pendingMu.Lock()
	defer c.pendingMu.Unlock()
	if c.pendingTopics["/t"] {
		t.Fatal("already-subscribed topic marked pending")
	}
}
