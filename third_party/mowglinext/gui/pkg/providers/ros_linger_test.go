package providers

import (
	"testing"
	"time"

	"github.com/mowglinext/mowglinext/pkg/foxglove"
)

func newTestRosProvider(linger time.Duration) *RosProvider {
	return &RosProvider{
		client:             foxglove.NewClient("ws://127.0.0.1:1"),
		subscribers:        make(map[string]map[string]*RosSubscriber),
		lastMessage:        make(map[string][]byte),
		foxgloveSubscribed: make(map[string]bool),
		pendingUnsub:       make(map[string]*time.Timer),
		unsubscribeLinger:  linger,
	}
}

func upstream(r *RosProvider, key string) (subscribed, pending bool) {
	r.mtx.Lock()
	defer r.mtx.Unlock()
	_, pending = r.pendingUnsub[key]
	return r.foxgloveSubscribed[key], pending
}

// Many downstream listeners on one key share a single upstream subscription.
func TestRosProviderDedupesUpstreamSubscription(t *testing.T) {
	r := newTestRosProvider(time.Hour)
	for _, id := range []string{"a", "b", "c", "d", "e", "f"} {
		_ = r.Subscribe("previewSummary", id, 0, func([]byte) {})
	}
	if sub, _ := upstream(r, "previewSummary"); !sub {
		t.Fatal("not subscribed upstream")
	}
	if n := r.client.SubscriberCount("/behavior_tree_node/preview_summary"); n != 1 {
		t.Fatalf("foxglove client entries = %d, want 1", n)
	}
	for _, id := range []string{"a", "b", "c", "d", "e"} {
		r.UnSubscribe("previewSummary", id)
	}
	if sub, pending := upstream(r, "previewSummary"); !sub || pending {
		t.Fatalf("subscribed=%v pending=%v with one listener left", sub, pending)
	}
}

// A listener that returns inside the linger window keeps the live upstream
// subscription (no unsubscribe/subscribe churn on the bridge).
func TestRosProviderLingerCancelsOnResubscribe(t *testing.T) {
	r := newTestRosProvider(50 * time.Millisecond)
	_ = r.Subscribe("activeAreaSettings", "ws1", 0, func([]byte) {})
	r.fanOut("activeAreaSettings", []byte(`{"data":"x"}`))
	r.UnSubscribe("activeAreaSettings", "ws1")
	if sub, pending := upstream(r, "activeAreaSettings"); !sub || !pending {
		t.Fatalf("after last leave: subscribed=%v pending=%v, want true/true", sub, pending)
	}
	got := make(chan []byte, 1)
	_ = r.Subscribe("activeAreaSettings", "ws2", 0, func(m []byte) { got <- m })
	select {
	case m := <-got:
		if string(m) != `{"data":"x"}` {
			t.Fatalf("replay = %s", m)
		}
	case <-time.After(time.Second):
		t.Fatal("cached message not replayed during linger")
	}
	time.Sleep(120 * time.Millisecond)
	if sub, pending := upstream(r, "activeAreaSettings"); !sub || pending {
		t.Fatalf("after resubscribe: subscribed=%v pending=%v, want true/false", sub, pending)
	}
	if n := r.client.SubscriberCount("/behavior_tree_node/active_area_settings"); n != 1 {
		t.Fatalf("foxglove client entries = %d, want 1", n)
	}
}

// With no listener returning, the upstream subscription is dropped after the
// linger and the cached message is discarded.
func TestRosProviderLingerExpires(t *testing.T) {
	r := newTestRosProvider(30 * time.Millisecond)
	_ = r.Subscribe("recordingTrajectory", "ws1", 0, func([]byte) {})
	r.fanOut("recordingTrajectory", []byte(`{}`))
	r.UnSubscribe("recordingTrajectory", "ws1")
	deadline := time.Now().Add(time.Second)
	for time.Now().Before(deadline) {
		if sub, pending := upstream(r, "recordingTrajectory"); !sub && !pending {
			r.mtx.Lock()
			_, cached := r.lastMessage["recordingTrajectory"]
			r.mtx.Unlock()
			if cached {
				t.Fatal("stale cache kept after upstream unsubscribe")
			}
			if n := r.client.SubscriberCount("/behavior_tree_node/recording_trajectory"); n != 0 {
				t.Fatalf("foxglove client entries = %d, want 0", n)
			}
			return
		}
		time.Sleep(5 * time.Millisecond)
	}
	t.Fatal("upstream subscription never dropped")
}

func TestRosProviderZeroLingerUnsubscribesImmediately(t *testing.T) {
	r := newTestRosProvider(0)
	_ = r.Subscribe("plan", "x", 0, func([]byte) {})
	r.UnSubscribe("plan", "x")
	if sub, pending := upstream(r, "plan"); sub || pending {
		t.Fatalf("subscribed=%v pending=%v, want false/false", sub, pending)
	}
}
