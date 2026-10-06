package providers

import (
	"fmt"
	"sync"
	"testing"
	"time"
)

func TestRosoutIsAQueuedTopic(t *testing.T) {
	def, ok := topicMap["rosout"]
	if !ok || def.ROS2Topic != "/rosout" || def.MsgType != "rcl_interfaces/msg/Log" {
		t.Fatalf("topicMap[rosout] = %+v, %v", def, ok)
	}
	if queuedTopics["rosout"] <= 0 {
		t.Fatal("rosout must be delivered through a queue: log lines are events, not state")
	}
}

// A burst published faster than the callback runs must arrive complete and in
// order, unlike the coalescing (latest-wins) mailbox.
func TestQueuedRosSubscriberDeliversEveryMessageInOrder(t *testing.T) {
	var mu sync.Mutex
	var got []string
	release := make(chan struct{})
	done := make(chan struct{})
	const n = 50
	sub := newQueuedRosSubscriber("rosout", "test", 100, func(msg []byte) {
		<-release // hold the first delivery so the rest pile up
		mu.Lock()
		got = append(got, string(msg))
		if len(got) == n {
			close(done)
		}
		mu.Unlock()
	})
	defer sub.Close()

	for i := 0; i < n; i++ {
		sub.Publish([]byte(fmt.Sprintf("%d", i)))
	}
	close(release)

	select {
	case <-done:
	case <-time.After(2 * time.Second):
		mu.Lock()
		defer mu.Unlock()
		t.Fatalf("got %d of %d messages", len(got), n)
	}
	for i, m := range got {
		if m != fmt.Sprintf("%d", i) {
			t.Fatalf("message %d = %s: order not preserved", i, m)
		}
	}
}

func TestQueuedRosSubscriberDropsOldestPastCap(t *testing.T) {
	sub := &RosSubscriber{queueCap: 3, close: make(chan struct{}), wake: make(chan struct{}, 1)}
	for i := 0; i < 5; i++ {
		sub.Publish([]byte(fmt.Sprintf("%d", i)))
	}
	if len(sub.queue) != 3 || string(sub.queue[0]) != "2" || string(sub.queue[2]) != "4" {
		t.Fatalf("queue = %q, want [2 3 4]", sub.queue)
	}
}
