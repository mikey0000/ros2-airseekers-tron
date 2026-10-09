package api

import (
	"context"
	"sync"
	"sync/atomic"
	"testing"
	"time"
)

// setupProbeCacheTest installs a fake probe and a controllable clock, resets
// the cache, and restores everything on cleanup.
func setupProbeCacheTest(t *testing.T, delay time.Duration) (*atomic.Int32, *time.Time) {
	t.Helper()
	oldFn, oldNow := cameraProbeFn, cameraProbeNow
	reset := func() {
		cameraProbeCache.Lock()
		cameraProbeCache.entries = map[string]probeEntry{}
		cameraProbeCache.inflight = map[string]*probeCall{}
		cameraProbeCache.Unlock()
	}
	reset()
	var calls atomic.Int32
	var mu sync.Mutex
	now := time.Unix(1000, 0)
	cameraProbeNow = func() time.Time { mu.Lock(); defer mu.Unlock(); return now }
	cameraProbeFn = func(ctx context.Context, base string) (bool, map[string]bool) {
		calls.Add(1)
		time.Sleep(delay)
		return true, map[string]bool{"/" + base: true}
	}
	t.Cleanup(func() { cameraProbeFn, cameraProbeNow = oldFn, oldNow; reset() })
	return &calls, &now
}

func TestProbeCacheWithinTTL(t *testing.T) {
	calls, _ := setupProbeCacheTest(t, 0)
	for i := 0; i < 2; i++ {
		ok, topics := probeCameraServerCached(context.Background(), "http://a")
		if !ok || !topics["/http://a"] {
			t.Fatalf("unexpected result %v %v", ok, topics)
		}
	}
	if n := calls.Load(); n != 1 {
		t.Fatalf("probe calls = %d, want 1", n)
	}
}

func TestProbeCacheExpires(t *testing.T) {
	calls, now := setupProbeCacheTest(t, 0)
	probeCameraServerCached(context.Background(), "http://a")
	// cameraProbeNow reads *now under a private mutex; the probe goroutine has
	// finished by the time the call above returned, so a direct write is safe.
	*now = now.Add(cameraProbeTTL + time.Millisecond)
	probeCameraServerCached(context.Background(), "http://a")
	if n := calls.Load(); n != 2 {
		t.Fatalf("probe calls = %d, want 2", n)
	}
}

func TestProbeCacheConcurrentSingleflight(t *testing.T) {
	calls, _ := setupProbeCacheTest(t, 50*time.Millisecond)
	var wg sync.WaitGroup
	for i := 0; i < 10; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			if ok, _ := probeCameraServerCached(context.Background(), "http://a"); !ok {
				t.Error("expected ok")
			}
		}()
	}
	wg.Wait()
	if n := calls.Load(); n != 1 {
		t.Fatalf("probe calls = %d, want 1", n)
	}
}

func TestProbeCacheSeparateBases(t *testing.T) {
	calls, _ := setupProbeCacheTest(t, 0)
	_, a := probeCameraServerCached(context.Background(), "http://a")
	_, b := probeCameraServerCached(context.Background(), "http://b")
	probeCameraServerCached(context.Background(), "http://a")
	if n := calls.Load(); n != 2 {
		t.Fatalf("probe calls = %d, want 2", n)
	}
	if !a["/http://a"] || !b["/http://b"] {
		t.Fatalf("results mixed up: %v %v", a, b)
	}
}

func TestProbeCacheCancelledWaiter(t *testing.T) {
	calls, _ := setupProbeCacheTest(t, 100*time.Millisecond)
	// The shared probe outlives the cancelled waiter; let it finish before the
	// cleanup restores the overridden vars.
	t.Cleanup(func() {
		for calls.Load() == 0 {
			time.Sleep(time.Millisecond)
		}
		for {
			cameraProbeCache.Lock()
			n := len(cameraProbeCache.inflight)
			cameraProbeCache.Unlock()
			if n == 0 {
				return
			}
			time.Sleep(time.Millisecond)
		}
	})
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if ok, topics := probeCameraServerCached(ctx, "http://a"); ok || topics != nil {
		t.Fatalf("got %v %v, want false nil", ok, topics)
	}
}
