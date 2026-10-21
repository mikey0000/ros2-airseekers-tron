package api

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"sync"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/foxglove"
	"github.com/mowglinext/mowglinext/pkg/msgs/geometry"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// fakeMapServer is an in-memory map_server_node: a live area list (with path
// channels), a "disk" copy, and per-service failure injection. With
// atomic=false it does not advertise replace_areas, like a map server built
// before ws/edge-layout2.
type fakeMapServer struct {
	*types.MockRosProvider
	mu        sync.Mutex
	atomic    bool
	refuse    string // non-empty: replace_areas answers success=false
	live      []mowgli.ReplaceMapArea
	disk      []mowgli.ReplaceMapArea
	calls     []string
	failAt    map[string]int // service → 1-based call number that fails
	callCount map[string]int
}

func newFakeMapServer(atomic bool, initial ...mowgli.ReplaceMapArea) *fakeMapServer {
	return &fakeMapServer{
		MockRosProvider: types.NewMockRosProvider(),
		atomic:          atomic,
		live:            append([]mowgli.ReplaceMapArea{}, initial...),
		disk:            append([]mowgli.ReplaceMapArea{}, initial...),
		failAt:          map[string]int{},
		callCount:       map[string]int{},
	}
}

var errInjected = errors.New("injected failure")

func (f *fakeMapServer) CallService(_ context.Context, service string, req any, res any, _ ...string) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.calls = append(f.calls, service)
	f.callCount[service]++
	if n, ok := f.failAt[service]; ok && f.callCount[service] == n {
		return errInjected
	}
	switch service {
	case mapServiceReplaceAreas:
		if !f.atomic {
			return fmt.Errorf("foxglove: CallService %s: %w", service, foxglove.ErrServiceNotAdvertised)
		}
		r := req.(*replaceAreasReq)
		out := res.(*replaceAreasRes)
		if f.refuse != "" {
			out.Message = f.refuse
			return nil
		}
		if len(r.IsNavigationArea) != len(r.Areas) || len(r.ChannelJSON) != len(r.Areas) || len(r.SettingsJSON) != len(r.Areas) {
			out.Message = "arrays not index-aligned"
			return nil
		}
		next := []mowgli.ReplaceMapArea{}
		for i, a := range r.Areas {
			if a.Obstacles == nil || a.ObstacleInfo == nil {
				return errors.New("msg is not a list type")
			}
			el := mowgli.ReplaceMapArea{Area: a, IsNavigationArea: r.IsNavigationArea[i]}
			if r.ChannelJSON[i] != "" {
				var ch mowgli.AreaChannel
				if err := json.Unmarshal([]byte(r.ChannelJSON[i]), &ch); err != nil {
					return err
				}
				el.Channel = &ch
			}
			next = append(next, el)
		}
		f.live, f.disk = next, append([]mowgli.ReplaceMapArea{}, next...)
		out.Success = true
	case mapServiceGetMowingArea:
		out := res.(*mowgli.GetMowingAreaRes)
		idx := int(req.(*mowgli.GetMowingAreaReq).Index)
		if idx < len(f.live) {
			out.Success = true
			out.Area = f.live[idx].Area
			out.Area.IsNavigationArea = f.live[idx].IsNavigationArea
		}
	case mowgli.GetAreaChannelService:
		out := res.(*mowgli.GetAreaChannelRes)
		idx := int(req.(*mowgli.GetAreaChannelReq).AreaIndex)
		if idx < len(f.live) && f.live[idx].Channel != nil {
			b, _ := json.Marshal(f.live[idx].Channel)
			out.Success, out.SettingsJSON = true, string(b)
		}
	case mowgli.SetAreaChannelService:
		r := req.(*mowgli.SetAreaChannelReq)
		var ch mowgli.AreaChannel
		_ = json.Unmarshal([]byte(r.SettingsJSON), &ch)
		f.live[r.AreaIndex].Channel = &ch
		res.(*mowgli.SetAreaChannelRes).Success = true
	case mapServiceClearMap:
		f.live = nil
		f.disk = nil // our map server persists the clear
		res.(*triggerRes).Success = true
	case mapServiceAddArea:
		add := req.(*mowgli.AddMowingAreaReq)
		if add.Area.Obstacles == nil || add.Area.ObstacleInfo == nil {
			return errors.New("msg is not a list type")
		}
		ok := len(add.Area.Area.Points) >= 3
		res.(*mowgli.AddMowingAreaRes).Success = ok
		if ok {
			f.live = append(f.live, mowgli.ReplaceMapArea{Area: add.Area, IsNavigationArea: add.IsNavigationArea})
		}
	case mapServiceSaveAreas:
		f.disk = append([]mowgli.ReplaceMapArea{}, f.live...)
		res.(*triggerRes).Success = true
	}
	return nil
}

func (f *fakeMapServer) names(list []mowgli.ReplaceMapArea) []string {
	out := []string{}
	for _, a := range list {
		out = append(out, a.Area.Name)
	}
	return out
}

func square(name string) mowgli.MapArea {
	return mowgli.MapArea{Name: name, Area: geometry.Polygon{Points: []geometry.Point32{{X: 0, Y: 0}, {X: 1, Y: 0}, {X: 1, Y: 1}}}}
}

func pathArea(name string) mowgli.ReplaceMapArea {
	return mowgli.ReplaceMapArea{Area: square(name), IsNavigationArea: true,
		Channel: &mowgli.AreaChannel{Points: [][2]float64{{0, 0.5}, {3, 0.5}}, WidthM: 0.7}}
}

func newMap(names ...string) *mowgli.ReplaceMapReq {
	req := &mowgli.ReplaceMapReq{}
	for _, n := range names {
		req.Areas = append(req.Areas, mowgli.ReplaceMapArea{Area: square(n)})
	}
	return req
}

func TestReplaceMap_AtomicServiceIsOneCallWithAlignedArrays(t *testing.T) {
	f := newFakeMapServer(true, mowgli.ReplaceMapArea{Area: square("old")})
	req := newMap("lawn")
	req.Areas = append(req.Areas, pathArea("Path 1"))

	require.NoError(t, replaceMapInternal(context.Background(), f, req))

	assert.Equal(t, []string{mapServiceReplaceAreas}, f.calls, "no clear_map/add_area when replace_areas exists")
	assert.Equal(t, []string{"lawn", "Path 1"}, f.names(f.disk))
	require.NotNil(t, f.live[1].Channel)
	assert.Equal(t, 0.7, f.live[1].Channel.WidthM)
	assert.True(t, f.live[1].IsNavigationArea)
	assert.Nil(t, f.live[0].Channel)
}

func TestReplaceMap_AtomicRefusalChangesNothing(t *testing.T) {
	f := newFakeMapServer(true, mowgli.ReplaceMapArea{Area: square("old")})
	f.refuse = "area 1 'lawn': self-intersecting"

	err := replaceMapInternal(context.Background(), f, newMap("lawn"))

	require.Error(t, err)
	assert.Contains(t, err.Error(), "self-intersecting")
	assert.Contains(t, err.Error(), "nothing was changed")
	assert.Equal(t, []string{mapServiceReplaceAreas}, f.calls, "a refusal must not fall back to clear/add")
	assert.Equal(t, []string{"old"}, f.names(f.disk))
}

func TestReplaceMap_AtomicTransportErrorDoesNotFallBack(t *testing.T) {
	f := newFakeMapServer(true, mowgli.ReplaceMapArea{Area: square("old")})
	f.failAt[mapServiceReplaceAreas] = 1

	require.Error(t, replaceMapInternal(context.Background(), f, newMap("lawn")))
	assert.Equal(t, []string{mapServiceReplaceAreas}, f.calls)
}

func TestReplaceMap_FallbackWhenReplaceAreasNotAdvertised(t *testing.T) {
	f := newFakeMapServer(false, mowgli.ReplaceMapArea{Area: square("old")})
	req := newMap("lawn")
	req.Areas = append(req.Areas, pathArea("Path 1"))

	require.NoError(t, replaceMapInternal(context.Background(), f, req))

	assert.Equal(t, []string{
		mapServiceReplaceAreas,
		mapServiceGetMowingArea, mapServiceGetMowingArea, // snapshot: 1 area + end marker
		mapServiceClearMap,
		mapServiceAddArea,
		mapServiceAddArea, mowgli.SetAreaChannelService,
		mapServiceSaveAreas,
	}, f.calls)
	assert.Equal(t, []string{"lawn", "Path 1"}, f.names(f.disk))
	require.NotNil(t, f.live[1].Channel)
}

func TestReplaceMap_FallbackSnapshotFailureChangesNothing(t *testing.T) {
	f := newFakeMapServer(false, mowgli.ReplaceMapArea{Area: square("old")})
	f.failAt[mapServiceGetMowingArea] = 1

	err := replaceMapInternal(context.Background(), f, newMap("lawn"))

	require.Error(t, err)
	assert.Contains(t, err.Error(), "nothing was changed")
	assert.NotContains(t, f.calls, mapServiceClearMap)
	assert.Equal(t, []string{"old"}, f.names(f.live))
}

func TestReplaceMap_FallbackRestoresPreviousMapWithPathsOnFailedAdd(t *testing.T) {
	f := newFakeMapServer(false, mowgli.ReplaceMapArea{Area: square("front")}, pathArea("drive"))
	f.failAt[mapServiceAddArea] = 2 // second area of the NEW map

	err := replaceMapInternal(context.Background(), f, newMap("a", "b", "c"))

	require.Error(t, err)
	assert.Contains(t, err.Error(), `adding area 2/3 "b"`)
	assert.Contains(t, err.Error(), "previous map (2 areas) was restored")
	assert.Equal(t, []string{"front", "drive"}, f.names(f.live))
	assert.Equal(t, []string{"front", "drive"}, f.names(f.disk))
	require.NotNil(t, f.live[1].Channel, "the path's centreline must survive the rollback")
	assert.Equal(t, 0.7, f.live[1].Channel.WidthM)
}

func TestReplaceMap_FallbackReportsPartialMapWhenRestoreFails(t *testing.T) {
	f := newFakeMapServer(false, mowgli.ReplaceMapArea{Area: square("front")})
	f.failAt[mapServiceAddArea] = 1
	f.failAt[mapServiceClearMap] = 2 // the restore's clear

	err := replaceMapInternal(context.Background(), f, newMap("a"))

	require.Error(t, err)
	assert.Contains(t, err.Error(), "PARTIAL map")
	assert.Contains(t, err.Error(), "Do not start mowing")
}

func TestReplaceMap_FallbackSaveFailureSaysNewMapNotOnDisk(t *testing.T) {
	f := newFakeMapServer(false, mowgli.ReplaceMapArea{Area: square("front")})
	f.failAt[mapServiceSaveAreas] = 1

	err := replaceMapInternal(context.Background(), f, newMap("a"))

	require.Error(t, err)
	assert.Contains(t, err.Error(), "NOT written to disk")
	assert.Equal(t, []string{"a"}, f.names(f.live))
}

func TestReplaceMap_ConcurrentSavesAreSerialized(t *testing.T) {
	f := newFakeMapServer(true)
	require.NoError(t, lockMapReplacement(context.Background(), nil)) // a save in progress

	waiting := make(chan struct{})
	done := make(chan error, 1)
	go func() {
		done <- replaceMapInternalWithLockWait(context.Background(), f, newMap("x"), func() { close(waiting) })
	}()
	<-waiting
	f.mu.Lock()
	assert.Empty(t, f.calls, "the second save must not touch map_server while the first holds the lock")
	f.mu.Unlock()
	unlockMapReplacement()
	require.NoError(t, <-done)
	assert.Equal(t, []string{"x"}, f.names(f.disk))
}

func TestReplaceMap_CancelledWhileWaitingForLock(t *testing.T) {
	f := newFakeMapServer(true)
	require.NoError(t, lockMapReplacement(context.Background(), nil))
	defer unlockMapReplacement()

	ctx, cancel := context.WithCancel(context.Background())
	err := replaceMapInternalWithLockWait(ctx, f, newMap("x"), cancel)

	require.ErrorIs(t, err, context.Canceled)
	assert.Empty(t, f.calls)
}
