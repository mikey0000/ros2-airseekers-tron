package api

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"log"
	"time"

	"github.com/mowglinext/mowglinext/pkg/foxglove"
	"github.com/mowglinext/mowglinext/pkg/msgs/geometry"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
)

// Map replace ("Save map" in the editor, and the OpenMower importer).
//
// Preferred path: mower_map's atomic ~/replace_areas (mower_interfaces/
// ReplaceAreas). map_server builds and validates the whole new set off to the
// side and swaps it in under its lock with one rebuild and one save, so no
// reader (mission, planner, boundary check) ever sees a half-built map, and a
// rejected set changes nothing.
//
// Fallback, for a map server that does not advertise replace_areas yet: the
// clear_map → add_area × N → save_areas sequence, hardened the way upstream
// MowgliNext hardened it (#655, #771):
//
//   - a process-wide lock serializes concurrent saves;
//   - the CURRENT map (areas plus path channels) is read back first. If
//     map_server cannot answer, nothing has been touched and the save fails
//     cleanly; if a later step fails, the snapshot is put back, so a failed
//     save does not leave the robot with half a garden;
//   - every call has its own deadline and the error says WHICH step failed.
//
// The fallback still has a mid-save window in which readers see a partial
// map; only replace_areas closes it. Area settings are keyed by area name on
// our map server and survive clear_map, so neither path re-sends them.

const (
	mapServiceAddArea       = "/map_server_node/add_area"
	mapServiceClearMap      = "/map_server_node/clear_map"
	mapServiceSaveAreas     = "/map_server_node/save_areas"
	mapServiceGetMowingArea = "/map_server_node/get_mowing_area"
	mapServiceReplaceAreas  = "/map_server_node/replace_areas"
	mapServiceReplaceType   = "mower_interfaces/srv/ReplaceAreas"

	// mapServiceCallTimeout bounds ONE map_server call (map_server has a
	// single executor thread; a keepout-mask rebuild can hold it a while).
	mapServiceCallTimeout = 60 * time.Second

	// maxSnapshotAreas stops the read-back loop if map_server never reports
	// the end of its area list.
	maxSnapshotAreas = 512
)

// mapReplacementLock serializes the GUI's whole replace transaction. A
// channel-backed lock lets a request stop waiting when its context expires.
var mapReplacementLock = make(chan struct{}, 1)

// triggerRes decodes a std_srvs/srv/Trigger response, including the message
// map_server uses to explain a refused save.
type triggerRes struct {
	Success bool   `json:"success"`
	Message string `json:"message"`
}

// replaceAreasReq / replaceAreasRes mirror mower_interfaces/srv/ReplaceAreas.
// The optional arrays are index-aligned with Areas; ” means "none" for a
// channel and "keep" for settings.
type replaceAreasReq struct {
	Areas            []mowgli.MapArea `json:"areas"`
	IsNavigationArea []bool           `json:"is_navigation_area"`
	ChannelJSON      []string         `json:"channel_json"`
	SettingsJSON     []string         `json:"settings_json"`
}

type replaceAreasRes struct {
	Success bool   `json:"success"`
	Message string `json:"message"`
}

// mapWriteBudget returns the context timeout for a whole replace: read-back +
// clear_map + add_area×N + save_areas. The per-call deadline bounds a stuck
// map_server; this is only the ceiling for the sequence (issue #341: big maps
// on a slow SBC). A 60 s base plus per-area headroom, capped at 6 min. Shared
// by ReplaceMapRoute and the OpenMower importer so the two never drift.
func mapWriteBudget(nAreas int) time.Duration {
	budget := 60*time.Second + time.Duration(nAreas)*5*time.Second
	if budget > 6*time.Minute {
		budget = 6 * time.Minute
	}
	return budget
}

// callMapService runs one map_server call under its own deadline.
func callMapService(ctx context.Context, provider types.IRosProvider, service string, req, res any, srvType string) error {
	callCtx, cancel := context.WithTimeout(ctx, mapServiceCallTimeout)
	defer cancel()
	err := provider.CallService(callCtx, service, req, res, srvType)
	if err != nil && errors.Is(err, context.DeadlineExceeded) {
		return fmt.Errorf("map_server did not answer within %s (is the ROS2 container busy or restarting?): %w", mapServiceCallTimeout, err)
	}
	return err
}

// normalizedArea returns the area as the bridge must receive it: repeated
// fields are empty slices, never nil (foxglove_bridge rejects null — "msg is
// not a list type").
func normalizedArea(area mowgli.MapArea) mowgli.MapArea {
	if area.Obstacles == nil {
		area.Obstacles = []geometry.Polygon{}
	}
	if area.ObstacleInfo == nil {
		area.ObstacleInfo = []mowgli.MapObstacleInfo{}
	}
	return area
}

// hasChannel reports whether a replace element carries usable path metadata.
func hasChannel(el mowgli.ReplaceMapArea) bool {
	return el.IsNavigationArea && el.Channel != nil && len(el.Channel.Points) >= 2
}

// replaceMapInternal is the ROS-side flow shared by the public PUT handler and
// the OpenMower importer; HTTP decoding and status codes are the caller's job.
//
// Failure contract (the returned error says which case applies):
//   - replace_areas refused or failed → nothing was changed;
//   - reading the current map failed  → nothing was changed;
//   - clear/add failed                → the previous map was put back, or, if
//     that failed too, map_server holds a partial map and the error says so;
//   - only save_areas failed          → the NEW map is live but not on disk.
func replaceMapInternal(ctx context.Context, provider types.IRosProvider, req *mowgli.ReplaceMapReq) error {
	return replaceMapInternalWithLockWait(ctx, provider, req, nil)
}

// replaceMapInternalWithLockWait exposes the point where a caller meets a held
// lock, so concurrency tests can coordinate without timing assumptions.
func replaceMapInternalWithLockWait(ctx context.Context, provider types.IRosProvider, req *mowgli.ReplaceMapReq, onWait func()) error {
	if req == nil {
		return errors.New("replaceMapInternal: nil request")
	}
	if err := lockMapReplacement(ctx, onWait); err != nil {
		return fmt.Errorf("replace map: waiting for another map replacement: %w", err)
	}
	defer unlockMapReplacement()

	done, err := replaceAreasAtomic(ctx, provider, req.Areas)
	if done {
		return err
	}
	return replaceMapSequence(ctx, provider, req.Areas)
}

// replaceAreasAtomic tries map_server's replace_areas. done=false means the
// service is not advertised and the caller should use the fallback sequence;
// any other outcome is final.
func replaceAreasAtomic(ctx context.Context, provider types.IRosProvider, areas []mowgli.ReplaceMapArea) (bool, error) {
	req := replaceAreasReq{
		Areas:            make([]mowgli.MapArea, len(areas)),
		IsNavigationArea: make([]bool, len(areas)),
		ChannelJSON:      make([]string, len(areas)),
		SettingsJSON:     make([]string, len(areas)),
	}
	for i, el := range areas {
		req.Areas[i] = normalizedArea(el.Area)
		req.Areas[i].IsNavigationArea = req.Areas[i].IsNavigationArea || el.IsNavigationArea
		req.IsNavigationArea[i] = el.IsNavigationArea
		if hasChannel(el) {
			payload, err := json.Marshal(el.Channel)
			if err != nil {
				return true, fmt.Errorf("replace map: encoding the path of area %d: %w", i+1, err)
			}
			req.ChannelJSON[i] = string(payload)
		}
	}
	var res replaceAreasRes
	err := callMapService(ctx, provider, mapServiceReplaceAreas, &req, &res, mapServiceReplaceType)
	if err != nil && errors.Is(err, foxglove.ErrServiceNotAdvertised) {
		return false, nil
	}
	if err != nil {
		return true, fmt.Errorf("replace map: replace_areas failed (the map server keeps the old map unless it finished the swap; reload the map to check): %w", err)
	}
	if !res.Success {
		return true, fmt.Errorf("replace map: map_server refused the new map, nothing was changed: %s", res.Message)
	}
	return true, nil
}

// replaceMapSequence is the hardened clear/add/save fallback.
func replaceMapSequence(ctx context.Context, provider types.IRosProvider, areas []mowgli.ReplaceMapArea) error {
	previous, err := snapshotMap(ctx, provider)
	if err != nil {
		return fmt.Errorf("replace map: could not read the current map, so nothing was changed: %w", err)
	}
	if err := writeAreas(ctx, provider, areas); err != nil {
		return fmt.Errorf("replace map failed while %w — %s", err, restorePreviousMap(ctx, provider, previous))
	}
	if err := persistAreas(ctx, provider); err != nil {
		return fmt.Errorf("replace map: the new map is live but was NOT written to disk and will be lost at the next restart — check free space on the robot, then save again: %w", err)
	}
	return nil
}

// snapshotMap reads the live area list (and the path channel of every
// navigation area) back from map_server.
func snapshotMap(ctx context.Context, provider types.IRosProvider) ([]mowgli.ReplaceMapArea, error) {
	snapshot := []mowgli.ReplaceMapArea{}
	for i := uint32(0); i < maxSnapshotAreas; i++ {
		req := mowgli.GetMowingAreaReq{Index: i}
		var res mowgli.GetMowingAreaRes
		if err := callMapService(ctx, provider, mapServiceGetMowingArea, &req, &res, "mowgli_interfaces/srv/GetMowingArea"); err != nil {
			return nil, err
		}
		if !res.Success {
			return snapshot, nil // past the last area
		}
		el := mowgli.ReplaceMapArea{Area: normalizedArea(res.Area), IsNavigationArea: res.Area.IsNavigationArea}
		if el.IsNavigationArea && i < 255 {
			el.Channel = readAreaChannel(ctx, provider, uint8(i))
		}
		snapshot = append(snapshot, el)
	}
	return nil, fmt.Errorf("map_server reported more than %d areas", maxSnapshotAreas)
}

// readAreaChannel is best effort: a path without its channel still restores
// as a plain navigation polygon.
func readAreaChannel(ctx context.Context, provider types.IRosProvider, index uint8) *mowgli.AreaChannel {
	var res mowgli.GetAreaChannelRes
	req := mowgli.GetAreaChannelReq{AreaIndex: index}
	if err := callMapService(ctx, provider, mowgli.GetAreaChannelService, &req, &res, mowgli.GetAreaChannelType); err != nil || !res.Success || res.SettingsJSON == "" {
		return nil
	}
	var ch mowgli.AreaChannel
	if json.Unmarshal([]byte(res.SettingsJSON), &ch) != nil || len(ch.Points) < 2 {
		return nil
	}
	return &ch
}

// writeAreas is clear_map → add_area × N (+ set_area_channel for paths). It
// returns the step that failed.
func writeAreas(ctx context.Context, provider types.IRosProvider, areas []mowgli.ReplaceMapArea) error {
	var cleared triggerRes
	if err := callMapService(ctx, provider, mapServiceClearMap, &mowgli.ClearMapReq{}, &cleared, "std_srvs/srv/Trigger"); err != nil {
		return fmt.Errorf("clearing the old map: %w", err)
	}
	for i, element := range areas {
		req := mowgli.AddMowingAreaReq{
			Area:             normalizedArea(element.Area),
			IsNavigationArea: element.IsNavigationArea,
		}
		var res mowgli.AddMowingAreaRes
		step := fmt.Sprintf("adding area %d/%d %q", i+1, len(areas), element.Area.Name)
		if err := callMapService(ctx, provider, mapServiceAddArea, &req, &res, "mowgli_interfaces/srv/AddMowingArea"); err != nil {
			return fmt.Errorf("%s: %w", step, err)
		}
		if !res.Success {
			return fmt.Errorf("%s: map_server rejected it (a polygon needs at least 3 points)", step)
		}
		if hasChannel(element) && i < 255 {
			setAreaChannel(ctx, provider, uint8(i), element.Channel)
		}
	}
	return nil
}

// setAreaChannel attaches path metadata to the just-added area. Best effort:
// a map server without the service still gets the band polygon (the path
// just cannot be re-edited as a line), so a failure is only logged.
func setAreaChannel(ctx context.Context, provider types.IRosProvider, index uint8, ch *mowgli.AreaChannel) {
	payload, err := json.Marshal(ch)
	if err != nil {
		return
	}
	var res mowgli.SetAreaChannelRes
	req := mowgli.SetAreaChannelReq{AreaIndex: index, SettingsJSON: string(payload)}
	if err := callMapService(ctx, provider, mowgli.SetAreaChannelService, &req, &res, mowgli.SetAreaChannelType); err != nil {
		log.Printf("set_area_channel %d failed (path saved as a plain navigation area): %v", index, err)
		return
	}
	if !res.Success {
		log.Printf("set_area_channel %d rejected: %s", index, res.Message)
	}
}

// persistAreas is the explicit save_areas commit.
func persistAreas(ctx context.Context, provider types.IRosProvider) error {
	var saved triggerRes
	if err := callMapService(ctx, provider, mapServiceSaveAreas, &mowgli.ClearMapReq{}, &saved, "std_srvs/srv/Trigger"); err != nil {
		return err
	}
	if !saved.Success {
		return fmt.Errorf("map_server refused: %s", saved.Message)
	}
	return nil
}

func lockMapReplacement(ctx context.Context, onWait func()) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	select {
	case mapReplacementLock <- struct{}{}:
	default:
		if onWait != nil {
			onWait()
		}
		select {
		case mapReplacementLock <- struct{}{}:
		case <-ctx.Done():
			return ctx.Err()
		}
	}
	// The context may have expired as the lock became free. Do not start a
	// transaction after its caller has left.
	if err := ctx.Err(); err != nil {
		<-mapReplacementLock
		return err
	}
	return nil
}

func unlockMapReplacement() {
	<-mapReplacementLock
}

// restorePreviousMap puts the snapshot back after a failed replace and
// describes the outcome for the operator. It runs on a fresh deadline: the
// usual reason for getting here is that the original one expired.
func restorePreviousMap(ctx context.Context, provider types.IRosProvider, previous []mowgli.ReplaceMapArea) string {
	restoreCtx, cancel := context.WithTimeout(context.WithoutCancel(ctx), mapWriteBudget(len(previous)))
	defer cancel()

	err := writeAreas(restoreCtx, provider, previous)
	if err == nil {
		err = persistAreas(restoreCtx, provider)
	}
	if err != nil {
		return fmt.Sprintf("restoring the previous map ALSO failed (%v): the robot now holds a PARTIAL map. Do not start mowing; keep this editor open and save again once ROS2 responds", err)
	}
	return fmt.Sprintf("the previous map (%d areas) was restored; your edits are still in the editor, save again", len(previous))
}
