package api

import (
	"context"
	"encoding/json"
	"fmt"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/foxglove"
	"github.com/mowglinext/mowglinext/pkg/msgs/geometry"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// noReplaceAreas is a map server built before replace_areas existed: the
// GUI must fall back to clear_map + add_area and still attach path channels.
type noReplaceAreas struct{ *types.MockRosProvider }

func (n noReplaceAreas) CallService(ctx context.Context, service string, req any, res any, t ...string) error {
	if service == mapServiceReplaceAreas {
		return fmt.Errorf("foxglove: CallService %s: %w", service, foxglove.ErrServiceNotAdvertised)
	}
	return n.MockRosProvider.CallService(ctx, service, req, res, t...)
}

func TestReplaceMap_SendsPathChannelAfterItsAddArea(t *testing.T) {
	inner := types.NewMockRosProvider()
	inner.ServiceResponder = func(_ string, _ any, res any) {
		switch r := res.(type) {
		case *mowgli.SetAreaChannelRes:
			r.Success = true
		case *mowgli.AddMowingAreaRes:
			r.Success = true
		case *triggerRes:
			r.Success = true
		}
	}
	mock := noReplaceAreas{inner}
	poly := mowgli.MapArea{Name: "x", Area: geometry.Polygon{Points: []geometry.Point32{{X: 0, Y: 0}, {X: 1, Y: 0}, {X: 1, Y: 1}}}}
	path := poly
	path.Name = "Path 1"
	req := &mowgli.ReplaceMapReq{Areas: []mowgli.ReplaceMapArea{
		{Area: poly},
		{Area: path, IsNavigationArea: true, Channel: &mowgli.AreaChannel{Points: [][2]float64{{0, 0.5}, {3, 0.5}}, WidthM: 0.7}},
		{Area: poly, IsNavigationArea: true}, // plain navigation polygon: no channel call
	}}
	require.NoError(t, replaceMapInternal(context.Background(), mock, req))

	var services []string
	for _, c := range inner.ServiceCalls {
		services = append(services, c.Service)
	}
	assert.Equal(t, []string{
		"/map_server_node/get_mowing_area", // snapshot (empty map)
		"/map_server_node/clear_map",
		"/map_server_node/add_area",
		"/map_server_node/add_area",
		mowgli.SetAreaChannelService,
		"/map_server_node/add_area",
		"/map_server_node/save_areas",
	}, services)

	chReq, ok := inner.ServiceCalls[4].Req.(*mowgli.SetAreaChannelReq)
	require.True(t, ok)
	assert.Equal(t, uint8(1), chReq.AreaIndex)
	var ch mowgli.AreaChannel
	require.NoError(t, json.Unmarshal([]byte(chReq.SettingsJSON), &ch))
	assert.Equal(t, 0.7, ch.WidthM)
	assert.Equal(t, [][2]float64{{0, 0.5}, {3, 0.5}}, ch.Points)
}

func TestReplaceMap_ChannelJSONRoundTripsThroughRequestBody(t *testing.T) {
	body := `{"areas":[{"area":{"name":"p","area":{"points":[]}},"is_navigation_area":true,"channel":{"points":[[1,2],[3,4]],"width_m":1.2}}]}`
	var req mowgli.ReplaceMapReq
	require.NoError(t, json.Unmarshal([]byte(body), &req))
	require.NotNil(t, req.Areas[0].Channel)
	assert.Equal(t, 1.2, req.Areas[0].Channel.WidthM)
}
