package api

import (
	"context"
	"encoding/json"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/msgs/geometry"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestReplaceMap_SendsPathChannelAfterItsAddArea(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceResponder = func(_ string, _ any, res any) {
		if r, ok := res.(*mowgli.SetAreaChannelRes); ok {
			r.Success = true
		}
	}
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
	for _, c := range mock.ServiceCalls {
		services = append(services, c.Service)
	}
	assert.Equal(t, []string{
		"/map_server_node/clear_map",
		"/map_server_node/add_area",
		"/map_server_node/add_area",
		mowgli.SetAreaChannelService,
		"/map_server_node/add_area",
		"/map_server_node/save_areas",
	}, services)

	chReq, ok := mock.ServiceCalls[3].Req.(*mowgli.SetAreaChannelReq)
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
