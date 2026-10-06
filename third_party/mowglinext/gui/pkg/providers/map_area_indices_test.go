package providers

import (
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/stretchr/testify/assert"
	"testing"
)

func TestSplitMapAreasPreservesROSIndices(t *testing.T) {
	working, navigation, indices := splitMapAreas([]mowgli.MapArea{
		{Name: "Front"}, {Name: "Passage", IsNavigationArea: true}, {Name: "Back"},
	})
	assert.Equal(t, []uint32{0, 2}, indices)
	assert.Equal(t, "Front", working[0].Name)
	assert.Equal(t, "Back", working[1].Name)
	assert.Equal(t, "Passage", navigation[0].Name)
}

func TestParseAreaChannel(t *testing.T) {
	ch := parseAreaChannel(mowgli.GetAreaChannelRes{Success: true, SettingsJSON: `{"points":[[0,1],[2,3.5]],"width_m":0.7}`})
	if assert.NotNil(t, ch) {
		assert.Equal(t, [][2]float64{{0, 1}, {2, 3.5}}, ch.Points)
		assert.Equal(t, 0.7, ch.WidthM)
	}
	assert.Nil(t, parseAreaChannel(mowgli.GetAreaChannelRes{Success: true, SettingsJSON: `{}`}))
	assert.Nil(t, parseAreaChannel(mowgli.GetAreaChannelRes{Success: false, SettingsJSON: `{"points":[[0,1],[2,3]],"width_m":1}`}))
	assert.Nil(t, parseAreaChannel(mowgli.GetAreaChannelRes{Success: true, SettingsJSON: `{"points":[[0,1]],"width_m":1}`}))
	assert.Nil(t, parseAreaChannel(mowgli.GetAreaChannelRes{Success: true, SettingsJSON: `garbage`}))
}
