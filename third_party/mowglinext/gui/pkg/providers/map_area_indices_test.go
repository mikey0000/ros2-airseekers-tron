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

func TestParseBoundaryLines(t *testing.T) {
	lines := parseBoundaryLines(mowgli.GetAreaChannelRes{Success: true, SettingsJSON: `{"version":1,"lines":[` +
		`{"id":"bl-1","name":"drop-off","points":[[0,1],[2,3.5],[4,3.5]],"width_m":0.2},` +
		`{"id":"bad","name":"","points":[[0,1]],"width_m":0}]}`})
	assert.Len(t, lines, 1, "a line with fewer than 2 points is dropped")
	assert.Equal(t, "bl-1", lines[0].ID)
	assert.Equal(t, "drop-off", lines[0].Name)
	assert.Equal(t, [2]float64{2, 3.5}, lines[0].Points[1])
	assert.Equal(t, 0.2, lines[0].WidthM)

	assert.Empty(t, parseBoundaryLines(mowgli.GetAreaChannelRes{Success: true, SettingsJSON: `{"version":1,"lines":[]}`}))
	assert.Nil(t, parseBoundaryLines(mowgli.GetAreaChannelRes{Success: false, SettingsJSON: `{"lines":[]}`}))
	assert.Nil(t, parseBoundaryLines(mowgli.GetAreaChannelRes{Success: true, SettingsJSON: `not json`}))
}
