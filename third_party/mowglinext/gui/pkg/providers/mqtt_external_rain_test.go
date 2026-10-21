package providers

import (
	"context"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestParseExternalRainPayload(t *testing.T) {
	for _, p := range []string{"yes", "YES", " on\n", "true", "rain", "Wet", "1"} {
		wet, ok := parseExternalRainPayload([]byte(p))
		assert.True(t, ok && wet, "%q", p)
	}
	for _, p := range []string{"no", "Off", "false", "dry", "0", " no "} {
		wet, ok := parseExternalRainPayload([]byte(p))
		assert.True(t, ok && !wet, "%q", p)
	}
	for _, p := range []string{"", "  ", "unknown", "unavailable", "maybe", "2", "{\"rain\":true}"} {
		_, ok := parseExternalRainPayload([]byte(p))
		assert.False(t, ok, "%q", p)
	}
}

func TestForwardExternalRain_CallsTheMissionService(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceResponder = func(_ string, _ any, res any) { res.(*setBoolRes).Success = true }

	require.NoError(t, forwardExternalRain(context.Background(), mock, []byte("yes")))
	require.NoError(t, forwardExternalRain(context.Background(), mock, []byte("dry")))

	require.Len(t, mock.ServiceCalls, 2)
	assert.Equal(t, "/behavior_tree_node/set_external_rain", mock.ServiceCalls[0].Service)
	assert.True(t, mock.ServiceCalls[0].Req.(*setBoolReq).Data)
	assert.False(t, mock.ServiceCalls[1].Req.(*setBoolReq).Data)
}

func TestForwardExternalRain_IgnoresGarbageWithoutCalling(t *testing.T) {
	mock := types.NewMockRosProvider()
	assert.Error(t, forwardExternalRain(context.Background(), mock, []byte("unavailable")))
	assert.Empty(t, mock.ServiceCalls, "a malformed payload must never flip the state")
}

func TestForwardExternalRain_ReportsAMissionRefusal(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceResponder = func(_ string, _ any, res any) {
		res.(*setBoolRes).Message = "use_external_rain is false"
	}
	err := forwardExternalRain(context.Background(), mock, []byte("yes"))
	require.Error(t, err)
	assert.Contains(t, err.Error(), "use_external_rain is false")
}
