package providers

import (
	"context"
	"fmt"
	"strings"
	"time"

	mqtt "github.com/mochi-mqtt/server/v2"
	"github.com/mochi-mqtt/server/v2/packets"
	types2 "github.com/mowglinext/mowglinext/pkg/types"
	"github.com/sirupsen/logrus"
)

// External rain verdict over MQTT (upstream MowgliNext contract, 59d59e80 /
// docs/MQTT_CONTROL.md): a roof weather station or Home Assistant automation
// publishes yes/no, retained, to <prefix>/external_rain. On the Tron stack this
// embedded broker is the MQTT endpoint, so the verdict is forwarded straight
// to mower_mission (node name behavior_tree_node), which ORs a fresh wet
// verdict with the mower's own rain sensors and expires it after
// external_rain_max_age_minutes. It can only add a reason to stop.

const (
	externalRainTopic       = "external_rain"
	externalRainService     = "/behavior_tree_node/set_external_rain"
	externalRainServiceType = "std_srvs/srv/SetBool"
	externalRainCallTimeout = 10 * time.Second
)

type setBoolReq struct {
	Data bool `json:"data"`
}

type setBoolRes struct {
	Success bool   `json:"success"`
	Message string `json:"message"`
}

// parseExternalRainPayload maps yes/on/true/rain/wet/1 to wet and
// no/off/false/dry/0 to dry (any case, surrounding whitespace ignored).
// Anything else (unknown, unavailable, empty) is not a verdict.
func parseExternalRainPayload(payload []byte) (wet bool, ok bool) {
	switch strings.ToLower(strings.TrimSpace(string(payload))) {
	case "yes", "on", "true", "rain", "wet", "1":
		return true, true
	case "no", "off", "false", "dry", "0":
		return false, true
	}
	return false, false
}

// forwardExternalRain parses one MQTT payload and hands the verdict to the
// mission. A payload that is not yes/no is ignored, never treated as dry.
func forwardExternalRain(ctx context.Context, ros types2.IRosProvider, payload []byte) error {
	wet, ok := parseExternalRainPayload(payload)
	if !ok {
		return fmt.Errorf("external_rain payload %q is not yes/no (or on/off, true/false, wet/dry, 1/0): ignored", string(payload))
	}
	callCtx, cancel := context.WithTimeout(ctx, externalRainCallTimeout)
	defer cancel()
	var res setBoolRes
	if err := ros.CallService(callCtx, externalRainService, &setBoolReq{Data: wet}, &res, externalRainServiceType); err != nil {
		return fmt.Errorf("external_rain: calling %s: %w", externalRainService, err)
	}
	if !res.Success {
		return fmt.Errorf("external_rain: mission did not apply the verdict: %s", res.Message)
	}
	return nil
}

// subscribeToExternalRain wires <prefix>/external_rain to the mission.
// Retained messages are accepted: the verdict is state, not operator intent.
func (hc *MqttProvider) subscribeToExternalRain() {
	topic := hc.prefix + "/" + externalRainTopic
	err := hc.server.Subscribe(topic, 2, func(_ *mqtt.Client, _ packets.Subscription, pk packets.Packet) {
		if err := forwardExternalRain(context.Background(), hc.rosProvider, pk.Payload); err != nil {
			logrus.Warn(err)
			return
		}
		logrus.Infof("external rain verdict %q forwarded to the mission", strings.TrimSpace(string(pk.Payload)))
	})
	if err != nil {
		logrus.Errorf("Failed to subscribe to %s: %v", topic, err)
	}
}
