package providers

import (
	"fmt"
	"regexp"
	"strconv"
	"strings"
	"time"
)

// notifyCatalogue is the server-side wording of every message id, per
// language. Placeholders: {area} {next} {coverage} {battery} {state} {minutes}
// and, for alert-node messages, {cause} {distance} {lat} {lon} {map} {detail}.
// An area with no operator name renders as the language's "Zone N".
var notifyCatalogue = map[string]map[string]string{
	"en": {
		"zoneFallback":           "Zone {index}",
		NotifyMsgMowStarted:      "Mowing started.",
		NotifyMsgZoneStarted:     "Started mowing {area}.",
		NotifyMsgZoneFinished:    "Finished {area} ({coverage} covered).",
		NotifyMsgZoneChanged:     "Finished {area} ({coverage} covered). Now mowing {next}.",
		NotifyMsgMowComplete:     "Mowing complete, heading back to the dock. Last zone {area}: {coverage} covered.",
		NotifyMsgDocked:          "Back on the dock, charging ({battery}).",
		NotifyMsgMowStopped:      "Mowing stopped ({state}).",
		NotifyMsgDigObstruction:  "Blocked: the wheels keep digging in at the same spot. The mission is held until you move the robot or send it home.",
		NotifyMsgNavFailed:       "Stuck: the robot could not complete its mission ({state}). Please check on it.",
		NotifyMsgEmergency:       "EMERGENCY STOP triggered ({state}). Blades and wheels are halted.",
		NotifyMsgBatteryLow:      "Battery low ({battery}), returning to the dock to recharge. Mowing resumes when charged.",
		NotifyMsgBatteryResumed:  "Recharged ({battery}), mowing resumes.",
		NotifyMsgRainDetected:    "Rain detected, returning to the dock.",
		NotifyMsgRainResumed:     "Rain over, mowing resumes.",
		NotifyMsgRainTimeout:     "Rain wait timed out, the mow was abandoned.",
		NotifyMsgRtkWaiting:      "Waiting for an RTK fix for {minutes}. Mowing is paused until GPS quality recovers.",
		NotifyMsgRtkRecovered:    "RTK fix recovered, mowing resumes.",
		NotifyMsgTheftLift:       "THEFT ALERT: the robot was lifted while parked ({state}). Last position: {map}",
		NotifyMsgTheftGeofence:   "THEFT ALERT: the robot moved {distance} from where it was parked while idle. Last position {lat}, {lon}: {map}",
		NotifyMsgTheftOutsideMap: "THEFT ALERT: the robot is outside the mapped area while not mowing ({distance} outside). Last position: {map}",
		NotifyMsgLiftDuringMow:   "Robot LIFTED while mowing: blade and wheels stopped ({state}).",
		NotifyMsgTiltDuringMow:   "Robot TILTED ({detail}) while mowing: blade and wheels stopped.",
		NotifyMsgEmergencyStop:   "Emergency stop: {cause} ({state}). Blades and wheels are halted.",
		NotifyMsgStuck:           "Stuck: the robot could not free itself and needs help ({detail}).",
		NotifyMsgPathBlocked:     "Path blocked for {minutes}: the robot is waiting for the obstacle to clear ({state}).",
		NotifyMsgMowIncomplete:   "Mowing incomplete: {detail}. Start again to mow the rest.",
		NotifyMsgDockFailed:      "Docking failed ({state}): {detail}. Please check on the robot.",
		NotifyMsgUndockFailed:    "Undocking failed: {detail}.",
		NotifyMsgBatteryCritical: "Battery critical ({battery}) and the robot is not on the dock ({state}).",
		NotifyMsgRtkLost:         "RTK fix lost for {minutes} during the mission ({detail}).",
		NotifyMsgTestNotifcation: "Test notification from your robot. Notifications are working.",
	},
	"fr": {
		"zoneFallback":           "Zone {index}",
		NotifyMsgMowStarted:      "Tonte démarrée.",
		NotifyMsgZoneStarted:     "Début de la tonte de {area}.",
		NotifyMsgZoneFinished:    "{area} terminée ({coverage} couverts).",
		NotifyMsgZoneChanged:     "{area} terminée ({coverage} couverts). Tonte de {next} en cours.",
		NotifyMsgMowComplete:     "Tonte terminée, retour à la station. Dernière zone {area} : {coverage} couverts.",
		NotifyMsgDocked:          "De retour à la station, en charge ({battery}).",
		NotifyMsgMowStopped:      "Tonte arrêtée ({state}).",
		NotifyMsgDigObstruction:  "Bloqué : les roues creusent au même endroit. La mission est suspendue jusqu'à ce que vous déplaciez le robot ou le renvoyiez à la station.",
		NotifyMsgNavFailed:       "Coincé : le robot n'a pas pu terminer sa mission ({state}). Allez voir sur place.",
		NotifyMsgEmergency:       "ARRÊT D'URGENCE déclenché ({state}). Lames et roues arrêtées.",
		NotifyMsgBatteryLow:      "Batterie faible ({battery}), retour à la station pour recharger. La tonte reprendra une fois chargé.",
		NotifyMsgBatteryResumed:  "Rechargé ({battery}), la tonte reprend.",
		NotifyMsgRainDetected:    "Pluie détectée, retour à la station.",
		NotifyMsgRainResumed:     "Fin de la pluie, la tonte reprend.",
		NotifyMsgRainTimeout:     "Attente de fin de pluie dépassée, la tonte est abandonnée.",
		NotifyMsgRtkWaiting:      "En attente d'un fix RTK depuis {minutes}. La tonte est en pause jusqu'au retour d'un GPS de qualité.",
		NotifyMsgRtkRecovered:    "Fix RTK retrouvé, la tonte reprend.",
		NotifyMsgTheftLift:       "ALERTE VOL : le robot a été soulevé alors qu'il était à l'arrêt ({state}). Dernière position : {map}",
		NotifyMsgTheftGeofence:   "ALERTE VOL : le robot s'est déplacé de {distance} par rapport à son point de stationnement alors qu'il était à l'arrêt. Dernière position {lat}, {lon} : {map}",
		NotifyMsgTheftOutsideMap: "ALERTE VOL : le robot est en dehors de la zone cartographiée alors qu'il ne tond pas ({distance} à l'extérieur). Dernière position : {map}",
		NotifyMsgLiftDuringMow:   "Robot SOULEVÉ pendant la tonte : lame et roues arrêtées ({state}).",
		NotifyMsgTiltDuringMow:   "Robot INCLINÉ ({detail}) pendant la tonte : lame et roues arrêtées.",
		NotifyMsgEmergencyStop:   "Arrêt d'urgence : {cause} ({state}). Lames et roues arrêtées.",
		NotifyMsgStuck:           "Coincé : le robot n'a pas réussi à se dégager seul et a besoin d'aide ({detail}).",
		NotifyMsgPathBlocked:     "Chemin bloqué depuis {minutes} : le robot attend que l'obstacle disparaisse ({state}).",
		NotifyMsgMowIncomplete:   "Tonte incomplète : {detail}. Relancez la tonte pour finir le reste.",
		NotifyMsgDockFailed:      "Échec de l'accostage ({state}) : {detail}. Allez voir le robot.",
		NotifyMsgUndockFailed:    "Échec du départ de la station : {detail}.",
		NotifyMsgBatteryCritical: "Batterie critique ({battery}) et le robot n'est pas sur la station ({state}).",
		NotifyMsgRtkLost:         "Fix RTK perdu depuis {minutes} pendant la mission ({detail}).",
		NotifyMsgTestNotifcation: "Notification de test de votre robot. Les notifications fonctionnent.",
	},
}

// notifyTags maps a message id to ntfy emoji tags (shown as icons in the app).
var notifyTags = map[string][]string{
	NotifyMsgMowStarted:      {"seedling"},
	NotifyMsgZoneStarted:     {"seedling"},
	NotifyMsgZoneFinished:    {"white_check_mark"},
	NotifyMsgZoneChanged:     {"white_check_mark"},
	NotifyMsgMowComplete:     {"tada"},
	NotifyMsgDocked:          {"electric_plug"},
	NotifyMsgMowStopped:      {"octagonal_sign"},
	NotifyMsgDigObstruction:  {"warning"},
	NotifyMsgNavFailed:       {"warning"},
	NotifyMsgEmergency:       {"rotating_light"},
	NotifyMsgBatteryLow:      {"battery"},
	NotifyMsgBatteryResumed:  {"battery"},
	NotifyMsgRainDetected:    {"cloud_with_rain"},
	NotifyMsgRainResumed:     {"sunny"},
	NotifyMsgRainTimeout:     {"cloud_with_rain"},
	NotifyMsgRtkWaiting:      {"satellite"},
	NotifyMsgRtkRecovered:    {"satellite"},
	NotifyMsgTheftLift:       {"rotating_light"},
	NotifyMsgTheftGeofence:   {"rotating_light"},
	NotifyMsgTheftOutsideMap: {"rotating_light"},
	NotifyMsgLiftDuringMow:   {"warning"},
	NotifyMsgTiltDuringMow:   {"warning"},
	NotifyMsgEmergencyStop:   {"rotating_light"},
	NotifyMsgStuck:           {"warning"},
	NotifyMsgPathBlocked:     {"warning"},
	NotifyMsgMowIncomplete:   {"warning"},
	NotifyMsgDockFailed:      {"warning"},
	NotifyMsgUndockFailed:    {"warning"},
	NotifyMsgBatteryCritical: {"battery"},
	NotifyMsgRtkLost:         {"satellite"},
	NotifyMsgTestNotifcation: {"bell"},
}

// NotifyMessage is a rendered notification ready for a sender.
type NotifyMessage struct {
	Title    string            `json:"title"`
	Body     string            `json:"body"`
	Kind     string            `json:"kind"`
	Message  string            `json:"message"`
	Priority int               `json:"priority"`
	Tags     []string          `json:"tags"`
	Params   map[string]string `json:"params"`
	At       time.Time         `json:"at"`
}

// RenderNotification turns an event into channel-agnostic title + body text.
func RenderNotification(language, title string, ev NotifyEvent) NotifyMessage {
	catalogue, ok := notifyCatalogue[language]
	if !ok {
		catalogue = notifyCatalogue[DefaultNotifyLanguage]
	}
	body, ok := catalogue[ev.Message]
	if !ok {
		// An id from a newer alert node: prefer its own sentence over the raw id.
		if strings.TrimSpace(ev.Text) != "" {
			body = ev.Text
		} else {
			body = ev.Message
		}
	}
	params := resolveAreaNames(catalogue["zoneFallback"], ev.Params)
	pairs := make([]string, 0, 2*len(params))
	for k, v := range params {
		pairs = append(pairs, "{"+k+"}", v)
	}
	body = tidyRendered(strings.NewReplacer(pairs...).Replace(body))
	tags := append([]string(nil), notifyTags[ev.Message]...)
	if tags == nil {
		tags = []string{}
	}
	return NotifyMessage{
		Title:    title,
		Body:     body,
		Kind:     ev.Kind,
		Message:  ev.Message,
		Priority: ev.Priority,
		Tags:     tags,
		Params:   params,
		At:       ev.At,
	}
}

var (
	leftoverPlaceholder = regexp.MustCompile(`\{\w+\}`)
	emptyParens         = regexp.MustCompile(`\s*\(\s*\)`)
	multiSpace          = regexp.MustCompile(` {2,}`)
	spaceBeforePunct    = regexp.MustCompile(` +([.,])`)
)

// tidyRendered cleans up after params the sender did not supply (alert params
// are an arbitrary subset): a "{x}" with no value must not leak into the push,
// and the gaps it leaves ("()", doubled spaces, a space before . or ,) are closed up.
func tidyRendered(body string) string {
	body = leftoverPlaceholder.ReplaceAllString(body, "")
	body = emptyParens.ReplaceAllString(body, "")
	body = multiSpace.ReplaceAllString(body, " ")
	body = spaceBeforePunct.ReplaceAllString(body, "$1")
	return strings.TrimSpace(body)
}

// resolveAreaNames returns a NEW param map where an empty area/next name is
// replaced by the language's "Zone N" using the matching 1-based index.
func resolveAreaNames(fallback string, in map[string]string) map[string]string {
	out := make(map[string]string, len(in))
	for k, v := range in {
		out[k] = v
	}
	fill := func(nameKey, indexKey string) {
		idx, hasIdx := out[indexKey]
		if !hasIdx {
			return
		}
		if strings.TrimSpace(out[nameKey]) == "" {
			out[nameKey] = strings.ReplaceAll(fallback, "{index}", idx)
		}
	}
	fill("area", "areaIndex")
	fill("next", "nextIndex")
	return out
}

func formatPercent(v float32) string {
	return fmt.Sprintf("%.0f %%", v)
}

func formatMinutes(d time.Duration) string {
	minutes := int(d.Round(time.Minute) / time.Minute)
	if minutes < 1 {
		minutes = 1
	}
	return fmt.Sprintf("%d min", minutes)
}

func itoa(i int) string { return strconv.Itoa(i) }

func maxFloat32(a, b float32) float32 {
	if a > b {
		return a
	}
	return b
}
