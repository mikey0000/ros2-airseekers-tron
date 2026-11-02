package providers

import (
	"context"
	"crypto/rand"
	"errors"
	"fmt"
	"os"
	"sort"
	"strings"
	"sync"
	"time"

	"github.com/godbus/dbus/v5"
)

// NetworkWifiProvider drives the host's NetworkManager over the system D-Bus:
// the Wi-Fi link status, scans, joining / forgetting networks and the two
// per-connection knobs the mower cares about (band lock, power save).
//
// The GUI container reaches the bus through the host socket mounted at
// /run/dbus/system_bus_socket (docker/docker-compose.gui.yml). Without the
// socket, or without NetworkManager on the host, every call reports the
// provider unavailable instead of failing: Status() answers available=false
// and the mutating calls return ErrWifiUnavailable.
//
// NetworkManager authorises uid 0 without asking polkit, and the GUI image
// runs as root, so no polkit rule is needed on the mower (docs/gui.md
// "Wi-Fi setup").
type NetworkWifiProvider struct {
	dial     func() (NMBus, error)
	iface    string
	scanWait time.Duration
	scanPoll time.Duration
	mu       sync.Mutex
	bus      NMBus
}

// NMBus is the slice of the system bus the provider uses: method calls and
// property reads on objects owned by org.freedesktop.NetworkManager. Tests
// stub it with an in-memory NetworkManager.
type NMBus interface {
	// Call invokes method ("interface.Method") on the object at path and
	// stores the reply values in out.
	Call(ctx context.Context, path dbus.ObjectPath, method string, args []any, out ...any) error
	// Property reads "interface.Property" of the object at path.
	Property(ctx context.Context, path dbus.ObjectPath, property string) (dbus.Variant, error)
}

// Sentinel errors; the API maps them to HTTP statuses.
var (
	ErrWifiUnavailable = errors.New("wi-fi setup unavailable")
	ErrWifiInvalid     = errors.New("invalid request")
	ErrWifiNotFound    = errors.New("not found")
)

const (
	nmService       = "org.freedesktop.NetworkManager"
	nmPath          = dbus.ObjectPath("/org/freedesktop/NetworkManager")
	nmSettingsPath  = dbus.ObjectPath("/org/freedesktop/NetworkManager/Settings")
	nmIface         = "org.freedesktop.NetworkManager"
	nmDeviceIface   = "org.freedesktop.NetworkManager.Device"
	nmWirelessIface = "org.freedesktop.NetworkManager.Device.Wireless"
	nmAPIface       = "org.freedesktop.NetworkManager.AccessPoint"
	nmActiveIface   = "org.freedesktop.NetworkManager.Connection.Active"
	nmIP4Iface      = "org.freedesktop.NetworkManager.IP4Config"
	nmSettingsIface = "org.freedesktop.NetworkManager.Settings"
	nmConnIface     = "org.freedesktop.NetworkManager.Settings.Connection"

	nmDeviceTypeWifi = 2
	nmNoObject       = dbus.ObjectPath("/")

	wifiSettingType     = "802-11-wireless"
	wifiSecuritySetting = "802-11-wireless-security"

	// NM_SETTING_WIRELESS_POWERSAVE_*
	nmPowersaveDefault = 0
	nmPowersaveIgnore  = 1
	nmPowersaveDisable = 2
	nmPowersaveEnable  = 3

	// NM_802_11_AP_FLAGS_PRIVACY and NM_802_11_AP_SEC_KEY_MGMT_*
	apFlagPrivacy    = 0x1
	apSecKeyMgmtPSK  = 0x100
	apSecKeyMgmt8021 = 0x200
	apSecKeyMgmtSAE  = 0x400
	apSecKeyMgmtOWE  = 0x800

	wifiCallTimeout     = 10 * time.Second
	defaultSystemBusSoc = "unix:path=/run/dbus/system_bus_socket"
)

// Wi-Fi security kinds reported for an access point or a saved connection.
const (
	WifiSecurityOpen       = "open"
	WifiSecurityWEP        = "wep"
	WifiSecurityWPA        = "wpa"       // WPA1 PSK only
	WifiSecurityWPA2       = "wpa2"      // RSN PSK
	WifiSecurityWPA2WPA3   = "wpa2-wpa3" // transition mode: PSK and SAE
	WifiSecurityWPA3       = "wpa3"      // SAE only
	WifiSecurityOWE        = "owe"
	WifiSecurityEnterprise = "enterprise"
)

var nmDeviceStates = map[uint32]string{
	0: "unknown", 10: "unmanaged", 20: "unavailable", 30: "disconnected",
	40: "prepare", 50: "config", 60: "need-auth", 70: "ip-config",
	80: "ip-check", 90: "secondaries", 100: "activated", 110: "deactivating", 120: "failed",
}

// WifiSavedNetwork is a NetworkManager Wi-Fi connection profile.
type WifiSavedNetwork struct {
	UUID        string `json:"uuid"`
	ID          string `json:"id"`
	SSID        string `json:"ssid"`
	Security    string `json:"security"`
	Band        string `json:"band"`      // "", "bg" (2.4 GHz) or "a" (5 GHz)
	Powersave   string `json:"powersave"` // default, ignore, disabled, enabled
	Autoconnect bool   `json:"autoconnect"`
	Hidden      bool   `json:"hidden"`
	Active      bool   `json:"active"`
}

// WifiStatus is the Wi-Fi device and its active connection.
type WifiStatus struct {
	Available  bool               `json:"available"`
	Reason     string             `json:"reason,omitempty"`
	Interface  string             `json:"interface,omitempty"`
	State      string             `json:"state,omitempty"`
	Connected  bool               `json:"connected"`
	SSID       string             `json:"ssid,omitempty"`
	BSSID      string             `json:"bssid,omitempty"`
	FreqMHz    uint32             `json:"freq_mhz,omitempty"`
	Strength   uint8              `json:"strength,omitempty"` // percent, NetworkManager's scale
	BitrateKbs uint32             `json:"bitrate_kbps,omitempty"`
	IP4        []string           `json:"ip4,omitempty"` // CIDR
	Connection *WifiSavedNetwork  `json:"connection,omitempty"`
	Saved      []WifiSavedNetwork `json:"saved"`
}

// WifiAccessPoint is one BSS from the device's scan list.
type WifiAccessPoint struct {
	SSID     string `json:"ssid"`
	BSSID    string `json:"bssid"`
	FreqMHz  uint32 `json:"freq_mhz"`
	Strength uint8  `json:"strength"`
	Security string `json:"security"`
	Active   bool   `json:"active"`
	Saved    bool   `json:"saved"`
}

// WifiScanResult is the scan list, strongest first.
type WifiScanResult struct {
	Scanned      bool              `json:"scanned"` // a fresh scan finished before the reply
	AccessPoints []WifiAccessPoint `json:"access_points"`
}

// WifiConnectRequest joins a network. UUID activates that saved profile;
// otherwise the saved profile for SSID is used, or a new one is created.
// A non-empty Password replaces the stored key. Remember=false keeps a new
// profile in memory only (lost when NetworkManager restarts).
type WifiConnectRequest struct {
	SSID     string `json:"ssid"`
	Password string `json:"password"`
	UUID     string `json:"uuid"`
	Hidden   bool   `json:"hidden"`
	Remember *bool  `json:"remember"`
}

// WifiConnectResult names the profile being activated. Activation itself is
// asynchronous: poll the status.
type WifiConnectResult struct {
	UUID    string `json:"uuid"`
	Created bool   `json:"created"`
}

// NewNetworkWifiProvider talks to the system bus named by
// DBUS_SYSTEM_BUS_ADDRESS, else /run/dbus/system_bus_socket. WIFI_INTERFACE
// pins the device (default: the first Wi-Fi device, preferring the active one).
func NewNetworkWifiProvider() *NetworkWifiProvider {
	return newNetworkWifiProvider(dialSystemBus, os.Getenv("WIFI_INTERFACE"))
}

// NewNetworkWifiProviderWithBus wraps an existing bus (tests).
func NewNetworkWifiProviderWithBus(bus NMBus, iface string) *NetworkWifiProvider {
	p := newNetworkWifiProvider(func() (NMBus, error) { return bus, nil }, iface)
	p.scanWait = 200 * time.Millisecond
	p.scanPoll = 10 * time.Millisecond
	return p
}

func newNetworkWifiProvider(dial func() (NMBus, error), iface string) *NetworkWifiProvider {
	return &NetworkWifiProvider{dial: dial, iface: iface, scanWait: 6 * time.Second, scanPoll: 250 * time.Millisecond}
}

// Status reports the Wi-Fi device; available=false (with a reason) when the
// bus, NetworkManager or a Wi-Fi device is missing.
func (p *NetworkWifiProvider) Status(ctx context.Context) WifiStatus {
	bus, dev, err := p.device(ctx)
	if err != nil {
		return WifiStatus{Available: false, Reason: unavailableReason(err), Saved: []WifiSavedNetwork{}}
	}
	st := WifiStatus{Available: true, Interface: dev.iface, State: nmDeviceStates[dev.state], Saved: []WifiSavedNetwork{}}
	activeConn := p.activeConnectionPath(ctx, bus, dev)
	saved, err := listWifiConnections(ctx, bus)
	if err == nil {
		for _, s := range saved {
			s.net.Active = activeConn != "" && s.path == activeConn
			st.Saved = append(st.Saved, s.net)
			if s.net.Active {
				active := s.net
				st.Connection = &active
			}
		}
	}
	st.Connected = dev.state == 100
	if ap := propPath(ctx, bus, dev.path, nmWirelessIface+".ActiveAccessPoint"); ap != "" && ap != nmNoObject {
		info := readAccessPoint(ctx, bus, ap)
		st.SSID, st.BSSID, st.FreqMHz, st.Strength = info.SSID, info.BSSID, info.FreqMHz, info.Strength
	}
	if st.SSID == "" && st.Connection != nil {
		st.SSID = st.Connection.SSID
	}
	if v, err := bus.Property(ctx, dev.path, nmWirelessIface+".Bitrate"); err == nil {
		st.BitrateKbs, _ = v.Value().(uint32)
	}
	if ip4 := propPath(ctx, bus, dev.path, nmDeviceIface+".Ip4Config"); ip4 != "" && ip4 != nmNoObject {
		st.IP4 = readIP4(ctx, bus, ip4)
	}
	return st
}

// Scan returns the access points the device knows. With rescan it first asks
// for a new scan and waits (bounded) for it to finish; a refused scan (one is
// already running, or NetworkManager rate-limits) still returns the list.
func (p *NetworkWifiProvider) Scan(ctx context.Context, rescan bool) (WifiScanResult, error) {
	bus, dev, err := p.device(ctx)
	if err != nil {
		return WifiScanResult{AccessPoints: []WifiAccessPoint{}}, err
	}
	scanned := false
	if rescan {
		before := lastScan(ctx, bus, dev.path)
		if err := bus.Call(ctx, dev.path, nmWirelessIface+".RequestScan", []any{map[string]dbus.Variant{}}); err == nil {
			deadline := time.Now().Add(p.scanWait)
			for time.Now().Before(deadline) {
				if lastScan(ctx, bus, dev.path) != before {
					scanned = true
					break
				}
				select {
				case <-ctx.Done():
					return WifiScanResult{AccessPoints: []WifiAccessPoint{}}, ctx.Err()
				case <-time.After(p.scanPoll):
				}
			}
		}
	}
	var paths []dbus.ObjectPath
	if err := bus.Call(ctx, dev.path, nmWirelessIface+".GetAllAccessPoints", nil, &paths); err != nil {
		return WifiScanResult{AccessPoints: []WifiAccessPoint{}}, fmt.Errorf("listing access points: %w", err)
	}
	active := propPath(ctx, bus, dev.path, nmWirelessIface+".ActiveAccessPoint")
	savedSSIDs := map[string]bool{}
	if saved, err := listWifiConnections(ctx, bus); err == nil {
		for _, s := range saved {
			savedSSIDs[s.net.SSID] = true
		}
	}
	aps := make([]WifiAccessPoint, 0, len(paths))
	for _, path := range paths {
		ap := readAccessPoint(ctx, bus, path)
		ap.Active = path == active
		ap.Saved = ap.SSID != "" && savedSSIDs[ap.SSID]
		aps = append(aps, ap)
	}
	sort.SliceStable(aps, func(i, j int) bool { return aps[i].Strength > aps[j].Strength })
	return WifiScanResult{Scanned: scanned, AccessPoints: aps}, nil
}

// Connect joins a network (see WifiConnectRequest).
func (p *NetworkWifiProvider) Connect(ctx context.Context, req WifiConnectRequest) (WifiConnectResult, error) {
	bus, dev, err := p.device(ctx)
	if err != nil {
		return WifiConnectResult{}, err
	}
	if req.Password != "" && !validPSK(req.Password) {
		return WifiConnectResult{}, fmt.Errorf("%w: a WPA password is 8 to 63 characters (or 64 hex digits)", ErrWifiInvalid)
	}
	saved, err := listWifiConnections(ctx, bus)
	if err != nil {
		return WifiConnectResult{}, err
	}
	var target *wifiConnection
	if req.UUID != "" {
		target = findConnection(saved, func(c wifiConnection) bool { return c.net.UUID == req.UUID })
		if target == nil {
			return WifiConnectResult{}, fmt.Errorf("%w: no saved network %s", ErrWifiNotFound, req.UUID)
		}
	} else {
		if req.SSID == "" || len(req.SSID) > 32 {
			return WifiConnectResult{}, fmt.Errorf("%w: ssid must be 1 to 32 bytes", ErrWifiInvalid)
		}
		target = findConnection(saved, func(c wifiConnection) bool { return c.net.SSID == req.SSID })
	}

	if target != nil {
		if req.Password != "" {
			if err := updateConnection(ctx, bus, target.path, func(s nmSettings) error {
				sec := s[wifiSecuritySetting]
				if sec == nil {
					return fmt.Errorf("%w: %s is an open network, it takes no password", ErrWifiInvalid, target.net.SSID)
				}
				sec["psk"] = dbus.MakeVariant(req.Password)
				return nil
			}); err != nil {
				return WifiConnectResult{}, err
			}
		}
		var active dbus.ObjectPath
		if err := bus.Call(ctx, nmPath, nmIface+".ActivateConnection",
			[]any{target.path, dev.path, nmNoObject}, &active); err != nil {
			return WifiConnectResult{}, fmt.Errorf("activating %s: %w", target.net.ID, err)
		}
		return WifiConnectResult{UUID: target.net.UUID}, nil
	}

	security := WifiSecurityWPA2
	if !req.Hidden {
		ap, ok := strongestAP(ctx, bus, dev.path, req.SSID)
		if !ok {
			return WifiConnectResult{}, fmt.Errorf("%w: %s is not in range (scan again, or join it as a hidden network)", ErrWifiNotFound, req.SSID)
		}
		security = ap.Security
	} else if req.Password == "" {
		security = WifiSecurityOpen
	}
	settings, err := newWifiSettings(req, security)
	if err != nil {
		return WifiConnectResult{}, err
	}
	remember := req.Remember == nil || *req.Remember
	var connPath, active dbus.ObjectPath
	if remember {
		err = bus.Call(ctx, nmPath, nmIface+".AddAndActivateConnection",
			[]any{settings, dev.path, nmNoObject}, &connPath, &active)
	} else {
		var result map[string]dbus.Variant
		err = bus.Call(ctx, nmPath, nmIface+".AddAndActivateConnection2",
			[]any{settings, dev.path, nmNoObject, map[string]dbus.Variant{"persist": dbus.MakeVariant("memory")}},
			&connPath, &active, &result)
	}
	if err != nil {
		return WifiConnectResult{}, fmt.Errorf("joining %s: %w", req.SSID, err)
	}
	uuid, _ := settings["connection"]["uuid"].Value().(string)
	return WifiConnectResult{UUID: uuid, Created: true}, nil
}

// Forget deletes a saved network profile. Forgetting the active one drops the link.
func (p *NetworkWifiProvider) Forget(ctx context.Context, uuid string) error {
	bus, _, err := p.device(ctx)
	if err != nil {
		return err
	}
	conn, err := p.connectionByUUID(ctx, bus, uuid)
	if err != nil {
		return err
	}
	if err := bus.Call(ctx, conn.path, nmConnIface+".Delete", nil); err != nil {
		return fmt.Errorf("deleting %s: %w", conn.net.ID, err)
	}
	return nil
}

// SetBand locks a saved network to "bg" (2.4 GHz), "a" (5 GHz) or "" (any)
// and re-activates it when it is the active one. uuid "" = the active network.
func (p *NetworkWifiProvider) SetBand(ctx context.Context, uuid, band string) (WifiSavedNetwork, error) {
	if band != "" && band != "bg" && band != "a" {
		return WifiSavedNetwork{}, fmt.Errorf("%w: band must be \"\", \"bg\" or \"a\"", ErrWifiInvalid)
	}
	return p.modifyWireless(ctx, uuid, func(w map[string]dbus.Variant) {
		if band == "" {
			delete(w, "band")
			delete(w, "channel") // a channel only makes sense with a band
		} else {
			w["band"] = dbus.MakeVariant(band)
		}
	})
}

// SetPowersave switches 802-11-wireless.powersave to enable (3) or disable (2)
// on a saved network and re-activates it when it is the active one.
func (p *NetworkWifiProvider) SetPowersave(ctx context.Context, uuid string, enabled bool) (WifiSavedNetwork, error) {
	value := uint32(nmPowersaveDisable)
	if enabled {
		value = nmPowersaveEnable
	}
	return p.modifyWireless(ctx, uuid, func(w map[string]dbus.Variant) {
		w["powersave"] = dbus.MakeVariant(value)
	})
}

func (p *NetworkWifiProvider) modifyWireless(ctx context.Context, uuid string, edit func(map[string]dbus.Variant)) (WifiSavedNetwork, error) {
	bus, dev, err := p.device(ctx)
	if err != nil {
		return WifiSavedNetwork{}, err
	}
	activeConn := p.activeConnectionPath(ctx, bus, dev)
	var conn wifiConnection
	if uuid == "" {
		saved, err := listWifiConnections(ctx, bus)
		if err != nil {
			return WifiSavedNetwork{}, err
		}
		found := findConnection(saved, func(c wifiConnection) bool { return activeConn != "" && c.path == activeConn })
		if found == nil {
			return WifiSavedNetwork{}, fmt.Errorf("%w: no active Wi-Fi network; name one by uuid", ErrWifiNotFound)
		}
		conn = *found
	} else if conn, err = p.connectionByUUID(ctx, bus, uuid); err != nil {
		return WifiSavedNetwork{}, err
	}
	if err := updateConnection(ctx, bus, conn.path, func(s nmSettings) error {
		if s[wifiSettingType] == nil {
			s[wifiSettingType] = map[string]dbus.Variant{}
		}
		edit(s[wifiSettingType])
		return nil
	}); err != nil {
		return WifiSavedNetwork{}, err
	}
	if conn.path == activeConn {
		var active dbus.ObjectPath
		if err := bus.Call(ctx, nmPath, nmIface+".ActivateConnection",
			[]any{conn.path, dev.path, nmNoObject}, &active); err != nil {
			return WifiSavedNetwork{}, fmt.Errorf("re-activating %s: %w", conn.net.ID, err)
		}
	}
	updated, err := readConnection(ctx, bus, conn.path)
	if err != nil || updated == nil {
		return conn.net, err
	}
	updated.net.Active = conn.path == activeConn
	return updated.net, nil
}

func (p *NetworkWifiProvider) connectionByUUID(ctx context.Context, bus NMBus, uuid string) (wifiConnection, error) {
	if uuid == "" {
		return wifiConnection{}, fmt.Errorf("%w: uuid is required", ErrWifiInvalid)
	}
	saved, err := listWifiConnections(ctx, bus)
	if err != nil {
		return wifiConnection{}, err
	}
	found := findConnection(saved, func(c wifiConnection) bool { return c.net.UUID == uuid })
	if found == nil {
		return wifiConnection{}, fmt.Errorf("%w: no saved network %s", ErrWifiNotFound, uuid)
	}
	return *found, nil
}

// --- bus + device -----------------------------------------------------------

type wifiDevice struct {
	path  dbus.ObjectPath
	iface string
	state uint32
}

// getBus returns the cached bus, dialling it on first use or after a failure.
func (p *NetworkWifiProvider) getBus() (NMBus, error) {
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.bus != nil {
		if c, ok := p.bus.(interface{ Connected() bool }); !ok || c.Connected() {
			return p.bus, nil
		}
		p.bus = nil
	}
	bus, err := p.dial()
	if err != nil {
		return nil, fmt.Errorf("%w: system D-Bus: %v", ErrWifiUnavailable, err)
	}
	p.bus = bus
	return bus, nil
}

func (p *NetworkWifiProvider) device(ctx context.Context) (NMBus, wifiDevice, error) {
	bus, err := p.getBus()
	if err != nil {
		return nil, wifiDevice{}, err
	}
	var devices []dbus.ObjectPath
	if err := bus.Call(ctx, nmPath, nmIface+".GetDevices", nil, &devices); err != nil {
		return nil, wifiDevice{}, fmt.Errorf("%w: NetworkManager: %v", ErrWifiUnavailable, err)
	}
	var candidates []wifiDevice
	for _, path := range devices {
		v, err := bus.Property(ctx, path, nmDeviceIface+".DeviceType")
		if err != nil {
			continue
		}
		if t, _ := v.Value().(uint32); t != nmDeviceTypeWifi {
			continue
		}
		dev := wifiDevice{path: path}
		if v, err := bus.Property(ctx, path, nmDeviceIface+".Interface"); err == nil {
			dev.iface, _ = v.Value().(string)
		}
		if v, err := bus.Property(ctx, path, nmDeviceIface+".State"); err == nil {
			dev.state, _ = v.Value().(uint32)
		}
		if p.iface != "" && dev.iface == p.iface {
			return bus, dev, nil
		}
		candidates = append(candidates, dev)
	}
	if p.iface != "" || len(candidates) == 0 {
		name := "a Wi-Fi device"
		if p.iface != "" {
			name = "Wi-Fi device " + p.iface
		}
		return nil, wifiDevice{}, fmt.Errorf("%w: NetworkManager manages no %s", ErrWifiUnavailable, name)
	}
	for _, dev := range candidates {
		if dev.state == 100 {
			return bus, dev, nil
		}
	}
	return bus, candidates[0], nil
}

func (p *NetworkWifiProvider) activeConnectionPath(ctx context.Context, bus NMBus, dev wifiDevice) dbus.ObjectPath {
	active := propPath(ctx, bus, dev.path, nmDeviceIface+".ActiveConnection")
	if active == "" || active == nmNoObject {
		return ""
	}
	return propPath(ctx, bus, active, nmActiveIface+".Connection")
}

func unavailableReason(err error) string {
	return strings.TrimPrefix(err.Error(), ErrWifiUnavailable.Error()+": ")
}

// --- settings ---------------------------------------------------------------

type nmSettings = map[string]map[string]dbus.Variant

type wifiConnection struct {
	path dbus.ObjectPath
	net  WifiSavedNetwork
}

func findConnection(list []wifiConnection, match func(wifiConnection) bool) *wifiConnection {
	for i := range list {
		if match(list[i]) {
			return &list[i]
		}
	}
	return nil
}

func listWifiConnections(ctx context.Context, bus NMBus) ([]wifiConnection, error) {
	var paths []dbus.ObjectPath
	if err := bus.Call(ctx, nmSettingsPath, nmSettingsIface+".ListConnections", nil, &paths); err != nil {
		return nil, fmt.Errorf("listing saved networks: %w", err)
	}
	out := []wifiConnection{}
	for _, path := range paths {
		conn, err := readConnection(ctx, bus, path)
		if err != nil || conn == nil {
			continue
		}
		out = append(out, *conn)
	}
	return out, nil
}

// readConnection returns nil (no error) for a profile that is not Wi-Fi.
func readConnection(ctx context.Context, bus NMBus, path dbus.ObjectPath) (*wifiConnection, error) {
	var s nmSettings
	if err := bus.Call(ctx, path, nmConnIface+".GetSettings", nil, &s); err != nil {
		return nil, err
	}
	c := s["connection"]
	if nmStr(c["type"]) != wifiSettingType {
		return nil, nil
	}
	w := s[wifiSettingType]
	net := WifiSavedNetwork{
		UUID:        nmStr(c["uuid"]),
		ID:          nmStr(c["id"]),
		SSID:        string(nmBytes(w["ssid"])),
		Band:        nmStr(w["band"]),
		Powersave:   powersaveName(w["powersave"]),
		Autoconnect: true,
		Hidden:      nmBool(w["hidden"]),
		Security:    connectionSecurity(s[wifiSecuritySetting]),
	}
	if v, ok := c["autoconnect"]; ok {
		net.Autoconnect = nmBool(v)
	}
	return &wifiConnection{path: path, net: net}, nil
}

// updateConnection is a GetSettings -> edit -> Update round trip. GetSettings
// carries no secrets; NetworkManager keeps the stored ones when an update has
// none, so the PSK survives a band / power-save change.
func updateConnection(ctx context.Context, bus NMBus, path dbus.ObjectPath, edit func(nmSettings) error) error {
	var s nmSettings
	if err := bus.Call(ctx, path, nmConnIface+".GetSettings", nil, &s); err != nil {
		return fmt.Errorf("reading network settings: %w", err)
	}
	if err := edit(s); err != nil {
		return err
	}
	// The deprecated address/route arrays duplicate address-data/route-data;
	// sending both back is rejected when they disagree.
	for _, ip := range []string{"ipv4", "ipv6"} {
		if sec := s[ip]; sec != nil {
			if _, ok := sec["address-data"]; ok {
				delete(sec, "addresses")
			}
			if _, ok := sec["route-data"]; ok {
				delete(sec, "routes")
			}
		}
	}
	if err := bus.Call(ctx, path, nmConnIface+".Update", []any{s}); err != nil {
		return fmt.Errorf("saving network settings: %w", err)
	}
	return nil
}

func newWifiSettings(req WifiConnectRequest, security string) (nmSettings, error) {
	uuid, err := newUUID()
	if err != nil {
		return nil, err
	}
	wireless := map[string]dbus.Variant{
		"ssid": dbus.MakeVariant([]byte(req.SSID)),
		"mode": dbus.MakeVariant("infrastructure"),
	}
	if req.Hidden {
		wireless["hidden"] = dbus.MakeVariant(true)
	}
	s := nmSettings{
		"connection": {
			"id":          dbus.MakeVariant(req.SSID),
			"uuid":        dbus.MakeVariant(uuid),
			"type":        dbus.MakeVariant(wifiSettingType),
			"autoconnect": dbus.MakeVariant(true),
		},
		wifiSettingType: wireless,
		"ipv4":          {"method": dbus.MakeVariant("auto")},
		"ipv6":          {"method": dbus.MakeVariant("auto")},
	}
	switch security {
	case WifiSecurityOpen:
		if req.Password != "" {
			return nil, fmt.Errorf("%w: %s is an open network, it takes no password", ErrWifiInvalid, req.SSID)
		}
	case WifiSecurityOWE:
		if req.Password != "" {
			return nil, fmt.Errorf("%w: %s is an enhanced-open network, it takes no password", ErrWifiInvalid, req.SSID)
		}
		s[wifiSecuritySetting] = map[string]dbus.Variant{"key-mgmt": dbus.MakeVariant("owe")}
	case WifiSecurityWPA, WifiSecurityWPA2, WifiSecurityWPA2WPA3, WifiSecurityWPA3:
		if req.Password == "" {
			return nil, fmt.Errorf("%w: %s needs a password", ErrWifiInvalid, req.SSID)
		}
		// WPA2/WPA3 transition networks join over wpa-psk: the 43455's
		// firmware SAE support is the less tested path. SAE-only needs "sae".
		keyMgmt := "wpa-psk"
		if security == WifiSecurityWPA3 {
			keyMgmt = "sae"
		}
		s[wifiSecuritySetting] = map[string]dbus.Variant{
			"key-mgmt": dbus.MakeVariant(keyMgmt),
			"psk":      dbus.MakeVariant(req.Password),
		}
	default:
		return nil, fmt.Errorf("%w: %s uses %s security, which this page cannot set up", ErrWifiInvalid, req.SSID, security)
	}
	return s, nil
}

// validPSK: 8..63 printable ASCII characters, or a 64-digit hex key.
func validPSK(psk string) bool {
	if len(psk) == 64 {
		for _, r := range psk {
			if !strings.ContainsRune("0123456789abcdefABCDEF", r) {
				return false
			}
		}
		return true
	}
	if len(psk) < 8 || len(psk) > 63 {
		return false
	}
	for _, r := range psk {
		if r < 0x20 || r > 0x7e {
			return false
		}
	}
	return true
}

func connectionSecurity(sec map[string]dbus.Variant) string {
	if sec == nil {
		return WifiSecurityOpen
	}
	switch nmStr(sec["key-mgmt"]) {
	case "wpa-psk":
		return WifiSecurityWPA2
	case "sae":
		return WifiSecurityWPA3
	case "owe":
		return WifiSecurityOWE
	case "none", "ieee8021x":
		return WifiSecurityWEP
	case "wpa-eap", "wpa-eap-suite-b-192":
		return WifiSecurityEnterprise
	}
	return WifiSecurityWPA2
}

func powersaveName(v dbus.Variant) string {
	n, _ := v.Value().(uint32)
	switch n {
	case nmPowersaveIgnore:
		return "ignore"
	case nmPowersaveDisable:
		return "disabled"
	case nmPowersaveEnable:
		return "enabled"
	}
	return "default"
}

// --- access points ----------------------------------------------------------

func readAccessPoint(ctx context.Context, bus NMBus, path dbus.ObjectPath) WifiAccessPoint {
	get := func(name string) any {
		v, err := bus.Property(ctx, path, nmAPIface+"."+name)
		if err != nil {
			return nil
		}
		return v.Value()
	}
	ap := WifiAccessPoint{}
	if b, ok := get("Ssid").([]byte); ok {
		ap.SSID = string(b)
	}
	ap.BSSID, _ = get("HwAddress").(string)
	ap.FreqMHz, _ = get("Frequency").(uint32)
	ap.Strength, _ = get("Strength").(uint8)
	flags, _ := get("Flags").(uint32)
	wpa, _ := get("WpaFlags").(uint32)
	rsn, _ := get("RsnFlags").(uint32)
	ap.Security = apSecurity(flags, wpa, rsn)
	return ap
}

// apSecurity classifies an AP from its Flags / WpaFlags / RsnFlags.
func apSecurity(flags, wpa, rsn uint32) string {
	all := wpa | rsn
	switch {
	case all&apSecKeyMgmt8021 != 0:
		return WifiSecurityEnterprise
	case rsn&apSecKeyMgmtSAE != 0 && all&apSecKeyMgmtPSK != 0:
		return WifiSecurityWPA2WPA3
	case rsn&apSecKeyMgmtSAE != 0:
		return WifiSecurityWPA3
	case rsn&apSecKeyMgmtPSK != 0:
		return WifiSecurityWPA2
	case wpa&apSecKeyMgmtPSK != 0:
		return WifiSecurityWPA
	case rsn&apSecKeyMgmtOWE != 0:
		return WifiSecurityOWE
	case flags&apFlagPrivacy != 0:
		return WifiSecurityWEP
	}
	return WifiSecurityOpen
}

func strongestAP(ctx context.Context, bus NMBus, dev dbus.ObjectPath, ssid string) (WifiAccessPoint, bool) {
	var paths []dbus.ObjectPath
	if err := bus.Call(ctx, dev, nmWirelessIface+".GetAllAccessPoints", nil, &paths); err != nil {
		return WifiAccessPoint{}, false
	}
	best, found := WifiAccessPoint{}, false
	for _, path := range paths {
		ap := readAccessPoint(ctx, bus, path)
		if ap.SSID == ssid && (!found || ap.Strength > best.Strength) {
			best, found = ap, true
		}
	}
	return best, found
}

func lastScan(ctx context.Context, bus NMBus, dev dbus.ObjectPath) int64 {
	v, err := bus.Property(ctx, dev, nmWirelessIface+".LastScan")
	if err != nil {
		return -1
	}
	n, _ := v.Value().(int64)
	return n
}

func readIP4(ctx context.Context, bus NMBus, path dbus.ObjectPath) []string {
	v, err := bus.Property(ctx, path, nmIP4Iface+".AddressData")
	if err != nil {
		return nil
	}
	list, _ := v.Value().([]map[string]dbus.Variant)
	var out []string
	for _, a := range list {
		addr := nmStr(a["address"])
		if addr == "" {
			continue
		}
		if prefix, ok := a["prefix"].Value().(uint32); ok {
			addr = fmt.Sprintf("%s/%d", addr, prefix)
		}
		out = append(out, addr)
	}
	return out
}

// --- variant helpers --------------------------------------------------------

func propPath(ctx context.Context, bus NMBus, path dbus.ObjectPath, prop string) dbus.ObjectPath {
	v, err := bus.Property(ctx, path, prop)
	if err != nil {
		return ""
	}
	p, _ := v.Value().(dbus.ObjectPath)
	return p
}

func nmStr(v dbus.Variant) string {
	s, _ := v.Value().(string)
	return s
}

func nmBool(v dbus.Variant) bool {
	b, _ := v.Value().(bool)
	return b
}

func nmBytes(v dbus.Variant) []byte {
	b, _ := v.Value().([]byte)
	return b
}

func newUUID() (string, error) {
	var b [16]byte
	if _, err := rand.Read(b[:]); err != nil {
		return "", err
	}
	b[6] = b[6]&0x0f | 0x40
	b[8] = b[8]&0x3f | 0x80
	return fmt.Sprintf("%x-%x-%x-%x-%x", b[0:4], b[4:6], b[6:8], b[8:10], b[10:16]), nil
}

// --- real system bus --------------------------------------------------------

type systemBus struct {
	conn *dbus.Conn
}

func dialSystemBus() (NMBus, error) {
	addr := os.Getenv("DBUS_SYSTEM_BUS_ADDRESS")
	if addr == "" {
		if _, err := os.Stat("/run/dbus/system_bus_socket"); err != nil {
			return nil, fmt.Errorf("no socket at /run/dbus/system_bus_socket (mount it into the container)")
		}
		addr = defaultSystemBusSoc
	}
	conn, err := dbus.Connect(addr)
	if err != nil {
		return nil, err
	}
	return &systemBus{conn: conn}, nil
}

func (b *systemBus) Connected() bool { return b.conn.Connected() }

func (b *systemBus) Call(ctx context.Context, path dbus.ObjectPath, method string, args []any, out ...any) error {
	ctx, cancel := context.WithTimeout(ctx, wifiCallTimeout)
	defer cancel()
	call := b.conn.Object(nmService, path).CallWithContext(ctx, method, 0, args...)
	if call.Err != nil {
		return call.Err
	}
	if len(out) == 0 {
		return nil
	}
	return call.Store(out...)
}

func (b *systemBus) Property(ctx context.Context, path dbus.ObjectPath, property string) (dbus.Variant, error) {
	dot := strings.LastIndex(property, ".")
	if dot < 0 {
		return dbus.Variant{}, fmt.Errorf("property %q has no interface", property)
	}
	var v dbus.Variant
	err := b.Call(ctx, path, "org.freedesktop.DBus.Properties.Get", []any{property[:dot], property[dot+1:]}, &v)
	return v, err
}
