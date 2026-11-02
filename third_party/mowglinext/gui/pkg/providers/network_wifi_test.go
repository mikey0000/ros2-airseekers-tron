package providers

import (
	"context"
	"errors"
	"fmt"
	"strings"
	"sync"
	"testing"

	"github.com/godbus/dbus/v5"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// fakeNM is an in-memory NetworkManager behind the NMBus interface. Object
// paths and property names follow the real D-Bus API; the identifiers are
// placeholders (documentation-range addresses, locally administered MACs).
type fakeNM struct {
	mu          sync.Mutex
	props       map[dbus.ObjectPath]map[string]any
	conns       map[dbus.ObjectPath]nmSettings
	connOrder   []dbus.ObjectPath
	calls       []string
	added       []nmSettings
	addOptions  map[string]dbus.Variant
	updates     map[dbus.ObjectPath]nmSettings
	activated   []dbus.ObjectPath
	nextConn    int
	scanRefused bool
	failGetDevs error
}

const (
	fakeWifiDev  = dbus.ObjectPath("/org/freedesktop/NetworkManager/Devices/3")
	fakeEthDev   = dbus.ObjectPath("/org/freedesktop/NetworkManager/Devices/2")
	fakeActive   = dbus.ObjectPath("/org/freedesktop/NetworkManager/ActiveConnection/7")
	fakeIP4      = dbus.ObjectPath("/org/freedesktop/NetworkManager/IP4Config/5")
	fakeAPHome24 = dbus.ObjectPath("/org/freedesktop/NetworkManager/AccessPoint/1")
	fakeAPHome5  = dbus.ObjectPath("/org/freedesktop/NetworkManager/AccessPoint/2")
	fakeAPCafe   = dbus.ObjectPath("/org/freedesktop/NetworkManager/AccessPoint/3")
	fakeAPSae    = dbus.ObjectPath("/org/freedesktop/NetworkManager/AccessPoint/4")
	fakeAPCorp   = dbus.ObjectPath("/org/freedesktop/NetworkManager/AccessPoint/5")
	fakeConnHome = dbus.ObjectPath("/org/freedesktop/NetworkManager/Settings/1")
	fakeConnEth  = dbus.ObjectPath("/org/freedesktop/NetworkManager/Settings/2")
	fakeConnOld  = dbus.ObjectPath("/org/freedesktop/NetworkManager/Settings/3")
)

func newFakeNM() *fakeNM {
	ap := func(ssid, bssid string, freq uint32, strength uint8, flags, wpa, rsn uint32) map[string]any {
		return map[string]any{
			nmAPIface + ".Ssid": []byte(ssid), nmAPIface + ".HwAddress": bssid,
			nmAPIface + ".Frequency": freq, nmAPIface + ".Strength": strength,
			nmAPIface + ".Flags": flags, nmAPIface + ".WpaFlags": wpa, nmAPIface + ".RsnFlags": rsn,
		}
	}
	f := &fakeNM{
		props: map[dbus.ObjectPath]map[string]any{
			fakeEthDev: {nmDeviceIface + ".DeviceType": uint32(1), nmDeviceIface + ".Interface": "eth0", nmDeviceIface + ".State": uint32(30)},
			fakeWifiDev: {
				nmDeviceIface + ".DeviceType":          uint32(nmDeviceTypeWifi),
				nmDeviceIface + ".Interface":           "wlan0",
				nmDeviceIface + ".State":               uint32(100),
				nmDeviceIface + ".ActiveConnection":    fakeActive,
				nmDeviceIface + ".Ip4Config":           fakeIP4,
				nmWirelessIface + ".ActiveAccessPoint": fakeAPHome5,
				nmWirelessIface + ".Bitrate":           uint32(173000),
				nmWirelessIface + ".LastScan":          int64(1000),
			},
			fakeActive: {nmActiveIface + ".Connection": fakeConnHome},
			fakeIP4: {nmIP4Iface + ".AddressData": []map[string]dbus.Variant{
				{"address": dbus.MakeVariant("192.0.2.10"), "prefix": dbus.MakeVariant(uint32(24))},
			}},
			fakeAPHome24: ap("ExampleHome", "02:00:00:00:00:01", 2437, 70, 1, 0, apSecKeyMgmtPSK),
			fakeAPHome5:  ap("ExampleHome", "02:00:00:00:00:02", 5180, 55, 1, 0, apSecKeyMgmtPSK),
			fakeAPCafe:   ap("ExampleCafe", "02:00:00:00:00:03", 2412, 40, 0, 0, 0),
			fakeAPSae:    ap("ExampleSae", "02:00:00:00:00:04", 5500, 80, 1, 0, apSecKeyMgmtSAE),
			fakeAPCorp:   ap("ExampleCorp", "02:00:00:00:00:05", 5240, 30, 1, 0, apSecKeyMgmt8021),
		},
		conns:   map[dbus.ObjectPath]nmSettings{},
		updates: map[dbus.ObjectPath]nmSettings{},
	}
	f.addConn(fakeConnHome, nmSettings{
		"connection": {"id": dbus.MakeVariant("ExampleHome"), "uuid": dbus.MakeVariant("uuid-home"), "type": dbus.MakeVariant(wifiSettingType)},
		wifiSettingType: {
			"ssid": dbus.MakeVariant([]byte("ExampleHome")), "mode": dbus.MakeVariant("infrastructure"),
			"powersave": dbus.MakeVariant(uint32(nmPowersaveEnable)),
		},
		wifiSecuritySetting: {"key-mgmt": dbus.MakeVariant("wpa-psk")},
		"ipv4": {
			"method":       dbus.MakeVariant("auto"),
			"address-data": dbus.MakeVariant([]map[string]dbus.Variant{}),
			"addresses":    dbus.MakeVariant([][]uint32{}),
		},
	})
	f.addConn(fakeConnEth, nmSettings{
		"connection": {"id": dbus.MakeVariant("Wired"), "uuid": dbus.MakeVariant("uuid-eth"), "type": dbus.MakeVariant("802-3-ethernet")},
	})
	f.addConn(fakeConnOld, nmSettings{
		"connection": {"id": dbus.MakeVariant("ExampleOld"), "uuid": dbus.MakeVariant("uuid-old"), "type": dbus.MakeVariant(wifiSettingType),
			"autoconnect": dbus.MakeVariant(false)},
		wifiSettingType: {"ssid": dbus.MakeVariant([]byte("ExampleOld")), "band": dbus.MakeVariant("bg")},
	})
	return f
}

func (f *fakeNM) addConn(path dbus.ObjectPath, s nmSettings) {
	f.conns[path] = s
	f.connOrder = append(f.connOrder, path)
}

func (f *fakeNM) Property(_ context.Context, path dbus.ObjectPath, property string) (dbus.Variant, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	v, ok := f.props[path][property]
	if !ok {
		return dbus.Variant{}, fmt.Errorf("no property %s on %s", property, path)
	}
	return dbus.MakeVariant(v), nil
}

func (f *fakeNM) Call(_ context.Context, path dbus.ObjectPath, method string, args []any, out ...any) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.calls = append(f.calls, method)
	reply := func(values ...any) error { return dbus.Store(values, out...) }
	switch method {
	case nmIface + ".GetDevices":
		if f.failGetDevs != nil {
			return f.failGetDevs
		}
		return reply([]dbus.ObjectPath{fakeEthDev, fakeWifiDev})
	case nmWirelessIface + ".GetAllAccessPoints":
		return reply([]dbus.ObjectPath{fakeAPHome24, fakeAPHome5, fakeAPCafe, fakeAPSae, fakeAPCorp})
	case nmWirelessIface + ".RequestScan":
		if f.scanRefused {
			return errors.New("org.freedesktop.NetworkManager.Device.NotAllowed: Scanning not allowed immediately following previous scan")
		}
		f.props[fakeWifiDev][nmWirelessIface+".LastScan"] = int64(2000)
		return nil
	case nmSettingsIface + ".ListConnections":
		return reply(append([]dbus.ObjectPath(nil), f.connOrder...))
	case nmConnIface + ".GetSettings":
		s, ok := f.conns[path]
		if !ok {
			return fmt.Errorf("unknown connection %s", path)
		}
		return reply(copySettings(s, true))
	case nmConnIface + ".Update":
		s := args[0].(nmSettings)
		f.updates[path] = copySettings(s, false)
		f.conns[path] = copySettings(s, true)
		return nil
	case nmConnIface + ".Delete":
		delete(f.conns, path)
		for i, p := range f.connOrder {
			if p == path {
				f.connOrder = append(f.connOrder[:i], f.connOrder[i+1:]...)
				break
			}
		}
		return nil
	case nmIface + ".ActivateConnection":
		f.activated = append(f.activated, args[0].(dbus.ObjectPath))
		return reply(fakeActive)
	case nmIface + ".AddAndActivateConnection", nmIface + ".AddAndActivateConnection2":
		s := args[0].(nmSettings)
		f.added = append(f.added, copySettings(s, false))
		if method == nmIface+".AddAndActivateConnection2" {
			f.addOptions = args[3].(map[string]dbus.Variant)
		}
		f.nextConn++
		p := dbus.ObjectPath(fmt.Sprintf("/org/freedesktop/NetworkManager/Settings/%d", 100+f.nextConn))
		f.addConn(p, copySettings(s, true))
		if method == nmIface+".AddAndActivateConnection2" {
			return reply(p, fakeActive, map[string]dbus.Variant{})
		}
		return reply(p, fakeActive)
	}
	return fmt.Errorf("fakeNM: unexpected call %s on %s", method, path)
}

// copySettings deep-copies; stripSecrets drops the psk like GetSettings does.
func copySettings(s nmSettings, stripSecrets bool) nmSettings {
	out := nmSettings{}
	for k, sec := range s {
		out[k] = map[string]dbus.Variant{}
		for kk, v := range sec {
			if stripSecrets && kk == "psk" {
				continue
			}
			out[k][kk] = v
		}
	}
	return out
}

func (f *fakeNM) called(method string) int {
	f.mu.Lock()
	defer f.mu.Unlock()
	n := 0
	for _, c := range f.calls {
		if strings.HasSuffix(c, "."+method) {
			n++
		}
	}
	return n
}

func newFakeProvider(t *testing.T) (*NetworkWifiProvider, *fakeNM) {
	t.Helper()
	f := newFakeNM()
	return NewNetworkWifiProviderWithBus(f, ""), f
}

func TestWifiStatusReportsActiveLink(t *testing.T) {
	p, _ := newFakeProvider(t)
	st := p.Status(context.Background())
	require.True(t, st.Available)
	assert.Equal(t, "wlan0", st.Interface)
	assert.Equal(t, "activated", st.State)
	assert.True(t, st.Connected)
	assert.Equal(t, "ExampleHome", st.SSID)
	assert.Equal(t, "02:00:00:00:00:02", st.BSSID)
	assert.EqualValues(t, 5180, st.FreqMHz)
	assert.EqualValues(t, 55, st.Strength)
	assert.EqualValues(t, 173000, st.BitrateKbs)
	assert.Equal(t, []string{"192.0.2.10/24"}, st.IP4)
	require.NotNil(t, st.Connection)
	assert.Equal(t, "uuid-home", st.Connection.UUID)
	assert.Equal(t, "enabled", st.Connection.Powersave)
	assert.Equal(t, WifiSecurityWPA2, st.Connection.Security)

	require.Len(t, st.Saved, 2, "the ethernet profile is not a Wi-Fi network")
	assert.True(t, st.Saved[0].Active)
	assert.Equal(t, "ExampleOld", st.Saved[1].SSID)
	assert.Equal(t, "bg", st.Saved[1].Band)
	assert.False(t, st.Saved[1].Autoconnect)
	assert.False(t, st.Saved[1].Active)
	assert.Equal(t, WifiSecurityOpen, st.Saved[1].Security)
}

func TestWifiUnavailableWithoutBus(t *testing.T) {
	p := newNetworkWifiProvider(func() (NMBus, error) { return nil, errors.New("no socket") }, "")
	st := p.Status(context.Background())
	assert.False(t, st.Available)
	assert.Contains(t, st.Reason, "no socket")
	assert.NotNil(t, st.Saved)

	_, err := p.Scan(context.Background(), false)
	assert.ErrorIs(t, err, ErrWifiUnavailable)
	_, err = p.Connect(context.Background(), WifiConnectRequest{SSID: "x", Password: "password1"})
	assert.ErrorIs(t, err, ErrWifiUnavailable)
	assert.ErrorIs(t, p.Forget(context.Background(), "u"), ErrWifiUnavailable)
}

func TestWifiUnavailableWithoutNetworkManager(t *testing.T) {
	p, f := newFakeProvider(t)
	f.failGetDevs = errors.New("org.freedesktop.DBus.Error.ServiceUnknown: The name is not activatable")
	st := p.Status(context.Background())
	assert.False(t, st.Available)
	assert.Contains(t, st.Reason, "NetworkManager")
}

func TestWifiUnavailableWithoutWifiDevice(t *testing.T) {
	f := newFakeNM()
	p := NewNetworkWifiProviderWithBus(f, "wlan9")
	st := p.Status(context.Background())
	assert.False(t, st.Available)
	assert.Contains(t, st.Reason, "wlan9")
}

func TestWifiScanListsStrongestFirstWithSecurity(t *testing.T) {
	p, f := newFakeProvider(t)
	res, err := p.Scan(context.Background(), false)
	require.NoError(t, err)
	assert.False(t, res.Scanned)
	assert.Zero(t, f.called("RequestScan"))
	require.Len(t, res.AccessPoints, 5)
	assert.Equal(t, "ExampleSae", res.AccessPoints[0].SSID)
	assert.Equal(t, WifiSecurityWPA3, res.AccessPoints[0].Security)
	sec := map[string]string{}
	for _, ap := range res.AccessPoints {
		sec[ap.BSSID] = ap.Security
		if ap.BSSID == "02:00:00:00:00:02" {
			assert.True(t, ap.Active)
			assert.True(t, ap.Saved)
		}
	}
	assert.Equal(t, WifiSecurityOpen, sec["02:00:00:00:00:03"])
	assert.Equal(t, WifiSecurityEnterprise, sec["02:00:00:00:00:05"])
	assert.Equal(t, WifiSecurityWPA2, sec["02:00:00:00:00:01"])
}

func TestWifiRescanWaitsForLastScan(t *testing.T) {
	p, f := newFakeProvider(t)
	res, err := p.Scan(context.Background(), true)
	require.NoError(t, err)
	assert.True(t, res.Scanned)
	assert.Equal(t, 1, f.called("RequestScan"))
}

func TestWifiRescanRefusedStillLists(t *testing.T) {
	p, f := newFakeProvider(t)
	f.scanRefused = true
	res, err := p.Scan(context.Background(), true)
	require.NoError(t, err)
	assert.False(t, res.Scanned)
	assert.Len(t, res.AccessPoints, 5)
}

func TestApSecurityClassification(t *testing.T) {
	assert.Equal(t, WifiSecurityWPA2WPA3, apSecurity(1, 0, apSecKeyMgmtPSK|apSecKeyMgmtSAE))
	assert.Equal(t, WifiSecurityWPA, apSecurity(1, apSecKeyMgmtPSK, 0))
	assert.Equal(t, WifiSecurityWEP, apSecurity(1, 0, 0))
	assert.Equal(t, WifiSecurityOWE, apSecurity(0, 0, apSecKeyMgmtOWE))
	assert.Equal(t, WifiSecurityOpen, apSecurity(0, 0, 0))
}

func TestWifiConnectNewNetworkWithPSK(t *testing.T) {
	p, f := newFakeProvider(t)
	// ExampleCafe is open; make a PSK network that is not saved yet.
	f.props[fakeAPCafe][nmAPIface+".RsnFlags"] = uint32(apSecKeyMgmtPSK | apSecKeyMgmtSAE)
	res, err := p.Connect(context.Background(), WifiConnectRequest{SSID: "ExampleCafe", Password: "correct horse"})
	require.NoError(t, err)
	assert.True(t, res.Created)
	assert.Regexp(t, `^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$`, res.UUID)
	require.Len(t, f.added, 1)
	s := f.added[0]
	assert.Equal(t, []byte("ExampleCafe"), s[wifiSettingType]["ssid"].Value())
	assert.Equal(t, "wpa-psk", s[wifiSecuritySetting]["key-mgmt"].Value(), "transition mode joins over wpa-psk")
	assert.Equal(t, "correct horse", s[wifiSecuritySetting]["psk"].Value())
	assert.Equal(t, res.UUID, s["connection"]["uuid"].Value())
	assert.Zero(t, f.called("AddAndActivateConnection2"))
}

func TestWifiConnectSaeOnlyUsesSae(t *testing.T) {
	p, f := newFakeProvider(t)
	_, err := p.Connect(context.Background(), WifiConnectRequest{SSID: "ExampleSae", Password: "longenough"})
	require.NoError(t, err)
	assert.Equal(t, "sae", f.added[0][wifiSecuritySetting]["key-mgmt"].Value())
}

func TestWifiConnectNotRememberedIsInMemory(t *testing.T) {
	p, f := newFakeProvider(t)
	no := false
	_, err := p.Connect(context.Background(), WifiConnectRequest{SSID: "ExampleCafe", Remember: &no})
	require.NoError(t, err)
	assert.Equal(t, 1, f.called("AddAndActivateConnection2"))
	assert.Equal(t, "memory", f.addOptions["persist"].Value())
	_, hasSec := f.added[0][wifiSecuritySetting]
	assert.False(t, hasSec, "open network: no security setting")
}

func TestWifiConnectSavedActivatesWithoutPassword(t *testing.T) {
	p, f := newFakeProvider(t)
	res, err := p.Connect(context.Background(), WifiConnectRequest{SSID: "ExampleHome"})
	require.NoError(t, err)
	assert.False(t, res.Created)
	assert.Equal(t, "uuid-home", res.UUID)
	assert.Equal(t, []dbus.ObjectPath{fakeConnHome}, f.activated)
	assert.Empty(t, f.added)
	assert.Zero(t, f.called("Update"))
}

func TestWifiConnectSavedWithNewPasswordUpdatesPSK(t *testing.T) {
	p, f := newFakeProvider(t)
	_, err := p.Connect(context.Background(), WifiConnectRequest{UUID: "uuid-home", Password: "a-new-passphrase"})
	require.NoError(t, err)
	upd := f.updates[fakeConnHome]
	require.NotNil(t, upd)
	assert.Equal(t, "a-new-passphrase", upd[wifiSecuritySetting]["psk"].Value())
	_, deprecated := upd["ipv4"]["addresses"]
	assert.False(t, deprecated, "deprecated ipv4.addresses is dropped when address-data is present")
	assert.Equal(t, []dbus.ObjectPath{fakeConnHome}, f.activated)
}

func TestWifiConnectValidation(t *testing.T) {
	p, f := newFakeProvider(t)
	ctx := context.Background()
	_, err := p.Connect(ctx, WifiConnectRequest{SSID: "ExampleCafe", Password: "short"})
	assert.ErrorIs(t, err, ErrWifiInvalid)
	_, err = p.Connect(ctx, WifiConnectRequest{})
	assert.ErrorIs(t, err, ErrWifiInvalid)
	_, err = p.Connect(ctx, WifiConnectRequest{SSID: strings.Repeat("x", 33)})
	assert.ErrorIs(t, err, ErrWifiInvalid)
	_, err = p.Connect(ctx, WifiConnectRequest{SSID: "ExampleSae"})
	assert.ErrorIs(t, err, ErrWifiInvalid, "PSK network without a password")
	_, err = p.Connect(ctx, WifiConnectRequest{SSID: "ExampleCafe", Password: "password1"})
	assert.ErrorIs(t, err, ErrWifiInvalid, "open network with a password")
	_, err = p.Connect(ctx, WifiConnectRequest{SSID: "ExampleCorp", Password: "password1"})
	assert.ErrorIs(t, err, ErrWifiInvalid, "enterprise is not supported")
	_, err = p.Connect(ctx, WifiConnectRequest{SSID: "Nowhere", Password: "password1"})
	assert.ErrorIs(t, err, ErrWifiNotFound)
	_, err = p.Connect(ctx, WifiConnectRequest{UUID: "uuid-missing"})
	assert.ErrorIs(t, err, ErrWifiNotFound)
	assert.True(t, validPSK(strings.Repeat("aB3", 21)+"f"), "64 hex digits")
	assert.False(t, validPSK(strings.Repeat("g", 64)))
	assert.Empty(t, f.added)
	assert.Empty(t, f.activated)
}

func TestWifiConnectHiddenNetwork(t *testing.T) {
	p, f := newFakeProvider(t)
	_, err := p.Connect(context.Background(), WifiConnectRequest{SSID: "Nowhere", Password: "password1", Hidden: true})
	require.NoError(t, err)
	assert.Equal(t, true, f.added[0][wifiSettingType]["hidden"].Value())
	assert.Equal(t, "wpa-psk", f.added[0][wifiSecuritySetting]["key-mgmt"].Value())
}

func TestWifiForgetDeletesProfile(t *testing.T) {
	p, f := newFakeProvider(t)
	require.NoError(t, p.Forget(context.Background(), "uuid-old"))
	_, ok := f.conns[fakeConnOld]
	assert.False(t, ok)
	assert.ErrorIs(t, p.Forget(context.Background(), "uuid-old"), ErrWifiNotFound)
	assert.ErrorIs(t, p.Forget(context.Background(), ""), ErrWifiInvalid)
	assert.ErrorIs(t, p.Forget(context.Background(), "uuid-eth"), ErrWifiNotFound, "only Wi-Fi profiles")
}

func TestWifiBandLockActiveReactivates(t *testing.T) {
	p, f := newFakeProvider(t)
	net, err := p.SetBand(context.Background(), "", "bg")
	require.NoError(t, err)
	assert.Equal(t, "bg", net.Band)
	assert.True(t, net.Active)
	assert.Equal(t, "bg", f.updates[fakeConnHome][wifiSettingType]["band"].Value())
	assert.Equal(t, uint32(nmPowersaveEnable), f.updates[fakeConnHome][wifiSettingType]["powersave"].Value(), "other keys survive")
	assert.Equal(t, []dbus.ObjectPath{fakeConnHome}, f.activated)
}

func TestWifiBandAutoClearsBandOnInactive(t *testing.T) {
	p, f := newFakeProvider(t)
	net, err := p.SetBand(context.Background(), "uuid-old", "")
	require.NoError(t, err)
	assert.Equal(t, "", net.Band)
	_, has := f.updates[fakeConnOld][wifiSettingType]["band"]
	assert.False(t, has)
	assert.Empty(t, f.activated, "an inactive profile is not activated")
	_, err = p.SetBand(context.Background(), "", "6ghz")
	assert.ErrorIs(t, err, ErrWifiInvalid)
}

func TestWifiPowersave(t *testing.T) {
	p, f := newFakeProvider(t)
	net, err := p.SetPowersave(context.Background(), "uuid-home", false)
	require.NoError(t, err)
	assert.Equal(t, "disabled", net.Powersave)
	assert.Equal(t, uint32(nmPowersaveDisable), f.updates[fakeConnHome][wifiSettingType]["powersave"].Value())
	assert.Len(t, f.activated, 1)

	net, err = p.SetPowersave(context.Background(), "uuid-old", true)
	require.NoError(t, err)
	assert.Equal(t, "enabled", net.Powersave)
	assert.Equal(t, uint32(nmPowersaveEnable), f.updates[fakeConnOld][wifiSettingType]["powersave"].Value())
	assert.Len(t, f.activated, 1)
}

func TestWifiNoActiveNetworkForImplicitTarget(t *testing.T) {
	p, f := newFakeProvider(t)
	delete(f.props[fakeWifiDev], nmDeviceIface+".ActiveConnection")
	_, err := p.SetPowersave(context.Background(), "", true)
	assert.ErrorIs(t, err, ErrWifiNotFound)
}
