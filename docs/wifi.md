# Wi-Fi: phone link to the GUI

Status: **plan only, not implemented** (needs owner approval: it changes host
networking on the robot).

## Problem

The phone joystick (boundary recording, manual driving) talks to the GUI
(`mower_gui`, host network, :4006) over the house Wi-Fi. At the edge of the
house AP's coverage the phone roams or loses the link. Live logs showed
`read: connection reset by peer` and `websocket: close 1005`. The robot stops
(correct: `cmd_vel_ws_relay` publishes zero after 0.25 s without commands), but
recording stalls until the link is back.

Already done in software (GUI, 2026-10-07):
- The server sends a heartbeat on every GUI WebSocket each second and drops
  silent peers after 5 s (`gui/pkg/api/ws_heartbeat.go`).
- The web client detects a dead or half-open socket after 3 s of silence and
  reconnects after 250 ms, 500 ms, then every 1 s (previously 1 s doubling up
  to 30 s). It also retries at once when the browser goes `online` or becomes
  visible (`web/src/hooks/reconnect.ts`).
- The joystick socket never queues commands while it is down: no stale twists
  are replayed. Driving resumes without a reload, and "Reconnecting…" badges
  are shown on the page and on the stick.

These changes make recovery fast. They cannot fix coverage. A robot-hosted AP
would fix coverage, because the phone walks next to the robot.

## Hardware facts (measured on the robot)

| | |
|---|---|
| Radio | one `phy#0`, driver `wl`. `wlan0` and `wlan1` are two virtual interfaces on the **same radio** (MACs `40:fd:…` / `42:fd:…`) |
| Supported modes | managed, AP, P2P. Combinations: `#{AP} <= 2, #{managed} <= 4, total <= 5, #channels <= 2` |
| `wlan0` | NetworkManager, house SSID, 5 GHz ch 36 / 80 MHz, 192.168.1.105 |
| `wlan1` | DOWN, unmanaged by NetworkManager |
| Regulatory | `country 00` (world, many channels NO-IR). AP beaconing on 5 GHz is likely refused until the country is set |
| Installed | `dnsmasq` binary present (service inactive). `hostapd` **not installed** |
| Also present | Quectel EC200A LTE modem (`usb0`) |
| GUI | `mower_gui` uses host networking, so it already listens on every interface. No container change is needed |

## Plan (on approval)

1. **Regulatory domain**: set the real country (`iw reg set NZ`, persist it via
   `/etc/default/crda` or the `cfg80211 ieee80211_regdom=` module option).
   Without this, the AP is limited to 2.4 GHz ch 1-11 or refused.
2. **Channel choice** (the key constraint, because there is one radio):
   - *Option A, same channel as wlan0 (recommended first try)*: run the AP on
     the STA's current 5 GHz channel. There is no channel switching, so both
     links keep full airtime. The drawback is that the AP must follow wlan0
     when the house AP changes channel (hostapd restarts and phones
     reassociate). A NetworkManager dispatcher hook on wlan0 `up` /
     `dhcp4-change` rewrites `channel=` and restarts hostapd.
   - *Option B, 2.4 GHz ch 1/6/11*: this uses the `#channels <= 2` support, so
     the radio time-shares between channels. It is simpler, but adds tens of ms
     of latency and jitter to both links, including NTRIP corrections on wlan0.
     Measure RTK correction age before accepting it.
3. **hostapd** (`apt install hostapd`, its own unit, not NetworkManager):
   `interface=wlan1`, `ssid=mower-<serial>`, WPA2-PSK (passphrase in
   `/userdata`), `hw_mode=a`/`g` per step 2, `ieee80211n=1`, `wmm_enabled=1`.
   Run `iw dev wlan1 set type __ap` (or `iw phy phy0 interface add`) first.
4. **Address + DHCP**: static `10.42.0.1/24` on wlan1 (systemd-networkd or the
   hostapd unit's `ExecStartPre=ip addr add`), plus `dnsmasq` bound only to
   wlan1 (`interface=wlan1`, `bind-interfaces`,
   `dhcp-range=10.42.0.10,10.42.0.50,12h`). Give no default route: the phone
   keeps using LTE for the internet (Android/iOS warn "no internet": accept
   it). Optionally serve `address=/mower.local/10.42.0.1`.
5. **Isolation**: no forwarding or NAT from wlan1 to wlan0
   (`net.ipv4.ip_forward` stays 0), so the house LAN is not bridged to the
   phone AP. Optionally, iptables can allow only :4006 (GUI) and :8080 (video)
   on wlan1.
6. **GUI**: no change (host network). The phone uses `http://10.42.0.1:4006`.
   The PWA origin differs from `192.168.1.105`, so install it once per origin.
7. **NetworkManager**: wlan1 stays unmanaged (it is already). wlan0 is not
   touched: NTRIP and internet stay on the house Wi-Fi (and on LTE as a
   fallback).
8. **Rollback**: `systemctl disable --now hostapd dnsmasq@wlan1` and
   `ip link set wlan1 down`.

## Risks / open questions

- The `wl` driver's AP+STA concurrency on one radio is unverified. Test it for
  an hour with the robot mowing: watch `wlan0` drops, NTRIP correction age
  (`/gps/status`), and the joystick RTT.
- When wlan0 changes channel, Option A briefly drops the AP. The GUI
  reconnect logic above now covers this.
- Power and heat: an extra beaconing interface costs a little power.
- The AP range only needs to cover the phone walking beside the robot, so the
  TX power can be lowered (`iw dev wlan1 set txpower fixed 1000`) to reduce
  interference with wlan0.
