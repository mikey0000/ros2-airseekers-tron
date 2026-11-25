# Handing the mower over, and reinstalling this stack

How to put a mower that runs this ROS 2 stack back on the vendor (stock) software before it
changes owner, what to remove first, how remote support can work with the new owner's
consent, and how to bring this stack back later. Written 2026-11-25. Nothing here reflashes
the device: the stock software never left it, it is only stopped and disabled while this stack runs.

`scripts/switch_stack.sh` does the software switch in both directions. It is a dry run unless
given `--yes`:

```bash
sudo /userdata/ros2_stack/scripts/switch_stack.sh status        # containers, vendor units, our host pieces
sudo /userdata/ros2_stack/scripts/switch_stack.sh stock         # print the plan
sudo /userdata/ros2_stack/scripts/switch_stack.sh stock --yes   # apply it, then: sudo reboot
```

The `stock` mode does the following:
- It refuses unless the mission is docked or idle.
- It stops `mower_jazzy` and `mower_gui` and keeps them from restarting.
- It runs `install_host_services.sh uninstall`: vendor udev rules, the stereo IMU back in vendor
  netlink mode, vendor socket buffers, and our units removed.
- It re-enables the vendor units that were enabled in the factory snapshot
  (`ros2_port_handoff/07_system_config/systemd/unit_enablement_state.txt`) through
  `mower_services.sh enable --all`.

It leaves the following alone:
- docker;
- the `mowgli-remote` Tailscale container;
- the vendor `tailscaled`, which stays disabled because it would join the vendor's tailnet;
- `mower-webshell`, which stays masked;
- all data on `/userdata`.

## Part A: handover checklist

Work top to bottom. Tick each item off.

### 1. Before switching

- [ ] Copy off anything you want to keep: bags (`deploy_to_mower.sh bags all`), maps
      (`/userdata/ros2/maps`), the GUI database (`/userdata/ros2/gui_db`), and the device yaml
      (`/userdata/ros2_stack/config/gui/mowgli_robot.yaml`, which holds the datum and GUI settings).
- [ ] Decide whether this stack stays installed (stopped) so it can be reloaded remotely later
      (Part B, case 1), or is removed (step 4 below).
- [ ] Mower docked and charging; no mow running.

### 2. Switch to the stock software

- [ ] `switch_stack.sh stock` (dry run), read the plan, then `switch_stack.sh stock --yes`.
- [ ] `sudo reboot`.
- [ ] `switch_stack.sh status`: vendor robot units `active`, `mower_jazzy` and `mower_gui` exited
      with `restart=no`, and `mowgli-remote` running if remote support was agreed (section 5).
- [ ] The vendor app sees the mower online. It undocks, mows a small area and docks.
      Stock needs its own map: the vendor app maps again from scratch.

### 3. Remove the previous owner's data and accounts

- [ ] In the vendor app: unbind or remove the mower from your account (or use the app's factory
      reset), so the new owner can bind it to theirs.
- [ ] Stock data from your use: `/userdata/RobotData/map`, `/userdata/RobotData/bag`,
      `/userdata/RobotData/reports`, `/userdata/RobotData/track.txt`. Delete these if the app
      reset does not.
      Leave `/userdata/RobotData/upgrade` alone: it holds the vendor firmware packages.
- [ ] This stack's data: `/userdata/ros2/` (maps, datum, bags, logs, crashes, incidents,
      `vio_bags`, `gui_db`). It holds images of your garden and the datum position.
      `sudo rm -rf /userdata/ros2/{maps,bags,mows,logs,log,crashes,incidents,vio_bags,gui_db,datum.env}`
      after the copy in step 1.
- [ ] Credentials:
  - the NTRIP account, in the GUI settings (stored in the device yaml) and in
    `/userdata/mower/ntrip.yaml`;
  - your Wi-Fi networks saved on the mower (the vendor app sets up the new owner's).
- [ ] The device yaml `/userdata/ros2_stack/config/gui/mowgli_robot.yaml` if the stack stays
      installed. It holds the datum; a reinstall records a new one.
- [ ] Change the host's login password from the vendor default, and give the new owner the new
      password. The default is the same on every unit and opens a root-capable shell to anyone on
      their network.

### 4. If this stack is removed completely

- [ ] `docker rm -f mower_jazzy mower_gui` and `docker image rm` the stack and GUI images.
- [ ] `sudo rm -rf /userdata/ros2_stack /userdata/ros2`.
- [ ] Keep docker if remote support (section 5) is agreed; the Tailscale container needs it.

### 5. Remote support: only with the new owner's informed consent

`mowgli-remote` is a userspace Tailscale node. Any tailnet member can reach every TCP port on the
mower through it: ssh as root-capable `airseekers`, and the camera streams. That is access into
someone else's home and garden, so:

- [ ] Agree it in writing with the new owner. Say what it reaches (shell, cameras, logs), who can
      use it, and how they switch it off (below).
- [ ] Preferred: put the node on the new owner's own tailnet and have them share it with you. They
      then control the access and can revoke it. Re-authenticate it with their auth key:
      `docker exec mowgli-remote tailscale up --force-reauth`, then open the login URL it prints.
      Then remove the node from your own tailnet in the admin console.
- [ ] If it stays on your tailnet by agreement: disable key expiry for the node in the admin console,
      otherwise it drops off after 180 days. Restrict it with a tailnet ACL to your own devices.
- [ ] If no remote support is agreed: `docker rm -f mowgli-remote && docker volume rm mowgli_remote_state`,
      and remove the node from your tailnet.
- [ ] How the owner turns it off: `docker stop mowgli-remote` on the host (it stays stopped across
      reboots), or ask you to remove it.

The vendor's own `tailscaled` stays disabled in every case: its init script joins the vendor's tailnet.

Keeping the node up to date: `sudo scripts/tailscale_sidecar.sh install` recreates `mowgli-remote` from
`tailscale/tailscale:stable` with the same state volume (same node, name and address) and installs
a daily systemd timer (`tailscale-sidecar-update.timer`, script in `/usr/local/sbin`). It pulls the image
and recreates the container only when the image changed. Tailscale's own `--auto-update` cannot work
inside this container: it has no capabilities and a read-only `/usr/local/bin`. `tailscale_sidecar.sh uninstall`
removes the timer and leaves the container running. It is independent of both stacks, so
`switch_stack.sh` never touches it.

### 6. Physical

- [ ] The RTK base station goes with the mower: its pairing is stored on both ends
      (docs/rtk_pairing.md).
- [ ] Blade and deck cleaned, the blade height reset in the app, and the charging contacts clean.

## Part B: reinstalling this stack

### Case 1: the stack is still on the mower (switched to stock with `switch_stack.sh stock`)

```bash
sudo /userdata/ros2_stack/scripts/switch_stack.sh ros2          # read the plan
sudo /userdata/ros2_stack/scripts/switch_stack.sh ros2 --yes
sudo reboot
```

The `ros2` mode does the following:
- It stops and disables the vendor robot units and `foxglove`. `rosmaster`, LTE, network
  switching, vmstat and the camera firewall stay as they are: from v1.3.33 the vendor network
  manager (`base_net_node`, routes, DNS, Wi-Fi profiles, LTE repair) runs under `rosmaster`, and
  without it LTE has no default route (`v1.3.33/DIFF_v1.3.27.md` in the parent folder).
- It reinstalls our host pieces (`install_host_services.sh`).
- It starts `mower_jazzy` and `mower_gui` with `restart=unless-stopped`.

What the vendor `base_net_node` does while our stack runs (strings of the v1.3.33 binary, its
source is not shipped; `analysis/2026-11-25_stock_v1.3.33_glean.md` section 3):
- It owns the Wi-Fi profile, routes and DNS: `nmcli` profile add/modify/delete
  (`deleteOtherWifiConnections`), a BSSID lock and MAC denylist on its profile, `wl roam_off`,
  `wpa_cli` bssid_ignore, policy routing per interface, `rp_filter=2`, `resolvectl`, and
  restarts of systemd-resolved, NetworkManager (`repair_wifi.sh`) and the modem
  (`repair_lte.py`, GPIO reset) when its pings fail. Expect wlan0/usb0 drops of up to about a
  minute during a repair.
- It only serves `/base_net/*` services; it does not need the other vendor nodes.
- Our side is compatible: `wifi_monitor` only reads (`iw`, `/proc/net/wireless`), the
  `mowgli-remote` Tailscale is userspace with `TS_ACCEPT_DNS=false`, and nothing of ours sets
  routes, rules, sysctls or resolved.
- Do not add Wi-Fi BSSID or band locks with `nmcli con modify` yourself: base_net rewrites
  them. A Wi-Fi network added from the GUI's Settings -> Network may be deleted by base_net's
  "other connections" cleanup (not verified); if it does not survive, add networks with the
  vendor app.
- `mower-lte.service` is disabled by the v1.3.33 install; LTE repair is base_net's job.
- Checks after a switch (by hand, on the mower): `ip rule`, `ip route show table all`,
  `resolvectl status`, `/userdata/log/net_manager.log`, and that `mowgli-remote` reaches the
  tailnet with the Wi-Fi AP off (LTE only).

Then:
1. Check the GUI on port 4006: status, RTK fixed, docked.
2. If `/userdata/ros2` was wiped: set the datum and NTRIP in the GUI, record the areas again, and
   check the dock pose (docs/deployment.md, docs/rtk_pairing.md).
3. A vendor OTA re-enables the vendor units (`/workspace/install.sh`) and wipes the
   `mower-webshell` mask (it deletes and re-copies every `mower*` unit). Run
   `switch_stack.sh ros2 --yes` and `sudo systemctl mask mower-webshell` again after one.

Over remote support: `ssh airseekers@<tailnet node name>` works with the stock software too, so
this can be done remotely, with the owner's agreement and the mower docked.

### Case 2: the stack was removed (or a fresh unit)

From a dev box with this repository (`git submodule update --init`):

1. `SSHPASS=... MOWER_HOST=<ip or tailnet name> scripts/deploy_to_mower.sh sync`
2. `scripts/deploy_to_mower.sh image` builds the arm64 Jazzy image on the mower (long; stack down).
3. `scripts/deploy_to_mower.sh build` runs colcon in the container.
4. The GUI: build from the main checkout (it needs the gitignored `third_party/mowglinext/gui/web/.env`
   for the map token): `ARCH=arm64 ./gui/build.sh`, then `docker save mower-gui:arm64 | ssh ... docker load`.
5. On the mower: `sudo scripts/switch_stack.sh ros2 --yes` (vendor units off, host pieces in,
   containers on), then reboot.
6. GUI: datum, NTRIP, record the areas, check docking (docs/deployment.md, docs/STATUS.md).

Optional pieces stay off by default: VIO (`vio:=true`, docs/vio.md). Coast mode is on by default
(`coast_enabled: false` in mission.yaml turns it off).
