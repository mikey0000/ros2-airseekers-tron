# Map page review (MowgliNext GUI, desktop and phone)

Scope: `third_party/mowglinext/gui/web/src/pages/MapPage.tsx` and `pages/map/components/*`, used
as the main mowing control surface. The review is based on the code, the component tests and the
owner's reports. It was not checked in a browser. Items are ranked by impact. **[done]** means it
ships in this change.

## What the page shows today (per surface)

| Element | Desktop | Phone |
|---|---|---|
| Mission controls | Bottom glass bar: Edit Map, Start (idle), Undock (docked), Home, Emergency On/Off, Mow area, Preview plan (idle), Manual Mow, More | Fixed bottom bar that scrolls sideways (icon-only except "Mow"): Start, Home, Undock, Edit, Mow, Preview, Area settings, Manual, More. Separate red "STOP" square in the corner |
| Stop / fault | `MissionStopControls`, centred at the top | Same |
| sub_state | `MissionSubState` pill, top left, **only when sub_state is non-empty** | Same, in the same corner as the Stop banner |
| Areas list, map offset/rotation, tracked obstacles | Right glass panel (240 px), always visible | Not available. Area settings sits behind the gear icon in the scrolling bar |
| Path tools, dock heading | Edit mode only (left vertical toolbar / dock heading panel) | Edit mode only, icon-only buttons in the scrolling bar |
| Plan preview card | Bottom left, above the toolbar | Top left, 64 px down, over the sub-state pill and Stop banner |
| Joystick | Recording, manual mowing or the Manual toggle; has its own Finish / Cancel / Home | Same |

## Findings, ranked by impact

1. **The owner cannot find "Home" on the phone. [done]** Home was a bare house icon, and the same
   house icon also meant "Stop manual mowing" and "Draw path to dock". Desktop showed Home even when
   already docked, while the phone hid it (`canHome`), so the two surfaces behaved differently. The
   confirm dialogs and joystick said "Return to dock". *Fix:* the button is now **Dock** (FR "Base")
   everywhere, with a dedicated dock/charger icon in both toolbars, the joystick, the
   fault/notice/stopped banners and path-to-dock. On the phone it is a labelled button. Both surfaces
   hide it when already docked (`canHome`). Stop manual mowing now uses a stop icon.
2. **Two different "STOP" buttons on the phone. [done]** The red corner "STOP" latched the
   **emergency** stop (`emergency 1`), while the top banner's "Stop mowing" / "Stop" sends mission
   STOP (8, which keeps the resume cursor). These are different commands under the same word. The
   desktop said "Emergency On" / "Emergency Off", which reads like a status, not an action. *Fix:* the
   corner button is now **E-STOP** (FR "URGENCE"). On desktop the labels are "Emergency stop" and
   "Release emergency stop". "Stop mowing" / "Stop" stay with the mission STOP.
3. **No persistent state on the phone, and overlapping top widgets. [done]** The state pill rendered
   only when sub_state was non-empty, so the owner often saw no state at all. When it did render, it
   sat at `top:12,left:16`, right under the centred Stop/fault banner. On the phone the preview card
   (`top:64`) covered both. *Fix:* there is now one top stack (banner first, then an
   **always-visible "STATE · sub_state" line**). The desktop stack keeps clear of the right panel. The
   phone's compact progress bar sits inside this line, so nothing overlaps the bottom bar. The preview
   card is hidden while live progress is shown.
4. **"Why is it stopped?" had no answer. [done]** The robot often stops for obstacle waits,
   stale-stereo costmap aborts, lift, the stop button or boundary checks. The GUI showed only the
   sub_state text (when present) and never said which camera saw what. *Fix:* the state line has a
   **Why stopped? / Details** button. It opens a popover with the mission reason, the last non-`none`
   `/obstacle_policy` (class, moving/static, distance, bearing, age), the latest detection's
   **camera frame_id** and classes, the stereo-stale costmap warning, and the safety inputs (emergency
   latch, lift, stop button, outside boundary, rain, critical nodes down). The data comes from the new
   `/behavior_tree_node/mow_progress` `why` block. Detections are subscribed only while the popover is
   open.
5. **Area settings, confirmations and editing tools were hard to find or inconsistent. [done]**
   - On desktop, area mow settings opened only by clicking a row in the right-hand list (no
     affordance). There is now an **Area settings** dropdown in the toolbar. On the phone the gear
     button is labelled ("Areas").
   - Undock asked for confirmation. Home did not, even mid-mow. Blade forward/backward in "More" spun
     the blade at once with no prompt. All three now go through one `confirmAction` dialog shape.
     Dock asks only during an active mow phase, and both blade-on entries ask (danger style).
   - The phone bar is grouped as mission controls (Start, Dock, Undock, Mow, Areas, Preview, Manual),
     then editing (Edit), then More. Previously Edit sat between Undock and Mow.
6. **Phone bar scrolls without a hint.** The 9-item bar is `overflowX:auto` with no fade or
   chevron, so items past the right edge (now Manual, Edit and More) can go unnoticed. *Recommend:* a
   right-edge fade/chevron, or a two-row layout on screens taller than 700 px.
7. **"Pause" / "Continue" in More duplicate Stop / Start under other names.** "Pause" sends STOP (8),
   the same as "Stop mowing", and "Continue" sends START. *Recommend:* remove them from More, or
   rename them to "Stop (keep progress)" and "Resume".
8. **Blade commands in More are visible in every state.** Blade forward/backward/off are offered while
   docked or during autonomous mowing, where the mission owns the blade. *Recommend:* show them only
   in MANUAL_MOWING or manual mode. The confirm (item 5) reduces the risk meanwhile.
9. **Map offset/rotation is always in prime space on desktop.** It is a one-time calibration but
   takes the bottom of the right panel on every visit. *Recommend:* collapse it behind a "Map
   alignment" disclosure, or move it to edit mode / Settings.
10. **Path tools only appear after "Edit Map".** The owner looked for them in view mode. On the phone
    they are three unlabeled icons in a scrolling bar. *Recommend:* add a "Paths" entry to More that
    enters edit mode with the path tool armed, and give the edit bar a labelled "Path" dropdown
    (Draw path / Path to dock / Edit path / Connect dock).
    **Partly done:** More now offers **Record area (drive the boundary)** and **Record path (drive
    it)**. A driven path is saved by the mission as a navigation band (0.7 m, "Path N", centreline
    via `set_area_channel`). The joystick overlay says which kind is being recorded. When a path
    recording finishes, the map enters edit mode and opens the path panel on the new path so its
    width can be adjusted (endpoint snapping happens in the GUI path tool on Save). Next step:
    promote both record entries into the edit toolbar's Path group.
11. **Dock heading panel shows whenever edit mode has a dock.** It covers part of the map even while
    drawing areas. *Recommend:* show it only after Place dock or dock selection.
12. **"Mow area" on the phone without the Start sheet** starts the area immediately (no settings
    check, no confirm), while Start opens the sheet. *Recommend:* route Mow area through the Start
    sheet everywhere (desktop already does when area settings are supported).
13. **Reconnecting badges** (joystick link, streams) are small and sit next to controls. A stale map
    can look live. *Recommend:* dim the robot icon and show "last seen Ns ago" when the pose stream
    is over 3 s old.
14. **Touch targets.** The phone bar meets 44 px. The new state line's Why button is 32 px tall on a
    14 px line, which is acceptable as a secondary action but worth raising to 40 px. Desktop
    `Space size=small` buttons are 32 px, fine for a mouse.

## Behaviour by mission state (after this change)

| State | Phone bar | Desktop bar | Top stack |
|---|---|---|---|
| IDLE_DOCKED / CHARGING | Start, Undock, Mow, Areas, Preview, Manual, Edit, More (Dock hidden on IDLE_DOCKED) | same + Emergency stop | state line |
| IDLE (off dock) | Start, **Dock**, Mow, Areas, Preview, ... | same | state line + Why stopped? |
| MOWING / TRANSIT / PLANNING | **Dock** (confirms), Mow, Areas, Manual, Edit, More | same | Stop mowing / Stop banner, state line + Details, progress bar (phone) or card (desktop) |
| RETURNING_HOME etc. | Dock (no confirm) | same | Stop, state line |
| MOWING_INCOMPLETE | Start, Dock, Preview | same | warning banner (Return to dock / Resume), state line |
| Latched faults | E-STOP, Dock (except boundary latch) | same | error banner with Reset, Why stopped? |
| RECORDING | joystick owns Finish / Cancel / Dock | Finish / Cancel | state line |

## Live mow progress (also in this change)

- `mower_mission/mow_progress.py` (ROS-free, unit-tested) reads the FSM without changing it. It is
  published by `mission_node.py` at 1 Hz from the existing periodic thread:
  - `/behavior_tree_node/mow_plan`: latched String JSON
    `{plan_id, area, subpaths:[[[x,y],...],...]}`. These are the drivable sub-paths, republished only
    when the plan changes.
  - `/behavior_tree_node/mow_progress`: latched String JSON
    `{plan_id, area, state, sub_state, sub_path, sub_paths, pose_index, total_poses,
    mowed_segments, current_segment, skipped, blade_on, percent, mowed_m, remaining_m,
    elapsed_s, eta_s, why}`. Indices are absolute into the concatenated sub-paths, the same units
    as the resume cursor.
- Go provider topics are `missionPlan` and `missionProgress` (unthrottled; both are small or rare).
- The map draws mowed stretches in solid deep green, the current sub-path in cyan, remaining
  stretches in the preview mint, and skipped stretches (detour skips and failed sub-paths) in
  orange. It also draws the robot's actual track over the last 5 minutes as a thin dotted line. The
  plain coverage plan is suppressed while the live overlay is drawn. The existing map-server
  mowed-mask raster (`/map_server_node/mow_progress`) still renders underneath, which shows the
  real blade footprint.
- The desktop progress card shows percent, sub-path n/m, elapsed time, ETA (from the session's
  mowing rate after 60 s), length left, skipped count and a legend. The phone shows a compact bar in
  the state line.
