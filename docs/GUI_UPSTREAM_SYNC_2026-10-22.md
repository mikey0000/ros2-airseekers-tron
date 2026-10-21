# GUI upstream sync check, 2026-10-22

What changed in upstream MowgliNext since our Airseekers work, what we can pull in, and a trial
merge. This is an addendum to `GUI_REBASE_ASSESSMENT.md` (2026-10-13), which covered the
vendored GUI's base `faf658bc` up to upstream `dev` at `36ce7f05`.

## 0. Which GUI the mower runs

There are two GUI trees, and only one of them is on the mower:

| Tree | Base | Role |
|---|---|---|
| `ros2_stack/third_party/mowglinext/gui` (vendored fork) | `faf658bc` (main, 2026-09-15) plus about 315 files of Airseekers delta | **This is what `mower_gui` runs.** `gui/build.sh` builds it (`CTX=third_party/mowglinext/gui`). |
| `upstream-pr/mowglinext-airseekers`, `feat/airseekers-backend` (PR #986, draft) | upstream `dev` at `ac6a08ec` (2026-10-13), plus 10 Airseekers commits | This is the upstream sidecar backend: `sensors/airseekers`, `HARDWARE_BACKEND=airseekers`, and its GUI wiring. It runs upstream's own ROS stack, so the mower does not use it today. |

The task brief described `feat/airseekers-backend` as the GUI on the mower. It is not. The
trial merge below is on that branch. Section 5 covers what carries over to the vendored GUI.

## 1. Inventory

- `origin/main` = `345dac46`. Since the brief's merge-base (`61c3faca`, 2026-10-05), main has
  only 3 real commits:
  - `caec4dff` fix(gui): debounce joystick release, so that a spurious stop does not brake the mower;
  - `749bc81f` fix(hardware): dead-IMU log timestamp uses ROS time;
  - `3f8ad76d` docs: landing page.
- `origin/dev` = `708ef0f4` (2026-10-19) is where the work happens. `feat/airseekers-backend`
  forked from dev at `ac6a08ec`. Since then upstream has 127 non-merge commits:
  - 42 touch `gui/web`;
  - 16 touch `gui/pkg`;
  - 64 touch `ros2/src`;
  - 17 touch `install`;
  - 24 touch CI.
- On our side, `feat/airseekers-backend` has 10 commits that are not in dev: the airseekers sidecar,
  backend wiring, docs, and `undock_forward`.
- Open upstream branches that are not merged yet and are worth watching:
  - `fix/gui-warnings-errors` (+80): monotonic diagnostic freshness, GNSS source identity;
  - `feat/browser-gamepad-part1`;
  - `fix/1006-empty-grid-deltas`;
  - `fix/critical-battery-dock-recovery`;
  - `feat/924-motion-authorization`.

### Groups (`ac6a08ec..origin/dev`, plus main)

| Group | Commits (examples) | Class |
|---|---|---|
| **MAVROS / Pixhawk backend, FCU tools.** Sidecar, FCU feature configurator, `.param` compatibility tool, MAVROS firmware card, drive tuning by odometry, CycloneDDS-mavros, Universal GNSS/MAVROS pins | ~50: `c70c26f2`, `45e89498`, `b11df0b1`, `39e99bd9`, `6079d67c`, `d4ebcc74` | (c) We have no FCU. Only the settings-page plumbing touches us (see conflicts). |
| **Hardware-backend plumbing in settings.** `HardwareBackendInfo` with `parameterRoutes`/`runtimeRouting`, `HardwareViewSwitcher`, `HardwareBackendCard`, backend-aware drive section | `d4ebcc74`, `45e89498`, `5bcb3e9d` | (b) This is the main conflict area with our airseekers wiring. It is resolved in the trial merge. |
| **Settings typing.** `settings.generated.ts` is generated from `mower_config.schema.json`, with a drift test | `5bcb3e9d`, `783ffc5d`, `230de3a4`, `7f2681f0` | (a) Useful. Our schema keys need a regeneration (done). |
| **GUI mower artwork.** WORX preset, preset geometry upgrades, health card that shows the configured mower and dock, and a **Tron-inspired shell with a paintable cover** | `1d73de04`, `1ebc4840`, `99563cd9`, `8404700a`, `a468b3ac` | (a) Cosmetic. The Tron shell is nice for the owner. |
| **Motor telemetry.** Unified wheel ESC / motor telemetry across backends | `35986cd0`, `2ba3897c` | (a) on the upstream branch. Our vendored GUI has its own Tron telemetry. |
| **Blackbox recorder.** A passive RAM recorder in the GUI backend (`gui/pkg/blackbox`, `BlackboxPanel`) | `57dd108f`, `2d3c52cb`, `de460665`, `91f412e9` | (a) Useful for incident capture. It uses about 0 CPU at rest and bounded RAM. Check RAM on the 4 GB mower before enabling it. |
| **Map editor.** "Switch the mowing lines off from their own panel" | `8468f70b` | (a) It is small, and it only makes sense together with the mowing-lines preview, which predates `ac6a08ec` (see section 5). |
| **Dig/obstruction.** The operator can resume from a `DIG_OBSTRUCTION` hold (GUI banner and BT) | `eb22905f` | (c) for us. We have no dig escalation. |
| **External rain verdict over MQTT** (Home Assistant / weather station), Rain settings page | `59d59e80`, `e7d843f5`, `b23e3fc4` | (b) The idea is useful. The ROS side lives in `mowgli_behavior` and `mqtt_bridge_node`, which we replaced, so the stack would need its own subscriber. |
| **Coverage and behaviour fixes in ROS2 `mowgli_*`.** Blade-readiness gating, resume cursor, headlands before interior, no dead-end tongues, sharp-corner pivots, deterministic replay harness | ~45: `83638e6c`, `a6400e55`, `f173ed42`, `500374f2`, `cd08c596`, `d42a93e7` | (c) for the mower: we run our own `mower_coverage` and `mower_mission`. Read them for ideas: headlands-first and dead-end-tongue pruning are relevant to our planner. |
| **Updater.** Read the OpenMower backend from `docker/.env` | `da1494f1` | (c) for the mower; we don't use the upstream updater. Merged on the PR branch. |
| **Installer, CI, docker.** GNSS source selection, MAVROS compose, ROS apt auth, submodule pins | ~40 | (c) |
| **Firmware (STM32 / YardForce)** | `708ef0f4` and others | (c) |
| **main: joystick release debounce** | `caec4dff` | **(a)** This is a safety and comfort fix for manual drive. It applies cleanly to the vendored GUI (section 5). |
| **main: IMU dead-log timestamp, docs site** | `749bc81f`, `3f8ad76d` | (c) |

### The areas the brief asked about

- **Row spacing, blade radius and blade offsets.** `tool_width`, `blade_radius`,
  `blade_x_offset` and `blade_y_offset` are schema keys upstream. The hardware measurement
  editor ("offset cutting tools", `20266108`, before `ac6a08ec`) edits the offsets. Upstream's ROS
  uses the offsets only in the URDF/TF (`mowgli.launch.py` → xacro). Its coverage planner does
  not use them. Nothing new arrived since `ac6a08ec`. Our planner binding stays our job,
  through the vendored GUI's `param_bindings.go`.
- **Per-area settings.** Upstream `MapArea` has `has_mow_angle`/`mow_angle_deg`, perimeter
  direction and a route start point (`b9c22332`, `6170282a`, before `ac6a08ec`). There is
  **no `edge_mode` (hard/soft)** upstream. That stays ours, through
  `map_server_node/set_area_settings`.
- **Map editing.** Nothing substantial is new since `ac6a08ec`, only `8468f70b`. Between
  `faf658bc` and `ac6a08ec`, upstream added these, and the vendored GUI has none of them:
  - map backup before every edit, with restore of the last 20 (`8baccda4`, `#887`), and
    validation before restore;
  - per-area mow angle and start marker;
  - the "mowing lines" preview from the real planner (`82abcba3`), which needs the
    `PreviewCoverage` and `SetAreaCoverageLines` services;
  - collapsible sidebar sections (`965a2239`);
  - map zoom to level 24 (`5776ce8b`).
- **Map-save API.** Upstream is **not atomic either**. `gui/pkg/api/map_replace.go` still does
  `clear_map` → `add_area` ×N → `save_areas`, because `map_server_node` has no replace service.
  Since `#771` (2026-09-26) and `#655` it hardens that sequence in three ways:
  - a process-wide `mapReplacementLock`, so concurrent saves are serialized;
  - a snapshot of the current map read back first (`get_mowing_area` ×N), and restored if any
    later step fails;
  - a deadline for each call, with step-specific errors.

  None of this is in the vendored GUI: `mowglinext.go` there has the bare clear-then-add
  sequence. Our `mower_map/map_server_node.py` already serves `get_mowing_area`, so the
  upstream code is contract-compatible. A real fix for the mid-save window, where mission or
  planner readers see a partial map, still needs a `replace_areas` service on our map server.
  That would be our own design and a candidate to upstream.
- **Schedule, mowing progress, coverage display.** Nothing new since `ac6a08ec`. Earlier:
  a coverage plan preview, never shown above 100 % planned (`ffadf027`).
- **Obstacle display and tagging.** Nothing new since `ac6a08ec`. Fleet peers on the map
  came earlier and are not relevant.
- **Docking UI.** Nothing new. The charge-hold resume status (`c3d63ed2`) predates `ac6a08ec`.
  `fix/critical-battery-dock-recovery` is still open.
- **i18n.** en and fr only. Strings arrive with their features; there are no new locales.
- **Security.** No security-tagged commits in `faf658bc..dev`.

## 2. Trial merge

- Worktree: `upstream-pr/mowglinext-airseekers-wt/upstream-sync`.
- Branch: `wip/upstream-sync`, made from `feat/airseekers-backend` @ `80ce09de`. The main
  checkout was not touched, and nothing was pushed.
- Commits:
  1. `b30873e9`: merge `origin/dev` (`708ef0f4`), with 8 conflicting files resolved;
  2. `6461f2fa`: merge `origin/main`. `docs/index.html` conflicted; the dev side was kept;
  3. `af8aae4d`: post-merge fixes for semantic conflicts (listed below).

### Textual conflicts and how they were resolved

| File | Resolution |
|---|---|
| `gui/web/src/pages/SettingsPage.tsx` | Took dev's `HardwareViewSwitcher`/`HardwareBackendCard` layout and added `AIRSEEKERS_WIRING_GROUP` to the wiring cards. In the drive section, `openmower` **or `airseekers`** renders `OPENMOWER_WHEEL_LOOP_GROUP` + `AIRSEEKERS_DRIVE_GROUP`. MAVROS and Mowgli use dev's `DriveMotorSection`. |
| `gui/pkg/api/settings_backend.go` | Kept our generalized `sidecarRuntimeEnvKeys`/`applySidecarRuntimeFallbacks`, which `settings.go` and the field-group test call. Dev had only renamed comments. |
| `gui/pkg/api/settings_backend_test.go` | The supported list includes `airseekers`. |
| `gui/web/src/i18n/locales/{en,fr}.json` | The text conflict was a diff misalignment. Resolved with a structural 3-way JSON merge (base `ac6a08ec`): dev key order, plus our `airseekers` keys. No value conflicts. |
| `install/lib/state.sh` | Dev's list (adds `MAVROS_GPS1_CANONICAL`) plus `AIRSEEKERS_IMAGE`. |
| `install/tests/test_env_output.sh` | Kept ours (`OPENMOWER_IMAGE`, `AIRSEEKERS_IMAGE`). |
| `docker/README.md` | Dev's MAVROS image line, plus our OpenMower and Airseekers rows. |
| `docs/index.html` (from main) | Kept the dev side. |

### Semantic conflicts that compiled or merged clean but were wrong (`af8aae4d`)

1. `hardwareBackends.ts settingsSectionsForBackend` has an exhaustive switch with no
   `airseekers` case, so `tsc` failed with TS2366. Added the case.
2. `hardwareParameterRoutes` had no `airseekers` entry, so `parameter_routes` would serialize
   as JSON `null`. Added an explicit empty map.
3. `updater.FirmwareProtocolExempt` only waived `openmower`. The Tron runs vendor firmware
   too, so the STM32 `firmware_protocol` check would block upstream updates on an airseekers
   robot. Added `airseekers`, with a test row. This bug was already on `feat/airseekers-backend`
   before the merge.
4. `settings.generated.ts` was regenerated for our schema keys (`undock_forward`,
   `airseekers_*`). Without it the drift test fails.

### Build and test status (in the worktree)

| Check | Result |
|---|---|
| Go `go build ./...`, `go test ./...` (golang:1.25 container, repository mounted) | **Pass.** 10 packages ok. |
| Web `yarn install --frozen-lockfile`, `npm run build` (`tsc && vite build`) | **Pass** |
| Web `npm test` (appearance registry + vitest) | **Pass.** 146 files, 1136 tests, after the regeneration. Before it: 2 failures in `settings-types.test.ts`. |
| Web `npm run lint` | **Pass.** 0 errors, 535 warnings (under the 900 cap) |
| ROS2 / colcon, installer shell tests, e2e Playwright | Not run |
| Visual check against the live mower | Skipped. The mower runs the vendored GUI, not this branch, so a vite check of this tree against the mower's API would mislead. |

`gofmt -l` reports about 20 untouched upstream files, which is upstream noise. The files changed
here are gofmt-clean.

## 3. Conflicts left

None in `wip/upstream-sync`. Before `feat/airseekers-backend` is fast-forwarded to it, these
are still open:

- the PR #986 diff against `dev` grows with merge commits. Upstream may prefer a rebase.
  That is the owner's call.
- the ROS2 side (`mowgli_behavior` with `undock_forward` versus dev's blade-readiness and
  resume changes in `coverage_nodes.cpp` and `behavior_tree_node.cpp`) merged textually but
  was not built or tested with colcon here.

## 4. Recommendations

1. **PR #986 branch.** Adopt `wip/upstream-sync`, or redo it as a rebase. Then run colcon
   build/test for `mowgli_behavior` and `mowgli_bringup`, and the airseekers sim harness.
   Commit `af8aae4d` belongs in the PR whichever way the owner goes. Item 3, the firmware
   exemption, is a real bug fix.
2. **Vendored GUI (what the mower runs).** Pull these in, cheapest first:
   - **Joystick release debounce** (`caec4dff`, 2 files). A `git merge-file` 3-way merge into
     `third_party/mowglinext/gui/web/src/pages/map/hooks/useManualMode{,.test}.ts` applies
     **clean**. Effort: XS.
   - **Map-replace hardening** (`map_replace.go` + `map_replace_test.go`, from `#771`/`#655`/
     `31974eea`). A 3-way merge of `mowglinext.go` against the vendored copy leaves 6 conflict
     hunks, because the fork changed the map routes. Effort: about half a day. It fixes the
     half-garden-after-failed-save case and serializes concurrent saves. It does **not** fix
     the mid-save window. For that, add `replace_areas` (one locked swap plus a save) to
     `mower_map/map_server_node.py` and call it from the GUI, falling back to the hardened
     sequence. Propose that upstream as well.
   - **Map backup before edit + restore** (`#887`, `8baccda4`). Effort: M.
   - **Settings types generated from the schema** (`5bcb3e9d`). Effort: S to M. It would catch
     `param_bindings` key drift.
   - The Tron shell artwork (`1d73de04`) is optional and cosmetic.
3. **Owner decisions.**
   - Re-fork the vendored GUI on `dev` (per `GUI_REBASE_ASSESSMENT.md`), or keep
     cherry-picking? Upstream added about 50 more GUI commits between 2026-10-13 and 2026-10-19.
   - Should `edge_mode` (hard/soft) and the blade-offset planner binding be proposed upstream
     as `MapArea` fields and schema bindings?
   - Should the blackbox recorder go on the 4 GB mower?
   - Should external rain over MQTT get a stack-side subscriber?
   - Merge or rebase for PR #986.

## 5. Deploying the GUI container (`ros2_stack/docs/deployment.md` §2, `docs/gui.md`)

The owner runs these commands from `ros2_stack/`. They build the **vendored** GUI:

```bash
ARCH=arm64 ./gui/build.sh                      # -> mower-gui:arm64 (no QEMU needed)
docker save mower-gui:arm64 | gzip | sshpass -e ssh $MOWER_USER@$MOWER_HOST 'gunzip | docker load'
sshpass -e ssh $MOWER_USER@$MOWER_HOST 'cd /userdata/ros2_stack && docker compose \
  -f docker/docker-compose.yml -f docker/docker-compose.gui.yml up -d --force-recreate mower_gui'
```

Hard-refresh the browser (Ctrl+Shift+R) afterwards. `config/gui/mowgli_robot.yaml` belongs to
the device, and `sync` does not overwrite it. The `feat/airseekers-backend` GUI is not deployed
this way. It ships as upstream's `GUI_IMAGE` (`mowglinext-gui`) with the upstream stack.

## 6. Execution, 2026-10-22 (after the owner's decisions)

Owner decisions: rebase the vendored GUI and PR #986 onto `dev`; no upstream PRs yet for
`edge_mode`, the blade binding or `replace_areas`; blackbox recorder out (4 GB RAM); yes to
rain over MQTT. Nothing was pushed or deployed.

### 6.1 PR #986: `feat/airseekers-backend-rebased`

- Worktree `upstream-pr/mowglinext-airseekers-wt/rebased`. It holds the same 10 commits,
  rebased onto `origin/dev` @ `b1d4a532` (2026-10-22). `feat/airseekers-backend` itself was
  not touched.
- Only "Wire HARDWARE_BACKEND=airseekers…" conflicted. It hit the same 8 files as the trial merge,
  and dev had not touched them since `708ef0f4`, so the trial resolutions were reused.
- The `af8aae4d` fixes are folded in, not added as a merge or fixup commit:
  - the `settingsSectionsForBackend` case, the empty `parameterRoutes` map, the
    `FirmwareProtocolExempt` airseekers row and the airseekers keys in `settings.generated.ts`
    went into the wiring commit;
  - the `undock_forward?` line went into the `undock_forward` commit.

  Every commit keeps the generated types in step with the schema. The wiring commit alone
  passes `tsc` and the settings-types drift test.
- The final tree's diff against dev equals the trial merge's diff, for every file we touch.
- Results:
  - `go build`, `go test ./...` (whole repo mounted): pass;
  - `tsc && vite build`: pass;
  - vitest: 146 files, 1134 tests, pass;
  - lint: 0 errors, 535 warnings.
  - **Not re-run:** colcon (no ROS image locally) for `mowgli_behavior`/`mowgli_bringup`.
    Their changes applied without conflicts over dev's BT changes.
- Push (owner): `git push --force-with-lease fork feat/airseekers-backend-rebased:feat/airseekers-backend`.
- Suggested PR description: `upstream-pr/PR_986_BODY_rebased.md`. It adds a rebase note, the
  dev-plumbing fixes and the firmware-exemption bullet, updated test numbers, and states that
  colcon was not re-run. It was not posted.

### 6.2 Vendored GUI: `ws/gui-dev`. The full re-fork onto `dev` is **blocked**, not done

**How the GUI is vendored.** It is a plain copy, not a subtree or submodule. The base is
recorded in `third_party/mowglinext/UPSTREAM_COMMIT` (`faf658bc`). `gui/build.sh` stamps the
image with that SHA. The fork history is about 50 monolithic stack+GUI commits, so there is
nothing to `git rebase`.

The 3-way simulation was re-run (base `faf658bc`, ours `6117732`, theirs `origin/dev`
`b1d4a532`):

- **61 conflicting files, 185 hunks.** Worst: `MapPage.tsx` 20, en/fr 12 each,
  `MapToolbarMobile`/`MowgliNextPage` 10 each.
- 3 paths were added on both sides.
- dev added 406 files. Many of them need dev's `mowgli_interfaces` (new `MapArea` fields,
  `PreviewCoverage`, `SetAreaCoverageLines`, …).

**Hard gate (`GUI_REBASE_TASKS.md` 1.1/1.2), still open.** Our `mowgli_interfaces/MapArea.msg`
has no `id`, on master and on `ws/edge-layout2`. dev's GUI keys area selection, map edits and
per-area schedules on `MapArea.id`. With every id 0 they resolve to the first area, which is a
**wrong-area-mowing** risk.

A dev-based GUI must not reach the mower until `mower_map` publishes stable, persisted ids.
So instead of a half-merged GUI, this branch carries the approved pieces onto the current
vendored base. `UPSTREAM_COMMIT` stays `faf658bc`.

| Commit | What |
|---|---|
| `58a20d5` | Joystick release debounce (upstream `caec4dff`). `git merge-file` 3-way merge, clean. |
| `86e9fc0` | **Map save.** First the atomic `/map_server_node/replace_areas` (`mower_interfaces/srv/ReplaceAreas`, from `ws/edge-layout2`), with index-aligned `is_navigation_area` and `channel_json` and `settings_json` = "keep". A refusal or transport error is final and never falls back to clear/add. When the service is **not advertised** (the map server predates edge-layout2), it falls back to upstream's hardened sequence (`map_replace.go`): a process-wide lock, a snapshot read-back, restore on failure, a deadline per call and step-specific errors. The snapshot also reads `get_area_channel`, so drawn paths survive a rollback, which upstream lacks. Area settings are keyed by name and survive `clear_map`. `foxglove.CallService` now wraps `ErrServiceNotAdvertised`. 12 new Go tests use a stateful fake map server; the importer and channel tests were updated. |
| `7d59d35` | Rain over MQTT (below). |

**Blackbox recorder.** It is absent: it came after `faf658bc`, and nothing was pulled from it.
A future re-fork must leave out `gui/pkg/blackbox` and `BlackboxPanel`, or default them off.

### 6.3 Rain over MQTT (`7d59d35`)

- **Upstream signal** (`59d59e80`, `docs/MQTT_CONTROL.md`): `<mqtt_topic_prefix>/external_rain`,
  plain string, case-insensitive. Retained messages are accepted.
  - `yes|on|true|rain|wet|1` means wet;
  - `no|off|false|dry|0` means dry;
  - anything else is ignored.

  Upstream's ROS side is `mqtt_bridge_node` → `/external_rain` (`std_msgs/Bool`,
  transient_local) → the BT, OR-ed with the mower's sensor, with a 120 min wet expiry and the
  `use_external_rain` switch.
- **Ours.** The MQTT broker is the GUI backend's embedded mochi server (`system.mqtt.*`,
  default prefix `/gui`, port `:1883`).
  - `pkg/providers/mqtt_external_rain.go` subscribes to `<prefix>/external_rain`, parses the
    payload with the same word list, and calls `/behavior_tree_node/set_external_rain`
    (`std_srvs/SetBool`). Garbage never calls the service.
  - `mower_mission` serves that service and also subscribes to upstream's `/external_rain`
    Bool.
  - `external_rain.py` holds the verdict and its expiry. `MissionNode._refresh_rain` sets the
    FSM `rain` input to the mower's own sensors (`/hardware_bridge/status`, `/mower_base/status`)
    OR a fresh wet verdict. It runs on every input and on every tick, so expiry needs no new
    message.

  The existing `rain_mode` / `rain_debounce_s` / `rain_delay_minutes` handling is unchanged: dock
  and wait, the preflight refuses to start in rain, and the mower resumes after the dry delay. It
  can only add a reason to stop. Parameters are in `mission.yaml`: `use_external_rain: true` and
  `external_rain_max_age_minutes: 120.0` (0 = never). There is no GUI settings binding yet.
- **To use it.** MQTT is off in `docker/docker-compose.gui.yml` (`MQTT_ENABLED=false`), and the
  embedded broker has **no authentication**. Enabling it, and choosing `MQTT_PREFIX` (Home
  Assistant's example uses `mowgli`), is the owner's call. Have the automation re-publish more
  often than the expiry.

### 6.4 Build and test status (`ws/gui-dev`)

| Check | Result |
|---|---|
| Go `go build ./...`; `go test` of `pkg/providers`, `pkg/foxglove`, the map/import tests | Pass |
| Go `go test ./pkg/api ./pkg/updater` | 11 failures, **identical on master `6117732`**: camera tests, schema/template parity (paths to `../ros2/...` and `install/` that the vendored layout lacks), param-binding drift. None comes from these commits. |
| Web `tsc && vite build` | Pass |
| vitest | 888/889. `FlashBoardComponent` "YardForce500B" fails only in the full run and passes alone (flaky, untouched). |
| Web lint | 12 errors, 946 warnings, **identical on master**, all in untouched files |
| `mower_mission` pytest (Jazzy dev image, fresh `mower_interfaces`/`mowgli_interfaces` build) | 608 passed, including 32 new external-rain tests |
| `ARCH=amd64` / `ARCH=arm64 ./gui/build.sh` | Both built (tags `mower-gui:gui-dev-amd64` and `mower-gui:gui-dev-arm64`) |
| Live-mower visual check | Not done. No map save was attempted, by rule. |

### 6.5 Deploy (owner, mower idle on the dock; `docs/deployment.md`)

From the `ws/gui-dev` tree (or master once it is merged):

```bash
./scripts/deploy_to_mower.sh sync        # mower_mission is Python: sync + restart is enough
sshpass -e ssh $MOWER_USER@$MOWER_HOST 'docker restart mower_jazzy'
ARCH=arm64 ./gui/build.sh                 # §2 -> mower-gui:arm64
docker save mower-gui:arm64 | gzip | sshpass -e ssh $MOWER_USER@$MOWER_HOST 'gunzip | docker load'
sshpass -e ssh $MOWER_USER@$MOWER_HOST 'cd /userdata/ros2_stack && docker compose \
  -f docker/docker-compose.yml -f docker/docker-compose.gui.yml up -d --force-recreate mower_gui'
```

Before `ws/edge-layout2` is deployed, map saves use the hardened fallback. After it is deployed,
they use `replace_areas` automatically.

### 6.6 Next steps for the re-fork

1. `GUI_REBASE_TASKS.md` 1.1/1.2: sync `mowgli_interfaces` to dev and make `mower_map` publish
   stable, persisted `MapArea.id`. This is the gate.
2. Then re-fork per `GUI_REBASE_TASKS.md` Phase 4 on the then-current dev. Leave out the
   blackbox. Carry this branch's three commits as patches: `map_replace.go` needs dev's
   `ProposedObstacles` normalisation, and `mqtt.go` needs to be checked against dev's MQTT
   provider.
