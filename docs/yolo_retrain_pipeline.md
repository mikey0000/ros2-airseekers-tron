# YOLO retrain pipeline — capture on the mower, local LLM labelling, NPU handoff

Status: **design only**. Nothing in this document has been run against the mower. Every
hardware/config fact below is cited from the read-only survey material; every *new* artefact
(the capture node, the label client, the converter, the QA harness) is described but does not
exist yet. Section §13 lists what must be confirmed on hardware before the numbers here are
trustworthy.

Scope: replace the **weights** of the vendor's `det_ros` YOLOv8 detector with a model trained
on frames captured from this robot, with the labelling step done by a **local, LAN-hosted
OpenAI-compatible vision LLM** (Ollama by default). Nothing leaves the mower's network.

## 0. Pipeline at a glance

```
 ┌─ ON THE MOWER (RK3588S, 20.04, Noetic) ───────────────────────────────────────────┐
 │  rear_camera  /dev/rear_camera  (video62, USB UVC 32e6:9221, 1920x1080@30)       │
 │  OA cams      /dev/left_oa_camera (video53), /dev/right_oa_camera (video44)       │
 │                uyvy 1920x1080@15, gated by /<cam>/start_capture                   │
 │  Metoak stereo /dev/videoSimor (video11), /dev/videoIsp (video22), MIPI-CSI      │
 │        │                                                                      │
 │        │ ROS 1 image topics (or direct V4L2, or MJPEG :8181)                    │
 │        ▼                                                                      │
 │  capture_node (ROS 2 / rclpy, Humble container)  ── writes ──▶ /userdata/frames/│
 │        frames/<cam>/<seq>_<stamp>.jpg  +  manifest.jsonl                        │
 │  disk guard (quota, rotation, near-duplicate drop)                              │
 └──────────────────────────────────┬───────────────────────────────────────────────┘
                                    │  LAN (scp / sshfs / NFS)
 ┌──────────────────────────────────▼───────────────────────────────────────────────┐
 │  LAN HOST  ── dataset assembly ──▶ Ultralytics YOLOv8n/s train ──▶ fp32/onnx    │
 │               │                                            │                     │
 │               │                                            ▼                     │
 │               │                            airockchip/ultralytics_yolov8 export    │
 │               │                                            │  9-output head      │
 │               │                                            ▼                     │
 │               │                            rknn-toolkit2 2.3.0 convert (int8)    │
 │               │                                            │                     │
 │               └── label_suggest (LLM) ──▶ labels/*.txt ◀─────┘  best_*.rknn     │
 │                    Ollama /v1/chat/completions, vision model,                  │
 │                    strict JSON bboxes, class vocabulary locked to the 22        │
 └──────────────────────────────────┬───────────────────────────────────────────────┘
                                    │ scp best_large_0208_new.rknn
 ┌──────────────────────────────────▼───────────────────────────────────────────────┐
 │  QA GATE (fp32 vs int8 parity, mAP, FP negatives, latency)  ── pass? ──▶ deploy│
 │  det_ros/model/ + param/cfg.yaml, restart mower-perception.service, rollback   │
 └──────────────────────────────────────────────────────────────────────────────────┘
```

## 1. Constraints that shape the design

These are the facts that make this pipeline awkward in specific ways. Everything later is a
consequence of one of them.

| Constraint | Value | Source | Consequence |
|---|---|---|---|
| Detector is a **closed binary** with hard-coded post-processing | `det_ros`, `libdet.so`: `init_yolov8_model` / `inference_yolov8_model` / `RunRKnnYolov8_2` | `mower_docs/03-yolo-perception.md`, `native_decompile/sym/libdet.so.symbols.txt` | Class list, class **order** and input geometry are ABI. Retraining ≠ swapping a `.pt`. |
| Model I/O contract (read out of the shipped `.rknn` metadata) | input `images` int8 NHWC **1×3×640×480**; 9 outputs: boxes `[1,64,S]`, scores, score-sum at S = 80×60, 40×30, 20×15; 22 score channels | `strings ros2_port_handoff/11_perception_models/best_large_0208.rknn` | Export must be the Rockchip-fork 9-output variant at 640×480, not stock Ultralytics ONNX. |
| RKNN compiler vs runtime | models compiled with **rknn-toolkit2 2.3.0**; survey reports device runtime **librknnrt 2.1.0** | `mower_docs/03-yolo-perception.md`; tree ships **two** `librknnrt.so`: `rknn2_runtime/librknn_api/aarch64` = 2.2.0, `rknn2_runtime/3rdparty/rknpu2/Linux/aarch64` = 2.1.0 | Match the compiler to whatever `det_ros` actually loads. Verify before converting (§6.3). |
| NPU headroom | 3 cores, ~6 TOPS; sampled Core0 25–30 %, Core1/Core2 ~0 % | `mower_docs/03-yolo-perception.md` | Training on-device is pointless; conversion + deploy + in-place QA is fine. Core 2 is free for an own node. |
| RAM | 3.8 GB total, **2.0 GB available** with the full stack running, no swap | `ros2_port_handoff/09_platform/meminfo.txt` | A 7B vision LLM does **not** fit on the mower. LAN-hosted by default. |
| CPU already loaded | `stereo_ros` ~83 %, `det_ros` ~70 %, `seg_ros` ~29 % of one core | `mower_docs/01-system-overview.md` | Capture must be throttled (≈1–2 Hz per camera) and prefer JPEG frames straight off the wire. |
| Disk | `/userdata` 32 GB, **24 GB free**; `/` 16 GB, 7.6 GB free | `ros2_port_handoff/09_platform/df.txt` | §10 budget. Frames and dataset must be quota-managed. |
| ROS 1 vs ROS 2 | vendor publishers are **Noetic**; our stack is **Humble in a container**; no `ros1_bridge` on the device | `ros2_stack/docs/deployment.md` | A `rclpy` capture node cannot see the image topics today. §3.1 picks a transport. |
| Cameras are **on demand** | rear + OA publish only after `/<cam>/start_capture`; the behaviour tree calls `stop_capture` | `ros2_port_handoff/07_system_config/mower_cam_keeper.py`, `10_ros_live_snapshot/services.txt` | Capture either runs docked/idle, or explicitly calls `start_capture` and puts it back. |
| Behaviour is closed | obstacle avoidance lives in closed nodes; new detections only matter if published as `/ai/det/*` | `mower_docs/03-yolo-perception.md` | Weight swaps are safe to *test* but only change behaviour through the existing topics. |
| OTA clobbers `/workspace` | OTA archives in `/userdata/RobotData/upgrade` | `mower_docs/07-cautions.md` | Keep the original `.rknn` + a re-apply script on `/userdata`. |

## 2. Capture sources

### 2.1 Physical / V4L2 layer

| Stream | Device node | Underlying | Format / geometry / rate | Owner node |
|---|---|---|---|---|
| Rear camera | `/dev/rear_camera` → `video62` | USB UVC `32e6:9221` ("FHD webcam") | `mjpeg2mono8` / `YUV422P`, **1920×1080 @ 30 fps** | `/rear_camera` (`base_cameras_node`) |
| Left OA camera | `/dev/left_oa_camera` → `video53` | Metoak module | `uyvy` / `UYVY422`, **1920×1080 @ 15 fps** | `/rear_camera` (same node, `mower_cam_keeper` drives it) |
| Right OA camera | `/dev/right_oa_camera` → `video44` | Metoak module | `uyvy` / `UYVY422`, **1920×1080 @ 15 fps** | idem |
| Front stereo (raw) | `/dev/videoSimor` (`video11`), `/dev/videoIsp` (`video22`) | MIPI-CSI, Metoak `libMoGeneralSDK` | raw `V4L2_PIX_FMT_SBGGR8` 640×360 / `YVYU` 1280×480; side-by-side 3240×810 and 1920×360 geometries observed | `stereo_ros` |

Sources: `ros2_port_handoff/09_platform/{dev_nodes,udev-symlinks,lsusb}.txt`,
`ros2_port_handoff/07_system_config/udev/mower.rules`,
`mower_drivers/base_cameras/config/mower_cameras.yaml`,
`06_vendor_sdks/metoak_tools_and_kmods/mo_init.sh`, `mower_docs/10-hardware.md`.

**Never open `/dev/video*` while the owning node has it open.** `base_cameras` and
`stereo_ros` hold the devices; a second `VIDIOC_STREAMON` on the same node fails with `EBUSY`
(and, for the Metoak path, fights the SDK's own pipeline). Direct-V4L2 capture is only viable
with the vendor node stopped (`systemctl stop mower-perception.service`), which is a bringup
experiment, not the steady state.

### 2.2 ROS topic layer (what `det_ros` actually consumes)

Verified from `strings devel/det_ros/lib/det_ros/det_ros` — the four image subscriptions are
exactly these, and **there is no `/rear_camera` subscription** (zero matches in the binary):

| Topic | Type | Notes |
|---|---|---|
| `/vio/right/image_raw` | `sensor_msgs/Image` | front right eye, **always publishing** (localization needs it), 640×480 |
| `/vio/depth/image_raw` | `sensor_msgs/Image` | stereo depth, feeds `/ai/det/dock_distance` |
| `/left_oa_camera/image_raw` | `sensor_msgs/Image` | only after `start_capture`; behind `publish_oa_left_detect` |
| `/right_oa_camera/image_raw` | `sensor_msgs/Image` | only after `start_capture`; behind `publish_oa_right_detect` |

The front **left** eye `/vio/left/image_raw` is the same geometry and is *not* consumed by
`det_ros` — extra data for a stereo-aware or single-eye model, not for reproducing the shipped
detector's behaviour.

**The rear camera is not part of `det_ros` at all.** Its live subscribers are `mower_charge`
(`/rear_camera/image_raw` + `/rear_camera/camera_info` — that is how the dock is found) and
`mower_base` (`/rear_camera/camera_status`). Two consequences for this pipeline:

* Rear frames are still worth capturing — same lawn, same mower, cheap — but they train a
  *rear-facing* detector, not the shipped front one. A model trained on all four views and
  deployed behind `det_ros` will see a front-camera distribution shift it was never validated
  on. Either train front-only (matching `det_ros`) or train two models and give the rear one its
  own node (§8 Option B).
* `mower_charge` shares `/rear_camera/image_raw` with anything else that subscribes. Adding
  capture pressure on that topic slows dock detection; keep the rear rate at 0.5 Hz or lower
  during charging tests.

Published debug output we can reuse as a visual check: `/ai/det/bgr_draw_box` (Image),
`/ai/det/bgr_draw_box_large`, `/ai/det/box_in_org` (foxglove `ImageAnnotations`),
`/ai/det/pointcloud_box_ob` (`PointCloud2`), `/ai/det/pointcloud_nearest_class`
(`mower_msgs/AIClouds`), `/ai/det/dock_distance`.

`det_ros/param/cfg.yaml` gates which of these exist (`publish_bgr_draw_box`,
`publish_bgr_draw_box_large`, `publish_box_annotations`, …). Turning on `bgr_draw_box` is the
cheapest possible deployment QA: view it over `foxglove_bridge` :8765.

### 2.3 Compressed topics — the cheapest capture path

Every camera also publishes a `CompressedImage` JPEG side-channel, throttled by
`pub_compress_freq` in `mower_cameras.yaml` (rear: every 3rd frame ≈ 10 Hz; OA cams: every
frame ≈ 15 Hz):

```
/rear_camera/image_raw/compressed
/left_oa_camera/image_raw/compressed
/right_oa_camera/image_raw/compressed
/vio/left/image_raw/compressed
/vio/right/image_raw/compressed
```

Subscribing to these and writing `msg.data` straight to `*.jpg` costs **zero decode**, no
`cv_bridge`, no OpenCV. That is the default capture path in §3. Only fall back to the raw
topics when a frame is needed at full fidelity (e.g. to measure the resize policy, §13 Q2) or
when the JPEG quality of the vendor encoder is visibly destroying small objects.

### 2.4 MJPEG over HTTP — the zero-install path

`mower-webcam.service` already runs `web_video_server` on `:8181` with
`_address:=0.0.0.0 _server_threads:=4 _ros_threads:=4`, and `mower-rtsp.service` fronts it with
`mediamtx` on `:8554`. `mower-cam-firewall.service` restricts both ports to the local Wi-Fi.

```bash
# one-off grab, no ROS, no node
curl -s 'http://<mower-ip>:8181/snapshot?topic=/vio/right/image_raw' -o front.jpg
curl -s 'http://<mower-ip>:8181/stream?topic=/rear_camera/image_raw&type=mjpeg' -o rear.mjpeg
```

Useful for spot checks and for the manual "go photograph the 20 things the model gets wrong"
step. Not usable as a training harvest: `web_video_server` is roughly single-threaded and a
held stream blocks concurrent requests, and re-encoding 1080p MJPEG over the wire costs more
CPU than writing the ROS message.

### 2.5 Existing bags

`/userdata/RobotData/bag/` already holds four ROS bags from a 2026-03-30 mapping run
(`mapping_2026-03-30_09-33-37_0.bag` + `_ext_0.bag`, and a task pair). `mower_cameras.yaml`
sets `bag_img_reduce: 4`, so their images are downscaled 4× — usable as a free bootstrap set
and as a smoke test for the whole pipeline before collecting anything new. Treat them as
read-only; back up `/userdata/RobotData` first (`mower_docs/07-cacautions.md`).

## 3. Capture node

Target package: `ros2_stack/src/mower_perception_capture/` (new, `ament_python`), mirroring
`mower_localization`'s layout (`resource/`, `package.xml`, `setup.py`, `setup.cfg`,
`mower_perception_capture/*.py`, `test/`).

```
ros2_stack/src/mower_perception_capture/
├── package.xml
├── setup.py                       # console_scripts: capture_node, label_suggest (later)
├── setup.cfg                      # flake8 max-line-length = 100, extend-ignore = E203
├── config/
│   └── capture.yaml               # topics, rates, quota, camera_info paths
├── mower_perception_capture/
│   ├── __init__.py
│   ├── capture_node.py            # the node below
│   ├── frameio.py                 # manifest append, quota/rotation, phash dedupe
│   └── label_suggest.py           # §4 (separate entry point, no ROS)
└── test/
    ├── conftest.py
    └── test_frameio.py            # pytest, no hardware
```

### 3.1 Getting ROS 1 images into an `rclpy` node

Three transports, in order of preference for the current device state:

**A. Run the capture node on the host under Noetic (`rospy`).** Zero new infrastructure, zero
bridge cost, image bytes never cross a process boundary. Cost: the node is ROS 1, which is
inconsistent with the rest of `ros2_stack`. Mitigation: keep the node ROS-agnostic —
`frameio.py` and the message handling take `(topic_name, stamp_ns, encoding, width, height,
jpeg_bytes)` tuples, so only `capture_node.py` is ROS-specific and a ~40-line `rospy` sibling
covers this case. **This is the recommended first cut.**

**B. Direct V4L2 grab from inside the Jazzy container** (`privileged: true`, `/dev` bind-mounted
in `docker/docker-compose.yml`). No ROS involved at all; only valid with the vendor camera node
stopped. Use it for the calibration set (§6.3), where you want frames the vendor pipeline
isn't touching.

**C. `ros1_bridge` for `sensor_msgs/Image` + `CompressedImage`.** Must be built from source
with `mower_msgs` (`deployment.md`), and `02-ros2-migration.md` explicitly warns that copying
camera images through the bridge costs CPU and RAM the board does not have. Only consider it
once `mower_perception_capture` needs to sit in the same graph as the rest of our stack. If you
do: bridge only `/vio/right/image_raw/compressed`, `/rear_camera/image_raw/compressed`,
`/left_oa_camera/image_raw/compressed`, `/right_oa_camera/image_raw/compressed` — never the raw
1920×1080 topics.

Independent of A/B/C, `/userdata/frames` must be bind-mounted into the container:

```yaml
# docker/docker-compose.yml, mower_jazzy service
    volumes:
      - ..:/work
      - /userdata/frames:/userdata/frames     # add; created by setup script, not committed
      - /dev:/dev
```

### 3.2 Node design

Responsibilities, in order of importance:

1. **Subscribe** to N image sources (compressed preferred, raw optional per source).
2. **Throttle** to `max_rate_hz` per source (default 1.0; 2.0 for `/vio/*`).
3. **Skip frames** that are near-duplicates of the last kept frame (perceptual hash, Hamming
   distance > `phash_min_distance`, default 6) — a parked mower otherwise writes 100 near
   identical JPEGs.
4. **Write** `frames/<session>/<cam>/<seq:06d>_<stamp_ns>.jpg` atomically (write `.tmp`, then
   `os.replace`).
5. **Append one manifest line** per frame (see schema below), `flush()`ed per line so a crash
   loses at most the tail.
6. **Guard the disk**: stop capturing (throttled WARN, still responsive) when the session
   directory exceeds `quota_bytes`; prune oldest *sessions* when total `/userdata/frames` usage
   exceeds `total_quota_bytes`. Never delete the newest session (it may be mid-capture).
7. **Optionally** call `/<cam>/start_capture` for the gated cameras and register a shutdown
   hook to `stop_capture` them again — only when `wake_cameras: true`, because the vendor
   behaviour tree also drives those services and we do not want to fight it.

Explicit non-goals: no NPU inference, no bag writing, no cloud upload, no re-encoding beyond
what the vendor publisher already did.

### 3.3 Manifest schema (`manifest.jsonl`, one object per frame)

```json
{"seq": 1841, "cam": "rear_camera", "topic": "/rear_camera/image_raw/compressed",
 "stamp_ns": 1772668800123456789, "wall_iso": "2026-03-05T09:20:00.123456+00:00",
 "encoding": "jpeg", "width": 1920, "height": 1080, "bytes": 271433, "phash": "3f9c1a77e0d24b51",
 "odom": [4.812, -1.204, 0.71], "light_status": "day", "battery_pct": 82,
 "session": "2026-03-05T09-15-00Z", "source": "compressed"}
```

* `stamp_ns` is the **message** header stamp (ROS 1 `rospy.Time` nanoseconds), not the write
  time — that is what makes the set usable for video-sequence labelling and for correlating
  with `/ai/det/*`.
* `odom` (optional, from `/odom`) gives pose so a reviewer can tell "same rock from 6 angles"
  from "six rocks".
* `light_status` (optional, `/vio/light_status` from `light_ai`) is how the day/dusk/night
  split gets made — `light_ai`'s own `model_1.rknn` already classifies it, so the dataset can
  be balanced across the three regimes without guessing.

### 3.4 Node skeleton

Written in the style of `mower_localization/gps_gate.py` (module docstring with a
`Parameters` table, dataclasses for config, throttled logging, no hardware in unit tests):

```python
#!/usr/bin/env python3
"""Frame capture for the YOLO retrain loop: ROS image topics -> /userdata/frames.

Subscribes to the vendor camera topics listed in §2.2/§2.3 and writes one JPEG per kept
frame plus a JSONL manifest line.  Never opens /dev/video*: the vendor nodes own the
devices and a second VIDIOC_STREAMON on the same node fails with EBUSY.

Parameters
----------
``topics``                 list of ``"name=topic"`` entries for the CompressedImage side-channels
                           (§2.3); default:
                           rear=/rear_camera/image_raw/compressed,
                           left_oa=/left_oa_camera/image_raw/compressed,
                           right_oa=/right_oa_camera/image_raw/compressed,
                           front=/vio/right/image_raw/compressed
``max_rate_hz``            per-source keep rate, default 1.0 (rear: use 0.5 — see §2.2)
``phash_min_distance``     perceptual-hash Hamming distance to treat a frame as new, default 6
``out_dir``                ``/userdata/frames``
``quota_bytes``            per-session cap before capture stops, default 2 GiB
``total_quota_bytes``      cap for all sessions before the oldest is pruned, default 12 GiB
``wake_cameras``           call /<cam>/start_capture for gated cameras and stop on exit, default False
``write_camera_info``      copy each camera's CameraInfo yaml into the session dir, default True

QoS: SensorDataQoS (best-effort, depth 5).  The vendor publishers are reliable, and a
best-effort subscriber is the one profile that connects to both reliable and best-effort
publishers; the depth bound is what keeps a 10 Hz JPEG stream from ballooning this node's
queue while JPEG encoding runs.
"""

from __future__ import annotations

import dataclasses
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from .frameio import FrameWriter, phash


@dataclasses.dataclass(frozen=True)
class Source:
    name: str
    topic: str
    compressed: bool = True
    rate_hz: float = 1.0


class CaptureNode(Node):
    def __init__(self) -> None:
        super().__init__('capture_node')
        self.declare_parameter('topics', [
            'rear=/rear_camera/image_raw/compressed',
            'left_oa=/left_oa_camera/image_raw/compressed',
            'right_oa=/right_oa_camera/image_raw/compressed',
            'front=/vio/right/image_raw/compressed',
        ])
        self.declare_parameter('max_rate_hz', 1.0)
        self.declare_parameter('phash_min_distance', 6)
        self.declare_parameter('out_dir', '/userdata/frames')
        self.declare_parameter('quota_bytes', 2 * 1024 ** 3)
        self.declare_parameter('total_quota_bytes', 12 * 1024 ** 3)
        self.declare_parameter('wake_cameras', False)

        self.sources = [Source(*s.split('=')) for s in self.get_parameter('topics').value]
        self.writer = FrameWriter(
            out_dir=self.get_parameter('out_dir').value,
            quota_bytes=int(self.get_parameter('quota_bytes').value),
            total_quota_bytes=int(self.get_parameter('total_quota_bytes').value),
            phash_min_distance=int(self.get_parameter('phash_min_distance').value),
        )
        self.last_keep: dict[str, float] = {s.name: 0.0 for s in self.sources}
        self.last_hash: dict[str, int] = {}
        self.written = 0

        for src in self.sources:
            self.create_subscription(
                CompressedImage, src.topic, self._make_cb(src),
                qos_profile_sensor_data)

    def _make_cb(self, src: Source):
        min_period = 1.0 / src.rate_hz

        def cb(msg: CompressedImage) -> None:
            now = time.monotonic()
            if now - self.last_keep[src.name] < min_period:
                return
            data = bytes(msg.data)
            h = phash(data)                      # dHash on the decoded thumbnail
            if self.last_hash.get(src.name) is not None and \
                    (h ^ self.last_hash[src.name]).bit_count() < self.writer.phash_min_distance:
                return
            self.last_keep[src.name] = now
            self.last_hash[src.name] = h
            rec = self.writer.write(src.name, src.topic, msg.header.stamp,
                                    encoding='jpeg', payload=data, source='compressed')
            if rec is None:                     # quota hit: writer is throttling itself
                self.get_logger().warn(
                    f'capture quota reached ({self.writer.quota_bytes} B), not writing')
                return
            self.written += 1
            if self.written % 500 == 0:
                self.get_logger().info(
                    f'captured {self.written} frames, '
                    f'{self.writer.session_bytes / 1e6:.1f} MB in this session')
        return cb
```

`frameio.py` is the part worth testing without hardware, and the part with real logic:

```python
@dataclasses.dataclass(frozen=True)
class FrameRecord:
    seq: int
    cam: str
    topic: str
    stamp_ns: int
    wall_iso: str
    encoding: str
    width: int
    height: int
    nbytes: int
    phash: str
    session: str
    source: str


class FrameWriter:
    """Atomic JPEG writes + append-only manifest + quota/rotation.

    write() returns None (and sets ``quota_hit``) rather than raising when the session
    quota is reached: a full disk must degrade to "stop capturing", never to a dead node
    in the middle of a mow.
    """

    def write(self, cam, topic, stamp_ns, *, encoding, payload, width=0, height=0,
              source='compressed') -> FrameRecord | None:
        if self.quota_hit:
            return None
        seq = self._next_seq(cam)
        path = self.session_dir / cam / f'{seq:06d}_{stamp_ns}.jpg'
        tmp = path.with_suffix('.jpg.tmp')
        tmp.write_bytes(payload)
        os.replace(tmp, path)
        rec = FrameRecord(seq, cam, topic, stamp_ns, _now_iso(), encoding,
                          width, height, len(payload), self._last_hash, self.session, source)
        with open(self.manifest, 'a') as fh:
            fh.write(json.dumps(dataclasses.asdict(rec)) + '\n')
        self.session_bytes += len(payload)
        if self.session_bytes >= self.quota_bytes:
            self.quota_hit = True
        self._enforce_total_quota()
        return rec
```

Suggested pytest targets (no ROS, no hardware, matching `mower_localization`'s
`test_gps_gate.py` shape): atomic-rename leaves no `.tmp` behind; manifest lines parse and
are in `seq` order; `quota_bytes` stops writes without raising and without truncating the
last good frame; `_enforce_total_quota` prunes the oldest session but never the current one;
`phash` rejects a re-encoded identical frame and accepts a shifted one.

### 3.5 Running it

```bash
# host (Noetic), transport A
rosrun mower_perception_capture capture_node \
  _topics:="[front=/vio/right/image_raw/compressed,rear=/rear_camera/image_raw/compressed]" \
  _max_rate_hz:=1.0 _out_dir:=/userdata/frames

# confirm the gated cameras are actually alive first
rosservice call /rear_camera/start_capture "{}"
rostopic hz /rear_camera/image_raw/compressed

# stop it again when finished — the vendor behaviour tree also drives these services
rosservice call /rear_camera/stop_capture "{}"
```

> `start_capture` is the **vendor node's** service; `mower_cam_keeper.service` and the
> behaviour tree also call it (`07_system_config/mower_cam_keeper.py`). Call it by hand only
> while parked, and hand it back with `stop_capture` afterwards — or set `wake_cameras: true`
> and let the node manage the pair symmetrically on start/exit. Never leave the rear camera
> streaming while docked: `mower_charge` reads `/rear_camera/image_raw` for dock detection and
> that is the wrong thing to slow down.

Operational rules:

* Capture **docked or parked**, blade off, ideally with the mower on AC. A 30-minute
  stationary harvest of the same lawn is worth more than 30 minutes of driving.
* If you must capture while driving: `max_rate_hz: 1.0`, `wake_cameras: false`, and check
  `top` — `stereo_ros` is already at ~83 % of a core and JPEG encoding adds to that.
* Turn the blade off for any session containing people/pets in frame (blade safety,
  `mower_docs/07-cautions.md`).
* Stop the node before pulling the SD/eMMC or running an OTA.

## 4. Label suggestions from a local LLM

### 4.1 Where the model runs

| Host | Model | Verdict |
|---|---|---|
| LAN workstation, `ollama serve` bound to the LAN | `qwen2.5vl:7b`, `llama3.2-vision:11b`, `minicpm-v:8b` | **Default.** Box with ≥16 GB RAM. Mower reaches it over `wlan0` (`192.168.1.105` at survey) or Tailscale. |
| Mower itself | any 7B VLM | **No.** 2.0 GB RAM available, no swap, shared with `stereo_ros` + `det_ros` + `seg_ros`. It would OOM the perception stack. |
| Mower itself | a ≤2B quantised VLM (`moondream`, `qwen2-vl:2b`) | Technically possible, but a 2B model's boxes are not good enough to bootstrap a safety-relevant detector. Revisit only if you want *triage* (which frames look interesting) rather than labels. |

`ollama serve` exposes an OpenAI-compatible surface, so the client is a dozen lines of
`requests`/`httpx` and is swappable for vLLM or llama.cpp's server without touching the
pipeline:

```bash
# on the LAN host
OLLAMA_HOST=0.0.0.0:11434 ollama serve
ollama pull qwen2.5vl:7b
curl -s http://<lan-host>:11434/v1/models | jq '.data[].id'

# from the mower — confirm reachability before wiring anything up
curl -s http://<lan-host>:11434/v1/chat/completions -H 'Content-Type: application/json' -d '{
  "model": "qwen2.5vl:7b", "temperature": 0, "max_tokens": 512,
  "messages": [{"role": "user", "content": "reply with the single word: ok"}]}'
```

**Firewall.** `ollama` has no auth. Bind it to the LAN interface only and, as with 8181/8554
(`mower-cam-firewall.service`), restrict with `iptables -A INPUT -p tcp --dport 11434 ! -s
<your-subnet> -j DROP`. Every frame you send to it leaves the mower.

### 4.2 The classifier-vocabulary trap

The shipped detector's 22 classes are an **ABI** — `det_ros`'s post-processing and the
`mower_msgs/AIClouds` fusion downstream are keyed to that list and order
(`best_large_list.txt`, and the dynamic/static/special id maps in `seg_ros/readme.md`).

So the LLM is **not** asked to invent classes. It is asked to pick from a closed vocabulary,
and the client rejects anything else:

```
idx class name      idx class name      idx class name
 0  person           8  shovel           16  book
 1  dog              9  manhole          17  backpack
 2  cat             10  brick            18  wood
 3  sports ball     11  trashbin         19  chair
 4  hedgehog        12  toycars          20  trunk
 5  rabbit          13  potted plant     21  dock
 6  stone           14  cans
 7  hoe             15  bottle
```

(Authoritative list: `src/mower_perception/det_ros/model/best_large_list.txt`, 22 entries,
index 0..21, `person … dock`, no trailing newline on the last one. Use the file, not this
table — and read it, do not hard-code it, so the indices survive a re-export.)

`dock` deserves special handling: it is the charging target, it is safety-relevant for
undocking, and it is rare. Do not let the LLM free-hand it; take it from
`mower_charge`'s dock-detection cues if possible, or hand-label the dock frames.

### 4.3 Request

```jsonc
POST http://<lan-host>:11434/v1/chat/completions
{
  "model": "qwen2.5vl:7b",
  "temperature": 0.0,
  "max_tokens": 768,
  "response_format": {"type": "json_object"},
  "messages": [{
    "role": "system",
    "content": "You label robot-camera frames for an object detector. Return ONLY JSON:\n\
{\"boxes\": [{\"class\": \"<one of the allowed names>\", \"box\": [x1,y1,x2,y2], \"confidence\": 0.0-1.0}]}\n\
Coordinates are integers in a 0-1000 normalized frame: 0 = left/top edge, 1000 = right/bottom edge.\n\
Allowed classes ONLY: person, dog, cat, sports ball, hedgehog, rabbit, stone, hoe, shovel,\n\
manhole, brick, trashbin, toycars, potted plant, cans, bottle, book, backpack, wood, chair, trunk, dock.\n\
Return {\"boxes\": []} if nothing from the list is visible. Never invent a class. Never guess."
  }, {
    "role": "user",
    "content": [
      {"type": "text", "text": "Label this frame. Camera: front stereo right, mower deck height."},
      {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64,<...>"}}
    ]
  }]
}
```

Practical notes, all of which cost accuracy if ignored:

* **Downscale before sending.** Vision LLMs degrade on very large images and the mower is
  bandwidth-bound. 1024 px on the long edge, JPEG q85, ~120–180 KB per request.
* **`temperature: 0`** and a fixed prompt, or the labels are not reproducible and re-running
  the batch silently changes the dataset.
* **Many models emit 0–1000 normalized coordinates** (Qwen2-VL family explicitly); others emit
  pixels or 0–1 floats. The converter in §5 detects and normalizes, but verify against one
  known frame before running a batch.
* **No history in the request.** Tracking/id reasoning across frames makes small models worse,
  not better, and it makes the batch non-reproducible.
* **Batch size 1, concurrency 2–4.** Serialise per session directory; log every request and
  raw response to `labels/raw/<stem>.json` so a bad batch can be re-run or audited.
* **Empty responses are a real outcome.** Treat `{"boxes": []}` as "candidate negative" and
  keep it — negatives are half the dataset (§5.3).

### 4.4 The hybrid that actually matters

A 7B VLM is a *worse* labeller for the 22 in-domain classes than the model already on the
NPU. Use it for what it is good at:

1. **Triage** — which frames contain anything at all (cheap `boxes: []` vs non-empty).
2. **Novelty** — objects the current model misses, or classes the current model
   systematically confuses. Run `best_large_0208.rknn` on the captured set
   (`rknn_model_zoo/examples/yolov8/python` on-device, or `rknn-toolkit-lite2` from the
   container) and diff against the LLM's boxes. **Disagreements are the training set.**
3. **Bootstrap labels for a genuinely new class** you intend to add to the vocabulary — which
   requires editing `best_large_list.txt`-equivalent ordering *and* re-checking the closed
   fusion mapping in `seg_ros/readme.md`. That is a bigger change than a weight swap; treat it
   as a separate project.

Everything the current model gets right and the LLM agrees with is a candidate
**pseudo-label**, not ground truth, and it is labelled as such in the manifest.

## 5. From suggestions to YOLO labels

### 5.1 Converter

`label_suggest.py` and `suggestions_to_yolo.py` are pure-Python, no ROS, unit-testable:

```python
# The class list is read from the shipped file, never hard-coded: the indices are the
# ABI that det_ros's post-processing and the closed fusion path depend on.
_LIST = Path('src/mower_perception/det_ros/model/best_large_list.txt').read_text()
ALLOWED = tuple(line.strip() for line in _LIST.splitlines() if line.strip())
CLASS_ID = {name: i for i, name in enumerate(ALLOWED)}
assert len(ALLOWED) == 22, f'expected 22 classes, got {len(ALLOWED)}'   # 22 for best_large

MAX_BOX_FRAC = 0.90          # reject boxes that are ~the whole frame (usually a failure)
MIN_CONF = 0.35              # below this the suggestion goes to review, never to labels/


def to_yolo_line(cls: str, box, w: int, h: int, conf: float) -> str | None:
    """One YOLO row: '<class_id> <cx> <cy> <bw> <bh>', all normalized to [0, 1]."""
    if cls not in CLASS_ID:
        return None                                  # hallucinated class -> dropped
    if conf < MIN_CONF:
        return None
    x1, y1, x2, y2 = _denorm(box, w, h)             # handles 0-1000 / 0-1 / pixel forms
    x1, x2 = sorted((max(0.0, x1), min(1.0, x2)))
    y1, y2 = sorted((max(0.0, y1), min(1.0, y2)))
    bw, bh = x2 - x1, y2 - y1
    if bw <= 0 or bh <= 0:
        return None
    if bw * bh > MAX_BOX_FRAC:
        return None
    return f'{CLASS_ID[cls]} {(x1 + bw / 2):.6f} {(y1 + bh / 2):.6f} {bw:.6f} {bh:.6f}'


def frame_labels(stem: str, suggestion: dict) -> tuple[str, str]:
    """Returns (yolo_txt_body, review_reason).  review_reason == '' means auto-accepted."""
    ...
```

`review_reason` is non-empty when: any class was out-of-vocabulary, any box was degenerate or
huge, more than 12 boxes survived (a sign of a bad response), the JSON did not parse, or the
LLM's confidence sat in the 0.35–0.5 band. Those frames go to `review/` and are labelled by
hand (labelme / CVAT run on the LAN host against the same JPEG files).

### 5.2 Dataset layout

Ultralytics expects sibling `images/` and `labels/` directories. Build them from the manifest
with a small deterministic script (`build_dataset.py`, reads `manifest.jsonl`, reads
`suggestions/*.json`, writes `review/`):

```
/userdata/dataset/v1/
├── images/{train,val}/  *.jpg          # symlink or hardlink to /userdata/frames (saves bytes)
├── labels/{train,val}/  *.txt          # '<id> cx cy w h' rows
├── review/                             # frames the client refused to auto-accept
├── data.yaml                           # names: [...22 exact names...], nc: 22, path: /userdata/dataset/v1
└── provenance.jsonl                    # frame -> manifest seq, source class (llm|pseudo|human|neg), model sha
```

Hardlink rather than copy: the frames are already on `/userdata` and duplicating a 12 GiB set
is the fastest way to exhaust the 24 GB budget (§10).

Split **by session and by pose, not by frame.** Consecutive 1 Hz frames of the same rock are
one sample in train and one in val otherwise, and the resulting mAP is fiction. Bucket by
`(session, odom cell ~0.5 m)` and assign whole buckets.

### 5.3 What goes in the box

| Source | Share of the final set | Notes |
|---|---|---|
| LLM suggestions, human-confirmed | 30–40 % | the expensive, high-value part |
| Current-model pseudo-labels where the LLM agrees | 40–50 % | free, and in-domain |
| `boxes: []` frames near a labelled object | 10–20 % | **negatives**. Without these the model invents stones in grass |
| Deliberate hard cases | 5–10 % | dusk/night (`light_status`), wet grass, backlit, `bgr_draw_box` false positives, partial occlusions at the frame edge |

Class balance follows the fusion semantics: dynamic (person, dog, cat, hedgehog, rabbit) must
be recall-dominant — a missed person is a safety failure — while static classes
(stone, manhole, brick, …) can tolerate a lower recall because `seg_ros`'s segmentation and
`bgr_image_monitor`/`image_quality` provide the depth-side backstop. Do not balance the dataset
uniformly across 22 classes; the shipped model clearly did not.

## 6. Training, export, conversion

### 6.1 Train (LAN host with a GPU)

```bash
git clone https://github.com/ultralytics/ultralytics && cd ultralytics
pip install -e .

# nc=22, names exactly as best_large_list.txt, and the detector's real geometry.
# Ultralytics takes imgsz as (height, width) -> the exported NCHW input becomes [1,3,640,480],
# matching the shipped model. A bare `imgsz=640` gives 640x640 and will NOT load in det_ros.
# pretrained yolov8n — yolov8s if the LAN host has >=16 GB VRAM and latency headroom
yolo detect train \
  model=yolov8n.pt \
  data=/userdata/dataset/v1/data.yaml \
  imgsz="[640,480]" \
  epochs=150 patience=25 batch=32 device=0 \
  project=/userdata/runs/v1 name=det_22cls

# Ultralytics licence is AGPL-3.0 — relevant only if you redistribute.
# https://github.com/ultralytics/ultralytics/blob/master/LICENSE
```

Non-negotiables, straight out of `mower_docs/03-yolo-perception.md`:

* **22 classes, same names, same order** as `best_large_list.txt`.
* **Input geometry 640×480** (NCHW `[1,3,640,480]`, i.e. 480 wide × 640 tall — *not* a
  640-wide landscape image; the shipped feature maps are 80×60, 40×30, 20×15, which is a
  stride-8 pyramid over a 480×640 canvas). Train and export with the tuple and **verify the
  exported ONNX input shape** before converting (§6.4). If Ultralytics refuses the tuple, that
  refusal is itself the finding: it means the vendor used a Rockchip-fork-specific imgsz path
  and you must reproduce that path exactly.
* **Preprocessing must match** too — letterbox vs centre-crop vs stretch. See §13 Q2; this is
  the single most common way a "trained" model scores well on paper and detects garbage in the
  field.
* Fresh weights, not fine-tuning `best_large_0208.rknn` (that is a compiled int8 blob; you
  cannot fine-tune from it). Start from `yolov8n.pt` and use the shipped `.rknn` only as a
  **teacher** for pseudo-labels.

### 6.2 Export to the Rockchip 9-output ONNX

Stock Ultralytics exports the raw per-scale head tensors (or, with `nms=True`, one decoded
output); `det_ros` needs three scales × (boxes, scores, score-sum) = **9 tensors**. Use
Rockchip's fork, which is what produced the shipped models
(`rknn_model_zoo/examples/yolov8/README.md` credits `airockchip/ultralytics_yolov8`):

```bash
git clone https://github.com/airockchip/ultralytics_yolov8 && cd ultralytics_yolov8
pip install -e .
python export.py \
  /userdata/runs/v1/det_22cls/weights/best.pt \
  --output best_large_0208.onnx --input_size 640 480 --opset 12 --simplify
```

YOLO11 has the same head layout and should load; that is untested on this device.

### 6.3 Convert to int8 `.rknn`

`rknn-toolkit2` runs on an **x86_64** Linux host (not on the RK3588), so this step is a LAN
step too. Calibration data must come from these cameras — that is the whole reason §2 exists.

```bash
pip install rknn-toolkit2==2.3.0     # match the shipped models; verify the runtime version below

# 200-500 representative frames across day/dusk/night and all four cameras.
# rknn-toolkit2 resizes to the model input shape itself, but keep the list to one path per line.
find /userdata/frames -name '*.jpg' | shuf -n 300 > /userdata/dataset/v1/calib_list.txt

cd ros2_port_handoff/13_open_source_upstreams/rknn_model_zoo/examples/yolov8/python
python convert.py /userdata/runs/v1/best_large_0208.onnx rk3588 i8 \
  /userdata/models/v1/best_large_0208.rknn
```

The example `convert.py` hard-codes `DATASET_PATH = '../../../datasets/COCO/coco_subset_20.txt'`,
so either edit that line to point at `calib_list.txt` or copy the ~30-line script into your
own `convert_mower.py` and set `dataset=` there. It also hard-codes the preprocessing you must
keep: `mean_values=[[0, 0, 0]]`, `std_values=[[255, 255, 255]]`, which matches the shipped
models' embedded quant table (`mean [0,0,0]`, `std [255,255,255]`, asym per-layer, input range
`[0,1]`, `rgb2bgr: False`). Keep those exact values or `det_ros`'s own preprocessing will not
match the model and every box will be subtly wrong — a failure that looks like "the retrain
didn't work".

**Before converting, pin the runtime version** — the tree ships two and the survey says a
third:

```bash
# on the mower
ldd /workspace/devel/det_ros/lib/det_ros/det_ros | grep rknnrt
strings $(ldd /workspace/devel/det_ros/lib/det_ros/det_ros | awk '/librknnrt/{print $3}') \
  | grep -m1 'librknnrt version'
# shipped copies, for reference:
#   rknn2_runtime/librknn_api/aarch64/librknnrt.so  -> 2.2.0
#   rknn2_runtime/3rdparty/rknpu2/Linux/aarch64/...  -> 2.1.0
```

Compile with a toolkit version the runtime accepts. The shipped models were built with 2.3.0
and load on the reported 2.1.0 runtime, so 2.3.0 is the known-good setting; if your converted
model fails `rknn_init` with a version error, step the toolkit down before touching anything
else.

### 6.4 Output contract self-check

Before the model goes anywhere near the mower, compare its metadata against the shipped model
byte-for-byte in the fields `det_ros` depends on:

```bash
for m in ros2_port_handoff/11_perception_models/best_large_0208.rknn /userdata/models/v1/best_large_0208.rknn; do
  echo "== $m"
  strings -a "$m" | grep -m1 -o "'images': {[^}]*}"
  strings -a "$m" | grep -o "'[0-9]\+': {'is_output': True[^}]*'shape': \[[0-9, ]*\]" | head -12
done
```

Expect: input `1, 3, 640, 480`; nine outputs at `80×60`, `40×30`, `20×15`; 22 score channels at
the largest scale. Any deviation means the export path is wrong — go back to §6.2, do not
"try it and see".

## 7. QA gate

Run everything below **before** the model goes into `det_ros/model/`. The gate is pass/fail;
no partial credit, because the mower is a blade-carrying robot and false negatives on `person`
are the failure mode that matters.

### 7.1 Offline (LAN host)

| Check | Threshold | Why |
|---|---|---|
| mAP50 on the pose-split val set | ≥ shipped baseline + 2 pts | below baseline, nothing to ship |
| mAP50-95 | ≥ baseline | guards against box regression |
| Per-class recall, `person`/`dog`/`cat`/`hedgehog`/`rabbit` | ≥ 0.95 each, day **and** night subsets | safety classes are recall-critical |
| Per-class recall, static classes | ≥ 0.70 | `seg_ros` backstops these |
| Hard-negative set: false positives per 1000 frames | ≤ baseline, and ≤ 0.5 on the clean-lawn subset | invented stones cause phantom obstacles |
| int8 vs fp32 agreement on the val set | ≥ 0.97 IoU>0.5 box match; mean score delta ≤ 0.05 | quantization drift is the usual silent regression |
| Class-name and order equality with `best_large_list.txt` | exact | ABI |
| Input geometry `1×3×640×480` | exact | ABI |

Get the *baseline* numbers by validating the current setup, not by trusting a number from the
vendor: `best_large_0208.rknn` cannot be validated directly by Ultralytics, so either
de-quantise-and-decode it or approximate the baseline with the vendor node's own behaviour
(`/ai/det/bgr_draw_box` on a fixed frame set, scored by hand). A rough baseline beats none.

### 7.2 On the mower, model still in a scratch location

```bash
# 1) run the new .rknn through the same path det_ros uses, from the Jazzy container
#    (rknn-toolkit-lite2, or the rknn_model_zoo yolov8 python demo) against a fixed
#    frame set in /userdata/frames/qa/ and diff against the 7.1 fp32 reference
# 2) NPU load while det_ros still runs the stock model. Rockchip's rknpu debugfs
#    (needs debugfs mounted on /sys/kernel/debug; `npu-load` from the Rockchip NPU
#    tools prints the same numbers). Expect Core0 +~25 pts, Core1/Core2 ~0.
cat /sys/kernel/debug/rknpu/load
# 3) latency: time 100 inferences; the new model must not push det_ros below its
#    current frame rate, or the perception stack starts dropping depth frames
```

Then the swap itself, in a way that is reversible in one command:

```bash
sudo systemctl stop mower-perception.service
sudo cp -a /workspace/src/mower_perception/det_ros/model/best_large_0208.rknn \
           /userdata/models/best_large_0208.rknn.orig      # rollback artefact, on /userdata
sudo cp /userdata/models/v1/best_large_0208.rknn \
        /workspace/src/mower_perception/det_ros/model/best_large_0208.rknn
sudo sed -i 's#^  model_path_detect:.*#  model_path_detect: "model/best_large_0208.rknn"#' \
        /workspace/src/mower_perception/det_ros/param/cfg.yaml
sudo sed -i 's/^  publish_bgr_draw_box: 0/  publish_bgr_draw_box: 1/' \
        /workspace/src/mower_perception/det_ros/param/cfg.yaml   # visual QA over foxglove
sudo systemctl start mower-perception.service
journalctl -u mower-perception -f | grep -i 'det_ros\|rknn\|init_yolov8'
```

Rollback is the inverse of that block, and `/userdata/models/best_large_0208.rknn.orig` is the
only copy that survives an OTA. Write a `deploy_model.sh` / `rollback_model.sh` pair rather than
retyping the commands; put them on `/userdata` and re-add them after any OTA.

### 7.3 Live acceptance

1. `foxglove_bridge` :8765 → `/ai/det/bgr_draw_box` (set `publish_bgr_draw_box: 1`) shows boxes
   tracking real obstacles in the **front** view with no jitter and no duplicate boxes.
   `bgr_draw_box` is an overlay on `det_ros`'s own input view, so it is the front camera only —
   there is no rear/OA overlay to look at unless you also enable `publish_bgr_draw_box_large`
   and the `publish_oa_left_detect` / `publish_oa_right_detect` flags. For rear coverage, look
   at `/ai/det/pointcloud_box_ob` in Foxglove 3D.
2. `/ai/det/pointcloud_nearest_class` and `/ai/det/dock_distance` look sane, and
   `/planning/get_obs` reports the same obstacle set it did before (i.e. the closed fusion
   path actually consumed the new detections rather than ignoring them).
3. Drive the **same short route** twice — once on the stock model, once on the new one — with
   the blade off, wheels on the ground, and record both runs. Same route, same conditions.
4. Confirm no new `/notice_code` / fault entries and no `/bumper_cloud` events attributable to
   phantom obstacles.
5. **Day and night.** The `light_ai` classifier exists precisely because lighting changes
   behaviour; a model that only passes at noon has not passed.

If any of these fail, roll back. A weight swap is cheap; a bad obstacle set in the field is
not.

## 8. Handoff to the NPU runtime

Two supported end states:

**Option A — swap `det_ros`'s weights** (§7.2). Least effort, and it keeps the entire closed
perception path (`/ai/det/*` → `fusion_ros_node` → `/ai/fusion/ai_fused_pointcloud` →
planner) intact, which is the only way a new model can change behaviour. Constraints: exact
class list and order, exact input geometry, 9-output head, `rknn-toolkit2` ≤ 2.3.0, model
path in `param/cfg.yaml`.

**Option B — our own node next to `det_ros`.** Write it in the Jazzy container with
`rknn-toolkit-lite2` (or C++ against `librknnrt`), subscribe to `/vio/right/image_raw` and
`/vio/depth/image_raw`, and pin it to the idle NPU core:

```python
rknn_set_core_mask(ctx, RKNN_NPU_CORE_2)     # same trick seg_ros uses for core 1
```

Reusable pieces are already in the tree under `mower_perception/rknn2_runtime/`
(`librknn_api/aarch64/librknnrt.so`, `utils/` image + file helpers, `rknn_server/`). This
option avoids the ABI entirely — a different class list, a different resolution, a different
architecture — but it buys nothing behaviourally until someone publishes in the format the
closed nodes consume, and that format still has to be recovered from live traffic
(`mower_docs/03-yolo-perception.md`, caveat 1). Treat Option B as a research branch.

Neither option changes obstacle-avoidance behaviour on its own. Only `/ai/det/*` does.

## 9. Repo layout

```
airseekers-decompile/
├── ros2_port_handoff/                     # read-only survey
│   ├── 05_driver_binaries/nodes/           # base_cameras_node, stereo_ros
│   ├── 06_vendor_sdks/metoak_tools_and_kmods/  # mo_init.sh: videoSimor/videoIsp geometry
│   ├── 07_system_config/                  # udev/mower.rules, mower-*.service, mower_cam_keeper.py
│   ├── 09_platform/{df,meminfo,dev_nodes,udev-symlinks,lsusb}.txt
│   ├── 10_ros_live_snapshot/               # topics_verbose.txt, services.txt (the topic/service truth)
│   ├── 11_perception_models/*.rknn        # the five shipped models — baseline + rollback source
│   └── 13_open_source_upstreams/rknn_model_zoo/examples/yolov8/   # convert.py + postprocess reference
├── src/mower_perception/                   # vendor source subset
│   ├── det_ros/{model,param,launch,CMakeLists.txt}   # best_large_list.txt, cfg.yaml
│   ├── seg_ros/readme.md                   # dynamic/static/special class id maps
│   └── rknn2_runtime/                      # librknnrt.so (2.1.0 and 2.2.0), utils, rknn_server
├── mower_docs/03-yolo-perception.md        # the model-format contract
├── docs/streaming-and-telemetry.md         # camera map, MJPEG/RTSP, persistence on /userdata
└── ros2_stack/                             # ← all new code lands here
    ├── docker/docker-compose.yml           # + /userdata/frames bind mount
    ├── src/mower_perception_capture/       # NEW: capture_node, frameio, label_suggest
    ├── docs/yolo_retrain_pipeline.md       # THIS FILE
    └── docs/deployment.md                  # container/d-gap context referenced throughout
```

Runtime artefacts on the mower (never in the repo):

```
/userdata/frames/<session>/{rear_camera,left_oa,right_oa,front}/*.jpg + manifest.jsonl
/userdata/frames/<session>/camera_info/*.yaml          # copies of the calibration in use
/userdata/dataset/v1/{images,labels}/{train,val}/, review/, calib_list.txt, data.yaml, provenance.jsonl
/userdata/models/v1/best_large_0208.rknn
/userdata/models/best_large_0208.rknn.orig             # rollback, OTA-proof
/userdata/deploy_model.sh /userdata/rollback_model.sh
```

## 10. Disk budget (24 GB free on `/userdata`)

Measured inputs: `/userdata` 32 GB total, 6.3 GB used, **24 GB free**; `/` 16 GB, 7.6 GB free.

| Item | Unit size | Budget | Notes |
|---|---|---|---|
| Captured frames, 4 cameras @ 1 Hz | 3 × 1920×1080 ≈ 270 KB + 1 × 640×480 ≈ 55 KB ≈ 0.87 MB/s | **≤ 6 GB** (≈ 7 000 s ≈ 115 min of 4-camera capture, ~28 000 frames before dedupe) | Session quota 2 GB, total quota 12 GB, oldest pruned first. Perceptual-hash dedupe (§3.2) cuts this hard when parked — a stationary mower writes far fewer frames than the rate limit allows |
| Calibration set (300 frames) | ≈ 80 MB | 80 MB | hardlinks into the dataset |
| Training dataset (hardlinks) | ≈ 0 (links) | **0 extra** | hardlink, never copy |
| Labelling responses (`labels/raw`) | ≈ 2 KB each | ≤ 50 MB | audit trail, cheap insurance |
| Ultralytics run dir (`/userdata/runs/v1`) | weights 6 MB, `best.pt`/`last.pt` ≈ 24 MB, results/plots ≈ 50 MB | **≤ 300 MB** | |
| ONNX export | ≈ 12 MB | 15 MB | |
| `.rknn` (int8, 22 cls) | ≈ 29 MB | 30 MB | matches `best_large_0208.rknn` = 28.8 MB |
| Model rollback + scripts | 29 MB | 30 MB | |
| Ollama models, **if** ever on-mower | 4–5 GB | **0 — do not** | 2.0 GB RAM available, would OOM |
| Docker image on `/` (Humble) | ≈ 2–3 GB of `/`'s 7.6 GB | — | separate filesystem from `/userdata` |
| **Headroom target** | — | **≥ 6 GB free** | `/userdata` also holds maps, bags, OTA archives |

Hard ceiling: capture stops at the session quota and prunes at the total quota — never grows
past `total_quota_bytes`. Check before and after every session:

```bash
df -h /userdata; du -sh /userdata/frames/* | sort -h | tail
```

If `/userdata` is tight, `ros2_stack/scripts/preflight.sh` already warns below 10 GB — keep at
least that much slack for OTA archives in `/userdata/RobotData/upgrade`.

## 11. Cautions

* **Nothing was run against the mower.** `ros2_stack/README.md` is explicit: no apt changes,
  no flashing, no `systemctl` on 192.168.1.105. This document keeps that rule; every command
  in it is written to be run by a human who has decided to.
* **Don't `apt upgrade`** the device — the closed binaries need Ubuntu 20.04 / Noetic / the
  vendor OpenCV 4.2 ABI (`mower_docs/02-ros2-migration.md`).
* **OTA replaces `/workspace`**, so it replaces swapped models, `cfg.yaml` edits and any
  `/etc` unit. Keep binaries and rollback artefacts on `/userdata`.
* **Do not disable the safety nodes.** `mower_base` enforces estop/bumper/lift; a
  perception experiment is not a reason to touch it.
* **Blade off for any capture or test session.** `96 cutter=1` is one service call away.
* **Privacy.** Frames from a home lawn contain houses, plates, neighbours and pets. Keep the
  LLM endpoint on your own LAN, keep the raw frames on `/userdata` (not in a git repo — add
  `frames/`, `dataset/`, `runs/` to `.gitignore` if you ever sync this tree), and delete
  sessions you no longer need.
* **Licensing.** Ultralytics is AGPL-3.0; `rknn_model_zoo` is Apache-2.0. Personal use on your
  own mower is fine; redistribution is a different question.
* **Ports.** 8181/8554 (cameras), 8765 (foxglove), 13344 (mower_logic), 11434 (your Ollama)
  are all unauthenticated. `mower-cam-firewall.service` only covers 8181/8554.

## 12. Phasing

| Phase | Deliverable | Acceptance | Blocked by |
|---|---|---|---|
| 0 | Verify facts: `det_ros`'s actual `librknnrt` version, the input geometry/orientation, `bgr_draw_box` on the current model | §13 Q1–Q3 answered in writing | — |
| 1 | `mower_perception_capture` under Noetic (`rospy` transport A), compressed topics only | 2000+ frames + manifest, quota enforced, `pytest` green | Phase 0 |
| 2 | One `curl :8181/snapshot` grab per camera + first LLM call over LAN | 4 labelled frames, boxes visually sane, coordinates confirmed to be 0–1000 | Phase 1 |
| 3 | Decide front-only vs all-views (§2.2), then full label batch (LLM + current-model pseudo-labels) and `build_dataset.py` | `data.yaml` with exactly 22 classes, pose-split train/val, `provenance.jsonl` | Phase 2 |
| 4 | Train + export (Rockchip fork) + `convert.py` | metadata self-check (§6.4) passes | Phase 3, x86 host |
| 5 | QA gate 7.1 | all thresholds met, baseline documented | Phase 4 |
| 6 | Deploy via §7.2 + live acceptance 7.3 | rollback script tested *before* the swap | Phase 5 |
| 7 | (Optional) Option B own NPU node on core 2 | coexists with `det_ros`, no NPU contention | Phase 6 |

Phase 1 alone is worth doing even if the retrain never happens: it turns "the mower got
something wrong once" into a reproducible frame set.

## 13. Open questions / to confirm on hardware

1. **Which `librknnrt` does `det_ros` actually load?** The tree ships 2.1.0 and 2.2.0 copies,
   the survey says 2.1.0. This decides the `rknn-toolkit2` version we can use (§6.3).
2. **What exactly does `det_ros` do to a frame before inference?** The shipped input is a
   **480-wide × 640-tall portrait** canvas (`[1,3,640,480]` NCHW; feature maps 80×60, 40×30,
   20×15). `/vio/right/image_raw` is 640×480 **landscape**. So either the node transposes /
   rotates, or it centre-crops to portrait, or the "1920×1080" in the configs is not what the
   detector sees. Training data must go through the identical transform. Settle it by running
   the stock model on a frame with one obvious object and checking where the box lands relative
   to the object — a square 20-px box on a person is the tell.
3. **Exact source geometry per topic.** `vision_localization/config/metoak/*.yaml` says
   `image_width: 640, image_height: 480` for `/vio/left` and `/vio/right`, and
   `docs/streaming-and-telemetry.md` agrees; `mower_cameras.yaml` says 1920×1080 for the OA and
   rear cameras. Confirm rather than assume:
   ```bash
   rostopic echo -n1 /vio/right/image_raw | head -20          # height/width/encoding/step
   rostopic echo -n1 /left_oa_camera/image_raw | head -20
   ```
4. **`best_small_0208.rknn` class count.** `best_small_list.txt` says two classes
   (`stone`, `leaf`), but the model metadata shows a **1-channel** score tensor at each scale
   (`[1,1,80,60]`, `[1,1,40,30]`, `[1,1,20,15]`), i.e. one class. The large model
   unambiguously has 22. Before touching `model_small_path_detect`, work out whether the small
   model is single-class or the list file is stale — a retrain that "fixes" the wrong thing
   here is worse than leaving it alone.
5. **Is `/perception/ctrl` or `/seg/set_obstacle_filtering` needed** to see detections, and
   does anything gate `det_ros` off the NPU under load?
6. **What does `/ai/det/box_in_org` (`foxglove_msgs/ImageAnnotations`) actually carry** —
   class IDs or names, and in which coordinate frame (original image or `/vio/right`)? If it
   carries class IDs, it is a free, perfectly-aligned label source for pseudo-labelling and
   removes most of the dependence on the LLM.
7. **Can a container reach the LAN LLM?** `docker-compose.yml` uses `network_mode: host`, so
   it should, but the mower firewall and the Wi-Fi/AP isolation need checking.
8. **Existing bag contents** — do the four bags in `/userdata/RobotData/bag/` carry
   `/vio/right/image_raw` (4× downscaled), or only OA/rear? That decides whether Phase 3 has a
   bootstrap set on day one.
9. **Front-only or all views?** `det_ros` sees front + OA only (§2.2); the rear camera feeds
   `mower_charge`. Deciding this before labelling avoids building a dataset that cannot be
   deployed behind `det_ros` — front-only is the defensible default, all-views requires the
   second node of §8 Option B and its own QA.

## 14. Provenance

Read for this document: `mower_docs/03-yolo-perception.md`,
`mower_docs/10-hardware.md`, `mower_docs/01-system-overview.md`, `mower_docs/02-ros2-migration.md`,
`mower_docs/07-cautions.md`, `mower_docs/09-reuse-strategy.md`,
`ros2_port_handoff/README.md` (hardware ↔ device map),
`ros2_port_handoff/07_system_config/{udev/mower.rules,systemd/mower-webcam.service,mower_cam_keeper.py}`,
`ros2_port_handoff/09_platform/{df,meminfo,dev_nodes,udev-symlinks,lsusb}.txt`,
`ros2_port_handoff/10_ros_live_snapshot/{topics_verbose.txt,services.txt,nodes.txt}`,
`ros2_port_handoff/11_perception_models/*.rknn` (metadata via `strings`),
`src/mower_perception/det_ros/{param/cfg.yaml,model/*_list.txt,CMakeLists.txt}`,
`src/mower_perception/seg_ros/readme.md` (class id maps),
`src/mower_perception/rknn2_runtime/` (both `librknnrt.so` copies),
`mower_drivers/base_cameras/config/mower_cameras.yaml`,
`ros2_port_handoff/13_open_source_upstreams/rknn_model_zoo/examples/yolov8/{README.md,python/convert.py,python/yolov8.py}`,
`docs/streaming-and-telemetry.md`,
`ros2_stack/docs/{deployment.md,README.md,um960.md,control.md}` (house style),
`ros2_stack/src/mower_localization/` (package layout and node style).

Verified first-hand in the repo while writing this: the `.rknn` input/output shapes, class
counts and quant table; the `det_ros` subscription list and default model paths
(`strings devel/det_ros/lib/det_ros/det_ros`); both `librknnrt.so` version strings; the
udev symlinks; `best_large_list.txt` (22 entries) vs `best_small_list.txt` (2 names, 1 score
channel in the model); the camera configs and their `pub_compress_freq` / `bag_img_reduce`
settings; the exact `convert.py` signature and mean/std defaults.

Not verified (needs the hardware): everything in §13, and every command in §3–§7.