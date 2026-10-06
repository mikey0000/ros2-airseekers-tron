# Copyright 2026 The mower_docking authors
# SPDX-License-Identifier: Apache-2.0
"""Motion-free check of the docking marker pipeline (rear camera -> ArUco -> base_link).

``docking_server`` only subscribes to the rear camera while a dock goal runs. This tool
runs the very same pipeline (:class:`aruco_detect.ArucoDetector` on
``/rear_camera/image_raw`` + ``camera_info``, :func:`dock_logic.marker_in_base` with the
``rear_camera_T_base`` extrinsics, :func:`dock_logic.marker_gate_ok`) for a fixed time
WITHOUT commanding anything, and reports detection rate, per-frame detector time, the
marker pose in base_link and whether the docking gates (0.5 m / 25 deg) would pass::

    python3 -m mower_docking.marker_check --duration 30
    python3 -m mower_docking.marker_check --duration 30 --params /path/docking.yaml

Owner test (robot parked, marker held / mounted 0.8 m behind the rear axle, facing the
camera): expect ``x ~ -0.8``, ``|y| < 0.1``, ``|yaw| < 10 deg`` and ``gate_ok`` true.
"""
from __future__ import annotations

import argparse
import math
import statistics
import time
from dataclasses import dataclass, field
from typing import List, Optional

from mower_docking import dock_logic as dl

DEFAULTS = {
    'image_topic': '/rear_camera/image_raw',
    'camera_info_topic': '/rear_camera/camera_info',
    'aruco_dictionary': 'DICT_4X4_50',
    'marker_size': 0.04,
    'marker_id': -1,
    'rear_camera_T_base': [-0.201, 0.0, 0.25, -1.5707963, 0.0, 1.5707963],
    'max_detection_rate_hz': 10.0,
    'max_lateral_error': 0.5,
    'max_yaw_error_deg': 25.0,
    'docked_marker_offset': 0.45,
}


@dataclass
class Sample:
    t: float
    obs: Optional[dl.MarkerObs]
    detect_ms: float


@dataclass
class Summary:
    frames: int
    detections: int
    duration_s: float
    detect_rate_hz: float
    frame_rate_hz: float
    detect_ms_mean: float
    detect_ms_max: float
    x: Optional[float] = None
    y: Optional[float] = None
    yaw_deg: Optional[float] = None
    x_std: Optional[float] = None
    y_std: Optional[float] = None
    yaw_std_deg: Optional[float] = None
    lateral: Optional[float] = None
    heading_deg: Optional[float] = None
    remaining: Optional[float] = None
    gate_ok: Optional[bool] = None
    gate_pass_ratio: Optional[float] = None
    marker_ids: List[int] = field(default_factory=list)


def summarize(samples: List[Sample], duration_s: float, docked_offset: float,
              max_lateral: float, max_yaw_deg: float) -> Summary:
    """Pure statistics over a recording (median pose; gate evaluated on the median and
    per detection)."""
    det = [s for s in samples if s.obs is not None]
    ms = [s.detect_ms for s in samples] or [0.0]
    dur = max(duration_s, 1e-9)
    out = Summary(frames=len(samples), detections=len(det), duration_s=duration_s,
                  detect_rate_hz=len(det) / dur, frame_rate_hz=len(samples) / dur,
                  detect_ms_mean=statistics.fmean(ms), detect_ms_max=max(ms))
    if not det:
        return out
    xs = [s.obs.x for s in det]
    ys = [s.obs.y for s in det]
    yaws = [s.obs.yaw for s in det]
    med = dl.MarkerObs(statistics.median(xs), statistics.median(ys), statistics.median(yaws))
    err = dl.dock_errors(med, docked_offset)
    yaw_rad = math.radians(max_yaw_deg)
    passes = sum(dl.marker_gate_ok(dl.dock_errors(s.obs, docked_offset), max_lateral, yaw_rad)
                 for s in det)
    sd = (lambda v: statistics.pstdev(v)) if len(det) > 1 else (lambda v: 0.0)
    out.x, out.y, out.yaw_deg = med.x, med.y, math.degrees(med.yaw)
    out.x_std, out.y_std, out.yaw_std_deg = sd(xs), sd(ys), math.degrees(sd(yaws))
    out.lateral, out.heading_deg = err.lateral, math.degrees(err.heading)
    out.remaining = err.remaining
    out.gate_ok = dl.marker_gate_ok(err, max_lateral, yaw_rad)
    out.gate_pass_ratio = passes / len(det)
    out.marker_ids = sorted({s.obs.marker_id for s in det})
    return out


def format_summary(s: Summary) -> str:
    lines = [f'frames={s.frames} ({s.frame_rate_hz:.1f} Hz processed)  '
             f'detections={s.detections} ({s.detect_rate_hz:.1f} Hz) in {s.duration_s:.1f} s',
             f'detector: mean {s.detect_ms_mean:.1f} ms, max {s.detect_ms_max:.1f} ms per frame']
    if s.detections:
        lines += [f'marker ids {s.marker_ids}; median pose in base_link: '
                  f'x={s.x:+.3f} m (sd {s.x_std:.3f})  y={s.y:+.3f} m (sd {s.y_std:.3f})  '
                  f'yaw={s.yaw_deg:+.1f} deg (sd {s.yaw_std_deg:.1f})',
                  f'dock errors: remaining={s.remaining:+.3f} m  lateral={s.lateral:+.3f} m  '
                  f'heading={s.heading_deg:+.1f} deg  -> gate_ok={s.gate_ok} '
                  f'({100 * s.gate_pass_ratio:.0f} % of detections pass)']
    else:
        lines.append('NO MARKER DETECTED')
    return '\n'.join(lines)


def _load_params(path: Optional[str]) -> dict:
    p = dict(DEFAULTS)
    if path:
        import yaml
        with open(path) as fh:
            data = yaml.safe_load(fh) or {}
        for node in data.values():
            p.update({k: v for k, v in (node.get('ros__parameters') or {}).items() if k in p})
    return p


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--duration', type=float, default=30.0)
    ap.add_argument('--params', default=None, help='docking.yaml (default: package one)')
    ap.add_argument('--save', default=None, help='write the last frame (with detections) here')
    args = ap.parse_args(argv)
    params_file = args.params
    if params_file is None:
        try:
            import os

            from ament_index_python.packages import get_package_share_directory
            params_file = os.path.join(get_package_share_directory('mower_docking'),
                                       'config', 'docking.yaml')
        except Exception:  # noqa: BLE001
            params_file = None
    P = _load_params(params_file)

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    from mower_docking import aruco_detect

    det = aruco_detect.ArucoDetector(P['aruco_dictionary'], float(P['marker_size']))
    ext = [float(v) for v in P['rear_camera_T_base']]
    period = 1.0 / float(P['max_detection_rate_hz']) if P['max_detection_rate_hz'] else 0.0
    state = {'K': None, 'D': None, 'last': 0.0, 'gray': None, 'corners': None}
    samples: List[Sample] = []

    rclpy.init()
    node = rclpy.create_node('marker_check')

    def on_info(msg):
        if len(msg.k) >= 9 and msg.k[0] > 0:
            state['K'], state['D'] = list(msg.k), list(msg.d) or [0.0] * 5

    def on_image(msg):
        now = time.monotonic()
        if state['K'] is None or now - state['last'] < period * 0.9:
            return
        state['last'] = now
        t0 = time.perf_counter()
        gray = aruco_detect.to_gray(msg.encoding, msg.data, msg.height, msg.width, msg.step)
        res = det.detect(gray, state['K'], state['D'], int(P['marker_id']))
        ms = 1000.0 * (time.perf_counter() - t0)
        obs = None
        if res:
            mid, rvec, tvec, corners = res[0]
            obs = dl.marker_in_base(rvec, tvec, ext[0:3], ext[3:6], stamp=now, marker_id=mid)
            state['corners'] = corners
        state['gray'] = gray
        samples.append(Sample(now, obs, ms))

    node.create_subscription(CameraInfo, P['camera_info_topic'], on_info,
                             qos_profile_sensor_data)
    node.create_subscription(Image, P['image_topic'], on_image, qos_profile_sensor_data)
    t_end = None
    t_start = time.monotonic()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        if t_end is None:
            if samples:
                t_start = samples[0].t
                t_end = t_start + args.duration
            elif time.monotonic() - t_start > 10.0:
                print('no image + camera_info within 10 s (K received: %s)'
                      % (state['K'] is not None))
                break
        elif time.monotonic() >= t_end:
            break
    dur = (time.monotonic() - t_start) if samples else 0.0
    node.destroy_node()
    rclpy.try_shutdown()

    s = summarize(samples, dur, float(P['docked_marker_offset']),
                  float(P['max_lateral_error']), float(P['max_yaw_error_deg']))
    print(format_summary(s))
    if args.save and state['gray'] is not None:
        import cv2
        import numpy as np
        img = cv2.cvtColor(state['gray'], cv2.COLOR_GRAY2BGR)
        if state['corners'] is not None:
            cv2.polylines(img, [np.int32(state['corners']).reshape(-1, 1, 2)], True,
                          (0, 0, 255), 2)
        cv2.imwrite(args.save, img)
    return 0 if s.detections else 1


if __name__ == '__main__':
    raise SystemExit(main())
