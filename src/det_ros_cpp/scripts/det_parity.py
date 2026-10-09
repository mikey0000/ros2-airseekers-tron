#!/usr/bin/env python3
"""Parity check: Python det_ros vs C++ det_ros_cpp on the same saved frames.

  det_parity.py capture --topic /left_oa_camera/image_raw --n 10 --out /userdata/ros2/parity
  det_parity.py compare --frames /userdata/ros2/parity [--core 2]

``compare`` runs the Python path in-process (mower_rknn letterbox + RknnRunner +
det_ros.yolo_postprocess.post_process, exactly what det_ros_node._detect does) and the
C++ node as a subprocess (topics remapped to /parity/*, so it can run next to the live
stack), feeds it every frame and matches the detections (same class, IoU) per frame.
Run inside the mower_jazzy container with the workspace sourced.
"""
import argparse
import glob
import os
import subprocess
import sys
import time

import numpy as np


def _ros():
    import rclpy
    from rclpy.node import Node
    return rclpy, Node


def capture(a):
    import cv2
    rclpy, Node = _ros()
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    n = Node('det_parity_capture')
    br, got = CvBridge(), []

    def cb(m):
        if len(got) < a.n and (not got or time.monotonic() - got[-1] > a.every):
            img = br.imgmsg_to_cv2(m, 'bgr8')
            p = os.path.join(a.out, f'{os.path.basename(a.topic.strip("/").split("/")[0])}_{len(got):03d}.png')
            cv2.imwrite(p, img)
            got.append(time.monotonic())
            print('saved', p)
    n.create_subscription(Image, a.topic, cb, rclpy.qos.qos_profile_sensor_data)
    t0 = time.monotonic()
    while len(got) < a.n and time.monotonic() - t0 < 60:
        rclpy.spin_once(n, timeout_sec=0.2)
    rclpy.shutdown()


def iou(a, b):
    if max(abs(x - y) for x, y in zip(a, b)) < 0.5:
        return 1.0  # identical (also degenerate zero-area boxes clamped to the image edge)
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / u if u > 0 else 0.0


def py_detect(runner, bgr, classes, a):
    from mower_rknn import bgr_to_rgb_nhwc, letterbox
    from det_ros.labels import label_for
    from det_ros.yolo_postprocess import post_process
    size = (480, 640)
    padded, scale, (px, py) = letterbox(bgr, size)
    outs = runner.run([bgr_to_rgb_nhwc(padded)[None, ...]])
    boxes, cls, scores = post_process(outs, img_size=size, obj_thresh=a.obj, nms_thresh=a.nms)
    h, w = bgr.shape[:2]
    res = []
    if boxes is not None:
        inv = 1.0 / scale
        for b, c, s in zip(boxes, cls, scores):
            x1 = min(max((b[0] - px) * inv, 0.0), w); y1 = min(max((b[1] - py) * inv, 0.0), h)
            x2 = min(max((b[2] - px) * inv, 0.0), w); y2 = min(max((b[3] - py) * inv, 0.0), h)
            res.append((label_for(int(c), classes), float(s), (x1, y1, x2, y2)))
    return res


def compare(a):
    import cv2
    rclpy, Node = _ros()
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    from vision_msgs.msg import Detection2DArray
    from mower_rknn import RknnRunner
    from det_ros.labels import BEST_LARGE_CLASSES

    frames = sorted(glob.glob(os.path.join(a.frames, '*.png')))
    if not frames:
        sys.exit(f'no *.png in {a.frames}')
    runner = RknnRunner(a.model, core_mask=a.core)
    py = {f: py_detect(runner, cv2.imread(f), BEST_LARGE_CLASSES, a) for f in frames}
    runner.release()

    from ament_index_python.packages import get_package_prefix
    exe = os.path.join(get_package_prefix('det_ros_cpp'), 'lib', 'det_ros_cpp', 'det_ros_cpp')
    # the binary itself (not `ros2 run`, whose child survives terminate())
    cmd = [exe, '--ros-args',
           '-r', '__node:=det_parity_cpp',
           '-r', '/ai/det/detections:=/parity/detections',
           '-r', '/ai/det/image_annotated:=/parity/image_annotated',
           '-p', f'model_path:={a.model}', '-p', 'left_topic:=/parity/image_raw',
           '-p', "right_topic:=''", '-p', "extra_topics:=['']", '-p', f"core_masks:=['{a.core}']",
           '-p', 'max_rate_hz:=0.0', '-p', f'obj_thresh:={a.obj}', '-p', f'nms_thresh:={a.nms}']
    proc = subprocess.Popen(cmd)
    try:
        _feed(a, frames, py, proc)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()


def _feed(a, frames, py, proc):
    import cv2
    rclpy, Node = _ros()
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    from vision_msgs.msg import Detection2DArray
    rclpy.init()
    n = Node('det_parity')
    br = CvBridge()
    got = {}
    n.create_subscription(Detection2DArray, '/parity/detections',
                          lambda m: got.__setitem__(m.header.stamp.sec, m), 10)
    pub = n.create_publisher(Image, '/parity/image_raw', rclpy.qos.qos_profile_sensor_data)
    t0 = time.monotonic()
    while pub.get_subscription_count() == 0 and time.monotonic() - t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.2)
    time.sleep(3.0)
    worst = 0.0
    total = matched = 0
    for i, f in enumerate(frames):
        m = br.cv2_to_imgmsg(cv2.imread(f), 'bgr8')
        m.header.stamp.sec = i + 1
        m.header.frame_id = 'parity'
        for _ in range(5):  # best effort: resend until answered
            pub.publish(m)
            t1 = time.monotonic()
            while i + 1 not in got and time.monotonic() - t1 < 2.0:
                rclpy.spin_once(n, timeout_sec=0.05)
            if i + 1 in got:
                break
        cpp = [(d.results[0].hypothesis.class_id, d.results[0].hypothesis.score,
                (d.bbox.center.position.x - d.bbox.size_x / 2, d.bbox.center.position.y - d.bbox.size_y / 2,
                 d.bbox.center.position.x + d.bbox.size_x / 2, d.bbox.center.position.y + d.bbox.size_y / 2))
               for d in got[i + 1].detections] if i + 1 in got else None
        if cpp is None:
            print(f'{os.path.basename(f)}: NO C++ RESULT')
            continue
        used = set()
        line = []
        for lab, s, b in py[f]:
            total += 1
            best = max(((iou(b, cb), j) for j, (cl, cs, cb) in enumerate(cpp)
                        if cl == lab and j not in used), default=(0.0, -1))
            if best[0] >= a.iou:
                matched += 1
                used.add(best[1])
                worst = max(worst, abs(s - cpp[best[1]][1]))
                line.append(f'{lab} {s:.3f}/{cpp[best[1]][1]:.3f} iou={best[0]:.3f}')
            else:
                line.append(f'{lab} {s:.3f} UNMATCHED py={tuple(round(v, 1) for v in b)} cpp='
                            + str([tuple(round(v, 1) for v in cb) for cl, cs, cb in cpp if cl == lab]))
        extra = len(cpp) - len(used)
        print(f'{os.path.basename(f)}: py={len(py[f])} cpp={len(cpp)} extra_cpp={extra} | '
              + '; '.join(line))
        total += extra
    print(f'PARITY: matched {matched}/{total} (IoU>={a.iou}), max |score diff| {worst:.4f}')
    rclpy.shutdown()
    if matched != total:
        raise SystemExit(1)


def main():
    p = argparse.ArgumentParser()
    sp = p.add_subparsers(dest='cmd', required=True)
    c = sp.add_parser('capture')
    c.add_argument('--topic', default='/left_oa_camera/image_raw')
    c.add_argument('--n', type=int, default=10)
    c.add_argument('--every', type=float, default=1.0)
    c.add_argument('--out', default='/userdata/ros2/parity')
    k = sp.add_parser('compare')
    k.add_argument('--frames', default='/userdata/ros2/parity')
    k.add_argument('--model', default='/userdata/ros2/models/best_large_0208.rknn')
    k.add_argument('--core', default='2')
    k.add_argument('--obj', type=float, default=0.25)
    k.add_argument('--nms', type=float, default=0.45)
    k.add_argument('--iou', type=float, default=0.9)
    a = p.parse_args()
    capture(a) if a.cmd == 'capture' else compare(a)


if __name__ == '__main__':
    main()
