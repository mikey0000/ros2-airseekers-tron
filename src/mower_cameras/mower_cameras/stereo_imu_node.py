"""``mower_cameras/stereo_imu`` — Metoak stereo IMU (ICM-40608, i2c 8-0068) -> ``/stereo_imu/data``.

Clean-room path, no Metoak SDK: the kernel ``inv_icm42600`` driver (Metoak build,
``/lib/kernel/driver/inv-icm42600*.ko``) in IIO mode. ``mo_init.sh`` loads it with
``Repot_m=1`` (= samples go to a vendor NETLINK socket, NO IIO device; that is why the IMU
looked "loaded but not attached"). ``scripts/setup_stereo_host.sh`` reloads it with
``Repot_m=0`` -> ``icm40608-gyro`` + ``icm40608-accel`` IIO devices. This node then:

* configures both devices through sysfs (rate, full-scale, x/y/z + timestamp channels,
  buffer length, watermark) and streams the IIO character-device buffers (``/dev/iio:deviceN``,
  FIFO-backed, no sysfs polling);
* pairs gyro + accel samples by their (identical) FIFO timestamps;
* maps the kernel timestamp (CLOCK_MONOTONIC with the Metoak ``TimeStamp_m=1``; detected
  at start) to ROS time exactly like ``stereo_cam`` maps the V4L2 buffer stamp, so the
  IMU and the stereo pair share one clock;
* subtracts a gyro bias measured over the first still window (``bias_window_s``); the
  accelerometer is published raw (gravity included, REP-145), bias left to the estimator.

Publishes ``sensor_msgs/Imu`` (frame ``stereo_camera_imu``, the chip's own axes; the URDF
places that frame with the vendor's factory IMU<->cam0 extrinsic), no orientation
(covariance[0] = -1), and ``~/bias`` (Vector3Stamped, rad/s) latched once found.
"""
from __future__ import annotations

import os
import select
import threading
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

from mower_cameras import iio_imu
from mower_cameras.stereo_pair import ClockMap

KIND_PREFIX = {'gyro': 'anglvel', 'accel': 'accel'}


def _w(path, value):
    with open(path, 'w') as f:
        f.write(str(value))


def _r(path):
    with open(path) as f:
        return f.read().strip()


def _closest(avail_text, wanted):
    vals = [float(v) for v in avail_text.split()]
    return min(vals, key=lambda v: abs(v - float(wanted)))


class IioStream:
    """One configured IIO device + its buffer fd."""

    def __init__(self, dev, kind, rate, scale, length, watermark, root=iio_imu.IIO_ROOT):
        self.dev, self.kind = dev, kind
        base = os.path.join(root, dev)
        pre = KIND_PREFIX[kind]
        _w(f'{base}/buffer/enable', 0)
        _w(f'{base}/sampling_frequency',
           _closest(_r(f'{base}/sampling_frequency_available'), rate))
        if scale > 0:
            _w(f'{base}/in_{pre}_scale', _closest(_r(f'{base}/in_{pre}_scale_available'), scale))
        se = f'{base}/scan_elements'
        chans = []
        for f in sorted(os.listdir(se)):
            if not f.endswith('_en'):
                continue
            ch = f[:-3]
            on = ch in (f'in_{pre}_x', f'in_{pre}_y', f'in_{pre}_z', 'in_timestamp')
            _w(f'{se}/{f}', int(on))
            if on:
                chans.append((ch, _r(f'{se}/{ch}_index'), _r(f'{se}/{ch}_type')))
        self.dtype, self.types = iio_imu.scan_layout(chans)
        self.names = [f'in_{pre}_{a}' for a in 'xyz']
        self.scale = float(_r(f'{base}/in_{pre}_scale'))
        self.rate = float(_r(f'{base}/sampling_frequency'))
        _w(f'{base}/buffer/length', int(length))
        _w(f'{base}/buffer/watermark', int(watermark))
        _w(f'{base}/buffer/enable', 1)
        self.base = base
        self.fd = os.open(f'/dev/{dev}', os.O_RDONLY | os.O_NONBLOCK)
        self._rest = b''
        self.stamper = iio_imu.BatchStamper(1e9 / self.rate)

    def read(self):
        """-> (timestamps ns int64 array, Nx3 float64 SI array) for the whole records read."""
        try:
            chunk = os.read(self.fd, self.dtype.itemsize * 256)
        except BlockingIOError:
            return None
        buf = self._rest + chunk
        n = len(buf) // self.dtype.itemsize * self.dtype.itemsize
        self._rest = buf[n:]
        if not n:
            return None
        d = iio_imu.decode_records(buf[:n], self.dtype, self.types)
        import numpy as np
        xyz = np.stack([d[k] for k in self.names], axis=1).astype(np.float64) * self.scale
        # the driver stamps a whole FIFO interrupt group with the IRQ time: re-stamp per sample
        ts, vals = self.stamper.feed(d['in_timestamp'], xyz.tolist())
        return (ts, vals) if len(ts) else None

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass
        try:
            _w(f'{self.base}/buffer/enable', 0)
        except OSError:
            pass


class StereoImuNode(Node):
    def __init__(self):
        super().__init__('stereo_imu')
        d = self.declare_parameter
        d('device_names', ['icm40608', 'icm42600', 'icm42605', 'icm42602', 'icm42622'])
        d('rate', 200.0)                 # Hz, nearest of 12.5..4000
        d('gyro_scale', 0.000266316)     # rad/s per LSB: +-500 dps (0 = keep driver default)
        d('accel_scale', 0.002394202)    # m/s^2 per LSB: +-8 g
        d('watermark', 4)                # samples per wake-up (latency = watermark / rate)
        d('frame_id', 'stereo_camera_imu')
        d('topic', '/stereo_imu/data')
        d('calibrate_gyro_bias', True)
        d('bias_window_s', 3.0)
        d('bias_max_std', 0.004)         # rad/s; a window noisier than this = moving
        # Noise densities measured at rest on the mower 2026-10-07 (200 Hz): see docs/vio.md.
        d('gyro_noise_std', 0.0010)      # rad/s per sample
        d('accel_noise_std', 0.02)       # m/s^2 per sample
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.p = {n: p(n) for n in ('device_names', 'rate', 'gyro_scale', 'accel_scale',
                                    'watermark', 'frame_id')}
        # best effort like sensor data, but a deep history: samples leave in FIFO bursts of
        # `watermark`, and depth 5 dropped ~8 % of them at the writer/reader.
        imu_qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Imu, p('topic'), imu_qos)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.bias_pub = self.create_publisher(Vector3Stamped, '~/bias', latched)
        self.bias = (iio_imu.RestBias(int(float(p('bias_window_s')) * float(p('rate'))),
                                      float(p('bias_max_std')))
                     if p('calibrate_gyro_bias') else None)
        gv, av = float(p('gyro_noise_std')) ** 2, float(p('accel_noise_std')) ** 2
        self.gcov = [gv, 0.0, 0.0, 0.0, gv, 0.0, 0.0, 0.0, gv]
        self.acov = [av, 0.0, 0.0, 0.0, av, 0.0, 0.0, 0.0, av]
        # pre-serialised Imu: ~10x cheaper than building sensor_msgs/Imu at 200 Hz in Python
        self.cdr = iio_imu.ImuCdr(self.p['frame_id'], self.gcov, self.acov)
        self.fast = True
        self.count = 0
        self.stop_evt = threading.Event()
        self.thread = threading.Thread(target=self._run, name='iio:stereo_imu', daemon=True)
        self.thread.start()
        self._t_stats = time.monotonic()
        self.create_timer(30.0, self._log_stats)

    # ------------------------------------------------------------------ streaming
    def _open(self):
        devs = iio_imu.find_devices(self.p['device_names'])
        if set(devs) != {'gyro', 'accel'}:
            raise OSError('no ICM-406xx/426xx IIO devices (driver in netlink mode? run '
                          'scripts/setup_stereo_host.sh on the host)')
        rate, wm = float(self.p['rate']), int(self.p['watermark'])
        streams = [IioStream(devs['gyro'], 'gyro', rate, float(self.p['gyro_scale']), 512, wm),
                   IioStream(devs['accel'], 'accel', rate, float(self.p['accel_scale']), 512,
                             wm)]
        self.get_logger().info(
            f'stereo_imu: gyro {devs["gyro"]} accel {devs["accel"]} @ {streams[0].rate:g} Hz, '
            f'scale gyro {streams[0].scale:g} rad/s accel {streams[1].scale:g} m/s^2, '
            f'record {streams[0].dtype.itemsize} B, watermark {wm}')
        return streams

    def _run(self):
        backoff = 1.0
        while not self.stop_evt.is_set():
            try:
                streams = self._open()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'stereo_imu: {exc}; retry in {backoff:.0f}s')
                self.stop_evt.wait(backoff)
                backoff = min(backoff * 2, 30.0)
                continue
            backoff = 1.0
            try:
                self._stream(streams)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'stereo_imu: stream failed: {exc}')
            finally:
                for s in streams:
                    s.close()
            self.stop_evt.wait(1.0)

    def _stream(self, streams):
        gyro, accel = streams
        pairer = iio_imu.ImuPairer(tol_ns=int(0.25e9 / max(gyro.rate, 1.0)))
        clock = None
        cmap = ClockMap()
        last_pub = 0
        min_dt = int(0.5e9 / max(gyro.rate, 1.0))
        last_data = time.monotonic()
        while not self.stop_evt.is_set():
            r, _, _ = select.select([gyro.fd, accel.fd], [], [], 1.0)
            if not r:
                if time.monotonic() - last_data > 3.0:
                    raise OSError('no IMU data for 3 s')
                continue
            last_data = time.monotonic()
            for s in streams:
                if s.fd not in r:
                    continue
                got = s.read()
                if got is None:
                    continue
                ts, xyz = got
                add = pairer.add_gyro if s is gyro else pairer.add_accel
                for t, v in zip(ts.tolist(), xyz):
                    add(t, v)
            out = pairer.pop()
            if not out:
                continue
            if clock is None:
                clock = iio_imu.choose_clock(out[-1][0], time.monotonic_ns(), time.time_ns())
                self.get_logger().info(f'stereo_imu: IIO timestamps are CLOCK_{clock.upper()}')
            for t, g, a in out:
                if self.bias is not None and self.bias.bias is None:
                    if self.bias.add(g):
                        self._publish_bias(t if clock == 'realtime' else cmap.to_ros(t))
                    continue        # do not publish an uncalibrated gyro
                if self.bias is not None:
                    b = self.bias.bias
                    g = (g[0] - b[0], g[1] - b[1], g[2] - b[2])
                stamp = t if clock == 'realtime' else cmap.to_ros(t)
                if stamp < last_pub + min_dt:   # rare kernel near-duplicate stamp
                    stamp = last_pub + min_dt
                last_pub = stamp
                self._publish(stamp, g, a)

    def _publish(self, stamp_ns, g, a):
        if self.fast:
            try:
                self.pub.publish(self.cdr.serialize(stamp_ns, g, a))
                self.count += 1
                return
            except TypeError:           # rclpy without publish(bytes)
                self.fast = False
        m = Imu()
        sec, nsec = divmod(int(stamp_ns), 1_000_000_000)
        m.header.stamp.sec, m.header.stamp.nanosec = sec, nsec
        m.header.frame_id = self.p['frame_id']
        m.orientation_covariance[0] = -1.0
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = g
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = a
        m.angular_velocity_covariance = self.gcov
        m.linear_acceleration_covariance = self.acov
        self.pub.publish(m)
        self.count += 1

    def _publish_bias(self, stamp_ns):
        b = self.bias.bias
        msg = Vector3Stamped()
        sec, nsec = divmod(int(stamp_ns), 1_000_000_000)
        msg.header.stamp.sec, msg.header.stamp.nanosec = sec, nsec
        msg.header.frame_id = self.p['frame_id']
        msg.vector.x, msg.vector.y, msg.vector.z = (float(v) for v in b)
        self.bias_pub.publish(msg)
        self.get_logger().info(
            f'stereo_imu: gyro bias {b[0]:+.5f} {b[1]:+.5f} {b[2]:+.5f} rad/s '
            f'(std {self.bias.std.max():.5f}, rejected windows {self.bias.windows_rejected})')

    def _log_stats(self):
        now = time.monotonic()
        dt, self._t_stats = now - self._t_stats, now
        self.get_logger().info(f'stereo_imu: {self.count / dt:.1f} Hz')
        self.count = 0

    def destroy_node(self):
        self.stop_evt.set()
        self.thread.join(timeout=3.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StereoImuNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
