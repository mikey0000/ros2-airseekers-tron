"""Drive-motor load detector: high wheel current with no ground progress (pure logic, no ROS).

Evidence and limits: docs/analysis/2026-10-22_wheel_scale_and_motor_current.md (A4).

* ``MotorInfo.current`` of the drive boards reaches the host once per second (the MOTORS
  frame rides with battery/version), so this is a confirmation signal, not a fast one.
* Steady forward driving: |L| + |R| median 8.4 A, p99 16.5 A, p99.9 20 A (mcu_node SI units,
  raw read as mA; absolute scale unverified, the thresholds are empirical).
* The 2026-10-21c 06:37 wheel-spin (reverse arc into soft ground): 23-31 A with RTK speed
  0.01 m/s against 0.12 commanded; this rule fires at 06:36:58.8, 2.0 s before the 4 s
  stuck guard. 0 other hits in that 73 min mow; 1 in the 21b stuck session (a 25 A reverse
  at 01:25:38, RTK 0.11 m/s against 0.15).
* It does NOT pre-empt bumper stops: the load rises <= 0.3-1 s before the bumper, within the
  normal forward spread, and the 1 Hz frames miss most of it.
* Pivots (|v_cmd| below ``min_cmd``) are out of scope: they draw 15-16 A on average.

Feed :meth:`on_motor` with each NEW drive-motor frame and :meth:`on_motion` with the commanded
linear speed and the measured ground speed (RTK / EKF) at any rate; :meth:`update` returns an
event dict when the rule fires (once per ``holdoff_s``).
"""

from collections import deque


class DriveLoadDetector:
    def __init__(self, stall_sum_a=20.0, min_cmd=0.08, max_ground_ratio=0.3, window_s=1.0,
                 confirm_frames=1, holdoff_s=10.0, stale_s=2.5, spin_rpm=2.0):
        self.stall_sum_a = stall_sum_a          # A, |left| + |right|
        self.min_cmd = min_cmd                  # m/s mean |commanded linear| over window_s
        self.max_ground_ratio = max_ground_ratio  # ground speed below this x commanded
        self.window_s = window_s
        self.confirm_frames = confirm_frames    # consecutive over-threshold motor frames
        self.holdoff_s = holdoff_s
        self.stale_s = stale_s                  # motor frame older than this: no decision
        self.spin_rpm = spin_rpm                # mean |wheel rpm| above this = 'spin', else 'stall'
        self.reset()

    def reset(self):
        self._motion = deque()
        self._motor = None                      # (t, left_a, right_a, left_rpm, right_rpm)
        self._run = 0
        self._last_fire = None
        self._pending = False

    def on_motion(self, t, v_cmd, ground_speed):
        self._motion.append((t, abs(v_cmd), abs(ground_speed)))
        while self._motion and self._motion[0][0] < t - self.window_s:
            self._motion.popleft()

    def on_motor(self, t, left_a, right_a, left_rpm=None, right_rpm=None):
        """One NEW MotorInfo frame (call it only when the frame changed / was received)."""
        self._motor = (t, left_a, right_a, left_rpm, right_rpm)
        self._pending = True

    def update(self, t):
        if not self._pending or self._motor is None:
            return None
        self._pending = False
        mt, la, ra, lr, rr = self._motor
        if t - mt > self.stale_s or not self._motion:
            self._run = 0
            return None
        n = len(self._motion)
        vc = sum(m[1] for m in self._motion) / n
        vg = sum(m[2] for m in self._motion) / n
        load = abs(la) + abs(ra)
        hit = load > self.stall_sum_a and vc >= self.min_cmd and vg < self.max_ground_ratio * vc
        self._run = self._run + 1 if hit else 0
        if self._run < self.confirm_frames:
            return None
        if self._last_fire is not None and t - self._last_fire < self.holdoff_s:
            return None
        self._last_fire = t
        kind = 'stall'
        if lr is not None and rr is not None and (abs(lr) + abs(rr)) / 2.0 > self.spin_rpm:
            kind = 'spin'
        return {'kind': kind, 'load_a': round(load, 1), 'cmd_mps': round(vc, 3),
                'ground_mps': round(vg, 3), 'left_a': la, 'right_a': ra}
