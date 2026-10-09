# SPDX-License-Identifier: GPL-3.0-or-later
"""ActionClient that drops unused feedback without an executor dispatch (rclpy Jazzy).

An rclpy ActionClient always has a feedback subscription, and the feedback topic carries the
feedback of EVERY goal of that action, not only ours: bt_navigator publishes NavigateToPose
feedback on each BT tick (up to ~100 Hz) and controller_server FollowPath feedback per control
cycle (20 Hz) for every transit, also the ones another node started. Each message wakes the
MultiThreadedExecutor, which builds a Task, hops through the thread pool and only then finds
that no feedback callback is registered (~1.5-4 ms of Python per message on the RK3588).

Here, while no feedback callback is registered, ready feedback is taken and discarded right in
``is_ready`` (the wait set must still be drained, it is level-triggered) and the waitable only
reports ready for the entities that matter (goal/cancel/result responses, status).

Identical copies live in mower_mission and mower_docking; keep them in sync.
"""

from rclpy.action import ActionClient


class QuietActionClient(ActionClient):

    def is_ready(self, wait_set):
        ready = super().is_ready(wait_set)
        if self._is_feedback_ready and not self._feedback_callbacks:
            fb_type = self._action_type.Impl.FeedbackMessage
            with self._lock:
                for _ in range(64):   # drain what is queued (bounded)
                    if self._client_handle.take_feedback(fb_type) is None:
                        break
            self._is_feedback_ready = False
            ready = (self._is_status_ready or self._is_goal_response_ready
                     or self._is_cancel_response_ready or self._is_result_response_ready)
        return ready
