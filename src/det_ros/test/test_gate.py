# SPDX-License-Identifier: GPL-3.0-or-later
"""Rear-camera NPU gating in DetRosNode (no NPU: node built with object.__new__)."""
import logging
import types

import pytest

from det_ros.det_ros_node import DetRosNode

TOPICS = ['/rear/image_raw', '/rear2/image_raw']


class FakeNode(DetRosNode):
    """DetRosNode with the rclpy bits stubbed so only the gating logic runs."""

    def __init__(self, idle=0.0, max_rate=5.0):  # noqa: super().__init__ deliberately skipped
        self.max_rate_hz = max_rate
        self.gate_open = False
        self.gated_idle_rate = idle
        self.created = []
        self.destroyed = []
        self._gated_subs = {}
        for t in TOPICS:
            w = types.SimpleNamespace(gated=True, offer=lambda m: None)
            self._gated_subs[t] = (w, self._mk(t) if idle > 0 else None)
        self.created.clear()

    def _mk(self, topic):
        sub = object()
        self.created.append(topic)
        return sub

    def create_subscription(self, msg_type, topic, cb, qos):
        return self._mk(topic)

    def destroy_subscription(self, sub):
        self.destroyed.append(sub)

    def get_logger(self):
        return logging.getLogger('fake')


def worker(gated):
    return types.SimpleNamespace(gated=gated)


def test_rate_open_gate_is_max():
    n = FakeNode(idle=1.0)
    n.gate_open = True
    assert n.rate_for(worker(True)) == 5.0


def test_rate_closed_with_idle_is_min():
    assert FakeNode(idle=1.0).rate_for(worker(True)) == 1.0
    assert FakeNode(idle=10.0).rate_for(worker(True)) == 5.0


def test_rate_closed_without_idle_is_max():
    assert FakeNode(idle=0.0).rate_for(worker(True)) == 5.0


def test_rate_non_gated_always_max():
    n = FakeNode(idle=1.0)
    assert n.rate_for(worker(False)) == 5.0
    n.gate_open = True
    assert n.rate_for(worker(False)) == 5.0


def test_open_creates_subscriptions():
    n = FakeNode(idle=0.0)
    n.set_gate(True)
    assert n.gate_open
    assert sorted(n.created) == sorted(TOPICS)
    assert all(sub is not None for _, sub in n._gated_subs.values())


def test_close_destroys_when_no_idle_rate():
    n = FakeNode(idle=0.0)
    n.set_gate(True)
    subs = [s for _, s in n._gated_subs.values()]
    n.set_gate(False)
    assert not n.gate_open
    assert n.destroyed == subs
    assert all(sub is None for _, sub in n._gated_subs.values())


def test_close_keeps_subscriptions_with_idle_rate():
    n = FakeNode(idle=1.0)
    n.set_gate(True)
    n.set_gate(False)
    assert n.destroyed == []
    assert n.created == []  # existing idle subscriptions reused, none recreated
    assert all(sub is not None for _, sub in n._gated_subs.values())


@pytest.mark.parametrize('idle', [0.0, 1.0])
def test_repeated_set_gate_is_noop(idle):
    n = FakeNode(idle=idle)
    n.set_gate(False)  # already closed
    assert n.created == [] and n.destroyed == []
    n.set_gate(True)
    created = list(n.created)
    n.set_gate(True)
    assert n.created == created and n.destroyed == []
