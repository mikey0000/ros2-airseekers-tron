"""Smoke test: the generated *_pb2 modules import and round-trip a message."""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

pytest.importorskip('google.protobuf', reason='python3-protobuf is not installed')


def test_teleop_mode_round_trip():
    from mower_proto import teleop_pb2

    mode = teleop_pb2.TeleopMode()
    mode.type = teleop_pb2.TeleopMode.START
    parsed = teleop_pb2.TeleopMode.FromString(mode.SerializeToString())
    assert parsed.type == teleop_pb2.TeleopMode.START
