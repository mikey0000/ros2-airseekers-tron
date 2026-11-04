"""mower_common.frames: planar /tf_static lookups (pure python)."""

import math
from types import SimpleNamespace as NS

import pytest

from mower_common import frames


def test_blade_offset_from_the_static_tree_chains_frames():
    st = {'blade_link': ('base_link', 0.23, 0.0, 0.0)}
    assert frames.blade_offset_from(st) == pytest.approx((0.23, 0.0))
    st = {'blade_link': ('deck', 0.1, 0.0, 0.0), 'deck': ('base_link', 0.2, 0.05, math.pi / 2)}
    assert frames.blade_offset_from(st) == pytest.approx((0.2, 0.15))
    assert frames.blade_offset_from({'blade_link': ('odom', 0, 0, 0)}) is None


def test_static_entry_is_planar_child_parent_xy_yaw():
    q = NS(x=0.0, y=0.0, z=math.sin(0.25), w=math.cos(0.25))
    t = NS(child_frame_id='blade_link', header=NS(frame_id='base_link'),
           transform=NS(translation=NS(x=0.23, y=0.0, z=-0.05), rotation=q))
    child, (parent, x, y, yaw) = frames.static_entry(t)
    assert (child, parent) == ('blade_link', 'base_link')
    assert (x, y, yaw) == pytest.approx((0.23, 0.0, 0.5))
