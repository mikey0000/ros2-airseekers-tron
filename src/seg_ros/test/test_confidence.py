"""Tests for ppseg_postprocess.confidence_mask / argmax_mask (2026-10-09).

The non-grass mapper only trusts confident pixels, so the uint8 confidence scale must be exact.
"""
import numpy as np

from seg_ros.ppseg_postprocess import argmax_mask, confidence_mask


def test_dominant_class_saturates():
    """A class 20 logits ahead is ~100% sure; must hit 255 so the 0.8 gate passes."""
    lg = np.zeros((1, 6, 2, 2), np.float32)
    lg[0, 3] = 20.0
    assert (confidence_mask(lg) == 255).all()


def test_equal_logits_give_one_sixth():
    """Uniform logits are maximally unsure: 255/6 -> 42, far below any trust gate."""
    assert (confidence_mask(np.zeros((1, 6, 2, 2), np.float32)) == 42).all()


def test_dtype_and_shape():
    """The node publishes this as a mono8 image, so it must be uint8 (H, W)."""
    out = confidence_mask(np.random.RandomState(0).randn(1, 6, 5, 7).astype(np.float32))
    assert out.dtype == np.uint8 and out.shape == (5, 7)


def test_three_dim_input_accepted():
    """Both [C,H,W] and [1,C,H,W] model outputs occur and must agree."""
    lg = np.random.RandomState(1).randn(6, 4, 3).astype(np.float32)
    a = confidence_mask(lg)
    assert a.shape == (4, 3)
    assert (a == confidence_mask(lg[None])).all()


def test_argmax_matches_softmax_max_and_confidence_value():
    """The class mask and its confidence must describe the same winning class."""
    lg = np.random.RandomState(2).randn(1, 6, 8, 9).astype(np.float32) * 3
    e = np.exp(lg[0] - lg[0].max(axis=0))
    sm = e / e.sum(axis=0)
    assert (argmax_mask(lg) == sm.argmax(axis=0)).all()
    assert (argmax_mask(lg[0]) == argmax_mask(lg)).all()
    assert np.abs(confidence_mask(lg).astype(int) - np.rint(255 * sm.max(axis=0))).max() <= 1
