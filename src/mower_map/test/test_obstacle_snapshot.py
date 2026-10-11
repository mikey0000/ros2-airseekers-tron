# SPDX-License-Identifier: GPL-3.0-or-later
"""ROS-free tests for mower_map.obstacle_snapshot (per-obstacle snapshot files).

Run: python3 -m pytest src/mower_map/test

The image encode/decode paths need python3-opencv; those tests are skipped on a
host without cv2 (the pure file-naming/pruning logic is always exercised).
"""

import os

import numpy as np
import pytest

from mower_map import obstacle_snapshot as snap

cv2 = snap.cv2
needs_cv2 = pytest.mark.skipif(cv2 is None, reason='python3-opencv not installed')


# ----------------------------------------------------------------- naming / pruning

def test_snapshot_filename_and_path():
    assert snap.snapshot_filename(7) == '7.jpg'
    assert snap.snapshot_path('/maps/obstacle_snapshots', 7) == '/maps/obstacle_snapshots/7.jpg'


def test_clear_snapshots_only_removes_our_files(tmp_path):
    (tmp_path / '1.jpg').write_bytes(b'x')
    (tmp_path / '2.jpg').write_bytes(b'x')
    (tmp_path / 'keep.txt').write_bytes(b'x')      # unrelated -> untouched
    (tmp_path / 'areas.dat').write_bytes(b'x')     # unrelated -> untouched
    assert snap.clear_snapshots(str(tmp_path)) == 2
    assert sorted(os.listdir(tmp_path)) == ['areas.dat', 'keep.txt']


def test_prune_snapshots_keeps_listed_ids(tmp_path):
    for i in (1, 2, 3):
        (tmp_path / snap.snapshot_filename(i)).write_bytes(b'x')
    assert snap.prune_snapshots(str(tmp_path), [2]) == 2
    assert sorted(os.listdir(tmp_path)) == ['2.jpg']


def test_prune_snapshots_missing_dir_is_noop(tmp_path):
    assert snap.prune_snapshots(str(tmp_path / 'nope'), []) == 0
    assert snap.remove_snapshot(str(tmp_path / 'nope'), 5) is False


# ----------------------------------------------------------------- encode / save

def _bgr(width=8, height=4):
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:, :, 2] = 40    # constant, compresses well
    return img


@needs_cv2
def test_frame_to_bgr_bgr8_roundtrip():
    img = _bgr()
    out = snap.frame_to_bgr(img.tobytes(), 'bgr8', 8, 4)
    assert out is not None and out.shape == (4, 8, 3)
    assert np.array_equal(out, img)
    assert out.flags.writeable          # must not alias the read-only msg buffer
    assert out.flags.c_contiguous


@needs_cv2
def test_frame_to_bgr_respects_row_step_padding():
    img = _bgr(width=3, height=2)
    padded = np.zeros((2, 6, 3), dtype=np.uint8)   # row_step 18, width*3 = 9 -> padded
    padded[:, :3] = img
    out = snap.frame_to_bgr(padded.tobytes(), 'bgr8', 3, 2, step=18)
    assert out is not None and np.array_equal(out, img)


@needs_cv2
def test_frame_to_bgr_rejects_unknown_encoding_and_bad_size():
    assert snap.frame_to_bgr(b'\x00' * 24, '32FC3', 2, 2) is None
    assert snap.frame_to_bgr(b'', 'bgr8', 8, 4) is None
    assert snap.frame_to_bgr(b'\x00' * 24, 'bgr8', 0, 4) is None


@needs_cv2
def test_encode_jpeg_returns_jpeg_magic():
    jpeg = snap.encode_jpeg(_bgr())
    assert jpeg is not None and jpeg[:2] == b'\xff\xd8'


@needs_cv2
def test_save_snapshot_writes_decodable_jpeg(tmp_path):
    img = _bgr(16, 12)
    img[:, :, 1] = 200
    path = snap.save_snapshot(str(tmp_path), 3, img.tobytes(), 'bgr8', 16, 12, quality=90)
    assert path == str(tmp_path / '3.jpg')
    decoded = cv2.imread(path)
    assert decoded is not None and decoded.shape == (12, 16, 3)
    assert not list(tmp_path.glob('*.tmp'))        # atomic write leaves no temp file


@needs_cv2
def test_save_snapshot_creates_directory_and_overwrites(tmp_path):
    target = tmp_path / 'sub' / 'dir'
    img = _bgr(8, 4)
    assert snap.save_snapshot(str(target), 1, img.tobytes(), 'bgr8', 8, 4) is not None
    img[:, :, 0] = 255
    assert snap.save_snapshot(str(target), 1, img.tobytes(), 'bgr8', 8, 4) is not None
    assert len(list(target.glob('*.jpg'))) == 1


@needs_cv2
def test_save_snapshot_bad_frame_returns_none(tmp_path):
    assert snap.save_snapshot(str(tmp_path), 9, b'', 'bgr8', 8, 4) is None
    assert not list(tmp_path.glob('*.jpg'))


# ----------------------------------------------------------------- sync (node wiring)

def _frame_at(now, width=8, height=4):
    return (now, _bgr(width, height).tobytes(), 'bgr8', width, height, width * 3)


@needs_cv2
def test_sync_snapshots_writes_new_and_prunes_gone(tmp_path):
    saved = set()
    frame = _frame_at(now=100.0)
    assert snap.sync_snapshots(str(tmp_path), [1, 2], saved, frame, 100.0) == [1, 2]
    assert saved == {1, 2}
    assert sorted(os.listdir(tmp_path)) == ['1.jpg', '2.jpg']
    # Obstacle 1 gone, 3 new: write 3, drop 1, keep 2 (already saved).
    assert snap.sync_snapshots(str(tmp_path), [2, 3], saved, _frame_at(101.0), 101.0) == [3]
    assert saved == {2, 3}
    assert sorted(os.listdir(tmp_path)) == ['2.jpg', '3.jpg']


def test_sync_snapshots_stale_frame_writes_nothing_and_keeps_ids(tmp_path):
    (tmp_path / '1.jpg').write_bytes(b'x')
    saved = {1}
    assert snap.sync_snapshots(str(tmp_path), [1, 2], saved, _frame_at(0.0), 5.0, max_age_s=1.0) == []
    assert saved == {1}                       # 2 not saved: frame too old
    assert snap.sync_snapshots(str(tmp_path), [1, 2], saved, None, 6.0) == []
    assert saved == {1}                       # no frame at all: nothing saved


def test_sync_snapshots_prunes_even_without_a_frame(tmp_path):
    for i in (1, 2):
        (tmp_path / snap.snapshot_filename(i)).write_bytes(b'x')
    saved = {1, 2}
    assert snap.sync_snapshots(str(tmp_path), [2], saved, None, 6.0) == []
    assert saved == {2}
    assert sorted(os.listdir(tmp_path)) == ['2.jpg']
