# SPDX-License-Identifier: GPL-3.0-or-later
"""Per-obstacle snapshot files for the auto obstacle marker.

When :mod:`mower_map.obstacle_tracker_node` first sees a new obstacle cluster
it saves the *front annotated camera frame* (the det_ros overlay image) to a
JPEG keyed by the tracker's obstacle id::

    <snapshot_dir>/<id>.jpg

The GUI (``mowglinext``) serves those files back at
``/api/mowglinext/obstacles/<id>/snapshot`` so the tracked-obstacle panel can
show what the mower actually saw. The node clears the directory on startup
because obstacle ids restart at 1 every run, and drops a file again when its
track disappears.

Everything here is ROS-free (one optional ``cv2`` import) so it can be unit
tested on a host without a ROS install; when ``cv2`` is missing snapshotting is
simply a no-op.
"""

from __future__ import annotations

import os

import numpy as np

try:  # python3-opencv is a runtime dependency of the node, but keep the helper importable without it
    import cv2
except Exception:  # noqa: BLE001 - a bare ImportError is not the only failure mode on some builds
    cv2 = None

_SUFFIX = '.jpg'

# sensor_msgs/Image encodings we know how to turn into a JPEG, mapped to their
# channel count. Anything else is rejected (the annotated feed is always bgr8).
_CHANNELS = {
    'bgr8': 3,
    'rgb8': 3,
    '8uc3': 3,
    'mono8': 1,
    '8uc1': 1,
}


def snapshot_filename(obstacle_id) -> str:
    """``7`` -> ``"7.jpg"``."""
    return '%d%s' % (int(obstacle_id), _SUFFIX)


def snapshot_path(directory, obstacle_id) -> str:
    """Full path of the snapshot for ``obstacle_id`` inside ``directory``."""
    return os.path.join(str(directory), snapshot_filename(obstacle_id))


def frame_to_bgr(data, encoding, width, height, step=0):
    """Decode a raw ``sensor_msgs/Image`` buffer into an (h, w, 3) BGR array.

    Returns ``None`` when ``cv2`` is unavailable or the encoding/size is not
    understood, so a bad frame never raises into the node's timer callback.
    """
    if cv2 is None or not data:
        return None
    ch = _CHANNELS.get(str(encoding or '').lower())
    if ch is None or width <= 0 or height <= 0:
        return None
    row = int(step) or int(width) * ch
    if row < int(width) * ch:
        row = int(width) * ch
    try:
        buf = np.frombuffer(data, dtype=np.uint8)
    except (TypeError, ValueError):
        return None
    if buf.size < row * int(height):
        return None
    rows = buf[:row * int(height)].reshape(int(height), row)[:, :int(width) * ch]
    img = rows.reshape(int(height), int(width), ch)
    if ch == 1:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    elif not str(encoding or '').lower().startswith('bgr'):
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if not img.flags.writeable:      # bgr8 stays a view into the (read-only) msg buffer
        img = img.copy()
    return np.ascontiguousarray(img)


def encode_jpeg(bgr, quality=80):
    """Encode a BGR array to JPEG bytes, or ``None`` on any failure."""
    if cv2 is None or bgr is None:
        return None
    try:
        ok, buf = cv2.imencode('.jpg', bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    except cv2.error:
        return None
    return buf.tobytes() if ok else None


def save_snapshot(directory, obstacle_id, data, encoding, width, height,
                  quality=80, step=0):
    """Encode ``data`` and write it atomically to ``<directory>/<id>.jpg``.

    Returns the written path, or ``None`` if the frame could not be encoded or
    the write failed (caller retries on a later frame).
    """
    jpeg = encode_jpeg(frame_to_bgr(data, encoding, width, height, step), quality)
    if jpeg is None:
        return None
    final = snapshot_path(directory, obstacle_id)
    tmp = final + '.tmp'
    try:
        os.makedirs(str(directory), exist_ok=True)
        with open(tmp, 'wb') as fh:
            fh.write(jpeg)
        os.replace(tmp, final)
    except OSError:
        return None
    return final


def remove_snapshot(directory, obstacle_id) -> bool:
    """Delete ``<directory>/<id>.jpg``. ``True`` if a file was removed."""
    try:
        os.remove(snapshot_path(directory, obstacle_id))
        return True
    except OSError:
        return False


def prune_snapshots(directory, keep_ids) -> int:
    """Remove every ``<id>.jpg`` whose id is not in ``keep_ids``.

    Only files matching our exact naming are touched, so an unrelated file
    dropped into the directory is left alone. Returns the number removed.
    """
    keep = {int(i) for i in keep_ids}
    try:
        names = os.listdir(directory)
    except OSError:
        return 0
    removed = 0
    for name in names:
        if not name.endswith(_SUFFIX):
            continue
        stem = name[:-len(_SUFFIX)]
        if not stem.isdigit() or int(stem) in keep:
            continue
        try:
            os.remove(os.path.join(str(directory), name))
            removed += 1
        except OSError:
            pass
    return removed


def clear_snapshots(directory) -> int:
    """Remove every snapshot in ``directory`` (called once at node startup)."""
    return prune_snapshots(directory, ())


def sync_snapshots(directory, current_ids, saved_ids, frame, now,
                   max_age_s=1.0, quality=80):
    """Bring the on-disk snapshots in line with the live obstacle tracks.

    Writes ``<id>.jpg`` for every id in ``current_ids`` that is not already in
    ``saved_ids`` (using the latest annotated ``frame`` when it is fresh) and
    deletes files whose id is no longer tracked. ``saved_ids`` is mutated in
    place. ``frame`` is the ``(monotonic, data, encoding, width, height, step)``
    tuple received from the annotated image topic, or ``None``.

    Returns the list of ids newly written to disk (for logging).
    """
    current = {int(i) for i in current_ids}
    fresh = frame is not None and now - frame[0] <= max_age_s
    written = []
    if fresh:
        for tid in current - saved_ids:
            if save_snapshot(directory, tid, frame[1], frame[2], frame[3], frame[4],
                             quality=quality, step=frame[5]):
                saved_ids.add(tid)
                written.append(tid)
    for tid in saved_ids - current:
        remove_snapshot(directory, tid)
        saved_ids.discard(tid)
    return written
