"""YOLOv8 (Rockchip export) post-processing.

The shipped ``best_*_0208.rknn`` models are airockchip/ultralytics_yolov8 exports with
the **9-output** layout: 3 detection scales, each contributing three tensors —
``box`` (DFL distribution), ``class`` (per-class logits), ``score_sum`` (ReduceSum
objectness). This is the exact layout `mower_docs/03-yolo-perception.md` describes.

This is a faithful port of ``rknn_model_zoo/examples/yolov8/python/yolov8.py``
``post_process``, with the ``dfl`` softmax reimplemented in numpy (no torch dependency)
and ``IMG_SIZE`` parameterised.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np

OBJ_THRESH = 0.25
NMS_THRESH = 0.45


def _dfl(position: np.ndarray) -> np.ndarray:
    """Distribution Focal Loss (DFL) softmax over the ``reg_max`` bins.

    ``position`` is ``[1, 4*reg_max, h, w]`` -> ``[1, 4, h, w]`` (soft-argmax over bins).
    """
    n, c, h, w = position.shape
    p_num = 4
    mc = c // p_num
    y = position.reshape(n, p_num, mc, h, w)
    y = np.exp(y - y.max(axis=2, keepdims=True))  # stable softmax over axis=2
    y = y / y.sum(axis=2, keepdims=True)
    acc = np.arange(mc, dtype=np.float32).reshape(1, 1, mc, 1, 1)
    return (y * acc).sum(axis=2)


def _box_process(position: np.ndarray, img_size: Tuple[int, int]) -> np.ndarray:
    """Convert a DFL box tensor into ``[1, 4, h, w]`` absolute ``xyxy`` coordinates.

    ``img_size`` is ``(width, height)`` of the model input. The shipped model is
    480 wide x 640 tall (see `det_ros/config/det.yaml`), so per-axis strides are
    computed explicitly rather than assuming a square input (the upstream example
    only works for square inputs).
    """
    img_w, img_h = img_size
    grid_h, grid_w = position.shape[2:4]
    col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
    grid_x = col.reshape(1, 1, grid_h, grid_w).astype(np.float32)
    grid_y = row.reshape(1, 1, grid_h, grid_w).astype(np.float32)
    stride_x = img_w / grid_w
    stride_y = img_h / grid_h

    d = _dfl(position)  # [1, 4, grid_h, grid_w] -> (x1, y1, x2, y2) offsets
    x1 = (grid_x + 0.5 - d[:, 0:1, :, :]) * stride_x
    y1 = (grid_y + 0.5 - d[:, 1:2, :, :]) * stride_y
    x2 = (grid_x + 0.5 + d[:, 2:3, :, :]) * stride_x
    y2 = (grid_y + 0.5 + d[:, 3:4, :, :]) * stride_y
    return np.concatenate((x1, y1, x2, y2), axis=1)  # [1, 4, grid_h, grid_w]


def _filter_boxes(boxes, box_confidences, box_class_probs, obj_thresh):
    box_confidences = box_confidences.reshape(-1)
    candidate, class_num = box_class_probs.shape

    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)

    pos = np.where(class_max_score * box_confidences >= obj_thresh)
    scores = (class_max_score * box_confidences)[pos]
    return boxes[pos], classes[pos], scores


def _nms_boxes(boxes, scores, nms_thresh):
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]

    areas = w * h
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])
        xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

        w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
        h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
        inter = w1 * h1

        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= nms_thresh)[0]
        order = order[inds + 1]
    return np.array(keep, dtype=np.int64)


def post_process(input_data: List[np.ndarray],
                 img_size: Tuple[int, int] = (480, 640),
                 obj_thresh: float = OBJ_THRESH,
                 nms_thresh: float = NMS_THRESH
                 ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
    """Run the full YOLOv8 head, returning ``(boxes, classes, scores)``.

    ``boxes`` are ``xyxy`` in model-input pixel space (i.e. still letterboxed) —
    the caller maps them back into the original frame with the letterbox scale/pad.

    Returns ``(None, None, None)`` when nothing passes the threshold.
    """
    default_branch = 3
    pair_per_branch = len(input_data) // default_branch

    boxes, classes_conf, scores = [], [], []
    for i in range(default_branch):
        boxes.append(_box_process(input_data[pair_per_branch * i], img_size))
        classes_conf.append(input_data[pair_per_branch * i + 1])
        # The third tensor per branch is the ReduceSum "score_sum" (objectness). The
        # reference Python post-process treats class score as confidence and ignores
        # objectness (it is ~1 for the airockchip export); kept for parity.
        scores.append(np.ones_like(
            input_data[pair_per_branch * i + 1][:, :1, :, :], dtype=np.float32))

    def sp_flatten(_in):
        ch = _in.shape[1]
        _in = _in.transpose(0, 2, 3, 1)
        return _in.reshape(-1, ch)

    boxes = [sp_flatten(v) for v in boxes]
    classes_conf = [sp_flatten(v) for v in classes_conf]
    scores = [sp_flatten(v) for v in scores]

    boxes = np.concatenate(boxes)
    classes_conf = np.concatenate(classes_conf)
    scores = np.concatenate(scores)

    boxes, classes, scores = _filter_boxes(boxes, scores, classes_conf, obj_thresh)

    nboxes, nclasses, nscores = [], [], []
    for c in set(classes):
        inds = np.where(classes == c)
        b = boxes[inds]
        cl = classes[inds]
        s = scores[inds]
        keep = _nms_boxes(b, s, nms_thresh)
        if len(keep) != 0:
            nboxes.append(b[keep])
            nclasses.append(cl[keep])
            nscores.append(s[keep])

    if not nclasses and not nscores:
        return None, None, None

    return (np.concatenate(nboxes),
            np.concatenate(nclasses).astype(np.int64),
            np.concatenate(nscores))
