"""Class lists of the shipped YOLOv8 models (order fixed by the export).

``best_large_0208.rknn``: 22 classes (vendor ``det_ros/model/best_large_list.txt``).
``best_small_0208.rknn``: the class tensors in the .rknn metadata are 1 channel wide
(``[1,1,80,60]``), i.e. a single-class model; its label is not recovered, so indices are
published for it unless ``classes`` is set.
"""

BEST_LARGE_CLASSES = [
    'person', 'dog', 'cat', 'sports ball', 'hedgehog', 'rabbit', 'stone', 'hoe',
    'shovel', 'manhole', 'brick', 'trashbin', 'toycars', 'potted plant', 'cans',
    'bottle', 'book', 'backpack', 'wood', 'chair', 'trunk', 'dock',
]


def default_classes(model_path: str):
    """Built-in class list for a known model basename, else ``[]``."""
    import os
    if os.path.basename(model_path).startswith('best_large'):
        return list(BEST_LARGE_CLASSES)
    return []


def label_for(cls_idx: int, classes) -> str:
    return classes[cls_idx] if 0 <= cls_idx < len(classes) else str(cls_idx)


def annotated_topic_for(image_topic: str) -> str:
    """``/left_oa_camera/image_raw`` -> ``/left_oa_camera/image_annotated``.

    The annotated image lives next to the source image in the camera namespace; a
    topic without a namespace gets ``<topic>_annotated``.
    """
    topic = '/' + image_topic.strip('/')
    ns, _, _name = topic.rpartition('/')
    if not ns:
        return topic + '_annotated'
    return ns + '/image_annotated'
