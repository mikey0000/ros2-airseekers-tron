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
