"""Model-path resolution shared by ``det_ros`` / ``seg_ros``.

On the mower the ``.rknn`` files live on the persistent ``/userdata`` partition
(``/userdata/ros2/models/``, installed by ``scripts/install_models.sh`` and bind-mounted
into the container by ``docker/docker-compose.yml``). For dev hosts / alternative layouts a
``models_dir`` can be supplied; it is tried with the basename of ``model_path``.

Pure Python (no ROS, no NPU) so it is unit-testable anywhere.
"""

from __future__ import annotations

import os
from typing import List, Optional

DEVICE_MODELS_DIR = '/userdata/ros2/models'


def candidate_model_paths(model_path: str, models_dir: str = '') -> List[str]:
    """Ordered list of paths to try for ``model_path``.

    1. ``model_path`` itself (absolute, or relative to the CWD),
    2. ``models_dir/<basename(model_path)>`` when ``models_dir`` is set,
    3. ``DEVICE_MODELS_DIR/<basename(model_path)>`` as the last resort.
    Duplicates are removed, order preserved.
    """
    out: List[str] = []
    base = os.path.basename(model_path) if model_path else ''
    for cand in (model_path,
                 os.path.join(models_dir, base) if models_dir and base else '',
                 os.path.join(DEVICE_MODELS_DIR, base) if base else ''):
        if cand and cand not in out:
            out.append(cand)
    return out


def resolve_model_path(model_path: str, models_dir: str = '') -> Optional[str]:
    """First existing file from :func:`candidate_model_paths`, else ``None``."""
    for cand in candidate_model_paths(model_path, models_dir):
        if os.path.isfile(cand):
            return cand
    return None


def rknn_available() -> bool:
    """True when ``rknnlite`` (rknn-toolkit-lite2) imported successfully."""
    from . import rknn_runner
    return rknn_runner._HAVE_RKNN  # noqa: SLF001


def rknn_import_error() -> str:
    from . import rknn_runner
    return '' if rknn_runner._HAVE_RKNN else str(getattr(rknn_runner, '_IMPORT_ERROR', ''))
