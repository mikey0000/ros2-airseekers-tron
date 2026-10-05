"""Common NPU-node startup: resolve the model, check the runtime, honour ``dry_run``.

``det_ros`` and ``seg_ros`` both call :func:`prepare_runner` from their constructor:

* everything OK            -> returns a loaded :class:`RknnRunner`;
* runtime/model missing and ``dry_run`` -> logs one clear warning, returns ``None``
  (the node stays alive, subscribes to nothing and publishes nothing);
* runtime/model missing and not ``dry_run`` -> raises :class:`NpuStartupError`, which
  the node's ``main()`` turns into a one-line fatal log + exit code 1 (no traceback).
"""

from __future__ import annotations

from typing import Optional

from .model_paths import (candidate_model_paths, resolve_model_path, rknn_available,
                          rknn_import_error)
from .rknn_runner import RknnRunner, RknnUnavailable

HINT_RUNTIME = ('install rknn-toolkit-lite2 2.3.0 (aarch64, cp310) in the container and '
                'bind-mount librknnrt.so at /usr/lib/librknnrt.so; see '
                'docs/cameras_and_video.md. Set dry_run:=true to keep the node up without NPU.')
HINT_MODEL = ('copy the models with scripts/install_models.sh (device path '
              '/userdata/ros2/models) or pass models_dir:=<dir>.')


class NpuStartupError(RuntimeError):
    """The node cannot run inference and ``dry_run`` is false."""


def startup_problem(model_path: str, models_dir: str = '') -> tuple:
    """Return ``(resolved_path_or_None, problem_str_or_None)`` without loading anything."""
    resolved = resolve_model_path(model_path, models_dir)
    if not rknn_available():
        return resolved, (f'rknnlite (rknn-toolkit-lite2) is not importable '
                          f'({rknn_import_error()}): {HINT_RUNTIME}')
    if resolved is None:
        tried = ', '.join(candidate_model_paths(model_path, models_dir))
        return None, f'model file not found (tried: {tried}): {HINT_MODEL}'
    return resolved, None


def prepare_runner(logger, model_path: str, models_dir: str, core_mask: str,
                   dry_run: bool) -> Optional[RknnRunner]:
    resolved, problem = startup_problem(model_path, models_dir)
    if problem is None:
        try:
            runner = RknnRunner(resolved, core_mask=core_mask)
            logger.info(f'loaded {resolved} on NPU core_mask={core_mask}')
            return runner
        except RknnUnavailable as exc:
            problem = f'NPU init failed for {resolved}: {exc}'
    if dry_run:
        logger.warning(f'dry_run: no inference, publishing nothing ({problem})')
        return None
    raise NpuStartupError(problem)
