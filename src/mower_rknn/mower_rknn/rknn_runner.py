"""``rknn_runner`` — a small, forgiving wrapper around ``rknn-toolkit-lite2``.

The mower ships ``librknnrt`` 2.1.0 (see `mower_docs/10-hardware.md`) and the deployment
toolkit is ``rknn-toolkit-lite2`` (on-device). We use ``rknnlite.api.RKNNLite`` here, not
the host-side ``rknn.api.RKNN`` (that one needs the full toolkit + a PC).

The import is guarded so the package imports on any host (CI / non-NPU dev box) and only
fails at inference time with a clear message instead of an ImportError.
"""

from __future__ import annotations

import logging
from typing import Dict, Optional

import numpy as np

log = logging.getLogger(__name__)


class RknnUnavailable(RuntimeError):
    """Raised when the RKNN Lite runtime cannot be imported/initialised."""


try:  # pragma: no cover - only importable on the RK3588S target
    from rknnlite.api import RKNNLite as _RKNNLite

    _HAVE_RKNN = True
except Exception as _e:  # noqa: BLE001
    _RKNNLite = None
    _HAVE_RKNN = False
    _IMPORT_ERROR = _e


# NPU core bitmask values (same numbering as the C API: 0=1, 1=2, 2=4).
# Exposed as a plain dict so it does not require rknnlite at import time.
CORE_MASK: Dict[str, int] = {
    "0": 1,
    "1": 2,
    "2": 4,
    "0_1": 3,
    "0_2": 5,
    "1_2": 6,
    "0_1_2": 7,
}


class RknnRunner:
    """Load and run a ``.rknn`` model on one (or more) NPU cores."""

    def __init__(self, model_path: str, core_mask: Optional[str] = None,
                 verbose: bool = False):
        if not _HAVE_RKNN:
            raise RknnUnavailable(
                "rknn-toolkit-lite2 is not importable on this host "
                f"({_IMPORT_ERROR}). Run inside the aarch64 Humble container on "
                "the mower (librknnrt 2.1.0).")

        self.model_path = model_path
        self.verbose = verbose
        self._core = self._resolve_core(core_mask)
        self._rknn = self._load()

    # ------------------------------------------------------------------ core

    @staticmethod
    def _resolve_core(core_mask: Optional[str]) -> Optional[int]:
        if core_mask is None:
            return None
        key = str(core_mask).strip()
        if key not in CORE_MASK:
            log.warning("unknown core_mask %r, using default (all cores)", core_mask)
            return None
        return CORE_MASK[key]

    # ------------------------------------------------------------------ model

    def _load(self) -> "_RKNNLite":
        rknn = _RKNNLite(verbose=self.verbose)
        ret = rknn.load_rknn(self.model_path)
        if ret != 0:
            raise RknnUnavailable(f"load_rknn({self.model_path!r}) failed, ret={ret}")

        kwargs: Dict[str, object] = {}
        if self._core is not None:
            kwargs["core_mask"] = self._core
        try:
            ret = rknn.init_runtime(**kwargs)
        except TypeError:  # older lite builds have no core_mask kwarg
            ret = rknn.init_runtime()
        if ret != 0:
            raise RknnUnavailable(f"init_runtime failed, ret={ret}")
        return rknn

    # ------------------------------------------------------------------ infer

    def run(self, inputs) -> list[np.ndarray]:
        """Run inference. ``inputs`` is a single np array or a list of them.

        Returns the list of output tensors as numpy arrays (already dequantised to
        float by the lite runtime when the model is int8).
        """
        if not isinstance(inputs, (list, tuple)):
            inputs = [inputs]
        outputs = self._rknn.inference(inputs=inputs)
        if outputs is None:
            raise RknnUnavailable("inference returned None (model not loaded?)")
        return outputs

    # ------------------------------------------------------------------ cleanup

    def release(self) -> None:
        if self._rknn is not None:
            try:
                self._rknn.release()
            finally:
                self._rknn = None

    def __enter__(self) -> "RknnRunner":
        return self

    def __exit__(self, *exc) -> None:
        self.release()
