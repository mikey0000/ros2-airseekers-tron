# mower_rknn — shared RK3588S NPU helpers

Thin wrapper over Rockchip's on-device runtime (`rknn-toolkit-lite2`). Used by `det_ros`
and `seg_ros`. Also: model-path resolution (`resolve_model_path`: device path
`/userdata/ros2/models/<file>`, then `models_dir`) and the shared startup policy
(`prepare_runner`: clear fatal error, or `dry_run`).

## Install (inside the Jazzy aarch64 container)

Version: the shipped `.rknn` models were compiled with toolkit **2.3.0** (stock rootfs
`librknnrt` is 2.1.0, the vendor workspace bundles 2.2.0). `docker/Dockerfile.jazzy`
installs the 2.3.0 wheel from `airockchip/rknn-toolkit2` (guarded):

```bash
pip3 install https://github.com/airockchip/rknn-toolkit2/raw/v2.3.0/rknn-toolkit-lite2/packages/rknn_toolkit_lite2-2.3.0-cp310-cp310-manylinux_2_17_aarch64.manylinux2014_aarch64.whl
```

The wheel does not contain `librknnrt.so`; bind-mount one at `/usr/lib/librknnrt.so`
(see `docs/cameras_and_video.md`; `scripts/install_models.sh --runtime` fetches 2.3.0).

The import in `rknn_runner.py` is guarded: on a non-NPU host (CI / x86 dev box) the
package still imports and raises a clear `RknnUnavailable` only when a model is actually
loaded.

## API

```python
from mower_rknn import RknnRunner, letterbox, bgr_to_rgb_nhwc

with RknnRunner("model/best_large_0208.rknn", core_mask="0") as runner:
    outputs = runner.run([tensor_nhwc_uint8])  # list of float np arrays
```

- `core_mask`: `"0"`, `"1"`, `"2"`, `"0_1"`, `"0_1_2"`, ... (NPU core bitmask).
- Inputs are raw **uint8 RGB NHWC** (no normalisation) — the int8 models bake the
  `/255` (det) or `(x-127.5)/127.5` (seg) input scaling into the quantised weights, so
  the runtime handles it (see `mower_docs/03-yolo-perception.md`).

## Model input shapes (recovered from `.rknn` metadata)

| Model | static shape (NCHW) | W × H |
|---|---|---|
| `best_large/small_0208.rknn` | `[1,3,640,480]` | 480 × 640 |
| `pplite-seg_20260630-1-6cls.rknn` | `[1,3,480,640]` | 640 × 480 |

Note the det and seg models are transposed relative to each other (det is portrait,
seg is landscape). These were read straight from the `.rknn` files in
`ros2_port_handoff/11_perception_models/`; confirm with the first on-hardware inference.
