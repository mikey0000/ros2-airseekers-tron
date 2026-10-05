"""``metoak_sdk`` — thin ctypes shim over the clean-room ``libmetoak.so``.

This is the ``source_mode: sdk`` capture path for `stereo_vio_bridge`. It binds
the reconstructed C API in ``metoak_reimpl/include/metoak.h`` (compiled from
``metoak_reimpl/user/metoak.c``) instead of OpenCV's ``VideoCapture``, so the
stereo front is captured through the open SDK replacement rather than the
closed ``libMoGeneralSDK``.

The API surface used here is the VIO-relevant subset::

    moLocalCreateCamera / moLocalOpenCamera
    moLocalGetOneFrame    (combined 1280x480 ISP stereo frame)
    moLocalSplitRGBFrame  (-> 2x 640x480 mono8 left/right)
    moLocalGetIMUData     (TDK ICM-40608 via IIO sysfs)
    moLocalCloseCamera / moLocalDeleteCamera

Build the library first (on the target, arm64)::

    cd metoak_reimpl/user && make          # -> libmetoak.so
"""
import ctypes
from ctypes import (
    byref, c_char_p, c_double, c_float, c_int32, c_uint8, c_uint16, c_uint32,
    c_uint64, c_void_p, POINTER,
)

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None


# --- struct layouts (must match metoak.h; DWARF-verified offsets) ---------
class MoFrame(ctypes.Structure):
    _fields_ = [
        ("TimeStamp", c_uint64),           # +0
        ("FrameId", c_uint64),             # +8
        ("StreamType", c_uint32),          # +16
        ("SensorId", c_uint32),            # +20
        ("ResolutionWidth", c_uint16),     # +24
        ("ResolutionHeight", c_uint16),    # +26
        ("BitsPerPixel", c_uint32),        # +28
        ("BytesUsed", c_uint32),           # +32
        ("FillLightStatus", c_int32),      # +36
        ("FillLightBrightness", c_int32),  # +40
        ("RGBImageType", c_int32),         # +44
        ("Reserved", c_uint8 * 64),        # +48
        ("Data", c_void_p),                # +112
    ]                                       # size 120


class MoIMUData(ctypes.Structure):
    _fields_ = [
        ("Timestamp", c_uint64),
        ("Temperature", c_float),
        ("LinearAccelerationX", c_float),
        ("LinearAccelerationY", c_float),
        ("LinearAccelerationZ", c_float),
        ("AngularVelocityX", c_float),
        ("AngularVelocityY", c_float),
        ("AngularVelocityZ", c_float),
    ]                                       # size 40


class MetoakSdk:
    """Load ``libmetoak.so`` and expose the VIO-relevant capture path."""

    def __init__(self, lib_path=None):
        if np is None:
            raise RuntimeError("metoak_sdk requires numpy")
        path = lib_path or "libmetoak.so"
        self.lib = ctypes.CDLL(path)
        self._bind()
        self.handle = self.lib.moLocalCreateCamera(b"Mp021", None)
        if not self.handle:
            raise RuntimeError("moLocalCreateCamera returned NULL")
        if self.lib.moLocalOpenCamera(self.handle) != 0:
            self.lib.moLocalDeleteCamera(self.handle)
            self.handle = None
            raise RuntimeError("moLocalOpenCamera failed (is /dev/video22 up? run mo_init.sh)")

    def _bind(self):
        lib = self.lib
        lib.moLocalCreateCamera.restype = c_void_p
        lib.moLocalCreateCamera.argtypes = [c_char_p, c_char_p]
        lib.moLocalOpenCamera.restype = c_int32
        lib.moLocalOpenCamera.argtypes = [c_void_p]
        lib.moLocalCloseCamera.restype = c_int32
        lib.moLocalCloseCamera.argtypes = [c_void_p]
        lib.moLocalDeleteCamera.restype = c_int32
        lib.moLocalDeleteCamera.argtypes = [c_void_p]
        lib.moLocalGetOneFrame.restype = c_int32
        lib.moLocalGetOneFrame.argtypes = [c_void_p, POINTER(MoFrame)]
        lib.moLocalSplitRGBFrame.restype = c_int32
        lib.moLocalSplitRGBFrame.argtypes = [POINTER(MoFrame), POINTER(MoFrame), POINTER(MoFrame)]
        lib.moLocalReleaseFrame.restype = c_int32
        lib.moLocalReleaseFrame.argtypes = [POINTER(MoFrame)]
        lib.moLocalGetIMUData.restype = c_int32
        lib.moLocalGetIMUData.argtypes = [c_void_p, POINTER(MoIMUData)]

    def _frame_to_array(self, fr):
        """Copy a ``MoFrame``'s buffer into a fresh mono8 numpy array."""
        if not fr.Data or not fr.BytesUsed:
            return None
        w, h = fr.ResolutionWidth, fr.ResolutionHeight
        buf = ctypes.string_at(fr.Data, fr.BytesUsed)
        arr = np.frombuffer(buf, dtype=np.uint8).copy()
        return arr.reshape((h, w)) if w and h else arr

    def grab(self):
        """Return ``(left, right)`` 640x480 mono8 numpy arrays, or ``None``."""
        fr = MoFrame()
        left = MoFrame()
        right = MoFrame()
        try:
            if self.lib.moLocalGetOneFrame(self.handle, byref(fr)) != 0:
                return None
            if self.lib.moLocalSplitRGBFrame(byref(fr), byref(left), byref(right)) != 0:
                return None
            l = self._frame_to_array(left)
            r = self._frame_to_array(right)
            return (l, r)
        finally:
            if left.Data:
                self.lib.moLocalReleaseFrame(byref(left))
            if right.Data:
                self.lib.moLocalReleaseFrame(byref(right))

    def get_imu(self):
        """Return ``(ax, ay, az, gx, gy, gz)`` in SI units, or ``None``."""
        d = MoIMUData()
        if self.lib.moLocalGetIMUData(self.handle, byref(d)) != 0:
            return None
        return (d.LinearAccelerationX, d.LinearAccelerationY, d.LinearAccelerationZ,
                d.AngularVelocityX, d.AngularVelocityY, d.AngularVelocityZ)

    def close(self):
        if self.handle:
            self.lib.moLocalCloseCamera(self.handle)
            self.lib.moLocalDeleteCamera(self.handle)
            self.handle = None
