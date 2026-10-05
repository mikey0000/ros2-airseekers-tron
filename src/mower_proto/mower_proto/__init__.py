"""Generated protobuf bindings for the Airseekers Tron mower protocol.

Regenerated from ``mower_docs/reference/proto/*.proto`` (the vendor's canonical
definitions) using ``protoc``. The ``.proto`` files carry ``package mower.*`` but
protoc flattens that namespace for Python — every message lives in its own
``*_pb2.py`` module here, imported as top-level-relative modules.

Usage::

    from mower_proto import teleop_pb2, status_pb2, map_pb2
    mode = teleop_pb2.TeleopMode()
    mode.type = teleop_pb2.TeleopMode.START

Regenerate with ``./generate_proto.sh`` (see that file for the protoc invocation
and the package-relative import fix-up).
"""
from mower_proto import (  # noqa: F401
    common_pb2,
    config_pb2,
    init_pb2,
    live_pb2,
    log_pb2,
    map_pb2,
    monitor_pb2,
    msg_pb2,
    rtk_pb2,
    security_pb2,
    status_pb2,
    task_pb2,
    teleop_pb2,
    upgrade_mcu_pb2,
    upgrade_pb2,
    webshell_agent_pb2,
)

__all__ = [m[0] for m in
           (("common_pb2",), ("config_pb2",), ("init_pb2",), ("live_pb2",),
            ("log_pb2",), ("map_pb2",), ("monitor_pb2",), ("msg_pb2",),
            ("rtk_pb2",), ("security_pb2",), ("status_pb2",), ("task_pb2",),
            ("teleop_pb2",), ("upgrade_mcu_pb2",), ("upgrade_pb2",),
            ("webshell_agent_pb2",))]
