# mower_proto

Generated Python protobuf bindings for the Airseekers Tron mower protocol.

Regenerated from `mower_docs/reference/proto/*.proto` (the vendor's canonical definitions;
identical to `ros2_port_handoff/03_ros_interfaces/mower_proto/`). `protoc` flattens the
`package mower.*` namespace for Python, so each file becomes a sibling `*_pb2.py` module.

```python
from mower_proto import teleop_pb2, status_pb2, map_pb2

mode = teleop_pb2.TeleopMode()
mode.type = teleop_pb2.TeleopMode.START
payload = mode.SerializeToString()   # b'\x08\x01'

rtt = status_pb2.RTT()
rtt.ParseFromString(payload)
```

## Regenerate

```bash
./mower_proto/generate_proto.sh
```

The script runs `protoc --python_out` over all `.proto` files and rewrites the emitted
top-level `import X_pb2` statements into package-relative `from . import X_pb2`.

The packages consumed by `base_ble` and the MQTT bridge are `common`, `map`, `teleop`,
`init`, `status` (which transitively pulls in `config`, `task`, `rtk`, `upgrade`,
`upgrade_mcu`). `msg` carries the app command enum used by the cloud bridge.
