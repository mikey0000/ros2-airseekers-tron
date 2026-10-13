#!/usr/bin/env bash
# Regenerate the Python protobuf bindings in mower_proto/ from the vendor .proto files.
#
#   ./generate_proto.sh
#
# The canonical source is mower_docs/reference/proto/*.proto (identical to the copy
# under ros2_port_handoff/03_ros_interfaces/mower_proto/). protoc flattens the
# `package mower.*` namespace for Python and emits top-level `import common_pb2`
# statements, which we rewrite into package-relative `from . import common_pb2` so the
# generated files work as a normal Python package.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"          # .../mower_proto/mower_proto
PKG_DIR="$HERE"                                               # the python package dir
REPO_ROOT="$(cd "$HERE/../../../.." && pwd)"                  # repo root
PROTO_DIR="$REPO_ROOT/mower_docs/reference/proto"

# REQUIRED: protoc matching the runtime. The checked-in files were generated with protoc
# 3.12.4; they import on Noble's `python3-protobuf` 3.21.12 too (gencode <= runtime).
# To regenerate, run inside the Noble dev container (`protobuf-compiler` 3.21.12,
# /usr/bin/protoc) with the whole repo mounted (the proto sources live outside ros2_stack/).
# Both 3.12.x and 3.21.x produce output that imports on Noble; do NOT generate with a
# protoc newer than the runtime. Note ROS ships a newer protoc (ortools_vendor) earlier on
# PATH, so we default to /usr/bin/protoc; override with PROTOC=...
PROTOC="${PROTOC:-/usr/bin/protoc}"
command -v "$PROTOC" >/dev/null 2>&1 || { echo "protoc not found: $PROTOC" >&2; exit 1; }
case "$("$PROTOC" --version)" in
  "libprotoc 3.12."*|"libprotoc 3.21."*) ;;
  *) echo "need protoc 3.12.x (old images) or 3.21.x (Noble), got: $("$PROTOC" --version)" >&2; exit 1 ;;
esac

"$PROTOC" -I "$PROTO_DIR" -I /usr/include \
    --python_out="$PKG_DIR" \
    "$PROTO_DIR"/*.proto

# protoc emits `import common_pb2 as common__pb2`; make them package-relative.
python3 - "$PKG_DIR" <<'PY'
import glob, pathlib, re, sys
pat = re.compile(r'^import (\w+_pb2) as (\w+__pb2)$')
for f in glob.glob(str(pathlib.Path(sys.argv[1]) / '*_pb2.py')):
    p = pathlib.Path(f)
    lines = p.read_text().split('\n')
    out = []
    for l in lines:
        m = pat.match(l)
        out.append(("from . import %s as %s" % (m.group(1), m.group(2))) if m else l)
    p.write_text('\n'.join(out))
PY

echo "regenerated $(ls "$PKG_DIR"/*_pb2.py | wc -l) pb2 modules in $PKG_DIR"
