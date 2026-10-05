"""Airseekers Tron BLE framing.

Faithful re-implementation of the framing from the surviving ``base_ble`` sources
``src/mower_drivers/base_ble/src/protocol_{read,write}.cc`` (see the vendor spec
``mower_docs/reference/proto/ble.md`` — "BLE protocol V2.3.0").

Frame layout (big-endian)::

    A5 | id len data... [| id len data...] | checksum | 5A

* ``id`` (1 byte) + ``len`` (1 byte) + ``data`` (``len`` bytes) make one **field**;
  a frame can carry several fields (e.g. ``84`` + ``85`` for a drive command).
* ``checksum`` = 8-bit sum of all id/len/data bytes, **before** escaping.
* "Escaping" applies to the field bytes between head and tail::

      0xA5 -> 0xF5 0x01      0x5A -> 0xF5 0x02      0xF5 -> 0xF5 0x03

Only the field bytes are escaped on write (the checksum is appended verbatim, exactly
as ``protocol_write.cc`` does); the read side then re-derives the checksum from the
unescaped content.

All command ids already carry the high bit (``0x80``–``0x9a``), so the ``id | 0x80``
that ``ProtocolWrite::WritePack`` applies to outgoing fields is a no-op today but is
kept here for fidelity.
"""

PROTOCOL_HEAD = 0xA5
PROTOCOL_END = 0x5A
ESCAPE_F5 = 0xF5

# byte -> (escape_prefix, escape_code)
_ESCAPE = {
    PROTOCOL_HEAD: (0xF5, 0x01),
    PROTOCOL_END: (0xF5, 0x02),
    ESCAPE_F5: (0xF5, 0x03),
}
_UNESCAPE = {code: byte for byte, (prefix, code) in _ESCAPE.items()}
_ESCAPE_PREFIX = 0xF5


class CmdId:
    """BLE command ids (all >= 0x80; high bit already set)."""

    HEARTBEAT = 0x80      # mower.RTT, echoed back
    WIFI_SSID = 0x81      # raw ssid bytes (paired with WIFI_PWD)
    WIFI_PWD = 0x82       # raw password bytes
    IP = 0x83             # (unused in the node binary)
    DRIVE_LINEAR = 0x84   # 1 signed byte, /100 -> m/s
    DRIVE_ANGULAR = 0x85  # 1 signed byte, /100 -> rad/s
    POSITION = 0x86       # PosData (deprecated, not dispatched)
    RTK_CONFIG = 0x87     # struct {uint16 addr; uint8 channel}
    CERT_ACTIVATE = 0x88  # struct {char host[16]; char token[16]}
    GET_INFO = 0x89       # request device info
    ERROR_CODE = 0x90     # sent BY the mower
    INIT = 0x91           # mower.init.Config (region)
    MAP_MODE = 0x92       # mower.BleMappingMode
    MAP_POINT = 0x93      # mower.MapPoint (deprecated)
    GET_VERSION = 0x94    # -> mower.VersionRsp
    CLEAR_FAULT = 0x95    # -> /clear_estop
    TELEOP = 0x96         # mower.teleop.TeleopMode
    MAP_POSE = 0x98       # mower.MapPose (dock / undock / RTK)
    LOC_STATUS = 0x9a     # -> mower.RtkStatusRsp (getRobotPose; doc listed 0x99)


def checksum8(data: bytes) -> int:
    """8-bit sum of ``data`` (the id/len/data bytes, before escaping)."""
    return sum(data) & 0xFF


def escape(data: bytes) -> bytes:
    """Escape ``A5``/``5A``/``F5`` per the protocol (write side)."""
    out = bytearray()
    for b in data:
        if b in _ESCAPE:
            prefix, code = _ESCAPE[b]
            out.append(prefix)
            out.append(code)
        else:
            out.append(b)
    return bytes(out)


def unescape(data: bytes) -> bytes:
    """Undo escaping (read side). A lone ``0xF5`` not followed by a valid code is kept
    as-is (the checksum byte is never escaped, so this is expected)."""
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        b = data[i]
        if b == _ESCAPE_PREFIX and i + 1 < n and data[i + 1] in _UNESCAPE:
            out.append(_UNESCAPE[data[i + 1]])
            i += 2
        else:
            out.append(b)
            i += 1
    return bytes(out)


def build_frame(fields) -> bytes:
    """Build a complete wire frame from an iterable of ``(id, data)`` fields.

    Mirrors ``ProtocolWrite::WritePack``/``Write``: field bytes escaped, checksum over
    the unescaped field bytes appended verbatim, wrapped in ``A5 ... 5A``.
    """
    raw = bytearray()
    for fid, data in fields:
        raw.append(fid | 0x80)  # mower sets the high bit (no-op for our ids)
        raw.append(len(data))
        raw.extend(data)
    body = escape(bytes(raw))
    return bytes([PROTOCOL_HEAD]) + body + bytes([checksum8(bytes(raw)), PROTOCOL_END])


def parse_frame(frame: bytes):
    """Parse one complete ``A5 ... 5A`` frame -> list of ``(id, data)``.

    Returns ``None`` on a bad head/tail, a checksum mismatch, or a malformed field.
    """
    if len(frame) < 5 or frame[0] != PROTOCOL_HEAD or frame[-1] != PROTOCOL_END:
        return None
    content = unescape(frame[1:-1])  # [id, len, data... , checksum]
    if len(content) < 2:
        return None
    if checksum8(content[:-1]) != content[-1]:
        return None

    fields = []
    i = 0
    payload_end = len(content) - 1  # index of the checksum byte
    while i + 2 <= payload_end:
        fid = content[i]
        flen = content[i + 1]
        data = content[i + 2:i + 2 + flen]
        if len(data) != flen:
            return None  # truncated field
        fields.append((fid, bytes(data)))
        i += 2 + flen
    return fields


def extract_frames(buf: bytes):
    """Split a raw byte buffer into ``(complete_frames, leftover)``.

    Frames are delimited by ``0xA5``/``0x5A``; a head with no trailing tail is kept in
    ``leftover`` for the next read, and any bytes before the first ``0xA5`` are dropped
    (matching ``ProtocolRead::DisposeThreadLoop``).
    """
    frames = []
    while True:
        start = buf.find(bytes([PROTOCOL_HEAD]))
        if start < 0:
            return frames, b''  # no head; drop garbage, nothing to keep
        end = buf.find(bytes([PROTOCOL_END]), start + 1)
        if end < 0:
            return frames, buf[start:]  # head but no tail yet; wait for more
        frames.append(buf[start:end + 1])
        buf = buf[end + 1:]
