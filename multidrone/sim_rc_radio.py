"""Host-side MAVLink 2 RC override sender for Gazebo SITL."""
from __future__ import annotations

import argparse
import signal
import socket
import struct
import time

# Channels 1..4 idle (roll/pitch/yaw mid, throttle low), ch8 low, the rest ignored.
_STICKS: tuple[int, ...] = (
    1500,
    1500,
    1000,
    1500,
    65535,
    65535,
    65535,
    1000,
    65535,
    65535,
    65535,
    65535,
    65535,
    65535,
    65535,
    65535,
    65535,
    65535,
)
_OVERRIDE = struct.Struct("<8HBB10H")
_TARGET_COMPONENT = 1
SEND_INTERVAL_S = 0.1
_MAGIC = 0xFD
_SENDER_SYSID = 255
_SENDER_COMPID = 190
_MSGID_RC_CHANNELS_OVERRIDE = 70
_CRC_EXTRA = 124


def _require_drone_id(drone_id: int) -> int:
    if drone_id < 1:
        raise ValueError(f"drone_id must be >= 1, got {drone_id!r}")
    return drone_id


def command_port(drone_id: int) -> int:
    return 14280 + _require_drone_id(drone_id)


def target_system_id(drone_id: int) -> int:
    return _require_drone_id(drone_id) + 1


def stick_channels() -> tuple[int, ...]:
    return _STICKS


def override_payload(drone_id: int) -> bytes:
    channels = stick_channels()
    return _OVERRIDE.pack(
        *channels[:8],
        target_system_id(drone_id),
        _TARGET_COMPONENT,
        *channels[8:],
    )


def _crc_accumulate(byte: int, crc: int) -> int:
    tmp = (byte ^ (crc & 0xFF)) & 0xFF
    tmp ^= (tmp << 4) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def mavlink2_frame(payload: bytes, seq: int) -> bytes:
    """MAVLink 2 frame: magic, header, payload, X.25 CRC. No signature."""
    header = bytes((
        len(payload),
        0,
        0,
        seq & 0xFF,
        _SENDER_SYSID,
        _SENDER_COMPID,
        _MSGID_RC_CHANNELS_OVERRIDE & 0xFF,
        (_MSGID_RC_CHANNELS_OVERRIDE >> 8) & 0xFF,
        (_MSGID_RC_CHANNELS_OVERRIDE >> 16) & 0xFF,
    ))
    body = header + payload
    crc = 0xFFFF
    for byte in body:
        crc = _crc_accumulate(byte, crc)
    crc = _crc_accumulate(_CRC_EXTRA, crc)
    return bytes((_MAGIC,)) + body + struct.pack("<H", crc)


def send_overrides(sock, drone_count: int, seq: int) -> None:
    for drone_id in range(1, drone_count + 1):
        frame = mavlink2_frame(override_payload(drone_id), seq)
        sock.sendto(frame, ("127.0.0.1", command_port(drone_id)))


def main() -> None:
    parser = argparse.ArgumentParser(prog="sim_rc_radio.py")
    parser.add_argument("--num", type=int, required=True)
    args = parser.parse_args()
    if args.num < 1:
        raise SystemExit(2)

    stop = False

    def _on_sigterm(_signum, _frame) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGTERM, _on_sigterm)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq = 0
    try:
        while not stop:
            send_overrides(sock, args.num, seq)
            seq = (seq + 1) & 0xFF
            time.sleep(SEND_INTERVAL_S)
    finally:
        sock.close()


if __name__ == "__main__":
    main()
