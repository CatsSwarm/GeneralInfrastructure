import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sim_rc_radio import (
    SEND_INTERVAL_S,
    command_port,
    main,
    mavlink2_frame,
    override_payload,
    send_overrides,
    stick_channels,
    target_system_id,
)


def test_ports_and_sysids():
    assert command_port(1) == 14281
    assert command_port(4) == 14284
    assert target_system_id(1) == 2
    assert target_system_id(4) == 5


def test_sticks_are_a_bound_idle_radio():
    ch = stick_channels()
    assert len(ch) == 18
    assert ch[0] == ch[1] == ch[3] == 1500
    assert ch[2] == 1000
    assert ch[7] == 1000
    assert ch[4] == ch[5] == ch[6] == 65535
    assert all(v == 65535 for v in ch[8:])
    assert 0 not in ch[:8]


def test_payload_matches_px4_struct():
    raw = override_payload(1)
    assert len(raw) == 38
    chan = struct.unpack_from("<8H", raw, 0)
    sysid, compid = struct.unpack_from("<BB", raw, 16)
    assert chan[:4] == (1500, 1500, 1000, 1500)
    assert chan[7] == 1000
    assert sysid == 2 and compid == 1
    assert struct.unpack_from("<H", raw, 18)[0] == 65535


def test_nonpositive_drone_id_raises():
    for drone_id in (0, -1):
        for fn in (command_port, target_system_id, override_payload):
            with pytest.raises(ValueError):
                fn(drone_id)


def mavlink_crc_x25(data: bytes, extra: int) -> bytes:
    """MAVLink X.25 CRC, init 0xFFFF. The extra byte is accumulated last."""
    crc = 0xFFFF
    for byte in (*data, extra):
        tmp = (byte ^ (crc & 0xFF)) & 0xFF
        tmp ^= (tmp << 4) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return struct.pack("<H", crc)


def test_frame_is_mavlink2_override():
    payload = override_payload(1)
    frame = mavlink2_frame(payload, seq=0)
    assert frame[0] == 0xFD
    assert frame[1] == 38
    assert frame[7:10] == bytes((70, 0, 0))
    assert frame[10:48] == payload
    assert frame[-2:] == mavlink_crc_x25(frame[1:-2], extra=124)


def test_send_overrides_one_datagram_per_drone():
    sent = []

    class Sock:
        def sendto(self, data, addr):
            sent.append((data, addr))

    send_overrides(Sock(), 4, seq=3)
    assert [addr for _, addr in sent] == [
        ("127.0.0.1", 14281),
        ("127.0.0.1", 14282),
        ("127.0.0.1", 14283),
        ("127.0.0.1", 14284),
    ]
    assert all(frame[0] == 0xFD for frame, _ in sent)


def test_interval_is_10_hz():
    assert SEND_INTERVAL_S == 0.1


def test_num_below_one_exits_without_sending(monkeypatch):
    monkeypatch.setattr(sys, "argv", ["sim_rc_radio.py", "--num", "0"])
    with pytest.raises(SystemExit) as caught:
        main()
    assert caught.value.code == 2
