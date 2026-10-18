import math
import subprocess
import sys
from pathlib import Path

import pytest
from pymavlink import mavutil

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.cascade_flight import load_flight  # noqa: E402
from calibration.cascade_sitl import (  # noqa: E402
    Setpoint,
    _attitude_wire,
    _commanded,
    _param_matches,
    _quaternion,
    _stop,
    setpoints,
    sitl_command,
)

def test_sitl_command_is_headless_one_iris():
    assert sitl_command(Path("runSimNoeticMulti.sh")) == [
        "env", "HEADLESS=1", "runSimNoeticMulti.sh", "--num", "1", "--world", "empty"
    ]

def test_position_nan_is_ignored_not_zero():
    flight = load_flight(ROOT / "calibration" / "log_2_UnknownDate.ulg")
    records = setpoints("position", flight)
    assert records
    partial = next(item for item in records if "x" in item.ignore or "y" in item.ignore)
    assert "x" not in partial.fields or math.isfinite(partial.fields["x"])
    assert 0.0 not in [partial.fields.get(name) for name in partial.ignore]
    assert "z" in partial.fields or "z" in partial.ignore


class _ParamMessage:
    def __init__(self, param_id: str, param_value: float, param_type: int):
        self.param_id = param_id
        self.param_value = param_value
        self.param_type = param_type

    def get_type(self):
        return "PARAM_VALUE"


def _int_bits(value: int) -> float:
    import struct

    return struct.unpack("<f", struct.pack("<i", value))[0]


def test_float_echo_matches_only_with_isclose():
    real = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    integer = mavutil.mavlink.MAV_PARAM_TYPE_INT32
    assert _param_matches(_ParamMessage("MPC_THR_HOVER", 0.0, real), 0.5) is False
    assert _param_matches(_ParamMessage("MPC_THR_HOVER", 0.5, real), 0.5) is True
    assert _param_matches(_ParamMessage("MC_ROLLRATE_MAX", _int_bits(1600), integer), 1600.0) is True
    assert _param_matches(_ParamMessage("MC_ROLLRATE_MAX", _int_bits(1600), integer), 1599.0) is False


def test_angle_attitude_uses_yaw_or_zero():
    ignore = frozenset({"rates"})
    missing = Setpoint("angle", 0.0, {"roll": 0.2, "pitch": -0.1}, ignore)
    wire = _attitude_wire(missing, _commanded(missing))
    assert wire is not None
    assert wire[1] & mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE == 0
    assert wire[2] == pytest.approx(_quaternion(0.2, -0.1, 0.0))


def test_stream_grid_holds_latest_command_and_last_yaw():
    from calibration.cascade_sitl import _stream_ticks

    rate_ignore = frozenset({"attitude", "yaw_rate"})
    rates = [
        Setpoint("rate", 0.0, {"roll_rate": 1.0, "pitch_rate": 0.2}, rate_ignore),
        Setpoint("rate", 0.005, {"roll_rate": 1.0, "pitch_rate": 0.2}, rate_ignore),
        Setpoint("rate", 0.05, {"roll_rate": 4.0, "pitch_rate": 0.2}, rate_ignore),
    ]
    grid = _stream_ticks(rates, hz=50.0)
    assert [item.t_s for item in grid] == pytest.approx([0.0, 0.02, 0.04, 0.05])
    assert [item.fields["roll_rate"] for item in grid] == pytest.approx([1.0, 1.0, 1.0, 4.0])

    angle_ignore = frozenset({"rates"})
    angles = [
        Setpoint("angle", 0.0, {"roll": 0.1, "pitch": 0.2, "yaw": 0.4}, angle_ignore),
        Setpoint("angle", 0.02, {"roll": 0.3, "pitch": -0.1}, angle_ignore),
    ]
    angled = _stream_ticks(angles, hz=50.0)
    assert angled[1].fields["yaw"] == pytest.approx(0.4)
    assert angled[1].fields["roll"] == pytest.approx(0.3)
    held = _attitude_wire(angled[1], _commanded(angled[1]))
    assert held is not None
    assert held[2] == pytest.approx(_quaternion(0.3, -0.1, 0.4))

    bare = _stream_ticks(
        [Setpoint("angle", 0.0, {"roll": 0.2, "pitch": -0.1}, angle_ignore)],
        hz=50.0,
    )
    assert bare[0].fields["yaw"] == pytest.approx(0.0)


class _HungProc:
    def __init__(self):
        self.waits: list = []

    def terminate(self):
        raise OSError("terminate failed")

    def kill(self):
        raise OSError("kill failed")

    def wait(self, timeout=None):
        self.waits.append(timeout)
        raise subprocess.TimeoutExpired("px4", 0 if timeout is None else timeout)


def test_stop_bounds_wait_and_runs_kill(monkeypatch):
    ran: list = []

    def fake_run(cmd, **kwargs):
        ran.append((list(cmd), kwargs))
        raise OSError("cleanup failed")

    monkeypatch.setattr("calibration.cascade_sitl.subprocess.run", fake_run)
    proc = _HungProc()
    _stop(proc)
    assert proc.waits
    assert all(timeout is not None for timeout in proc.waits)
    assert any("--kill" in command for command, _kwargs in ran)
