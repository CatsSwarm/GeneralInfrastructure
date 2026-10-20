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
    _Vehicle,
    _altitude_thrust,
    _attitude_wire,
    _climb_thrust,
    _climb_wire,
    _commanded,
    _copy_px4_log,
    fly_log,
    _hold_altitude,
    _ned_wire,
    _param_matches,
    _quaternion,
    _send_gains,
    _stop,
    _stream,
    setpoints,
    sitl_command,
)
from calibration.cascade_thrust import spin_rad_s  # noqa: E402

def test_sitl_command_is_headless_one_iris():
    script = ROOT / "runSimNoeticMulti.sh"
    assert sitl_command(Path("runSimNoeticMulti.sh")) == [
        "env", "HEADLESS=1", str(script), "--num", "1", "--world", "empty"
    ]
    assert script.is_file()


def test_sitl_command_show_gazebo_keeps_the_window():
    script = ROOT / "runSimNoeticMulti.sh"
    assert sitl_command(Path("runSimNoeticMulti.sh"), show_gazebo=True) == [
        "env", "HEADLESS=0", str(script), "--num", "1", "--world", "empty"
    ]
    text = script.read_text(encoding="utf-8")
    assert '[ -t 0 ]' in text
    assert "Starting gazebo client" in (ROOT / "multidrone" / "sitl_multiple_run.sh").read_text(encoding="utf-8")

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


def test_absent_parameter_does_not_stop_the_rest(monkeypatch):
    monkeypatch.setattr("calibration.cascade_sitl._PARAM_TIMEOUT_S", 0.05)

    class _Master:
        def __init__(self):
            self.mav = type("Mav", (), {
                "set_attitude_target_send": staticmethod(lambda *args: None),
                "set_position_target_local_ned_send": staticmethod(lambda *args: None),
            })()
            self.sent: list[str] = []
            self._pending: list[str] = []

        def param_fetch_one(self, name):
            if name != "MPC_HOLD_DZ":
                self._pending.append(name)

        def param_set_send(self, name, value, param_type):
            self.sent.append(name)
            self._pending.append(name)

        def recv_match(self, blocking=True, timeout=None, **kwargs):
            if not self._pending:
                return None
            name = self._pending.pop(0)
            return _ParamMessage(name, 12.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    master = _Master()
    _send_gains(master, {"MPC_HOLD_DZ": 0.1, "MC_ROLL_P": 12.0})
    assert master.sent == ["MC_ROLL_P"]


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
    assert ran[0][0] == [str(ROOT / "runSimNoeticMulti.sh"), "--kill"]


def test_climb_thrust_is_a_gentle_climb_on_the_iris_map():
    hover = 0.34378432720094537
    thrust = _climb_thrust(hover)
    ratio = (spin_rad_s(thrust) / spin_rad_s(hover)) ** 2
    assert 1.05 < ratio < 1.4


def test_climb_wire_holds_level_with_that_thrust():
    thrust = _climb_thrust(0.34378432720094537)
    wire = _climb_wire(thrust)
    assert wire[0] == "attitude"
    mask = wire[1]
    assert mask & mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE == 0
    assert mask & mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE == 0
    assert mask & mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE == 0
    assert mask & mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE == 0
    assert wire[2] == (1.0, 0.0, 0.0, 0.0)
    assert wire[3] == pytest.approx(0.0)
    assert wire[4] == pytest.approx(0.0)
    assert wire[6] == pytest.approx(thrust)


class _ClimbMessage:
    def __init__(self, kind: str, **fields):
        self._kind = kind
        self.__dict__.update(fields)

    def get_type(self):
        return self._kind

    def get_srcSystem(self):
        return 1


class _ClimbMaster:
    """Arms on the first setpoint, then stays below 10 m for two polls."""

    def __init__(self):
        self.mav = self
        self.target_system = 1
        self.target_component = 1
        self.events: list[tuple] = []
        self._armed = False
        self._after_arm = 0
        self._height = False

    def set_attitude_target_send(
        self, time_boot_ms, system, component, typemask, quaternion, roll_rate, pitch_rate, yaw_rate, thrust
    ):
        self.events.append(("attitude", float(pitch_rate), float(thrust), tuple(quaternion)))

    def set_position_target_local_ned_send(self, *args):
        return None

    def command_long_send(self, *args):
        return None

    def recv_match(self, blocking=True, timeout=None, **kwargs):
        if not any(kind == "attitude" for kind, *_rest in self.events):
            return None
        if not self._armed:
            self._armed = True
            base = (
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                | mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            custom = mavutil.PX4_CUSTOM_MAIN_MODE_OFFBOARD << 16
            return _ClimbMessage("HEARTBEAT", base_mode=base, custom_mode=custom)
        self._after_arm += 1
        if self._after_arm <= 2:
            return None
        if not self._height:
            self._height = True
            self.events.append(("altitude", -10.0))
            return _ClimbMessage(
                "LOCAL_POSITION_NED", x=0.0, y=0.0, z=-10.0, vx=0.0, vy=0.0, vz=0.0
            )
        return None


def test_replay_stays_level_until_ten_metres_and_floors_thrust():
    hover = 0.34378432720094537
    records = [
        Setpoint(
            "rate",
            1.26,
            {"roll_rate": 0.15, "pitch_rate": 0.49, "thrust_z": -0.067},
            frozenset({"attitude", "yaw_rate"}),
        )
    ]
    master = _ClimbMaster()
    _stream(master, records, {}, hover_command=hover)
    altitude_at = next(index for index, event in enumerate(master.events) if event[0] == "altitude")
    climb = [event for event in master.events[:altitude_at] if event[0] == "attitude"]
    assert climb
    thrust = _climb_thrust(hover)
    for _kind, pitch_rate, sent_thrust, quaternion in climb:
        assert pitch_rate == pytest.approx(0.0)
        assert quaternion == (1.0, 0.0, 0.0, 0.0)
        assert sent_thrust == pytest.approx(thrust)
    replay = [event for event in master.events[altitude_at:] if event[0] == "attitude"]
    assert replay
    _kind, pitch_rate, sent_thrust, _quaternion = replay[0]
    assert pitch_rate == pytest.approx(0.49)
    assert sent_thrust == pytest.approx(hover)


def test_altitude_thrust_returns_to_the_climb_height():
    hover = 0.344
    assert _altitude_thrust(10.0, 0.0, hover) == pytest.approx(hover)
    assert _altitude_thrust(8.0, 0.0, hover) > hover
    assert _altitude_thrust(12.0, 0.0, hover) < hover
    assert _altitude_thrust(10.0, 1.0, hover) > _altitude_thrust(10.0, 0.0, hover)
    capped = _altitude_thrust(0.0, 5.0, hover)
    assert capped == pytest.approx(_altitude_thrust(0.0, 0.0, hover))
    ratio = (spin_rad_s(capped) / spin_rad_s(hover)) ** 2
    assert 1.2 < ratio < 1.8


def test_ned_hold_keeps_the_climb_altitude():
    wire = _ned_wire({"x": 1.0, "y": -2.0, "z": -1.2, "vx": 0.3, "vy": 0.0, "vz": 0.4})
    held = _hold_altitude(wire, _Vehicle(height_m=8.0, vz_down=0.2), 0.344)
    assert held is not None
    assert held[2][0] == pytest.approx(1.0)
    assert held[2][1] == pytest.approx(-2.0)
    assert held[2][2] == pytest.approx(-10.0)
    assert held[2][5] < 0.0
    assert held[1] & mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE == 0
    assert held[1] & mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE == 0


class _DropMaster(_ClimbMaster):
    def __init__(self):
        super().__init__()
        self._dropped = False

    def recv_match(self, blocking=True, timeout=None, **kwargs):
        message = super().recv_match(blocking=blocking, timeout=timeout, **kwargs)
        if message is not None or not self._height or self._dropped:
            return message
        self._dropped = True
        return _ClimbMessage(
            "LOCAL_POSITION_NED", x=0.0, y=0.0, z=-8.0, vx=0.0, vy=0.0, vz=0.4
        )


def test_replay_adds_thrust_when_height_falls():
    hover = 0.34378432720094537
    records = [
        Setpoint(
            "rate",
            1.26,
            {"roll_rate": 0.15, "pitch_rate": 0.49, "thrust_z": -0.067},
            frozenset({"attitude", "yaw_rate"}),
        )
    ]
    master = _DropMaster()
    series = {name: ([], []) for name in ("x", "y", "z", "vx", "vy", "vz", "roll", "pitch", "roll_rate", "pitch_rate")}
    _stream(master, records, series, hover_command=hover)
    replay = [event for event in master.events if event[0] == "attitude" and event[1] == pytest.approx(0.49)]
    assert replay
    assert replay[0][2] == pytest.approx(_altitude_thrust(8.0, 0.4, hover))
    assert replay[0][2] > hover


def test_copy_px4_log_uses_the_newest_ulg(tmp_path: Path, monkeypatch):
    calls = []

    def run(command, **_kwargs):
        calls.append(command)
        if command[1] == "exec":
            return subprocess.CompletedProcess(command, 0, stdout="10 /tmp/rootfs/1/log/a.ulg\n")
        return subprocess.CompletedProcess(command, 0, stdout="")

    monkeypatch.setattr("calibration.cascade_sitl.subprocess.run", run)
    dest = tmp_path / "logs" / "trial_001.ulg"
    assert _copy_px4_log(dest) is True
    assert calls[1] == ["docker", "cp", "px4-noetic-sim-ros:/tmp/rootfs/1/log/a.ulg", str(dest)]


def test_fly_log_copies_before_the_container_is_removed(tmp_path: Path, monkeypatch):
    order = []
    monkeypatch.setattr(
        "calibration.cascade_sitl._copy_px4_log",
        lambda dest: order.append(("copy", Path(dest))) or True,
    )
    monkeypatch.setattr("calibration.cascade_sitl._stop", lambda proc: order.append("stop"))
    monkeypatch.setattr("calibration.cascade_sitl.subprocess.run", lambda *args, **kwargs: None)
    flight = load_flight(ROOT / "calibration" / "log_2_UnknownDate.ulg")
    model = tmp_path / "iris.sdf.jinja"
    model.write_text("<sdf></sdf>", encoding="utf-8")
    gains = tmp_path / "gains.json"
    gains.write_text('{"MPC_THR_HOVER": 0.5}\n', encoding="utf-8")

    class Master:
        def recv_match(self, **kwargs):
            return None

    class Proc:
        def terminate(self):
            return None

    fly_log(
        "rate",
        flight,
        model,
        gains,
        runner=lambda command: Proc(),
        connect=lambda port: Master(),
        flight_log=tmp_path / "logs" / "trial_001.ulg",
    )
    assert order[0][1].name == "trial_001.ulg"
    assert order[1] == "stop"


def _sitl_files(tmp_path: Path):
    model = tmp_path / "iris.sdf.jinja"
    model.write_text("<sdf></sdf>", encoding="utf-8")
    gains = tmp_path / "gains.json"
    gains.write_text('{"MPC_THR_HOVER": 0.5}\n', encoding="utf-8")
    flight = load_flight(ROOT / "calibration" / "log_2_UnknownDate.ulg")
    return flight, model, gains


class _StartProc:
    def terminate(self):
        return None

    def wait(self, timeout=None):
        return 0


class _QuietMaster:
    def recv_match(self, **kwargs):
        return None


def _fake_clock(monkeypatch):
    now = {"t": 0.0}

    def monotonic():
        return now["t"]

    def sleep(seconds):
        now["t"] += float(seconds)

    monkeypatch.setattr("calibration.cascade_sitl.time.monotonic", monotonic)
    monkeypatch.setattr("calibration.cascade_sitl.time.sleep", sleep)
    return now


def _names_from_docker(monkeypatch, listings: list[str], order: list):
    """Return scripted ``docker ps`` names. Later polls repeat the last listing."""
    pending = list(listings)

    def run(cmd, **kwargs):
        command = list(cmd)
        if command[:2] == ["docker", "ps"]:
            order.append("ps")
            stdout = pending.pop(0) if pending else listings[-1]
            return subprocess.CompletedProcess(command, 0, stdout=stdout)
        order.append("other")
        return subprocess.CompletedProcess(command, 0, stdout="")

    monkeypatch.setattr("calibration.cascade_sitl.subprocess.run", run)
    monkeypatch.setattr(
        "calibration.cascade_sitl.time.sleep",
        lambda seconds: order.append("sleep") if float(seconds) > 0 else None,
    )


def test_fly_log_waits_until_px4_container_is_gone_before_start(tmp_path: Path, monkeypatch):
    order: list = []
    _names_from_docker(
        monkeypatch,
        ["px4-noetic-sim-ros\n", "other-container\n"],
        order,
    )
    flight, model, gains = _sitl_files(tmp_path)

    def runner(command):
        order.append("start")
        return _StartProc()

    fly_log(
        "rate",
        flight,
        model,
        gains,
        runner=runner,
        connect=lambda port: _QuietMaster(),
    )
    assert order[:4] == ["ps", "sleep", "ps", "start"]


def test_fly_log_does_not_start_while_px4_container_remains(tmp_path: Path, monkeypatch):
    _fake_clock(monkeypatch)
    order: list = []

    def run(cmd, **kwargs):
        command = list(cmd)
        if command[:2] == ["docker", "ps"]:
            order.append("ps")
            return subprocess.CompletedProcess(command, 0, stdout="px4-noetic-sim-ros\n")
        order.append("other")
        return subprocess.CompletedProcess(command, 0, stdout="")

    monkeypatch.setattr("calibration.cascade_sitl.subprocess.run", run)
    flight, model, gains = _sitl_files(tmp_path)
    starts: list = []
    with pytest.raises(TimeoutError, match="px4-noetic-sim-ros"):
        fly_log(
            "rate",
            flight,
            model,
            gains,
            runner=lambda command: starts.append("start") or _StartProc(),
            connect=lambda port: _QuietMaster(),
        )
    assert starts == []
    assert "ps" in order


class _LiveHeartbeat:
    def __init__(self, beats: list):
        self._beats = beats
        self.mav = self

    def set_attitude_target_send(self, *args):
        return None

    def set_position_target_local_ned_send(self, *args):
        return None

    def wait_heartbeat(self, timeout=None):
        self._beats.append(timeout)
        if len(self._beats) < 3:
            return None
        return object()


def test_fly_log_retries_a_heartbeat_timeout_then_flies(tmp_path: Path, monkeypatch):
    monkeypatch.setattr(
        "calibration.cascade_sitl.subprocess.run",
        lambda *args, **kwargs: subprocess.CompletedProcess(args[0] if args else [], 0, stdout=""),
    )
    monkeypatch.setattr("calibration.cascade_sitl._send_gains", lambda *args, **kwargs: None)
    monkeypatch.setattr("calibration.cascade_sitl._request_streams", lambda *args, **kwargs: None)
    monkeypatch.setattr("calibration.cascade_sitl._stream", lambda *args, **kwargs: None)
    flight, model, gains = _sitl_files(tmp_path)
    beats: list = []
    starts: list = []

    def runner(command):
        starts.append(list(command))
        return _StartProc()

    series = fly_log(
        "velocity",
        flight,
        model,
        gains,
        runner=runner,
        connect=lambda port: _LiveHeartbeat(beats),
    )
    assert len(starts) == 3
    assert len(beats) == 3
    assert "vx" in series


def test_fly_log_stops_after_three_heartbeat_timeouts(tmp_path: Path, monkeypatch):
    monkeypatch.setattr(
        "calibration.cascade_sitl.subprocess.run",
        lambda *args, **kwargs: subprocess.CompletedProcess(args[0] if args else [], 0, stdout=""),
    )
    flight, model, gains = _sitl_files(tmp_path)
    starts: list = []

    class _Silent:
        def __init__(self):
            self.mav = self

        def set_attitude_target_send(self, *args):
            return None

        def set_position_target_local_ned_send(self, *args):
            return None

        def wait_heartbeat(self, timeout=None):
            return None

    with pytest.raises(TimeoutError, match="no heartbeat on udpin:0.0.0.0:14541"):
        fly_log(
            "velocity",
            flight,
            model,
            gains,
            runner=lambda command: starts.append("start") or _StartProc(),
            connect=lambda port: _Silent(),
        )
    assert len(starts) == 3
