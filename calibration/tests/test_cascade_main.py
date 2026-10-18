import math
import struct
import sys
import time
from pathlib import Path

import pytest
from pymavlink import mavutil

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.cascade_flight import AxisLog, CascadeFlight, load_flight  # noqa: E402
from calibration.cascade_sitl import fly_log, setpoints, sitl_command  # noqa: E402
from calibration.run_cascade import popen_sim  # noqa: E402

_REAL32 = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
_INT32 = mavutil.mavlink.MAV_PARAM_TYPE_INT32
_DO_SET_MODE = mavutil.mavlink.MAV_CMD_DO_SET_MODE
_ARM = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM

class _Proc:
    def terminate(self):
        return None

class _Master:
    def recv_match(self, **kwargs):
        return None

def test_popen_sim_writes_gazebo_output_to_a_file(tmp_path: Path):
    log = tmp_path / "sitl.log"
    proc = popen_sim([sys.executable, "-c", "print('gazebo-banner')"], log)
    proc.wait(timeout=10)
    assert log.read_text(encoding="utf-8") == "gazebo-banner\n"


def test_fly_log_uses_headless_instance_one(tmp_path: Path, monkeypatch):
    monkeypatch.setattr("calibration.cascade_sitl.subprocess.run", lambda *args, **kwargs: None)
    commands: list[list[str]] = []
    ports: list[str] = []

    def runner(command):
        commands.append(list(command))
        return _Proc()

    def connect(port):
        ports.append(port)
        return _Master()

    model = tmp_path / "iris.sdf.jinja"
    model.write_text("<sdf></sdf>", encoding="utf-8")
    gains = tmp_path / "iris_controller.json"
    gains.write_text('{"MPC_THR_HOVER": 0.5, "MC_ROLL_P": 12.0}\n', encoding="utf-8")
    flight = load_flight(ROOT / "calibration" / "log_2_UnknownDate.ulg")
    fly_log("rate", flight, model, gains, runner=runner, connect=connect)
    assert commands == [sitl_command(Path("runSimNoeticMulti.sh"))]
    assert ports == ["udpin:0.0.0.0:14541"]


def _int_bits(value: int) -> float:
    return struct.unpack("<f", struct.pack("<i", int(value)))[0]


def _axis(times, values):
    count = len(times)
    return AxisLog(tuple(times), tuple(values), tuple(values), tuple(True for _ in range(count)))


def _rate_flight(times, roll_rate, pitch_rate):
    thrust = [-0.34 for _ in times]
    empty = _axis([times[0]], [0.0])
    return CascadeFlight(
        "synthetic",
        0.34,
        {},
        _axis(times, roll_rate),
        _axis(times, pitch_rate),
        empty,
        empty,
        empty,
        empty,
        empty,
        empty,
        empty,
        empty,
        empty,
        _axis(times, thrust),
    )


class _Message:
    def __init__(self, kind: str, **fields):
        self._kind = kind
        for name, value in fields.items():
            setattr(self, name, value)

    def get_type(self):
        return self._kind

    def get_srcSystem(self):
        return int(getattr(self, "src_system", 1))


class _LiveMav:
    def __init__(self, owner):
        self.owner = owner

    def set_attitude_target_send(
        self, time_boot_ms, system, component, typemask, quaternion, roll_rate, pitch_rate, yaw_rate, thrust
    ):
        now = time.monotonic()
        if self.owner.first_send_at is None:
            self.owner.first_send_at = now
        self.owner.attitudes.append(
            {
                "t": now,
                "roll_rate": float(roll_rate),
                "pitch_rate": float(pitch_rate),
                "q": tuple(quaternion),
            }
        )

    def set_position_target_local_ned_send(self, *args):
        return None

    def command_long_send(self, system, component, command, confirmation, *params):
        command = int(command)
        self.owner.commands.append(command)
        if command == _DO_SET_MODE:
            self.owner.saw_mode = True
        elif command == _ARM:
            self.owner.saw_arm = True


class _LiveMaster:
    """Live link with no socket. Param echoes, then a delayed arm heartbeat."""

    def __init__(self, types: dict, echoes: dict, arm_delay_s: float):
        self.types = types
        self.echoes = echoes
        self.arm_delay_s = arm_delay_s
        self.mav = _LiveMav(self)
        self.target_system = 1
        self.target_component = 1
        self.sent: list[tuple[str, float, int]] = []
        self.commands: list[int] = []
        self.attitudes: list[dict] = []
        self._pending: list[tuple[str, str]] = []
        self.first_send_at = None
        self.armed_at = None
        self.saw_mode = False
        self.saw_arm = False
        self._phase = "wait"
        self._sent_early_position = False

    def wait_heartbeat(self, timeout=None):
        return _Message("HEARTBEAT", base_mode=0, custom_mode=0)

    def param_fetch_one(self, name: str) -> None:
        self._pending.append(("fetch", str(name)))

    def param_set_send(self, name, value, param_type) -> None:
        self.sent.append((str(name), float(value), int(param_type)))
        self._pending.append(("set", str(name)))

    def recv_match(self, blocking=True, timeout=None, **kwargs):
        if self._pending:
            kind, name = self._pending.pop(0)
            if kind == "fetch":
                return _Message(
                    "PARAM_VALUE",
                    param_id=name,
                    param_value=0.0,
                    param_type=self.types[name],
                )
            return _Message(
                "PARAM_VALUE",
                param_id=name,
                param_value=self.echoes[name],
                param_type=self.types[name],
            )
        if self.first_send_at is None:
            return None
        elapsed = time.monotonic() - self.first_send_at
        if self._phase == "wait":
            if not self._sent_early_position:
                self._sent_early_position = True
                return _Message("LOCAL_POSITION_NED", x=0.0, y=0.0, z=-9.0, vx=0.0, vy=0.0, vz=0.0)
            if elapsed >= self.arm_delay_s and self.saw_mode and self.saw_arm:
                self._phase = "ack_arm"
                return _Message(
                    "COMMAND_ACK",
                    command=_DO_SET_MODE,
                    result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
                )
            return None
        if self._phase == "ack_arm":
            self._phase = "heartbeat"
            return _Message(
                "COMMAND_ACK",
                command=_ARM,
                result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
            )
        if self._phase == "heartbeat":
            self._phase = "sample"
            self.armed_at = time.monotonic()
            base = (
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                | mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            custom = mavutil.PX4_CUSTOM_MAIN_MODE_OFFBOARD << 16
            return _Message("HEARTBEAT", base_mode=base, custom_mode=custom, src_system=1)
        if self._phase == "sample":
            self._phase = "drop"
            return _Message("ATTITUDE", roll=0.42, pitch=0.0, rollspeed=0.1, pitchspeed=-0.2)
        if self._phase == "drop":
            self._phase = "coast"
            base = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            return _Message("HEARTBEAT", base_mode=base, custom_mode=0, src_system=1)
        return None


def _files(tmp_path: Path, gains: dict):
    model = tmp_path / "iris.sdf.jinja"
    model.write_text("<sdf></sdf>", encoding="utf-8")
    path = tmp_path / "iris_controller.json"
    body = "{" + ", ".join(f'"{name}": {value}' for name, value in gains.items()) + "}"
    path.write_text(body + "\n", encoding="utf-8")
    return model, path


def test_live_master_param_types_arm_clock_and_resample(tmp_path: Path, monkeypatch):
    monkeypatch.setattr("calibration.cascade_sitl.subprocess.run", lambda *args, **kwargs: None)
    times = [5.0 + index * 0.005 for index in range(121)]
    roll_rate = [1.0 if time_s < 5.4 else 4.0 for time_s in times]
    flight = _rate_flight(times, roll_rate, [0.2 for _ in times])
    records = setpoints("rate", flight)
    assert len(records) == 121
    gains = {"MPC_THR_HOVER": 0.5, "MC_ROLL_P": 12.0, "MC_ROLLRATE_MAX": 1600}
    master = _LiveMaster(
        {
            "MPC_THR_HOVER": _REAL32,
            "MC_ROLL_P": _REAL32,
            "MC_ROLLRATE_MAX": _INT32,
        },
        {
            "MPC_THR_HOVER": 0.5,
            "MC_ROLL_P": 12.0,
            "MC_ROLLRATE_MAX": _int_bits(1600),
        },
        arm_delay_s=0.3,
    )
    model, gain_path = _files(tmp_path, gains)
    series = fly_log(
        "rate",
        flight,
        model,
        gain_path,
        runner=lambda command: _Proc(),
        connect=lambda port: master,
    )
    sent = {name: (value, param_type) for name, value, param_type in master.sent}
    assert sent["MPC_THR_HOVER"] == pytest.approx((0.5, _REAL32))
    assert sent["MC_ROLL_P"] == pytest.approx((12.0, _REAL32))
    assert sent["MC_ROLLRATE_MAX"][1] == _INT32
    assert sent["MC_ROLLRATE_MAX"][0] == pytest.approx(_int_bits(1600))
    assert master.armed_at is not None and master.first_send_at is not None
    assert master.armed_at - master.first_send_at > 0.2
    roll_times, roll_values = series["roll"]
    assert roll_times
    assert roll_times[0] == pytest.approx(5.0, abs=0.05)
    assert roll_values[0] == pytest.approx(0.42)
    assert -9.0 not in series["z"][1]
    assert master.commands.count(_ARM) >= 2
    assert master.commands.count(_DO_SET_MODE) >= 2
    post = [item for item in master.attitudes if item["t"] >= master.armed_at]
    assert 20 <= len(post) <= 40
    assert len(master.attitudes) < len(records) / 2
    assert any(math.isclose(item["roll_rate"], 1.0) for item in post)
    assert any(math.isclose(item["roll_rate"], 4.0) for item in post)


def test_hover_echo_of_zero_does_not_confirm(tmp_path: Path, monkeypatch):
    monkeypatch.setattr("calibration.cascade_sitl.subprocess.run", lambda *args, **kwargs: None)
    flight = _rate_flight([0.0, 0.02], [1.0, 1.0], [0.2, 0.2])
    master = _LiveMaster(
        {"MPC_THR_HOVER": _REAL32},
        {"MPC_THR_HOVER": 0.0},
        arm_delay_s=0.0,
    )
    model, gain_path = _files(tmp_path, {"MPC_THR_HOVER": 0.5})
    with pytest.raises((ValueError, TimeoutError)):
        fly_log(
            "rate",
            flight,
            model,
            gain_path,
            runner=lambda command: _Proc(),
            connect=lambda port: master,
        )
    assert master.sent
    name, value, param_type = master.sent[0]
    assert name == "MPC_THR_HOVER"
    assert param_type == _REAL32
    assert value == pytest.approx(0.5)


class _BoomProc:
    def terminate(self):
        raise OSError("terminate failed")

    def kill(self):
        raise OSError("kill failed")

    def wait(self, timeout=None):
        raise OSError("wait failed")


class _SilentMaster:
    def __init__(self):
        self.mav = _LiveMav(self)
        self.target_system = 1
        self.target_component = 1
        self.first_send_at = None
        self.attitudes: list = []
        self.commands: list = []
        self.saw_mode = False
        self.saw_arm = False

    def wait_heartbeat(self, timeout=None):
        return None


def test_cleanup_oserror_does_not_replace_flight_error(tmp_path: Path, monkeypatch):
    monkeypatch.setattr("calibration.cascade_sitl.subprocess.run", lambda *args, **kwargs: None)
    flight = _rate_flight([0.0], [1.0], [0.2])
    model, gain_path = _files(tmp_path, {"MPC_THR_HOVER": 0.5})
    with pytest.raises(TimeoutError, match="heartbeat"):
        fly_log(
            "rate",
            flight,
            model,
            gain_path,
            runner=lambda command: _BoomProc(),
            connect=lambda port: _SilentMaster(),
        )
