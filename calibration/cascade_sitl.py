"""Setpoint records, the headless sim command, and one offboard replay."""

from __future__ import annotations

import json
import math
import struct
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from pymavlink import mavutil

from calibration.cascade_flight import AxisLog, CascadeFlight


@dataclass(frozen=True)
class Setpoint:
    step: str
    t_s: float
    fields: dict[str, float]
    ignore: frozenset[str]


_REPO_ROOT = Path(__file__).resolve().parents[1]


def _script_path(script: Path) -> Path:
    """Absolute path. ``env`` and ``subprocess`` do not search the working directory."""
    path = Path(script)
    if not path.is_absolute():
        path = _REPO_ROOT / path
    return path


def sitl_command(script: Path) -> list[str]:
    return ["env", "HEADLESS=1", str(_script_path(script)), "--num", "1", "--world", "empty"]


def _nearest_command(source: AxisLog, query_t: np.ndarray) -> np.ndarray:
    """Nearest sample. A non-finite chosen command stays NaN."""
    source_t = np.asarray(source.t_s, dtype=np.float64)
    index = np.searchsorted(source_t, query_t, side="left")
    index = np.clip(index, 0, len(source_t) - 1)
    earlier = np.clip(index - 1, 0, len(source_t) - 1)
    use_earlier = np.abs(query_t - source_t[earlier]) <= np.abs(source_t[index] - query_t)
    chosen = np.where(use_earlier, earlier, index)
    values = np.asarray(source.command, dtype=np.float64)[chosen]
    return np.where(np.isfinite(values), values, np.nan)


def _finite(value: float) -> bool:
    return math.isfinite(float(value))


def _rate_setpoints(flight: CascadeFlight) -> list[Setpoint]:
    thrust = _nearest_command(
        flight.thrust_z, np.asarray(flight.roll_rate.t_s, dtype=np.float64)
    )
    ignore = frozenset({"attitude", "yaw_rate"})
    records: list[Setpoint] = []
    for index, t_s in enumerate(flight.roll_rate.t_s):
        if not flight.roll_rate.airborne[index]:
            continue
        roll = flight.roll_rate.command[index]
        pitch = flight.pitch_rate.command[index]
        if not _finite(roll) or not _finite(pitch):
            continue
        fields = {
            "roll_rate": float(roll),
            "pitch_rate": float(pitch),
        }
        thrust_value = float(thrust[index])
        if _finite(thrust_value):
            fields["thrust_z"] = thrust_value
        records.append(Setpoint("rate", float(t_s), fields, ignore))
    return records


def _angle_setpoints(flight: CascadeFlight) -> list[Setpoint]:
    thrust = _nearest_command(
        flight.thrust_z, np.asarray(flight.roll.t_s, dtype=np.float64)
    )
    ignore = frozenset({"rates"})
    records: list[Setpoint] = []
    for index, t_s in enumerate(flight.roll.t_s):
        fields: dict[str, float] = {}
        roll = flight.roll.command[index]
        pitch = flight.pitch.command[index]
        if _finite(roll):
            fields["roll"] = float(roll)
        if _finite(pitch):
            fields["pitch"] = float(pitch)
        thrust_value = float(thrust[index])
        if _finite(thrust_value):
            fields["thrust_z"] = thrust_value
        yaw = flight.yaw.command[index]
        if _finite(yaw):
            fields["yaw"] = float(yaw)
        records.append(Setpoint("angle", float(t_s), fields, ignore))
    return records


def _velocity_setpoints(flight: CascadeFlight) -> list[Setpoint]:
    records: list[Setpoint] = []
    for index, t_s in enumerate(flight.vx.t_s):
        fields: dict[str, float] = {}
        ignored = {"x", "y", "z"}
        for name, axis in (("vx", flight.vx), ("vy", flight.vy), ("vz", flight.vz)):
            value = axis.command[index]
            if _finite(value):
                fields[name] = float(value)
            else:
                ignored.add(name)
        records.append(Setpoint("velocity", float(t_s), fields, frozenset(ignored)))
    return records


def _position_setpoints(flight: CascadeFlight) -> list[Setpoint]:
    yaw = _nearest_command(flight.yaw, np.asarray(flight.x.t_s, dtype=np.float64))
    records: list[Setpoint] = []
    for index, t_s in enumerate(flight.x.t_s):
        fields: dict[str, float] = {}
        ignored: set[str] = set()
        for name, axis in (("x", flight.x), ("y", flight.y), ("z", flight.z)):
            value = axis.command[index]
            if _finite(value):
                fields[name] = float(value)
            else:
                ignored.add(name)
        yaw_value = float(yaw[index])
        if _finite(yaw_value):
            fields["yaw"] = yaw_value
        records.append(Setpoint("position", float(t_s), fields, frozenset(ignored)))
    return records


def setpoints(step: str, flight: CascadeFlight) -> list[Setpoint]:
    builders = {
        "rate": _rate_setpoints,
        "angle": _angle_setpoints,
        "velocity": _velocity_setpoints,
        "position": _position_setpoints,
    }
    try:
        build = builders[step]
    except KeyError:
        raise ValueError(f"unknown cascade step: {step}") from None
    return build(flight)


_LINK = "udpin:0.0.0.0:14541"
_SERIES: tuple[str, ...] = (
    "roll_rate",
    "pitch_rate",
    "roll",
    "pitch",
    "vx",
    "vy",
    "vz",
    "x",
    "y",
    "z",
)
_RATE_NAMES = ("roll_rate", "pitch_rate", "yaw_rate")
_ANGLE_NAMES = ("roll", "pitch", "yaw")
_REAL_PARAM_TYPES = {
    mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    mavutil.mavlink.MAV_PARAM_TYPE_REAL64,
}
_ROLL_RATE_IGNORE = mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
_PITCH_RATE_IGNORE = mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
_YAW_RATE_IGNORE = mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
_THROTTLE_IGNORE = mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE
_ATTITUDE_IGNORE = mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE
_NED_AXES: tuple[tuple[str, int], ...] = (
    ("x", mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE),
    ("y", mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE),
    ("z", mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE),
    ("vx", mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE),
    ("vy", mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE),
    ("vz", mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE),
    ("ax", mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE),
    ("ay", mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE),
    ("az", mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE),
    ("yaw", mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE),
    ("yaw_rate", mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE),
)
_NED_FRAME = mavutil.mavlink.MAV_FRAME_LOCAL_NED
_MODE_PERIOD_S = 0.5
_FORCE_ARM_AFTER_S = 5.0
_PX4_FORCE_ARM = 21196.0
# Docker boot plus up to two gzserver restarts (5s each) before PX4 mavlink.
_HEARTBEAT_TIMEOUT_S = 90.0
_STREAM_HZ = 50.0
_PARAM_TIMEOUT_S = 2.0
_ACK_TIMEOUT_S = 60.0
_ECHO_REL_TOL = 1e-5
_ECHO_ABS_TOL = 1e-5


def fly_log(
    step: str,
    flight,
    model: Path,
    gains_path: Path,
    *,
    runner,
    connect,
) -> dict[str, tuple[list[float], list[float]]]:
    """Replay one cascade step. ``runner`` and ``connect`` are required keywords.

    The host iris jinja is the file ``runSimNoeticMulti.sh`` already bind-mounts.
    This function does not copy that file into an image. Scored sample times are
    the replay clock. A non-blocking ``recv_match`` of ``None`` ends a drain; on
    a live link that does not end the rest of the step.
    """
    gains = _load_gains(gains_path)
    if not Path(model).is_file():
        raise FileNotFoundError(model)
    proc = runner(sitl_command(Path("runSimNoeticMulti.sh")))
    try:
        master = connect(_LINK)
        if _live(master):
            _wait_heartbeat(master)
        _send_gains(master, gains)
        if _live(master):
            _request_streams(master)
        series = {name: ([], []) for name in _SERIES}
        _stream(master, setpoints(step, flight), series)
        return {name: (times, values) for name, (times, values) in series.items()}
    finally:
        _stop(proc)


_PROC_WAIT_S = 30.0
_KILL_WAIT_S = 5.0
_SIM_KILL_TIMEOUT_S = 30.0


def _stop(proc) -> None:
    """End the sim process, then run ``runSimNoeticMulti.sh --kill``."""
    _end_process(proc)
    _kill_sim()


def _end_process(proc) -> None:
    try:
        terminate = getattr(proc, "terminate", None)
        if terminate is not None:
            terminate()
    except OSError:
        pass
    wait = getattr(proc, "wait", None)
    if wait is None:
        return
    try:
        wait(timeout=_PROC_WAIT_S)
        return
    except subprocess.TimeoutExpired:
        pass
    except OSError:
        return
    try:
        kill = getattr(proc, "kill", None)
        if kill is not None:
            kill()
    except OSError:
        pass
    try:
        wait(timeout=_KILL_WAIT_S)
    except (subprocess.TimeoutExpired, OSError):
        pass


def _kill_sim() -> None:
    try:
        subprocess.run(
            [str(_script_path(Path("runSimNoeticMulti.sh"))), "--kill"],
            check=False,
            timeout=_SIM_KILL_TIMEOUT_S,
        )
    except (OSError, subprocess.TimeoutExpired):
        pass


def _load_gains(path: Path) -> dict[str, float]:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{path} must be a JSON object of controller parameters")
    gains: dict[str, float] = {}
    for name, value in payload.items():
        key = str(name)
        if len(key) > 16:
            raise ValueError(f"parameter name {key!r} is longer than 16 characters")
        gains[key] = float(value)
    return gains


def _live(master) -> bool:
    mav = getattr(master, "mav", None)
    return callable(getattr(mav, "set_attitude_target_send", None)) and callable(
        getattr(mav, "set_position_target_local_ned_send", None)
    )


def _wait_heartbeat(master) -> None:
    wait = getattr(master, "wait_heartbeat", None)
    if wait is None:
        return
    if wait(timeout=_HEARTBEAT_TIMEOUT_S) is None:
        raise TimeoutError(f"no heartbeat on {_LINK}")


@dataclass
class _Vehicle:
    armed: bool = False
    offboard: bool = False

    @property
    def ready(self) -> bool:
        return self.armed and self.offboard


def _wire_param(value: float, param_type: int) -> tuple[float, int]:
    """Float params stay on their real type. Integer params go out as int32 bits."""
    if param_type in _REAL_PARAM_TYPES:
        return float(value), int(param_type)
    bits = int(round(float(value)))
    wired = struct.unpack("<f", struct.pack("<i", bits))[0]
    return wired, int(mavutil.mavlink.MAV_PARAM_TYPE_INT32)


def _send_gains(master, gains: dict[str, float]) -> None:
    if not _live(master):
        return
    for name, value in gains.items():
        master.param_fetch_one(name)
        try:
            current = _wait_param(master, name, _PARAM_TIMEOUT_S)
        except TimeoutError:
            print(f"skip {name}: firmware has no such parameter", file=sys.stderr)
            continue
        wired, param_type = _wire_param(float(value), int(current.param_type))
        master.param_set_send(name, wired, param_type)
        echo = _wait_param(master, name, _PARAM_TIMEOUT_S)
        if not _param_matches(echo, float(value)):
            raise ValueError(f"{name} PARAM_VALUE does not match requested {float(value)}")


def _wait_param(master, name: str, timeout_s: float):
    """Wait for this parameter's echo. A missing echo is a timeout."""
    deadline = time.monotonic() + timeout_s
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError(f"no PARAM_VALUE for {name}")
        message = master.recv_match(blocking=True, timeout=min(0.2, remaining))
        if message is None:
            continue
        if _message_type(message) != "PARAM_VALUE":
            continue
        if _param_name(message) == name:
            return message


def _param_name(message) -> str:
    raw = getattr(message, "param_id", "")
    if isinstance(raw, bytes):
        raw = raw.decode("ascii", errors="ignore")
    return str(raw).split("\x00", 1)[0].strip()


def _param_matches(message, requested: float) -> bool:
    echoed = float(message.param_value)
    param_type = getattr(message, "param_type", None)
    if param_type is None or int(param_type) in _REAL_PARAM_TYPES:
        return math.isclose(echoed, requested, rel_tol=_ECHO_REL_TOL, abs_tol=_ECHO_ABS_TOL)
    stored = struct.unpack("<i", struct.pack("<f", echoed))[0]
    return stored == int(round(float(requested)))


def _request_streams(master) -> None:
    for message_id in (
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
    ):
        _command_long(
            master,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            float(message_id),
            1e6 / _STREAM_HZ,
            0,
            0,
            0,
            0,
            0,
        )


def _command_long(master, command: int, *params: float) -> None:
    master.mav.command_long_send(
        int(master.target_system),
        int(master.target_component),
        command,
        0,
        *params,
    )


def _remember_yaw(commanded: dict[str, float], yaw_hold: list[float]) -> dict[str, float]:
    """Keep the latest finite yaw. Roll and pitch reuse it when this sample has none."""
    prepared = dict(commanded)
    if "yaw" in prepared and math.isfinite(float(prepared["yaw"])):
        yaw_hold[0] = float(prepared["yaw"])
    if "roll" in prepared and "pitch" in prepared and "yaw" not in prepared:
        prepared["yaw"] = float(yaw_hold[0])
    return prepared


def _grid_times(t0: float, t1: float, hz: float) -> list[float]:
    step_count = int(math.floor((t1 - t0) * float(hz) + 1e-9))
    times = [t0 + index / float(hz) for index in range(step_count + 1)]
    if t1 - times[-1] > 1e-6:
        times.append(t1)
    return times


def _stream_ticks(records: list[Setpoint], hz: float = _STREAM_HZ) -> list[Setpoint]:
    """50 Hz ticks. Each tick carries the latest finite command at or before it."""
    if hz <= 0:
        raise ValueError(f"hz must be positive, got {hz}")
    if not records:
        return []
    t0 = float(records[0].t_s)
    t1 = float(records[-1].t_s)
    yaw_hold = [0.0]
    index = 0
    carried: Setpoint | None = None
    ticks: list[Setpoint] = []
    for tick_t in _grid_times(t0, t1, hz):
        while index < len(records) and float(records[index].t_s) <= tick_t + 1e-9:
            record = records[index]
            prepared = _remember_yaw(_commanded(record), yaw_hold)
            if _wire(record, prepared) is not None:
                carried = Setpoint(record.step, float(tick_t), prepared, record.ignore)
            index += 1
        if carried is not None:
            ticks.append(
                Setpoint(carried.step, float(tick_t), dict(carried.fields), carried.ignore)
            )
    return ticks


def _hold_setpoint(records: list[Setpoint]) -> Setpoint | None:
    yaw_hold = [0.0]
    for record in records:
        prepared = _remember_yaw(_commanded(record), yaw_hold)
        if _wire(record, prepared) is not None:
            return Setpoint(record.step, float(record.t_s), prepared, record.ignore)
    return None


def _stream(master, records: list[Setpoint], series: dict) -> None:
    """Hold the first finite setpoint until armed in offboard, then play at 50 Hz.

    A master that cannot send setpoints returns without pacing the log. Scored
    samples use the clock that starts when the vehicle is armed in offboard.
    """
    if not _live(master):
        return
    if not records:
        raise TimeoutError("no COMMAND_ACK for OFFBOARD and arm")
    hold = _hold_setpoint(records)
    if hold is None:
        raise TimeoutError("no COMMAND_ACK for OFFBOARD and arm")
    state = _Vehicle()
    arm_started = time.monotonic()
    deadline = arm_started + _ACK_TIMEOUT_S
    last_mode_s = 0.0
    next_tick = arm_started
    period = 1.0 / _STREAM_HZ
    while not state.ready:
        if time.monotonic() > deadline:
            raise TimeoutError("no COMMAND_ACK for OFFBOARD and arm")
        _pace(next_tick)
        next_tick += period
        _poll(master, state, series, None)
        if state.ready:
            break
        wire = _wire(hold, _commanded(hold))
        if wire is not None:
            _transmit(master, hold, wire)
        last_mode_s = _request_if_needed(master, state, last_mode_s, arm_started)
    origin = time.monotonic()
    t0 = float(records[0].t_s)
    for tick in _stream_ticks(records, _STREAM_HZ):
        _pace(origin + (float(tick.t_s) - t0))
        _poll(master, state, series, _replay_time(t0, origin))
        wire = _wire(tick, _commanded(tick))
        if wire is not None:
            _transmit(master, tick, wire)
        if not state.ready:
            last_mode_s = _request_if_needed(master, state, last_mode_s, arm_started)
    _poll(master, state, series, _replay_time(t0, origin))


def _request_if_needed(master, state: _Vehicle, last_mode_s: float, arm_started: float) -> float:
    if state.ready:
        return last_mode_s
    now = time.monotonic()
    if now - last_mode_s < _MODE_PERIOD_S:
        return last_mode_s
    _request_offboard_and_arm(master, now - arm_started)
    return now


def _pace(target_s: float) -> None:
    delay_s = target_s - time.monotonic()
    if delay_s > 0.0:
        time.sleep(delay_s)


def _poll(master, state: _Vehicle, series: dict, t_s: float | None) -> None:
    """Drain queued messages. ``t_s is None`` leaves them out of the score.

    The first time the heartbeat shows armed offboard, stop so the caller
    can start the replay clock before the next sample.
    """
    while True:
        message = master.recv_match(blocking=False)
        if message is None:
            return
        was_ready = state.ready
        _note_vehicle(state, message, master)
        if t_s is not None and was_ready:
            _record(series, message, t_s)
        if not was_ready and state.ready:
            return


def _note_vehicle(state: _Vehicle, message, master) -> None:
    if _message_type(message) != "HEARTBEAT":
        return
    source = getattr(message, "get_srcSystem", None)
    target = getattr(master, "target_system", None)
    if callable(source) and target is not None and int(source()) != int(target):
        return
    base_mode = int(message.base_mode)
    state.armed = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    main_mode = (int(message.custom_mode) & 0xFF0000) >> 16
    state.offboard = main_mode == mavutil.PX4_CUSTOM_MAIN_MODE_OFFBOARD


def _request_offboard_and_arm(master, elapsed_s: float) -> None:
    base_mode, main_mode, sub_mode = mavutil.px4_map["OFFBOARD"]
    _command_long(
        master,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        float(base_mode),
        float(main_mode),
        float(sub_mode),
        0,
        0,
        0,
        0,
    )
    force = _PX4_FORCE_ARM if elapsed_s >= _FORCE_ARM_AFTER_S else 0.0
    _command_long(
        master,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        1,
        force,
        0,
        0,
        0,
        0,
        0,
    )


def _commanded(record: Setpoint) -> dict[str, float]:
    """Copy finite ``fields`` only. Ignored and omitted names stay out."""
    blocked = set(record.ignore)
    if "rates" in record.ignore:
        blocked.update(_RATE_NAMES)
    if "attitude" in record.ignore:
        blocked.update(_ANGLE_NAMES)
    commanded: dict[str, float] = {}
    for name, value in record.fields.items():
        if name in blocked:
            continue
        number = float(value)
        if not math.isfinite(number):
            continue
        commanded[name] = number
    return commanded


def _wire(record: Setpoint, commanded: dict[str, float]):
    if record.step in ("rate", "angle"):
        return _attitude_wire(record, commanded)
    if record.step in ("velocity", "position"):
        return _ned_wire(commanded)
    raise ValueError(f"unknown cascade step: {record.step}")


def _attitude_wire(record: Setpoint, commanded: dict[str, float]):
    """Mask every axis that is not in ``commanded``. Do not fill it with 0."""
    mask = (
        _ROLL_RATE_IGNORE
        | _PITCH_RATE_IGNORE
        | _YAW_RATE_IGNORE
        | _THROTTLE_IGNORE
        | _ATTITUDE_IGNORE
    )
    both_rates = "roll_rate" in commanded and "pitch_rate" in commanded
    if both_rates:
        mask &= ~(_ROLL_RATE_IGNORE | _PITCH_RATE_IGNORE)
    if "yaw_rate" in commanded and "yaw_rate" not in record.ignore:
        mask &= ~_YAW_RATE_IGNORE
    if "thrust_z" in commanded:
        mask &= ~_THROTTLE_IGNORE
    roll_pitch = "roll" in commanded and "pitch" in commanded
    if roll_pitch and "attitude" not in record.ignore:
        yaw = commanded["yaw"] if "yaw" in commanded else 0.0
        mask &= ~_ATTITUDE_IGNORE
        quaternion = _quaternion(commanded["roll"], commanded["pitch"], yaw)
    else:
        quaternion = (1.0, 0.0, 0.0, 0.0)
    if mask & _ATTITUDE_IGNORE and not both_rates:
        return None
    return (
        "attitude",
        mask,
        quaternion,
        commanded.get("roll_rate", math.nan),
        commanded.get("pitch_rate", math.nan),
        commanded.get("yaw_rate", math.nan),
        -commanded["thrust_z"] if "thrust_z" in commanded else math.nan,
    )


def _quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    half_roll, half_pitch, half_yaw = roll * 0.5, pitch * 0.5, yaw * 0.5
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def _ned_wire(commanded: dict[str, float]):
    mask = 0
    values: list[float] = []
    commanded_motion = False
    for name, bit in _NED_AXES:
        mask |= bit
        if name in commanded:
            mask &= ~bit
            values.append(commanded[name])
            if name not in ("yaw", "yaw_rate"):
                commanded_motion = True
        else:
            values.append(math.nan)
    if not commanded_motion:
        return None
    return ("ned", mask, values)


def _transmit(master, record: Setpoint, wire) -> None:
    system = int(getattr(master, "target_system", 0) or 0)
    component = int(getattr(master, "target_component", 0) or 0)
    time_boot_ms = int(float(record.t_s) * 1000.0)
    kind = wire[0]
    if kind == "attitude":
        _, mask, quaternion, roll_rate, pitch_rate, yaw_rate, thrust = wire
        master.mav.set_attitude_target_send(
            time_boot_ms,
            system,
            component,
            mask,
            quaternion,
            roll_rate,
            pitch_rate,
            yaw_rate,
            thrust,
        )
        return
    _, mask, values = wire
    master.mav.set_position_target_local_ned_send(
        time_boot_ms,
        system,
        component,
        _NED_FRAME,
        mask,
        *values,
    )


def _message_type(message) -> str:
    get_type = getattr(message, "get_type", None)
    if get_type is None:
        return ""
    return str(get_type())


def _replay_time(t0: float, origin: float) -> float:
    return t0 + (time.monotonic() - origin)


def _record(series: dict, message, t_s: float) -> None:
    kind = _message_type(message)
    if kind == "ATTITUDE":
        _append(series, "roll_rate", t_s, message.rollspeed)
        _append(series, "pitch_rate", t_s, message.pitchspeed)
        _append(series, "roll", t_s, message.roll)
        _append(series, "pitch", t_s, message.pitch)
    elif kind == "LOCAL_POSITION_NED":
        _append(series, "x", t_s, message.x)
        _append(series, "y", t_s, message.y)
        _append(series, "z", t_s, message.z)
        _append(series, "vx", t_s, message.vx)
        _append(series, "vy", t_s, message.vy)
        _append(series, "vz", t_s, message.vz)


def _append(series: dict, name: str, t_s: float, value: float) -> None:
    series[name][0].append(float(t_s))
    series[name][1].append(float(value))
