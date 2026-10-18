"""Load one PX4 ULG into cascade rate, angle, velocity, and position series."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from pyulog import ULog

from calibration.cascade_hover import hover_command

_US_TO_S = 1e-6


@dataclass(frozen=True)
class AxisLog:
    t_s: tuple[float, ...]
    command: tuple[float, ...]
    response: tuple[float, ...]
    airborne: tuple[bool, ...]


@dataclass(frozen=True)
class CascadeFlight:
    path: str
    hover_command: float
    initial_parameters: dict[str, float]
    roll_rate: AxisLog
    pitch_rate: AxisLog
    roll: AxisLog
    pitch: AxisLog
    yaw: AxisLog
    vx: AxisLog
    vy: AxisLog
    vz: AxisLog
    x: AxisLog
    y: AxisLog
    z: AxisLog
    thrust_z: AxisLog


def _roll_pitch(w, x, y, z):
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sine_pitch = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
    return roll, np.arcsin(sine_pitch)


def roll_pitch_rad(q: tuple[float, float, float, float]) -> tuple[float, float]:
    """Roll and pitch from a PX4 attitude quaternion (w, x, y, z)."""
    roll, pitch = _roll_pitch(*q)
    return float(roll), float(pitch)


def _nearest_indices(source_t: np.ndarray, query_t: np.ndarray) -> np.ndarray:
    index = np.searchsorted(source_t, query_t, side="left")
    index = np.clip(index, 0, len(source_t) - 1)
    earlier = np.clip(index - 1, 0, len(source_t) - 1)
    use_earlier = np.abs(query_t - source_t[earlier]) <= np.abs(source_t[index] - query_t)
    return np.where(use_earlier, earlier, index)


def _nearest_command(source_t: np.ndarray, source, query_t: np.ndarray) -> np.ndarray:
    values = np.asarray(source, dtype=np.float64)[_nearest_indices(source_t, query_t)]
    return np.where(np.isfinite(values), values, np.nan)


def _airborne(landed_t: np.ndarray, landed, query_t: np.ndarray) -> tuple[bool, ...]:
    nearest = np.asarray(landed)[_nearest_indices(landed_t, query_t)]
    return tuple(not bool(flag) for flag in nearest)


def _seconds(timestamps: np.ndarray, origin_us: float) -> tuple[float, ...]:
    return tuple(float(value) for value in (timestamps - origin_us) * _US_TO_S)


def _floats(values) -> tuple[float, ...]:
    return tuple(float(value) for value in np.asarray(values, dtype=np.float64))


def _axis(
    t_s: tuple[float, ...],
    command,
    response,
    airborne: tuple[bool, ...],
) -> AxisLog:
    command_values = command if isinstance(command, tuple) else _floats(command)
    response_values = response if isinstance(response, tuple) else _floats(response)
    if len({len(t_s), len(command_values), len(response_values), len(airborne)}) != 1:
        raise ValueError("axis series lengths differ")
    return AxisLog(t_s, command_values, response_values, airborne)


def _roll_pitch_series(attitude) -> tuple[np.ndarray, np.ndarray]:
    return _roll_pitch(
        np.asarray(attitude["q[0]"], dtype=np.float64),
        np.asarray(attitude["q[1]"], dtype=np.float64),
        np.asarray(attitude["q[2]"], dtype=np.float64),
        np.asarray(attitude["q[3]"], dtype=np.float64),
    )


def load_flight(path: Path) -> CascadeFlight:
    """Read one ULG. Commands are the nearest sample on each response time."""
    ulog = ULog(str(path))
    rates = ulog.get_dataset("vehicle_rates_setpoint").data
    angular = ulog.get_dataset("vehicle_angular_velocity").data
    attitude_setpoint = ulog.get_dataset("vehicle_attitude_setpoint").data
    attitude = ulog.get_dataset("vehicle_attitude").data
    position_setpoint = ulog.get_dataset("vehicle_local_position_setpoint").data
    position = ulog.get_dataset("vehicle_local_position").data
    detected = ulog.get_dataset("vehicle_land_detected").data

    rate_t = np.asarray(angular["timestamp"], dtype=np.float64)
    angle_t = np.asarray(attitude["timestamp"], dtype=np.float64)
    position_t = np.asarray(position["timestamp"], dtype=np.float64)
    origin_us = float(min(rate_t[0], angle_t[0], position_t[0]))
    landed_t = np.asarray(detected["timestamp"], dtype=np.float64)
    rates_t = np.asarray(rates["timestamp"], dtype=np.float64)
    attitude_setpoint_t = np.asarray(attitude_setpoint["timestamp"], dtype=np.float64)
    position_setpoint_t = np.asarray(position_setpoint["timestamp"], dtype=np.float64)

    rate_s = _seconds(rate_t, origin_us)
    angle_s = _seconds(angle_t, origin_us)
    position_s = _seconds(position_t, origin_us)
    rate_air = _airborne(landed_t, detected["landed"], rate_t)
    angle_air = _airborne(landed_t, detected["landed"], angle_t)
    position_air = _airborne(landed_t, detected["landed"], position_t)
    thrust = _floats(_nearest_command(rates_t, rates["thrust_body[2]"], rate_t))
    roll_response, pitch_response = _roll_pitch_series(attitude)
    yaw_command = _nearest_command(
        attitude_setpoint_t, attitude_setpoint["yaw_body"], angle_t
    )

    def position_command(field: str) -> np.ndarray:
        return _nearest_command(position_setpoint_t, position_setpoint[field], position_t)

    return CascadeFlight(
        path=str(path),
        hover_command=hover_command(path),
        initial_parameters={
            key: float(value) for key, value in ulog.initial_parameters.items()
        },
        roll_rate=_axis(
            rate_s,
            _nearest_command(rates_t, rates["roll"], rate_t),
            angular["xyz[0]"],
            rate_air,
        ),
        pitch_rate=_axis(
            rate_s,
            _nearest_command(rates_t, rates["pitch"], rate_t),
            angular["xyz[1]"],
            rate_air,
        ),
        roll=_axis(
            angle_s,
            _nearest_command(attitude_setpoint_t, attitude_setpoint["roll_body"], angle_t),
            roll_response,
            angle_air,
        ),
        pitch=_axis(
            angle_s,
            _nearest_command(attitude_setpoint_t, attitude_setpoint["pitch_body"], angle_t),
            pitch_response,
            angle_air,
        ),
        yaw=_axis(angle_s, yaw_command, yaw_command, angle_air),
        vx=_axis(position_s, position_command("vx"), position["vx"], position_air),
        vy=_axis(position_s, position_command("vy"), position["vy"], position_air),
        vz=_axis(position_s, position_command("vz"), position["vz"], position_air),
        x=_axis(position_s, position_command("x"), position["x"], position_air),
        y=_axis(position_s, position_command("y"), position["y"], position_air),
        z=_axis(position_s, position_command("z"), position["z"], position_air),
        thrust_z=_axis(rate_s, thrust, thrust, rate_air),
    )
