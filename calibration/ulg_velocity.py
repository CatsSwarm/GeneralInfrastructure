"""Load PX4 ULG velocity command, response, and controller parameters."""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

from pyulog import ULog

BASEBALL_DIR = Path("/home/valentin/RL/TESTFLIGHTS/Baseball")
BASEBALL_LOGS: tuple[str, str, str] = (
    "Group3RLcat2_log_41_2026-8-25-17-05-30.ulg",
    "Group3RLcat3_log_18_2026-8-25-17-05-26.ulg",
    "Group3RLcat4_log_28_2026-8-25-17-05-24.ulg",
)

_CONTROLLER_KEY = re.compile(r"^(?:MPC_|MC_)")
_US_TO_S = 1e-6


@dataclass(frozen=True)
class VelocitySample:
    t_s: float
    vx: float
    vy: float
    vz: float


@dataclass(frozen=True)
class LogVelocity:
    path: str
    command: list[VelocitySample]
    response: list[VelocitySample]
    z_m: list[float]
    initial_parameters: dict[str, float]


def controller_parameters(initial: dict[str, float]) -> dict[str, float]:
    return {key: value for key, value in initial.items() if _CONTROLLER_KEY.match(key)}


def _seconds_from(timestamps, origin_us: float) -> list[float]:
    return [(float(timestamp) - origin_us) * _US_TO_S for timestamp in timestamps]


def _velocity_samples(data, times_s: list[float]) -> list[VelocitySample]:
    vx = data["vx"]
    vy = data["vy"]
    vz = data["vz"]
    return [
        VelocitySample(times_s[index], float(vx[index]), float(vy[index]), float(vz[index]))
        for index in range(len(times_s))
    ]


def load_log(path: Path) -> LogVelocity:
    ulog = ULog(str(path))
    response_data = ulog.get_dataset("vehicle_local_position").data
    command_data = ulog.get_dataset("vehicle_local_position_setpoint").data
    origin_us = float(response_data["timestamp"][0])
    response_times = _seconds_from(response_data["timestamp"], origin_us)
    command_times = _seconds_from(command_data["timestamp"], origin_us)
    return LogVelocity(
        path=str(path),
        command=_velocity_samples(command_data, command_times),
        response=_velocity_samples(response_data, response_times),
        z_m=[float(value) for value in response_data["z"]],
        initial_parameters={
            key: float(value) for key, value in ulog.initial_parameters.items()
        },
    )
