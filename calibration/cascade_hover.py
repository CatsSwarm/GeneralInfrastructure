"""Hover command from one PX4 ULG."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from pyulog import ULog

HOVER_ABS_VZ_MPS: float = 0.1


def hover_mean(
    landed: list[bool],
    vz: list[float],
    motors: list[tuple[float, float, float, float]],
) -> float:
    total = 0.0
    count = 0
    for is_landed, speed, controls in zip(landed, vz, motors, strict=True):
        if (not is_landed) and abs(speed) < HOVER_ABS_VZ_MPS:
            total += float(controls[0]) + float(controls[1]) + float(controls[2]) + float(controls[3])
            count += 4
    if count == 0:
        raise ValueError("hover window is empty")
    return total / count


def _nearest_indices(source_t: np.ndarray, query_t: np.ndarray) -> np.ndarray:
    index = np.searchsorted(source_t, query_t, side="left")
    index = np.clip(index, 0, len(source_t) - 1)
    earlier = np.clip(index - 1, 0, len(source_t) - 1)
    use_earlier = np.abs(query_t - source_t[earlier]) <= np.abs(source_t[index] - query_t)
    return np.where(use_earlier, earlier, index)


def hover_command(path: Path) -> float:
    ulog = ULog(str(path))
    motors = ulog.get_dataset("actuator_motors").data
    position = ulog.get_dataset("vehicle_local_position").data
    detected = ulog.get_dataset("vehicle_land_detected").data
    position_t = np.asarray(position["timestamp"], dtype=np.float64)
    motor_t = np.asarray(motors["timestamp"], dtype=np.float64)
    landed_t = np.asarray(detected["timestamp"], dtype=np.float64)
    interpolated = np.column_stack([
        np.interp(
            position_t,
            motor_t,
            np.asarray(motors[f"control[{channel}]"], dtype=np.float64),
        )
        for channel in range(4)
    ])
    motor_samples = [tuple(float(value) for value in row) for row in interpolated]
    nearest = _nearest_indices(landed_t, position_t)
    landed_flags = [bool(detected["landed"][int(index)]) for index in nearest]
    vz = [float(value) for value in position["vz"]]
    return hover_mean(landed_flags, vz, motor_samples)
