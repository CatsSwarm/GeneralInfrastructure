"""Velocity-only offboard setpoint grid for PX4 SITL."""

from __future__ import annotations

import numpy as np

from calibration.ulg_velocity import VelocitySample

# Ignore position, acceleration, yaw, and yaw rate. Use vx, vy, vz.
VELOCITY_ONLY_TYPEMASK: int = 3527
MAV_FRAME_LOCAL_NED: int = 1

_OFFBOARD_BIND_PORT_BASE = 14540


def offboard_bind_port(px4_instance: int) -> int:
    if px4_instance < 0:
        raise ValueError(f"px4_instance must be >= 0, got {px4_instance}")
    return _OFFBOARD_BIND_PORT_BASE + px4_instance


def stream_at(samples: list[VelocitySample], hz: float = 50.0) -> list[VelocitySample]:
    """Sample the series on a uniform clock. No time shift and no lag search."""
    if hz <= 0:
        raise ValueError(f"hz must be positive, got {hz}")
    if not samples:
        raise ValueError("samples must be non-empty")
    times = np.asarray([sample.t_s for sample in samples], dtype=float)
    if times.size > 1 and np.any(np.diff(times) < 0.0):
        raise ValueError("sample times must be non-decreasing")
    t0 = float(times[0])
    t1 = float(times[-1])
    step_count = int(np.floor((t1 - t0) * float(hz) + 1e-9))
    grid_t = t0 + np.arange(step_count + 1, dtype=float) / float(hz)
    # Keep a 1/hz step, and include the final sample when it falls between steps.
    if t1 - float(grid_t[-1]) > 1e-6:
        grid_t = np.append(grid_t, t1)
    vx = np.interp(grid_t, times, np.asarray([sample.vx for sample in samples], dtype=float))
    vy = np.interp(grid_t, times, np.asarray([sample.vy for sample in samples], dtype=float))
    vz = np.interp(grid_t, times, np.asarray([sample.vz for sample in samples], dtype=float))
    return [
        VelocitySample(float(grid_t[index]), float(vx[index]), float(vy[index]), float(vz[index]))
        for index in range(int(grid_t.size))
    ]
