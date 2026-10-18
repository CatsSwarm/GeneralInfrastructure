"""Unshifted velocity RMS and the iris accept rule."""

from __future__ import annotations

import numpy as np

from calibration.ulg_velocity import VelocitySample

ACCEPT_RMS: dict[str, float] = {"vx": 0.20, "vy": 0.10, "vz": 0.08}
AIRBORNE_ABS_Z_M: float = 0.4

_AXES = ("vx", "vy", "vz")


def _column(samples: list[VelocitySample], axis: str) -> np.ndarray:
    return np.array([getattr(sample, axis) for sample in samples], dtype=float)


def _on_response(
    samples: list[VelocitySample], t_response: np.ndarray
) -> dict[str, np.ndarray]:
    t_src = _column(samples, "t_s")
    return {
        axis: np.interp(t_response, t_src, _column(samples, axis)) for axis in _AXES
    }


def rms_velocity(
    sim: list[VelocitySample],
    response: list[VelocitySample],
    z_m: list[float],
    command: list[VelocitySample],
) -> dict[str, float]:
    if len(z_m) != len(response):
        raise ValueError("z_m and response must have the same length")
    t_response = _column(response, "t_s")
    sim_on = _on_response(sim, t_response)
    command_on = _on_response(command, t_response)
    finite_command = np.ones(len(response), dtype=bool)
    for axis in _AXES:
        finite_command &= np.isfinite(command_on[axis])
    keep = (np.abs(np.asarray(z_m, dtype=float)) > AIRBORNE_ABS_Z_M) & finite_command
    if not np.any(keep):
        raise ValueError("no airborne samples with a finite setpoint")
    scored: dict[str, float] = {}
    for axis in _AXES:
        error = sim_on[axis][keep] - _column(response, axis)[keep]
        scored[axis] = float(np.sqrt(np.mean(np.square(error))))
    return scored


def passes(per_log_rms: list[dict[str, float]]) -> bool:
    return all(row[axis] <= ACCEPT_RMS[axis] for row in per_log_rms for axis in _AXES)


def _total(per_log_rms: list[dict[str, float]]) -> float:
    return sum(row[axis] for row in per_log_rms for axis in _AXES)


def select_best(
    candidates: list[tuple[str, list[dict[str, float]]]],
) -> tuple[str, bool]:
    if not candidates:
        raise ValueError("candidates is empty")
    accepted = [item for item in candidates if passes(item[1])]
    pool = accepted if accepted else list(candidates)
    name, _logs = min(pool, key=lambda item: _total(item[1]))
    return name, bool(accepted)
