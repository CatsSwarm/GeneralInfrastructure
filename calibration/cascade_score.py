"""Unshifted per-axis RMS of sim minus log."""

from __future__ import annotations

import numpy as np


def axis_rms(
    t_log: list[float],
    command: list[float],
    response: list[float],
    airborne: list[bool],
    t_sim: list[float],
    sim: list[float],
) -> float:
    """RMS of sim interpolated onto log time, with no time shift.

    A sample counts only where that axis's command, the log response,
    and the interpolated sim are finite and the sample is airborne.
    Non-finite commands are left out, not filled in.
    """
    response_v = np.asarray(response, dtype=float)
    sim_on = np.interp(
        np.asarray(t_log, dtype=float),
        np.asarray(t_sim, dtype=float),
        np.asarray(sim, dtype=float),
    )
    keep = (
        np.asarray(airborne, dtype=bool)
        & np.isfinite(np.asarray(command, dtype=float))
        & np.isfinite(response_v)
        & np.isfinite(sim_on)
    )
    if not np.any(keep):
        raise ValueError("no airborne samples with a finite command")
    error = sim_on[keep] - response_v[keep]
    return float(np.sqrt(np.mean(np.square(error))))


def rms_sum(values: list[float]) -> float:
    """Sum of per-axis RMS values. There is no accept threshold."""
    return float(sum(values))
