"""Shared flight wiring for the cascade tune programs."""

from __future__ import annotations

import inspect
from pathlib import Path

from calibration.cascade_flight import WINDOW_END_S, WINDOW_START_S, load_flight, window_flight
from calibration.cascade_score import axis_rms, rms_sum
from calibration.cascade_session import Session

AXES: dict[str, tuple[str, ...]] = {
    "rate": ("roll_rate", "pitch_rate"),
    "angle": ("roll_rate", "pitch_rate", "roll", "pitch"),
    "velocity": ("roll_rate", "pitch_rate", "vx", "vy", "vz"),
    "position": ("roll_rate", "pitch_rate", "x", "y", "z"),
}
_EMPTY_AXIS = "no airborne samples with a finite command"


def load_window(ulg: Path):
    """Scored slice: the same window ``run_cascade`` uses."""
    return window_flight(load_flight(ulg), WINDOW_START_S, WINDOW_END_S)


def stage_session(stage_root: Path, ulg: Path) -> Session:
    """Session rooted at ``stage_root``, not under a timestamp folder."""
    directory = Path(stage_root)
    directory.mkdir(parents=True, exist_ok=True)
    session = Session(directory, ulg)
    session._per_stage = True
    return session


def score_series(step: str, flight, series: dict) -> float:
    """Sum per-axis RMS. Drop an axis whose command is never finite."""
    values: list[float] = []
    empty: ValueError | None = None
    for name in AXES[step]:
        axis = getattr(flight, name)
        t_sim, sim = series[name]
        try:
            values.append(
                axis_rms(
                    axis.t_s,
                    axis.command,
                    axis.response,
                    axis.airborne,
                    t_sim,
                    sim,
                )
            )
        except ValueError as exc:
            if str(exc) != _EMPTY_AXIS:
                raise
            empty = exc
    if not values:
        if empty is None:
            raise ValueError(_EMPTY_AXIS)
        raise empty
    return rms_sum(values)


def _accepts_flight_log(fly) -> bool:
    try:
        return "flight_log" in inspect.signature(fly).parameters
    except (TypeError, ValueError):
        return False


def fly_and_record(
    session: Session,
    fly,
    step: str,
    flight,
    model: Path,
    gains_path: Path,
    parameters: dict,
) -> float:
    """Fly one trial, append it to the session, and return the RMS sum."""
    log_path = session.log_file(step)
    if _accepts_flight_log(fly):
        series = fly(step, flight, model, gains_path, flight_log=log_path)
    else:
        series = fly(step, flight, model, gains_path)
        log_path = None
    score = score_series(step, flight, series)
    session.record(step, parameters, score, flight, series, log_path)
    return score
