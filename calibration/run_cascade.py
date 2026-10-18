"""Cascade order: hover thrust, motor arms, then angle, velocity, and position."""

from __future__ import annotations

import json
import subprocess
import sys
from functools import partial
from pathlib import Path

from calibration.cascade_flight import load_flight
from calibration.cascade_sitl import fly_log
from calibration.cascade_gains import gain_file, save_gains, search_keys
from calibration.cascade_hover import hover_command
from calibration.cascade_model import read_arms, write_model
from calibration.cascade_score import axis_rms, rms_sum
from calibration.cascade_search import search_knobs
from calibration.cascade_thrust import motor_constant

_AXES: dict[str, tuple[str, ...]] = {
    "rate": ("roll_rate", "pitch_rate"),
    "angle": ("roll", "pitch"),
    "velocity": ("vx", "vy", "vz"),
    "position": ("x", "y", "z"),
}
_EMPTY_AXIS = "no airborne samples with a finite command"


def _closeness(step: str, flight, series: dict) -> float:
    """Sum per-axis RMS. Drop an axis whose command is never finite."""
    values: list[float] = []
    empty: ValueError | None = None
    for name in _AXES[step]:
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


def run_cascade(
    ulg: Path,
    model: Path,
    gains_path: Path,
    fly,
    report_path: Path,
) -> dict[str, float]:
    """Write hover thrust, search arms, then outer gains. Return each step's RMS."""
    flight = load_flight(ulg)
    constant = motor_constant(hover_command(ulg))
    arm_x, arm_y = read_arms(model)
    write_model(model, constant, arm_x, arm_y)
    initial = flight.initial_parameters
    save_gains(gains_path, gain_file(initial))

    def evaluate_arms(knobs: dict[str, float]) -> float:
        write_model(model, constant, knobs["arm_x"], knobs["arm_y"])
        return _closeness("rate", flight, fly("rate", flight, model, gains_path))

    arms, rate_score = search_knobs({"arm_x": arm_x, "arm_y": arm_y}, evaluate_arms)
    write_model(model, constant, arms["arm_x"], arms["arm_y"])

    accepted: dict[str, float] = {}
    scores = {"rate": rate_score}

    def evaluate_gains(step: str, knobs: dict[str, float]) -> float:
        save_gains(gains_path, gain_file(initial, {**accepted, **knobs}))
        return _closeness(step, flight, fly(step, flight, model, gains_path))

    for step in ("angle", "velocity", "position"):
        start = {key: initial[key] for key in search_keys(step)}
        best, score = search_knobs(
            start, lambda knobs, step=step: evaluate_gains(step, knobs)
        )
        accepted.update(best)
        save_gains(gains_path, gain_file(initial, accepted))
        scores[step] = score

    report_path.write_text(json.dumps(scores, indent=2) + "\n", encoding="utf-8")
    return scores


_DEFAULT_ULG = Path("calibration/log_2_UnknownDate.ulg")
_DEFAULT_MODEL = Path("calibration/models/iris/iris.sdf.jinja")
_DEFAULT_GAINS = Path("calibration/iris_controller.json")
_DEFAULT_REPORT = Path("calibration/results/cascade_report.json")


def _connect(port: str):
    """Pymavlink ``udpin`` for the first vehicle. ``port`` is ``udpin:0.0.0.0:14541``."""
    from pymavlink import mavutil

    return mavutil.mavlink_connection(port)


def main(argv: list[str] | None = None) -> int:
    """Calibrate the mounted iris. No arguments uses the cascade log and gain file."""
    args = list(sys.argv[1:] if argv is None else argv)
    if len(args) > 1:
        print("usage: python3 -m calibration.run_cascade [ULG]", file=sys.stderr)
        return 2
    ulg = Path(args[0]) if args else _DEFAULT_ULG
    run_cascade(
        ulg,
        _DEFAULT_MODEL,
        _DEFAULT_GAINS,
        partial(fly_log, runner=subprocess.Popen, connect=_connect),
        _DEFAULT_REPORT,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
