"""Rate loop: frozen geometry, hover thrust, then motor lag and rate P."""

from __future__ import annotations

import json
import sys
from functools import partial
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from calibration.cascade_gains import RATE_P_KEYS, gain_file, save_gains
from calibration.cascade_model import read_arms, read_plant, write_model
from calibration.cascade_search import search_knobs
from calibration.cascade_sitl import fly_log
from calibration.cascade_thrust import motor_constant
from calibration.cascade_tune import fly_and_record, load_window, stage_session
from calibration.run_cascade import (
    SHOW_GAZEBO,
    _DEFAULT_GAINS,
    _DEFAULT_MODEL,
    _DEFAULT_REPORT,
    _DEFAULT_SITL_LOG,
    _DEFAULT_ULG,
    _connect,
    _repo_path,
    popen_sim,
)

_DEFAULT_STAGE = Path("calibration/results/cascade")


def _down(plant: dict[str, float], time_constant_up: float) -> float:
    return time_constant_up * (plant["time_constant_down"] / plant["time_constant_up"])


def _write_plant(
    model: Path,
    hover_constant: float,
    arm_x: float,
    arm_y: float,
    plant: dict[str, float],
    time_constant_up: float,
) -> None:
    write_model(
        model,
        hover_constant,
        arm_x,
        arm_y,
        time_constant_up=time_constant_up,
        time_constant_down=_down(plant, time_constant_up),
        prop_radius=plant["prop_radius"],
        max_rot_velocity=plant["max_rot_velocity"],
    )


def _parameters(
    gains: dict[str, float],
    hover_constant: float,
    arm_x: float,
    arm_y: float,
    plant: dict[str, float],
    time_constant_up: float,
) -> dict[str, float]:
    return {
        **gains,
        "motorConstant": hover_constant,
        "arm_x": arm_x,
        "arm_y": arm_y,
        "time_constant_up": time_constant_up,
        "time_constant_down": _down(plant, time_constant_up),
        "prop_radius": plant["prop_radius"],
        "max_rot_velocity": plant["max_rot_velocity"],
    }


def tune_rate(
    ulg: Path,
    model: Path,
    gains_path: Path,
    fly,
    report_path: Path,
    stage_root: Path,
) -> dict[str, float]:
    """Search lag, then roll and pitch rate P. Fly only the rate step.

    Arms, prop radius, and ``max_rot_velocity`` stay at the model values.
    Hover ``motorConstant`` is written at that radius. ``timeConstantDown``
    keeps the ratio read before the search. The model and the gain file are
    rewritten to the lowest rate score after both searches.
    """
    flight = load_window(ulg)
    hover_constant = motor_constant(flight.hover_command)
    arm_x, arm_y = read_arms(model)
    plant = read_plant(model)
    initial = flight.initial_parameters
    session = stage_session(stage_root, ulg)
    kept = {
        "score": None,
        "up": plant["time_constant_up"],
        "p": {key: float(initial[key]) for key in RATE_P_KEYS},
    }

    def consider(score: float, time_constant_up: float, gains: dict[str, float]) -> None:
        if kept["score"] is None or score < kept["score"]:
            kept["score"] = score
            kept["up"] = time_constant_up
            kept["p"] = {key: float(gains[key]) for key in RATE_P_KEYS}

    def evaluate_lag(knobs: dict[str, float]) -> float:
        time_constant_up = knobs["time_constant_up"]
        _write_plant(model, hover_constant, arm_x, arm_y, plant, time_constant_up)
        gains = gain_file(initial)
        save_gains(gains_path, gains)
        score = fly_and_record(
            session,
            fly,
            "rate",
            flight,
            model,
            gains_path,
            _parameters(gains, hover_constant, arm_x, arm_y, plant, time_constant_up),
        )
        consider(score, time_constant_up, gains)
        return score

    lag_best, _lag_score = search_knobs(
        {"time_constant_up": plant["time_constant_up"]},
        evaluate_lag,
    )
    # search_knobs leaves the last factor applied. Rate P is scored at the kept lag.
    kept_up = lag_best["time_constant_up"]
    _write_plant(model, hover_constant, arm_x, arm_y, plant, kept_up)

    def evaluate_p(knobs: dict[str, float]) -> float:
        gains = gain_file(initial, knobs)
        save_gains(gains_path, gains)
        score = fly_and_record(
            session,
            fly,
            "rate",
            flight,
            model,
            gains_path,
            _parameters(gains, hover_constant, arm_x, arm_y, plant, kept_up),
        )
        consider(score, kept_up, gains)
        return score

    search_knobs({key: float(initial[key]) for key in RATE_P_KEYS}, evaluate_p)
    _write_plant(model, hover_constant, arm_x, arm_y, plant, kept["up"])
    save_gains(gains_path, gain_file(initial, kept["p"]))
    session.save_stage("rate")
    scores = {"rate": float(kept["score"])}
    report = Path(report_path)
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(json.dumps(scores, indent=2) + "\n", encoding="utf-8")
    return scores


def main(argv: list[str] | None = None) -> int:
    """Tune the rate loop. No arguments uses the cascade log and gain file."""
    args = list(sys.argv[1:] if argv is None else argv)
    if len(args) > 1:
        print("usage: python3 -m calibration.tune_rate [ULG]", file=sys.stderr)
        return 2
    ulg = Path(args[0]) if args else _repo_path(_DEFAULT_ULG)
    tune_rate(
        ulg,
        _repo_path(_DEFAULT_MODEL),
        _repo_path(_DEFAULT_GAINS),
        partial(
            fly_log,
            runner=partial(popen_sim, log_path=_repo_path(_DEFAULT_SITL_LOG)),
            connect=_connect,
            show_gazebo=SHOW_GAZEBO,
        ),
        _repo_path(_DEFAULT_REPORT),
        _repo_path(_DEFAULT_STAGE),
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
