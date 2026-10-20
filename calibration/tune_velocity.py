"""Velocity loop: XY and Z velocity PID, scored with the rate axes still in the sum."""

from __future__ import annotations

import json
import sys
from functools import partial
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from calibration.cascade_gains import ANGLE_KEYS, RATE_P_KEYS, gain_file, save_gains, search_keys
from calibration.cascade_search import search_knobs
from calibration.cascade_sitl import fly_log
from calibration.cascade_stage import PREVIOUS, load_converged
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
_INNER = RATE_P_KEYS + ANGLE_KEYS


def _restored_inner(gains_path: Path, converged: dict) -> dict[str, float]:
    """Rate and angle P from the earlier loops.

    An existing gain file wins over the angle stage. Only keys present in
    that source are kept. Velocity trials override the six velocity gains.
    """
    path = Path(gains_path)
    if path.is_file():
        source = json.loads(path.read_text(encoding="utf-8"))
    else:
        source = converged.get("parameters") or {}
    if not isinstance(source, dict):
        return {}
    return {key: float(source[key]) for key in _INNER if key in source}


def tune_velocity(
    ulg: Path,
    model: Path,
    gains_path: Path,
    fly,
    report_path: Path,
    stage_root: Path,
) -> dict[str, float]:
    """Search the six velocity PID gains. Leave rate P and angle P alone.

    Those four gains come from the gain file when that file exists, otherwise
    from the angle stage parameters when those keys are present. Velocity
    trials override only the velocity gains. ``search_knobs`` can leave the
    gain file on a rejected factor. The kept velocity gains are written back
    before the stage is saved, with the same inner-loop gains.
    """
    converged = load_converged(stage_root, PREVIOUS["velocity"])
    flight = load_window(ulg)
    initial = flight.initial_parameters
    keys = search_keys("velocity")
    inner = _restored_inner(gains_path, converged)
    session = stage_session(stage_root, ulg)

    def overrides(knobs: dict[str, float]) -> dict[str, float]:
        return {**inner, **knobs}

    def evaluate(knobs: dict[str, float]) -> float:
        gains = gain_file(initial, overrides(knobs))
        save_gains(gains_path, gains)
        return fly_and_record(
            session,
            fly,
            "velocity",
            flight,
            model,
            gains_path,
            gains,
        )

    best, score = search_knobs(
        {key: float(initial[key]) for key in keys},
        evaluate,
    )
    save_gains(gains_path, gain_file(initial, overrides(best)))
    session.save_stage("velocity")
    scores = {"velocity": float(score)}
    report = Path(report_path)
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(json.dumps(scores, indent=2) + "\n", encoding="utf-8")
    return scores


def tune_velocity_main(
    stage_root: Path | None = None,
    argv: list[str] | None = None,
) -> int:
    """Tune the velocity loop. Missing angle convergence returns 2 and does not fly."""
    args = [] if argv is None else list(argv)
    if len(args) > 1:
        print("usage: python3 -m calibration.tune_velocity [ULG]", file=sys.stderr)
        return 2
    stage = _repo_path(_DEFAULT_STAGE) if stage_root is None else Path(stage_root)
    previous = PREVIOUS["velocity"]
    try:
        load_converged(stage, previous)
    except (FileNotFoundError, ValueError):
        print(
            f"{previous} has not converged; run python3 -m calibration.tune_{previous}",
            file=sys.stderr,
        )
        return 2
    ulg = Path(args[0]) if args else _repo_path(_DEFAULT_ULG)
    tune_velocity(
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
        stage,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(tune_velocity_main(argv=sys.argv[1:]))
