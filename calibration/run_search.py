#!/usr/bin/env python3
"""Score the current iris dynamics and its coordinate-descent neighbors on the three Baseball logs.

The sim is not restarted here. The host jinja is read when the container starts, so each
neighbor is published and this process waits until a new ``runSimNoeticMulti.sh`` is up.
``--write`` publishes the selected dynamics even when ``accepted`` is false.
Each Baseball replay is a log series, so the sender does not position-hold.
Traces are kept per candidate. After ``select_best``, only the selected
candidate is written to ``calibration/results/iris/<ulg-stem>.csv``, which is
what ``run_compare`` reads.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from calibration.iris_dynamics import IRIS_MODEL_JINJA, load_dynamics, publish_dynamics
from calibration.replay_offboard import replay, write_velocity_csv
from calibration.score import rms_velocity, select_best
from calibration.search import labeled_candidates
from calibration.ulg_velocity import (
    BASEBALL_DIR,
    BASEBALL_LOGS,
    VelocitySample,
    controller_parameters,
    load_log,
)

DYNAMICS_JSON = Path("calibration/iris_dynamics.json")
_REPO_ROOT = Path(__file__).resolve().parents[1]


def iris_csv_path(log_path: Path) -> Path:
    return _REPO_ROOT / "calibration" / "results" / "iris" / f"{Path(log_path).stem}.csv"


def log_series_replay_kwargs(samples, params, px4_instance: int) -> dict:
    return {
        "samples": samples,
        "params": params,
        "px4_instance": px4_instance,
        "log_series": True,
        "position_hold": False,
    }


def _dynamics_json() -> Path:
    return _REPO_ROOT / DYNAMICS_JSON


def _model_path() -> Path:
    return _REPO_ROOT / IRIS_MODEL_JINJA


def _finite_command(samples: list[VelocitySample]) -> list[VelocitySample]:
    return [
        sample
        for sample in samples
        if math.isfinite(sample.vx) and math.isfinite(sample.vy) and math.isfinite(sample.vz)
    ]


def _score_log(log, instance: int) -> tuple[dict[str, float], list[VelocitySample]]:
    command = _finite_command(log.command)
    if not command:
        raise ValueError(f"{log.path} has no finite command samples")
    trace = replay(
        **log_series_replay_kwargs(
            command,
            controller_parameters(log.initial_parameters),
            instance,
        )
    )
    if not trace:
        raise RuntimeError(f"replay recorded no LOCAL_POSITION_NED samples for {log.path}")
    return rms_velocity(trace, log.response, log.z_m, log.command), trace


def commit_selected_traces(
    traces: dict[str, dict[str, list[VelocitySample]]],
    selected: str,
    log_paths: list[Path],
) -> None:
    """Leave the compare CSVs as the selected candidate, not the last neighbor."""
    chosen = traces[selected]
    for log_path in log_paths:
        stem = Path(log_path).stem
        write_velocity_csv(iris_csv_path(log_path), chosen[stem])


def _print_rms(name: str, log_name: str, rms: dict[str, float]) -> None:
    print(
        f"{name} {log_name} vx={rms['vx']:.6g} vy={rms['vy']:.6g} vz={rms['vz']:.6g}",
        flush=True,
    )


def _wait_for_restart(name: str, dynamics_json: Path, model_path: Path) -> None:
    print(
        f"{name} written to {dynamics_json} and {model_path}",
        flush=True,
    )
    print(
        "start a new runSimNoeticMulti.sh so the mount picks up that jinja, then press Enter",
        flush=True,
    )
    try:
        input()
    except EOFError as exc:
        raise SystemExit("no confirmation that the sim was restarted") from exc


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Score iris dynamics neighbors on the three Baseball logs")
    parser.add_argument("--instance", type=int, default=0, help="PX4 SITL instance (default 0)")
    parser.add_argument(
        "--write",
        action="store_true",
        help="publish the selected dynamics to the host json and jinja",
    )
    args = parser.parse_args(argv)

    dynamics_json = _dynamics_json()
    model_path = _model_path()
    center = load_dynamics(dynamics_json)
    candidates = labeled_candidates(center)
    logs = [load_log(BASEBALL_DIR / name) for name in BASEBALL_LOGS]
    snapshot_json = dynamics_json.read_bytes()
    snapshot_model = model_path.read_bytes()
    wrote_winner = False

    try:
        scored: list[tuple[str, list[dict[str, float]]]] = []
        traces: dict[str, dict[str, list[VelocitySample]]] = {}
        chosen_dynamics = {name: dynamics for name, dynamics in candidates}
        for name, dynamics in candidates:
            if name != "center":
                publish_dynamics(dynamics, dynamics_json, model_path)
                _wait_for_restart(name, dynamics_json, model_path)
            else:
                print(
                    "scoring center on the sim that is already running",
                    flush=True,
                )
            per_log: list[dict[str, float]] = []
            traces[name] = {}
            for log in logs:
                rms, trace = _score_log(log, args.instance)
                per_log.append(rms)
                traces[name][Path(log.path).stem] = trace
                _print_rms(name, Path(log.path).name, rms)
            scored.append((name, per_log))
        selected, accepted = select_best(scored)
        commit_selected_traces(traces, selected, [Path(log.path) for log in logs])
        print(f"selected={selected} accepted={accepted}", flush=True)
        if args.write:
            publish_dynamics(chosen_dynamics[selected], dynamics_json, model_path)
            wrote_winner = True
            print(
                f"wrote {selected} to {dynamics_json} and {model_path}",
                flush=True,
            )
            print(
                "start a new runSimNoeticMulti.sh before the next search so the mount shows this file",
                flush=True,
            )
        return 0
    finally:
        if not wrote_winner:
            dynamics_json.write_bytes(snapshot_json)
            model_path.write_bytes(snapshot_model)
            print(
                "dynamics files restored to their contents from the start of this search",
                flush=True,
            )


if __name__ == "__main__":
    raise SystemExit(main())
