#!/usr/bin/env python3
"""Write the iris-versus-SimCon velocity report.

Iris velocities are CSV files written by the replay, columns ``t_s,vx,vy,vz``.
One file per Baseball log stem and one per synthetic case name, read from
``--iris-dir`` (default ``calibration/results/iris``). SimCon ``fly`` receives
the log command with non-finite samples dropped, or ``stream_at`` of a synthetic
case at 50 Hz from t=0 through ``duration_s``. A missing log iris CSV stores
that row's iris RMS fields as null and forces ``accepted`` false. A missing
synthetic CSV stores null ``iris_vs_simcon_rms`` and does not change
``accepted``. ``passes`` is not changed, and a SimCon RMS does not change
``accepted``.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import fields
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from calibration.iris_dynamics import load_dynamics
from calibration.offboard_stream import stream_at
from calibration.report import REPORT_PATH, build_report, write_report
from calibration.score import rms_velocity
from calibration.simcon_velocity import fly
from calibration.synthetic import SyntheticCase, synthetic_cases
from calibration.ulg_velocity import (
    BASEBALL_DIR,
    BASEBALL_LOGS,
    VelocitySample,
    load_log,
)

_REPO_ROOT = Path(__file__).resolve().parents[1]
_DYNAMICS_JSON = Path("calibration/iris_dynamics.json")
_IRIS_DIR = Path("calibration/results/iris")
_AXES = ("vx", "vy", "vz")
_CSV_HEADER = ["t_s", "vx", "vy", "vz"]
_STREAM_HZ = 50.0


def iris_vs_simcon_rms(iris: list, simcon: list) -> dict[str, float]:
    """RMS of SimCon interpolated onto iris timestamps. No z mask and no shift."""
    if not iris or not simcon:
        raise ValueError("iris and simcon must be non-empty")
    t_iris = _times(iris)
    t_sim = _times(simcon)
    if t_sim.size > 1 and np.any(np.diff(t_sim) < 0.0):
        raise ValueError("simcon times must be non-decreasing")
    scored: dict[str, float] = {}
    for axis in _AXES:
        sim_on = np.interp(t_iris, t_sim, _column(simcon, axis))
        error = sim_on - _column(iris, axis)
        scored[axis] = float(np.sqrt(np.mean(np.square(error))))
    return scored


def simcon_rms_against_log(sim_samples, log) -> dict[str, float]:
    return rms_velocity(sim_samples, log.response, log.z_m, log.command)


def _times(samples: list[VelocitySample]) -> np.ndarray:
    return np.array([sample.t_s for sample in samples], dtype=float)


def _column(samples: list[VelocitySample], axis: str) -> np.ndarray:
    return np.array([getattr(sample, axis) for sample in samples], dtype=float)


def _null_rms() -> dict[str, None]:
    return {axis: None for axis in _AXES}


def _finite_command(samples: list[VelocitySample]) -> list[VelocitySample]:
    return [
        sample
        for sample in samples
        if math.isfinite(sample.vx) and math.isfinite(sample.vy) and math.isfinite(sample.vz)
    ]


def _synthetic_command(case: SyntheticCase) -> list[VelocitySample]:
    step_count = int(math.floor(case.duration_s * _STREAM_HZ + 1e-9))
    samples: list[VelocitySample] = []
    for index in range(step_count + 1):
        t_s = case.duration_s if index == step_count else index / _STREAM_HZ
        vx, vy, vz = case.velocity_at(t_s)
        samples.append(VelocitySample(t_s, vx, vy, vz))
    return stream_at(samples, hz=_STREAM_HZ)


def load_iris_csv(path: Path) -> list[VelocitySample]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        try:
            header = next(reader)
        except StopIteration as exc:
            raise ValueError(f"{path} is empty") from exc
        if [cell.strip() for cell in header] != _CSV_HEADER:
            raise ValueError(f"{path} columns must be t_s,vx,vy,vz")
        samples: list[VelocitySample] = []
        for row in reader:
            if not row or all(not cell.strip() for cell in row):
                continue
            if len(row) != 4:
                raise ValueError(f"{path} row does not have 4 columns")
            t_s, vx, vy, vz = (float(cell) for cell in row)
            samples.append(VelocitySample(t_s, vx, vy, vz))
    if not samples:
        raise ValueError(f"{path} has no velocity samples")
    return samples


def _iris_csv(directory: Path, stem: str) -> Path:
    return directory / f"{stem}.csv"


def _log_row(log, iris_dir: Path) -> dict:
    command = _finite_command(log.command)
    if not command:
        raise ValueError(f"{log.path} has no finite command samples")
    simcon_rms = simcon_rms_against_log(fly(command), log)
    stem = Path(log.path).stem
    csv_path = _iris_csv(iris_dir, stem)
    if csv_path.is_file():
        iris_rms = rms_velocity(load_iris_csv(csv_path), log.response, log.z_m, log.command)
    else:
        iris_rms = _null_rms()
    return {"name": stem, "iris_rms": iris_rms, "simcon_rms": simcon_rms}


def _synthetic_row(case: SyntheticCase, iris_dir: Path) -> dict:
    simcon = fly(_synthetic_command(case))
    csv_path = _iris_csv(iris_dir, case.name)
    if csv_path.is_file():
        compared = iris_vs_simcon_rms(load_iris_csv(csv_path), simcon)
    else:
        compared = _null_rms()
    return {"name": case.name, "iris_vs_simcon_rms": compared}


def _has_null_iris(rows: list[dict], key: str) -> bool:
    return any(row[key][axis] is None for row in rows for axis in _AXES)


def _report(dynamics, log_rows: list[dict], synthetic_rows: list[dict]) -> dict:
    # passes cannot score a null log iris_rms. Synthetic nulls stay on the row.
    # When every log iris_rms is numeric, accepted is passes on the log rows.
    if _has_null_iris(log_rows, "iris_rms"):
        return {
            "dynamics": {item.name: getattr(dynamics, item.name) for item in fields(dynamics)},
            "logs": list(log_rows),
            "synthetic": list(synthetic_rows),
            "accepted": False,
        }
    return build_report(dynamics, log_rows, synthetic_rows)


def _format_rms(rms: dict) -> str:
    parts = []
    for axis in _AXES:
        value = rms[axis]
        text = "null" if value is None else f"{value:.6g}"
        parts.append(f"{axis}={text}")
    return " ".join(parts)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Write the iris-versus-SimCon velocity report")
    parser.add_argument(
        "--iris-dir",
        type=Path,
        default=_REPO_ROOT / _IRIS_DIR,
        help="replay CSVs named <ulg-stem>.csv and <synthetic-case>.csv",
    )
    parser.add_argument(
        "--dynamics",
        type=Path,
        default=_REPO_ROOT / _DYNAMICS_JSON,
        help="iris dynamics JSON recorded in the report",
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=_REPO_ROOT / REPORT_PATH,
        help="output report path",
    )
    args = parser.parse_args(argv)

    dynamics = load_dynamics(args.dynamics)
    log_rows = [_log_row(load_log(BASEBALL_DIR / name), args.iris_dir) for name in BASEBALL_LOGS]
    for row in log_rows:
        print(f"{row['name']} iris {_format_rms(row['iris_rms'])} simcon {_format_rms(row['simcon_rms'])}", flush=True)
    synthetic_rows = [_synthetic_row(case, args.iris_dir) for case in synthetic_cases()]
    for row in synthetic_rows:
        print(f"{row['name']} iris_vs_simcon {_format_rms(row['iris_vs_simcon_rms'])}", flush=True)
    report = _report(dynamics, log_rows, synthetic_rows)
    write_report(args.report, report)
    print(f"accepted={report['accepted']} wrote {args.report}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
