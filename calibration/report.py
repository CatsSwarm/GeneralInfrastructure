"""Velocity calibration report: iris accept call plus SimCon columns."""

from __future__ import annotations

import json
from dataclasses import fields
from pathlib import Path

from calibration.iris_dynamics import IrisDynamics
from calibration.score import passes

# Relative to the repo root the caller passes in.
REPORT_PATH = Path("calibration/results/velocity_report.json")


def build_report(
    dynamics: IrisDynamics,
    log_rows: list[dict],
    synthetic_rows: list[dict],
) -> dict:
    return {
        "dynamics": {
            item.name: getattr(dynamics, item.name) for item in fields(IrisDynamics)
        },
        "logs": list(log_rows),
        "synthetic": list(synthetic_rows),
        "accepted": passes([row["iris_rms"] for row in log_rows]),
    }


def write_report(path: Path, report: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(report, indent=2, sort_keys=True) + "\n"
    path.write_text(text, encoding="utf-8")
