"""Stable cascade convergence directory and the previous-loop check."""

from __future__ import annotations

import json
from pathlib import Path

PREVIOUS = {
    "rate": None,
    "angle": "rate",
    "velocity": "angle",
    "position": "velocity",
}


def converged_path(stage_root: Path, step: str) -> Path:
    """``stage_root/stages/<step>/best.json``."""
    return Path(stage_root) / "stages" / step / "best.json"


def load_converged(stage_root: Path, step: str) -> dict:
    """Load one loop's best trial. Missing file raises ``FileNotFoundError``."""
    path = converged_path(stage_root, step)
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or payload.get("step") != step:
        raise ValueError(f"{path} step does not match {step!r}")
    return payload
