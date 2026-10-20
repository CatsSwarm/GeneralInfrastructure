"""Logged MC_/MPC_ gains, searchable rate P, and search-knob refusals."""

import json
import re
from pathlib import Path

RATE_P_KEYS: tuple[str, ...] = ("MC_ROLLRATE_P", "MC_PITCHRATE_P")
RATE_KEYS: tuple[str, ...] = (
    "MC_ROLLRATE_P",
    "MC_ROLLRATE_I",
    "MC_ROLLRATE_D",
    "MC_ROLLRATE_FF",
    "MC_ROLLRATE_K",
    "MC_PITCHRATE_P",
    "MC_PITCHRATE_I",
    "MC_PITCHRATE_D",
    "MC_PITCHRATE_FF",
    "MC_PITCHRATE_K",
)
ANGLE_KEYS: tuple[str, ...] = ("MC_ROLL_P", "MC_PITCH_P")
VELOCITY_KEYS: tuple[str, ...] = (
    "MPC_XY_VEL_P_ACC",
    "MPC_XY_VEL_I_ACC",
    "MPC_XY_VEL_D_ACC",
    "MPC_Z_VEL_P_ACC",
    "MPC_Z_VEL_I_ACC",
    "MPC_Z_VEL_D_ACC",
)
POSITION_KEYS: tuple[str, ...] = ("MPC_XY_P", "MPC_Z_P")

_SEARCHABLE = frozenset(RATE_P_KEYS + ANGLE_KEYS + VELOCITY_KEYS + POSITION_KEYS)
_CONTROLLER = re.compile(r"^(?:MPC_|MC_)")
_HOVER = "MPC_THR_HOVER"
_HOVER_THRUST = 0.5
_STEPS = {
    "rate": RATE_P_KEYS,
    "angle": ANGLE_KEYS,
    "velocity": VELOCITY_KEYS,
    "position": POSITION_KEYS,
}


def search_keys(step: str) -> tuple[str, ...]:
    """Search knobs for one cascade step. Rate search is P only."""
    try:
        return _STEPS[step]
    except KeyError:
        raise ValueError(f"unknown step {step!r}") from None


def gain_file(
    initial: dict[str, float], overrides: dict[str, float] | None = None
) -> dict[str, float]:
    """Copy logged MC_/MPC_ gains, apply search overrides, force hover to 0.5."""
    if overrides:
        refused = [name for name in overrides if name not in _SEARCHABLE]
        if refused:
            raise ValueError(
                "overrides must be rate P, angle, velocity, or position gains: "
                + ", ".join(refused)
            )
    gained = {
        name: value for name, value in initial.items() if _CONTROLLER.match(name)
    }
    if overrides:
        gained.update(overrides)
    gained[_HOVER] = _HOVER_THRUST
    return gained


def save_gains(path: Path, gains: dict[str, float]) -> None:
    """Write the gain map as indented JSON with a trailing newline."""
    path.write_text(json.dumps(gains, indent=2) + "\n", encoding="utf-8")
