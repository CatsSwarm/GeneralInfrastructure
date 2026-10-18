"""Coordinate-descent neighbors for iris dynamics."""

from __future__ import annotations

from dataclasses import fields, replace

from calibration.iris_dynamics import IrisDynamics

BOUNDS: dict[str, tuple[float, float]] = {
    "mass_kg": (0.4, 2.0),
    "ixx": (0.005, 0.08),
    "iyy": (0.005, 0.08),
    "izz": (0.01, 0.12),
    "motor_constant": (1e-6, 2e-5),
    "moment_constant": (0.01, 0.2),
    "time_constant_up": (0.005, 0.08),
    "time_constant_down": (0.005, 0.08),
    "rotor_drag_coefficient": (0.0, 0.002),
}


def _clamp(field_name: str, value: float) -> float:
    low, high = BOUNDS[field_name]
    if value < low:
        return low
    if value > high:
        return high
    return value


def labeled_candidates(center: IrisDynamics) -> list[tuple[str, IrisDynamics]]:
    """Center first, then one-field neighbors in dataclass field order, low then high."""
    rows: list[tuple[str, IrisDynamics]] = [("center", center)]
    for item in fields(IrisDynamics):
        current = getattr(center, item.name)
        for factor in (0.5, 2.0):
            proposed = _clamp(item.name, current * factor)
            if proposed == current:
                continue
            rows.append(
                (
                    f"{item.name}*{factor:g}",
                    replace(center, **{item.name: proposed}),
                )
            )
    return rows


def neighbors(center: IrisDynamics) -> list[IrisDynamics]:
    return [dynamics for name, dynamics in labeled_candidates(center) if name != "center"]


def candidate_count(center: IrisDynamics) -> int:
    return 1 + len(neighbors(center))
