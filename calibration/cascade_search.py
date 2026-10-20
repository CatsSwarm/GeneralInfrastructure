"""One-knob sweep that keeps a candidate only when its score strictly falls."""

from __future__ import annotations

from collections.abc import Callable

FACTORS: tuple[float, ...] = (0.7, 0.85, 1.15, 1.3)


def keep(current: float, proposed: float) -> bool:
    return proposed < current


def search_knobs(
    start: dict[str, float],
    evaluate: Callable[[dict[str, float]], float],
) -> tuple[dict[str, float], float]:
    knobs = dict(start)
    score = evaluate(dict(knobs))
    for name in tuple(knobs):
        base = knobs[name]
        best = dict(knobs)
        best_score = score
        for factor in FACTORS:
            trial = dict(knobs)
            trial[name] = base * factor
            trial_score = evaluate(trial)
            if keep(best_score, trial_score):
                best = trial
                best_score = trial_score
        knobs = best
        score = best_score
    return knobs, score
