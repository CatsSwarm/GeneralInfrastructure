"""One-knob sweep that keeps a candidate only when its score strictly falls."""

from __future__ import annotations

from collections.abc import Callable

FACTORS: tuple[float, float] = (1.1, 0.9)


def keep(current: float, proposed: float) -> bool:
    return proposed < current


def search_knobs(
    start: dict[str, float],
    evaluate: Callable[[dict[str, float]], float],
) -> tuple[dict[str, float], float]:
    knobs = dict(start)
    score = evaluate(dict(knobs))
    names = tuple(knobs)
    while True:
        kept_any = False
        for name in names:
            base = knobs[name]
            best = knobs
            best_score = score
            kept_knob = False
            for factor in FACTORS:
                trial = dict(knobs)
                trial[name] = base * factor
                trial_score = evaluate(trial)
                if keep(best_score, trial_score):
                    best = trial
                    best_score = trial_score
                    kept_knob = True
            if kept_knob:
                knobs = best
                score = best_score
                kept_any = True
        if not kept_any:
            return knobs, score
