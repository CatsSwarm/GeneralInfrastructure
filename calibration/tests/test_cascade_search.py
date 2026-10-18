import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.cascade_search import keep, search_knobs  # noqa: E402


def test_keep_requires_a_strict_drop():
    assert keep(1.0, 0.9) is True
    assert keep(1.0, 1.0) is False
    assert keep(1.0, 1.1) is False


def test_search_moves_only_the_knob_that_gets_closer():
    def evaluate(knobs: dict[str, float]) -> float:
        return abs(knobs["arm_x"] - 0.13 * 0.9) + abs(knobs["arm_y"] - 0.21)

    best, score = search_knobs({"arm_x": 0.13, "arm_y": 0.21}, evaluate)
    assert best["arm_y"] == pytest.approx(0.21)
    assert best["arm_x"] == pytest.approx(0.13 * 0.9)
    assert score == pytest.approx(0.0)
