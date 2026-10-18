import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.cascade_score import axis_rms, rms_sum  # noqa: E402


def test_unshifted_rms_keeps_a_0_2_s_lag():
    t_log = [0.0, 1.0, 2.0]
    response = [0.0, 1.0, 1.0]
    airborne = [True, True, True]
    command = [0.0, 1.0, 1.0]
    late = axis_rms(t_log, command, response, airborne, [0.2, 1.2, 2.2], [0.0, 1.0, 1.0])
    aligned = axis_rms(t_log, command, response, airborne, t_log, response)
    assert aligned == pytest.approx(0.0)
    assert late > aligned
    assert late == pytest.approx(math.sqrt((0.2 ** 2) / 3))


def test_axis_rms_ignores_nonfinite_response_and_sim():
    score = axis_rms(
        [0.0, 1.0, 2.0],
        [1.0, 1.0, 1.0],
        [0.0, math.nan, 2.0],
        [True, True, True],
        [0.0, 1.0, 2.0],
        [0.0, 9.0, math.nan],
    )
    assert score == pytest.approx(0.0)
    with pytest.raises(ValueError, match="no airborne samples with a finite command"):
        axis_rms([0.0], [1.0], [math.nan], [True], [0.0], [4.0])


def test_nonfinite_command_drops_that_axis_only():
    z = axis_rms([0.0, 1.0], [1.0, 1.0], [1.0, 1.0], [True, True], [0.0, 1.0], [1.0, 1.0])
    assert z == pytest.approx(0.0)
    with pytest.raises(ValueError):
        axis_rms([0.0], [math.nan], [5.0], [True], [0.0], [5.0])
    assert rms_sum([z, 0.25]) == pytest.approx(0.25)
