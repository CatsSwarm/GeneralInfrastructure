import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.cascade_flight import load_flight, roll_pitch_rad  # noqa: E402

def test_roll_pitch_identity_and_quarter_roll():
    assert roll_pitch_rad((1.0, 0.0, 0.0, 0.0)) == pytest.approx((0.0, 0.0))
    half = math.sqrt(2) / 2
    assert roll_pitch_rad((half, half, 0.0, 0.0)) == pytest.approx((math.pi / 2, 0.0))

def test_log2_flight_has_rate_and_partial_position():
    flight = load_flight(ROOT / "calibration" / "log_2_UnknownDate.ulg")
    assert flight.hover_command == pytest.approx(0.344, abs=0.01)
    assert len(flight.roll_rate.response) > 1000
    assert flight.initial_parameters["MC_ROLL_P"] == pytest.approx(12.0)
    assert math.isnan(flight.x.command[0]) or any(math.isnan(v) for v in flight.x.command)
    assert any(math.isfinite(v) for v in flight.z.command)
