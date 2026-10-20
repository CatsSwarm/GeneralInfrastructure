import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.cascade_flight import (  # noqa: E402
    WINDOW_END_S,
    WINDOW_START_S,
    load_flight,
    roll_pitch_rad,
    window_flight,
)
from calibration.cascade_hover import hover_command  # noqa: E402
from calibration.cascade_sitl import setpoints  # noqa: E402

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


def test_calibration_window_is_50_to_80_seconds():
    ulg = ROOT / "calibration" / "log_2_UnknownDate.ulg"
    full = load_flight(ulg)
    flight = window_flight(full, WINDOW_START_S, WINDOW_END_S)
    assert (WINDOW_START_S, WINDOW_END_S) == (50.0, 80.0)
    for name in (
        "roll_rate", "pitch_rate", "yaw_rate", "roll", "pitch", "yaw",
        "vx", "vy", "vz", "x", "y", "z", "thrust_z",
    ):
        axis = getattr(flight, name)
        assert axis.t_s
        assert axis.t_s[0] >= WINDOW_START_S
        assert axis.t_s[-1] <= WINDOW_END_S
        assert len(axis.t_s) < len(getattr(full, name).t_s)
    records = setpoints("rate", flight)
    assert records[0].t_s >= WINDOW_START_S
    assert records[-1].t_s <= WINDOW_END_S
    assert records[-1].t_s - records[0].t_s == pytest.approx(30.0, abs=0.5)
    assert flight.hover_command == pytest.approx(
        hover_command(ulg, t_start_s=WINDOW_START_S, t_end_s=WINDOW_END_S)
    )
    assert flight.initial_parameters["MC_ROLL_P"] == pytest.approx(12.0)
