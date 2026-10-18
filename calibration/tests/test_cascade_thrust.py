import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.cascade_thrust import motor_constant, spin_rad_s  # noqa: E402


def test_hover_spin_is_affine_and_clamped():
    assert spin_rad_s(0.34) == pytest.approx(440.0)
    assert spin_rad_s(1.0) == pytest.approx(1100.0)
    assert spin_rad_s(2.0) == pytest.approx(1100.0)


def test_motor_constant_holds_weight_at_command_h():
    # spin 440, not 0.34 * 1100, and not thrust at spin 1100 = weight / 0.34
    assert motor_constant(0.34) == pytest.approx((1.5 * 9.81) / (4 * 440.0**2))
    assert motor_constant(0.34) != pytest.approx((1.5 * 9.81) / (0.34 * 4 * 1100.0**2))
