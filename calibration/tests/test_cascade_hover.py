from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.cascade_hover import hover_command, hover_mean  # noqa: E402

def test_hover_mean_keeps_flat_airborne_samples_only():
    landed = [False, False, False, True]
    vz = [0.0, 0.1, 0.05, 0.0]
    motors = [(0.2, 0.2, 0.2, 0.2), (0.9, 0.9, 0.9, 0.9), (0.4, 0.4, 0.4, 0.4), (0.1, 0.1, 0.1, 0.1)]
    assert hover_mean(landed, vz, motors) == pytest.approx(0.3)


def test_hover_mean_excludes_nonfinite_vz():
    landed = [False, False, False]
    vz = [float("nan"), 0.05, 0.2]
    motors = [
        (0.8, 0.8, 0.8, 0.8),
        (0.25, 0.25, 0.25, 0.25),
        (0.1, 0.1, 0.1, 0.1),
    ]
    assert hover_mean(landed, vz, motors) == pytest.approx(0.25)

def test_log2_hover_command_is_about_0_34():
    h = hover_command(ROOT / "calibration" / "log_2_UnknownDate.ulg")
    assert h == pytest.approx(0.344, abs=0.01)
