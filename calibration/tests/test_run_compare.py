from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.iris_dynamics import STOCK  # noqa: E402
from calibration.run_compare import _report, iris_vs_simcon_rms  # noqa: E402
from calibration.ulg_velocity import VelocitySample  # noqa: E402

def test_compare_rms_is_unshifted():
    iris = [VelocitySample(0.0, 0.0, 0.0, 0.0), VelocitySample(1.0, 1.0, 0.0, 0.0)]
    simcon = [VelocitySample(0.0, 0.0, 0.0, 0.0), VelocitySample(1.0, 1.0, 0.0, 0.0)]
    assert iris_vs_simcon_rms(iris, simcon)["vx"] == pytest.approx(0.0)
    late = [VelocitySample(0.0, 0.0, 0.0, 0.0), VelocitySample(1.0, 0.0, 0.0, 0.0)]
    assert iris_vs_simcon_rms(iris, late)["vx"] > 0.1
    with pytest.raises(ValueError):
        iris_vs_simcon_rms([], simcon)


def test_null_log_iris_rms_forces_accepted_false():
    good = {"vx": 0.1, "vy": 0.05, "vz": 0.04}
    log_rows = [
        {
            "name": "cat2",
            "iris_rms": {"vx": None, "vy": 0.05, "vz": 0.04},
            "simcon_rms": good,
        }
    ]
    synthetic_rows = [{"name": "step_vx_p1", "iris_vs_simcon_rms": good}]
    report = _report(STOCK, log_rows, synthetic_rows)
    assert report["accepted"] is False


def test_null_synthetic_rms_keeps_in_bar_log_accept():
    good = {"vx": 0.1, "vy": 0.05, "vz": 0.04}
    log_rows = [{"name": "cat2", "iris_rms": good, "simcon_rms": good}]
    synthetic_rows = [
        {"name": "step_vx_p1", "iris_vs_simcon_rms": {"vx": None, "vy": None, "vz": None}}
    ]
    report = _report(STOCK, log_rows, synthetic_rows)
    assert report["accepted"] is True
    assert report["synthetic"][0]["iris_vs_simcon_rms"]["vx"] is None
