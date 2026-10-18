import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.cascade_gains import (  # noqa: E402
    ANGLE_KEYS,
    gain_file,
    save_gains,
    search_keys,
)


def test_search_keys_and_hover_restore_and_yaw_refusal():
    assert search_keys("rate") == ()
    assert search_keys("angle") == ANGLE_KEYS
    initial = {
        "MC_ROLLRATE_P": 0.08,
        "MC_ROLLRATE_FF": 0.0,
        "MC_ROLLRATE_K": 0.55,
        "MC_ROLL_P": 12.0,
        "MC_YAW_P": 2.0,
        "MPC_THR_HOVER": 0.6,
        "EKF2_GPS_CTRL": 7.0,
    }
    gained = gain_file(initial, {"MC_ROLL_P": 10.0})
    assert gained["MC_ROLL_P"] == pytest.approx(10.0)
    assert gained["MC_ROLLRATE_FF"] == pytest.approx(0.0)
    assert gained["MC_ROLLRATE_K"] == pytest.approx(0.55)
    assert gained["MPC_THR_HOVER"] == pytest.approx(0.5)
    assert "EKF2_GPS_CTRL" not in gained
    with pytest.raises(ValueError):
        gain_file(initial, {"MC_YAW_P": 4.0})
    with pytest.raises(ValueError):
        gain_file(initial, {"MPC_THR_HOVER": 0.4})


def test_save_gains_roundtrip(tmp_path: Path):
    path = tmp_path / "iris_controller.json"
    save_gains(path, {"MC_ROLL_P": 12.0, "MPC_THR_HOVER": 0.5})
    assert json.loads(path.read_text(encoding="utf-8"))["MC_ROLL_P"] == pytest.approx(12.0)
