from pathlib import Path
import math
import sys
import types

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.simcon_velocity import (  # noqa: E402
    SIMCON_ROOT,
    _HOVER_Z_NED,
    _load_cat_config,
    desired_from_velocity,
    integration_start_s,
)
from calibration.ulg_velocity import VelocitySample  # noqa: E402


def test_desired_is_pure_velocity_ned():
    desired = desired_from_velocity(1.0, -2.0, 0.25)
    assert math.isnan(desired["pos"][0]) and math.isnan(desired["pos"][1]) and math.isnan(desired["pos"][2])
    assert desired["vel"] == [1.0, -2.0, 0.25]
    assert desired["yaw_rate"] == 0.0
    assert SIMCON_ROOT.name == "Quadcopter_SimCon"
    assert not (ROOT / "TrainingRoom").exists()
    assert _HOVER_Z_NED == pytest.approx(-1.3)


def test_integration_starts_at_first_sample_time():
    samples = [
        VelocitySample(0.90, 0.0, 0.0, 1.10),
        VelocitySample(2.30, 0.0, 0.0, -0.60),
    ]
    assert integration_start_s(samples) == pytest.approx(samples[0].t_s)


def test_failed_load_drone_config_raises(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    (tmp_path / "cat_drone_config.json").write_text("{}\n", encoding="utf-8")
    module = types.ModuleType("load_drone_config")

    def load_drone_config(path):
        raise RuntimeError("config unreadable")

    module.load_drone_config = load_drone_config
    monkeypatch.setitem(sys.modules, "load_drone_config", module)
    with pytest.raises(RuntimeError, match="config unreadable"):
        _load_cat_config(str(tmp_path))
