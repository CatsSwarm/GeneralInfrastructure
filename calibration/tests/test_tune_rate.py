import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.cascade_flight import load_flight  # noqa: E402
from calibration.cascade_model import read_arms, read_plant  # noqa: E402


def test_load_converged_requires_the_matching_step(tmp_path: Path):
    from calibration.cascade_stage import PREVIOUS, converged_path, load_converged

    assert PREVIOUS["rate"] is None
    assert PREVIOUS["angle"] == "rate"
    path = converged_path(tmp_path, "rate")
    assert path == tmp_path / "stages" / "rate" / "best.json"
    with pytest.raises(FileNotFoundError):
        load_converged(tmp_path, "rate")
    path.parent.mkdir(parents=True)
    path.write_text(json.dumps({"step": "angle"}) + "\n", encoding="utf-8")
    with pytest.raises(ValueError):
        load_converged(tmp_path, "rate")


def test_rate_step_freezes_geometry_and_restores_the_best_lag(tmp_path: Path):
    model = tmp_path / "iris.sdf.jinja"
    source = (ROOT / "calibration/models/iris/iris.sdf.jinja").read_text(encoding="utf-8")
    model.write_text(source, encoding="utf-8")
    before_arms = read_arms(model)
    before = read_plant(model)
    logged_p = load_flight(ROOT / "calibration" / "log_2_UnknownDate.ulg").initial_parameters["MC_ROLLRATE_P"]
    seen: list[tuple[str, float, float]] = []

    def fly(step, flight, model_path, gains_path):
        plant = read_plant(model_path)
        gains = json.loads(Path(gains_path).read_text(encoding="utf-8"))
        seen.append((step, plant["time_constant_up"], gains["MC_ROLLRATE_P"]))
        level = 1.0
        if plant["time_constant_up"] == pytest.approx(before["time_constant_up"] * 0.85):
            level = 0.0
        if gains["MC_ROLLRATE_P"] != pytest.approx(logged_p):
            level = 10.0
        return {name: ([0.0], [level]) for name in (
            "roll_rate", "pitch_rate", "roll", "pitch", "vx", "vy", "vz", "x", "y", "z")}

    from calibration.tune_rate import tune_rate

    tune_rate(
        ROOT / "calibration" / "log_2_UnknownDate.ulg",
        model,
        tmp_path / "iris_controller.json",
        fly,
        tmp_path / "cascade_report.json",
        tmp_path / "cascade",
    )
    assert read_arms(model)[0] == pytest.approx(before_arms[0])
    assert read_arms(model)[1] == pytest.approx(before_arms[1])
    after = read_plant(model)
    assert after["prop_radius"] == pytest.approx(before["prop_radius"])
    assert after["max_rot_velocity"] == pytest.approx(before["max_rot_velocity"])
    assert after["time_constant_up"] == pytest.approx(before["time_constant_up"] * 0.85)
    assert after["time_constant_down"] == pytest.approx(
        after["time_constant_up"] * before["time_constant_down"] / before["time_constant_up"]
    )
    saved_gains = json.loads((tmp_path / "iris_controller.json").read_text(encoding="utf-8"))
    assert saved_gains["MC_ROLLRATE_P"] == pytest.approx(logged_p)
    assert saved_gains["MC_ROLLRATE_I"] == pytest.approx(
        load_flight(ROOT / "calibration" / "log_2_UnknownDate.ulg").initial_parameters["MC_ROLLRATE_I"]
    )
    rate_lags = [lag for step, lag, _p in seen if step == "rate"]
    assert any(lag == pytest.approx(before["time_constant_up"] * 0.7) for lag in rate_lags)
    assert {step for step, _lag, _p in seen} == {"rate"}
    best = json.loads((tmp_path / "cascade" / "stages" / "rate" / "best.json").read_text(encoding="utf-8"))
    assert best["step"] == "rate"
    assert best["parameters"]["time_constant_up"] == pytest.approx(before["time_constant_up"] * 0.85)
    assert best["parameters"]["MC_ROLLRATE_P"] == pytest.approx(logged_p)
