import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.cascade_flight import load_flight, window_flight  # noqa: E402
from calibration.cascade_model import read_plant  # noqa: E402


def test_angle_refuses_to_start_before_rate_converges(tmp_path: Path):
    from calibration.tune_angle import tune_angle_main

    code = tune_angle_main(stage_root=tmp_path / "cascade")
    assert code == 2


def test_angle_rejects_a_gain_that_breaks_rate(tmp_path: Path):
    model = tmp_path / "iris.sdf.jinja"
    model.write_text(
        (ROOT / "calibration/models/iris/iris.sdf.jinja").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    stage = tmp_path / "cascade"
    rate_best = stage / "stages" / "rate"
    rate_best.mkdir(parents=True)
    (rate_best / "best.json").write_text(
        json.dumps({"step": "rate", "trial": 1, "score": 1.0, "parameters": {}, "log": None}) + "\n",
        encoding="utf-8",
    )
    flight = window_flight(load_flight(ROOT / "calibration" / "log_2_UnknownDate.ulg"), 50.0, 80.0)
    initial_p = flight.initial_parameters["MC_ROLL_P"]
    initial_pitch = flight.initial_parameters["MC_PITCH_P"]
    before = read_plant(model)
    seen: list[str] = []

    def fly(step, flight, model_path, gains_path):
        seen.append(step)
        gains = json.loads(Path(gains_path).read_text(encoding="utf-8"))
        series = {name: ([0.0, 1.0], [0.0, 0.0]) for name in (
            "roll_rate", "pitch_rate", "roll", "pitch", "vx", "vy", "vz", "x", "y", "z")}
        if gains["MC_ROLL_P"] == pytest.approx(initial_p * 0.7):
            series["roll"] = (list(flight.roll.t_s), list(flight.roll.response))
            series["roll_rate"] = ([0.0], [1.0e6])
        return series

    from calibration.tune_angle import tune_angle

    tune_angle(
        ROOT / "calibration" / "log_2_UnknownDate.ulg",
        model,
        tmp_path / "iris_controller.json",
        fly,
        tmp_path / "cascade_report.json",
        stage,
    )
    assert set(seen) == {"angle"}
    saved = json.loads((tmp_path / "iris_controller.json").read_text(encoding="utf-8"))
    assert saved["MC_ROLL_P"] == pytest.approx(initial_p)
    assert saved["MC_PITCH_P"] == pytest.approx(initial_pitch)
    assert read_plant(model)["time_constant_up"] == pytest.approx(before["time_constant_up"])
    best = json.loads((stage / "stages" / "angle" / "best.json").read_text(encoding="utf-8"))
    assert best["step"] == "angle"


def _angle_series():
    return {name: ([0.0, 1.0], [0.0, 0.0]) for name in (
        "roll_rate", "pitch_rate", "roll", "pitch", "vx", "vy", "vz", "x", "y", "z")}


def _rate_best(stage: Path, parameters: dict) -> None:
    rate_best = stage / "stages" / "rate"
    rate_best.mkdir(parents=True)
    (rate_best / "best.json").write_text(
        json.dumps({
            "step": "rate",
            "trial": 1,
            "score": 1.0,
            "parameters": parameters,
            "log": None,
        }) + "\n",
        encoding="utf-8",
    )


def test_angle_keeps_rate_p_from_the_gain_file(tmp_path: Path):
    flight = window_flight(load_flight(ROOT / "calibration" / "log_2_UnknownDate.ulg"), 50.0, 80.0)
    logged_roll = flight.initial_parameters["MC_ROLLRATE_P"]
    logged_pitch = flight.initial_parameters["MC_PITCHRATE_P"]
    kept_roll = logged_roll + 1.0
    kept_pitch = logged_pitch + 2.0
    stage = tmp_path / "cascade"
    _rate_best(stage, {
        "MC_ROLLRATE_P": logged_roll + 10.0,
        "MC_PITCHRATE_P": logged_pitch + 20.0,
    })
    gains_path = tmp_path / "iris_controller.json"
    gains_path.write_text(
        json.dumps({"MC_ROLLRATE_P": kept_roll, "MC_PITCHRATE_P": kept_pitch}) + "\n",
        encoding="utf-8",
    )
    seen: list[str] = []

    def fly(step, flight, model_path, gains_path):
        seen.append(step)
        gains = json.loads(Path(gains_path).read_text(encoding="utf-8"))
        assert gains["MC_ROLLRATE_P"] == pytest.approx(kept_roll)
        assert gains["MC_PITCHRATE_P"] == pytest.approx(kept_pitch)
        return _angle_series()

    from calibration.tune_angle import tune_angle

    tune_angle(
        ROOT / "calibration" / "log_2_UnknownDate.ulg",
        tmp_path / "iris.sdf.jinja",
        gains_path,
        fly,
        tmp_path / "cascade_report.json",
        stage,
    )
    assert set(seen) == {"angle"}
    saved = json.loads(gains_path.read_text(encoding="utf-8"))
    assert saved["MC_ROLLRATE_P"] == pytest.approx(kept_roll)
    assert saved["MC_PITCHRATE_P"] == pytest.approx(kept_pitch)


def test_angle_keeps_rate_p_from_the_rate_best_file(tmp_path: Path):
    flight = window_flight(load_flight(ROOT / "calibration" / "log_2_UnknownDate.ulg"), 50.0, 80.0)
    kept_roll = flight.initial_parameters["MC_ROLLRATE_P"] + 3.0
    kept_pitch = flight.initial_parameters["MC_PITCHRATE_P"] + 4.0
    stage = tmp_path / "cascade"
    _rate_best(stage, {"MC_ROLLRATE_P": kept_roll, "MC_PITCHRATE_P": kept_pitch})
    gains_path = tmp_path / "iris_controller.json"
    seen: list[str] = []

    def fly(step, flight, model_path, gains_path):
        seen.append(step)
        gains = json.loads(Path(gains_path).read_text(encoding="utf-8"))
        assert gains["MC_ROLLRATE_P"] == pytest.approx(kept_roll)
        assert gains["MC_PITCHRATE_P"] == pytest.approx(kept_pitch)
        return _angle_series()

    from calibration.tune_angle import tune_angle

    tune_angle(
        ROOT / "calibration" / "log_2_UnknownDate.ulg",
        tmp_path / "iris.sdf.jinja",
        gains_path,
        fly,
        tmp_path / "cascade_report.json",
        stage,
    )
    assert set(seen) == {"angle"}
    saved = json.loads(gains_path.read_text(encoding="utf-8"))
    assert saved["MC_ROLLRATE_P"] == pytest.approx(kept_roll)
    assert saved["MC_PITCHRATE_P"] == pytest.approx(kept_pitch)
