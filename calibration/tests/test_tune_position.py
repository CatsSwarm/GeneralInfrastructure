import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.cascade_flight import load_flight, window_flight  # noqa: E402
from calibration.cascade_model import read_plant  # noqa: E402

_POSITION = ("MPC_XY_P", "MPC_Z_P")
_INNER = (
    "MC_ROLLRATE_P",
    "MC_PITCHRATE_P",
    "MC_ROLL_P",
    "MC_PITCH_P",
    "MPC_XY_VEL_P_ACC",
    "MPC_XY_VEL_I_ACC",
    "MPC_XY_VEL_D_ACC",
    "MPC_Z_VEL_P_ACC",
    "MPC_Z_VEL_I_ACC",
    "MPC_Z_VEL_D_ACC",
)
_REFUSAL = "velocity has not converged; run python3 -m calibration.tune_velocity"


def test_position_refuses_to_start_before_velocity_converges(tmp_path: Path, capsys):
    from calibration.tune_position import tune_position_main

    stage = tmp_path / "cascade"
    code = tune_position_main(stage_root=stage)
    assert code == 2
    assert capsys.readouterr().err.strip() == _REFUSAL
    assert not (stage / "parameter_history.jsonl").exists()

    velocity = stage / "stages" / "velocity"
    velocity.mkdir(parents=True)
    (velocity / "best.json").write_text(
        json.dumps({"step": "angle"}) + "\n",
        encoding="utf-8",
    )
    code = tune_position_main(stage_root=stage)
    assert code == 2
    assert capsys.readouterr().err.strip() == _REFUSAL
    assert not (stage / "stages" / "position").exists()


def test_position_rejects_a_gain_that_breaks_rate(tmp_path: Path):
    model = tmp_path / "iris.sdf.jinja"
    model.write_text(
        (ROOT / "calibration/models/iris/iris.sdf.jinja").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    stage = tmp_path / "cascade"
    _velocity_best(stage, {})
    flight = window_flight(load_flight(ROOT / "calibration" / "log_2_UnknownDate.ulg"), 50.0, 80.0)
    initial = {key: flight.initial_parameters[key] for key in _POSITION}
    before = read_plant(model)
    seen: list[str] = []
    flown_p: list[float] = []

    def fly(step, flight, model_path, gains_path):
        seen.append(step)
        gains = json.loads(Path(gains_path).read_text(encoding="utf-8"))
        flown_p.append(gains["MPC_XY_P"])
        series = _position_series()
        if gains["MPC_XY_P"] == pytest.approx(initial["MPC_XY_P"] * 0.7):
            series["x"] = (list(flight.x.t_s), list(flight.x.response))
            series["roll_rate"] = ([0.0], [1.0e6])
        return series

    from calibration.tune_position import tune_position

    tune_position(
        ROOT / "calibration" / "log_2_UnknownDate.ulg",
        model,
        tmp_path / "iris_controller.json",
        fly,
        tmp_path / "cascade_report.json",
        stage,
    )
    assert set(seen) == {"position"}
    assert any(value == pytest.approx(initial["MPC_XY_P"] * 0.7) for value in flown_p)
    saved = json.loads((tmp_path / "iris_controller.json").read_text(encoding="utf-8"))
    for key, value in initial.items():
        assert saved[key] == pytest.approx(value)
    assert read_plant(model)["time_constant_up"] == pytest.approx(before["time_constant_up"])
    best = json.loads((stage / "stages" / "position" / "best.json").read_text(encoding="utf-8"))
    assert best["step"] == "position"
    assert best["parameters"]["MPC_XY_P"] == pytest.approx(initial["MPC_XY_P"])


def test_position_keeps_inner_gains_from_the_gain_file(tmp_path: Path):
    flight = window_flight(load_flight(ROOT / "calibration" / "log_2_UnknownDate.ulg"), 50.0, 80.0)
    logged = flight.initial_parameters
    kept = {key: logged[key] + offset for offset, key in enumerate(_INNER, start=1)}
    stage = tmp_path / "cascade"
    _velocity_best(stage, {key: value + 10.0 for key, value in kept.items()})
    gains_path = tmp_path / "iris_controller.json"
    gains_path.write_text(json.dumps(kept) + "\n", encoding="utf-8")
    seen: list[str] = []

    def fly(step, flight, model_path, gains_path):
        seen.append(step)
        gains = json.loads(Path(gains_path).read_text(encoding="utf-8"))
        for key, value in kept.items():
            assert gains[key] == pytest.approx(value)
        return _position_series()

    from calibration.tune_position import tune_position

    tune_position(
        ROOT / "calibration" / "log_2_UnknownDate.ulg",
        tmp_path / "iris.sdf.jinja",
        gains_path,
        fly,
        tmp_path / "cascade_report.json",
        stage,
    )
    assert set(seen) == {"position"}
    saved = json.loads(gains_path.read_text(encoding="utf-8"))
    for key, value in kept.items():
        assert saved[key] == pytest.approx(value)


def test_position_keeps_inner_gains_from_the_velocity_best_file(tmp_path: Path):
    flight = window_flight(load_flight(ROOT / "calibration" / "log_2_UnknownDate.ulg"), 50.0, 80.0)
    logged = flight.initial_parameters
    kept = {key: logged[key] + offset for offset, key in enumerate(_INNER, start=1)}
    stage = tmp_path / "cascade"
    _velocity_best(stage, kept)
    gains_path = tmp_path / "iris_controller.json"
    seen: list[str] = []

    def fly(step, flight, model_path, gains_path):
        seen.append(step)
        gains = json.loads(Path(gains_path).read_text(encoding="utf-8"))
        for key, value in kept.items():
            assert gains[key] == pytest.approx(value)
        return _position_series()

    from calibration.tune_position import tune_position

    tune_position(
        ROOT / "calibration" / "log_2_UnknownDate.ulg",
        tmp_path / "iris.sdf.jinja",
        gains_path,
        fly,
        tmp_path / "cascade_report.json",
        stage,
    )
    assert set(seen) == {"position"}
    saved = json.loads(gains_path.read_text(encoding="utf-8"))
    for key, value in kept.items():
        assert saved[key] == pytest.approx(value)


def _position_series():
    return {name: ([0.0, 1.0], [0.0, 0.0]) for name in (
        "roll_rate", "pitch_rate", "roll", "pitch", "vx", "vy", "vz", "x", "y", "z")}


def _velocity_best(stage: Path, parameters: dict) -> None:
    velocity_best = stage / "stages" / "velocity"
    velocity_best.mkdir(parents=True)
    (velocity_best / "best.json").write_text(
        json.dumps({
            "step": "velocity",
            "trial": 1,
            "score": 1.0,
            "parameters": parameters,
            "log": None,
        }) + "\n",
        encoding="utf-8",
    )
