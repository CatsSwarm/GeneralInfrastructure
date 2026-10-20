import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.cascade_flight import AxisLog  # noqa: E402
from calibration.cascade_session import _PLOT_LABELS, open_session  # noqa: E402
from calibration.cascade_tune import fly_and_record, stage_session  # noqa: E402


def _flight():
    axis = AxisLog((0.0, 1.0, 2.0), (0.2, 0.2, 0.4), (0.0, 1.0, 0.5), (True, True, True))
    return SimpleNamespace(**{name: axis for name, _label in _PLOT_LABELS})


def _series(sim):
    return {name: ([0.0, 1.0, 2.0], sim) for name, _label in _PLOT_LABELS}


def test_session_logs_ulg_parameters_and_comparison_plot(tmp_path: Path):
    ulg = Path("calibration/log_2_UnknownDate.ulg")
    session = open_session(tmp_path, ulg)
    flight = _flight()
    session.record(
        "rate",
        {"MC_ROLL_P": 12.0, "arm_x": 0.13, "motorConstant": 1.8e-5},
        1.5,
        flight,
        _series([0.2, 0.4, 0.6]),
    )
    session.record(
        "rate",
        {"MC_ROLL_P": 10.8, "arm_x": 0.117, "motorConstant": 1.8e-5},
        1.1,
        flight,
        _series([0.05, 0.9, 0.55]),
    )

    lines = session.history_path.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 2
    first, second = json.loads(lines[0]), json.loads(lines[1])
    assert first["ulg"] == str(ulg)
    assert first["step"] == "rate"
    assert first["parameters"]["MC_ROLL_P"] == 12.0
    assert first["improved"] is False
    assert second["trial"] == 2
    assert second["parameters"]["arm_x"] == 0.117
    assert second["improved"] is True
    assert second["score"] == 1.1

    state = json.loads(session.state_path.read_text(encoding="utf-8"))
    assert state["step"] == "rate"
    assert state["trial"] == 2
    assert state["best_score"]["rate"] == 1.1
    assert state["parameters"]["MC_ROLL_P"] == 10.8
    html = session.plot_path.read_text(encoding="utf-8")
    assert session.plot_path.name == "comparison.html"
    assert "plotly" in html
    assert "RMS sum" in html
    assert "target (ULG response)" in html
    assert "current (sim)" in html
    assert "legend2" in html
    assert "legend3" in html
    shown = html.replace("\\u002f", "/")
    for _name, label in _PLOT_LABELS:
        assert label in shown
    assert "plotly_click" in html
    assert "click to show this flight" in html
    assert '"2"' in html and "0.9" in html
    assert (tmp_path / "latest.txt").read_text(encoding="utf-8").strip() == str(session.directory)


def test_save_stage_writes_the_best_trial_and_refuses_an_empty_step(tmp_path: Path):
    session = open_session(tmp_path, Path("calibration/log_2_UnknownDate.ulg"))
    flight = _flight()
    session.record("rate", {"MC_ROLLRATE_P": 0.08}, 2.0, flight, _series([0.2, 0.4, 0.6]))
    session.record("rate", {"MC_ROLLRATE_P": 0.056}, 1.0, flight, _series([0.05, 0.9, 0.55]))
    session.record("rate", {"MC_ROLLRATE_P": 0.092}, 3.0, flight, _series([4.0, 4.0, 4.0]))
    with pytest.raises(ValueError):
        session.save_stage("angle")
    assert not (session.directory / "stages" / "angle").exists()
    stage = session.save_stage("rate")
    saved = json.loads((stage / "best.json").read_text(encoding="utf-8"))
    assert saved["step"] == "rate"
    assert saved["trial"] == 2
    assert saved["score"] == 1.0
    assert saved["parameters"]["MC_ROLLRATE_P"] == pytest.approx(0.056)
    html = (stage / "comparison.html").read_text(encoding="utf-8")
    assert "trial 2" in html
    assert "trial 3" not in html
    assert "0.9" in html


def test_save_stage_replaces_best_json_on_the_same_step(tmp_path: Path):
    session = open_session(tmp_path, Path("calibration/log_2_UnknownDate.ulg"))
    flight = _flight()
    session.record("rate", {"MC_ROLLRATE_P": 0.08}, 2.0, flight, _series([0.2, 0.4, 0.6]))
    session.save_stage("rate")
    session.record("rate", {"MC_ROLLRATE_P": 0.056}, 1.0, flight, _series([0.05, 0.9, 0.55]))
    stage = session.save_stage("rate")
    saved = json.loads((stage / "best.json").read_text(encoding="utf-8"))
    assert saved["trial"] == 2
    assert saved["score"] == 1.0
    assert saved["parameters"]["MC_ROLLRATE_P"] == pytest.approx(0.056)
    assert (stage / "comparison.html").is_file()


def test_two_stages_keep_their_log_and_history_trial(tmp_path: Path):
    """A later stage must not replace an earlier stage's winning log or trial row."""
    root = tmp_path / "cascade"
    stale = root / "stages" / "rate" / "parameter_history.jsonl"
    stale.parent.mkdir(parents=True)
    stale.write_text(
        json.dumps({"trial": 7, "step": "rate", "score": 99.0}) + "\n",
        encoding="utf-8",
    )
    ulg = Path("calibration/log_2_UnknownDate.ulg")
    model = tmp_path / "iris"
    gains = tmp_path / "gains.json"
    seen = {"rate": 0, "angle": 0}

    def fly(step, flight, model_path, gains_path, flight_log):
        seen[step] += 1
        if step == "rate" and seen[step] == 1:
            flight_log.write_text("rate-winner", encoding="utf-8")
            return _series([0.0, 1.0, 0.5])
        if step == "rate":
            flight_log.write_text("rate-other", encoding="utf-8")
            return _series([4.0, 4.0, 4.0])
        flight_log.write_text("angle-flight", encoding="utf-8")
        return _series([0.2, 0.4, 0.6])

    flight = _flight()
    rate = stage_session(root, ulg)
    fly_and_record(rate, fly, "rate", flight, model, gains, {"MC_ROLLRATE_P": 0.08})
    fly_and_record(rate, fly, "rate", flight, model, gains, {"MC_ROLLRATE_P": 0.1})
    rate.save_stage("rate")
    angle = stage_session(root, ulg)
    fly_and_record(angle, fly, "angle", flight, model, gains, {"MC_ROLL_P": 6.5})
    angle.save_stage("angle")

    rate_best = json.loads((root / "stages" / "rate" / "best.json").read_text(encoding="utf-8"))
    angle_best = json.loads((root / "stages" / "angle" / "best.json").read_text(encoding="utf-8"))
    rate_log = Path(rate_best["log"])
    angle_log = Path(angle_best["log"])
    assert rate_log != angle_log
    assert rate_log.is_relative_to(root / "stages" / "rate")
    assert angle_log.is_relative_to(root / "stages" / "angle")
    assert rate_log.name == "trial_001.ulg"
    assert rate_log.read_text(encoding="utf-8") == "rate-winner"
    assert angle_log.read_text(encoding="utf-8") == "angle-flight"

    rate_rows = [
        json.loads(line)
        for line in (root / "stages" / "rate" / "parameter_history.jsonl").read_text(encoding="utf-8").splitlines()
        if line
    ]
    angle_rows = [
        json.loads(line)
        for line in (root / "stages" / "angle" / "parameter_history.jsonl").read_text(encoding="utf-8").splitlines()
        if line
    ]
    assert [(row["trial"], row["step"]) for row in rate_rows] == [(1, "rate"), (2, "rate")]
    assert [(row["trial"], row["step"]) for row in angle_rows] == [(1, "angle")]
    assert all(row["score"] != 99.0 for row in rate_rows)


def test_run_cascade_writes_a_session_per_trial(tmp_path: Path):
    """Per-trial logs belong to tune_rate: one history row and one stage log per flight."""
    calls = {"n": 0}

    def fly(step, flight, model_path, gains_path, flight_log=None):
        calls["n"] += 1
        if flight_log is not None:
            flight_log.write_text(step, encoding="utf-8")
        return {name: ([0.0, 1.0], [0.0, 0.0]) for name in (
            "roll_rate", "pitch_rate", "roll", "pitch", "vx", "vy", "vz", "x", "y", "z")}

    model = tmp_path / "iris.sdf.jinja"
    model.write_text((ROOT / "calibration/models/iris/iris.sdf.jinja").read_text(encoding="utf-8"), encoding="utf-8")
    from calibration.tune_rate import tune_rate

    tune_rate(
        ROOT / "calibration" / "log_2_UnknownDate.ulg",
        model,
        tmp_path / "iris_controller.json",
        fly,
        tmp_path / "cascade_report.json",
        tmp_path / "cascade",
    )
    history = tmp_path / "cascade" / "stages" / "rate" / "parameter_history.jsonl"
    lines = [line for line in history.read_text(encoding="utf-8").splitlines() if line]
    assert calls["n"] > 1
    assert len(lines) == calls["n"]
    record = json.loads(lines[0])
    assert record["trial"] == 1
    assert record["parameters"]["MC_ROLL_P"] == 12.0
    assert "motorConstant" in record["parameters"]
    assert Path(record["log"]).is_relative_to(tmp_path / "cascade" / "stages" / "rate")
    html = (tmp_path / "cascade" / "stages" / "rate" / "comparison.html").read_text(encoding="utf-8")
    assert "plotly" in html
    assert "RMS sum" in html
