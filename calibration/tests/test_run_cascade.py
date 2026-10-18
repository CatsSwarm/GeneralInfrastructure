import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.run_cascade import run_cascade  # noqa: E402


def test_outer_step_does_not_move_motor_constant_or_arms(tmp_path: Path):
    model = tmp_path / "iris.sdf.jinja"
    model.write_text((ROOT / "calibration/models/iris/iris.sdf.jinja").read_text(encoding="utf-8"), encoding="utf-8")
    seen: list[str] = []
    first_motors: list[str] = []

    def fly(step, flight, model_path, gains_path):
        seen.append(step)
        if step == "rate" and len(seen) == 1:
            first_motors.append(model_path.read_text(encoding="utf-8"))
        return {name: ([0.0], [0.0]) for name in (
            "roll_rate", "pitch_rate", "roll", "pitch", "vx", "vy", "vz", "x", "y", "z")}

    report = run_cascade(
        ROOT / "calibration" / "log_2_UnknownDate.ulg",
        model,
        tmp_path / "iris_controller.json",
        fly,
        tmp_path / "cascade_report.json",
    )
    assert seen[0] == "rate"
    assert seen.index("angle") > seen.index("rate")
    assert seen.index("velocity") > seen.index("angle")
    assert seen.index("position") > seen.index("velocity")
    text = model.read_text(encoding="utf-8")
    assert "<mass>1.5</mass>" in text
    assert first_motors[0].count("<motorConstant>") == text.count("<motorConstant>")
    assert first_motors[0][first_motors[0].find("<motorConstant>"):first_motors[0].find("</motorConstant>")] in text
    saved = json.loads((tmp_path / "cascade_report.json").read_text(encoding="utf-8"))
    assert set(saved) == {"rate", "angle", "velocity", "position"}
    assert report["position"] == saved["position"]
