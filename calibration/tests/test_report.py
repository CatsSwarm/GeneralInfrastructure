from pathlib import Path
import json
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.iris_dynamics import STOCK  # noqa: E402
from calibration.report import build_report, write_report  # noqa: E402

def test_accepted_ignores_simcon(tmp_path: Path):
    good = {"vx": 0.1, "vy": 0.05, "vz": 0.04}
    report = build_report(
        STOCK,
        [{"name": "cat2", "iris_rms": good, "simcon_rms": {"vx": 9.0, "vy": 9.0, "vz": 9.0}}],
        [{"name": "step_vx_p1", "iris_vs_simcon_rms": good}],
    )
    assert report["accepted"] is True
    assert report["dynamics"]["mass_kg"] == 1.5
    bad = dict(good)
    bad["vx"] = 0.21
    report = build_report(STOCK, [{"name": "cat2", "iris_rms": bad, "simcon_rms": good}], [])
    assert report["accepted"] is False
    path = tmp_path / "velocity_report.json"
    write_report(path, report)
    assert json.loads(path.read_text())["accepted"] is False
