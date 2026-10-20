import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.run_cascade import _DEFAULT_MODEL, _DEFAULT_ULG, _repo_path, main, run_cascade  # noqa: E402


def test_default_paths_resolve_when_cwd_is_calibration(monkeypatch):
    monkeypatch.chdir(ROOT / "calibration")
    assert _repo_path(_DEFAULT_ULG).is_file()
    assert _repo_path(_DEFAULT_MODEL).is_file()


def test_file_launch_imports_when_cwd_is_not_the_repo(tmp_path: Path):
    env = os.environ.copy()
    env.pop("PYTHONPATH", None)
    proc = subprocess.run(
        [sys.executable, str(ROOT / "calibration" / "run_cascade.py"), "a", "b"],
        cwd=tmp_path,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 2
    assert "usage:" in proc.stderr
    assert "No module named 'calibration'" not in proc.stderr


def test_run_cascade_prints_the_four_programs(capsys):
    code = main(["log.ulg"])
    assert code == 2
    err = capsys.readouterr().err
    assert "python3 -m calibration.tune_rate" in err
    assert "tune_angle" in err
    assert "tune_velocity" in err
    assert "tune_position" in err


def test_run_cascade_does_not_call_fly(tmp_path: Path, capsys):
    calls = []

    def fly(*args, **kwargs):
        calls.append(args)
        return {
            name: ([0.0, 1.0], [0.0, 0.0])
            for name in (
                "roll_rate",
                "pitch_rate",
                "roll",
                "pitch",
                "vx",
                "vy",
                "vz",
                "x",
                "y",
                "z",
            )
        }

    model = tmp_path / "iris.sdf.jinja"
    model.write_text(
        (ROOT / "calibration/models/iris/iris.sdf.jinja").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    before = model.read_text(encoding="utf-8")
    code = run_cascade(
        ROOT / "calibration" / "log_2_UnknownDate.ulg",
        model,
        tmp_path / "iris_controller.json",
        fly,
        tmp_path / "cascade_report.json",
        tmp_path / "sessions",
    )
    assert calls == []
    assert model.read_text(encoding="utf-8") == before
    assert not (tmp_path / "cascade_report.json").exists()
    assert code == 2
    err = capsys.readouterr().err
    assert "python3 -m calibration.tune_rate" in err
    assert "tune_angle" in err
    assert "tune_velocity" in err
    assert "tune_position" in err
