from pathlib import Path
import math
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.iris_dynamics import (  # noqa: E402
    IRIS_MODEL_JINJA,
    STOCK,
    apply_to_sdf,
    load_dynamics,
    publish_dynamics,
    save_dynamics,
)

SDF = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='iris'>
    <link name='base_link'>
      <inertial>
        <mass>1.5</mass>
        <inertia><ixx>0.029125</ixx><iyy>0.029125</iyy><izz>0.055225</izz></inertia>
      </inertial>
    </link>
    <link name='rotor_0'><inertial><mass>0.005</mass></inertial></link>
    <plugin name='front_right_motor_model' filename='libgazebo_motor_model.so'>
      <motorConstant>5.84e-06</motorConstant>
      <momentConstant>0.06</momentConstant>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <rotorDragCoefficient>0.000175</rotorDragCoefficient>
    </plugin>
  </model>
</sdf>
"""

def test_stock_json_roundtrip_and_rotor_mass_untouched(tmp_path: Path):
    path = tmp_path / "dyn.json"
    save_dynamics(path, STOCK)
    assert load_dynamics(path).mass_kg == pytest.approx(1.5)
    # The checked-in file is the published set. A search may replace STOCK.
    tuned = load_dynamics(ROOT / "calibration" / "iris_dynamics.json")
    assert all(math.isfinite(getattr(tuned, name)) for name in tuned.__dataclass_fields__)
    sdf = tmp_path / "iris.sdf"
    sdf.write_text(SDF, encoding="utf-8")
    apply_to_sdf(sdf, STOCK.__class__(0.8, 0.01, 0.02, 0.03, 1e-5, 0.05, 0.02, 0.03, 0.001))
    text = sdf.read_text(encoding="utf-8")
    assert "<mass>0.8</mass>" in text
    assert "<mass>0.005</mass>" in text
    assert _tag(text, "motorConstant") == pytest.approx(1e-5)
    assert _tag(text, "rotorDragCoefficient") == pytest.approx(0.001)
    apply_to_sdf(sdf, STOCK)
    restored = sdf.read_text(encoding="utf-8")
    assert "<mass>1.5</mass>" in restored
    assert "<mass>0.005</mass>" in restored

def test_host_jinja_is_the_mounted_model():
    text = (ROOT / IRIS_MODEL_JINJA).read_text(encoding="utf-8")
    assert "{{ mavlink_tcp_port }}" in text
    assert "link name='base_link'" in text or 'link name="base_link"' in text
    assert "<mass>1.5</mass>" in text
    docker = (ROOT / "Dockerfiles" / "PX4NoeticSimNvidia.dockerfile").read_text(encoding="utf-8")
    assert "iris.sdf.jinja" not in docker

def test_publish_writes_json_and_model(tmp_path: Path):
    model = tmp_path / "iris.sdf.jinja"
    model.write_text(SDF, encoding="utf-8")
    js = tmp_path / "dyn.json"
    tuned = STOCK.__class__(0.8, 0.01, 0.02, 0.03, 1e-5, 0.05, 0.02, 0.03, 0.001)
    publish_dynamics(tuned, js, model)
    assert load_dynamics(js).mass_kg == pytest.approx(0.8)
    assert "<mass>0.8</mass>" in model.read_text(encoding="utf-8")

def test_missing_path_exits_nonzero_with_one_stderr_line():
    script = ROOT / "calibration" / "apply_iris_dynamics.py"
    proc = subprocess.run(
        [
            sys.executable,
            str(script),
            "/no/such/iris.sdf.jinja",
            "/no/such/iris_dynamics.json",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode != 0
    lines = [line for line in proc.stderr.splitlines() if line.strip()]
    assert len(lines) == 1


def _tag(text: str, name: str) -> float:
    start = text.index(f"<{name}>") + len(name) + 2
    end = text.index(f"</{name}>")
    return float(text[start:end])
