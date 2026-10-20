import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.cascade_model import read_arms, read_plant, write_model  # noqa: E402

SDF = """
<sdf>
  <link name='base_link'><inertial><mass>1.5</mass><inertia><ixx>0.029125</ixx></inertia></inertial></link>
  <link name='rotor_0'><pose>0.13 -0.22 0.023 0 0 0</pose></link>
  <link name='rotor_1'><pose>-0.13 0.20 0.023 0 0 0</pose></link>
  <link name='rotor_2'><pose>0.13 0.22 0.023 0 0 0</pose></link>
  <link name='rotor_3'><pose>-0.13 -0.20 0.023 0 0 0</pose></link>
  <plugin filename='libgazebo_motor_model.so'><motorConstant>2.92e-06</motorConstant><momentConstant>0.06</momentConstant><maxRotVelocity>1100</maxRotVelocity></plugin>
  <plugin filename='libgazebo_motor_model.so'><motorConstant>2.92e-06</motorConstant><momentConstant>0.06</momentConstant><maxRotVelocity>1100</maxRotVelocity></plugin>
</sdf>
"""

def test_write_model_sets_thrust_and_shared_arms(tmp_path: Path):
    path = tmp_path / "iris.sdf.jinja"
    path.write_text(SDF, encoding="utf-8")
    assert read_arms(path) == pytest.approx((0.13, 0.21))
    write_model(path, 1.9e-05, 0.2, 0.3)
    text = path.read_text(encoding="utf-8")
    assert "<mass>1.5</mass>" in text
    assert "<momentConstant>0.06</momentConstant>" in text
    assert "<maxRotVelocity>1100</maxRotVelocity>" in text
    assert text.count("<motorConstant>1.9e-05</motorConstant>") == 2
    assert "<pose>0.2 -0.3 0.023 0 0 0</pose>" in text
    assert "<pose>-0.2 0.3 0.023 0 0 0</pose>" in text
    assert "<pose>0.2 0.3 0.023 0 0 0</pose>" in text
    assert "<pose>-0.2 -0.3 0.023 0 0 0</pose>" in text


def test_write_model_sets_time_constant_prop_and_max_spin(tmp_path: Path):
    rotor = (
        "<pose>0.13 -0.19 0.023 0 0 0</pose>"
        "<collision><geometry><cylinder><length>0.005</length>"
        "<radius>0.128</radius></cylinder></geometry></collision>"
    )
    links = "".join(
        f"<link name='rotor_{index}'>{rotor}</link>" for index in range(4)
    )
    plugins = "".join(
        "<plugin filename='libgazebo_motor_model.so'>"
        "<timeConstantUp>0.0125</timeConstantUp>"
        "<timeConstantDown>0.025</timeConstantDown>"
        "<maxRotVelocity>1100</maxRotVelocity>"
        "<motorConstant>1e-5</motorConstant></plugin>"
        for _ in range(4)
    )
    path = tmp_path / "iris.sdf.jinja"
    path.write_text(f"<sdf>{links}{plugins}</sdf>", encoding="utf-8")
    assert read_plant(path)["prop_radius"] == pytest.approx(0.128)
    assert read_plant(path)["time_constant_up"] == pytest.approx(0.0125)
    write_model(
        path,
        2e-5,
        0.13,
        0.19,
        time_constant_up=0.02,
        time_constant_down=0.04,
        prop_radius=0.1,
        max_rot_velocity=900,
    )
    text = path.read_text(encoding="utf-8")
    assert text.count("<timeConstantUp>0.02</timeConstantUp>") == 4
    assert text.count("<timeConstantDown>0.04</timeConstantDown>") == 4
    assert text.count("<maxRotVelocity>900</maxRotVelocity>") == 4
    assert text.count("<radius>0.1</radius>") == 4
    assert read_plant(path)["max_rot_velocity"] == pytest.approx(900)
