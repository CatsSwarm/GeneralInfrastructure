"""Host-side iris rigid-body and motor parameters for the Gazebo model."""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, fields
from pathlib import Path

IRIS_MODEL_JINJA = Path("calibration/models/iris/iris.sdf.jinja")

_BASE_LINK_RE = re.compile(
    r"<link\b[^>]*\bname\s*=\s*(['\"])base_link\1[^>]*>.*?</link>",
    re.DOTALL,
)
_MOTOR_PLUGIN_RE = re.compile(
    r"<plugin\b[^>]*\bfilename\s*=\s*(['\"])libgazebo_motor_model\.so\1[^>]*>.*?</plugin>",
    re.DOTALL,
)
_LINK_TAGS = {
    "mass": "mass_kg",
    "ixx": "ixx",
    "iyy": "iyy",
    "izz": "izz",
}
_PLUGIN_TAGS = {
    "motorConstant": "motor_constant",
    "momentConstant": "moment_constant",
    "timeConstantUp": "time_constant_up",
    "timeConstantDown": "time_constant_down",
    "rotorDragCoefficient": "rotor_drag_coefficient",
}


@dataclass(frozen=True)
class IrisDynamics:
    mass_kg: float
    ixx: float
    iyy: float
    izz: float
    motor_constant: float
    moment_constant: float
    time_constant_up: float
    time_constant_down: float
    rotor_drag_coefficient: float


STOCK = IrisDynamics(
    1.5,
    0.029125,
    0.029125,
    0.055225,
    5.84e-06,
    0.06,
    0.0125,
    0.025,
    0.000175,
)


def load_dynamics(path: Path) -> IrisDynamics:
    raw = json.loads(path.read_text(encoding="utf-8"))
    values = {item.name: float(raw[item.name]) for item in fields(IrisDynamics)}
    return IrisDynamics(**values)


def save_dynamics(path: Path, dynamics: IrisDynamics) -> None:
    payload = {item.name: getattr(dynamics, item.name) for item in fields(IrisDynamics)}
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def apply_to_sdf(sdf_path: Path, dynamics: IrisDynamics) -> None:
    text = sdf_path.read_text(encoding="utf-8")
    if _BASE_LINK_RE.search(text) is None:
        raise ValueError(f"base_link not found in {sdf_path}")
    text = _BASE_LINK_RE.sub(
        lambda match: _fill_tags(match.group(0), _LINK_TAGS, dynamics),
        text,
        count=1,
    )
    if _MOTOR_PLUGIN_RE.search(text) is None:
        raise ValueError(f"libgazebo_motor_model.so plugin not found in {sdf_path}")
    text = _MOTOR_PLUGIN_RE.sub(
        lambda match: _fill_tags(match.group(0), _PLUGIN_TAGS, dynamics),
        text,
    )
    sdf_path.write_text(text, encoding="utf-8")


def publish_dynamics(dynamics: IrisDynamics, json_path: Path, model_path: Path) -> None:
    save_dynamics(json_path, dynamics)
    apply_to_sdf(model_path, dynamics)


def _fill_tags(block: str, tags: dict[str, str], dynamics: IrisDynamics) -> str:
    for tag, field_name in tags.items():
        pattern = re.compile(rf"<{tag}>[^<]*</{tag}>")
        value = _format_number(getattr(dynamics, field_name))
        block, count = pattern.subn(f"<{tag}>{value}</{tag}>", block)
        if count == 0:
            raise ValueError(f"<{tag}> not found")
    return block


def _format_number(value: float) -> str:
    return format(value, ".16g")
