"""Write one Iris motorConstant and shared rotor arm lengths into an SDF jinja."""

import re
from pathlib import Path

ROTOR_SIGNS: tuple[tuple[str, int, int], ...] = (
    ("rotor_0", 1, -1),
    ("rotor_1", -1, 1),
    ("rotor_2", 1, 1),
    ("rotor_3", -1, -1),
)
ROTOR_Z_M: float = 0.023

_LINK_RE = re.compile(
    r"<link\b[^>]*\bname='(rotor_[0-3])'[^>]*>(.*?)</link>",
    re.DOTALL,
)
_POSE_RE = re.compile(r"<pose>\s*([^<]*?)\s*</pose>")
_MOTOR_PLUGIN_RE = re.compile(
    r"<plugin\b[^>]*\bfilename='libgazebo_motor_model\.so'[^>]*>.*?</plugin>",
    re.DOTALL,
)
_MOTOR_CONSTANT_RE = re.compile(r"<motorConstant>[^<]*</motorConstant>")
_RADIUS_RE = re.compile(r"(<radius>)[^<]*(</radius>)")


def _format(value: float) -> str:
    return format(value, ".16g")


def _first_pose(link_body: str, name: str) -> tuple[float, float, float]:
    match = _POSE_RE.search(link_body)
    if match is None:
        raise ValueError(f"{name} link has no pose")
    parts = match.group(1).split()
    if len(parts) < 3:
        raise ValueError(f"{name} pose is incomplete")
    return float(parts[0]), float(parts[1]), float(parts[2])


def _rotor_links(text: str) -> dict[str, re.Match[str]]:
    found = {match.group(1): match for match in _LINK_RE.finditer(text)}
    missing = [name for name, _sign_x, _sign_y in ROTOR_SIGNS if name not in found]
    if missing:
        raise ValueError(f"missing rotor links: {', '.join(missing)}")
    return found


def _splice_body(match: re.Match[str], new_body: str) -> str:
    full = match.group(0)
    start = match.start(2) - match.start(0)
    end = match.end(2) - match.start(0)
    return full[:start] + new_body + full[end:]


def read_arms(model_path: Path) -> tuple[float, float]:
    """Return |x| of rotor_0 and the mean of the four rotor |y| values."""
    links = _rotor_links(model_path.read_text(encoding="utf-8"))
    arm_x = abs(_first_pose(links["rotor_0"].group(2), "rotor_0")[0])
    arm_y = [
        abs(_first_pose(links[name].group(2), name)[1])
        for name, _sign_x, _sign_y in ROTOR_SIGNS
    ]
    return arm_x, sum(arm_y) / len(arm_y)


def _tag_value(text: str, tag: str) -> float:
    match = re.search(rf"<{tag}>([^<]*)</{tag}>", text)
    if match is None:
        raise ValueError(f"missing {tag}")
    return float(match.group(1))


def _replace_tag(text: str, tag: str, value: float) -> str:
    pattern = re.compile(rf"(<{tag}>)[^<]*(</{tag}>)")
    replaced, count = pattern.subn(rf"\g<1>{_format(value)}\g<2>", text)
    if count == 0:
        raise ValueError(f"missing {tag}")
    return replaced


def read_plant(model_path: Path) -> dict[str, float]:
    """Motor time constants, prop radius, and maximum rotor speed from the iris jinja."""
    text = model_path.read_text(encoding="utf-8")
    radius = re.search(
        r"<cylinder>\s*<length>[^<]*</length>\s*<radius>([^<]*)</radius>",
        text,
    )
    if radius is None:
        raise ValueError("missing propeller radius")
    return {
        "time_constant_up": _tag_value(text, "timeConstantUp"),
        "time_constant_down": _tag_value(text, "timeConstantDown"),
        "prop_radius": float(radius.group(1)),
        "max_rot_velocity": _tag_value(text, "maxRotVelocity"),
    }


def write_model(
    model_path: Path,
    motor_constant: float,
    arm_x: float,
    arm_y: float,
    *,
    time_constant_up: float | None = None,
    time_constant_down: float | None = None,
    prop_radius: float | None = None,
    max_rot_velocity: float | None = None,
) -> None:
    """Set motorConstant, rotor poses, and any motor plant fields that are given."""
    text = model_path.read_text(encoding="utf-8")
    signs = {name: (sign_x, sign_y) for name, sign_x, sign_y in ROTOR_SIGNS}
    _rotor_links(text)

    def replace_link(match: re.Match[str]) -> str:
        name = match.group(1)
        sign_x, sign_y = signs[name]
        pose = (
            f"<pose>{_format(sign_x * arm_x)} {_format(sign_y * arm_y)} "
            f"{_format(ROTOR_Z_M)} 0 0 0</pose>"
        )
        body, count = _POSE_RE.subn(pose, match.group(2), count=1)
        if count != 1:
            raise ValueError(f"{name} link has no pose")
        return _splice_body(match, body)

    def replace_radius(match: re.Match[str]) -> str:
        if prop_radius is None:
            return match.group(0)
        body, count = _RADIUS_RE.subn(
            rf"\g<1>{_format(prop_radius)}\g<2>", match.group(2)
        )
        if count == 0:
            raise ValueError(f"{match.group(1)} link has no propeller radius")
        return _splice_body(match, body)

    text = _LINK_RE.sub(replace_link, text)
    if prop_radius is not None:
        text = _LINK_RE.sub(replace_radius, text)
    if time_constant_up is not None:
        text = _replace_tag(text, "timeConstantUp", time_constant_up)
    if time_constant_down is not None:
        text = _replace_tag(text, "timeConstantDown", time_constant_down)
    if max_rot_velocity is not None:
        text = _replace_tag(text, "maxRotVelocity", max_rot_velocity)
    constant = f"<motorConstant>{_format(motor_constant)}</motorConstant>"
    text = _MOTOR_PLUGIN_RE.sub(
        lambda match: _MOTOR_CONSTANT_RE.sub(constant, match.group(0)),
        text,
    )
    model_path.write_text(text, encoding="utf-8")
