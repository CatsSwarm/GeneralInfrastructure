"""Apply a saved iris dynamics JSON file to an SDF or jinja model."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from calibration.iris_dynamics import apply_to_sdf, load_dynamics  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if len(args) != 2:
        print(
            "usage: python3 calibration/apply_iris_dynamics.py JINJA JSON",
            file=sys.stderr,
        )
        return 2
    jinja = Path(args[0])
    dynamics_path = Path(args[1])
    try:
        if not jinja.is_file() or not dynamics_path.is_file():
            missing = jinja if not jinja.is_file() else dynamics_path
            print(f"apply_iris_dynamics: {missing} not found", file=sys.stderr)
            return 1
        apply_to_sdf(jinja, load_dynamics(dynamics_path))
    except FileNotFoundError as exc:
        missing = exc.filename if exc.filename else exc
        print(f"apply_iris_dynamics: {missing} not found", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
