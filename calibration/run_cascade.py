"""``run_cascade`` prints the four tune programs. It does not fly or search."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

_TUNE_COMMANDS = (
    "python3 -m calibration.tune_rate",
    "python3 -m calibration.tune_angle",
    "python3 -m calibration.tune_velocity",
    "python3 -m calibration.tune_position",
)


def run_cascade(*_args, **_kwargs) -> int:
    """Print the four tune programs and return 2. Does not fly or search."""
    for command in _TUNE_COMMANDS:
        print(command, file=sys.stderr)
    return 2


_DEFAULT_ULG = Path("calibration/log_2_UnknownDate.ulg")
_DEFAULT_MODEL = Path("calibration/models/iris/iris.sdf.jinja")
_DEFAULT_GAINS = Path("calibration/iris_controller.json")
_DEFAULT_REPORT = Path("calibration/results/cascade_report.json")
_DEFAULT_SITL_LOG = Path("calibration/results/sitl.log")
# Set True to open the Gazebo window. False keeps the sim headless.
SHOW_GAZEBO = True


def _repo_path(path: Path) -> Path:
    """Relative cascade paths are from the repo root, not the process cwd."""
    path = Path(path)
    if path.is_absolute():
        return path
    return _ROOT / path


def popen_sim(command: list[str], log_path: Path) -> subprocess.Popen:
    """Start the sim with stdout on a file.

    A pipe on this host makes ``gzserver`` abort during Gazebo transport
    startup (empty header, then ``std::length_error``).
    """
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log = log_path.open("w", encoding="utf-8", errors="replace")
    proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
    original_wait = proc.wait

    def wait(timeout=None):
        try:
            return original_wait(timeout)
        finally:
            if proc.poll() is not None and not log.closed:
                log.close()

    proc.wait = wait  # type: ignore[method-assign]
    return proc


def _connect(port: str):
    """Pymavlink ``udpin`` for the first vehicle. ``port`` is ``udpin:0.0.0.0:14541``."""
    from pymavlink import mavutil

    return mavutil.mavlink_connection(port)


def main(argv: list[str] | None = None) -> int:
    """Print the four tune programs. This entry does not fly."""
    args = list(sys.argv[1:] if argv is None else argv)
    if len(args) > 1:
        print("usage: python3 -m calibration.run_cascade [ULG]", file=sys.stderr)
        return 2
    return run_cascade()


if __name__ == "__main__":
    raise SystemExit(main())
