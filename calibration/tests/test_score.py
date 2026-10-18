from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.score import passes, rms_velocity, select_best  # noqa: E402
from calibration.ulg_velocity import VelocitySample  # noqa: E402

def _series(values, t0=0.0, dt=0.1):
    return [VelocitySample(t0 + i * dt, v, 0.0, 0.0) for i, v in enumerate(values)]

def test_lagged_trace_is_not_shifted():
    # Response steps 0 → 1 at t=1. Sim does the same 0.2 s later.
    # A 0.2 s shift would score ~0. The score must stay on the log clock.
    response = _series([0.0] * 10 + [1.0] * 10)
    sim = _series([0.0] * 12 + [1.0] * 8)
    z = [1.0] * 20
    command = _series([1.0] * 20)
    rms = rms_velocity(sim, response, z, command)
    assert rms["vx"] > 0.1

def test_ground_and_nan_command_are_dropped():
    response = _series([1.0, 1.0, 1.0])
    sim = _series([1.0, 0.0, 1.0])
    z = [0.1, 1.0, 1.0]  # first sample still on the ground
    command = [
        VelocitySample(0.0, 1.0, 0.0, 0.0),
        VelocitySample(0.1, float("nan"), 0.0, 0.0),
        VelocitySample(0.2, 1.0, 0.0, 0.0),
    ]
    rms = rms_velocity(sim, response, z, command)
    assert rms["vx"] == pytest.approx(0.0)

def test_select_best_does_not_loosen_and_prefers_a_pass():
    fail_low = [{"vx": 0.21, "vy": 0.0, "vz": 0.0}]
    pass_high = [{"vx": 0.19, "vy": 0.09, "vz": 0.07}]
    name, ok = select_best([("low", fail_low), ("high", pass_high)])
    assert name == "high" and ok is True
    name, ok = select_best([("a", fail_low), ("b", [{"vx": 0.5, "vy": 0.0, "vz": 0.0}])])
    assert name == "a" and ok is False
    assert passes(fail_low) is False
    assert passes([{"vx": 0.20, "vy": 0.10, "vz": 0.08}]) is True

def test_empty_mask_raises():
    response = _series([1.0])
    with pytest.raises(ValueError):
        rms_velocity(response, response, [0.0], response)
