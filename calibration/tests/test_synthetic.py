from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.synthetic import synthetic_cases  # noqa: E402


def test_case_table_and_five_meter_cap():
    cases = synthetic_cases()
    assert [c.name for c in cases] == [
        "step_vx_p1", "step_vx_m1", "step_vx_p3", "step_vx_m3", "step_vx_p5", "step_vx_m5",
        "step_vy_p1", "step_vy_m1", "step_vy_p3", "step_vy_m3", "step_vy_p5", "step_vy_m5",
        "sine_vx_0.2hz", "sine_vx_1.0hz", "sine_vy_0.2hz", "sine_vy_1.0hz",
        "chirp_vx", "chirp_vy",
    ]
    by = {c.name: c for c in cases}
    assert by["step_vx_p5"].velocity_at(0.0) == (5.0, 0.0, 0.0)
    assert by["step_vx_p5"].velocity_at(4.0) == (5.0, 0.0, 0.0)
    assert by["step_vy_m3"].velocity_at(1.0) == (0.0, -3.0, 0.0)
    assert by["sine_vx_1.0hz"].velocity_at(0.25) == pytest.approx((2.0, 0.0, 0.0))
    assert by["chirp_vx"].velocity_at(10.0) == pytest.approx((-1.0, 0.0, 0.0))
    for case in cases:
        for t in (0.0, case.duration_s / 2, case.duration_s):
            vx, vy, vz = case.velocity_at(t)
            assert abs(vx) <= 5.0 and abs(vy) <= 5.0 and vz == 0.0
    with pytest.raises(ValueError):
        by["step_vx_p1"].velocity_at(4.01)
