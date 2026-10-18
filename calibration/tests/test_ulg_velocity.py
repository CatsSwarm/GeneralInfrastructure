from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.ulg_velocity import (  # noqa: E402
    BASEBALL_DIR,
    BASEBALL_LOGS,
    controller_parameters,
    load_log,
)


def test_controller_parameters_keeps_mpc_mc_only():
    kept = controller_parameters({
        "MPC_THR_HOVER": 0.5,
        "MC_ROLLRATE_MAX": 1600.0,
        "EKF2_GPS_CTRL": 7.0,
        "SYS_AUTOSTART": 10015.0,
        "BAT1_N_CELLS": 4.0,
    })
    assert kept == {"MPC_THR_HOVER": 0.5, "MC_ROLLRATE_MAX": 1600.0}


def test_baseball_cat2_and_cat3_hover_and_down_limit():
    cat2 = load_log(BASEBALL_DIR / BASEBALL_LOGS[0])
    cat3 = load_log(BASEBALL_DIR / BASEBALL_LOGS[1])
    assert cat2.initial_parameters["MPC_THR_HOVER"] == pytest.approx(0.5)
    assert cat2.initial_parameters["MPC_Z_VEL_MAX_DN"] == pytest.approx(1.5)
    assert cat3.initial_parameters["MPC_Z_VEL_MAX_DN"] == pytest.approx(3.0)
    assert "EKF2_GPS_CTRL" not in controller_parameters(cat3.initial_parameters)
    assert len(cat2.response) > 1000
    assert len(cat2.z_m) == len(cat2.response)
    finite_vx = [s.vx for s in cat2.command if s.vx == s.vx]
    assert max(abs(v) for v in finite_vx) == pytest.approx(3.5, abs=0.05)
