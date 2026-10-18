from pathlib import Path
import struct
import sys

import pytest
from pymavlink import mavutil

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.offboard_stream import (  # noqa: E402
    MAV_FRAME_LOCAL_NED,
    VELOCITY_ONLY_TYPEMASK,
    offboard_bind_port,
    stream_at,
)
from calibration.replay_offboard import (  # noqa: E402
    apply_params,
    requests_position_hold,
    trace_sample,
    write_velocity_csv,
)
from calibration.run_search import iris_csv_path, log_series_replay_kwargs  # noqa: E402
from calibration.ulg_velocity import VelocitySample  # noqa: E402

_REAL32 = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
_REAL64 = mavutil.mavlink.MAV_PARAM_TYPE_REAL64
_INT32 = mavutil.mavlink.MAV_PARAM_TYPE_INT32


def test_mask_port_and_unshifted_grid():
    assert VELOCITY_ONLY_TYPEMASK == 3527
    assert MAV_FRAME_LOCAL_NED == 1
    assert offboard_bind_port(0) == 14540
    assert offboard_bind_port(2) == 14542
    with pytest.raises(ValueError):
        offboard_bind_port(-1)
    samples = [VelocitySample(0.0, 0.0, 0.0, 0.0), VelocitySample(1.0, 2.0, 0.0, 0.0)]
    grid = stream_at(samples, hz=2.0)
    assert [s.t_s for s in grid] == pytest.approx([0.0, 0.5, 1.0])
    assert grid[1].vx == pytest.approx(1.0)


class _ParamValue:
    def __init__(self, param_id: str, param_value: float, param_type: int):
        self.param_id = param_id
        self.param_value = param_value
        self.param_type = param_type


class _ScriptedLink:
    """PARAM_VALUE script with no socket. Two echoes per key: fetch, then set."""

    def __init__(self, messages: list[_ParamValue]):
        self._pending = list(messages)
        self.sent: list[tuple[str, float, int]] = []

    def param_fetch_one(self, name: str) -> None:
        return None

    def param_set_send(self, name, value, param_type) -> None:
        self.sent.append((str(name), float(value), int(param_type)))

    def recv_match(self, type=None, blocking=True, timeout=None):
        if not self._pending:
            return None
        return self._pending.pop(0)


def _pair(name: str, param_type: int, fetched: float, echoed: float) -> list[_ParamValue]:
    return [
        _ParamValue(name, fetched, param_type),
        _ParamValue(name, echoed, param_type),
    ]


def _int_bits(value: int) -> float:
    return struct.unpack("<f", struct.pack("<i", value))[0]


@pytest.mark.parametrize(
    "name",
    [
        "EKF2_GPS_CTRL",
        "SYS_AUTOSTART",
        "BAT_N_CELLS",
        "GPS_1_CONFIG",
        "CA_ROTOR_COUNT",
        "PWM_MAIN_MIN",
        "SENS_BOARD_ROT",
    ],
)
def test_disallowed_parameter_is_not_set(name):
    link = _ScriptedLink(
        _pair(name, _REAL32, 1.0, 1.0) + _pair("MPC_XY_P", _REAL32, 0.2, 0.2)
    )
    with pytest.raises(ValueError, match=name):
        apply_params(link, {name: 1.0, "MPC_XY_P": 0.2})
    assert link.sent == []


def test_mpc_thr_hover_other_than_half_is_not_set():
    link = _ScriptedLink(_pair("MPC_THR_HOVER", _REAL32, 0.7, 0.7) + _pair("MC_ROLL_P", _REAL32, 6.0, 6.0))
    with pytest.raises(ValueError, match="MPC_THR_HOVER"):
        apply_params(link, {"MPC_THR_HOVER": 0.7, "MC_ROLL_P": 6.0})
    assert link.sent == []


@pytest.mark.parametrize("param_type", [_REAL32, _REAL64])
def test_real_param_echo_must_match_request(param_type):
    link = _ScriptedLink(_pair("MPC_XY_P", param_type, 0.1, 0.1))
    with pytest.raises(ValueError, match="MPC_XY_P"):
        apply_params(link, {"MPC_XY_P": 0.2})


def test_integer_param_echo_must_match_rounded_request():
    link = _ScriptedLink(
        _pair("MC_ROLLRATE_MAX", _INT32, _int_bits(1500), _int_bits(1599))
    )
    with pytest.raises(ValueError, match="MC_ROLLRATE_MAX"):
        apply_params(link, {"MC_ROLLRATE_MAX": 1600.4})


def test_allowed_params_set_when_hover_is_half_and_echo_matches():
    link = _ScriptedLink(
        _pair("MPC_THR_HOVER", _REAL32, 0.4, 0.5)
        + _pair("MPC_XY_P", _REAL32, 0.1, 0.2 + 1e-8)
        + _pair("MC_ROLLRATE_MAX", _INT32, _int_bits(1500), _int_bits(1600))
    )
    apply_params(
        link,
        {"MPC_THR_HOVER": 0.5, "MPC_XY_P": 0.2, "MC_ROLLRATE_MAX": 1600.4},
    )
    assert [item[0] for item in link.sent] == [
        "MPC_THR_HOVER",
        "MPC_XY_P",
        "MC_ROLLRATE_MAX",
    ]
    assert link.sent[2][1] == pytest.approx(_int_bits(1600))


def test_log_series_fast_first_sample_does_not_request_position_hold():
    # cat2's first finite setpoint is about 1.1 m/s. A log series does not hold.
    opening = VelocitySample(0.90, 0.0, 0.0, 1.10)
    assert (opening.vx**2 + opening.vy**2 + opening.vz**2) ** 0.5 >= 0.2
    assert requests_position_hold([opening], log_series=True, position_hold=True) is False
    call = log_series_replay_kwargs([opening], {"MPC_XY_P": 0.2}, 0)
    assert call["log_series"] is True
    assert call["position_hold"] is False
    assert (
        requests_position_hold(
            call["samples"],
            log_series=call["log_series"],
            position_hold=call["position_hold"],
        )
        is False
    )


def test_synthetic_series_requests_position_hold_from_rest():
    # A sine or chirp opens near (0, 0, 0). The synthetic flag still holds.
    opening = VelocitySample(0.0, 0.0, 0.0, 0.0)
    assert (opening.vx**2 + opening.vy**2 + opening.vz**2) ** 0.5 < 0.2
    assert requests_position_hold([opening], log_series=False, position_hold=True) is True


def test_search_csv_is_selected_model_not_last_neighbor(tmp_path: Path, monkeypatch):
    # The last neighbor is scored after the winner. run_compare reads
    # calibration/results/iris/<stem>.csv, so that file must stay the winner.
    from dataclasses import replace

    from calibration.iris_dynamics import STOCK, save_dynamics
    from calibration.run_compare import load_iris_csv
    from calibration.run_search import main
    from calibration.ulg_velocity import LogVelocity

    stem = "Group3RLcat2_log_41_2026-8-25-17-05-30"
    log = LogVelocity(
        path=str(tmp_path / f"{stem}.ulg"),
        command=[
            VelocitySample(0.0, 1.0, 0.0, 0.0),
            VelocitySample(1.0, 1.0, 0.0, 0.0),
        ],
        response=[
            VelocitySample(0.0, 0.0, 0.0, 0.0),
            VelocitySample(1.0, 0.0, 0.0, 0.0),
        ],
        z_m=[-1.3, -1.3],
        initial_parameters={"MPC_THR_HOVER": 0.5},
    )
    selected = [
        VelocitySample(0.0, 0.0, 0.0, 0.0),
        VelocitySample(1.0, 0.0, 0.0, 0.0),
    ]
    last_neighbor = [
        VelocitySample(0.0, 5.0, 0.0, 0.0),
        VelocitySample(1.0, 5.0, 0.0, 0.0),
    ]
    traces = iter((selected, last_neighbor))
    dynamics_json = tmp_path / "iris_dynamics.json"
    model_path = tmp_path / "iris.sdf.jinja"
    save_dynamics(dynamics_json, STOCK)
    model_path.write_text("unused\n", encoding="utf-8")
    csv_path = tmp_path / "iris" / f"{stem}.csv"

    monkeypatch.setattr("calibration.run_search._dynamics_json", lambda: dynamics_json)
    monkeypatch.setattr("calibration.run_search._model_path", lambda: model_path)
    monkeypatch.setattr("calibration.run_search.BASEBALL_LOGS", (f"{stem}.ulg",))
    monkeypatch.setattr("calibration.run_search.load_log", lambda path: log)
    monkeypatch.setattr("calibration.run_search.iris_csv_path", lambda path: csv_path)
    monkeypatch.setattr(
        "calibration.run_search.labeled_candidates",
        lambda center: [("center", center), ("mass_kg*0.5", replace(STOCK, mass_kg=0.75))],
    )
    monkeypatch.setattr("calibration.run_search.replay", lambda **kwargs: next(traces))
    monkeypatch.setattr("calibration.run_search.publish_dynamics", lambda *args, **kwargs: None)
    monkeypatch.setattr("calibration.run_search._wait_for_restart", lambda *args, **kwargs: None)

    assert main(["--write"]) == 0
    loaded = load_iris_csv(csv_path)
    assert [sample.vx for sample in loaded] == pytest.approx([0.0, 0.0])


def test_csv_writer_stamps_setpoint_time_not_wall_clock(tmp_path: Path):
    setpoint = VelocitySample(0.90, 0.0, 0.0, 1.10)
    wall_clock_s = 12.5
    captured = trace_sample(setpoint, 0.2, -0.1, 0.4)
    stem = "Group3RLcat2_log_41_2026-8-25-17-05-30"
    path = tmp_path / f"{stem}.csv"
    write_velocity_csv(path, [captured])
    header, row = path.read_text(encoding="utf-8").splitlines()[:2]
    assert header == "t_s,vx,vy,vz"
    assert float(row.split(",")[0]) == pytest.approx(setpoint.t_s)
    assert str(wall_clock_s) not in path.read_text(encoding="utf-8")
    assert iris_csv_path(Path(f"/tmp/{stem}.ulg")).as_posix().endswith(
        f"calibration/results/iris/{stem}.csv"
    )
