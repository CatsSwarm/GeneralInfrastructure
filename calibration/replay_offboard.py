#!/usr/bin/env python3
"""Stream a velocity series to PX4 SITL as offboard setpoints.

Connects ``udpin:127.0.0.1:<offboard_bind_port>``, waits for a heartbeat, and
optionally PARAM_SETs a JSON object of controller parameters before any
setpoint. Only ``MPC_`` / ``MC_`` keys are sent, ``MPC_THR_HOVER`` must be
0.5, and the post-set ``PARAM_VALUE`` must store the requested number. Arms, switches to OFFBOARD, and streams ``stream_at`` output as
SET_POSITION_TARGET_LOCAL_NED at 50 Hz.

A log series (``log_series=True``) does not position-hold. It streams the logged
samples on the log clock from the first sample. A synthetic series requests the
hold with ``position_hold=True``: NED z=-1.3 until |z| > 0.4 m and speed is under
0.2 m/s, including when the first sample is near rest. That hold is before the
case clock and is not part of the returned trace.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import struct
import sys
import time
from pathlib import Path

from pymavlink import mavutil

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from calibration.offboard_stream import (
    MAV_FRAME_LOCAL_NED,
    VELOCITY_ONLY_TYPEMASK,
    offboard_bind_port,
    stream_at,
)
from calibration.ulg_velocity import VelocitySample

STREAM_HZ = 50.0
HOVER_Z_NED = -1.3
AIRBORNE_ABS_Z_M = 0.4
HOVER_SPEED_MPS = 0.2
_WARMUP_S = 1.0
_HEARTBEAT_TIMEOUT_S = 30.0
_ARM_TIMEOUT_S = 60.0
_HOLD_TIMEOUT_S = 120.0
_PARAM_TIMEOUT_S = 2.0
_COMMAND_PERIOD_S = 0.5
_FORCE_ARM_AFTER_S = 5.0
_PX4_FORCE_ARM = 21196.0
_HOVER_THRUST = 0.5
_ECHO_REL_TOL = 1e-5
_ECHO_ABS_TOL = 1e-5

_CONTROLLER_KEY = re.compile(r"^(MPC_|MC_)")
_REAL_PARAM_TYPES = {
    mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    mavutil.mavlink.MAV_PARAM_TYPE_REAL64,
}


def _position_hold_typemask() -> int:
    mask = mavutil.mavlink
    return (
        mask.POSITION_TARGET_TYPEMASK_VX_IGNORE
        | mask.POSITION_TARGET_TYPEMASK_VY_IGNORE
        | mask.POSITION_TARGET_TYPEMASK_VZ_IGNORE
        | mask.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mask.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mask.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mask.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | mask.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )


def requests_position_hold(
    samples: list[VelocitySample], *, log_series: bool, position_hold: bool
) -> bool:
    """Log series never holds. Synthetic series hold only when asked."""
    if not samples:
        raise ValueError("series has no samples")
    if log_series:
        return False
    return bool(position_hold)


def trace_sample(setpoint: VelocitySample, vx: float, vy: float, vz: float) -> VelocitySample:
    """Velocity captured at the active setpoint's log time."""
    return VelocitySample(float(setpoint.t_s), float(vx), float(vy), float(vz))


def write_velocity_csv(path: Path, samples: list[VelocitySample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(["t_s", "vx", "vy", "vz"])
        for sample in samples:
            writer.writerow([sample.t_s, sample.vx, sample.vy, sample.vz])


def load_samples(path: Path) -> list[VelocitySample]:
    payload = json.loads(path.read_text())
    if not isinstance(payload, list) or not payload:
        raise ValueError(f"{path} must be a non-empty JSON list of samples")
    samples: list[VelocitySample] = []
    for index, row in enumerate(payload):
        if not isinstance(row, dict):
            raise ValueError(f"{path} sample {index} must be an object")
        try:
            samples.append(
                VelocitySample(
                    float(row["t_s"]),
                    float(row["vx"]),
                    float(row["vy"]),
                    float(row["vz"]),
                )
            )
        except KeyError as exc:
            raise ValueError(f"{path} sample {index} missing {exc.args[0]}") from exc
    return samples


def load_params(path: Path | None) -> dict[str, float]:
    if path is None:
        return {}
    payload = json.loads(path.read_text())
    if not isinstance(payload, dict):
        raise ValueError(f"{path} must be a JSON object of controller parameters")
    return {str(key): float(value) for key, value in payload.items()}


def _param_name(message) -> str:
    raw = message.param_id
    if isinstance(raw, bytes):
        raw = raw.decode("ascii", errors="ignore")
    return str(raw).split("\x00", 1)[0].strip()


def _wire_param(value: float, param_type: int) -> float:
    if param_type in _REAL_PARAM_TYPES:
        return float(value)
    bits = int(round(float(value)))
    return struct.unpack("<f", struct.pack("<i", bits))[0]


def _wait_param(master, name: str, timeout_s: float):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        message = master.recv_match(
            type="PARAM_VALUE",
            blocking=True,
            timeout=min(0.2, max(remaining, 0.0)),
        )
        if message is not None and _param_name(message) == name:
            return message
    raise TimeoutError(f"no PARAM_VALUE for {name}")


def _require_sendable_params(params: dict[str, float]) -> None:
    """Refuse anything other than MPC_/MC_ before a PARAM_SET is sent.

    MPC_THR_HOVER stays 0.5. A bad file must not change the estimator, battery,
    or mixer, and must not retune hover thrust.
    """
    for name, value in params.items():
        if _CONTROLLER_KEY.match(name) is None:
            raise ValueError(
                f"refusing PARAM_SET of {name}: only MPC_ and MC_ parameters may be sent"
            )
        if name == "MPC_THR_HOVER" and not math.isclose(
            float(value), _HOVER_THRUST, rel_tol=0.0, abs_tol=1e-9
        ):
            raise ValueError(f"MPC_THR_HOVER must stay {_HOVER_THRUST}, got {value}")


def _require_param_echo(message, requested: float) -> None:
    """The post-set PARAM_VALUE is PX4's stored value, not just the name."""
    name = _param_name(message)
    param_type = int(message.param_type)
    echoed = float(message.param_value)
    if param_type in _REAL_PARAM_TYPES:
        if not math.isclose(echoed, float(requested), rel_tol=_ECHO_REL_TOL, abs_tol=_ECHO_ABS_TOL):
            raise ValueError(
                f"{name} PARAM_VALUE {echoed} does not match requested {float(requested)}"
            )
        return
    stored = struct.unpack("<i", struct.pack("<f", echoed))[0]
    wanted = int(round(float(requested)))
    if stored != wanted:
        raise ValueError(f"{name} PARAM_VALUE {stored} does not match requested {wanted}")


def apply_params(master, params: dict[str, float]) -> None:
    """PARAM_SET each allowed key. Call after the heartbeat and before setpoints."""
    _require_sendable_params(params)
    for name, value in params.items():
        if len(name) > 16:
            raise ValueError(f"parameter name {name!r} is longer than 16 characters")
        master.param_fetch_one(name)
        current = _wait_param(master, name, _PARAM_TIMEOUT_S)
        master.param_set_send(name, _wire_param(value, int(current.param_type)), int(current.param_type))
        echo = _wait_param(master, name, _PARAM_TIMEOUT_S)
        _require_param_echo(echo, value)


def _send_velocity(master, sample: VelocitySample, time_boot_ms: int) -> None:
    master.mav.set_position_target_local_ned_send(
        time_boot_ms,
        master.target_system,
        master.target_component,
        MAV_FRAME_LOCAL_NED,
        VELOCITY_ONLY_TYPEMASK,
        0.0,
        0.0,
        0.0,
        float(sample.vx),
        float(sample.vy),
        float(sample.vz),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def _send_position_hold(master, x_m: float, y_m: float, time_boot_ms: int) -> None:
    master.mav.set_position_target_local_ned_send(
        time_boot_ms,
        master.target_system,
        master.target_component,
        MAV_FRAME_LOCAL_NED,
        _position_hold_typemask(),
        float(x_m),
        float(y_m),
        HOVER_Z_NED,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def _request_local_position(master) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        1e6 / STREAM_HZ,
        0,
        0,
        0,
        0,
        0,
    )


class _Vehicle:
    def __init__(self, master):
        self.master = master
        self.position = None
        self.armed = False
        self.offboard = False
        self.hold_xy: tuple[float, float] | None = None

    def drain(self) -> None:
        for _ in range(100):
            message = self.master.recv_match(blocking=False)
            if message is None:
                return
            kind = message.get_type()
            if kind == "LOCAL_POSITION_NED":
                self.position = message
                if self.hold_xy is None:
                    self.hold_xy = (float(message.x), float(message.y))
            elif kind == "HEARTBEAT" and message.get_srcSystem() == self.master.target_system:
                base_mode = int(message.base_mode)
                self.armed = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                main_mode = (int(message.custom_mode) & 0xFF0000) >> 16
                self.offboard = main_mode == mavutil.PX4_CUSTOM_MAIN_MODE_OFFBOARD

    def settled(self) -> bool:
        if self.position is None:
            return False
        speed = math.hypot(self.position.vx, self.position.vy, self.position.vz)
        return abs(float(self.position.z)) > AIRBORNE_ABS_Z_M and speed < HOVER_SPEED_MPS


def _pace(target_s: float) -> None:
    delay_s = target_s - time.monotonic()
    if delay_s > 0.0:
        time.sleep(delay_s)


def _boot_ms(origin_s: float) -> int:
    return int((time.monotonic() - origin_s) * 1000.0)


def _request_offboard(master) -> None:
    base_mode, main_mode, sub_mode = mavutil.px4_map["OFFBOARD"]
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        base_mode,
        main_mode,
        sub_mode,
        0,
        0,
        0,
        0,
    )


def _request_offboard_and_arm(
    master,
    vehicle: _Vehicle,
    now_s: float,
    last_command_s: float,
    arm_started_s: float,
) -> float:
    if now_s - last_command_s < _COMMAND_PERIOD_S:
        return last_command_s
    if not vehicle.offboard:
        _request_offboard(master)
    if not vehicle.armed:
        force = now_s - arm_started_s >= _FORCE_ARM_AFTER_S
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            _PX4_FORCE_ARM if force else 0,
            0,
            0,
            0,
            0,
            0,
        )
    return now_s


def replay(
    samples: list[VelocitySample],
    params: dict[str, float],
    px4_instance: int,
    *,
    log_series: bool = False,
    position_hold: bool = False,
) -> list[VelocitySample]:
    port = offboard_bind_port(px4_instance)
    url = f"udpin:127.0.0.1:{port}"
    print(f"connecting {url}")
    master = mavutil.mavlink_connection(url)
    heartbeat = master.wait_heartbeat(timeout=_HEARTBEAT_TIMEOUT_S)
    if heartbeat is None:
        raise TimeoutError(f"no heartbeat on {url}")
    print(f"heartbeat from system {master.target_system} component {master.target_component}")
    if params:
        print(f"PARAM_SET {len(params)} controller parameters")
        apply_params(master, params)
    _request_local_position(master)

    grid = stream_at(samples, hz=STREAM_HZ)
    needs_hold = requests_position_hold(
        samples, log_series=log_series, position_hold=position_hold
    )
    vehicle = _Vehicle(master)
    origin_s = time.monotonic()
    next_tick_s = origin_s
    period_s = 1.0 / STREAM_HZ
    position_mask = _position_hold_typemask()

    def send_current(sample: VelocitySample | None) -> None:
        time_boot_ms = _boot_ms(origin_s)
        if sample is None:
            hold_x, hold_y = vehicle.hold_xy if vehicle.hold_xy is not None else (0.0, 0.0)
            _send_position_hold(master, hold_x, hold_y, time_boot_ms)
        else:
            _send_velocity(master, sample, time_boot_ms)

    warmup_setpoint = None if needs_hold else grid[0]
    warmup_until_s = time.monotonic() + _WARMUP_S
    while time.monotonic() < warmup_until_s:
        _pace(next_tick_s)
        next_tick_s += period_s
        vehicle.drain()
        send_current(warmup_setpoint)

    arm_started_s = time.monotonic()
    arm_deadline_s = arm_started_s + _ARM_TIMEOUT_S
    last_command_s = 0.0
    while not (vehicle.armed and vehicle.offboard):
        if time.monotonic() > arm_deadline_s:
            raise TimeoutError("PX4 did not arm in OFFBOARD")
        _pace(next_tick_s)
        next_tick_s += period_s
        vehicle.drain()
        send_current(warmup_setpoint)
        last_command_s = _request_offboard_and_arm(
            master, vehicle, time.monotonic(), last_command_s, arm_started_s
        )
    print("armed in OFFBOARD")

    if needs_hold:
        print(f"position hold NED z={HOVER_Z_NED} mask={position_mask}")
        hold_deadline_s = time.monotonic() + _HOLD_TIMEOUT_S
        last_position_request_s = 0.0
        while not vehicle.settled():
            now_s = time.monotonic()
            if now_s > hold_deadline_s:
                raise TimeoutError(
                    f"position hold did not reach |z| > {AIRBORNE_ABS_Z_M} m "
                    f"and speed < {HOVER_SPEED_MPS} m/s"
                )
            _pace(next_tick_s)
            next_tick_s += period_s
            vehicle.drain()
            if vehicle.position is None and now_s - last_position_request_s >= 1.0:
                _request_local_position(master)
                last_position_request_s = now_s
            send_current(None)
            if not vehicle.offboard or not vehicle.armed:
                last_command_s = _request_offboard_and_arm(
                    master, vehicle, time.monotonic(), last_command_s, arm_started_s
                )
        print("hover reached; starting series clock")

    clock_start_s = time.monotonic()
    series_t0 = grid[0].t_s
    trace: list[VelocitySample] = []
    print(f"streaming {len(grid)} velocity setpoints at {STREAM_HZ:g} Hz")
    for sample in grid:
        _pace(clock_start_s + (sample.t_s - series_t0))
        vehicle.drain()
        send_current(sample)
        if vehicle.position is not None:
            position = vehicle.position
            trace.append(
                trace_sample(
                    sample,
                    float(position.vx),
                    float(position.vy),
                    float(position.vz),
                )
            )
        if not vehicle.offboard or not vehicle.armed:
            last_command_s = _request_offboard_and_arm(
                master, vehicle, time.monotonic(), last_command_s, arm_started_s
            )
    print("series complete")
    return trace


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Stream velocity setpoints to PX4 SITL offboard")
    parser.add_argument(
        "--samples",
        required=True,
        type=Path,
        help="JSON list of objects with t_s, vx, vy, vz",
    )
    parser.add_argument(
        "--params",
        type=Path,
        default=None,
        help="JSON object of controller parameters; omitted sends none",
    )
    parser.add_argument("--instance", type=int, default=0, help="PX4 SITL instance (default 0)")
    parser.add_argument(
        "--log-series",
        action="store_true",
        help="log series: stream the recorded setpoints and do not position-hold",
    )
    parser.add_argument(
        "--position-hold",
        action="store_true",
        help="synthetic series: position-hold before the case clock",
    )
    args = parser.parse_args(argv)
    try:
        replay(
            load_samples(args.samples),
            load_params(args.params),
            args.instance,
            log_series=args.log_series,
            position_hold=args.position_hold,
        )
    except (OSError, ValueError, TimeoutError, json.JSONDecodeError, struct.error) as exc:
        print(f"replay_offboard: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
