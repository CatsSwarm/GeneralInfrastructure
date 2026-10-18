"""Step Quadcopter_SimCon from a NED velocity series."""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

from calibration.ulg_velocity import VelocitySample

SIMCON_ROOT = Path("/home/valentin/RL/CatSwarm/TrainingRoom/Quadcopter_SimCon")

_HOVER_Z_NED = -1.3


def integration_start_s(samples: list[VelocitySample]) -> float:
    """First command time. ``fly`` does not integrate the gap before it."""
    if not samples:
        raise ValueError("samples is empty")
    return float(samples[0].t_s)


def desired_from_velocity(vx: float, vy: float, vz: float) -> dict:
    return {
        "pos": [math.nan, math.nan, math.nan],
        "vel": [vx, vy, vz],
        "yaw_rate": 0.0,
    }


def fly(samples: list[VelocitySample], ts: float = 0.002) -> list[VelocitySample]:
    """Fly quad 0 and return its NED velocity at each input sample time.

    Commands are interpolated onto the simulator clock with ``numpy.interp``.
    Integration starts at the first sample time. The initial NED position is
    z=-1.3 and the initial velocity is zero. Each step is ``desiredState``
    (XYZ_VEL), then the controller, then ``quads.update``, matching
    ``quad_sim_system_manager``.
    """
    if ts <= 0.0:
        raise ValueError(f"ts must be positive, got {ts}")
    if not samples:
        return []

    os.environ.setdefault("MPLBACKEND", "Agg")
    simulation = str(SIMCON_ROOT / "Simulation")
    if simulation not in sys.path:
        sys.path.insert(0, simulation)

    from ctrl import Control, ControlType
    from quadFiles.quad import QuadcopterSwarm
    from scipy.integrate import ode
    from trajectory import (
        PositionTrajectoryType,
        Trajectory,
        WaypointTimeMode,
        YawTrajectoryType,
    )
    from utils.windModel import Wind

    times = np.array([sample.t_s for sample in samples], dtype=float)
    if times.size > 1 and np.any(np.diff(times) < 0.0):
        raise ValueError("sample times must be non-decreasing")
    cmd_vx = np.array([sample.vx for sample in samples], dtype=float)
    cmd_vy = np.array([sample.vy for sample in samples], dtype=float)
    cmd_vz = np.array([sample.vz for sample in samples], dtype=float)

    drone_params, control_params = _load_cat_config(simulation)
    quads = QuadcopterSwarm(numOfQuads=1, Ti=0, params=drone_params)
    # QuadcopterSwarm passes first_step/atol/rtol as strings. Current scipy
    # rejects those. Same dopri5 settings, as floats. SimCon files stay as they are.
    quads.integrator = ode(quads.state_dot_wrapper).set_integrator(
        "dopri5",
        first_step=float("0.00005"),
        atol=float("1e-6"),
        rtol=float("1e-6"),
    )
    t0 = integration_start_s(samples)
    quads.Ti = t0
    quads.setInitialQuadPos(np.array([0.0, 0.0, _HOVER_Z_NED]), 0)
    quads.setInitialQuadVel(np.array([0.0, 0.0, 0.0]), 0)

    # Same trajSelect as _real_flight_traj_select for a velocity controller.
    traj_select = np.array(
        [
            PositionTrajectoryType.POS_WAYPOINT_TIMED.value,
            YawTrajectoryType.FOLLOW.value,
            WaypointTimeMode.AVERAGE_SPEED.value,
        ]
    )
    traj = Trajectory(quads, ControlType.XYZ_VEL, traj_select)
    ctrl = Control(quads, traj.yawType, control_params=control_params)
    wind = Wind("None", 2.0, 90, -15)

    rec_t = [t0]
    rec_vx = [float(quads.vel[0, 0])]
    rec_vy = [float(quads.vel[0, 1])]
    rec_vz = [float(quads.vel[0, 2])]

    t_end = float(times[-1])
    span = t_end - t0
    n_steps = int(round(span / ts)) if span > 0.0 else 0
    if t0 + n_steps * ts < t_end - 1e-9:
        n_steps += 1

    for step in range(n_steps):
        t = t0 + step * ts
        desired = desired_from_velocity(
            float(np.interp(t, times, cmd_vx)),
            float(np.interp(t, times, cmd_vy)),
            float(np.interp(t, times, cmd_vz)),
        )
        traj.desiredState(t, ts, quads, desired=desired)
        # XYZ_VEL desiredState ignores `desired` and writes testVelControl into sDes.
        _assign_desired(traj, desired)
        ctrl.controller(traj, quads, ts)
        quads.update(t, ts, ctrl.w_cmd, wind)
        rec_t.append(t0 + (step + 1) * ts)
        rec_vx.append(float(quads.vel[0, 0]))
        rec_vy.append(float(quads.vel[0, 1]))
        rec_vz.append(float(quads.vel[0, 2]))

    out_vx = np.interp(times, rec_t, rec_vx)
    out_vy = np.interp(times, rec_t, rec_vy)
    out_vz = np.interp(times, rec_t, rec_vz)
    return [
        VelocitySample(sample.t_s, float(out_vx[index]), float(out_vy[index]), float(out_vz[index]))
        for index, sample in enumerate(samples)
    ]


def _assign_desired(traj, desired: dict) -> None:
    pos = desired["pos"]
    vel = desired["vel"]
    vec = np.array(
        [
            pos[0],
            pos[1],
            pos[2],
            vel[0],
            vel[1],
            vel[2],
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            float(desired["yaw_rate"]),
        ],
        dtype=float,
    )
    traj.sDes = np.tile(vec.reshape(-1, 1), (1, traj.numOfQuads))


def _load_cat_config(simulation: str):
    config_path = os.path.join(simulation, "cat_drone_config.json")
    if not os.path.isfile(config_path):
        raise FileNotFoundError(config_path)
    from load_drone_config import load_drone_config

    return load_drone_config(config_path)
