# Iris Velocity Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fit a repo-local Gazebo iris model so its `vx`, `vy`, and `vz` follow the three Baseball ULG flights, then report the same commands flown in Quadcopter_SimCon.

**Architecture:** Pure Python pieces (log reader, RMS score, synthetic commands, SDF patch, report) are unit-tested and share locked dataclasses. A later search script replays velocity setpoints on Noetic SITL and writes the winning model. SimCon is imported and stepped from this repo; none of its files change.

**Tech Stack:** Python 3, pytest, numpy, pyulog, pymavlink; Gazebo Classic iris on `runSimNoeticMulti.sh`; Quadcopter_SimCon at `/home/valentin/RL/CatSwarm/TrainingRoom/Quadcopter_SimCon`.

**Spec:** `docs/superpowers/specs/2026-10-15-iris-velocity-calibration-design.md`

## Global Constraints

- Fit target is the three logs under `/home/valentin/RL/TESTFLIGHTS/Baseball/` named in the spec. Logged velocity is the aircraft. The setpoint is the command.
- Replay logged setpoints as recorded. Do not time-shift any trace.
- Airborne mask: `|z| > 0.4` m on NED `vehicle_local_position.z`, and the setpoint sample is finite.
- Accept only when every log has `vx` RMS ≤ 0.20 m/s, `vy` RMS ≤ 0.10 m/s, `vz` RMS ≤ 0.08 m/s. If none pass, keep the best set and do not loosen the bar.
- Synthetic commands stay within 5 m/s on `vx` and on `vy`. They are not part of the accept score.
- PX4 parameters that may be sent to SITL match `^(MPC_|MC_)` from that log's initial set. Do not send `EKF2_`, `SYS_`, `BAT_`, `GPS_`, `CA_`, `PWM_`, or `SENS_` (a real-airframe dump will not boot Gazebo).
- `MPC_THR_HOVER` stays 0.5. Do not edit it to suit the model.
- Adjust only iris `base_link` mass and inertia, and the four `libgazebo_motor_model.so` fields `motorConstant`, `momentConstant`, `timeConstantUp`, `timeConstantDown`, `rotorDragCoefficient`. Do not change rotor-link mass, geometry, or `inject_iris_sensors.py`.
- Do not edit any file under `TrainingRoom/Quadcopter_SimCon`.
- Sim: one iris, empty world, Noetic Gazebo Classic. No network, no system manager.
- **No git commit or push** unless the user explicitly asks.
- Every implementer and reviewer uses model `grok-4.7-high`. Fast models are forbidden.

## Execution

The user asked for parallel subagent reviewers and test-first implementation.

Dispatch implementers in a wave together. They own disjoint files. Dispatch that wave's reviewers together only after every implementer in the wave has returned. A reviewer reads that task's diff, not the whole branch. After Wave 3, one whole-branch reviewer on `grok-4.7-high`.

| Wave | Tasks | Why together |
|------|--------|----------------|
| 1 | 1, 3, 4, 5 | Disjoint files. Tasks 3, 4, and 5 do not import Task 1. |
| 2 | 2, 6, 7 | Each imports `VelocitySample` from Task 1 and writes its own module. |
| 3 | 8, 9 | Task 8 imports the score. Task 9 imports the score and the dynamics type. Disjoint files. |
| 4 | 10 | Imports the report, the stepper, the stream, the cases, and the log loader. |

The parent session coordinates. It does not write the production code.

## Review Focus

These are the failures the spec allows if a task forgets them. Each has a test in the owning task.

- A trace that matches the log only after a 0.2 s shift must still fail the unshifted RMS (Task 2).
- `EKF2_GPS_CTRL` from a log must not be in the parameter set sent to SITL (Task 1).
- Patching dynamics must leave `rotor_0` mass alone (Task 4).
- No synthetic sample may exceed 5 m/s on `vx` or `vy` (Task 3).
- One log with `vx` RMS 0.21 rejects the set even when the other eight numbers are 0 (Task 2).

## File structure

| Path | Responsibility |
|------|----------------|
| `calibration/ulg_velocity.py` | Load one ULG into command, response, `z`, and initial parameters |
| `calibration/score.py` | Unshifted airborne RMS and accept/select rules |
| `calibration/synthetic.py` | The 18 synthetic cases |
| `calibration/iris_dynamics.py` | JSON load/save and SDF patch |
| `calibration/iris_dynamics.json` | Stock iris numbers until a search overwrites them |
| `calibration/apply_iris_dynamics.py` | CLI: `sdf_path json_path` |
| `calibration/offboard_stream.py` | Type mask, bind port, setpoint grid |
| `calibration/replay_offboard.py` | Live MAVLink replay. Not imported by tests |
| `calibration/simcon_velocity.py` | Step Quadcopter_SimCon from a velocity series |
| `calibration/report.py` | Write `calibration/results/velocity_report.json` |
| `calibration/search.py` | Coordinate-descent neighbors and ranking |
| `calibration/run_search.py` | Live SITL search. Pytest calls only `candidate_count` |
| `calibration/run_compare.py` | Live SimCon compare. Not imported by tests |
| `calibration/tests/test_*.py` | Unit tests, no Gazebo and no SimCon process |
| `multidrone/sitl_multiple_run.sh` | Apply the JSON to `/tmp/iris_N.sdf` after color inject |
| `runSimNoeticMulti.sh` | Bind-mount the applier and the JSON |

---

### Task 1: ULG velocity and controller parameters

**Files:**
- Create: `calibration/ulg_velocity.py`
- Test: `calibration/tests/test_ulg_velocity.py`

**Interfaces:**
- Consumes: nothing
- Produces:
  - `BASEBALL_DIR: Path` = `/home/valentin/RL/TESTFLIGHTS/Baseball`
  - `BASEBALL_LOGS: tuple[str, str, str]` the three filenames in the spec, cat2 then cat3 then cat4
  - `@dataclass(frozen=True) class VelocitySample` with `t_s: float`, `vx: float`, `vy: float`, `vz: float`
  - `@dataclass(frozen=True) class LogVelocity` with `path: str`, `command: list[VelocitySample]`, `response: list[VelocitySample]`, `z_m: list[float]` (same length and times as `response`), `initial_parameters: dict[str, float]`
  - `load_log(path: Path) -> LogVelocity`
  - `controller_parameters(initial: dict[str, float]) -> dict[str, float]` — keys matching `^(MPC_|MC_)` only

- [ ] **Step 1: Write the failing test**

```python
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_ulg_velocity.py -v` from `general_infrastructure/`

Expected: FAIL with `ModuleNotFoundError` or `ImportError` for `calibration.ulg_velocity`

- [ ] **Step 3: Implement `load_log` and `controller_parameters` in `calibration/ulg_velocity.py`**

Read `vehicle_local_position` (`timestamp`, `vx`, `vy`, `vz`, `z`) and `vehicle_local_position_setpoint` (`timestamp`, `vx`, `vy`, `vz`) with pyulog. Times are seconds from the first response timestamp. Copy `initial_parameters` as floats. Do not drop NaN setpoints here; the score owns that mask.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest calibration/tests/test_ulg_velocity.py -v`

Expected: PASS

---

### Task 2: Unshifted RMS and the accept rule

**Files:**
- Create: `calibration/score.py`
- Test: `calibration/tests/test_score.py`

**Interfaces:**
- Consumes: `VelocitySample` from `calibration.ulg_velocity`
- Produces:
  - `ACCEPT_RMS: dict[str, float]` = `{"vx": 0.20, "vy": 0.10, "vz": 0.08}`
  - `AIRBORNE_ABS_Z_M: float` = `0.4`
  - `rms_velocity(sim: list[VelocitySample], response: list[VelocitySample], z_m: list[float], command: list[VelocitySample]) -> dict[str, float]`
  - `passes(per_log_rms: list[dict[str, float]]) -> bool`
  - `select_best(candidates: list[tuple[str, list[dict[str, float]]]]) -> tuple[str, bool]`
    - `per_log_rms` has one dict per log, keys `vx` `vy` `vz`
    - among sets where `passes` is true, return the name with the smallest sum of those numbers and `True`
    - if none pass, return the smallest sum and `False`
    - empty `candidates` raises `ValueError`

- [ ] **Step 1: Write the failing test**

```python
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_score.py -v`

Expected: FAIL with import error for `calibration.score`

- [ ] **Step 3: Implement `rms_velocity`, `passes`, and `select_best` in `calibration/score.py`**

Interpolate `sim` onto response timestamps with `numpy.interp` per axis. Interpolate the command the same way and drop non-finite command samples. Do not search for a lag. Require `len(z_m) == len(response)`.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest calibration/tests/test_score.py -v`

Expected: PASS

---

### Task 3: Synthetic velocity cases

**Files:**
- Create: `calibration/synthetic.py`
- Test: `calibration/tests/test_synthetic.py`

**Interfaces:**
- Consumes: nothing
- Produces:
  - `MAX_ABS_V_MPS: float` = `5.0`
  - `@dataclass(frozen=True) class SyntheticCase` with `name: str`, `duration_s: float`, and `velocity_at(self, t_s: float) -> tuple[float, float, float]`
  - `synthetic_cases() -> list[SyntheticCase]` in this order:
    - `step_vx_p1`, `step_vx_m1`, `step_vx_p3`, `step_vx_m3`, `step_vx_p5`, `step_vx_m5`
    - `step_vy_p1`, `step_vy_m1`, `step_vy_p3`, `step_vy_m3`, `step_vy_p5`, `step_vy_m5`
    - `sine_vx_0.2hz`, `sine_vx_1.0hz`, `sine_vy_0.2hz`, `sine_vy_1.0hz`
    - `chirp_vx`, `chirp_vy`
  - Steps last 4 s and are constant. Sines last 10 s, amplitude 2 m/s. Chirps last 20 s, amplitude 1 m/s, frequency linear from 0.1 Hz to 2 Hz.
  - Chirp phase is `2π * (0.1 * t + 1.9 * t² / (2 * 20))`. The unused axes and `vz` are 0.
  - `t_s` outside `[0, duration_s]` raises `ValueError`.

- [ ] **Step 1: Write the failing test**

```python
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_synthetic.py -v`

Expected: FAIL with import error for `calibration.synthetic`

- [ ] **Step 3: Implement `SyntheticCase` and `synthetic_cases` in `calibration/synthetic.py`**

Sine is `amplitude * sin(2π f t)` on the named axis. Use the chirp phase from the interface. Steps do not change with time.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest calibration/tests/test_synthetic.py -v`

Expected: PASS

---

### Task 4: Iris dynamics file and SDF patch

**Files:**
- Create: `calibration/iris_dynamics.py`
- Create: `calibration/iris_dynamics.json`
- Create: `calibration/apply_iris_dynamics.py`
- Test: `calibration/tests/test_iris_dynamics.py`

**Interfaces:**
- Consumes: nothing
- Produces:
  - `@dataclass(frozen=True) class IrisDynamics` with fields `mass_kg`, `ixx`, `iyy`, `izz`, `motor_constant`, `moment_constant`, `time_constant_up`, `time_constant_down`, `rotor_drag_coefficient` (all `float`)
  - `STOCK: IrisDynamics` = mass `1.5`, ixx `0.029125`, iyy `0.029125`, izz `0.055225`, motor_constant `5.84e-06`, moment_constant `0.06`, time_constant_up `0.0125`, time_constant_down `0.025`, rotor_drag_coefficient `0.000175`
  - `load_dynamics(path: Path) -> IrisDynamics`
  - `save_dynamics(path: Path, dynamics: IrisDynamics) -> None`
  - `apply_to_sdf(sdf_path: Path, dynamics: IrisDynamics) -> None` — writes the file in place
  - CLI `python3 calibration/apply_iris_dynamics.py SDF JSON` calls `load_dynamics` then `apply_to_sdf` and returns exit 0

`calibration/iris_dynamics.json` is `STOCK`.

- [ ] **Step 1: Write the failing test**

```python
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.iris_dynamics import (  # noqa: E402
    STOCK,
    apply_to_sdf,
    load_dynamics,
    save_dynamics,
)

SDF = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='iris'>
    <link name='base_link'>
      <inertial>
        <mass>1.5</mass>
        <inertia><ixx>0.029125</ixx><iyy>0.029125</iyy><izz>0.055225</izz></inertia>
      </inertial>
    </link>
    <link name='rotor_0'><inertial><mass>0.005</mass></inertial></link>
    <plugin name='front_right_motor_model' filename='libgazebo_motor_model.so'>
      <motorConstant>5.84e-06</motorConstant>
      <momentConstant>0.06</momentConstant>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <rotorDragCoefficient>0.000175</rotorDragCoefficient>
    </plugin>
  </model>
</sdf>
"""

def test_stock_json_roundtrip_and_rotor_mass_untouched(tmp_path: Path):
    path = tmp_path / "dyn.json"
    save_dynamics(path, STOCK)
    assert load_dynamics(path).mass_kg == pytest.approx(1.5)
    tuned = load_dynamics(ROOT / "calibration" / "iris_dynamics.json")
    assert tuned == STOCK
    sdf = tmp_path / "iris.sdf"
    sdf.write_text(SDF, encoding="utf-8")
    apply_to_sdf(sdf, STOCK.__class__(0.8, 0.01, 0.02, 0.03, 1e-5, 0.05, 0.02, 0.03, 0.001))
    text = sdf.read_text(encoding="utf-8")
    assert "<mass>0.8</mass>" in text
    assert "<mass>0.005</mass>" in text
    assert _tag(text, "motorConstant") == pytest.approx(1e-5)
    assert _tag(text, "rotorDragCoefficient") == pytest.approx(0.001)
    apply_to_sdf(sdf, STOCK)
    restored = sdf.read_text(encoding="utf-8")
    assert "<mass>1.5</mass>" in restored
    assert "<mass>0.005</mass>" in restored

def _tag(text: str, name: str) -> float:
    start = text.index(f"<{name}>") + len(name) + 2
    end = text.index(f"</{name}>")
    return float(text[start:end])
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_iris_dynamics.py -v`

Expected: FAIL with import error for `calibration.iris_dynamics`

- [ ] **Step 3: Implement the dataclass, JSON, and `apply_to_sdf`**

Change `mass`, `ixx`, `iyy`, `izz` only inside `link name="base_link"` (quotes may be single or double). Change the five plugin fields on every `libgazebo_motor_model.so` plugin. Leave every other `mass` element alone. A second apply with different numbers replaces the first.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest calibration/tests/test_iris_dynamics.py -v`

Expected: PASS

---

### Task 5: Spawn hook

**Files:**
- Modify: `multidrone/sitl_multiple_run.sh` (the iris block after `inject_iris_colors.py`)
- Modify: `runSimNoeticMulti.sh` (both `DOCKER_VOLUMES` arrays)
- Test: `calibration/tests/test_spawn_hook.py`

**Interfaces:**
- Consumes: `calibration/apply_iris_dynamics.py` and `calibration/iris_dynamics.json` from Task 4
- Produces: on iris spawn, `python3` applies the JSON to `/tmp/${MODEL}_${N}.sdf` after the color inject. Both docker volume lists mount:
  - `${SCRIPT_DIR}/calibration/apply_iris_dynamics.py` → `/home/valentin/PX4-Autopilot/Tools/simulation/apply_iris_dynamics.py:ro`
  - `${SCRIPT_DIR}/calibration/iris_dynamics.json` → `/home/valentin/PX4-Autopilot/Tools/simulation/iris_dynamics.json:ro`
  The sitl script calls that container path. A missing applier prints a warning and still spawns. It does not skip the color inject.

- [ ] **Step 1: Write the failing test**

```python
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SITL = (ROOT / "multidrone" / "sitl_multiple_run.sh").read_text()
RUN = (ROOT / "runSimNoeticMulti.sh").read_text()

def test_sitl_applies_dynamics_after_colors():
    assert SITL.index("inject_iris_colors.py") < SITL.index("apply_iris_dynamics.py")
    assert "iris_dynamics.json" in SITL

def test_runsim_mounts_applier_in_both_volume_lists():
    assert RUN.count("calibration/apply_iris_dynamics.py") == 2
    assert RUN.count("calibration/iris_dynamics.json") == 2
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_spawn_hook.py -v`

Expected: FAIL because `apply_iris_dynamics.py` is absent from the scripts

- [ ] **Step 3: Add the mount lines and the sitl call**

In `sitl_multiple_run.sh`, inside the existing `if [ "$MODEL" = "iris" ]` block, after the color inject succeeds or warns, run the applier when the file exists. Use `APPLY_IRIS_DYNAMICS` and `IRIS_DYNAMICS_JSON` environment overrides with the container paths as defaults.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest calibration/tests/test_spawn_hook.py -v`

Expected: PASS

---

### Task 6: Offboard velocity stream

**Files:**
- Create: `calibration/offboard_stream.py`
- Create: `calibration/replay_offboard.py`
- Test: `calibration/tests/test_offboard_stream.py`

**Interfaces:**
- Consumes: `VelocitySample` from Task 1
- Produces:
  - `VELOCITY_ONLY_TYPEMASK: int` = `3527` (ignore position, acceleration, yaw, and yaw rate; use `vx vy vz`)
  - `MAV_FRAME_LOCAL_NED: int` = `1`
  - `offboard_bind_port(px4_instance: int) -> int` = `14540 + px4_instance`; `px4_instance < 0` raises `ValueError`
  - `stream_at(samples: list[VelocitySample], hz: float = 50.0) -> list[VelocitySample]` — grid from the first sample time to the last, step `1/hz`, values from `numpy.interp` per axis, same clock, no lag search
  - `replay_offboard.py` connects `udpin:127.0.0.1:<offboard_bind_port>`, waits for a heartbeat, sends `SET_POSITION_TARGET_LOCAL_NED` at 50 Hz with that mask and frame. `--params FILE` is a JSON object of `controller_parameters` output; after the heartbeat and before the stream it `PARAM_SET`s each key. Missing `--params` sends no parameters. Before a series whose first sample is not already a hover, the sender position-holds NED `z = -1.3` until `|z| > 0.4` and the speed is under 0.2 m/s, then starts the series clock. Tests do not import this module.

- [ ] **Step 1: Write the failing test**

```python
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.offboard_stream import (  # noqa: E402
    MAV_FRAME_LOCAL_NED,
    VELOCITY_ONLY_TYPEMASK,
    offboard_bind_port,
    stream_at,
)
from calibration.ulg_velocity import VelocitySample  # noqa: E402

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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_offboard_stream.py -v`

Expected: FAIL with import error for `calibration.offboard_stream`

- [ ] **Step 3: Implement `offboard_stream.py` and the live sender**

The sender arms, switches to offboard, and streams `stream_at` output. Hover to about 1.3 m before a synthetic case is the caller's job: it passes a series that already includes that hover. This task only streams the series it is given. Instance comes from `--instance` (default 0).

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest calibration/tests/test_offboard_stream.py -v`

Expected: PASS

---

### Task 7: SimCon velocity stepper

**Files:**
- Create: `calibration/simcon_velocity.py`
- Test: `calibration/tests/test_simcon_velocity.py`

**Interfaces:**
- Consumes: `VelocitySample`
- Produces:
  - `SIMCON_ROOT: Path` = `/home/valentin/RL/CatSwarm/TrainingRoom/Quadcopter_SimCon`
  - `desired_from_velocity(vx: float, vy: float, vz: float) -> dict` with keys `pos`, `vel`, `yaw_rate`. `pos` is `[nan, nan, nan]`, `vel` is `[vx, vy, vz]`, `yaw_rate` is `0.0`
  - `fly(samples: list[VelocitySample], ts: float = 0.002) -> list[VelocitySample]`
    - inserts `SIMCON_ROOT` / `Simulation` on `sys.path` inside the function
    - steps with `Trajectory.desiredState(..., desired=desired_from_velocity(...))`, then the controller, then `quads.update`, matching the order in `run_3D_simulation.py` `quad_sim_system_manager` but with this `desired` dict
    - initial condition is NED position `z = -1.3` m and zero velocity (the logged hover), then the samples play
    - returns quad 0 NED velocity at each stored sample time
    - does not call `utils.makeFigures` and does not write under `SIMCON_ROOT`

- [ ] **Step 1: Write the failing test**

```python
from pathlib import Path
import math
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.simcon_velocity import SIMCON_ROOT, desired_from_velocity  # noqa: E402

def test_desired_is_pure_velocity_ned():
    desired = desired_from_velocity(1.0, -2.0, 0.25)
    assert math.isnan(desired["pos"][0]) and math.isnan(desired["pos"][1]) and math.isnan(desired["pos"][2])
    assert desired["vel"] == [1.0, -2.0, 0.25]
    assert desired["yaw_rate"] == 0.0
    assert SIMCON_ROOT.name == "Quadcopter_SimCon"
    assert not (ROOT / "TrainingRoom").exists()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_simcon_velocity.py -v`

Expected: FAIL with import error for `calibration.simcon_velocity`

- [ ] **Step 3: Implement `desired_from_velocity` and `fly`**

Read `quad_sim_system_manager` and `run_real_flight_comparison` for the constructor arguments of `QuadcopterSwarm`, `Trajectory`, `Control`, and `Wind`. Use `Wind("None", ...)`. Control type is `XYZ_VEL`. Interpolate `samples` onto the simulator clock with `numpy.interp`. Do not edit SimCon.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest calibration/tests/test_simcon_velocity.py -v`

Expected: PASS. Do not call `fly` in the test.

---

### Task 8: Velocity report

**Files:**
- Create: `calibration/report.py`
- Test: `calibration/tests/test_report.py`

**Interfaces:**
- Consumes: `passes` from `calibration.score`, `IrisDynamics` from `calibration.iris_dynamics`
- Produces:
  - `REPORT_PATH: Path` = `calibration/results/velocity_report.json` relative to the repo root argument
  - `build_report(dynamics: IrisDynamics, log_rows: list[dict], synthetic_rows: list[dict]) -> dict`
    - `log_rows` items have `name: str` and `iris_rms` / `simcon_rms` dicts of `vx vy vz`
    - `synthetic_rows` items have `name: str` and `iris_vs_simcon_rms`
    - top-level keys: `dynamics` (the nine field names), `logs`, `synthetic`, `accepted`
    - `accepted` is `passes` over every `iris_rms` only. SimCon numbers do not affect it
  - `write_report(path: Path, report: dict) -> None`

- [ ] **Step 1: Write the failing test**

```python
from pathlib import Path
import json
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.iris_dynamics import STOCK  # noqa: E402
from calibration.report import build_report, write_report  # noqa: E402

def test_accepted_ignores_simcon(tmp_path: Path):
    good = {"vx": 0.1, "vy": 0.05, "vz": 0.04}
    report = build_report(
        STOCK,
        [{"name": "cat2", "iris_rms": good, "simcon_rms": {"vx": 9.0, "vy": 9.0, "vz": 9.0}}],
        [{"name": "step_vx_p1", "iris_vs_simcon_rms": good}],
    )
    assert report["accepted"] is True
    assert report["dynamics"]["mass_kg"] == 1.5
    bad = dict(good)
    bad["vx"] = 0.21
    report = build_report(STOCK, [{"name": "cat2", "iris_rms": bad, "simcon_rms": good}], [])
    assert report["accepted"] is False
    path = tmp_path / "velocity_report.json"
    write_report(path, report)
    assert json.loads(path.read_text())["accepted"] is False
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_report.py -v`

Expected: FAIL with import error for `calibration.report`

- [ ] **Step 3: Implement `build_report` and `write_report`**

`write_report` creates the parent directory. Pretty-print JSON with sorted keys.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest calibration/tests/test_report.py -v`

Expected: PASS

---

### Task 9: Search policy and live search entry

**Files:**
- Create: `calibration/search.py`
- Create: `calibration/run_search.py`
- Test: `calibration/tests/test_search.py`

**Interfaces:**
- Consumes: `IrisDynamics`, `STOCK`, `select_best`
- Produces:
  - `BOUNDS: dict[str, tuple[float, float]]` — `mass_kg` (0.4, 2.0), `ixx` (0.005, 0.08), `iyy` (0.005, 0.08), `izz` (0.01, 0.12), `motor_constant` (1e-6, 2e-5), `moment_constant` (0.01, 0.2), `time_constant_up` (0.005, 0.08), `time_constant_down` (0.005, 0.08), `rotor_drag_coefficient` (0.0, 0.002)
  - `neighbors(center: IrisDynamics) -> list[IrisDynamics]` — for each field, two copies of `center` with that field at `clamp(center * 0.5)` and `clamp(center * 2)`, skipping a value that clamps back to `center`. Other fields stay at `center`. Order is the field order on `IrisDynamics`, low then high.
  - `candidate_count(center: IrisDynamics) -> int` = `1 + len(neighbors(center))` (the center plus the neighbors)
  - `run_search.py` scores the center and each neighbor on all three logs. For each log it restarts nothing itself: it calls the replay with `--params` set to `controller_parameters` of that log, then `select_best` on the nine RMS numbers. `--write PATH` saves the chosen `IrisDynamics` even when `accepted` is false. Pytest does not start SITL. The SDF is read at spawn, so a new JSON needs a new `runSimNoeticMulti.sh` before the next candidate.

- [ ] **Step 1: Write the failing test**

```python
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.iris_dynamics import STOCK  # noqa: E402
from calibration.search import candidate_count, neighbors  # noqa: E402

def test_stock_neighbors_move_one_field_and_clamp_mass():
    found = neighbors(STOCK)
    assert candidate_count(STOCK) == 1 + len(found)
    masses = sorted({d.mass_kg for d in found})
    assert 0.75 in masses
    assert 2.0 in masses  # 3.0 clamped to the 2.0 bound
    assert 3.0 not in masses
    for d in found:
        changed = [name for name in d.__dataclass_fields__ if getattr(d, name) != getattr(STOCK, name)]
        assert len(changed) == 1
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_search.py -v`

Expected: FAIL with import error for `calibration.search`

- [ ] **Step 3: Implement `neighbors` and the live entry**

Clamp with the bounds table. `run_search.py` prints the nine RMS numbers and `accepted`. It does not change the accept thresholds. It does not import Quadcopter_SimCon.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest calibration/tests/test_search.py -v`

Expected: PASS

---

### Task 10: Side-by-side compare entry

**Files:**
- Create: `calibration/run_compare.py`
- Test: `calibration/tests/test_run_compare.py`

**Interfaces:**
- Consumes: `load_log`, `BASEBALL_LOGS`, `synthetic_cases`, `fly`, `rms_velocity`, `build_report`, `write_report`, `load_dynamics`
- Produces:
  - `simcon_rms_against_log(sim_samples, log) -> dict[str, float]` calling `rms_velocity`
  - `iris_vs_simcon_rms(iris: list, simcon: list) -> dict[str, float]` — interpolate `simcon` onto `iris` timestamps, no `z` mask, no time shift, RMS per axis. Empty input raises `ValueError`
  - `run_compare.py` loads `iris_dynamics.json`, flies each Baseball log's command and each synthetic case through `fly`, reads iris velocities from a directory of CSV files written by the replay (`t_s,vx,vy,vz` header), and writes `calibration/results/velocity_report.json`. A missing iris CSV for a row records that row's iris RMS fields as `null` and forces `accepted` false. Tests do not call `fly`.

- [ ] **Step 1: Write the failing test**

```python
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.run_compare import iris_vs_simcon_rms  # noqa: E402
from calibration.ulg_velocity import VelocitySample  # noqa: E402

def test_compare_rms_is_unshifted():
    iris = [VelocitySample(0.0, 0.0, 0.0, 0.0), VelocitySample(1.0, 1.0, 0.0, 0.0)]
    simcon = [VelocitySample(0.0, 0.0, 0.0, 0.0), VelocitySample(1.0, 1.0, 0.0, 0.0)]
    assert iris_vs_simcon_rms(iris, simcon)["vx"] == pytest.approx(0.0)
    late = [VelocitySample(0.0, 0.0, 0.0, 0.0), VelocitySample(1.0, 0.0, 0.0, 0.0)]
    assert iris_vs_simcon_rms(iris, late)["vx"] > 0.1
    with pytest.raises(ValueError):
        iris_vs_simcon_rms([], simcon)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_run_compare.py -v`

Expected: FAIL with import error for `calibration.run_compare`

- [ ] **Step 3: Implement `iris_vs_simcon_rms` and `run_compare.py`**

CSV columns are exactly `t_s,vx,vy,vz`. SimCon `fly` receives the log command samples (NaN samples dropped) or, for a synthetic case, `stream_at` of that case at 50 Hz from `t=0` to `duration_s`. Do not edit SimCon. Do not let a SimCon RMS change `accepted`.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest calibration/tests/test_run_compare.py calibration/tests -v`

Expected: PASS for the whole `calibration/tests` directory

---

## Live runs (after the unit tests are green)

These are not pytest. Run them only once Tasks 1–10 have passed.

1. `./runSimNoeticMulti.sh --num 1 --world empty`
2. `python3 calibration/run_search.py --write calibration/iris_dynamics.json` against instance 0. Restart the sim when the script says the SDF must be respawned (the JSON is read at spawn).
3. `python3 calibration/run_compare.py` writing `calibration/results/velocity_report.json`.

The proof of calibration is that report: nine iris-versus-log RMS numbers, `accepted`, and the SimCon columns beside them. A SimCon gap stays in the file.
