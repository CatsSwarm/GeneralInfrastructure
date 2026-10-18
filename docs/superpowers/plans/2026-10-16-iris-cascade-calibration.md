# Iris Cascade Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A Python program reads one PX4 ULG and writes the mounted iris jinja and `calibration/iris_controller.json` so the sim's roll/pitch rate, angle, velocity, and position sit as close as a frozen inner-to-outer search can get.

**Architecture:** Pure functions load the log, compute `motorConstant`, patch rotor arms, build the gain file, and score unshifted per-axis RMS. A search keeps a knob only when that step's RMS sum falls. An orchestrator runs the steps in order and takes a `fly` callable. The Gazebo/MAVLink fly is a separate module tests do not launch.

**Tech Stack:** Python 3, pytest, numpy, pyulog, pymavlink; Gazebo Classic iris via `HEADLESS=1 ./runSimNoeticMulti.sh --num 1 --world empty`.

**Spec:** `docs/superpowers/specs/2026-10-16-iris-cascade-calibration-design.md`

## Global Constraints

- Flight file: `calibration/log_2_UnknownDate.ulg`. The program takes the ULG path as its input.
- Order: hover thrust, then motor x/y, then angle gains, then velocity gains, then position gains. A later step does not move an earlier knob.
- Hover window: `vehicle_land_detected.landed` is false and `abs(vehicle_local_position.vz) < 0.1`. Every such sample counts. `h` is the mean of `actuator_motors` `control[0]` through `control[3]`.
- Armed spin is `min(c * 1000 + 100, 1100)` rad/s. One motor's thrust is `motorConstant * spin^2`. `motorConstant = (1.5 * 9.81) / (4 * spin(h)^2)`. Same value on all four plugins. Mass stays 1.5 kg. `maxRotVelocity` stays 1100. `g` is 9.81.
- Do not set full-command thrust to weight / `h`. Thrust at command `h` equals the weight.
- Inertia, `momentConstant`, time constants, rotor drag, rotor height (0.023 m), and rotor quadrant signs stay as they are. `|y|` becomes one length for every rotor.
- Rate copy, then frozen: `MC_ROLLRATE_P`, `MC_ROLLRATE_I`, `MC_ROLLRATE_D`, `MC_ROLLRATE_FF`, `MC_ROLLRATE_K`, and the same `MC_PITCHRATE_*` fields.
- Angle search: `MC_ROLL_P`, `MC_PITCH_P` only. Velocity search: `MPC_XY_VEL_P_ACC`, `MPC_XY_VEL_I_ACC`, `MPC_XY_VEL_D_ACC`, `MPC_Z_VEL_P_ACC`, `MPC_Z_VEL_I_ACC`, `MPC_Z_VEL_D_ACC`. Position search: `MPC_XY_P`, `MPC_Z_P`.
- Yaw parameters stay at the logged values and are not search knobs. `MPC_THR_HOVER` is written as 0.5 and is not a search knob.
- Closeness is the sum of per-axis RMS of `(sim - log)` with no time shift. Each axis is scored only where that axis's command is finite and the sample is airborne. No numeric pass/fail gate.
- Gain file: `calibration/iris_controller.json`. Model file: `calibration/models/iris/iris.sdf.jinja`.
- Sim launch is the existing one-iris empty-world Noetic script. Do not `COPY` the jinja into the image.
- Do not edit Quadcopter_SimCon, the Baseball search gates, or yaw matching.
- **No git commit or push** unless the user explicitly asks.
- Every implementer and reviewer uses model `grok-4.7-high`. Fast models are forbidden.

## Execution

Dispatch implementers in a wave together. They own disjoint files. Dispatch that wave's reviewers together only after every implementer in the wave has returned. A reviewer reads that task's diff, not the whole branch. After Wave 3, one whole-branch reviewer on `grok-4.7-high`.

| Wave | Tasks | Why together |
|------|--------|----------------|
| 1 | 1, 2, 3, 4, 5 | Disjoint files. None imports another task's module |
| 2 | 6, 7 | Task 6 owns the flight type. Task 7 owns the keep/search functions and imports nothing from Task 6 |
| 3 | 8, 9 | Task 8 imports Tasks 1–7. Task 9 imports Task 6 and does not import Task 8 |
| 4 | 10 | Imports `run_cascade`, `setpoints`, and `sitl_command` |

The parent session coordinates. It does not write the production code. Whole-branch review runs after Wave 4.

## Review Focus

These are the failures the spec allows if a task forgets them. Each has a test in the owning task.

- `motorConstant` from `h = 0.34` must use spin 440, not `0.34 * 1100` and not full-command thrust = weight / `h` (Task 2).
- A trace that matches the log only after a 0.2 s shift must still score the unshifted error (Task 5).
- A position sample with finite `z` and non-finite `x` still scores altitude and does not score `x` (Task 5).
- An outer step must not change `motorConstant` or an arm length already accepted (Task 8).
- A non-finite position command must stay out of the setpoint fields as a used axis; it must not be sent as 0 (Task 9).

## File structure

| Path | Responsibility |
|------|----------------|
| `calibration/cascade_hover.py` | Hover window and `h` |
| `calibration/cascade_thrust.py` | `motor_constant(h)` |
| `calibration/cascade_model.py` | Write `motorConstant` and the two arm lengths into the host jinja |
| `calibration/cascade_gains.py` | Logged gain file, search keys, refusals |
| `calibration/cascade_score.py` | Unshifted per-axis RMS sum |
| `calibration/cascade_flight.py` | Load every cascade series from one ULG |
| `calibration/cascade_search.py` | Keep rule and ±10% one-knob sweeps |
| `calibration/run_cascade.py` | Step order with an injected `fly` |
| `calibration/cascade_sitl.py` | Setpoint records and the SITL command. Live `fly_log` is not what pytest calls |
| `calibration/tests/test_cascade_*.py` | Unit tests. No Gazebo process |

---

### Task 1: Hover command

**Files:**
- Create: `calibration/cascade_hover.py`
- Test: `calibration/tests/test_cascade_hover.py`

**Interfaces:**
- Consumes: nothing
- Produces:
  - `HOVER_ABS_VZ_MPS: float` = `0.1`
  - `hover_mean(landed: list[bool], vz: list[float], motors: list[tuple[float, float, float, float]]) -> float`
  - `hover_command(path: Path) -> float`

`hover_mean` uses samples where `landed` is false and `abs(vz) < 0.1`. `h` is the mean of the four motor numbers on those samples. `|vz| == 0.1` is outside the window. `hover_command` aligns `actuator_motors` onto `vehicle_local_position` timestamps with linear interpolation of `control[0..3]`, and aligns `vehicle_land_detected.landed` by the nearest sample in time. Do not drop the file because `ulog_info` reports corruption.

- [ ] **Step 1: Write the failing test**

```python
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.cascade_hover import hover_command, hover_mean  # noqa: E402

def test_hover_mean_keeps_flat_airborne_samples_only():
    landed = [False, False, False, True]
    vz = [0.0, 0.1, 0.05, 0.0]
    motors = [(0.2, 0.2, 0.2, 0.2), (0.9, 0.9, 0.9, 0.9), (0.4, 0.4, 0.4, 0.4), (0.1, 0.1, 0.1, 0.1)]
    assert hover_mean(landed, vz, motors) == pytest.approx(0.3)

def test_log2_hover_command_is_about_0_34():
    h = hover_command(ROOT / "calibration" / "log_2_UnknownDate.ulg")
    assert h == pytest.approx(0.344, abs=0.01)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_cascade_hover.py -v` from `general_infrastructure/`

Expected: FAIL with `ModuleNotFoundError` or `ImportError` for `calibration.cascade_hover`

- [ ] **Step 3: Implement `hover_mean` and `hover_command` in `calibration/cascade_hover.py`**

`hover_mean` averages the four controls on the mask above. Equal weights. `hover_command` reads the ULG with pyulog.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest calibration/tests/test_cascade_hover.py -v`

Expected: PASS

- [ ] **Step 5: Do not commit**

The global constraint forbids commits unless the user asks.

---

### Task 2: motorConstant at the hover command

**Files:**
- Create: `calibration/cascade_thrust.py`
- Test: `calibration/tests/test_cascade_thrust.py`

**Interfaces:**
- Consumes: nothing
- Produces:
  - `MASS_KG: float` = `1.5`
  - `G_MPS2: float` = `9.81`
  - `MAX_SPIN_RAD_S: float` = `1100.0`
  - `def spin_rad_s(command: float) -> float`
  - `def motor_constant(hover_command: float) -> float`

`spin_rad_s(c) = min(c * 1000 + 100, 1100)`. `motor_constant` uses four motors at `spin_rad_s(hover_command)`.

- [ ] **Step 1: Write the failing test**

```python
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.cascade_thrust import motor_constant, spin_rad_s  # noqa: E402

def test_hover_spin_is_affine_and_clamped():
    assert spin_rad_s(0.34) == pytest.approx(440.0)
    assert spin_rad_s(1.0) == pytest.approx(1100.0)
    assert spin_rad_s(2.0) == pytest.approx(1100.0)

def test_motor_constant_holds_weight_at_command_h():
    # spin 440, not 0.34 * 1100, and not thrust at spin 1100 = weight / 0.34
    assert motor_constant(0.34) == pytest.approx((1.5 * 9.81) / (4 * 440.0**2))
    assert motor_constant(0.34) != pytest.approx((1.5 * 9.81) / (0.34 * 4 * 1100.0**2))
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_cascade_thrust.py -v`

Expected: FAIL with `ImportError` for `calibration.cascade_thrust`

- [ ] **Step 3: Implement `spin_rad_s` and `motor_constant` in `calibration/cascade_thrust.py`**

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest calibration/tests/test_cascade_thrust.py -v`

Expected: PASS

- [ ] **Step 5: Do not commit**

---

### Task 3: Write motorConstant and arm lengths

**Files:**
- Create: `calibration/cascade_model.py`
- Test: `calibration/tests/test_cascade_model.py`

**Interfaces:**
- Consumes: nothing
- Produces:
  - `ROTOR_SIGNS: tuple[tuple[str, int, int], ...]` = `(("rotor_0", 1, -1), ("rotor_1", -1, 1), ("rotor_2", 1, 1), ("rotor_3", -1, -1))`
  - `ROTOR_Z_M: float` = `0.023`
  - `def read_arms(model_path: Path) -> tuple[float, float]` — `|x|` of `rotor_0`, mean of the four `|y|`
  - `def write_model(model_path: Path, motor_constant: float, arm_x: float, arm_y: float) -> None`

`write_model` sets every `motorConstant` to the same value and each rotor link pose to `sign_x * arm_x`, `sign_y * arm_y`, `0.023`. It does not change `<mass>`, `<ixx>`, `<momentConstant>`, `<timeConstantUp>`, `<timeConstantDown>`, `<rotorDragCoefficient>`, or `<maxRotVelocity>`.

- [ ] **Step 1: Write the failing test**

```python
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.cascade_model import read_arms, write_model  # noqa: E402

SDF = """
<sdf>
  <link name='base_link'><inertial><mass>1.5</mass><inertia><ixx>0.029125</ixx></inertia></inertial></link>
  <link name='rotor_0'><pose>0.13 -0.22 0.023 0 0 0</pose></link>
  <link name='rotor_1'><pose>-0.13 0.20 0.023 0 0 0</pose></link>
  <link name='rotor_2'><pose>0.13 0.22 0.023 0 0 0</pose></link>
  <link name='rotor_3'><pose>-0.13 -0.20 0.023 0 0 0</pose></link>
  <plugin filename='libgazebo_motor_model.so'><motorConstant>2.92e-06</motorConstant><momentConstant>0.06</momentConstant><maxRotVelocity>1100</maxRotVelocity></plugin>
  <plugin filename='libgazebo_motor_model.so'><motorConstant>2.92e-06</motorConstant><momentConstant>0.06</momentConstant><maxRotVelocity>1100</maxRotVelocity></plugin>
</sdf>
"""

def test_write_model_sets_thrust_and_shared_arms(tmp_path: Path):
    path = tmp_path / "iris.sdf.jinja"
    path.write_text(SDF, encoding="utf-8")
    assert read_arms(path) == pytest.approx((0.13, 0.21))
    write_model(path, 1.9e-05, 0.2, 0.3)
    text = path.read_text(encoding="utf-8")
    assert "<mass>1.5</mass>" in text
    assert "<momentConstant>0.06</momentConstant>" in text
    assert "<maxRotVelocity>1100</maxRotVelocity>" in text
    assert text.count("<motorConstant>1.9e-05</motorConstant>") == 2
    assert "<pose>0.2 -0.3 0.023 0 0 0</pose>" in text
    assert "<pose>-0.2 0.3 0.023 0 0 0</pose>" in text
    assert "<pose>0.2 0.3 0.023 0 0 0</pose>" in text
    assert "<pose>-0.2 -0.3 0.023 0 0 0</pose>" in text
```

`_format` the written `motorConstant` with Python `format(value, ".16g")` so `1.9e-05` stays stable. The test's count assertion must use whatever `.16g` emits for `1.9e-05`. If that is `1.9e-05`, keep the assertion above.

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_cascade_model.py -v`

Expected: FAIL with `ImportError` for `calibration.cascade_model`

- [ ] **Step 3: Implement `read_arms` and `write_model` in `calibration/cascade_model.py`**

Pose regex is the first `<pose>` inside each `rotor_N` link. Replace every `<motorConstant>` in `libgazebo_motor_model.so` plugins only.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest calibration/tests/test_cascade_model.py -v`

Expected: PASS

- [ ] **Step 5: Do not commit**

---

### Task 4: Controller gain file

**Files:**
- Create: `calibration/cascade_gains.py`
- Test: `calibration/tests/test_cascade_gains.py`

**Interfaces:**
- Consumes: nothing
- Produces:
  - `RATE_KEYS`, `ANGLE_KEYS`, `VELOCITY_KEYS`, `POSITION_KEYS: tuple[str, ...]` with the names in Global Constraints, in that order
  - `def search_keys(step: str) -> tuple[str, ...]` — `"rate"` returns `()`, `"angle"` / `"velocity"` / `"position"` return those tuples. Any other step raises `ValueError`
  - `def gain_file(initial: dict[str, float], overrides: dict[str, float] | None = None) -> dict[str, float]`
  - `def save_gains(path: Path, gains: dict[str, float]) -> None`

`gain_file` copies every initial key matching `^(MPC_|MC_)`. It then applies `overrides` only when every key is in `ANGLE_KEYS + VELOCITY_KEYS + POSITION_KEYS`. `MC_YAW_P` or `MC_YAWRATE_P` in `overrides` raises `ValueError`. `MPC_THR_HOVER` in the result is `0.5` even when `initial` or `overrides` says otherwise. `overrides` that set `MPC_THR_HOVER` raise `ValueError`. Rate keys are taken from `initial` and are not overridable. `save_gains` writes JSON with indent 2 and a trailing newline.

- [ ] **Step 1: Write the failing test**

```python
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.cascade_gains import (  # noqa: E402
    ANGLE_KEYS,
    gain_file,
    save_gains,
    search_keys,
)

def test_search_keys_and_hover_restore_and_yaw_refusal():
    assert search_keys("rate") == ()
    assert search_keys("angle") == ANGLE_KEYS
    initial = {
        "MC_ROLLRATE_P": 0.08,
        "MC_ROLLRATE_FF": 0.0,
        "MC_ROLLRATE_K": 0.55,
        "MC_ROLL_P": 12.0,
        "MC_YAW_P": 2.0,
        "MPC_THR_HOVER": 0.6,
        "EKF2_GPS_CTRL": 7.0,
    }
    gained = gain_file(initial, {"MC_ROLL_P": 10.0})
    assert gained["MC_ROLL_P"] == pytest.approx(10.0)
    assert gained["MC_ROLLRATE_FF"] == pytest.approx(0.0)
    assert gained["MC_ROLLRATE_K"] == pytest.approx(0.55)
    assert gained["MPC_THR_HOVER"] == pytest.approx(0.5)
    assert "EKF2_GPS_CTRL" not in gained
    with pytest.raises(ValueError):
        gain_file(initial, {"MC_YAW_P": 4.0})
    with pytest.raises(ValueError):
        gain_file(initial, {"MPC_THR_HOVER": 0.4})

def test_save_gains_roundtrip(tmp_path: Path):
    path = tmp_path / "iris_controller.json"
    save_gains(path, {"MC_ROLL_P": 12.0, "MPC_THR_HOVER": 0.5})
    assert json.loads(path.read_text(encoding="utf-8"))["MC_ROLL_P"] == pytest.approx(12.0)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_cascade_gains.py -v`

Expected: FAIL with `ImportError` for `calibration.cascade_gains`

- [ ] **Step 3: Implement the gain helpers in `calibration/cascade_gains.py`**

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest calibration/tests/test_cascade_gains.py -v`

Expected: PASS

- [ ] **Step 5: Do not commit**

---

### Task 5: Per-axis RMS sum

**Files:**
- Create: `calibration/cascade_score.py`
- Test: `calibration/tests/test_cascade_score.py`

**Interfaces:**
- Consumes: nothing
- Produces:
  - `def axis_rms(t_log: list[float], command: list[float], response: list[float], airborne: list[bool], t_sim: list[float], sim: list[float]) -> float`
  - `def rms_sum(values: list[float]) -> float`

Interpolate `sim` onto `t_log` with `numpy.interp`. Do not shift `t_sim`. A log sample counts when `airborne` is true and `command` is finite. Non-finite `command` values are not interpolated into numbers; that sample is out. If a step passes a list of axis RMS values, `rms_sum` is their sum. `axis_rms` raises `ValueError` when the mask keeps nothing. There is no accept threshold and no shift argument.

- [ ] **Step 1: Write the failing test**

```python
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.cascade_score import axis_rms, rms_sum  # noqa: E402

def test_unshifted_rms_keeps_a_0_2_s_lag():
    t_log = [0.0, 1.0, 2.0]
    response = [0.0, 1.0, 1.0]
    airborne = [True, True, True]
    command = [0.0, 1.0, 1.0]
    late = axis_rms(t_log, command, response, airborne, [0.2, 1.2, 2.2], [0.0, 1.0, 1.0])
    aligned = axis_rms(t_log, command, response, airborne, t_log, response)
    assert aligned == pytest.approx(0.0)
    assert late > 0.2

def test_nonfinite_command_drops_that_axis_only():
    z = axis_rms([0.0, 1.0], [1.0, 1.0], [1.0, 1.0], [True, True], [0.0, 1.0], [1.0, 1.0])
    assert z == pytest.approx(0.0)
    with pytest.raises(ValueError):
        axis_rms([0.0], [math.nan], [5.0], [True], [0.0], [5.0])
    assert rms_sum([z, 0.25]) == pytest.approx(0.25)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_cascade_score.py -v`

Expected: FAIL with `ImportError` for `calibration.cascade_score`

- [ ] **Step 3: Implement `axis_rms` and `rms_sum` in `calibration/cascade_score.py`**

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest calibration/tests/test_cascade_score.py -v`

Expected: PASS

- [ ] **Step 5: Do not commit**

---

### Task 6: Flight series

**Files:**
- Create: `calibration/cascade_flight.py`
- Test: `calibration/tests/test_cascade_flight.py`

**Interfaces:**
- Consumes: `hover_command` from `calibration.cascade_hover`
- Produces:
  - `@dataclass(frozen=True) class AxisLog` with `t_s: tuple[float, ...]`, `command: tuple[float, ...]`, `response: tuple[float, ...]`, `airborne: tuple[bool, ...]` — command, response, and airborne share `t_s`
  - `@dataclass(frozen=True) class CascadeFlight` with `path: str`, `hover_command: float`, `initial_parameters: dict[str, float]`, and axis fields `roll_rate`, `pitch_rate`, `roll`, `pitch`, `vx`, `vy`, `vz`, `x`, `y`, `z: AxisLog`, plus `thrust_z: AxisLog` whose response is `vehicle_rates_setpoint.thrust_body[2]` aligned onto the rate timestamps (command equals response; it is the thrust sent with rate and angle targets)
  - `def roll_pitch_rad(q: tuple[float, float, float, float]) -> tuple[float, float]`
  - `def load_flight(path: Path) -> CascadeFlight`

`q` is PX4 `q[0..3]` as w, x, y, z.

`roll = atan2(2(w x + y z), 1 - 2(x^2 + y^2))`, `pitch = asin(clamp(2(w y - z x), -1, 1))`.

Rate command is `vehicle_rates_setpoint` `roll` / `pitch`. Rate response is `vehicle_angular_velocity` `xyz[0]` / `xyz[1]`. Angle command is `roll_body` / `pitch_body`. Angle response is `roll_pitch_rad` of `vehicle_attitude.q`. Velocity and position use `vehicle_local_position_setpoint` and `vehicle_local_position`. Airborne is nearest `landed == false`. Align a command onto response time by the nearest command sample. If that sample is non-finite, store NaN. Do not linearly interpolate across NaN.

- [ ] **Step 1: Write the failing test**

```python
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.cascade_flight import load_flight, roll_pitch_rad  # noqa: E402

def test_roll_pitch_identity_and_quarter_roll():
    assert roll_pitch_rad((1.0, 0.0, 0.0, 0.0)) == pytest.approx((0.0, 0.0))
    half = math.sqrt(2) / 2
    assert roll_pitch_rad((half, half, 0.0, 0.0)) == pytest.approx((math.pi / 2, 0.0))

def test_log2_flight_has_rate_and_partial_position():
    flight = load_flight(ROOT / "calibration" / "log_2_UnknownDate.ulg")
    assert flight.hover_command == pytest.approx(0.344, abs=0.01)
    assert len(flight.roll_rate.response) > 1000
    assert flight.initial_parameters["MC_ROLL_P"] == pytest.approx(12.0)
    assert math.isnan(flight.x.command[0]) or any(math.isnan(v) for v in flight.x.command)
    assert any(math.isfinite(v) for v in flight.z.command)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_cascade_flight.py -v`

Expected: FAIL with `ImportError` for `calibration.cascade_flight`

- [ ] **Step 3: Implement `roll_pitch_rad` and `load_flight` in `calibration/cascade_flight.py`**

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest calibration/tests/test_cascade_flight.py -v`

Expected: PASS

- [ ] **Step 5: Do not commit**

---

### Task 7: Keep a closer candidate

**Files:**
- Create: `calibration/cascade_search.py`
- Test: `calibration/tests/test_cascade_search.py`

**Interfaces:**
- Consumes: nothing
- Produces:
  - `FACTORS: tuple[float, float]` = `(1.1, 0.9)`
  - `def keep(current: float, proposed: float) -> bool` — true only when `proposed < current`
  - `def search_knobs(start: dict[str, float], evaluate) -> tuple[dict[str, float], float]`

`evaluate(knobs: dict[str, float]) -> float` returns the step RMS sum. `search_knobs` evaluates `start` first. Then, one knob at a time in insertion order, it tries `value * 1.1` and `value * 0.9`. It keeps the try only when `keep` is true, and if both improve it keeps the lower RMS. It repeats full rounds until a round keeps nothing. It does not shrink `FACTORS` and it does not add knobs.

- [ ] **Step 1: Write the failing test**

```python
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.cascade_search import keep, search_knobs  # noqa: E402

def test_keep_requires_a_strict_drop():
    assert keep(1.0, 0.9) is True
    assert keep(1.0, 1.0) is False
    assert keep(1.0, 1.1) is False

def test_search_moves_only_the_knob_that_gets_closer():
    def evaluate(knobs: dict[str, float]) -> float:
        return abs(knobs["arm_x"] - 0.13 * 0.9) + abs(knobs["arm_y"] - 0.21)

    best, score = search_knobs({"arm_x": 0.13, "arm_y": 0.21}, evaluate)
    assert best["arm_y"] == pytest.approx(0.21)
    assert best["arm_x"] == pytest.approx(0.13 * 0.9)
    assert score == pytest.approx(0.0)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_cascade_search.py -v`

Expected: FAIL with `ImportError` for `calibration.cascade_search`

- [ ] **Step 3: Implement `keep` and `search_knobs` in `calibration/cascade_search.py`**

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest calibration/tests/test_cascade_search.py -v`

Expected: PASS

- [ ] **Step 5: Do not commit**

---

### Task 8: Cascade order

**Files:**
- Create: `calibration/run_cascade.py`
- Test: `calibration/tests/test_run_cascade.py`

**Interfaces:**
- Consumes: `hover_command`, `motor_constant`, `read_arms`, `write_model`, `gain_file`, `save_gains`, `search_keys`, `load_flight`, `AxisLog`, `axis_rms`, `rms_sum`, `search_knobs`
- Produces:
  - `def run_cascade(ulg: Path, model: Path, gains_path: Path, fly, report_path: Path) -> dict[str, float]`
  - `fly(step: str, flight, model: Path, gains_path: Path) -> dict[str, tuple[list[float], list[float]]]` — keys are axis names `roll_rate`, `pitch_rate`, `roll`, `pitch`, `vx`, `vy`, `vz`, `x`, `y`, `z`. Each value is `(t_sim, sim)`

Order inside `run_cascade`: compute `h` and write `motor_constant(h)` with the arms from `read_arms`; save `gain_file(flight.initial_parameters)` with no overrides; search `arm_x` and `arm_y`; search angle keys; search velocity keys; search position keys. Rate gains are not knobs. After the arm search, further `write_model` calls use the accepted arms and the same `motor_constant`. Gain saves after each accepted gain knob use `gain_file(initial, overrides)` where `overrides` holds only accepted search keys. `fly` is called only after the hover write. The report JSON maps `rate`, `angle`, `velocity`, `position` to that step's final RMS sum.

Score axes: rate uses `roll_rate` and `pitch_rate`; angle uses `roll` and `pitch`; velocity uses `vx`, `vy`, `vz`; position uses `x`, `y`, `z`. An axis whose `axis_rms` raises because every command is non-finite is omitted from that step's sum. If every axis raises, let the error propagate.

- [ ] **Step 1: Write the failing test**

```python
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.run_cascade import run_cascade  # noqa: E402

def test_outer_step_does_not_move_motor_constant_or_arms(tmp_path: Path):
    model = tmp_path / "iris.sdf.jinja"
    model.write_text((ROOT / "calibration/models/iris/iris.sdf.jinja").read_text(encoding="utf-8"), encoding="utf-8")
    seen: list[str] = []
    first_motors: list[str] = []

    def fly(step, flight, model_path, gains_path):
        seen.append(step)
        if step == "rate" and len(seen) == 1:
            first_motors.append(model_path.read_text(encoding="utf-8"))
        return {name: ([0.0], [0.0]) for name in (
            "roll_rate", "pitch_rate", "roll", "pitch", "vx", "vy", "vz", "x", "y", "z")}

    report = run_cascade(
        ROOT / "calibration" / "log_2_UnknownDate.ulg",
        model,
        tmp_path / "iris_controller.json",
        fly,
        tmp_path / "cascade_report.json",
    )
    assert seen[0] == "rate"
    assert seen.index("angle") > seen.index("rate")
    assert seen.index("velocity") > seen.index("angle")
    assert seen.index("position") > seen.index("velocity")
    text = model.read_text(encoding="utf-8")
    assert "<mass>1.5</mass>" in text
    assert first_motors[0].count("<motorConstant>") == text.count("<motorConstant>")
    assert first_motors[0][first_motors[0].find("<motorConstant>"):first_motors[0].find("</motorConstant>")] in text
    saved = json.loads((tmp_path / "cascade_report.json").read_text(encoding="utf-8"))
    assert set(saved) == {"rate", "angle", "velocity", "position"}
    assert report["position"] == saved["position"]
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_run_cascade.py -v`

Expected: FAIL with `ImportError` for `calibration.run_cascade`

- [ ] **Step 3: Implement `run_cascade` in `calibration/run_cascade.py`**

Arm knobs are not gain-file keys. Pass them through `write_model`. Gain knobs start at `flight.initial_parameters[key]` and go through `search_knobs` and `gain_file`.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest calibration/tests/test_run_cascade.py calibration/tests/test_cascade_hover.py calibration/tests/test_cascade_thrust.py calibration/tests/test_cascade_model.py calibration/tests/test_cascade_gains.py calibration/tests/test_cascade_score.py calibration/tests/test_cascade_flight.py calibration/tests/test_cascade_search.py -q`

Expected: PASS

- [ ] **Step 5: Do not commit**

---

### Task 9: SITL setpoints

**Files:**
- Create: `calibration/cascade_sitl.py`
- Test: `calibration/tests/test_cascade_sitl.py`

**Interfaces:**
- Consumes: `CascadeFlight` and `AxisLog` from `calibration.cascade_flight`
- Produces:
  - `@dataclass(frozen=True) class Setpoint` with `step: str`, `t_s: float`, `fields: dict[str, float]`, `ignore: frozenset[str]`
  - `def sitl_command(script: Path) -> list[str]`
  - `def setpoints(step: str, flight) -> list[Setpoint]`

`sitl_command` returns `["env", "HEADLESS=1", str(script), "--num", "1", "--world", "empty"]`.

`setpoints("rate", flight)` emits one record per `roll_rate` sample that is airborne and has finite roll and pitch rate commands. `fields` holds `roll_rate`, `pitch_rate`, and `thrust_z` from `flight.thrust_z` nearest in time. `ignore` is `frozenset({"attitude", "yaw_rate"})`.

`setpoints("angle", flight)` uses `roll` / `pitch` commands plus nearest `thrust_z`. `ignore` is `frozenset({"rates"})`. Include `yaw` from `vehicle_attitude_setpoint.yaw_body` aligned onto the angle timestamps as `fields["yaw"]`. Yaw is sent and not scored. Add `yaw` on the flight in this task if Task 6 has no yaw field: extend `CascadeFlight` with `yaw: AxisLog` whose response is unused (`response` may equal `command`) and whose command is `yaw_body`. Update Task 6's loader. The yaw field is not a search axis and `run_cascade` does not score it.

`setpoints("velocity", flight)` uses `vx`, `vy`, `vz` on the velocity timestamps (use `vx.t_s`). `ignore` is `frozenset({"x", "y", "z"})` plus any of `vx`/`vy`/`vz` whose command is non-finite. Finite velocity axes are in `fields`.

`setpoints("position", flight)` uses `x.t_s`. A non-finite `x`, `y`, or `z` is named in `ignore` and is absent from `fields`. It is never written as `0`. Finite axes are in `fields`. `yaw` is included when finite.

Do not open a socket, start Docker, or import a live `fly_log` into the test module's assertions. `fly_log` may exist in this file for a later manual run; pytest must not call it. No test calls `fly_log`.

- [ ] **Step 1: Write the failing test**

```python
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.cascade_flight import load_flight  # noqa: E402
from calibration.cascade_sitl import setpoints, sitl_command  # noqa: E402

def test_sitl_command_is_headless_one_iris():
    assert sitl_command(Path("runSimNoeticMulti.sh")) == [
        "env", "HEADLESS=1", "runSimNoeticMulti.sh", "--num", "1", "--world", "empty"
    ]

def test_position_nan_is_ignored_not_zero():
    flight = load_flight(ROOT / "calibration" / "log_2_UnknownDate.ulg")
    records = setpoints("position", flight)
    assert records
    partial = next(item for item in records if "x" in item.ignore or "y" in item.ignore)
    assert "x" not in partial.fields or math.isfinite(partial.fields["x"])
    assert 0.0 not in [partial.fields.get(name) for name in partial.ignore]
    assert "z" in partial.fields or "z" in partial.ignore
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_cascade_sitl.py -v`

Expected: FAIL with `ImportError` for `calibration.cascade_sitl`

- [ ] **Step 3: Implement `sitl_command` and `setpoints` in `calibration/cascade_sitl.py`**

Extend `CascadeFlight` with `yaw` in `calibration/cascade_flight.py` as specified above. Keep Task 6's tests passing.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest calibration/tests/test_cascade_sitl.py calibration/tests/test_cascade_flight.py calibration/tests/test_run_cascade.py -q`

Expected: PASS

- [ ] **Step 5: Do not commit**

---

### Task 10: Program entry

**Files:**
- Modify: `calibration/run_cascade.py`
- Modify: `calibration/cascade_sitl.py`
- Test: `calibration/tests/test_cascade_main.py`

**Interfaces:**
- Consumes: `run_cascade`, `sitl_command`, `setpoints`, `load_flight`, `save_gains` path from the caller
- Produces:
  - `def fly_log(step: str, flight, model: Path, gains_path: Path, *, runner, connect) -> dict[str, tuple[list[float], list[float]]]`
  - `def main(argv: list[str] | None = None) -> int`

`main` with no arguments uses `calibration/log_2_UnknownDate.ulg`, `calibration/models/iris/iris.sdf.jinja`, `calibration/iris_controller.json`, and `calibration/results/cascade_report.json`. `main` calls `run_cascade` with `fly_log`.

`fly_log` calls `runner(sitl_command(Path("runSimNoeticMulti.sh")))` and `connect("udpin:0.0.0.0:14541")`. It loads the gain file and sends those parameters before setpoints. It streams `setpoints(step, flight)` and returns sim series keyed the way `run_cascade`'s `fly` requires. `runner` and `connect` are required keyword arguments so tests never start Docker or a socket. The real adapters live in `main` only: `runner` is `subprocess.Popen`, `connect` is a pymavlink `udpin` on port 14541 (first vehicle is `px4 -i 1`).

The test does not parse MAVLink frames. It uses a fake `runner` and a fake `connect` that record the command and the port, and a master whose `recv_match` returns one local-position sample and then `None`. `fly_log` may return short series from that master. Assert the recorded command equals `sitl_command(...)` and the recorded port is `14541`.

- [ ] **Step 1: Write the failing test**

```python
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.cascade_flight import load_flight  # noqa: E402
from calibration.cascade_sitl import fly_log, sitl_command  # noqa: E402

class _Proc:
    def terminate(self):
        return None

class _Master:
    def recv_match(self, **kwargs):
        return None

def test_fly_log_uses_headless_instance_one(tmp_path: Path):
    commands: list[list[str]] = []
    ports: list[str] = []

    def runner(command):
        commands.append(list(command))
        return _Proc()

    def connect(port):
        ports.append(port)
        return _Master()

    model = tmp_path / "iris.sdf.jinja"
    model.write_text("<sdf></sdf>", encoding="utf-8")
    gains = tmp_path / "iris_controller.json"
    gains.write_text('{"MPC_THR_HOVER": 0.5, "MC_ROLL_P": 12.0}\n', encoding="utf-8")
    flight = load_flight(ROOT / "calibration" / "log_2_UnknownDate.ulg")
    fly_log("rate", flight, model, gains, runner=runner, connect=connect)
    assert commands == [sitl_command(Path("runSimNoeticMulti.sh"))]
    assert ports == ["udpin:0.0.0.0:14541"]
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_cascade_main.py -v`

Expected: FAIL with `ImportError` for `fly_log`

- [ ] **Step 3: Implement `fly_log` and `main`**

`fly_log` lives in `calibration/cascade_sitl.py`. `main` lives in `calibration/run_cascade.py` and passes `fly_log` into `run_cascade`. Do not call `main` from the test.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest calibration/tests -q`

Expected: PASS, including the existing calibration tests.

- [ ] **Step 5: Do not commit**

---

## Self-review notes

- Hover window, spin map, arm signs, gain names, RMS rule, step order, SITL command, and the instance-1 port are pinned in tasks.
- `fly_log` is invoked with injected `runner` and `connect`. Pytest does not start Gazebo. A real calibration is `python3 -m calibration.run_cascade` with the default paths, and its proof is `calibration/results/cascade_report.json`.
