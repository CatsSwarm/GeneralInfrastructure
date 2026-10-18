# Iris cascade calibration

One program reads a PX4 ULG and writes the iris model that the Noetic sim bind-mounts, plus the controller gains that replay uses. It fits the stack from the inside out so roll and pitch rate, angle, velocity, and position stay close to that flight. There is no numeric pass gate. Closeness is the unshifted per-axis RMS of `(sim - log)`.

## What it changes

`python3 -m calibration.run_cascade` does this, in order:

1. Hover thrust. `h` is the mean `actuator_motors` command while the log is airborne and `|vz| < 0.1`. `motorConstant` is set so four motors at `spin(h) = min(h*1000+100, 1100)` rad/s hold 1.5 kg.
2. Arms. One `|x|` and one `|y|` length, scored on roll and pitch rate.
3. Angle gains (`MC_ROLL_P`, `MC_PITCH_P`).
4. Velocity gains (`MPC_*_VEL_*_ACC`).
5. Position gains (`MPC_XY_P`, `MPC_Z_P`).

Rate gains, yaw, mass, and inertia stay at the values from the log and the model. `MPC_THR_HOVER` is forced to 0.5. Each search tries `×1.1` and `×0.9` on one knob at a time and keeps a trial only when the RMS sum strictly falls. It stops when a full round keeps nothing.

## Live replay

Each trial starts a headless one-iris empty world:

```bash
HEADLESS=1 ./runSimNoeticMulti.sh --num 1 --world empty
```

The first vehicle is PX4 instance 1, link `udpin:0.0.0.0:14541`. `runSimNoeticMulti.sh` bind-mounts `models/iris/iris.sdf.jinja` read-only over the in-image iris jinja. Integer `MC_`/`MPC_` parameters go out as INT32. The scored clock starts when the vehicle is armed in offboard. Setpoints stream at 50 Hz. Pytest does not start Gazebo.

## Run

From the repository root:

```bash
python3 -m calibration.run_cascade
python3 -m calibration.run_cascade path/to/flight.ulg
python3 -m pytest calibration/tests -q
```

No argument uses `log_2_UnknownDate.ulg`. One extra argument is a usage error. The program returns 0 when the search finishes.

| Output | Role |
| --- | --- |
| `models/iris/iris.sdf.jinja` | `motorConstant` and the two arm lengths |
| `iris_controller.json` | logged `MC_`/`MPC_` gains, with the accepted outer overrides |
| `results/cascade_report.json` | RMS sum for `rate`, `angle`, `velocity`, `position` |

## Files

| File | Role |
| --- | --- |
| `run_cascade.py` | Order, defaults, and the program entry |
| `cascade_hover.py` | Hover command `h` |
| `cascade_thrust.py` | Spin map and `motorConstant` |
| `cascade_model.py` | Read and write the iris jinja |
| `cascade_gains.py` | Gain file; rate stays frozen |
| `cascade_flight.py` | Per-axis command and response from the ULG |
| `cascade_score.py` | Unshifted per-axis RMS |
| `cascade_search.py` | One-knob ±10% search |
| `cascade_sitl.py` | Headless replay and the 50 Hz stream |
| `log_2_UnknownDate.ulg` | Default flight |
| `tests/test_cascade_*.py`, `tests/test_run_cascade.py` | Unit tests; no Docker, no sockets |
