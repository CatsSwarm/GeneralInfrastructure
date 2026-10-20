# Iris cascade calibration

Four programs read a PX4 ULG and write the iris model that the Noetic sim bind-mounts, plus the controller gains that replay uses. They fit the stack from the inside out so roll and pitch rate, angle, velocity, and position stay close to that flight. There is no numeric pass gate. Closeness is the unshifted per-axis RMS of `(sim - log)`.

## What it changes

From the repository root, in order:

```bash
python3 -m calibration.tune_rate
python3 -m calibration.tune_angle
python3 -m calibration.tune_velocity
python3 -m calibration.tune_position
```

`tune_rate` freezes rotor x/y, prop radius, and `maxRotVelocity`, fits hover `motorConstant`, then searches `timeConstantUp` (`timeConstantDown` keeps the original ratio) and `MC_ROLLRATE_P` / `MC_PITCHRATE_P`.

Each later command refuses to fly until `calibration/results/cascade/stages/<previous>/best.json` exists. `tune_angle` searches `MC_ROLL_P` and `MC_PITCH_P`. `tune_velocity` searches the six `MPC_*_VEL_*_ACC` gains. `tune_position` searches `MPC_XY_P` and `MPC_Z_P`. Angle, velocity, and position scores include roll-rate and pitch-rate RMS.

Factors are `0.7`, `0.85`, `1.15`, and `1.3`, once per knob. A knob is kept only when the RMS sum strictly falls. Yaw gains, rate I/D/FF/K, mass, and inertia are not knobs. `MPC_THR_HOVER` is forced to 0.5.

`python3 -m calibration.run_cascade` prints those four commands and returns 2. It does not fly.

## Live replay

Each trial starts one iris on an empty world. `SHOW_GAZEBO` is in `run_cascade.py`. The tune programs use that flag.

Headless launch:

```bash
HEADLESS=1 ./runSimNoeticMulti.sh --num 1 --world empty
```

The first vehicle is PX4 instance 1, link `udpin:0.0.0.0:14541`. `runSimNoeticMulti.sh` bind-mounts `models/iris/iris.sdf.jinja` read-only over the in-image iris jinja. Integer `MC_`/`MPC_` parameters go out as INT32. Calibration uses log time 50 s through 80 s only. Hover `motorConstant` is fit on that slice. After arming in offboard the stream holds a level attitude at a thrust just above that hover command until the local height is at least 10 m. The scored replay is then those 30 s. During the replay, thrust (and, on the velocity and position steps, `z`/`vz`) is corrected so the height stays at that altitude. Setpoints stream at 50 Hz. Pytest does not start Gazebo.

## Run

From the repository root:

```bash
python3 -m calibration.tune_rate
python3 -m calibration.tune_angle
python3 -m calibration.tune_velocity
python3 -m calibration.tune_position
python3 -m pytest calibration/tests -q
```

No argument uses `log_2_UnknownDate.ulg`. One extra argument is a usage error. Each tune program returns 0 when its search finishes. `python3 -m calibration.run_cascade` prints the four commands and returns 2.

| Output | Role |
| --- | --- |
| `models/iris/iris.sdf.jinja` | Hover `motorConstant` and `timeConstantUp`. Rotor x/y, prop radius, and `maxRotVelocity` stay frozen |
| `iris_controller.json` | Logged `MC_`/`MPC_` gains, with the kept overrides |
| `results/cascade_report.json` | RMS sum for the loop just tuned |
| `results/cascade/stages/<step>/parameter_history.jsonl` | One line per trial of that step: ULG path, step, RMS, and the parameter set flown. A new run of the step starts this file clean |
| `results/cascade/state.json` | Current step, trial, best RMS, and the parameters of the latest trial |
| `results/cascade/comparison.html` | Interactive Plotly page. Click a point on the RMS plot to show that flight |
| `results/cascade/stages/<step>/best.json` | Lowest-score trial. `"step"` is `rate`, `angle`, `velocity`, or `position` |
| `results/cascade/stages/<step>/logs/trial_NNN.ulg` | PX4 log copied out of the sim container at the end of that flight |

## Files

| File | Role |
| --- | --- |
| `tune_rate.py` | Frozen geometry, hover `motorConstant`, then `timeConstantUp` and rate P |
| `tune_angle.py` | Angle P after `stages/rate/best.json` |
| `tune_velocity.py` | Velocity PID after `stages/angle/best.json` |
| `tune_position.py` | Position P after `stages/velocity/best.json` |
| `run_cascade.py` | Prints the four tune commands and returns 2 |
| `cascade_hover.py` | Hover command `h` |
| `cascade_thrust.py` | Spin map and `motorConstant` |
| `cascade_model.py` | Read and write the iris jinja |
| `cascade_gains.py` | Logged `MC_`/`MPC_` gains; `MPC_THR_HOVER` forced to 0.5 |
| `cascade_flight.py` | Per-axis command and response from the ULG |
| `cascade_score.py` | Unshifted per-axis RMS |
| `cascade_search.py` | One pass per knob: `0.7`, `0.85`, `1.15`, `1.3` |
| `cascade_sitl.py` | Headless replay and the 50 Hz stream |
| `cascade_session.py` | Per-run parameter history and the comparison plot |
| `log_2_UnknownDate.ulg` | Default flight |
| `tests/test_tune_*.py`, `tests/test_cascade_*.py`, `tests/test_run_cascade.py` | Unit tests; no Docker, no sockets |
