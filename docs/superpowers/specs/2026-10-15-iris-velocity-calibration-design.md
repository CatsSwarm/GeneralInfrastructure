# Gazebo iris velocity calibration to the Baseball flights

Date: 2026-10-15
Status: approved

## Intent

The Gazebo iris is the aircraft used when the world has objects the Python sim cannot show, and it runs the same PX4 controller as the real aircraft. Quadcopter_SimCon was fitted to that real aircraft, but its controller is a separate Python implementation. The iris model in Gazebo still flies like the stock simulator.

This work fits the Gazebo iris model so that, under the PX4 setup stored in the real flights, its velocity follows those flights. The same velocity commands are then flown in Quadcopter_SimCon. Both should come out close. A gap between them is a result to keep.

## Decisions (locked)

| Topic | Choice |
|--------|--------|
| Fit target | The three PX4 ULG flights in `/home/valentin/RL/TESTFLIGHTS/Baseball/` |
| Logged channels | `vehicle_local_position` `vx vy vz` is the aircraft. `vehicle_local_position_setpoint` `vx vy vz` is the command |
| Command path | Offboard velocity, as in the logs (`offboard_control_mode.velocity` on, position unused, `trajectory_setpoint` velocity finite) |
| PX4 setup | The initial parameter set stored in the log being replayed. Not searched |
| What is adjusted | The Gazebo iris model: mass, inertia, rotor, and drag |
| What is measured | Drone velocity `vx`, `vy`, `vz` |
| Commands | Taken from those logs, or written for the test. No network output |
| Speed cap | Synthetic commands stay within 5 m/s on each of `vx` and `vy`. Logged setpoints are replayed as recorded (they already peak at 3.5 m/s) |
| SimCon role | Same commands in, velocity out. Compared and reported. The iris is not adjusted to shrink a SimCon gap |
| SimCon code | `Quadcopter_SimCon` plant and controller stay as they are, including `Simulation/quadFiles/initQuad.py` |
| Sim stack | Existing Gazebo Classic iris on the Noetic PX4 SITL (`runSimNoeticMulti.sh`, one iris, empty world) |

The three logs:

- `Group3RLcat2_log_41_2026-8-25-17-05-30.ulg`
- `Group3RLcat3_log_18_2026-8-25-17-05-26.ulg`
- `Group3RLcat4_log_28_2026-8-25-17-05-24.ulg`

Each is about 2.5 minutes of velocity-offboard flight, airborne near 1.3 m. cat3 stores `MPC_Z_VEL_MAX_DN = 3`; cat2 and cat4 store `1.5`. Replaying a log uses that log's own initial parameters, so the difference is kept.

## Goals

- A repo-local iris model parameter set that the Noetic SITL applies on spawn.
- A replay of each log's velocity setpoints on that iris, and a score of iris velocity against the logged velocity.
- The same setpoint series flown in Quadcopter_SimCon, with iris, SimCon, and log velocities written side by side.
- A small set of synthetic velocity commands, up to 5 m/s, flown on both the iris and SimCon, velocities written side by side.

## Non-goals

- Driving either sim from a network, from `train_swarm_envhyb.py`, or from the system manager.
- Editing Quadcopter_SimCon parameters or its controller to improve the match.
- Searching or editing PX4 gains. The logged parameter set is the setup.
- Matching position, attitude, motor speed, or accelerometer traces.
- Noble, Gazebo Jetty, or FlightGear.
- More than one iris, or a world other than empty.
- Raising the synthetic command above 5 m/s on `vx` or `vy`.

## Anti-goals

- A model that matches SimCon and misses the logs.
- Time-shifting a trace to hide lag. Lag is part of the response.
- Treating the aircraft's tracking error against its own setpoint as the thing to copy. The thing to copy is the logged velocity.

## Score

Airborne samples only: `|z| > 0.4` m in NED `vehicle_local_position.z`, and the setpoint sample is finite. Interpolate the iris (and SimCon) velocity onto the log timestamps. Do not shift either trace in time.

Per log, per axis, RMS of `(v_sim - v_log)`.

**Call:** the iris parameter set is accepted when every log meets all three:

- `vx` RMS ≤ 0.20 m/s
- `vy` RMS ≤ 0.10 m/s
- `vz` RMS ≤ 0.08 m/s

These sit next to the tracking residual already in the logs (about 0.20 m/s RMS on `vx`, about 0.06 m/s on `vz`). If several sets pass, keep the one with the smallest sum of the nine RMS numbers. If none pass, keep the best set, write the nine numbers, and stop. Do not loosen the bar.

The synthetic cases have no logged aircraft. They are not part of the score. They are only the iris-versus-SimCon report.

## Synthetic commands

**Call:** one axis at a time, the other two velocity setpoints held at 0, `vz` included and held at 0. Each case is its own run, starting from a steady hover at the logged height (about 1.3 m).

| Case | Command |
|------|---------|
| Steps | ±1, ±3, ±5 m/s on `vx`, then the same on `vy`. Hold each step 4 s |
| Sines | 0.2 Hz and 1.0 Hz, amplitude 2 m/s, `vx` then `vy`, 10 s each |
| Chirps | 0.1 Hz to 2 Hz, amplitude 1 m/s, `vx` then `vy`, 20 s each |

Yaw is held at the hover yaw. No extra yaw command.

## PX4 and the iris model

On each replay, SITL boots with that log's initial parameter set. `MPC_THR_HOVER` in these logs is 0.5. The model has to hover under that frozen value; the hover thrust parameter is not moved to suit a heavier iris.

The numbers that may change are iris mass, inertia, rotor (thrust and torque coefficients, motor time constant), and drag. Geometry stays the stock iris frame. Sensor inject (`inject_iris_sensors.py`, optical flow, lidar) stays as it is.

**Call:** the chosen numbers live in this repo and are applied to the iris SDF the SITL spawns. The copy inside the image is not the only copy. A restart of `runSimNoeticMulti.sh` picks up the calibrated model without an image rebuild.

## SimCon comparison

Feed the same `vx, vy, vz` setpoint series the iris received. Use the existing real-flight entry on the log replays (`run_real_flight_comparison` in `TrainingRoom/Quadcopter_SimCon/Simulation/run_3D_simulation.py`, control type `XY_VEL_Z_POS`) or an equivalent that does not change plant or controller code. Synthetic series use that same velocity-setpoint input.

Write, per log and per synthetic case: the command, the logged velocity when a log exists, the iris velocity, the SimCon velocity, and the per-axis RMS of iris-versus-log and iris-versus-SimCon. Save the numbers in the repo. Plots are optional and local; they are not required for the score.

## Left to the builder

- The search method over mass, inertia, rotor, and drag.
- Which SDF fields carry those four groups, as long as the groups above are the ones that change.
- How the repo-local numbers are applied at spawn.
- Whether a synthetic case is one flight or several, as long as each case starts from hover and the commands match the table.
- The on-disk layout of the numeric report.

## Test boundary

Unit tests cover the pieces that do not need Gazebo or SimCon running: reading the three logs into timestamped `vx, vy, vz` command and response plus the initial parameter set; the RMS score, including the airborne and finite-sample rules and the refusal to time-shift; the synthetic command generator, including the 5 m/s cap and the case table; and the file that stores the iris numbers the spawner applies.

The Gazebo replays and the SimCon runs are the calibration. Their proof is the score against the three logs and the side-by-side velocity report, not a unit test that pretends to fly.
