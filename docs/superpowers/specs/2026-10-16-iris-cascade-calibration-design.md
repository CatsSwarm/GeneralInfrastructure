# Gazebo iris cascade calibration from one PX4 ULG

Date: 2026-10-16
Status: approved

## Intent

The Gazebo iris should fly like the real drone in `calibration/log_2_UnknownDate.ulg`. The match is built from the inside of the control stack outward. Each stage sets one physical or controller quantity, then leaves it alone. A later stage does not move an earlier knob to chase a leftover error.

One Python program reads that ULG and writes the iris file the simulation mounts, plus the controller gains the simulation loads beside it.

This replaces, for this flight, the free search in `docs/superpowers/specs/2026-10-15-iris-velocity-calibration-design.md`. That search moved mass, inertia, and rotor coefficients together and judged only velocity. This program does not.

## Decisions (locked)

| Topic | Choice |
|--------|--------|
| Flight | `calibration/log_2_UnknownDate.ulg`. The program takes the ULG path as its input |
| Order | Hover thrust, then motor x/y, then angle gains, then velocity gains, then position gains |
| Hover window | Airborne (`vehicle_land_detected.landed` false) and `abs(vehicle_local_position.vz) < 0.1` m/s. Every such sample in the file counts, including hovers at different heights |
| Hover number | Mean of `actuator_motors` `control[0]` through `control[3]` over that window. That mean is the hover command `h` |
| Thrust write | Same `motorConstant` on all four motor plugins, set so a command of `h` holds the 1.5 kg weight. Body mass stays 1.5 kg. `maxRotVelocity` stays 1100 |
| Inertia | `ixx`, `iyy`, `izz` stay at the values already in the iris file |
| Rate controller | Copied from the log into the sim, then not searched |
| Motor positions | Horizontal pose of `rotor_0` through `rotor_3`. Height stays. Each rotor stays in its current quadrant |
| Rate match | Sim receives the logged rate demands. Roll rate and pitch rate must move closer to the log. Yaw rate is not matched |
| Angle match | Starts from the logged angle gains. Sim receives the logged angle setpoints. Roll and pitch must move closer to the log. Adjusted before velocity |
| Velocity match | Starts from the logged velocity gains. Sim receives the logged velocity setpoints. `vx`, `vy`, `vz` must move closer to the log |
| Position match | Starts from the logged position gains. Sim receives the logged position setpoints. `x`, `y`, and altitude `z` must move closer to the log |
| Closeness | As close as the sim can get. A step keeps an edit only when the sim curve moves closer, and stops when the next edit does not |
| Frozen after a step | Thrust ratio, motor positions, rate gains, and any gain already tweaked |
| Yaw | Outside the cascade. Not matched, not searched |
| Drone file | `calibration/models/iris/iris.sdf.jinja`, the file `runSimNoeticMulti.sh` bind-mounts over the in-image iris jinja |
| Gains file | Written beside that model. The sim loads it as `MC_` / `MPC_` parameters |
| Sim | The program starts the existing one-iris Noetic simulation, replays the log's demands for the current step, and writes the jinja and the gain file between runs |

On this log the hover mean is about 0.34. That is a check on the window, not a number to hardcode.

The log is a manual position flight (`flag_control_offboard_enabled` is off, `flag_control_manual_enabled` is on). The program still isolates each loop by feeding that loop's logged setpoint. Position `x` and `y` setpoints are finite on about 30% of samples, `z` on about 66%, velocity setpoints on about 99%. An axis with a non-finite command is left out of that axis's score.

`ulog_info` reports data corruption. The program still reads the file when the topics below are present.

## Goals

- A Python program: ULG path in, calibrated iris jinja and gain file out.
- Hover `h` computed from the window above, then one `motorConstant` written so a command of `h` holds the 1.5 kg weight. See Thrust.
- Motor `x` and `y` moved until roll rate (`vehicle_angular_velocity.xyz[0]`) and pitch rate (`xyz[1]`) sit as close as this search can get to the log, with the sim given `vehicle_rates_setpoint` `roll` and `pitch`.
- Angle, then velocity, then position gains tweaked the same way, each against its own logged command and response.
- Inner results left unchanged once their step has stopped.

## Non-goals

- Yaw rate, yaw angle, or yaw gains.
- Changing mass, inertia, `momentConstant`, motor time constants, rotor drag, `maxRotVelocity`, or rotor height.
- Using `hover_thrust_estimate` or `MPC_THR_HOVER` as the hover number. `MPC_THR_HOVER` stays at the log's value (0.5) and is not a search knob.
- The Baseball three-log search, its RMS gates (0.20 / 0.10 / 0.08), or fitting Quadcopter_SimCon.
- Matching accelerometer traces, motor RPM, or the aircraft's own tracking error against its setpoint as a separate target. The target is the logged vehicle response.
- Noble, Gazebo Jetty, FlightGear, more than one iris, or a world other than empty.

## Anti-goals

- Going back to an inner knob because an outer loop is still off.
- Time-shifting a trace to hide lag.
- Treating a numeric gate as success. Success is the closest curve this cascade reaches, even when error remains.

## What is compared

Interpolate the sim onto the log timestamps. Do not shift either trace. Score airborne samples only.

**Call:** each axis is scored on its own. A sample counts for an axis when that axis's command is finite. A position sample with finite `z` and non-finite `x` still scores altitude. `x` and `y` are finite together with `z` on about 19% of setpoint samples; `z` alone is finite on about 66%. Dropping a sample unless all three are finite would fit `MPC_Z_P` only on the short 3D-hold bursts, and would leave out the altitude-hold segments the position step is supposed to match.

**Call:** closeness is the sum of per-axis RMS of `(sim - log)` for the signals of that step. Keep a candidate only when that sum falls. Stop the step when it does not.

| Step | Command | Response |
|------|---------|----------|
| Rate | `vehicle_rates_setpoint` `roll`, `pitch` | `vehicle_angular_velocity` `xyz[0]`, `xyz[1]` |
| Angle | `vehicle_attitude_setpoint` `roll_body`, `pitch_body` | Roll and pitch from `vehicle_attitude` `q` |
| Velocity | `vehicle_local_position_setpoint` `vx`, `vy`, `vz` | `vehicle_local_position` `vx`, `vy`, `vz` |
| Position | `vehicle_local_position_setpoint` `x`, `y`, `z` | `vehicle_local_position` `x`, `y`, `z` |

## Parameters

The replay applies the log's initial `MC_` and `MPC_` set, then the gain file. After boot, the airframe post may have set `MPC_THR_HOVER` to 0.6; the replay puts back the log's 0.5 before any step.

Copied from the log and then frozen:

- Roll and pitch rate: `MC_ROLLRATE_P`, `MC_ROLLRATE_I`, `MC_ROLLRATE_D`, `MC_ROLLRATE_FF`, `MC_ROLLRATE_K`, and the same `MC_PITCHRATE_*` fields.
- **Call:** `FF` and `K` are copied with P, I, and D. They are part of the rate controller stored in the log. They are not searched.

Searched, in order, starting from the logged value:

- Angle: `MC_ROLL_P`, `MC_PITCH_P`. The log has no angle I or D.
- Velocity: `MPC_XY_VEL_P_ACC`, `MPC_XY_VEL_I_ACC`, `MPC_XY_VEL_D_ACC`, `MPC_Z_VEL_P_ACC`, `MPC_Z_VEL_I_ACC`, `MPC_Z_VEL_D_ACC`.
- Position: `MPC_XY_P`, `MPC_Z_P`. The log has no position I or D.

Yaw parameters stay at the logged values. They are not in the gain-file search.

**Call:** the gain file is `calibration/iris_controller.json`, a map of the `MC_` / `MPC_` names above to floats. The existing replay may send only `MC_` and `MPC_` names, and it must accept this file's keys, including the rate `FF` and `K` values. It must not refuse the file for containing those keys, and it must not require `MPC_THR_HOVER` to be absent: the value 0.5 is sent so the post's 0.6 does not survive.

## Motor geometry

**Call:** the search uses two lengths, an `|x|` arm and a `|y|` arm. They are written onto `rotor_0` through `rotor_3` with each rotor's current sign. The existing `|y|` split (0.20 m and 0.22 m) becomes one `|y|`. Rotor `z` stays 0.023 m. This is the moment arm in torque / inertia: `x` for pitch, `y` for roll. Inertia stays put, and the hover step has already set the force.

## Iris file

`runSimNoeticMulti.sh` bind-mounts `calibration/models/iris/iris.sdf.jinja` read-only onto the in-container iris jinja. A search writes the host file. The next sim start reads it through the mount. No image rebuild.

Only `motorConstant` (all four plugins, one value) and the four rotor `x` `y` poses change in that file.

## Thrust

Armed, this iris maps a motor command `c` to spin `min(c * 1000 + 100, 1100)` rad/s. Thrust of one motor is `motorConstant * spin^2`. Full command (`c = 1`) spins at 1100.

`motorConstant` is chosen so four motors at command `h` produce the weight:

`motorConstant = (1.5 * 9.81) / (4 * spin(h)^2)`

**Call:** `g` is 9.81 m/s². Full-command thrust is then larger than weight / `h`, because spin at `h` is not `h` times spin at full command. That is the accepted outcome. The program does not also force full-command thrust to equal weight / `h`.

## Left to the builder

- The Python module name and CLI, as long as the input is a ULG path and the outputs are the jinja and `calibration/iris_controller.json`.
- The search schedule inside a step: step size, one knob at a time or together, as long as a candidate is kept only when the step's RMS sum falls, and earlier knobs stay frozen.
- How the sim process is started and stopped, as long as it is the existing one-iris Noetic launch on an empty world and it sees the host jinja through the current mount.
- How roll and pitch are read from `vehicle_attitude.q`, as long as both the log and the sim use the same conversion.
- The on-disk report of each step's final RMS. The absence of a numeric pass/fail gate is locked.

## Test boundary

Unit tests cover the pieces that do not need Gazebo: the hover window and the mean of `control[0..3]`; `motorConstant` from `h` using the spin map and `g = 9.81`, so thrust at command `h` equals the 1.5 kg weight; writing that one `motorConstant` and the two arm lengths into the host jinja without touching mass, inertia, rotor `z`, or the other motor tags; writing the gain file and refusing to put yaw or `MPC_THR_HOVER` changes other than restoring 0.5 into the search; the RMS sum, including airborne samples, per-axis finite commands, and the refusal to time-shift.

The Gazebo replays are the calibration. Their proof is the per-step RMS on this ULG, not a unit test that pretends to fly.
