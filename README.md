# CatSwarm General Infrastructure

This project provides the general infrastructure for multi-drone Reinforcement Learning (RL) simulations using PX4 Autopilot and ROS Noetic. It acts as the bridge between high-level RL policies and the low-level flight control stack.

## Architecture

The system follows a modular architecture:

1.  **Simulation Environment**: Docker images for PX4 SITL — Noetic/Focal (`px4-noetic-sim-ros`, Gazebo Classic) and Noble/24.04 (`px4-noble-sim-ros`, Gazebo Jetty + FlightGear 2024). Prefer `./runSimNoble.sh` for Gazebo Jetty; FlightGear Rascal via `./fixedwing/runSimFlightGearRascal.sh` (injectable `fixedwing/fg_spawn.env`) or `make px4_sitl_nolockstep flightgear_rascal` inside the Noble image (see `Dockerfiles/README.md`). Nested sim images live in `Dockerfiles/` (`PX4dockerfiles`). Iris velocity calibration (`calibration/`) bind-mounts a host iris jinja into the Noetic sim and scores it against the Baseball logs; a search rewrites that host file between sim restarts. Cascade calibration is four programs. `python3 -m calibration.run_cascade` prints `python3 -m calibration.tune_rate`, `tune_angle`, `tune_velocity`, and `tune_position` and returns 2. `tune_rate` reads one PX4 ULG and writes that same jinja plus `calibration/iris_controller.json`: it freezes rotor x/y, prop radius, and `maxRotVelocity`, fits hover `motorConstant`, then searches `timeConstantUp` and `MC_ROLLRATE_P` / `MC_PITCHRATE_P`. `tune_angle` flies only after `calibration/results/cascade/stages/rate/best.json` exists with `"step": "rate"`; it searches `MC_ROLL_P` and `MC_PITCH_P` and keeps a gain only when the sum of roll-rate, pitch-rate, roll, and pitch RMS falls. `tune_velocity` flies only after `calibration/results/cascade/stages/angle/best.json` exists with `"step": "angle"`; it searches `MPC_XY_VEL_P_ACC`, `MPC_XY_VEL_I_ACC`, `MPC_XY_VEL_D_ACC`, `MPC_Z_VEL_P_ACC`, `MPC_Z_VEL_I_ACC`, and `MPC_Z_VEL_D_ACC`, keeps a gain only when the sum of roll-rate, pitch-rate, vx, vy, and vz RMS falls, and leaves rate P and angle P as the angle loop left them. `tune_position` flies only after `calibration/results/cascade/stages/velocity/best.json` exists with `"step": "velocity"`; it searches `MPC_XY_P` and `MPC_Z_P` and keeps a gain only when the sum of roll-rate, pitch-rate, x, y, and z RMS falls.
2.  **Communication Bridge**:
    - **MAVLink (UDP)**: Connects the PX4 SITL instances to the bridge.
    - **Bridges (C++)**: `mavlink_to_ZMQ` and `zmq_commands_mavlink` convert MAVLink messages to/from ZeroMQ.
    - **ZeroMQ (ZMQ)**: Provides a high-performance IPC layer for the Python control stack.
3.  **Control Stack (Python)**:
    - `system_manager.py` (assumed): Manages the main control loop.
    - `rl_policy.py`: Implements the RL inference using PyTorch.
4.  **Companion Pi (real)**: `deploymentScript/startup_scripts/start_companion_drone_tmux.sh` + `install-companion-boot.sh`. After HA/SM/VIO start, pane stdout is `pipe-pane`d to `~/RL/logs/tmux/`; journald `Storage=persistent`. See `startup_scripts/README.md`.

## Key Files

### Simulation & Setup
- **`fixedwing/runSimFlightGearRascal.sh`**: Noble + FlightGear Rascal plane. Mounts `fixedwing/fg_spawn.env` (`FG_ARGS_EX`) for in-air spawn (default 500 m / ~30 m/s) without rebuilding the image.
- **`fixedwing/run_straight_flight.py`**: Starts the Rascal sim (unless `--no-sim`), arms if needed, switches OFFBOARD, streams body-forward velocity setpoints for straight flight.
- **`runSimNoeticMulti.sh`**: The main entry point. Starts the Dockerized PX4/ROS simulation environment for multiple drones. It handles container management, X11 forwarding, and cleaning up processes. Mounts `calibration/models/iris/iris.sdf.jinja` read-only over the in-image iris jinja (both volume lists), iris sensors inject (px4flow + lidar), iris OB-color inject, + `10015_gazebo-classic_iris.post` so SITL publishes `OPTICAL_FLOW` / `DISTANCE_SENSOR` (matches cage rangefinder). `--world NAME` (default `empty`) mounts host `Dockerfiles/models` at `.../models/catswarm_host` (does not overlay PX4 `iris`); non-empty worlds also mount `NAME.world` and set `PX4_HOME_*` from `origin.json`. `--world cylinders` uses PX4 empty.world + four visual-only cylinders (0.50 m diameter, 10 cm tall); `--cylinder-radius` (default 10). JPEG HTTP preview on `5700+(id-1)`. The host script also runs `multidrone/sim_rc_radio.py`, which sends a bound stick override to PX4 so takeoff can arm.
- **`Dockerfiles/scripts/generate_real_area.py`**: Host-only generator (`--lat --lon --radius-m --out`) for Gazebo Classic ortho + heightmap + OSM solids. Run from CatSwarm root; see `Dockerfiles/README.md`. GUI does not call it.
- **`multidrone/inject_iris_sensors.py`**: Injects nested `model://px4flow` + `model://lidar` into generated multi-SITL iris SDF before spawn. (`inject_iris_lidar.py` is a thin compat wrapper.)
- **`multidrone/inject_iris_colors.py`**: Recolors iris mesh visuals at spawn to Observation Board `droneColors.ts` palette (`iris_N` → OB id `N`).
- **`multidrone/run_multidrone_bridges.sh`**: Sets up the communication layer. It creates a tmux session with pairs of bridge processes (`mavlink_to_ZMQ` and `zmq_commands_mavlink`) for each drone defined in the positions file.
- **`multidrone/positions.txt`**: Configuration file defining the initial spawn positions (x, y, z, yaw) for the drones.
- **`hardware_adapter.sh`**: Legacy/Single-drone script to start the hardware adapter bridges.

### Core Logic
- **`rl_policy.py`**: Contains the `RLPolicy` class, a PyTorch module implementing the actor network (Encoder -> GRU -> Gaussian Head).
- **`simpleMavlinkTest.py` / `simpleMavlinkSwitchTo.py`**: Utility scripts for testing MAVLink connections and switching flight modes.
- **`multidrone/simpleZMQtakeoffland.py`**: Sends takeoff and land commands via ZMQ.
    - Usage: `python3 multidrone/simpleZMQtakeoffland.py --takeoff --altitude=15.0`
    - Usage: `python3 multidrone/simpleZMQtakeoffland.py --land --zmq=7793`
- **`multidrone/takeoffland.py`**: Sends takeoff and land commands via MAVLink UDP.
    - Usage: `python3 multidrone/takeoffland.py --takeoff --altitude=12.0 --udp=14541`
    - Usage: `python3 multidrone/takeoffland.py --land --udp=14542`

## Usage

1.  **Start the Simulation**:
    ```bash
    ./runSimNoeticMulti.sh --num 3
    ./runSimNoeticMulti.sh --num 3 --world teradyon
    ./runSimNoeticMulti.sh --num 3 --world cylinders --cylinder-radius 10
    ```
    (Adjust `--num` for the number of drones. `--world` default is PX4 `empty`; generate host maps first. `cylinders` is empty.world + four visual markers, 10 cm tall.)

2.  **Start Communication Bridges**:
    ```bash
    ./multidrone/run_multidrone_bridges.sh
    ```
    This will start a tmux session named `multidrone_bridges`.

3.  **Run Control Logic**:
    (Depending on your specific workflow, e.g., running the system manager or RL training script).

## Project Updates

For a detailed history of changes and fixes, please refer to **[UPDATES.md](UPDATES.md)**.

**Updating Policy**:
- Every agent working on this project **MUST** update `UPDATES.md` with a summary of their changes, fixes, and new features.
- Entries should be chronological (newest first).
- Keep descriptions laconic but self-contained so that future agents can quickly understand the project state.
