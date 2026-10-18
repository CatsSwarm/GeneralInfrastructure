# Project Updates

This file documents the development progress and changes made to the `CatSwarm/general_infrastructure` project by the AI agent.

## [2026-10-17] Iris cascade calibration
- `python3 -m calibration.run_cascade` reads a ULG (default `calibration/log_2_UnknownDate.ulg`) and writes the mounted iris jinja plus `calibration/iris_controller.json`.
- Order: hover `motorConstant` so command `h` holds 1.5 kg, then one `|x|` and one `|y|` arm, then angle, velocity, and position gains. Rate gains, yaw, mass, and inertia stay put. No numeric pass gate. Closeness is unshifted per-axis RMS.
- Live replay is headless one iris on an empty world, port 14541. Integer `MC_`/`MPC_` parameters go out as INT32. The scored clock starts when the vehicle is armed in offboard. The stream is 50 Hz. Pytest does not start Gazebo. `python3 -m pytest calibration/tests -q` — 63 passed.

## [2026-10-16] Iris cascade calibration spec
- Spec `docs/superpowers/specs/2026-10-16-iris-cascade-calibration-design.md` (approved): one Python program reads a PX4 ULG and writes the mounted iris jinja plus `calibration/iris_controller.json`. Flight is `calibration/log_2_UnknownDate.ulg`.
- Order: hover `motorConstant` so command `h` holds 1.5 kg, then rotor `|x|`/`|y|` on roll/pitch rate, then angle, velocity, and position gains. Yaw, mass, and inertia stay put. No numeric pass gate. Replaces the Baseball free search for this flight.
- Plan `docs/superpowers/plans/2026-10-16-iris-cascade-calibration.md`. Unit tests do not start Gazebo.

## [2026-10-16] Iris search scored, not accepted
- One coordinate-descent pass (center plus 18 neighbors) on the three Baseball logs. None met vx ≤ 0.20, vy ≤ 0.10, vz ≤ 0.08. Kept the smallest RMS sum: `motor_constant` 2.92e-06, other eight fields stock (mass 1.5, ixx/iyy 0.029125, izz 0.055225, moment 0.06, time up/down 0.0125/0.025, rotor drag 0.000175). `accepted` is false.
- Iris RMS (selected set) cat2 0.611/0.080/0.075, cat3 0.663/0.275/0.100, cat4 0.644/0.226/0.098 (vx/vy/vz). Stock center was cat2 0.327/0.221/0.907, cat3 0.662/0.275/0.110, cat4 0.652/0.228/0.099. SimCon, not fitted: cat2 0.070/0.041/0.056, cat3 0.072/0.051/0.047, cat4 0.080/0.061/0.055. Report: `calibration/results/velocity_report.json`.
- `run_search` writes `calibration/results/iris/<stem>.csv` only for the selected candidate, so `run_compare` does not score the last neighbor. Pytest: `python3 -m pytest calibration/tests -q` (37 passed).
- This host's indirect GLX aborts `gzserver` (`glXCreateContext`, then `Header is empty` / `std::length_error`). `HEADLESS=1 ./runSimNoeticMulti.sh` clears `DISPLAY`, skips `docker -it`, and the sitl script waits instead of running `gzclient`. Ray lidar and the optical-flow mockup do not need rendering. First vehicle is `px4 -i 1` (onboard mavlink remote port 14541).

## [2026-10-16] Iris velocity calibration
- `calibration/` loads the three Baseball ULGs, scores unshifted airborne RMS (accept at vx ≤ 0.20, vy ≤ 0.10, vz ≤ 0.08), patches the host iris jinja, and writes `calibration/results/velocity_report.json`. Quadcopter_SimCon is not edited.
- Both `runSimNoeticMulti.sh` volume lists bind-mount `calibration/models/iris/iris.sdf.jinja` read-only over the in-image iris jinja. Restart the sim after a search write.
- Log replay does not position-hold. Synthetic iris replay holds only with `--position-hold`. A missing synthetic CSV does not change `accepted`. Pytest: `python3 -m pytest calibration/tests -q` (36 passed).
- Open: a multi-candidate search overwrites the same three iris CSVs, so `run_compare` reads the last neighbor. Log warmup still sends the first velocity setpoint before the series clock.

## [2026-10-15] Iris velocity calibration spec and plan
- Spec `docs/superpowers/specs/2026-10-15-iris-velocity-calibration-design.md` (approved): fit the Gazebo iris model to the three Baseball ULG velocity responses; PX4 `MPC_`/`MC_` setup comes from the log; Quadcopter_SimCon is a same-command comparison, not a second fit.
- Plan `docs/superpowers/plans/2026-10-15-iris-velocity-calibration.md`: parallel test-first waves. No Gazebo run in the unit tests.

## [2026-09-20] Companion tmux pipe-pane + persistent journald
- After HA/SM/VIO windows exist, `companion_tmux_pipe_session` `pipe-pane -o`s every pane to `~/RL/logs/tmux/<UTC>_<session_window_pane>.log`.
- `install-companion-boot.sh` writes `/etc/systemd/journald.d/companion-persistent.conf` (`Storage=persistent`) and restarts journald before enabling `companion-drone.service`.
- Tests: `util/tests/test_companion_tmux.py`. Re-run install-companion-boot on the Pi (or copy the drop-in) so journal survives reboot.

## [2026-09-19] Apply RTK/COMM COMM_SIM pub from MultiInput
- `switch_rtk_WIFI_RF.sh` / `switch_comm_WIFI_RF.sh` used `COMM_PUB_PORT=7800+id` after MultiInput moved to `zmqCommSimOutPort` 19901+. `ZMQ_to_comm` republished air-RX peers on 7801; `comm_to_ZMQ` SUBed 19901 → OB GS dots green, interdrone red, PAIRING stuck.
- `companion_comm.z2c_comm_pub_port` reads MultiInput (fallback 7800+id). Restart companion or at least `comm_to_ZMQ` + `ZMQ_to_comm`. Tests: `util/tests/test_companion_comm.py`. Pair HA **v1.34.2**.

## [2026-09-18] Cylinder height 10 cm
- `CYLINDER_HEIGHT_M` is 0.10 (was 0.05). SDF length and link pose (`height/2`) still sit the visual on world z=0.

## [2026-09-18] Cylinder height 5 cm
- `CYLINDER_HEIGHT_M` is 0.05 (was 1.0). SDF length and link pose (`height/2`) follow so the visual still sits on world z=0.

## [2026-09-18] Cylinder spawn z=0 (base on ground)
- `place_cylinders` spawn `-z 0` (was height/2). SDF link pose stays `0 0 0.5` so the 1 m visual sits on world z=0, not floating.

## [2026-09-18] Cylinders world + JPEG 5700
- `--world cylinders` uses empty.world + four visual cylinders (`--cylinder-radius`, default 10).
- JPEG HTTP `5700+(id-1)`.

## [2026-09-12] VIO Apply gz joint collapsed iris to origin
- `gz joint --pos-t` on revolute `vio_cam_pitch` (unlimited effort) yanked the parent iris: live D3 went from (0, 6, 1.1) to (0, 0, 0) with rotors stacked at origin; D1/D2 already vanished the same way after Apply.
- `sim_vio.sh pitch` is a no-op. `vio_cam_pitch` is `fixed`. Look-down remains the nested SDF pose (`CATSWARM_VIO_PITCH`). Restart SITL to restore vanished models (cannot un-collapse in place). Pair OB **1.48.12** (Apply does not call pitch).

## [2026-09-12] VIO on no longer flips iris at spawn
- Cause: Start Sim `sim_vio.sh pitch` set world pose of nested `iris_N::vio_cam` (`gz model`); the joint yanked the iris over.
- Pitch at spawn is the SDF nested pose (`CATSWARM_VIO_PITCH`, default −90 → Gazebo +π/2). `runSimNoeticMulti.sh --vio-pitch`. Camera link `gravity` false.
- `sim_vio.sh pitch` is joint-only (no nested `gz model`).

## [2026-09-12] sim_vio.sh pitch: fail empty gz + pose nested cam
- `gz joint` without `--verbose` exits 0 for a missing `iris_*` (no subscribers); Start Sim treated pitch as applied while the camera was still forward.
- Pitch now uses `--verbose` and fails on `No subscribers`. After joint cmd, poses nested `iris_N::vio_cam` (lever 0.10 m, Gazebo pitch = −FRD deg) because joint PID does not hold look-down.

## [2026-09-12] Sim VIO pitch: real gz flag + Z-up sign
- `gz joint --pos-t0` is invalid on Gazebo 11 and still exits 0, so VIO Apply never moved `vio_cam_pitch` (T_B_C said −90 down, camera stayed forward → stuck Initializing).
- Flag is `--pos-t`. FRD −90° maps to Gazebo **+π/2** (iris `base_link` is Z-up; −π/2 looks at the sky). Pitch fails Apply if gz prints `Invalid arguments`.

## [2026-09-11] CATSWARM_VIO_CAM / --vio-cam gates Gazebo VIO camera
- `runSimNoeticMulti.sh --vio-cam=0|1` (default 1) exports `CATSWARM_VIO_CAM` into the container.
- `inject_iris_sensors.py` skips `vio_cam` when off; lidar/OF unchanged.
- `sitl_multiple_run.sh` skips `vio_cam_tcp` when `CATSWARM_VIO_CAM=0`.

## [2026-09-11] VIO pitch joint signed degrees (−90 look down)
- `pitch_deg_to_joint_rad` / `sim_vio.sh pitch` pass signed degrees through to `gz joint --pos-t0` (0 = forward, −90 = −π/2 down). Tests lock −90, not +90.

## [2026-09-11] generate() layout test locks spec contract
- Nested `test_generate_layout_mocked`: `alt_amsl≈200`, world spherical coords, OSM `buildings.dae` + SDF link. `generate()` unchanged. Nested Dockerfiles 1.8.2.

## [2026-09-11] Dedicated SITL mavlink for host VIO IMU
- `10015_gazebo-classic_iris.post` starts a VIO mavlink instance: PX4 local `14680+px4_instance`, remote `14640+px4_instance`, `HIGHRES_IMU` 100 Hz + GPS/NED. Does not share HA onboard `14540+i`.
- `sim_vio.sh` start passes `--mavlink=udp:127.0.0.1:$((14640 + id))`. Restart SITL so `.post` runs, then Apply VIO.

## [2026-09-11] Real-area docs + gitignore; spec approved
- `Dockerfiles/README.md` Noetic: generator CLI, Teradyon, `--volume` catswarm_host, Esri/Terrarium/OSM ODbL attribution. `Dockerfiles/.gitignore` generated rasters only (not ksql DAE). Spec `2026-09-11-real-area-gazebo-world-design.md` status `approved`.

## [2026-09-11] vio_cam_tcp listen survives rospy socket timeout
- rospy sets a process-wide `socket.setdefaulttimeout`; listen `accept()` then raised `socket.timeout`, the thread died, and host `svo_pi` printed `VIO skipped: no camera` (`:5600` refused).
- `_default_bind` forces blocking (`settimeout(None)`); `serve_frames` retries `accept()` on timeout.

## [2026-09-11] runSim --world + catswarm_host model path
- `runSimNoeticMulti.sh --world NAME` (default `empty`). Missing world exits after parse, before cleanup trap (does not kill a running sim). Host `Dockerfiles/models` mounted at `.../models/catswarm_host` (iris overlay avoided). Non-empty: mount `NAME.world`, set `PX4_HOME_*` from `origin.json`. Always pass `-w ${WORLD}` into sitl.
- `sitl_multiple_run.sh` appends `GAZEBO_MODEL_PATH` `.../models/catswarm_host` immediately after `setup_gazebo.bash`.

## [2026-09-11] Host real-area generate() mosaic + atomic replace
- Nested `Dockerfiles/scripts/generate_real_area.py` (PX4dockerfiles 1.8.0): `generate()` with injectable `http_get`; Terrarium/Esri mosaic; Overpass GET; two-texture heightmap SDF; atomic `.generating` replace (previous `out` kept on failure). `main` wired. Tests never hit the network.

## [2026-09-11] Host sim_vio libs + Gazebo camera plugin path
- `sim_vio.sh` start exports `SCHURVINS_HOST_PREFIX` + `LD_LIBRARY_PATH` (`sys/devel/opencv/ros`) so host `svo_pi` finds Focal SVO/OpenCV libs.
- `sitl_multiple_run.sh` prepends `/usr/lib/x86_64-linux-gnu/gazebo-11/plugins` to `LD_LIBRARY_PATH` so `libgazebo_ros_camera.so` can dlopen `libCameraPlugin.so` (VIO topics had no publisher).

## [2026-09-11] OSM parse + COLLADA extrusion
- Nested `Dockerfiles/scripts/generate_real_area.py` (PX4dockerfiles 1.7.0): `parse_osm_xml` / `write_buildings_dae`; fixture OSM (height, levels, bridge, degenerate skipped). No `generate()` yet.

## [2026-09-11] Host real-area heightmap, origin.json, world XML
- Nested `Dockerfiles/scripts/generate_real_area.py` (PX4dockerfiles 1.6.0): `encode_heightmap` `(u8, size_z, pos_z)`; ENU aeqd; `origin.json`; world SDF spherical coords, no ground_plane. No `generate()` yet.

## [2026-09-11] Host real-area generator CLI (bbox, height, args)
- Nested `Dockerfiles/scripts/generate_real_area.py` (PX4dockerfiles 1.5.0): `--lat --lon --radius-m --out`; WGS84 bbox; OSM extrusion height; `HEIGHTMAP_N=257`. No `generate()` yet.

## [2026-09-11] vio_cam_tcp keeps listen socket across probe
- `serve_frames` loops `accept()` for process lifetime. Probe connect+close / send error accepts the next client; `srv` closes only when `gray_iter` is exhausted.

## [2026-09-11] Sim VIO camera, TCP republisher, host sim_vio
- `inject_iris_sensors.py` adds pitchable `vio_cam` (320×240, 60° hfov, joint `vio_cam_pitch`) and keeps OF/lidar.
- `multidrone/vio_cam_tcp.py` republishes `/iris_{id}/vio_cam/image_raw` as SVOF grayscale on `5600+(id-1)` (host-mounted; no image rebuild). `sitl_multiple_run.sh` starts it after spawn.
- `deploymentScript/startup_scripts/util/sim_vio.sh` start/stop/pitch host `svo_pi --source gazebo`; skip never fails the swarm; pitch returns gz exit code.

## [2026-09-10] Apply RTK keeps WiFi COMM argv
- `switch_rtk_WIFI_RF.sh` relaunch of `hardware_adapter_<id>.3` loads `~/.config/companion-comm` (`load_companion_comm` / `z2c_extra_args`) and omits `--serial-comm-tx` when fabric is wifi (same table as `switch_comm_WIFI_RF.sh`). Does not overwrite companion-comm.

## [2026-09-10] Companion COMM fabric persist + switch_comm_WIFI_RF
- `~/.config/companion-comm`: `COMPANION_COMM_FABRIC` (`wifi`|`rf`, default rf) and `COMPANION_COMM_GS_HOST`. Helper `util/companion_comm.py`: `load_companion_comm` / `save_companion_comm` / `z2c_extra_args` (Task 7 tokens `:18811`/`:18812`; rf → `[]`).
- `switch_comm_WIFI_RF.sh <id> --wifi|--rf [--gs-host=]`: persist; `companion_tmux_bind_session`; restart `hardware_adapter_<id>.3` only (no GPS/RTCM). Wifi omits `--serial-comm-tx`; keeps `--serialcomm` / RTK `--rtk-zmq-bind`.
- `start_companion_drone_tmux.sh` reads companion-comm (default rf), exports fabric/host, passes `--wifi-comm-host` into `hardware_adapter_multi.sh`.
- Pair OB **1.53.0**: Fleet COMM RF | WiFi + Apply COMM → `POST /api/deploy/comm_mode` (this script).

## [2026-09-07] Radio free: killall not pkill -f ZMQ_to_comm_c
- `pkill -f bin/ZMQ_to_comm_c` matched the SSH/deploy bash line that contained that text and aborted UART free. `ensure_companion_radio_soft_config.sh` now `killall -TERM ZMQ_to_comm_c` + bracketed python pkill.

## [2026-09-07] SSH tmux finds systemd companion session
- Cause: `companion-drone.service` oneshot `User=pi` has no `XDG_RUNTIME_DIR`; tmux 3.2+ puts `catswarm_sim` in `/tmp/tmux-<uid>/`. SSH PAM sets `XDG_RUNTIME_DIR=/run/user/<uid>` → `switch_rtk_WIFI_RF.sh: tmux session catswarm_sim not found` after RF Apply/Check killed `ZMQ_to_comm`.
- `companion_tmux_bind_session` tries current server, then `/tmp`, then runtime dir. Sourced by `switch_rtk_WIFI_RF.sh` and `ensure_companion_radio_soft_config.sh`.
- Pin `TMUX_TMPDIR=/tmp` in the unit, `run-companion-drone.sh`, and `start_companion_drone_tmux.sh`. Tests: `util/tests/test_companion_tmux.py`. Sync `startup_scripts` (Update); pin takes effect on next companion start.

## [2026-09-06] VIO tmux python falls back to system python3 for picamera2
- `companion_vio_start_in_tmux` keeps the conda interpreter when it can `import picamera2`; otherwise uses `/usr/bin/python3` (Pi Debian picamera2 is 3.13; conda `python3` after `conda activate RL` is 3.11 and must not be used as the fallback).


## [2026-09-06] SCHURVINS_ROOT prefers CatSwarm/SchurVINS
- Default is `${CATSWARM_ROOT}/SchurVINS` when that directory exists, else `$(cd "${CATSWARM_ROOT}/.." && pwd)/SchurVINS`. Covers Pi `~/RL/SchurVINS` and laptop `CatSwarm/SchurVINS` (GI as CATSWARM_ROOT). Env `SCHURVINS_ROOT` still wins.
- Spec helper `default_schurvins_root` in `companion_vio_spec.py`; tests in `test_companion_vio.py`.

## [2026-09-06] Companion VIO: pkill feeder; pass IMX500 calib/options
- `--kill` also pkills `svo_pi.feeder` (with supervisor and `/svo_pi/svo_pi`) so a moved-window feeder cannot keep the Unix socket or camera.
- `vio_N` supervisor launch passes `--calib=${SCHURVINS}/svo_ros/param/calib/imx500_320.yaml` and `--options=.../vio_mono.yaml`.

## [2026-09-06] Companion vio_N window; HIGHRES 100 Hz default; skip on missing camera
- Companion start opens `vio_${DRONE_ID}` after GPS (`SCHURVINS_ROOT` default `$(cd CATSWARM_ROOT/.. && pwd)/SchurVINS`). Missing CSI camera or `svo_pi` → supervisor prints skip, window parks with `sleep infinity`, start script never `exit 1`.
- QGC stream helper per-stream defaults: HIGHRES_IMU msgid 105 @ 100 Hz (10000 us), OPTICAL_FLOW_RAD 50 Hz. `--hz` / `COMPANION_QGC_STREAM_HZ` still overrides all streams when set; omit `--hz` unless that env is set so HIGHRES is not stomped back to 50 Hz.
- `--kill` pkills `svo_pi.supervisor` and `/svo_pi/svo_pi` before tmux kill-session.

## [2026-08-14] OS gcc for local HA/SM builds
- `compile.sh` and `simplemavlinktest/Makefile` use `/usr/bin/gcc` / `/usr/bin/g++`. Companion deploy unchanged (HA Makefile / SM `build.sh` already pin the compiler).

## [2026-08-14] SITL: iris colors match Observation Board
- `inject_iris_colors.py` rewrites Gazebo script materials to OB palette at spawn (`iris_N` → drone `N`).
- Wired in `sitl_multiple_run.sh`; bind-mounted by `runSimNoeticMulti.sh`; baked in Noetic Dockerfile.
- Spec: ObservationBoard `docs/superpowers/specs/2026-08-12-gazebo-drone-colors-design.md`. Restart sim to see colors (host mount; rebuild image only for bake path).

## [2026-08-11] mavlink-server :14580 required for RTCM inject
- Cause: fleet `mavlink-server.conf` wrote `[[udp_server]]` **without** `address`; `run-mavlink-server.sh` skipped it → no `udpserver://0.0.0.0:14580`; Apply RTK bridge sent RTCM into the void (D2 worked only because conf was hand-fixed).
- Fix: template + `mavlink-server-configuration.sh` include `address = "0.0.0.0"`; `ensure_mavlink_rtcm_udp.sh` runs on Apply RTK (`--sink=mavlink_rtcm`).

## [2026-08-11] RTCM mavlink bridge: RF vs WiFi exclusive
- Companion switch + `startRtcmToMavlinkPI`: serial/RF Apply → rf-only bridge; WiFi Apply → wifi-only. No forced dual path for mavlink on wifi mode.
- OB Apply RTK always passes `--sink=mavlink_rtcm`.

## [2026-08-08] QGC default HIGHRES_IMU + OPTICAL_FLOW_RAD streams
- Cause: QGC on mavlink-server TCP `:5760` showed `DISTANCE_SENSOR` but not `HIGHRES_IMU` / `OPTICAL_FLOW_RAD`; HA `SET_MESSAGE_INTERVAL` on UDP `:14540` does not raise those rates on the shared QGC path.
- Fix: `startup_scripts/util/qgc_mavlink_streams.py` requests msgid 105/106 @ 50 Hz on `tcp:127.0.0.1:5760`, refreshed every 10 s; hooked from `start_companion_drone_tmux.sh` (+ kill on `--kill`). Override: `COMPANION_QGC_MAVLINK` / `COMPANION_QGC_STREAM_HZ`.
- Tests: `startup_scripts/util/tests/test_qgc_mavlink_streams.py`. Sync `startup_scripts` to Pi and restart companion (or run the script once).

## [2026-08-05] Companion sniff/profile: USB F9P @ 230400 + ROVER_WIRE
- `sniff_companion_gps_profile.py`: after ACM@115200, probe `ttyUSB*` @ 230400 then 115200 (MON-VER) → `usb_f9p|/dev/ttyUSB*`.
- `companion_gps_module.sh`: f9p fleet sets `ROVER_BAUD=230400` for `ttyUSB*`, `ROVER_WIRE=ubx`; persist/load/export `ROVER_WIRE` (LC29H → nmea).
- `flush_companion_gps_from_hw.sh` / `switch_EAUSB_DAUART.sh`: comments + `--f9p` prefers present `ttyUSB0` when no ACM.

## [2026-08-04] S5 XY wander: mockup OF+GPS fight; S5 RECAPTURE
- Cause: `EKF2_OF_CTRL=1` with Gazebo mockup OF + GPS fused → EKF XY drifts metres in S5; Python lacked CPP 8 m hold RECAPTURE so VelocityPID chased jumps.
- Fix: `.post` boots `EKF2_OF_CTRL=0` (OF still streamed); HA GPS-quality NO FIX sets OF on / GPS off; PY S5 RECAPTURE err>8 m. Restart sim + HA `zmq_commands_mavlink` + Python SMs; Land/Restart/Takeoff.

## [2026-08-04] Sim OF/ranger from 2 cm (match real air)
- Cause: `.post` set `SENS_FLOW_MINHGT=0.7` and stock `model://lidar` clamps at 0.2 m — blocked low-AGL OF/ranger while real air works from ~2 cm; S5 XY wandered (Gazebo drift >1 m/5 s on D2).
- Fix: `SENS_FLOW_MINHGT=0.02`; `inject_iris_sensors` inlines lidar with `min_distance`/`ray min` 0.02. Restart `./runSimNoeticMulti.sh --num=4` (lidar needs respawn; param also in `.post`).

## [2026-08-04] Ranger/OF: offboard streams + Gazebo loopback
- Cause: `.post` streamed `DISTANCE_SENSOR`/`OPTICAL_FLOW_RAD` only on GCS (`udp_gcs_port_local`); HA uses offboard `14580+i→14540+i`. Also gzserver multicast `Network is unreachable` → later `simulator_mavlink poll timeout` froze HIL (`bottom_clearance=-1`).
- Fix: `.post` also streams both on `$udp_offboard_port_local`; `runSimNoeticMulti.sh` sets `GAZEBO_IP=127.0.0.1` + `GAZEBO_MASTER_URI=http://127.0.0.1:11345`. Restart sim via `./runSimNoeticMulti.sh --num=4`.

## [2026-08-02] Final-review fixes: OF mockup default, GPS boot params, C bridges flag
- **I1:** `inject_iris_sensors.py` defaults to `libgazebo_opticalflow_mockup_plugin.so` (velocity+lidar; works headless). `CATSWARM_OF_MODE=camera|both` for the OpenCV `model://px4flow` plugin (needs GPU/X11). Until `OPTICAL_FLOW_RAD` is confirmed on the operator display, treat OB **NO FIX** as estimator-loss risk (GPS off + no flow), not safe GPS denial.
- **I2:** `run_multidrone_bridges.sh` no longer passes `--zmqFlightData` to the C `zmq_commands_mavlink` (unknown option → rc=1). Keep `--zmqFlightData` on Python `hardware_adapter_multi.sh` only.
- **I3:** airframe `.post` now `param set EKF2_GPS_CTRL 7` + `SYS_HAS_GPS 1` at boot (matches HA `_GPS_ON`; undoes persisted NO FIX). **`SYS_HAS_GPS` is reboot-required** — boot-time set is the recovery path; mid-sim NO FIX still relies on live `EKF2_GPS_CTRL=0`.
- Tests: `multidrone/tests/test_inject_iris_sensors.py` (mockup default + camera/both).

## [2026-08-02] `sitl_multiple_run.sh`: spawn model before starting px4 (OF boot-order fix)
- Cause (Task 2 finding, fixed in Task 7): `spawn_model` started `bin/px4` in the background **before** running jinja-gen/inject/`gz model --spawn-file`. PX4's `Sensors::init()` calls `InitializeVehicleOpticalFlow()` exactly once at boot and only wires up `OPTICAL_FLOW_RAD` if `sensor_optical_flow` is already `.advertised()` (i.e. already published at least once) at that instant — starting px4 first guaranteed the model/px4flow didn't exist yet, so the check always failed for the whole session.
- Fix: reordered so jinja-gen → inject → `gz model --spawn-file` all run **before** `bin/px4` is started, matching the reference single-instance `sitl_run.sh`.
- Camera OF under headless/Xvfb still may not publish; prefer mockup inject (entry above). `DISTANCE_SENSOR` streams correctly. Details: `ObservationBoard/.superpowers/sdd/task-7-report.md`.

## [2026-08-01] Airframe `.post` + bake multi-SITL sensors into Noetic image
- `multidrone/airframes/10015_gazebo-classic_iris.post`: OF stream/params + GPS-on boot params (see 2026-08-02 entry) + takeoff params; kept `DISTANCE_SENSOR` stream.
- `Dockerfiles/PX4NoeticSimNvidia.dockerfile` bakes inject / `sitl_multiple_run.sh` / `.post`. Build context = `general_infrastructure/` (see `Dockerfiles/PX4_noetic_sim_build.sh`). Details: `Dockerfiles/UPDATES.md` 1.4.0.

## [2026-08-01] Multi-SITL iris: sensors inject
- `inject_iris_sensors.py`: lidar always; OF via mockup (default) or `model://px4flow` (`CATSWARM_OF_MODE`). Compat wrapper: `inject_iris_lidar.py`. Tests: `multidrone/tests/test_inject_iris_sensors.py`.

## [2026-07-31] Noetic Dockerfile: PX4 v1.17.0
- `Dockerfiles/PX4NoeticSimNvidia.dockerfile` pins `v1.17.0` (`ARG PX4_TAG`); explicit `ninja … px4` + `sitl_gazebo-classic`. Rebuild: `Dockerfiles/./PX4_noetic_sim_build.sh`.

## [2026-07-31] Multi-SITL iris: downward Gazebo lidar (DISTANCE_SENSOR)
- `multidrone/inject_iris_lidar.py` injects nested `model://lidar` (not an inline link — topic must be `lidar`).
- `sitl_multiple_run.sh` runs inject after jinja; `runSimNoeticMulti.sh` mounts inject + `airframes/10015_gazebo-classic_iris.post` (streams DISTANCE_SENSOR @ 20 Hz).
- Verified: mavlink msgid 132 @ ~0.19 m AGL on 14541–14544; HA FLIG `bottom_clearance_m=0.19`. Restart sim (`./runSimNoeticMulti.sh --num=4`); SM `takeoff_height_source=rangefinder`.

## [2026-07-27] F9P→PX4: 1 Hz meas rate for flight
- Fleet profile `f9p` uses `DA_RATE_MS=1000` (was 100 / 10 Hz) to avoid high-rate noise in flight.
- PX4 stay-alive still via emulate GGA heartbeat (~450 ms).

## [2026-07-27] Boot: recognize F9P — stop 30s USB0 wait before tmux
- `run-companion-drone.sh` mapped f9p→ea and waited on `/dev/ttyUSB0` (up to 30s) before
  creating `catswarm_sim` — looked like “tmux never started” with only F9P plugged.
- Wait ACM for f9p (brief 10s); required UARTs only (skip blocking on optional AMA4).
- `companion_gps_ensure_ports` migrate no longer defaults module to ea over a saved F9P.

## [2026-07-27] A/B: FC reboot fixed GPS; F9P+LC29H both OK on AMA0
- GPS_1_CONFIG debug sweeps wedged FC GPS until soft reboot (wire was fine).
- Both modules plugged: EA then F9P each produce GPS_RAW fix_type=3 → FLIG gps≠0.

## [2026-07-27] F9P→PX4: synth RMC + 10 Hz; `--f9p` port flush
- F9P often lacks steady RMC (PX4 needs GGA+RMC); `emulate_gps_to_px4` synthesizes RMC from GGA.
- F9P CFG: GSV off + RMC all interfaces; fleet F9P rate 10 Hz; `switch --f9p` flushes ACM0@115200.
- If FC still has no GPS_RAW while GPIO14 TX passes: check physical Pi header pin 8 → FC GPS1 RX.

## [2026-07-27] F9P→PX4: 10 Hz + GGA/RMC-only NMEA bridge
- Root cause: F9P at 1 Hz with GSA/GSV flooded AMA0; PX4 never set GPS-present / GPS_RAW.
- `rover_zmq` forwards only GGA/RMC on `--nmea-zmq-bind`; F9P fleet default `DA_RATE_MS=100`.
- `switch_EAUSB_DAUART.sh --f9p` flushes ACM0@115200 (was saving EA USB0 defaults).

## [2026-07-27] Companion GPS boot: prefer saved, else sniff+persist
- `companion_gps_boot_resolve_available`: if saved rover tty missing, sniff F9P→EA→DA→UART,
  flush `~/.config/companion-gps`, then start that profile (no more silent GPS skip when USB0
  LC29H is present but saved F9P ACM is not).
- `start_companion_drone_tmux.sh` `start_gps_combo` uses the resolver; adds `--f9p`; topology
  labels F9P vs EA correctly.

## [2026-07-25] Fix Noble image FG prebuild (no sitl launch)
- Root cause: `make … flightgear_rascal` always runs `sitl_run.sh`/`fgfs`; `DONT_RUN=1` does not skip FG path.
- Dockerfile now builds `flightgear_bridge` via ninja only; Rascal launch remains runtime (`fixedwing/runSimFlightGearRascal.sh`).

## [2026-07-25] Fixed-wing FlightGear Rascal host helpers
- Noble image: pre-build nolockstep + `flightgear_bridge` (not the launch target).
- `fixedwing/fg_spawn.env` + `runSimFlightGearRascal.sh`: injectable in-air spawn (500 m, ~30 m/s).
- `fixedwing/run_straight_flight.py`: start sim + OFFBOARD body-forward velocity hold.

## [2026-07-25] Drop --disable-rembrandt for FG 2024
- FG 2024 rejects `disable-rembrandt` (GUI dialog + usage dump); removed via `Dockerfiles/patch_px4_flightgear_sitl.sh`.
- `Dockerfiles/` now has its own `UPDATES.md`; README documents Rascal SITL + FG 2024 pitfalls (MAVLink race, rembrandt, fgfs/dbus libs).

## [2026-07-25] PX4 FG SITL patches (Rascal + FG 2024)
- `patch_px4_flightgear_sitl.sh`: add omitted `1039_flightgear_rascal` to airframes CMakeLists; patch `FG_run.py` (disable TerraSync, drop duplicate `model-hz`, dedupe CLI for FG 2024, keep both `--generic`).
- Wired into `PX4NobleSimNvidia.dockerfile` after PX4 v1.17.0 checkout.

## [2026-07-25] Fix fgfs wrapper for SITL + FlightGear
- `fgfs` no longer exports AppImage `LD_LIBRARY_PATH` into `dbus-run-session` (broke `fgfs --version`).
- Documented `make px4_sitl_nolockstep flightgear_rascal` build/run flow (build px4 before bridge if MAVLink NOTFOUND).

## [2026-07-25] Dockerfiles README: Noble / FlightGear usage
- Replaced stale VS Code-only `Dockerfiles/README.md` with Noble/Noetic image table, build/run scripts, FlightGear start notes.

## [2026-07-25] PX4 sim image: Ubuntu 24.04 (Noble) + Gazebo Jetty + FlightGear 2024
- Renamed Jammy sim Docker assets to Noble (`PX4NobleSimNvidia.dockerfile`, `runSimNoble.sh`, image tag `px4-noble-sim-ros`).
- Base `nvidia/cuda:13.1.2-base-ubuntu24.04`; Gazebo `gz-jetty`; PX4 `v1.17.0`; FlightGear 2024.1.6 AppImage + data pack.
- Build+runtime smoke verified: UFO@KSFO under `dbus-run-session`+`xvfb`.

## [2026-07-21] Deploy: UART layout v3 + GPIO14 TX self-test
- Strip conflicting `dtparam=uart0=on`; probe point is header pin 8 (GPIO14), not legacy UART4 pin 32.
- `verify_px4_nmea_uart_tx.sh` samples GPIO14 during live NMEA/burst; hooked from `phase_peripherals`.

## [2026-07-21] Deploy: idempotent peripherals phase (UART layout v2)
- Soft Update previously skipped UART rewrite once an old marker existed, leaving stale boot config.
- `ensure_fleet_uart_boot_config` rewrites to `fleet-uart-layout: 2` (uart0=NMEA→PX4); new `--phase=peripherals`.
- ObservationBoard Update always runs the peripherals phase plus companion-gps ensure.

## [2026-07-20] Deploy: skip install when SystemManagerMain already in place
- CMake install of `SystemManagerMain` failed when built path equaled dest; now chown/chmod in place instead.

## [2026-07-20] Deploy: companion apt package set + offline-debs
- Canonical `COMPANION_APT_SYSTEM`/`COMPANION_APT_BUILD` lists; `phase_build` always ensures build packages.
- Offline fallback via `offline-debs/` + `fetch-offline-debs.sh`; OB rsyncs debs with deploy scripts.

## [2026-07-20] Deploy build: install Eigen3 for SystemManagerCPP
- Soft Update `phase_build` failed on Pis without Eigen; `phase_system` now installs `libeigen3-dev`.
- `phase_build` installs it only if missing (offline-safe); `libcppzmq-dev` optional (vendored header).

## [2026-07-20] Deploy/boot: ensure PX4 NMEA UART0 before reboot
- `ensure_companion_uart_ports.sh` migrates saved `COMPANION_PX4_GPS_PORT` ttyAMA4→ttyAMA0, keeps DA off UART0.
- Runs from companion boot wrapper, deploy `phase_verify`, and ObservationBoard `deploy_pi5` before reboot.

## [2026-07-20] Pi NMEA→PX4 on UART0 + EA 10 Hz
- `startup_scripts`: NMEA→PX4 default `/dev/ttyAMA0` (GPIO14 TX); optional DA UART default `/dev/ttyAMA4`.
- Rover init phase-2 docs updated to 10 Hz; UART tables in README/SCRIPTS.md updated.

## Historical summary
- **MAVLink multi-drone control** (`mavlinkTakeoffLandAlt.py`, `mavlinkSwitchToMode.py`, `drone_control.sh`): added dynamic port config for shared (14550) vs per-drone (14540+i/14030+i) links, SysID-aware targeting via heartbeat wait, telemetry-polling feedback loops replacing fixed sleeps, closed-loop altitude control with early exit, and authoritative flightmode string reporting.
- **ZMQ-based command tooling** (`simpleZMQtakeoffland.py`, `zmqTakeoffLandAlt.py`, `takeoffland.py`): unified `--takeoff`/`--land`/`--altitude` CLI across scripts, added 10 Hz OFFBOARD position-setpoint control loop (`quadPosNedCmd`/`quadModeCmd`), renamed/added ZMQ port args, improved command reliability with LINGER + post-send delay.
- **Hardware adapter** (`hardware_adapter.py`/`.sh`, `hardware_adapter_multi.sh`, `command_queue.h`, `zmq_topics.h`, `zmq_commands_mavlink.c`): switched to client-mode command uplink targeting each drone's port/SysID, added position/mode command types and handlers, refactored to async send/receive threads.
- **Multi-drone orchestration**: `run_multidrone_bridges.sh` parses `positions.txt` to spawn a tmux session with paired MAVLink↔ZMQ bridges per drone; `runSimNoeticMulti.sh` supports mounting custom `positions.txt`; parallel CatSwarm GUI work added `.csm` project support and table-based initial-position config.
- **Project bootstrap**: initial `README.md`/`UPDATES.md`; `sysid.py` converted from notebook to standalone script; `system_manager.py` implemented drift-corrected 100 Hz loop timing.
