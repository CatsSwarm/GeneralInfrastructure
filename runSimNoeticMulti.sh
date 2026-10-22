#!/bin/bash
# Script to start PX4 multi-drone simulation in Docker with a single command
# This script starts the Docker container and automatically runs the simulation
# based on positions defined in positions.txt

# Get script directory to make paths invariant to project location
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=docker_sudo.sh
source "${SCRIPT_DIR}/docker_sudo.sh"
# shellcheck source=multidrone/write_sitl_spawn_map.sh
source "${SCRIPT_DIR}/multidrone/write_sitl_spawn_map.sh"

CONTAINER_NAME="px4-noetic-sim-ros"

# Cleanup function to be called on script exit or --kill
cleanup_on_exit() {
    echo ""
    echo "Cleaning up..."
    
    # Remove container
    echo "Removing container..."
    catswarm_docker rm -f ${CONTAINER_NAME} 2>/dev/null || true
    
    # Clean up X11
    echo "Cleaning up X11 resources..."
    xhost -local:docker 2>/dev/null || true
    xhost - 2>/dev/null || true
    rm -f /tmp/.X*-lock 2>/dev/null || true
    
    # Clean up processes
    echo "Killing related processes..."
    pkill -x px4 2>/dev/null || true
    pkill gzclient 2>/dev/null || true
    pkill gzserver 2>/dev/null || true
    pkill -f "gz master" 2>/dev/null || true
    pkill -f "gazebo.*master" 2>/dev/null || true
    
    # Kill process on port 11345 if any
    if lsof -ti:11345 >/dev/null 2>&1; then
        echo "Killing process using Gazebo master port 11345..."
        lsof -ti:11345 | xargs kill -9 2>/dev/null || true
    fi
}

# Default values
NUM_DRONES=1
POSITIONS_FILE="${SCRIPT_DIR}/multidrone/positions.txt"
WORLD=empty
CYLINDER_RADIUS=10
VIO_CAM=1
VIO_PITCH=-90

# Argument parsing
while [[ $# -gt 0 ]]; do
    case "$1" in
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Script to start PX4 multi-drone simulation in Docker."
            echo ""
            echo "Options:"
            echo "  --help, -h               Show this help message and exit"
            echo "  --kill                   Clean up/kill existing simulation containers and processes"
            echo "  --num=N, --num N         Number of drones to spawn (default: 1)"
            echo "  --file=PATH, --file PATH Path to positions file (default: multidrone/positions.txt)"
            echo "  --world=NAME, --world NAME  Gazebo world (default: empty). Host maps in Dockerfiles/models/NAME/"
            echo "  --cylinder-radius=R, --cylinder-radius R  Cylinders disk radius meters (default: 10)"
            echo "  --vio-cam=0|1, --vio-cam 0|1  Inject Gazebo VIO camera + vio_cam_tcp (default: 1)"
            echo "  --vio-pitch=DEG, --vio-pitch DEG  FRD camera pitch degrees [-90, 0] baked into iris SDF (default: -90)"
            echo ""
            exit 0
            ;;
        --kill)
            echo "Kill command received."
            cleanup_on_exit
            exit 0
            ;;
        --num=*)
            NUM_DRONES="${1#*=}"
            shift
            ;;
        --num)
            NUM_DRONES="$2"
            shift 2
            ;;
        --file=*)
            POSITIONS_FILE="${1#*=}"
            shift
            ;;
        --file)
            POSITIONS_FILE="$2"
            shift 2
            ;;
        --world=*)
            WORLD="${1#*=}"
            shift
            ;;
        --world)
            WORLD="$2"
            shift 2
            ;;
        --cylinder-radius=*)
            CYLINDER_RADIUS="${1#*=}"
            shift
            ;;
        --cylinder-radius)
            CYLINDER_RADIUS="$2"
            shift 2
            ;;
        --vio-cam=*)
            VIO_CAM="${1#*=}"
            shift
            ;;
        --vio-cam)
            VIO_CAM="$2"
            shift 2
            ;;
        --vio-pitch=*)
            VIO_PITCH="${1#*=}"
            shift
            ;;
        --vio-pitch)
            VIO_PITCH="$2"
            shift 2
            ;;
        [0-9]*)
            NUM_DRONES="$1"
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information."
            exit 1
            ;;
    esac
done

if [[ "$WORLD" != "empty" && "$WORLD" != "cylinders" ]]; then
    HOST_MODELS="${SCRIPT_DIR}/Dockerfiles/models"
    WORLD_FILE="${HOST_MODELS}/${WORLD}/${WORLD}.world"
    ORIGIN_FILE="${HOST_MODELS}/${WORLD}/origin.json"
    if [[ ! -f "$WORLD_FILE" || ! -f "$ORIGIN_FILE" ]]; then
        echo "World '${WORLD}' not found (need ${WORLD_FILE} and origin.json)."
        echo "Generate it first, e.g.:"
        echo "  python3 ${SCRIPT_DIR}/Dockerfiles/scripts/generate_real_area.py --lat 32.869354 --lon 35.274463 --radius-m 500 --out ${SCRIPT_DIR}/Dockerfiles/models/teradyon"
        exit 1
    fi
    PX4_HOME_LAT=$(python3 -c "import json; print(json.load(open('${ORIGIN_FILE}'))['lat'])")
    PX4_HOME_LON=$(python3 -c "import json; print(json.load(open('${ORIGIN_FILE}'))['lon'])")
    PX4_HOME_ALT=$(python3 -c "import json; print(json.load(open('${ORIGIN_FILE}'))['alt_amsl'])")
fi

# Set trap to cleanup on script exit
trap cleanup_on_exit EXIT INT TERM

# Initial cleanup to ensure clean state
cleanup_on_exit

# Setup X11 forwarding
xhost + 2>/dev/null || true
xhost +local:docker 2>/dev/null || true

# Verify display is accessible
if [ -z "$DISPLAY" ]; then
    echo "Warning: DISPLAY environment variable is not set"
    export DISPLAY=:0
fi

# Set XAUTHORITY if not set
if [ -z "$XAUTHORITY" ] && [ -f "$HOME/.Xauthority" ]; then
    export XAUTHORITY="$HOME/.Xauthority"
fi

# Build docker run command with conditional XAUTHORITY volume
XAUTH_FILE="${XAUTHORITY:-$HOME/.Xauthority}"

# Use the image PX4 tree. Do not mount a host catkin checkout over
# /home/valentin/PX4-Autopilot: that replaces build/px4_sitl_default/bin/px4.
# File overlays (positions, sitl_multiple_run2.sh, and for HIL worlds the
# iris camera template) still bind onto the image tree.
DEFAULT_POSITIONS="${SCRIPT_DIR}/multidrone/positions.txt"
if [ "$POSITIONS_FILE" = "$DEFAULT_POSITIONS" ]; then
    # Use default positions file location
    CONTAINER_POSITIONS_PATH="/home/valentin/PX4-Autopilot/Tools/simulation/positions.txt"
    DOCKER_VOLUMES=(
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
        --volume="${POSITIONS_FILE}:${CONTAINER_POSITIONS_PATH}:rw"
        --volume="${SCRIPT_DIR}/multidrone/sitl_multiple_run.sh:/home/valentin/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_multiple_run2.sh:rw"
        --volume="${SCRIPT_DIR}/multidrone/inject_iris_sensors.py:/home/valentin/PX4-Autopilot/Tools/simulation/inject_iris_sensors.py:ro"
        --volume="${SCRIPT_DIR}/multidrone/inject_iris_colors.py:/home/valentin/PX4-Autopilot/Tools/simulation/inject_iris_colors.py:ro"
        --volume="${SCRIPT_DIR}/multidrone/vio_cam_tcp.py:/home/valentin/PX4-Autopilot/Tools/simulation/vio_cam_tcp.py:ro"
        --volume="${SCRIPT_DIR}/multidrone/spawn_cylinders.py:/home/valentin/PX4-Autopilot/Tools/simulation/gazebo-classic/spawn_cylinders.py:ro"
        --volume="${SCRIPT_DIR}/multidrone/airframes/10015_gazebo-classic_iris.post:/home/valentin/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/10015_gazebo-classic_iris.post:ro"
        --volume="${SCRIPT_DIR}/Dockerfiles/models:/home/valentin/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/catswarm_host:ro"
    )
else
    # Use custom positions file
    CONTAINER_POSITIONS_PATH="/tmp/custom_positions.txt"
    DOCKER_VOLUMES=(
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
        --volume="${SCRIPT_DIR}/multidrone/positions.txt:/home/valentin/PX4-Autopilot/Tools/simulation/positions.txt:rw"
        --volume="${POSITIONS_FILE}:${CONTAINER_POSITIONS_PATH}:ro"
        --volume="${SCRIPT_DIR}/multidrone/sitl_multiple_run.sh:/home/valentin/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_multiple_run2.sh:rw"
        --volume="${SCRIPT_DIR}/multidrone/inject_iris_sensors.py:/home/valentin/PX4-Autopilot/Tools/simulation/inject_iris_sensors.py:ro"
        --volume="${SCRIPT_DIR}/multidrone/inject_iris_colors.py:/home/valentin/PX4-Autopilot/Tools/simulation/inject_iris_colors.py:ro"
        --volume="${SCRIPT_DIR}/multidrone/vio_cam_tcp.py:/home/valentin/PX4-Autopilot/Tools/simulation/vio_cam_tcp.py:ro"
        --volume="${SCRIPT_DIR}/multidrone/spawn_cylinders.py:/home/valentin/PX4-Autopilot/Tools/simulation/gazebo-classic/spawn_cylinders.py:ro"
        --volume="${SCRIPT_DIR}/multidrone/airframes/10015_gazebo-classic_iris.post:/home/valentin/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/10015_gazebo-classic_iris.post:ro"
        --volume="${SCRIPT_DIR}/Dockerfiles/models:/home/valentin/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/catswarm_host:ro"
    )
fi

# HIL worlds: iris_1 camera template (20 Hz HEVC), local props, city assets.
# x265enc is the encoder. nvh265enc stays off unless CATSWARM_GST_NVENC=1.
if [ "$WORLD" = "simple_hil" ] || [ "$WORLD" = "hil_city" ]; then
    CITY_ASSETS="${SCRIPT_DIR}/multidrone/city_assets"
    HIL_WORLDS="${SCRIPT_DIR}/multidrone/worlds"
    NEED_CITY_ASSETS=0
    if [ "${WORLD}" = "hil_city" ] && [ ! -f "${CITY_ASSETS}/worlds/hil_city.world" ]; then
        NEED_CITY_ASSETS=1
    fi
    if [ "${WORLD}" = "simple_hil" ] && [ ! -d "${CITY_ASSETS}/models/house_1" ]; then
        NEED_CITY_ASSETS=1
    fi
    if [ "${NEED_CITY_ASSETS}" = "1" ] && [ -x "${SCRIPT_DIR}/multidrone/fetch_hil_city.sh" ]; then
        echo "City models missing; fetching OSRF citysim assets (once)…"
        "${SCRIPT_DIR}/multidrone/fetch_hil_city.sh" || true
    fi
    DOCKER_VOLUMES+=(
        --volume="${SCRIPT_DIR}/multidrone/iris.sdf.jinja:/home/valentin/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.jinja:ro"
        --volume="${SCRIPT_DIR}/multidrone/models:/home/valentin/catswarm_models:ro"
        --volume="${HIL_WORLDS}:/home/valentin/catswarm_worlds:ro"
    )
    GST_CAMERA_PLUGIN="${SCRIPT_DIR}/../vision_hil/host/gst_camera_plugin/libgazebo_gst_camera_plugin.so"
    if [ -f "${GST_CAMERA_PLUGIN}" ]; then
        DOCKER_VOLUMES+=(
            --volume="${GST_CAMERA_PLUGIN}:/home/valentin/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic/libgazebo_gst_camera_plugin.so:ro"
        )
        echo "GstCameraPlugin: bind-mount ${GST_CAMERA_PLUGIN}"
    fi
    GST_NVENC_PLUGIN="${SCRIPT_DIR}/../vision_hil/host/gst_nvenc/plugin"
    if [ "${CATSWARM_GST_NVENC:-0}" = "1" ] && [ -f "${GST_NVENC_PLUGIN}/libgstnvenc.so" ]; then
        DOCKER_VOLUMES+=(
            --volume="${GST_NVENC_PLUGIN}:/opt/catswarm/gst:ro"
        )
        echo "GstNvenc: GST_PLUGIN_PATH=/opt/catswarm/gst (${GST_NVENC_PLUGIN})"
    else
        echo "GstNvenc: skipped (CATSWARM_GST_NVENC=${CATSWARM_GST_NVENC:-0}); encoder is x265enc"
    fi
    if [ -d "${CITY_ASSETS}/models" ]; then
        DOCKER_VOLUMES+=(--volume="${CITY_ASSETS}:/home/valentin/catswarm_city:ro")
    fi
    echo "Gazebo world: ${WORLD}"
    NUM_DRONES="${NUM_DRONES}" POSITIONS_FILE="${POSITIONS_FILE}" CATSWARM_WORLD="${WORLD}" write_sitl_spawn_map
fi

# Add XAUTHORITY volume only if file exists
if [ -f "$XAUTH_FILE" ]; then
    DOCKER_VOLUMES+=(--volume="${XAUTH_FILE}:${XAUTH_FILE}:ro")
fi

PX4_HOME_ENV=()
if [[ "$WORLD" != "empty" && "$WORLD" != "cylinders" ]]; then
    DOCKER_VOLUMES+=(--volume="${WORLD_FILE}:/home/valentin/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/${WORLD}.world:ro")
    PX4_HOME_ENV=(
        --env="PX4_HOME_LAT=${PX4_HOME_LAT}"
        --env="PX4_HOME_LON=${PX4_HOME_LON}"
        --env="PX4_HOME_ALT=${PX4_HOME_ALT}"
    )
fi

# Pin Gazebo transport to loopback. Without this, gzserver floods
# "Exception sending a multicast message: Network is unreachable" on hosts
# with no working multicast route; after a while simulator_mavlink poll
# timeouts freeze HIL (DISTANCE_SENSOR / OF / IMU stop → FLIG bottom_clearance=-1).
DOCKER_GPU=()
if [ "$WORLD" = "simple_hil" ] || [ "$WORLD" = "hil_city" ]; then
    if [ "${CATSWARM_SIM_GPU:-1}" != "0" ] && command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; then
        DOCKER_GPU=(--gpus all)
        if command -v nvidia-settings >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
            # Adaptive PowerMizer parks the Ada GPU. 1080p @ 20 Hz then misses realtime.
            nvidia-settings -a '[gpu:0]/GPUPowerMizerMode=1' >/dev/null 2>&1 || true
            echo "NVIDIA PowerMizer=1 (max performance)"
        fi
    fi
fi
HIL_ENV=()
if [ "$WORLD" = "simple_hil" ] || [ "$WORLD" = "hil_city" ]; then
    HIL_ENV=(
        --env="PX4_VIDEO_HOST_IP=${PX4_VIDEO_HOST_IP:-127.0.0.1}"
        --env="CATSWARM_HIL_CAM_PITCH=${CATSWARM_HIL_CAM_PITCH:-45}"
        --env="CATSWARM_GZCLIENT=${CATSWARM_GZCLIENT:-1}"
        --env="CATSWARM_GST_BITRATE=${CATSWARM_GST_BITRATE:-4000}"
        --env="CATSWARM_GST_SPEED_PRESET=${CATSWARM_GST_SPEED_PRESET:-1}"
        --env="CATSWARM_GST_NVENC=${CATSWARM_GST_NVENC:-0}"
        --env="__NV_PRIME_RENDER_OFFLOAD=1"
        --env="__GLX_VENDOR_LIBRARY_NAME=nvidia"
        --env="NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display,video"
    )
fi
catswarm_docker run -it --net=host \
           --cap-drop=all \
           --privileged \
           "${DOCKER_GPU[@]}" \
           --env="DISPLAY=$DISPLAY" \
           --env="QT_X11_NO_MITSHM=1" \
           --env="CATSWARM_OF_MODE=${CATSWARM_OF_MODE:-mockup}" \
           --env="CATSWARM_VIO_CAM=${VIO_CAM}" \
           --env="CATSWARM_VIO_PITCH=${VIO_PITCH}" \
           --env="CATSWARM_WORLD=${WORLD}" \
           --env="CATSWARM_CYLINDER_RADIUS=${CYLINDER_RADIUS}" \
           --env="GAZEBO_IP=127.0.0.1" \
           --env="GAZEBO_MASTER_URI=http://127.0.0.1:11345" \
           --env="XAUTHORITY=${XAUTH_FILE}" \
           "${HIL_ENV[@]}" \
           "${PX4_HOME_ENV[@]}" \
           "${DOCKER_VOLUMES[@]}" \
           --name=${CONTAINER_NAME} \
           ${CONTAINER_NAME} \
           /bin/bash -c "./Tools/simulation/gazebo-classic/sitl_multiple_run2.sh -p ${CONTAINER_POSITIONS_PATH} -n ${NUM_DRONES} -w ${WORLD}"

# Note: Cleanup is handled by the trap function on exit
