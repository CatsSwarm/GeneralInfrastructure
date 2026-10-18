from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RUN = (ROOT / "runSimNoeticMulti.sh").read_text()
DOCKER = (ROOT / "Dockerfiles" / "PX4NoeticSimNvidia.dockerfile").read_text()
MOUNT = (
    '--volume="${SCRIPT_DIR}/calibration/models/iris/iris.sdf.jinja:'
    "/home/valentin/PX4-Autopilot/Tools/simulation/gazebo-classic/"
    'sitl_gazebo-classic/models/iris/iris.sdf.jinja:ro"'
)

def test_runsim_mounts_host_iris_jinja_in_both_volume_lists():
    assert RUN.count(MOUNT) == 2
    assert "docker cp" not in RUN
    assert "iris.sdf.jinja" not in DOCKER
