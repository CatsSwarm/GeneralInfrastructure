from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SITL = ROOT / "multidrone" / "sitl_multiple_run.sh"
RUN = ROOT / "runSimNoeticMulti.sh"


def test_sitl_appends_catswarm_host_after_setup():
    text = SITL.read_text()
    i_src = text.index("setup_gazebo.bash")
    i_host = text.index("models/catswarm_host")
    assert i_src < i_host
    assert "GAZEBO_MODEL_PATH" in text


def test_runsim_help_and_world_flag():
    import subprocess
    out = subprocess.check_output(["bash", str(RUN), "--help"], text=True)
    assert "--world" in out
    assert "--vio-cam" in out
    assert "--vio-pitch" in out
    assert "CATSWARM_VIO_PITCH" in RUN.read_text()


def test_runsim_missing_world_exits_before_docker(tmp_path):
    import subprocess
    proc = subprocess.run(
        ["bash", str(RUN), "--world", "definitely_missing_world_xyz", "--num", "1"],
        cwd=str(ROOT), capture_output=True, text=True, timeout=15,
    )
    assert proc.returncode != 0
    assert "generate_real_area.py" in (proc.stdout + proc.stderr)


def test_runsim_origin_gate_skips_cylinders():
    text = RUN.read_text()
    # The required-origin block must not treat cylinders as a host map.
    gate = text.split("if [[ \"$WORLD\" != \"empty\"")[1].split("fi")[0]
    assert "cylinders" in gate


def test_runsim_cylinders_is_empty_like_and_has_radius_flag():
    import subprocess
    out = subprocess.check_output(["bash", str(RUN), "--help"], text=True)
    assert "--cylinder-radius" in out
    text = RUN.read_text()
    assert "CATSWARM_CYLINDER_RADIUS" in text
    assert "CATSWARM_WORLD" in text
    # cylinders must not share the origin.json required-file path with teradyon
    assert 'WORLD" != "cylinders"' in text or "cylinders" in text
    sitl = SITL.read_text()
    assert "spawn_cylinders" in sitl
    assert "CATSWARM_WORLD" in sitl


def test_runsim_starts_host_rc_radio_and_kills_it_on_cleanup():
    text = RUN.read_text()
    start = text.index("sim_rc_radio.py")
    docker = text.index("docker run ")
    assert start < docker
    assert "--num" in text[start:docker]
    cleanup = text.split("cleanup_on_exit()")[1].split("Default values")[0]
    assert "sim_rc_radio.py" in cleanup
    assert "pkill" in cleanup
    launch = text.index('sim_rc_radio.py" --num')
    assert launch < docker
    after_trap = text.split("trap cleanup_on_exit EXIT INT TERM", 1)[1]
    assert after_trap.index("cleanup_on_exit") < after_trap.index('sim_rc_radio.py" --num')
    real_docker = text.index('docker run "${DOCKER_TTY[@]}"')
    assert "sim_rc_radio.py" not in text[real_docker:]
