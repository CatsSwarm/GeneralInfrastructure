from pathlib import Path
from types import SimpleNamespace
import socket
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]
GI = ROOT.parent
sys.path.insert(0, str(ROOT))

from vio_cam_tcp import (  # noqa: E402
    _default_bind,
    _image_to_gray,
    camera_tcp_port,
    jpeg_preview_port,
    pack_svof,
    rgb_jpeg_bytes,
    rgb_to_gray,
    serve_frames,
    serve_latest_jpeg,
)


def test_port():
    assert camera_tcp_port(1) == 5600
    assert camera_tcp_port(4) == 5603


def test_gray_and_svof_header():
    rgb = bytes([255, 0, 0, 0, 255, 0, 0, 0, 255, 10, 10, 10])  # 2x2
    g = rgb_to_gray(rgb, 2, 2)
    assert len(g) == 4
    # ITU-R BT.601 Y=0.299R+0.587G+0.114B, truncated to uint8
    assert list(g) == [76, 149, 29, 10]
    blob = pack_svof(2, 2, 7, g)
    assert blob[:4] == b"SVOF"
    w, h, t_ns, nbytes = struct.unpack_from("<HHQI", blob, 4)
    assert (w, h, t_ns, nbytes) == (2, 2, 7, 4)
    assert blob[20:] == g


def test_import_does_not_load_rospy():
    assert "rospy" not in sys.modules


def test_serve_frames_binds_and_sends_svof():
    sent = bytearray()
    bound = []

    class _Conn:
        def sendall(self, data):
            sent.extend(data)

        def close(self):
            pass

    class _Srv:
        def accept(self):
            return _Conn(), ("127.0.0.1", 9)

        def close(self):
            pass

    def bind_fn(addr):
        bound.append(addr)
        return _Srv()

    gray = b"abcd"
    serve_frames(4, bind_fn=bind_fn, gray_iter=[(2, 2, 9, gray)])
    assert bound == [("0.0.0.0", 5603)]
    want = struct.pack("<4sHHQI", b"SVOF", 2, 2, 9, 4) + gray
    assert bytes(sent) == want


def test_default_bind_clears_socket_timeout():
    prev = socket.getdefaulttimeout()
    socket.setdefaulttimeout(0.05)
    try:
        srv = _default_bind(("127.0.0.1", 0))
        try:
            assert srv.gettimeout() is None
        finally:
            srv.close()
    finally:
        socket.setdefaulttimeout(prev)


def test_serve_frames_retries_after_accept_timeout():
    sent = bytearray()

    class _Conn:
        def sendall(self, data):
            sent.extend(data)

        def close(self):
            pass

    n = {"n": 0}

    class _Srv:
        def accept(self):
            n["n"] += 1
            if n["n"] == 1:
                raise socket.timeout("timed out")
            return _Conn(), ("127.0.0.1", 9)

        def close(self):
            pass

        def settimeout(self, _t):
            pass

    gray = b"abcd"
    serve_frames(1, bind_fn=lambda addr: _Srv(), gray_iter=[(2, 2, 9, gray)])
    want = struct.pack("<4sHHQI", b"SVOF", 2, 2, 9, 4) + gray
    assert bytes(sent) == want
    assert n["n"] == 2


def test_serve_frames_probe_then_second_client_receives_svof():
    """Supervisor TCP probe (connect+close) must not consume the listen socket."""
    sent = bytearray()
    accepts = []
    srv_closed = []

    class _Probe:
        def sendall(self, data):
            raise BrokenPipeError("probe close")

        def close(self):
            pass

    class _Client:
        def sendall(self, data):
            sent.extend(data)

        def close(self):
            pass

    clients = [_Probe(), _Client()]

    class _Srv:
        def accept(self):
            accepts.append(1)
            return clients.pop(0), ("127.0.0.1", 9)

        def close(self):
            srv_closed.append(1)

    gray = b"abcd"
    serve_frames(1, bind_fn=lambda addr: _Srv(), gray_iter=[(2, 2, 9, gray)])
    want = struct.pack("<4sHHQI", b"SVOF", 2, 2, 9, 4) + gray
    assert bytes(sent) == want
    assert len(accepts) == 2
    assert srv_closed == [1]


def test_sitl_restarts_gzserver_when_it_aborts():
    text = (ROOT / "sitl_multiple_run.sh").read_text()
    window = text[text.index('echo "Starting gazebo"') : text.index("Reading positions from")]
    assert "gzserver aborted on attempt" in window
    assert "gzserver failed to stay up" in window


def test_sitl_ros1_starts_roscore_and_gazebo_api_plugin():
    text = (ROOT / "sitl_multiple_run.sh").read_text()
    assert "libgazebo_ros_api_plugin.so" in text
    assert "roscore" in text


def test_sitl_exports_gazebo11_plugin_dir_for_camera():
    text = (ROOT / "sitl_multiple_run.sh").read_text()
    assert "gazebo-11/plugins" in text
    assert "LD_LIBRARY_PATH" in text




def test_sitl_gates_vio_cam_tcp_on_env():
    text = (ROOT / "sitl_multiple_run.sh").read_text()
    start = text[text.index("VIO_CAM_TCP=") :]
    assert "CATSWARM_VIO_CAM" in start
    assert 'python3 "$VIO_CAM_TCP"' in start
    assert start.index("CATSWARM_VIO_CAM") < start.index('python3 "$VIO_CAM_TCP"')


def test_run_sim_mounts_vio_cam_tcp():
    text = (GI / "runSimNoeticMulti.sh").read_text()
    assert (
        "multidrone/vio_cam_tcp.py:/home/valentin/PX4-Autopilot/Tools/simulation/vio_cam_tcp.py"
        in text
    )
    assert "CATSWARM_VIO_CAM" in text
    assert "CATSWARM_VIO_PITCH" in text
    assert "--vio-cam" in text
    assert "--vio-pitch" in text


def test_image_to_gray_uses_row_step():
    # 2x2 rgb8 with 2 pad bytes/row (step=8, width*3=6)
    data = bytes(
        [
            255, 0, 0, 0, 255, 0, 11, 22,
            0, 0, 255, 10, 10, 10, 33, 44,
        ]
    )
    msg = SimpleNamespace(
        encoding="rgb8",
        width=2,
        height=2,
        step=8,
        data=data,
    )
    assert list(_image_to_gray(msg)) == [76, 149, 29, 10]


def test_jpeg_preview_port():
    from vio_cam_tcp import jpeg_preview_port
    assert jpeg_preview_port(1) == 5700
    assert jpeg_preview_port(4) == 5703


def test_rgb_jpeg_bytes_uses_encoder():
    from vio_cam_tcp import rgb_jpeg_bytes
    rgb = bytes([255, 0, 0, 0, 255, 0])
    got = rgb_jpeg_bytes(rgb, 2, 1, encode=lambda r, w, h: b"JPEG" + bytes([w, h, len(r)]))
    assert got == b"JPEG" + bytes([2, 1, 6])


def test_serve_latest_jpeg_http():
    from vio_cam_tcp import serve_latest_jpeg
    sent = bytearray()
    bound = []

    class _Conn:
        def recv(self, n):
            return b"GET / HTTP/1.0\r\n\r\n"

        def sendall(self, data):
            sent.extend(data)

        def close(self):
            pass

    class _Srv:
        def __init__(self):
            self._n = 0

        def accept(self):
            self._n += 1
            if self._n > 1:
                raise StopIteration
            return _Conn(), ("127.0.0.1", 9)

        def close(self):
            pass

    def bind_fn(addr):
        bound.append(addr)
        return _Srv()

    try:
        serve_latest_jpeg(2, bind_fn=bind_fn, jpeg_iter=[b"\xff\xd8fake"])
    except StopIteration:
        pass
    assert bound == [("0.0.0.0", 5701)]
    text = bytes(sent)
    assert text.startswith(b"HTTP/1.0 200 OK")
    assert b"Content-Type: image/jpeg" in text
    assert text.endswith(b"\xff\xd8fake")
