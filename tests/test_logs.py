import logging
import time
import warnings

import numpy as np
from test_camera import pattern, sessions, wait_until  # noqa: F401 - sessions is a fixture

from xblick.camera import CameraPublisher, ImageSubscriber
from xblick.diagnostics import DiagnosticsPublisher
from xblick.logs import setup
from xblick.msg import CameraInfo, DiagnosticArray
from xblick.rgbd import CameraSpec, RGBDInputs


def test_setup_hides_what_says_nothing(capsys):
    try:
        with warnings.catch_warnings(record=True) as caught:  # the filters restored afterwards
            warnings.simplefilter("default")  # what a fresh interpreter does; setup's filters go before it
            log = setup("xblick-test", ignore=["xFormers is not available"])
            warnings.warn("old API", FutureWarning, stacklevel=1)
            warnings.warn("xFormers is not available (Block)", UserWarning, stacklevel=1)
            warnings.warn("the depth is empty", UserWarning, stacklevel=1)
        assert [str(w.message) for w in caught] == ["the depth is empty"]
        log.info("ready")
        logging.getLogger("xblick").info("robot/x/color: publisher found")
        out = capsys.readouterr().out
        assert "xblick-test: ready\n" in out and "xblick-test: robot/x/color: publisher found\n" in out
    finally:
        logging.getLogger().handlers.clear()  # it wrote to the captured stdout


def test_describe_and_waiting(sessions):  # noqa: F811
    """A subscription says what it waits for; a camera's inputs say which of them it is."""
    a, b = sessions
    spec = CameraSpec.parse("robot/camera:depth=depth_to_color")
    inputs = RGBDInputs(b, spec, 4, "shm", __import__("threading").Event())
    assert inputs.waiting() == ("robot/camera/color: no publisher answers robot/camera/color/info yet; "
                                "robot/camera/depth_to_color: no publisher answers "
                                "robot/camera/depth_to_color/info yet")
    info = CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="cam")
    color = CameraPublisher(a, "robot/camera/color", encoding="rgb8", frame_id="cam", camera_info=info)
    wait_until(lambda: "shm" in inputs.readers["color"].subscriber.describe())
    assert inputs.readers["color"].subscriber.describe() == "shm (asked for), no frame yet"
    wait_until(lambda: color.subscribed)
    color.publish(pattern(), stamp_ns=1)
    wait_until(lambda: inputs.readers["color"].subscriber.received == 1)
    assert inputs.waiting().startswith("robot/camera/color: shm (asked for), 1 frames since it was chosen; ")
    depth = CameraPublisher(a, "robot/camera/depth_to_color", encoding="16UC1", frame_id="cam",
                            camera_info=info, codec="png")
    wait_until(lambda: depth.subscribed)
    depth.publish(np.ones((48, 64), np.uint16), stamp_ns=2)  # another stamp than the colour's
    wait_until(lambda: "no colour frame has its partners" in inputs.waiting())
    inputs.close()
    color.close()
    depth.close()
    sub = ImageSubscriber(b, "robot/nobody/color", transport="auto")
    assert sub.describe() == "no publisher answers robot/nobody/color/info yet"
    sub.close()


def test_diagnostics_publisher(sessions):  # noqa: F811
    a, b = sessions
    got = []
    subscriber = b.declare_subscriber("robot/camera/diagnostics",
                                      lambda s: got.append(DiagnosticArray.deserialize(s.payload.to_bytes())))
    diagnostics = DiagnosticsPublisher(a, "robot/camera", "nemo")
    time.sleep(0.3)
    diagnostics.publish(DiagnosticsPublisher.WARN, "waiting: robot/camera/color", {"frames/s": 0.0, "objects": 2})
    wait_until(lambda: got)
    (status,) = got[0].status
    assert status.name == "nemo" and status.level == DiagnosticsPublisher.WARN
    assert [(kv.key, kv.value) for kv in status.values] == [("frames/s", "0.00"), ("objects", "2")]
    diagnostics.close()
    subscriber.undeclare()
