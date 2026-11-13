# Your first stream

In this tutorial you publish a camera stream, look at it with the command line tools, and read it in Python. You
need an installed Durchblick and a running router, as [install](install.md) describes. The tutorial takes about
ten minutes and needs no camera and no GPU.

## 1. Publish a test camera

```bash
blick test-camera
```

The command publishes a moving test pattern with 640 x 480 pixels at 15 Hz. The name of the stream is
`robot/test_camera/color`. The options `--namespace`, `--camera`, `--stream`, `--size` and `--fps` change the name
and the format. Leave the command running and open a second terminal.

## 2. Look at what is published

```bash
blick list
```

```
robot/test_camera/color    rgb8    raw,compressed,shm    source 'test'    host 7620b5c5    durchblick 0.3.53
```

The line shows the encoding of the stream and the [transports](../concepts/transports.md) that the publisher
offers. `blick list` only queries the publishers. The test camera does no work for it.

```bash
blick hz robot/test_camera/color
```

`hz` subscribes to the stream. It prints the frame rate, the age of the frames and the number of missing frames.
The test camera starts to write its ring only now, because a publisher works only on request.

```bash
blick snapshot robot/test_camera/color frame.png
```

`snapshot` saves one frame as an image file.

## 3. Read the stream in Python

```python
import blick as db

session = db.open_session()                       # connects to the router of this PC
images = db.ImageSubscriber(session, "robot/test_camera/color", transport="auto")

frame = images.latest(timeout=1.0)                # the newest frame, or None after 1 s
print(frame.image.shape, frame.encoding, frame.age_s)

session.close()
```

`frame.image` is a NumPy array with the shape (480, 640, 3). The subscriber chose the transport `shm`, because the
publisher is on the same host. The frame came from a ring in shared memory, and no pixel travelled over the
network.

Call `latest()` again to get the next frame. `latest()` always returns the newest frame, so a slow loop skips
frames. [Latest value](../concepts/latest-value.md) explains why.

## 4. Publish your own stream

```python
import time
import numpy as np
import blick as db

session = db.open_session()
info = db.CameraInfo.pinhole(640, 480, fx=554.3, fy=554.3, cx=319.5, cy=239.5, frame_id="my_camera")
camera = db.CameraPublisher(session, "demo/my_camera/color", encoding="rgb8", frame_id="my_camera",
                            camera_info=info, source="test")

while True:
    image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    if camera.subscribed:                         # somebody reads the stream
        camera.publish(image, stamp_ns=time.time_ns())
    time.sleep(1 / 30)
```

Two points matter in this example.

- `stamp_ns` is the capture time of the image. A real camera node passes the time at which the camera exposed the
  image. [Timestamps](../concepts/timestamps.md) explains why this time matters.
- `camera.subscribed` is false while nobody reads the stream. A producer can then skip its work.

Run `blick list` again. Your stream appears beside the test camera.

## 5. Look at everything

```bash
pip install "xblick[rerun]"
blick rerun
```

`blick rerun` opens the [Rerun](https://rerun.io) viewer and shows every stream and every result that it
finds. It needs no options. The [viewer guide](../guides/viewer.md) has the details.

## Next steps

- [Your first node](first-node.md): compute a result from a stream and publish it.
- [Streams and topics](../concepts/streams-and-topics.md): how the names are built.
- [Python reference](../reference/python-api.md) and [C++ reference](../reference/cpp-api.md).
