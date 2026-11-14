# Streams in VLC or a browser

`xblick view` shows camera streams in VLC or a browser. It serves them as MJPEG over HTTP, so any MJPEG player can
show them. The server is Python's standard library; the package needs nothing more.

```bash
xblick view test/camera/color
```

```text
xblick view: serving 1 stream(s) on http://127.0.0.1:8090/ (this PC only; --http 0.0.0.0:PORT opens it to the network)
xblick view: test/camera/color: http://127.0.0.1:8090/test/camera/color.mjpg
xblick view: all of them: http://127.0.0.1:8090/
xblick view: started VLC (1 window(s))
xblick view: serving; Ctrl-C to stop
```

## Which player starts

| Call | What happens |
| --- | --- |
| `xblick view STREAM ...` | VLC, one window per stream |
| ... without VLC, on a terminal | one line that VLC is not installed, then the question `open the browser view instead? [y/N]` |
| ... without VLC, without a terminal | one line that VLC is not installed and that `--browser` opens the browser view; no question |
| `xblick view STREAM ... --browser` | the page with all streams side by side, in the system's browser |
| `xblick view STREAM ... --no-player` | no player; the addresses only |
| any of these on a system without a display | no player and no question; the addresses and how to open them from another PC |

In every case the command keeps serving until Ctrl-C.

```bash
xblick view test/camera/color                                 # VLC
xblick view test/camera/color test/camera/depth --browser     # both streams side by side in the browser
xblick view cam/video0/color --connect 192.0.2.10             # a stream of another PC
```

Put the streams before `--connect`, because `--connect` accepts several endpoints.

**VLC** is the program `vlc` on `PATH`. Install it yourself, for example with `sudo apt install vlc`. It starts with
`--network-caching=100`, a cache of 100 ms: little delay and a few frames of buffer. With several streams, VLC opens
one window per stream. The browser page shows them side by side.

**The browser** is the system's browser, as Python's `webbrowser` module finds it. The environment variable `BROWSER`
chooses another one.

**No display**: on Linux, a system on which neither `DISPLAY` (X11) nor `WAYLAND_DISPLAY` (Wayland) is set has no
display. Every graphical program needs one of the two. On such a system the command starts neither VLC nor a browser.
Python would otherwise start a text browser such as `lynx` in the terminal. It asks no question either. macOS and
Windows always count as having a display.

## What it shows

| Stream | Served as |
| --- | --- |
| JPEG on the compressed transport (colour, 8-bit grey) | the same bytes, unchanged |
| depth and 16-bit grey (RVL or PNG) | colours, encoded as JPEG (quality 85) |
| other PNG frames | encoded as JPEG (quality 85) |
| a stream without compressed transport (for example float disparity) | refused with one line |
| a point cloud | refused with one line; `xblick rerun` shows clouds |

The depth colours have a fixed range: red at 300 mm, through yellow and green, to blue at 5000 mm. Nearer is red,
farther is blue, and no depth (0) is black. The same distance has the same colour in every frame. It is the map of
`xblick.overlay.depth_colours`.

A stream that no publisher answers for within `--seconds` (2 s) is an error. The line lists the image streams that
were found.

## Addresses

| Address | Content |
| --- | --- |
| `http://127.0.0.1:8090/` | a page with every stream and its name, in a grid; no scripts and nothing from other servers |
| `http://127.0.0.1:8090/<stream>.mjpg` | one stream, `multipart/x-mixed-replace` |

The port is 8090. A port that another program uses is an error; give another one with `--http 127.0.0.1:PORT`.

## Watching from another PC

By default the server listens on 127.0.0.1, so only this PC can watch. Two ways open it to another PC:

- Forward the port with SSH and keep the default. On the other PC:

  ```bash
  ssh -L 8090:127.0.0.1:8090 robot-pc     # then open http://127.0.0.1:8090/ there
  ```

- Start with `--http 0.0.0.0:8090`. **There is no password: anyone who can reach the address can watch the
  streams.** The start line says so.

On a system without a display, the command prints both ways with its own port and host name.

## Slow clients

Each client gets the newest frame whenever it can take one. A slow client skips frames. It holds up neither the
subscriber nor the other clients. A client that takes no frame for 10 s is dropped, and a client that disconnects is
cleaned up at once. Ctrl-C ends the command, also while clients watch. VLC windows that the command started end with
it. When you close all of them yourself, the command says so in one line and keeps serving until Ctrl-C.

The [command line reference](../reference/cli.md#xblick-view-streams-) lists all options.
