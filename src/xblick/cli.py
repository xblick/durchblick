"""``xblick`` command line: look at what is published, measure a stream, save a frame, publish a test pattern.

    xblick router                                 # the Zenoh router every PC needs, instead of zenohd (router.py)
    xblick list                                   # the streams found within 2 s (--traffic: every topic)
    xblick list --connect 192.0.2.10 --zenoh-mode client  # ... of another PC, from a PC without a router
    xblick hz test/camera/color --connect …  # frame rate, latency from capture, dropped frames (or clouds)
    xblick snapshot test/camera/color out.png --connect …
    xblick test-camera                            # a moving test pattern as test/camera/color
    xblick cam /dev/video0                        # a live camera FFmpeg opens as cam/video0/color (needs xblick[cam])
    xblick cam                                    # ... chosen from this PC's cameras with the arrow keys
    xblick rerun --connect …                      # every stream and node result in the Rerun viewer
    xblick view test/camera/color                 # a stream in VLC or a browser, served as MJPEG (view.py)
    xblick diagnostics test --connect …          # how the cameras' publishers are doing (<camera>/diagnostics)
    xblick mps --pipe-dir /tmp/xblick-mps         # NVIDIA MPS for the nodes on this PC's GPU (mps.py)
    xblick record --dir ~/recordings --connect …  # everything into MCAP files until stopped (recording.py)
    xblick recorder stop                          # tell the record and play node: record, stop, status, ...
    xblick play ~/recordings/recorder_*.mcap      # the same node, playing a recording onto its topics
    xblick recorder play recorder_2026-11-02T08-22-13.531839Z.mcap --loop  # ... or told to over Zenoh
    xblick recorder record --key 'sim/*' --key robot/joint_states --blocking --log-time header --file ep.mcap
    xblick play episode.mcap --pace step          # in lockstep: xblick recorder step [--frames N]
    xblick param --nodes                          # the nodes with parameters (parameters.py)
    xblick param --node test/camera --get  # their values; --describe, --set NAME:=VALUE [--atomically]
    xblick relay --stream test/camera/color      # xblick-relay with these arguments (needs xblick[relay])
    xblick                                             # without a command: the terminal interface (tui.py)
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import shutil
import sys
import time
from pathlib import Path

import numpy as np

from . import cam, diagnostics, mps, recording, rerun_bridge, router, survey, topics, view
from . import session as zsession
from .camera import CameraPublisher, ImageSubscriber, find_streams, wait_for_info
from .msg import CameraInfo
from .node import end_on_signals
from .paths import expand_path
from .points import PointCloudSubscriber


def _list(args) -> int:
    if not args.traffic:
        return _list_streams(args)
    with zsession.open_from_args(args, prog=f"xblick {args.command}") as z:
        meter = survey.TrafficMeter(z, args.prefix.rstrip("/") + "/**" if args.prefix else "**")
        time.sleep(args.seconds)
        meter.close()
        traffic = meter.snapshot()
        why = zsession.why_nothing(z)
    if not traffic:
        print(f"nothing received within {args.seconds:g} s" + (f"; {why}" if why else ""), file=sys.stderr)
        return 1
    for key in sorted(traffic):
        t = traffic[key]
        print(f"{key:60s} {t.count / args.seconds:6.1f} Hz  {t.bytes / max(t.count, 1) / 1e3:9.1f} kB  {t.encoding}")
    return 0


def _list_streams(args) -> int:
    with zsession.open_from_args(args, prog=f"xblick {args.command}") as z:
        found = find_streams(z, args.prefix, args.seconds)
        why = zsession.why_nothing(z)
    if not found:
        print(f"no stream answered within {args.seconds:g} s" + (f"; {why}" if why else ""), file=sys.stderr)
        return 1
    for base, r in found:
        if r.get("kind") == "image":
            what = r.get("encoding", "?") + (" labels" if r.get("labels") else "")
        elif r.get("kind") == "detections":
            what = r.get("type", "?").rpartition("/")[2]
        else:
            what = ",".join(r.get("fields", ["?"]))
        print(f"{base:45s} {what:8s} {','.join(r.get('transports', [])):26s} "
              f"source {r.get('source', '')!r:8s} host {str(r.get('host'))[:8]}  xblick {r.get('xblick')}"
              + (f"  relayed on {', '.join(h[:8] for h in r['relayed_by'])}" if r.get("relayed_by") else ""))
    return 0


def _is_cloud(z, base: str) -> bool:
    """True if the stream's publisher says it is a point cloud stream (a stream that does not answer is taken as
    images; its subscriber refuses it later if it is not)."""
    return any(r.get("kind") == "points" for r in wait_for_info(z, base))


def _hz(args) -> int:
    with zsession.open_from_args(args, prog=f"xblick {args.command}") as z:
        if _is_cloud(z, args.stream):
            subscriber = PointCloudSubscriber(z, args.stream, transport=args.transport)
        else:
            subscriber = ImageSubscriber(z, args.stream, transport=args.transport)
        reported = None
        end = time.monotonic() + args.seconds if args.seconds else None
        window_start, frames, ages, sends = time.monotonic(), 0, [], []
        last_seq = None
        missing = 0
        while end is None or time.monotonic() < end:
            frame = subscriber.latest(timeout=1.0)
            now = time.monotonic()
            if frame is not None:
                frames += 1
                ages.append((frame.receive_ns - frame.stamp_ns) / 1e9)
                if frame.age_at_send_ns is not None:
                    sends.append(frame.age_at_send_ns / 1e9)
                if last_seq is not None and frame.seq is not None:
                    missing += max(frame.seq - last_seq - 1, 0)
                last_seq = frame.seq
            if subscriber.transport != reported:
                reported = subscriber.transport
                print(f"transport: {reported}")
            if now - window_start >= 1.0:
                if frames:
                    shape = "" if frame is None else getattr(frame, "image", getattr(frame, "points", None)).shape
                    send = (f"at send p50 {np.median(sends) * 1e3:6.1f} ms max {np.max(sends) * 1e3:6.1f} ms"
                            if sends else "at send -")
                    print(f"{frames / (now - window_start):5.1f} Hz  age at receive p50 {np.median(ages) * 1e3:7.1f} ms  "
                          f"max {np.max(ages) * 1e3:7.1f} ms  {send}  {shape}  skipped since start (seq gaps) "
                          f"{missing}")
                else:
                    print("no frames")
                window_start, frames, ages, sends = now, 0, [], []
        subscriber.close()
    return 0


def _snapshot(args) -> int:
    from PIL import Image as PILImage

    with zsession.open_from_args(args, prog=f"xblick {args.command}") as z:
        subscriber = ImageSubscriber(z, args.stream, transport=args.transport)
        frame = subscriber.latest(timeout=args.timeout)
        subscriber.close()
    if frame is None:
        print(f"no frame on {args.stream} within {args.timeout:g} s", file=sys.stderr)
        return 1
    image = frame.image
    if frame.encoding in ("bgr8", "bgra8"):
        image = image[..., [2, 1, 0]]
    elif frame.encoding in ("rgba8",):
        image = image[..., :3]
    elif image.dtype != np.uint8:
        print(f"{frame.encoding} is saved as 16-bit PNG without scaling", file=sys.stderr)
        image = image.astype(np.uint16)
    PILImage.fromarray(np.ascontiguousarray(image)).save(args.output)
    print(f"{args.output}: {frame.encoding} {frame.image.shape}, seq {frame.seq}, source {frame.source!r}, "
          f"frame {frame.header.frame_id!r}, age {frame.age_s * 1e3:.1f} ms")
    return 0


def test_pattern(width: int, height: int, t: float) -> np.ndarray:
    """Colour bars with a white bar moving across them (motion and latency are visible)."""
    x = np.arange(width)
    bars = np.array([[255, 255, 255], [255, 255, 0], [0, 255, 255], [0, 255, 0], [255, 0, 255], [255, 0, 0],
                     [0, 0, 255], [0, 0, 0]], np.uint8)
    row = bars[(x * len(bars)) // width]
    image = np.repeat(row[None], height, axis=0)
    position = int((t * 0.25 % 1.0) * width)
    image[:, max(position - 4, 0):position + 4] = 255
    return image


def _say_if_alone(z, prog: str):
    """A publisher's hint at its start when its session reached no router and no other session: then nobody receives
    its stream (a first-time user without zenohd). Only a hint: the program goes on as it is."""
    alone = zsession.alone(z)
    if alone:
        print(f"{prog}: nobody can receive this stream yet: {alone}", flush=True)


def _test_camera(args) -> int:
    base = topics.stream_base(args.namespace, args.camera, args.stream)
    w, h = args.size
    f = 0.5 * h / np.tan(np.radians(args.vfov) / 2)
    info = CameraInfo.pinhole(w, h, f, f, (w - 1) / 2, (h - 1) / 2, frame_id=args.frame_id)
    with zsession.open_from_args(args, prog=f"xblick {args.command}") as z:
        _say_if_alone(z, "xblick test-camera")
        publisher = CameraPublisher(z, base, encoding="rgb8", frame_id=args.frame_id, camera_info=info,
                                    source="test")
        print(f"xblick test-camera: {w}x{h} rgb8 at {args.fps:g} Hz (Ctrl-C to stop)", flush=True)
        print(f"xblick test-camera: publishing the stream {base}", flush=True)
        print(f"xblick test-camera: start a node with --camera {base.rpartition('/')[0]} (the camera, not the stream)",
              flush=True)
        period, start = 1.0 / args.fps, time.monotonic()
        try:
            while True:
                now = time.monotonic()
                publisher.publish(test_pattern(w, h, now - start))
                time.sleep(max(period - (time.monotonic() - now), 0.0))
        except KeyboardInterrupt:
            pass
        publisher.close()
    return 0


def _cam(args, parser) -> int:
    """``xblick cam``: a live camera as a stream (cam.py), its options the parameters of the node <camera>."""
    from .parameters import ParameterError, Parameters
    from .video import VideoError, camera_of

    try:
        cam.import_av()  # at once, before the session
        if args.stream is not None:
            camera_of(args.stream)  # checked before a camera is chosen
        if args.source is None:
            args.source = cam.select_source()
        camera = cam.camera_for(args.source, cam.input_format(args.source, args.format), args.stream)
        namespace, _, name = camera.rpartition("/")
    except (ImportError, VideoError, cam.CamError) as error:
        raise SystemExit(f"xblick cam: {error}") from None
    with zsession.open_from_args(args, prog=f"xblick {args.command}") as z:
        _say_if_alone(z, "xblick cam")
        params = Parameters(z, name, namespace)
        try:
            params.configure(args)
            params.declare_options(parser, args)
            params.check_overrides()
            camera = cam.LiveCamera(z, args.source, args.stream, fmt=args.format, size=args.size, fps=args.fps,
                                    pixel_format=args.pixel_format, camera_info=args.camera_info, hfov=args.hfov,
                                    vfov=args.vfov, frame_id=args.frame_id, timeout=args.timeout,
                                    reconnect=args.reconnect, options=cam.parse_options(args.option),
                                    mjpeg=args.mjpeg)
            camera.run()
        except (cam.CamError, VideoError, ParameterError, cam.ffopts.OptionError) as error:
            raise SystemExit(f"xblick cam: {error}") from None
        except KeyboardInterrupt:
            pass
        finally:
            params.close()
    return 0


def _diagnostics(args) -> int:
    key = (args.prefix.strip("/") + "/**/" if args.prefix else "**/") + topics.DIAGNOSTICS
    lines = []
    errors = []

    def on_sample(sample):
        try:
            camera, array = diagnostics.read(sample)
        except Exception as error:  # noqa: BLE001 - shown in the main thread
            errors.append(error)
            return
        clock = time.strftime("%H:%M:%S", time.localtime(array.header.stamp.to_ns() / 1e9))
        for status in array.status:
            line = f"{clock}  {camera:28s} {status.name:16s} {diagnostics.level_name(status.level):5s} {status.message}"
            if args.values:
                line += "".join(f"\n    {kv.key}: {kv.value}" for kv in status.values)
            lines.append(line)

    with zsession.open_from_args(args, prog=f"xblick {args.command}") as z:
        subscriber = z.declare_subscriber(key, on_sample)
        start = time.monotonic()
        try:
            while not args.seconds or time.monotonic() - start < args.seconds:
                time.sleep(0.2)
                while lines:
                    print(lines.pop(0), flush=True)
                if errors:
                    raise errors[0]
        except KeyboardInterrupt:
            pass
        finally:
            subscriber.undeclare()
    return 0


def _bridged(found: list[tuple[str, dict]]) -> dict[str, list[str]]:
    """What find_streams found, as RerunBridge's arguments: label images (a segmenter's instances) are drawn over their
    camera's images, not as streams of their own; image streams without a compressed form (float disparity, feature
    maps), which the bridge reads, are left out."""
    def of(kind, labels=False):
        return [base for base, r in found if r.get("kind") == kind and bool(r.get("labels")) == labels
                and (kind != "image" or "compressed" in r.get("transports", ("compressed",)))]

    return {"bases": of("image"), "clouds": of("points"), "detections": of("detections"),
            "segmentations": of("image", labels=True)}


def _rerun(args) -> int:
    rr = rerun_bridge.import_rerun()
    with zsession.open_from_args(args, prog=f"xblick {args.command}") as z:
        discover = not (args.streams or args.points)
        if discover:
            found = _bridged(find_streams(z, args.prefix, args.seconds))
            if not found["bases"] + found["clouds"]:
                why = zsession.why_nothing(z)
                print(f"no stream answered within {args.seconds:g} s" + (f"; {why}" if why else ""), file=sys.stderr)
                return 1
        else:
            found = {"bases": args.streams, "clouds": args.points or [], "detections": [], "segmentations": []}
        found["detections"] += args.detections or []
        found["segmentations"] += args.segmentation or []
        found["trajectories"] = args.trajectory or []
        found["bases"] = [b for b in found["bases"] if b not in found["segmentations"]]
        rr.init("xblick", strict=True)
        if args.connect_viewer:
            rr.connect_grpc(args.connect_viewer)
        elif args.save:
            rr.save(args.save)
        elif args.serve is not None:
            url = rr.serve_grpc(grpc_port=args.serve)
            print(f"viewers connect with: rerun --connect {url}", flush=True)
            if args.web is not None:
                rr.serve_web_viewer(web_port=args.web, open_browser=False, connect_to=url)
                print(f"web viewer on port {args.web}", flush=True)
        else:
            rr.spawn(executable_path=rerun_bridge.viewer_executable())
        bridge = rerun_bridge.RerunBridge(z, rr, cloud_rate=args.cloud_rate, stale_after=args.stale_after,
                                          ages=not args.no_ages, **found)
        print(f"bridging {', '.join(bridge.bases + bridge.clouds + bridge.detections + bridge.segmentations)} to Rerun "
              "(Ctrl-C to stop)", flush=True)
        try:
            while True:
                if discover:  # a node started later; the lookup takes the frame-rate line's time
                    started = time.monotonic()
                    added = bridge.add(**_bridged(find_streams(z, args.prefix, min(args.seconds, args.report))))
                    if added:
                        print(f"bridging {', '.join(added)} as well", flush=True)
                    time.sleep(max(0.0, args.report - (time.monotonic() - started)))
                else:
                    time.sleep(args.report)
                bridge.check()
                streams = bridge.bases + bridge.clouds
                keys = streams + sorted(k for k in bridge.frames if k not in streams)  # + bodies, imu, diagnostics
                print("  ".join(f"{k} {bridge.frames[k] / args.report:.1f} Hz" for k in keys), flush=True)
                bridge.frames.clear()
        except KeyboardInterrupt:
            pass
        finally:
            bridge.close()
            # what Rerun still buffers goes to the viewer or the file (--save) before the process ends
            rr.disconnect()
    return 0


def _record(args) -> int:
    recording.import_mcap()  # at once, not at the first record command
    with zsession.open_from_args(args, prog=f"xblick {args.command}") as z:
        recorder = recording.Recorder(z, expand_path(args.dir), name=args.name, namespace=args.namespace,
                                      prefix=args.prefix, points=args.points, split=args.split_minutes * 60.0,
                                      compression=args.compression)
        try:
            print(f"xblick record: ready, control on {recording.control_key(args.name)}", flush=True)
            if not args.paused:
                recorder.record(keys=args.key, blocking=args.blocking, log_time=args.log_time, file=args.file)
            while True:
                time.sleep(args.report)
                recorder.check()
                _report(recorder.status(), "xblick record")
        except KeyboardInterrupt:
            pass
        finally:
            recorder.close()  # the file is complete
            recorder.check()
    return 0


def _report(s: dict, prog: str):
    """The record and play node's status line; ``prog`` the command that started it."""
    if s["state"] == "recording":
        print(f"{prog}: recording {s['file']}, {s['messages']} messages, {s['bytes'] / 1e6:.1f} MB, "
              f"{s['topics']} topics, {s['dropped']} dropped", flush=True)
    elif s["play"] is not None:
        print(f"{prog}: {s['state']} at {s['play']['position_s']:.1f} s, {s['play']['sent']} messages", flush=True)
    else:
        print(f"{prog}: idle", flush=True)


def _recorder(args) -> int:
    with zsession.open_from_args(args, prog=f"xblick {args.command}") as z:
        if args.command_ == "status":
            result = recording.query_status(z, args.name)
        else:
            options = {}
            if args.command_ == "play":
                options = {"files": args.files, "rate": args.rate, "loop": args.loop, "prefix": args.prefix,
                           "keys": args.key or [], "cuda_device": args.cuda_device, "pace": args.pace,
                           "clock": args.clock, **_video_options(args)}
            elif args.command_ == "record":
                options = {k: v for k, v in (("keys", args.key), ("blocking", args.blocking or None),
                                             ("log_time", args.log_time), ("file", args.file)) if v is not None}
            elif args.command_ == "step":
                options = {k: v for k, v in (("frames", args.frames), ("until_ns", args.until_ns), ("key", args.key))
                           if v is not None}
            try:
                result = recording.request(z, args.name, args.command_, **options)
            except recording.RecordingError as error:
                print(json.dumps(error.result, indent=1))
                raise SystemExit(f"xblick recorder: {error}") from None
    print(json.dumps(result, indent=1))
    return 0


def _play(args) -> int:
    # absolute: the files on the command line are relative to the working directory, while Recorder.play takes a
    # relative file as relative to the node's directory (files given over Zenoh)
    files = [expand_path(f).absolute() for f in args.files]
    directory = expand_path(args.dir) if args.dir else files[0].resolve().parent
    with zsession.open_from_args(args, prog=f"xblick {args.command}") as z:
        recorder = recording.Recorder(z, directory, name=args.name, prog="xblick play")
        try:
            try:
                player = recorder.play(files, rate=args.rate, loop=args.loop, prefix=args.prefix,
                                       keys=args.key or [], cuda_device=args.cuda_device, pace=args.pace,
                                       clock=args.clock, **_video_options(args))
            except (ValueError, FileNotFoundError, ImportError) as error:
                raise SystemExit(f"xblick play: {error}") from None
            print(f"xblick play: {len(player.topics)} topics" + (f" below {args.prefix}" if args.prefix else "")
                  + f", control on {recording.control_key(args.name)}", flush=True)
            while True:
                ended = recorder.wait_played(args.report)
                recorder.check()
                if ended:
                    if args.exit and recorder.state == "idle":
                        break
                    time.sleep(args.report)  # nothing playing: the next report then
                _report(recorder.status(), "xblick play")
        except KeyboardInterrupt:
            pass
        finally:
            recorder.close()
            recorder.check()
    return 0


def _add_video_arguments(p):
    """xblick play's (and xblick recorder play's) options for videos and image sequences (video.py)."""
    v = p.add_argument_group("videos", "files that are not MCAP (a video FFmpeg reads, an image directory or glob; "
                                       "needs xblick[video]): played at once, one stream each, on one clock")
    v.add_argument("--stream", action="append", metavar="BASE",
                   help="the stream base of a video, e.g. cam/rc_visard/left (repeat: one per video, in their "
                        "order)")
    v.add_argument("--camera", default=None, help="one video: its stream is <camera>/color (instead of --stream)")
    v.add_argument("--offset-ms", action="append", type=float, metavar="MS",
                   help="a video's stamps and send times moved by this much (repeat: one per video, or none)")
    v.add_argument("--camera-info", action="append", metavar="FILE",
                   help="a video's ROS calibration file, of its size (repeat: one per video, or none)")
    v.add_argument("--frame-id", action="append", metavar="ID",
                   help="a video's frame id (repeat: one per video, or none; default: the base, / as _)")
    v.add_argument("--size", default=None, metavar="WxH", help="the frames scaled to this size")
    v.add_argument("--scale", type=float, default=None, help="the frames scaled by this factor")
    v.add_argument("--start", type=float, default=None, metavar="S", help="from this second of the videos")
    v.add_argument("--end", type=float, default=None, metavar="S", help="up to this second of the videos")
    v.add_argument("--fps", type=float, default=None,
                   help="the frames' times from this rate, not the files' (image sequences: needed)")
    v.add_argument("--hfov", type=float, default=None, metavar="DEG",
                   help="without --camera-info: a pinhole of this horizontal field of view (default 60)")
    v.add_argument("--vfov", type=float, default=None, metavar="DEG", help="... or of this vertical one")


def _video_options(args) -> dict:
    """The video options given (none: an empty dict, which an MCAP playback takes)."""
    from .video import parse_size

    if args.camera and args.stream:
        raise SystemExit("xblick play: --camera or --stream, not both")
    streams = [f"{args.camera}/color"] if args.camera else args.stream
    options = {"streams": streams, "offsets_ms": args.offset_ms, "camera_infos": args.camera_info,
               "frame_ids": args.frame_id, "size": parse_size(args.size) if args.size else None, "scale": args.scale,
               "start": args.start, "end": args.end, "fps": args.fps, "hfov": args.hfov, "vfov": args.vfov}
    return {k: v for k, v in options.items() if v is not None}


def _param(args) -> int:
    from . import parameters
    from .msg import ParameterType

    with zsession.open_from_args(args, prog=f"xblick {args.command}") as z:
        if args.nodes:
            found = parameters.find_nodes(z, args.timeout)
            if not found:
                print(f"no node offers parameters within {args.timeout:g} s", file=sys.stderr)
                return 1
            print("\n".join(found))
            return 0
        if not args.node:
            raise SystemExit("xblick param: --node <namespace>/<name> (or --nodes)")
        client = parameters.ParameterClient(z, args.node, args.timeout)
        if args.set:
            names = [s.partition(":=")[0] for s in args.set]
            if any(":=" not in s for s in args.set):
                raise SystemExit("xblick param: --set NAME:=VALUE")
            descriptors = dict(zip(names, client.describe(names)))
            values = {}
            for item in args.set:
                name, _, given = item.partition(":=")
                if descriptors[name].type == ParameterType.NOT_SET:
                    raise SystemExit(f"xblick param: {args.node} has no parameter {name}")
                values[name] = parameters.from_text(descriptors[name], given)
            results = ([client.set_atomically(values)] if args.atomically else client.set(values))
            failed = False
            for name, r in zip(["(atomically)"] if args.atomically else names, results):
                print(f"{name}: {'set' if r.successful else 'refused: ' + r.reason}")
                failed = failed or not r.successful
            return 1 if failed else 0
        names = args.names or client.list()
        if args.dump:
            values = {n: v.value() for n, v in zip(names, client.get(names))}
            Path(args.dump).write_text(parameters.dump_params_file({client.name: values}))
            print(f"{len(values)} parameters of {client.name} in {args.dump}")
            return 0
        descriptors = client.describe(names) if args.describe else None
        for i, (name, value) in enumerate(zip(names, client.get(names))):
            line = f"{name} = {parameters.text(value.value())}"
            if descriptors is not None:
                d = descriptors[i]
                ranges = d.floating_point_range or d.integer_range
                extra = [ParameterType.NAMES[d.type]]
                if ranges:
                    r = ranges[0]
                    extra.append(f"{r.from_value:g} to {r.to_value:g}" + (f" step {r.step:g}" if r.step else ""))
                if d.additional_constraints:
                    extra.append(d.additional_constraints)
                if d.read_only:
                    extra.append("read-only")
                line += f"  ({'; '.join(extra)})" + (f"  {d.description}" if d.description else "")
            print(line)
    return 0


RELAY = "xblick-relay"      # the relay program (github.com/xblick/xblick-relay)


def relay_program() -> str:
    """The relay program that ``xblick relay`` starts, looked for in this order and nowhere else:

    1. the installed package xblick-relay (``pip install xblick[relay]``): its ``xblick_relay.binary()``;
    2. ``xblick-relay`` on PATH (a CMake build of the relay repository).

    An installed xblick-relay without its program raises; the lookup does not go on to PATH then."""
    if importlib.util.find_spec("xblick_relay") is not None:
        return str(importlib.import_module("xblick_relay").binary())
    found = shutil.which(RELAY)
    if found is None:
        raise FileNotFoundError(f"{RELAY} not found: neither the package xblick-relay is installed nor {RELAY} on "
                                f"PATH. Install it with: pip install 'xblick[relay]'. Or build it with CMake from "
                                f"https://github.com/xblick/xblick-relay and put its bin directory on PATH.")
    return found


def _relay(arguments: list[str]) -> int:
    """``xblick relay [arguments]``: replaces this process by the relay program, with the arguments as they stand
    (signals and the exit code are the relay's own)."""
    try:
        program = relay_program()
    except FileNotFoundError as error:
        print(f"xblick relay: {error}", file=sys.stderr)
        return 1
    sys.stdout.flush()
    sys.stderr.flush()
    os.execv(program, [program, *arguments])


SCRIPTS = ("xblick",)  # pyproject.toml's console scripts


def build_parser(prog: str = "xblick") -> argparse.ArgumentParser:
    """The commands' parser (the terminal interface checks its ``xblick rerun`` command line with it)."""
    parser = argparse.ArgumentParser(prog=prog, description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="command", required=True)

    p = commands.add_parser("list", help="the camera streams (their info records), or with --traffic every topic")
    p.add_argument("prefix", nargs="?", default="", help="only keys below this prefix, e.g. test")
    p.add_argument("--seconds", type=float, default=2.0)
    p.add_argument("--traffic", action="store_true",
                   help="subscribe to every topic and show rates and sizes (makes publishers produce everything)")
    zsession.add_arguments(p)
    p.set_defaults(run=_list)

    for name, run, help_text in (("hz", _hz, "rate and latency of a camera stream"),
                                 ("snapshot", _snapshot, "save the next frame of a camera stream")):
        p = commands.add_parser(name, help=help_text)
        p.add_argument("stream", help="stream base, e.g. test/camera/color")
        if name == "snapshot":
            p.add_argument("output", help="image file (PNG keeps 16-bit depth)")
            p.add_argument("--timeout", type=float, default=5.0)
        else:
            p.add_argument("--seconds", type=float, default=0.0, help="stop after this time (default: never)")
        p.add_argument("--transport", choices=("auto", "compressed", "raw", "shm"), default="auto",
                       help="auto: shared memory on the publisher's host, else compressed")
        zsession.add_arguments(p)
        p.set_defaults(run=run)

    p = commands.add_parser("test-camera", help="publish a moving test pattern")
    p.add_argument("--namespace", default="test")
    p.add_argument("--camera", default="camera")
    p.add_argument("--stream", default="color")
    p.add_argument("--frame-id", default="TestCamera")
    p.add_argument("--size", type=int, nargs=2, default=(640, 480), metavar=("W", "H"))
    p.add_argument("--vfov", type=float, default=60.0, help="vertical field of view in degrees")
    p.add_argument("--fps", type=float, default=15.0)
    zsession.add_arguments(p)
    p.set_defaults(run=_test_camera)

    p = commands.add_parser("cam", help="publish a live camera that FFmpeg opens: a V4L2 device, an RTSP or HTTP URL "
                                        "(needs xblick[cam])", description=cam.__doc__,
                            formatter_class=argparse.RawDescriptionHelpFormatter)
    cam.add_arguments(p)
    from .parameters import Parameters

    Parameters.add_arguments(p)
    zsession.add_arguments(p)
    p.set_defaults(run=lambda a, parser=p: _cam(a, parser))

    p = commands.add_parser("router", help="run a Zenoh router (the one every PC needs) from the Zenoh library inside "
                                           "the package, instead of zenohd; until Ctrl-C",
                            description=router.__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--listen", nargs="+", default=[router.LISTEN], metavar="ENDPOINT",
                   help=f"where it listens (default {router.LISTEN}, as zenohd in the documentation)")
    p.add_argument("--connect", nargs="*", default=[], metavar="ENDPOINT",
                   help="the routers of other PCs to connect to (zenohd's -e), e.g. tcp/other-pc:7447")
    p.set_defaults(run=lambda a: router.serve(a.listen, a.connect))

    p = commands.add_parser("view", help="show camera streams in VLC (one window per stream) or a browser, served as MJPEG over "
                                         "HTTP",
                            description=view.__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("streams", nargs="+", metavar="STREAM", help="the stream bases to show, e.g. test/camera/color")
    opener = p.add_mutually_exclusive_group()
    opener.add_argument("--browser", action="store_true",
                        help="open the grid page in the system's browser instead of VLC: all streams side by side")
    opener.add_argument("--no-player", action="store_true", help="start no player: serve and print the addresses")
    p.add_argument("--http", default=view.LISTEN, metavar="ADDRESS:PORT",
                   help=f"where the HTTP server listens (default {view.LISTEN}: this PC only). 0.0.0.0:{view.PORT} "
                        "opens it to the network WITHOUT A PASSWORD: anyone who can reach the address can watch")
    p.add_argument("--seconds", type=float, default=2.0, help="how long to wait for each stream's publisher")
    zsession.add_arguments(p)
    p.set_defaults(run=_view)

    p = commands.add_parser("diagnostics", help="how the cameras' publishers are doing (<camera>/diagnostics)")
    p.add_argument("prefix", nargs="?", default="", help="only cameras below this prefix, e.g. test")
    p.add_argument("--values", action="store_true", help="every value, not only level and message")
    p.add_argument("--seconds", type=float, default=0.0, help="stop after this time (default: never)")
    zsession.add_arguments(p)
    p.set_defaults(run=_diagnostics)

    p = commands.add_parser("rerun", help="show camera streams in the Rerun viewer (needs xblick[rerun])",
                            description=rerun_bridge.__doc__,
                            formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("streams", nargs="*",
                   help="stream bases (default: every stream, label image and detections that answer, below --prefix, "
                        "looked for again every --report seconds)")
    p.add_argument("--points", action="append", metavar="BASE",
                   help="a point cloud stream to show as well, e.g. test/camera/points (repeat for more)")
    p.add_argument("--cloud-rate", type=float, default=5.0, help="point clouds per second sent to the viewer")
    p.add_argument("--stale-after", type=float, default=2.0,
                   help="clear bodies, boxes and masks whose topic sent nothing for this many seconds (a node that "
                        "stopped sends no empty result); raise it for a node slower than this, 0: keep them")
    p.add_argument("--detections", action="append", metavar="KEY",
                   help="a vision node's boxes (vision_msgs Detection2DArray or Detection3DArray) to show, "
                        "<camera>/<node>/<name>, e.g. test/camera/boxer/boxes_3d (repeat for more)")
    p.add_argument("--segmentation", action="append", metavar="KEY",
                   help="a label image stream (mono8 or mono16 ids, 0 none) to draw over its camera's image, "
                        "<camera>/<node>/<name>, e.g. test/camera/sam3/instances (repeat for more)")
    p.add_argument("--trajectory", action="append", metavar="KEY",
                   help="poses in the world (tf2_msgs/TFMessage) to draw as a path with axes at the newest, e.g. "
                        "test/camera/vslam/tf, and test/camera/tf to compare (repeat for more)")
    p.add_argument("--no-ages", action="store_true",
                   help="leave out the rates and ages (rates/<topic> Hz, ages/<topic> and ages_received/<topic> ms, "
                        "the table ages_now)")
    p.add_argument("--prefix", default="", help="without streams: only streams below this prefix, e.g. test")
    p.add_argument("--seconds", type=float, default=2.0, help="without streams: how long to look for them")
    p.add_argument("--report", type=float, default=5.0, help="seconds between the frame-rate lines")
    viewer = p.add_mutually_exclusive_group()
    viewer.add_argument("--spawn", action="store_true", help="start a viewer on this machine (the default)")
    viewer.add_argument("--connect-viewer", metavar="URL",
                        help="send to a running viewer, e.g. rerun+http://<laptop>:9876/proxy (start it with: rerun)")
    viewer.add_argument("--serve", type=int, metavar="PORT", nargs="?", const=9876,
                        help="let viewers connect here (default port 9876)")
    viewer.add_argument("--save", metavar="FILE", help="write a recording (.rrd) instead")
    p.add_argument("--web", type=int, metavar="PORT", nargs="?", const=9090,
                   help="with --serve: also serve the web viewer (default port 9090)")
    zsession.add_arguments(p)
    p.set_defaults(run=_rerun)

    p = commands.add_parser("param", help="a node's parameters (ROS 2's services): list, describe, get, set, dump")
    p.add_argument("--node", default=None, metavar="NAMESPACE/NAME", help="the node, e.g. test/camera")
    p.add_argument("--nodes", action="store_true", help="the nodes that offer parameters")
    p.add_argument("names", nargs="*", help="only these parameters (default: all)")
    what = p.add_mutually_exclusive_group()
    what.add_argument("--list", action="store_true", help="the names and values (the default)")
    what.add_argument("--get", action="store_true", help="the same")
    what.add_argument("--describe", action="store_true", help="with type, range, constraints, read-only")
    what.add_argument("--set", action="append", metavar="NAME:=VALUE", help="change (repeat; values as YAML scalars)")
    what.add_argument("--dump", default=None, metavar="FILE", help="write them as a ROS params file")
    p.add_argument("--atomically", action="store_true", help="with --set: all or none")
    p.add_argument("--timeout", type=float, default=2.0)
    zsession.add_arguments(p)
    p.set_defaults(run=_param)

    p = commands.add_parser("mps", help="run NVIDIA's MPS control daemon for the nodes on this PC's GPU (mps.py)",
                            description=mps.__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--pipe-dir", required=True,
                   help="the daemon's directory: the clients' CUDA_MPS_PIPE_DIRECTORY (short: it names Unix sockets)")
    p.add_argument("--log-dir", default=None, help="the daemon's log (default <pipe-dir>/log)")
    p.add_argument("--name", default=None, help="its name on Zenoh: xblick/mps/<name> (default: the host's, "
                                                 f"here {mps.default_name()})")
    zsession.add_arguments(p)
    p.set_defaults(run=lambda a: mps.serve(a.pipe_dir, a.log_dir, a.name, a.connect, a.listen, a.config,
                                           a.zenoh_mode))

    p = commands.add_parser("record", help="the record and play node, recording the topics into MCAP files from its "
                                           "start (needs xblick[record]); plays too when told to",
                            description=recording.__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--dir", required=True, help="where the files go (${VAR} and ~ expanded)")
    p.add_argument("--name", default="recorder", help="the recorder's name: its files' and its control topics' "
                                                      "(xblick/<name>/control, xblick/<name>/status)")
    p.add_argument("--paused", action="store_true", help="idle at the start: wait for a command (record, play) on "
                                                          "the control topic")
    p.add_argument("--namespace", default="*", help="only these robots' topics, e.g. robot (default: all)")
    p.add_argument("--prefix", default="", help="only streams and detections below this prefix")
    p.add_argument("--points", action="store_true", help="point clouds too (large; they come again from depth)")
    p.add_argument("--split-minutes", type=float, default=10.0, help="a new file after this time (0: one file)")
    p.add_argument("--compression", choices=("none", "lz4", "zstd"), default="lz4",
                   help="of the files' chunks (images are JPEG or PNG already: lz4 is cheap)")
    p.add_argument("--key", action="append", metavar="KEY_EXPR",
                   help="record exactly these key expressions (repeat for more), not what it finds")
    p.add_argument("--blocking", action="store_true",
                   help="for the keys of blocking publishers: block instead of dropping and check their counts at the "
                        "end (needs --key; the other keys are recorded dropping, unchecked)")
    p.add_argument("--log-time", choices=recording.LOG_TIMES, default="receive",
                   help="the messages' log time: when they came, or their own stamp (a simulation's time)")
    p.add_argument("--file", default=None, help="the file (relative to --dir; one, not split)")
    p.add_argument("--report", type=float, default=10.0, help="seconds between the status lines")
    zsession.add_arguments(p)
    p.set_defaults(run=_record)

    p = commands.add_parser("recorder", help="tell a running record and play node (xblick record, xblick play) "
                                             "to record, play, pause, resume, step or stop, or ask its status, over "
                                             "Zenoh; prints the result")
    told = p.add_subparsers(dest="command_", required=True, metavar="command")
    for command, text in (("record", "record into a new file"), ("stop", "stop recording or playing"),
                          ("pause", "pause the playback"), ("resume", "resume the playback"),
                          ("status", "the node's state"), ("play", "play recordings"),
                          ("step", "a playback in lockstep (--pace step) one step on: a frame by default")):
        q = told.add_parser(command, help=text)
        if command == "record":
            q.add_argument("--key", action="append", metavar="KEY_EXPR", help="record exactly these (repeat)")
            q.add_argument("--blocking", action="store_true",
                           help="block for blocking publishers, check their counts at stop")
            q.add_argument("--log-time", choices=recording.LOG_TIMES, default=None, help="receive (default) or header")
            q.add_argument("--file", default=None, help="the file, relative to the node's --dir")
        if command == "step":
            given = q.add_mutually_exclusive_group()
            given.add_argument("--frames", type=int, default=None, help="this many frames (log times)")
            given.add_argument("--until-ns", type=int, default=None, help="the frames up to this log time")
            given.add_argument("--key", default=None, help="up to the next message on this key (pattern)")
        if command == "play":
            q.add_argument("files", nargs="+", help="the recordings, relative to the node's --dir or absolute")
            q.add_argument("--prefix", default="", help="publish below this prefix (default: on the recorded topics)")
            q.add_argument("--rate", type=float, default=1.0, help="times the recorded pace")
            q.add_argument("--loop", action="store_true", help="from the beginning again at the end")
            q.add_argument("--key", action="append", metavar="KEY_EXPR",
                           help="only topics that match this key expression")
            q.add_argument("--cuda-device", type=int, default=None, metavar="ORDINAL",
                           help="image streams also in a CUDA ring on this device")
            q.add_argument("--pace", choices=recording.PACES, default="wall",
                           help="the recorded pace (wall), or in lockstep: step commands, or a clock topic")
            q.add_argument("--clock", default=None, metavar="KEY",
                           help="with --pace clock: its rosgraph_msgs/Clock key")
            _add_video_arguments(q)
        q.add_argument("--name", default="recorder", help="the node's name")
        zsession.add_arguments(q)
        q.set_defaults(run=_recorder)

    p = commands.add_parser("play", help="the record and play node, playing MCAP recordings (needs "
                                         "xblick[record]) or videos (xblick[video]) at its start; runs until "
                                         "Ctrl-C or SIGTERM, as xblick record")
    p.add_argument("files", nargs="+", help="xblick record's files, played one after another; or videos (any "
                                            "file that is not MCAP, an image directory or glob), played at once")
    p.add_argument("--dir", default=None, help="the node's directory: where it records to and finds files given "
                                               "over Zenoh (default: the first file's)")
    p.add_argument("--prefix", default="", help="publish below this prefix (default: on the recorded topics)")
    p.add_argument("--rate", type=float, default=1.0, help="times the recorded pace")
    p.add_argument("--loop", action="store_true")
    p.add_argument("--key", action="append", metavar="KEY_EXPR",
                   help="only topics that match this key expression, e.g. 'test/camera/*' (repeat for more)")
    p.add_argument("--cuda-device", type=int, default=None, metavar="ORDINAL",
                   help="image streams also in a CUDA ring on this device, as a camera with a GPU offers them "
                        "(default: shared memory only)")
    p.add_argument("--name", default="recorder", help="the node's name: its control topics "
                                                      "(xblick/<name>/control, xblick/<name>/status)")
    p.add_argument("--report", type=float, default=10.0, help="seconds between the status lines")
    p.add_argument("--exit", action="store_true", help="end when the playback has ended (and it does nothing else)")
    p.add_argument("--pace", choices=recording.PACES, default="wall",
                   help="wall: the recorded pace; step: a frame (one log time) per step command (xblick recorder "
                        "step); clock: up to the time of each rosgraph_msgs/Clock sample on --clock")
    p.add_argument("--clock", default=None, metavar="KEY", help="with --pace clock: the key of the clock")
    _add_video_arguments(p)
    zsession.add_arguments(p)
    p.set_defaults(run=_play)

    # main() hands `xblick relay ...` to the relay before this parser: argparse cannot pass options such as --help or
    # --stream through. The entry here lists the command in `xblick --help`.
    p = commands.add_parser("relay", add_help=False,
                            help=f"start {RELAY} with the arguments that follow (needs xblick[relay] or {RELAY} on "
                                 f"PATH); xblick relay --help: its options")
    p.add_argument("arguments", nargs=argparse.REMAINDER)
    p.set_defaults(run=lambda a: _relay(a.arguments))
    return parser


def _view(args) -> int:
    session = zsession.open_from_args(args, prog=f"xblick {args.command}")
    try:
        player = "browser" if args.browser else "none" if args.no_player else "vlc"
        return view.run(session, args.streams, args.http, player, seconds=args.seconds)
    finally:
        session.close()


def main(argv=None) -> int:
    """The command line; without a command the terminal interface (tui.py)."""
    if argv is None:
        argv = sys.argv[1:]
    invoked = Path(sys.argv[0]).name if sys.argv and sys.argv[0] else ""
    prog = invoked if invoked in SCRIPTS else "xblick"
    if argv and argv[0] == "relay":
        return _relay(argv[1:])
    if not argv or argv[0].split("=")[0] in INTERFACE_OPTIONS:
        from . import tui  # Textual only for the interface: the commands start without importing it

        return tui.run(_interface_connection(argv, prog))
    args = parse(argv, prog)
    end_on_signals()  # SIGTERM and SIGINT as Ctrl-C, also as a background job of a script
    try:
        return args.run(args)
    except (TimeoutError, zsession.NoAnswer) as error:  # did not answer, or a command sent once: one line
        print(f"{prog} {args.command}: {error}", file=sys.stderr)
        return 1
    except RuntimeError as error:
        if args.command not in ("param", "recorder"):  # a node's refusal or "not sent" there; elsewhere a fault
            raise
        print(f"{prog} {args.command}: {error}", file=sys.stderr)
        return 1


INTERFACE_OPTIONS = ("--connect", "--listen", "--zenoh-mode", "--config")  # without a command: the interface's


def _interface_connection(argv: list[str], prog: str):
    """The terminal interface's first connection from ``<prog> [--connect ...] [--listen ...] [--zenoh-mode ...]
    [--config FILE]``: none, this PC's router; with endpoints, a client unless it listens or --zenoh-mode peer (as
    the connect dialog: a peer reaches the other sessions directly, which over a tunnel it cannot)."""
    from .tui import Connection

    parser = argparse.ArgumentParser(prog=prog, description="the terminal interface; a command: " + prog + " -h")
    parser.add_argument("--connect", nargs="+", default=[], metavar="ENDPOINT")
    parser.add_argument("--listen", nargs="+", default=[], metavar="ENDPOINT")
    parser.add_argument("--zenoh-mode", choices=("peer", "client"), default=None)
    parser.add_argument("--config", default=None, metavar="FILE")
    args = parser.parse_args(argv)
    if not argv:
        return None
    mode = args.zenoh_mode or ("client" if args.connect and not args.listen else "peer")
    endpoints = lambda given: [zsession.endpoint(e) for e in given]  # noqa: E731 - host or host:port as the commands
    try:
        return Connection(endpoints(args.connect), endpoints(args.listen), args.config,
                          mode if not args.config else "peer")
    except (ValueError, FileNotFoundError) as error:
        parser.error(str(error))


def parse(argv: list[str], prog: str = "xblick") -> argparse.Namespace:
    """A command line's arguments, checked; a wrong one exits as argparse does (SystemExit 2, the reason on stderr)."""
    parser = build_parser(prog)
    args = parser.parse_args(argv)
    if args.command == "rerun" and args.web is not None and args.serve is None:
        parser.error("--web needs --serve")
    return args


if __name__ == "__main__":
    sys.exit(main())
