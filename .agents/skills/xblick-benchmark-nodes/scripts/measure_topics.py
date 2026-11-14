"""Measure Durchblick topics for a fixed time: rate, age at send and age at receive per topic, and optionally the
load and memory of GPUs. Prints one JSON object per topic and one per GPU on stdout; progress goes to stderr.

    uv run python scripts/measure_topics.py test/camera/color/image_raw/compressed \
        test/camera/sam3/instances/image_raw/compressed --seconds 60 --gpus 0 1 --warmup 10

Give a topic (a Zenoh key: a stream's transport topic, a node's result topic) or a key expression such as
`<camera>/**`. The age at
send is the publisher's own measure (attachment publish_ns minus the sample's stamp): for a node's result it is the
time from the capture of the input to the result leaving the node, exact on any host. The age at receive (arrival
minus stamp) adds the way to this host and compares two clocks: across PCs it is as good as their synchronisation.

Exit codes: 0 measured, 2 bad arguments, 3 a topic received nothing (named on stderr), 4 nvidia-smi failed.
"""

import argparse
import json
import subprocess
import sys
import threading
import time
from collections import defaultdict

import numpy as np

from xblick import session as db_session
from xblick import stamps


class Topic:
    def __init__(self):
        self.lock = threading.Lock()
        self.arrivals = []            # monotonic seconds
        self.age_send_ms = []
        self.age_receive_ms = []
        self.keys = set()
        self.unstamped = 0

    def take(self, sample):
        now_ns, now = time.time_ns(), time.monotonic()
        key = str(sample.key_expr)
        try:
            stamp = stamps.header_stamp(key, str(sample.encoding), sample.payload.to_bytes())
        except ValueError:
            stamp = None
        send = stamps.age_at_send_ns(sample) if stamp is not None else None
        with self.lock:
            self.keys.add(key)
            self.arrivals.append(now)
            if stamp is None:
                self.unstamped += 1
            else:
                self.age_receive_ms.append((now_ns - stamp) / 1e6)
            if send is not None:
                self.age_send_ms.append(send / 1e6)


def percentiles(values) -> dict | None:
    if len(values) == 0:
        return None
    v = np.asarray(values)
    return {"p50": round(float(np.percentile(v, 50)), 2), "p95": round(float(np.percentile(v, 95)), 2),
            "max": round(float(v.max()), 2), "n": int(v.size)}


def gpu_sampler(gpus, stop, out):
    query = ["nvidia-smi", "--query-gpu=index,utilization.gpu,memory.used", "--format=csv,noheader,nounits",
             "-i", ",".join(map(str, gpus))]
    while not stop.is_set():
        try:
            lines = subprocess.run(query, capture_output=True, text=True, check=True, timeout=10).stdout.splitlines()
        except (OSError, subprocess.SubprocessError) as e:
            out["error"] = repr(e)
            return
        for line in lines:
            index, util, mem = (x.strip() for x in line.split(","))
            out[int(index)].append((float(util), float(mem)))
        stop.wait(1.0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("topics", nargs="+", help="topics (Zenoh keys) or key expressions to measure")
    parser.add_argument("--seconds", type=float, default=60.0, help="measurement time after the warm-up [s]")
    parser.add_argument("--warmup", type=float, default=5.0, help="time before the measurement starts [s]")
    parser.add_argument("--gpus", type=int, nargs="*", default=[], help="GPU indices to sample with nvidia-smi")
    db_session.add_arguments(parser)
    args = parser.parse_args()
    if args.seconds <= 0 or args.warmup < 0:
        parser.error("--seconds must be positive and --warmup not negative")

    session = db_session.open_from_args(args)
    topics = {t: Topic() for t in args.topics}
    subscribers = [session.declare_subscriber(t, topics[t].take) for t in args.topics]
    print(f"warming up {args.warmup:.0f} s", file=sys.stderr)
    time.sleep(args.warmup)
    for t in topics.values():                     # the warm-up does not count
        with t.lock:
            t.arrivals.clear(), t.age_send_ms.clear(), t.age_receive_ms.clear()
    stop = threading.Event()
    gpu_samples = defaultdict(list)
    sampler = None
    if args.gpus:
        sampler = threading.Thread(target=gpu_sampler, args=(args.gpus, stop, gpu_samples), daemon=True)
        sampler.start()
    print(f"measuring {args.seconds:.0f} s", file=sys.stderr)
    start = time.monotonic()
    time.sleep(args.seconds)
    stop.set()
    elapsed = time.monotonic() - start
    for s in subscribers:
        s.undeclare()
    session.close()

    silent = []
    for name, t in topics.items():
        with t.lock:
            n = len(t.arrivals)
            gaps = np.diff(t.arrivals) * 1000 if n > 1 else []
            print(json.dumps({"topic": name, "keys": sorted(t.keys), "samples": n, "rate_hz": round(n / elapsed, 2),
                              "interval_ms": percentiles(gaps), "age_at_send_ms": percentiles(t.age_send_ms),
                              "age_at_receive_ms": percentiles(t.age_receive_ms), "unstamped": t.unstamped}))
            if n == 0:
                silent.append(name)
    if sampler is not None:
        sampler.join(timeout=12)
        if "error" in gpu_samples:
            print(f"nvidia-smi failed: {gpu_samples['error']}", file=sys.stderr)
            return 4
        for index in args.gpus:
            samples = gpu_samples.get(index, [])
            util = [u for u, _ in samples]
            mem = [m for _, m in samples]
            print(json.dumps({"gpu": index, "samples": len(samples), "utilization_pct": percentiles(util),
                              "memory_used_mib": percentiles(mem)}))
    if silent:
        print(f"received nothing on: {', '.join(silent)}", file=sys.stderr)
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
