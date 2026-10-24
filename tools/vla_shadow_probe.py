"""P1 shadow probe: real camera frame -> NoMaD proposal (no execution).

Pulls frames from the body adapter's loopback preview (never a second camera
session), posts goal+observation to the VLA service, and appends each proposal
to tmp/vla-shadow-log.jsonl. Proposals are NEVER sent to the motion gateway —
this is the P1 shadow baseline per docs/VLA-INTEGRATION-PLAN.md.

Usage:
  python tools/vla_shadow_probe.py            # one goal + one infer
  python tools/vla_shadow_probe.py --loop 10  # goal once, then 10 inferences
                                              # at ~2 Hz
"""
import argparse
import base64
import io
import json
import os
import sys
import time
import urllib.request

ADAPTER = os.environ.get("XUEGEROS_ADAPTER_URL", "http://127.0.0.1:18110")
VLA = os.environ.get("XUEGEROS_VLA_URL", "http://192.168.1.166:18300")
LOG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tmp", "vla-shadow-log.jsonl")


def fetch_jpeg_from_preview(timeout=8):
    """Read the adapter MJPEG loopback until one complete JPEG frame arrives."""
    req = urllib.request.Request(f"{ADAPTER}/v1/preview.mjpeg")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        buf = b""
        deadline = time.time() + timeout
        while time.time() < deadline:
            chunk = r.read(65536)
            if not chunk:
                break
            buf += chunk
            start = buf.find(b"\xff\xd8")
            end = buf.find(b"\xff\xd9", start + 2) if start >= 0 else -1
            if start >= 0 and end > 0:
                return buf[start:end + 2]
    raise RuntimeError("no complete JPEG in preview stream")


def post(path, payload, timeout=15):
    req = urllib.request.Request(
        VLA + path,
        data=json.dumps(payload).encode(),
        headers={"content-type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.load(r)


def record(entry):
    os.makedirs(os.path.dirname(LOG), exist_ok=True)
    with open(LOG, "a", encoding="utf-8") as f:
        f.write(json.dumps(entry, ensure_ascii=False) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--loop", type=int, default=0, help="number of inferences after goal (0 = single)")
    ap.add_argument("--interval", type=float, default=0.5)
    args = ap.parse_args()

    goal_jpeg = fetch_jpeg_from_preview()
    r = post("/goal", {"image": base64.b64encode(goal_jpeg).decode()})
    print(f"goal set from live frame ({len(goal_jpeg)} B): {r}")
    record({"t": time.time(), "kind": "goal", "bytes": len(goal_jpeg)})

    n = max(1, args.loop)
    for i in range(n):
        frame = fetch_jpeg_from_preview()
        out = post("/infer", {"image": base64.b64encode(frame).decode()})
        entry = {
            "t": time.time(),
            "kind": "infer",
            "bytes": len(frame),
            "weights_loaded": out.get("weights_loaded"),
            "chosen_waypoint": out.get("chosen_waypoint"),
            "velocity_proposal": out.get("velocity_proposal"),
            "dist_pred": out.get("dist_pred"),
            "latency_ms": out.get("latency_ms"),
        }
        record(entry)
        wp = out.get("chosen_waypoint")
        v = out.get("velocity_proposal", {})
        print(
            f"[{i+1}/{n}] waypoint={wp} v={v.get('v')} w={v.get('w')} "
            f"dist={out.get('dist_pred')} lat={out.get('latency_ms')}ms "
            f"weights={out.get('weights_loaded')}"
        )
        if i < n - 1:
            time.sleep(args.interval)

    print(f"log: {LOG}")


if __name__ == "__main__":
    main()
