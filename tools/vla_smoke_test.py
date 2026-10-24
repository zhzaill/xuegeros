"""Smoke test for vla_server: synthetic frames, /goal + /infer, latency stats."""
import base64
import io
import json
import time
import urllib.request

import numpy as np
from PIL import Image

BASE = "http://127.0.0.1:18300"


def synth_jpeg(seed):
    rng = np.random.default_rng(seed)
    arr = rng.integers(0, 255, (240, 320, 3), dtype=np.uint8)
    buf = io.BytesIO()
    Image.fromarray(arr).save(buf, format="JPEG", quality=80)
    return base64.b64encode(buf.getvalue()).decode()


def post(path, payload, timeout=30):
    req = urllib.request.Request(
        BASE + path,
        data=json.dumps(payload).encode(),
        headers={"content-type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.load(r)


goal = synth_jpeg(1)
print("POST /goal ->", post("/goal", {"image": goal}))

lat = []
resp = None
for i in range(12):
    r = post("/infer", {"image": synth_jpeg(10 + i)})
    if i == 0:
        print("first /infer ->", json.dumps(r, ensure_ascii=False)[:400])
    resp = r
    lat.append(r["latency_ms"])

lat_sorted = sorted(lat)
print(f"latency ms: min={lat_sorted[0]} p50={lat_sorted[len(lat)//2]} max={lat_sorted[-1]} (n={len(lat)}, warmup excluded=None)")
print("weights_loaded:", resp["weights_loaded"], "| context_bootstrapped:", resp["context_bootstrapped"])
print("velocity_proposal:", resp["velocity_proposal"], "| dist_pred:", resp["dist_pred"])

h = json.load(urllib.request.urlopen(BASE + "/health", timeout=10))
print("health vram_mb:", h["vram_mb"], "| last_latency_ms:", h["last_latency_ms"])
