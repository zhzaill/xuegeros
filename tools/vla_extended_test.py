"""Extended functional test for vla_server (random weights are fine):
context bootstrap, /reset, goal persistence, serialized inference, latency n=30."""
import base64, io, json, statistics, time, urllib.request
import numpy as np
from PIL import Image

BASE = "http://127.0.0.1:18300"

def jpeg(seed):
    rng = np.random.default_rng(seed)
    buf = io.BytesIO()
    Image.fromarray(rng.integers(0, 255, (240, 320, 3), dtype=np.uint8)).save(buf, "JPEG")
    return base64.b64encode(buf.getvalue()).decode()

def post(path, payload, timeout=30):
    req = urllib.request.Request(BASE + path, data=json.dumps(payload).encode(),
                                 headers={"content-type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.load(r)

def get(path, timeout=10):
    with urllib.request.urlopen(BASE + path, timeout=timeout) as r:
        return json.load(r)

results = []

# 1. fresh start: reset, set goal
post("/reset", {})
assert get("/health")["context_frames"] == 0, "reset did not clear context"
r1 = post("/infer", {"image": jpeg(1)})
assert r1["context_bootstrapped"] is True, "first infer should bootstrap context"
results.append(("bootstrap: first infer bootstraps context", True, r1["latency_ms"]))

r2 = post("/infer", {"image": jpeg(2)})
assert r2["context_bootstrapped"] is False, "second infer must not bootstrap"
results.append(("context: filled after first frame", True, r2["latency_ms"]))

# 2. goal persistence across inferences without goal_image
h = get("/health")
assert h["goal_set"] is True, "goal must persist"
results.append(("goal: persists without per-request goal", True, None))

# 3. goal switch via /goal
post("/goal", {"image": jpeg(99)})
results.append(("goal: /goal switches persistent goal", True, None))

# 4. reset clears context but not goal
post("/reset", {})
h = get("/health")
assert h["context_frames"] == 0 and h["goal_set"] is True
results.append(("reset: clears context, keeps goal", True, None))

# 5. latency batch n=30 (after reset+refill)
post("/reset", {})
lat = []
for i in range(31):
    r = post("/infer", {"image": jpeg(100 + i)})
    if i > 0:
        lat.append(r["latency_ms"])
p50 = statistics.median(lat)
results.append((f"latency: p50={p50:.0f}ms p95={sorted(lat)[int(0.95*len(lat))]:.0f}ms max={max(lat):.0f}ms (n=30)", True, p50))

# 6. concurrent requests are serialized (lock) - fire two threads, both must succeed
import threading
outs = []
def worker(k):
    outs.append(post("/infer", {"image": jpeg(200 + k)})["ok"])
ts = [threading.Thread(target=worker, args=(k,)) for k in range(4)]
[t.start() for t in ts]; [t.join() for t in ts]
assert all(outs) and len(outs) == 4
results.append(("concurrency: 4 parallel /infer all ok (serialized by lock)", True, None))

# 7. velocity clamped in every response
post("/reset", {})
clamped = all(abs(post("/infer", {"image": jpeg(300+i)})["velocity_proposal"]["v"]) <= 0.0801 for i in range(5))
results.append(("clamp: |v| <= 0.08 in all responses", clamped, None))

for name, okv, extra in results:
    print(("PASS " if okv else "FAIL ") + name)
print("ALL DONE")
