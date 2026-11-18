#!/usr/bin/env python3
"""Three screenshot exits per docs/SIM-CONTRACT.md §5, each with provenance.

    python sim/probe/capture.py --kind all            # camera_front + map_top + workbench
    python sim/probe/capture.py --kind map_top
    python sim/probe/capture.py --kind camera_front
    python sim/probe/capture.py --kind workbench --out-dir tmp\\captures

Why three separate kinds instead of one "screenshot" helper:

* ``camera_front`` - the shadow backend has no camera at all (URDF has no
  ``camera_link``, no ROS image topic, the MJPEG on the real car is a picture
  stream, not a control-grade image source).  The only honest receipt is
  ``availability=unavailable`` *with* a reason and **no image file**.  A
  workbench render or an MJPEG grab must never be attached here.
* ``map_top`` - a top-down image may only come from one named map.  Today only
  ``maps.truth`` (SDF geometry projection) exists; ``maps.slam`` and
  ``maps.vehicle`` are ``unavailable`` in the backend and must not be filled in
  from truth.  The receipt carries ``map_kind`` + ``map_version``.
* ``workbench`` - the Three.js workbench render.  Carries pose and the actual
  orbit camera, and is stamped ``workbench_not_camera_evidence``.

Every receipt carries the §5 common fields: ``captured_at``, ``age_s``,
``provenance``, ``frame``, plus ``backend`` and ``physical=false``.  Each image
is written next to its ``.json`` sidecar so the metadata travels with the PNG.

``provenance`` follows the team word list (measured / glimpsed / inferred /
unknown).  Per the Lead ruling: ``map_top`` on the truth map is ``measured``
(exact SDF geometry, no estimation), ``workbench`` is ``glimpsed`` (a rendered
view, not a data source), and ``camera_front`` is none of the tiers - it is
``unavailable``, because there is no image frame to attribute.

Loopback only: the sim URL is asserted to be ``127.0.0.1:18142`` (or an explicit
``--sim`` loopback override); port ``18202`` (the physical robotd port) is
refused outright.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import time
import uuid
from datetime import datetime
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.parse import urlparse
from urllib.request import urlopen

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError as exc:  # pragma: no cover - environment check
    raise SystemExit(f"capture.py needs Pillow: {exc}") from exc

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SIM = "http://127.0.0.1:18142"
DEFAULT_OUT = REPO_ROOT / "tmp" / "captures"
PHYSICAL_PORT = 18202
ALLOWED_PORTS = {18142, 18242}

# §5 says provenance is one of measured / glimpsed / inferred.  These renders are
# projections of backend state, never sensor frames, so they are "inferred" and
# say so in provenance_note.
CJK_FONT_HINTS = (
    r"C:\Windows\Fonts\msyh.ttc",
    r"C:\Windows\Fonts\msyhbd.ttc",
    r"C:\Windows\Fonts\simhei.ttf",
)


# --------------------------------------------------------------------------- io


def fetch_json(url: str, timeout: float = 5.0, attempts: int = 3) -> dict:
    """GET JSON with a short retry.

    The local shadow backend is a plain ThreadingHTTPServer; under a burst of
    concurrent panel polls plus a capture it can refuse a connection outright
    (``net::ERR_CONNECTION_REFUSED``) even though it never restarted.  That is a
    transport hiccup, not a contract result, so it is retried; HTTP status
    errors (404/500) are raised immediately.
    """
    last: Exception | None = None
    for index in range(max(1, attempts)):
        try:
            with urlopen(url, timeout=timeout) as response:  # noqa: S310 - loopback guard
                return json.loads(response.read().decode("utf-8"))
        except HTTPError:
            raise
        except URLError as exc:
            last = exc
            time.sleep(0.15 * (index + 1))
    raise last if last is not None else RuntimeError(f"fetch failed: {url}")


def check_loopback(url: str) -> str:
    parsed = urlparse(url)
    if parsed.hostname not in ("127.0.0.1", "localhost"):
        raise SystemExit(f"refusing non-loopback sim URL: {url}")
    port = parsed.port or 80
    if port == PHYSICAL_PORT:
        raise SystemExit(f"refusing the physical robotd port {PHYSICAL_PORT}")
    if port not in ALLOWED_PORTS:
        raise SystemExit(f"unexpected sim port {port}; allowed: {sorted(ALLOWED_PORTS)}")
    return url.rstrip("/")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def now_iso() -> tuple[str, float]:
    epoch = time.time()
    return datetime.fromtimestamp(epoch).astimezone().isoformat(timespec="milliseconds"), epoch


def pose_dict(state: dict) -> dict:
    pose = state.get("pose") or {}
    return {
        "x": round(float(pose.get("x", 0.0)), 4),
        "y": round(float(pose.get("y", 0.0)), 4),
        "yaw": round(float(pose.get("yaw", 0.0)), 4),
    }


def write_receipt(out_dir: Path, kind: str, stamp: str, receipt: dict) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"{kind}_{stamp}.json"
    path.write_text(json.dumps(receipt, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return path


# ------------------------------------------------------------------ rendering


def _font(size: int):
    """Return (font, cjk_ok).  Falls back to ASCII labels when no CJK face."""
    for candidate in CJK_FONT_HINTS:
        if os.path.exists(candidate):
            try:
                return ImageFont.truetype(candidate, size), True
            except OSError:
                continue
    for candidate in (r"C:\Windows\Fonts\arial.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(candidate, size), False
        except OSError:
            continue
    return ImageFont.load_default(), False


def render_truth_map(scene: dict, pose: dict, meta: dict, out_path: Path) -> dict:
    """Project the SDF scene description top-down.  Truth only, never SLAM."""
    resolution = float(meta.get("resolution_m") or 0.05)
    width_m = float(scene.get("width") or 6.0)
    height_m = float(scene.get("height") or 6.0)
    cols = max(1, int(round(width_m / resolution)))
    rows = max(1, int(round(height_m / resolution)))
    cell_px = 8
    margin = 26
    header = 78
    footer = 78
    width_px = cols * cell_px + margin * 2
    height_px = rows * cell_px + margin * 2 + header + footer

    image = Image.new("RGB", (width_px, height_px), (24, 30, 27))
    draw = ImageDraw.Draw(image)
    font_big, cjk = _font(19)
    font_small, _ = _font(14)

    def label(zh: str, en: str) -> str:
        return zh if cjk else en

    map_top = margin + header
    draw.rectangle(
        [margin, map_top, margin + cols * cell_px - 1, map_top + rows * cell_px - 1],
        fill=(245, 245, 242),
    )

    def to_px(x: float, y: float) -> tuple[float, float]:
        col = (x + width_m / 2.0) / resolution * cell_px + margin
        row = (height_m / 2.0 - y) / resolution * cell_px + map_top
        return col, row

    # half-metre grid, so the picture can be read as a metric map
    step_m = 0.5
    steps_x = int(width_m / step_m) + 1
    steps_y = int(height_m / step_m) + 1
    for index in range(steps_x + 1):
        x = -width_m / 2.0 + index * step_m
        if x > width_m / 2.0 + 1e-9:
            break
        col, _ = to_px(x, 0.0)
        draw.line([(col, map_top), (col, map_top + rows * cell_px)], fill=(214, 216, 210), width=1)
    for index in range(steps_y + 1):
        y = -height_m / 2.0 + index * step_m
        if y > height_m / 2.0 + 1e-9:
            break
        _, row = to_px(0.0, y)
        draw.line([(margin, row), (margin + cols * cell_px, row)], fill=(214, 216, 210), width=1)

    drawn = 0
    for item in scene.get("obstacles") or []:
        color = item.get("color") or "#444c47"
        try:
            rgb = tuple(int(color.lstrip("#")[i : i + 2], 16) for i in (0, 2, 4))
        except (ValueError, IndexError):
            rgb = (68, 76, 71)
        if item.get("type") == "box":
            size = item.get("size") or [0.1, 0.1]
            half_x, half_y = float(size[0]) / 2.0, float(size[1]) / 2.0
            yaw = float(item.get("yaw") or 0.0)
            corners = []
            for dx, dy in ((-half_x, -half_y), (half_x, -half_y), (half_x, half_y), (-half_x, half_y)):
                wx = float(item["x"]) + dx * math.cos(yaw) - dy * math.sin(yaw)
                wy = float(item["y"]) + dx * math.sin(yaw) + dy * math.cos(yaw)
                corners.append(to_px(wx, wy))
            draw.polygon(corners, fill=rgb, outline=(40, 44, 42))
            drawn += 1
        elif item.get("type") == "cylinder":
            radius = float(item.get("radius") or 0.1)
            cx, cy = float(item["x"]), float(item["y"])
            left, top = to_px(cx - radius, cy + radius)
            right, bottom = to_px(cx + radius, cy - radius)
            draw.ellipse([left, top, right, bottom], fill=rgb, outline=(40, 44, 42))
            drawn += 1

    # pose marker: same ROS frame as /api/state (x forward, y left, yaw CCW)
    px, py = to_px(pose["x"], pose["y"])
    robot_r = 9
    draw.ellipse([px - robot_r, py - robot_r, px + robot_r, py + robot_r], fill=(30, 90, 230))
    draw.line(
        [
            px,
            py,
            px + 26 * math.cos(pose["yaw"]),
            py - 26 * math.sin(pose["yaw"]),
        ],
        fill=(30, 90, 230),
        width=3,
    )

    header_text = [
        label("真值俯视图（SDF 几何投影）", "TRUTH MAP (SDF geometry projection)"),
        label(
            f"map_kind=truth · map_version={meta.get('map_version') or 'n/a'} · 分辨率 {resolution} m · 已知格 {cols * rows}",
            f"map_kind=truth · map_version={meta.get('map_version') or 'n/a'} · resolution {resolution} m · known_cells {cols * rows}",
        ),
        label(
            f"scene={meta.get('scene')} · world_sha256={(meta.get('world_sha256') or 'n/a')[:16]} · captured_at={meta.get('captured_at')}",
            f"scene={meta.get('scene')} · world_sha256={(meta.get('world_sha256') or 'n/a')[:16]} · captured_at={meta.get('captured_at')}",
        ),
    ]
    y_cursor = 10
    for index, line in enumerate(header_text):
        draw.text((margin, y_cursor), line, fill=(232, 240, 234) if index == 0 else (170, 184, 175),
                  font=font_big if index == 0 else font_small)
        y_cursor += 22 if index == 0 else 19

    footer_y = map_top + rows * cell_px + 8
    footer_text = [
        label(
            f"位姿 x={pose['x']:.3f} y={pose['y']:.3f} yaw={math.degrees(pose['yaw']):.1f}° · {drawn} 个几何体 · physical=false · 运动学影子后端",
            f"pose x={pose['x']:.3f} y={pose['y']:.3f} yaw={math.degrees(pose['yaw']):.1f} deg · {drawn} primitives · physical=false · kinematic shadow backend",
        ),
        label(
            "这是 SDF 真值投影：不是 SLAM 建图结果，不是雷达实测，也不是相机证据；只用于评估与显示。",
            "SDF truth projection: NOT a SLAM result, NOT a lidar measurement, NOT camera evidence. Evaluation only.",
        ),
    ]
    for index, line in enumerate(footer_text):
        draw.text((margin, footer_y + index * 20), line, fill=(170, 184, 175), font=font_small)

    if hasattr(out_path, 'write'):
        image.save(out_path, format='PNG')
    else:
        out_path.parent.mkdir(parents=True, exist_ok=True)
        image.save(out_path)
    return {"width": width_px, "height": height_px, "primitives_drawn": drawn, "known_cells": cols * rows}


# -------------------------------------------------------------------- browser


def tabbit_cli() -> Path | None:
    override = os.environ.get("TABBIT_CLI")
    if override and Path(override).exists():
        return Path(override)
    local = os.environ.get("LOCALAPPDATA")
    if not local:
        return None
    candidate = Path(local) / "Tabbit" / "LocalAgent" / "bin" / "tabbit-cli.exe"
    return candidate if candidate.exists() else None


WORKBENCH_JS = r"""
const task = "__TASK__";
// Listeners go on before navigation so early failures are not missed.  Console
// text alone cannot tell "expected /api/strategy 404 while Half A ships it" from
// a real JS exception, so failures are captured structurally:
//   js_errors          uncaught exceptions (these are always real bugs)
//   http_errors        responses >= 400 with their URL
//   request_failures   transport-level failures (aborted/refused)
const jsErrors = [];
const httpErrors = [];
const requestFailures = [];
page.on("console", (message) => {
  if (message.type() === "error") jsErrors.push("console: " + message.text());
});
page.on("pageerror", (error) => jsErrors.push("pageerror: " + error.message));
page.on("response", (response) => {
  if (response.status() >= 400) httpErrors.push({url: response.url(), status: response.status()});
});
page.on("requestfailed", (request) => {
  requestFailures.push({url: request.url(), error: String(request.failure() && request.failure().errorText)});
});
await page.goto("__URL__/", {waitUntil: "domcontentloaded"});
await page.waitForFunction(() => {
  const pose = document.getElementById("pose");
  return pose && /m$/.test(pose.textContent) && typeof window.__workbenchView === "function";
}, null, {timeout: 20000});
// let the panels paint at least one contract refresh before the frame is taken
await page.waitForFunction(() => {
  const li = document.querySelectorAll("#sensors li[data-state]");
  const maps = document.querySelectorAll("#maps li[data-state]");
  return li.length >= 2 && maps.length === 3 && !/读取中/.test(document.getElementById("sensors").textContent);
}, null, {timeout: 20000});
await page.waitForTimeout(600);
const view = await page.evaluate(() => window.__workbenchView());
const panels = await page.evaluate(() => ({
  pose: document.getElementById("pose").textContent,
  yaw: document.getElementById("yaw").textContent,
  state: document.getElementById("state").textContent,
  physical: document.getElementById("physical").textContent,
  backend: document.getElementById("backend").textContent,
  strategy: document.getElementById("strategy") ? {
    text: document.getElementById("strategy").textContent,
    level: document.getElementById("strategy").dataset.level || null,
  } : null,
  goal_buttons: [...document.querySelectorAll("[data-goal]")].map((button) => ({
    goal: button.dataset.goal, disabled: button.disabled,
  })),
  result: document.getElementById("result") ? document.getElementById("result").textContent : null,
  sensors: [...document.querySelectorAll("#sensors li")].map((li) => ({
    state: li.dataset.state,
    sensor: li.dataset.sensor,
    border: getComputedStyle(li).borderLeftColor,
  })),
  maps: [...document.querySelectorAll("#maps li")].map((li) => ({
    state: li.dataset.state, kind: li.dataset.mapKind, border: getComputedStyle(li).borderLeftColor,
  })),
  stamp: document.getElementById("contract-stamp").textContent,
  webgl: (() => {
    const canvas = document.getElementById("view");
    const gl = canvas && (canvas.getContext("webgl2") || canvas.getContext("webgl"));
    return {present: !!gl, width: canvas ? canvas.width : null, height: canvas ? canvas.height : null};
  })(),
}));
const shot = await page.screenshot({fullPage: false});
// NOTE: this program runs in a Node realm; the browser `location`/`document`
// globals only exist inside page.evaluate / waitForFunction callbacks.
return {
  view, panels,
  consoleErrors: jsErrors, httpErrors, requestFailures,
  shot_path: shot.path, shot_sha256: shot.sha256 || null, url: page.url(),
};
"""


def parse_cli_json(stdout: str) -> dict:
    text = stdout.strip()
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        pass
    for marker in ('{"result"', '{"ok"'):
        index = text.find(marker)
        if index >= 0:
            try:
                return json.JSONDecoder().raw_decode(text[index:])[0]
            except json.JSONDecodeError:
                continue
    raise ValueError(f"tabbit-cli produced no JSON payload: {stdout[:400]}")


def capture_workbench_browser(sim_url: str, timeout_s: float = 150.0) -> dict:
    cli = tabbit_cli()
    if cli is None:
        raise RuntimeError("tabbit_cli_missing: 未找到 Tabbit CLI，无法取得工作台画面")
    task = "xuegeros-capture"
    request_id = f"wb-{uuid.uuid4().hex[:12]}"
    script = WORKBENCH_JS.replace("__URL__", sim_url).replace("__TASK__", task)
    completed = subprocess.run(  # noqa: S603 - fixed launcher path, JS on stdin
        [str(cli), "nodejs", "--task", task, "--request-id", request_id, "--timeout-ms", "120000"],
        input=script.encode("utf-8"),
        capture_output=True,
        timeout=timeout_s,
    )
    stdout = completed.stdout.decode("utf-8", errors="replace")
    stderr = completed.stderr.decode("utf-8", errors="replace")
    if completed.returncode != 0:
        raise RuntimeError(f"tabbit_cli_exit_{completed.returncode}: {(stdout or stderr)[:400]}")
    payload = parse_cli_json(stdout)
    result = payload.get("result") or {}
    value = result.get("value")
    if value is None:
        resource = result.get("resourceId")
        raise RuntimeError(f"tabbit_cli_returned_no_value: resource={resource} status={payload.get('status')}")
    value["_task"] = task
    value["_request_id"] = request_id
    value["_exit_code"] = completed.returncode
    return value


def finish_workbench_task() -> str | None:
    cli = tabbit_cli()
    if cli is None:
        return None
    try:
        completed = subprocess.run(  # noqa: S603
            [str(cli), "finish", "--task", "xuegeros-capture"],
            capture_output=True,
            timeout=45,
        )
        return f"finish exit={completed.returncode}"
    except (OSError, subprocess.SubprocessError) as exc:  # pragma: no cover
        return f"finish_failed: {type(exc).__name__}: {exc}"


# --------------------------------------------------------------------- kinds


def capture_camera_front(sim: str, out_dir: Path, stamp: str) -> dict:
    captured_at, epoch = now_iso()
    state = fetch_json(f"{sim}/api/state")
    sensors = fetch_json(f"{sim}/api/sensors")
    camera = next((item for item in sensors.get("sensors", []) if item.get("id") == "camera"), None)
    if camera is None:
        availability = "missing"
        reason = "后端 /api/sensors 未报告 camera 项"
        frame = None
    else:
        availability = camera.get("availability")
        reason = camera.get("reason")
        frame = camera.get("frame")
    receipt = {
        "kind": "camera_front",
        "captured_at": captured_at,
        "captured_at_epoch": round(epoch, 3),
        "timestamp": captured_at,
        "age_s": None,
        "frame": frame,
        "frame_reason": None if frame else "URDF 没有 camera_link，仿真侧没有相机 frame",
        "pose": pose_dict(state),
        "provenance": "unavailable",
        "provenance_note": "没有图像帧可溯源；unavailable 是哨兵，不属于 measured/glimpsed/inferred/unknown 四档",
        "backend": (state.get("scene") or {}).get("backend", "kinematic-shadow"),
        "physical": state.get("physical"),
        "availability": "unavailable",
        "source": "unavailable",
        "reason": reason,
        "image": None,
        "image_path": None,
        "substituted_by": None,
        "no_substitute": True,
        "workbench_not_used": True,
        "mjpeg_not_used": True,
        "sensor_snapshot": camera,
        "sim_url": sim,
        "schema_note": "契约 §5：camera_front 必须有图像与 source；仿真无相机时只输出 unavailable + 原因，"
                       "不写图片，不用工作台画面或真车 MJPEG 顶替。",
    }
    receipt_path = write_receipt(out_dir, "camera_front", stamp, receipt)
    receipt["receipt_path"] = str(receipt_path)
    return receipt


def capture_map_top(sim: str, out_dir: Path, stamp: str, map_kind: str = "truth") -> dict:
    captured_at, epoch = now_iso()
    maps_payload = fetch_json(f"{sim}/api/maps")
    state = fetch_json(f"{sim}/api/state")
    maps = maps_payload.get("maps") or {}
    entry = maps.get(map_kind) or {"availability": "missing", "reason": "后端 /api/maps 无此图"}
    availability = entry.get("availability")
    base = {
        "kind": "map_top",
        "captured_at": captured_at,
        "captured_at_epoch": round(epoch, 3),
        "timestamp": captured_at,
        "frame": "map" if map_kind == "truth" else f"map:{map_kind}",
        "pose": pose_dict(state),
        "backend": (state.get("scene") or {}).get("backend", "kinematic-shadow"),
        "physical": state.get("physical"),
        "map_kind": map_kind,
        "map_version": entry.get("map_version"),
        "availability": availability,
        "sim_url": sim,
        "maps_snapshot": entry,
    }
    if availability != "ok":
        # §4: an unavailable map never gets filled in from the truth map.
        base.update({
            "provenance": "unavailable",
            "provenance_note": "地图不可用，未渲染，未用 truth 或渲染画面顶替；unavailable 是哨兵，不落四档词表",
            "age_s": None,
            "image": None,
            "image_path": None,
            "reason": entry.get("reason") or "后端未说明原因",
            "no_substitute": True,
        })
        receipt_path = write_receipt(out_dir, f"map_top_{map_kind}", stamp, base)
        base["receipt_path"] = str(receipt_path)
        return base

    generated = float(maps_payload.get("generated_at") or epoch)
    scene = state.get("scene") or {}
    scene_name = entry.get("scene") or scene.get("name")
    world_rel = entry.get("source")
    world_sha = entry.get("world_sha256")
    world_sha_source = "backend /api/maps"
    if not world_sha and world_rel:
        world_path = (REPO_ROOT / world_rel).resolve()
        if REPO_ROOT not in world_path.parents:
            world_path = None
        if world_path is not None and world_path.is_file():
            world_sha = sha256_file(world_path)
            world_sha_source = "probe: sha256(world file)"
    map_version = entry.get("map_version")
    map_version_source = "backend /api/maps"
    if not map_version and world_sha:
        map_version = f"{map_kind}/{world_sha[:12]}"
        map_version_source = "probe: map_kind/sha256(world)[:12] (后端未提供 map_version)"
    resolution = float(entry.get("resolution_m") or 0.05)
    scene_mismatch = bool(scene_name and scene.get("name") and scene_name != scene.get("name"))
    meta = {
        "captured_at": captured_at,
        "map_version": map_version,
        "scene": scene_name,
        "world_sha256": world_sha,
        "resolution_m": resolution,
    }
    image_path = out_dir / f"map_top_{map_kind}_{stamp}.png"
    drawn = render_truth_map(scene, base["pose"], meta, image_path)
    known_cells = entry.get("known_cells") or drawn["known_cells"]
    known_basis = "backend /api/maps" if entry.get("known_cells") is not None else \
        "probe: truth 图在场景边界内全格已知（SDF 几何直接投影），known_cells = cols*rows"
    base.update({
        # Lead 口径：truth 真值图 = measured（精确几何，无估计），
        # slam/vehicle 若将来可用同样是传感器/车端实测，仍归 measured。
        "provenance": "measured",
        "provenance_note": f"SDF 真值几何的精确投影（无估计误差），按 Lead 口径记 measured；"
                           f"不是雷达建图、不是 SLAM 结果，也不是相机帧（map_kind={map_kind}）",
        "age_s": round(max(0.0, epoch - generated), 3),
        "map_version": map_version,
        "map_version_from_backend": entry.get("map_version") is not None,
        "map_version_source": map_version_source,
        "map_generated_at": round(generated, 3),
        "world_sha256": world_sha,
        "world_sha256_source": world_sha_source,
        "resolution_m": resolution,
        "known_cells": known_cells,
        "known_cells_basis": known_basis,
        "scene": scene_name,
        "scene_mismatch": scene_mismatch,
        "renderer": "sim/probe/capture.py:render_truth_map (Pillow)",
        "source_endpoints": ["/api/maps", "/api/state"],
        "image": {
            "path": str(image_path),
            "bytes": image_path.stat().st_size,
            "sha256": sha256_file(image_path),
            "width": drawn["width"],
            "height": drawn["height"],
            "primitives_drawn": drawn["primitives_drawn"],
        },
        "image_path": str(image_path),
        "no_substitute": False,
        "schema_note": "契约 §5：map_top 必须指名 map_kind 与 map_version；slam/vehicle 不可用时不得用 truth 顶替。",
    })
    if scene_mismatch:
        base["notes"] = [f"/api/maps.truth.scene={scene_name} 与 /api/state.scene.name={scene.get('name')} 不一致"]
    receipt_path = write_receipt(out_dir, f"map_top_{map_kind}", stamp, base)
    base["receipt_path"] = str(receipt_path)
    return base


def view_label(view: dict | None) -> dict | None:
    if not view or not view.get("position_m") or not view.get("target_m"):
        return None
    dx = view["position_m"][0] - view["target_m"][0]
    dy = view["position_m"][1] - view["target_m"][1]
    dz = view["position_m"][2] - view["target_m"][2]
    radius = math.sqrt(dx * dx + dy * dy + dz * dz)
    return {
        "azimuth_deg": round(math.degrees(math.atan2(dx, dz)), 2),
        "elevation_deg": round(math.degrees(math.asin(max(-1.0, min(1.0, dy / radius)))), 2) if radius else None,
        "radius_m": round(radius, 3),
        "fov_deg": view.get("fov_deg"),
        "canvas_px": view.get("canvas_px"),
    }


def capture_workbench(sim: str, out_dir: Path, stamp: str, allow_browser: bool = True) -> dict:
    captured_at, epoch = now_iso()
    state = fetch_json(f"{sim}/api/state")
    base = {
        "kind": "workbench",
        "captured_at": captured_at,
        "captured_at_epoch": round(epoch, 3),
        "timestamp": captured_at,
        "frame": "workbench:threejs-canvas",
        "pose": pose_dict(state),
        "backend": "three.js-workbench",
        "physical": state.get("physical"),
        "sim_url": sim,
        "workbench_not_camera_evidence": True,
        "camera_evidence": False,
    }
    if not allow_browser:
        base.update({
            "availability": "unavailable", "reason": "调用方禁用浏览器（--no-browser）",
            "provenance": "unavailable", "age_s": None, "image": None, "image_path": None,
        })
        receipt_path = write_receipt(out_dir, "workbench", stamp, base)
        base["receipt_path"] = str(receipt_path)
        return base
    # The browser run is retried once or twice when it hits a *transient* loopback
    # transport failure (the shadow HTTP server can refuse a connection under a
    # burst of polls without restarting).  The number of attempts is recorded so a
    # single hiccup cannot masquerade as a clean first try.
    captured: dict | None = None
    last_error: Exception | None = None
    attempts = 0
    while attempts < 3:
        attempts += 1
        try:
            candidate = capture_workbench_browser(sim)
        except (OSError, RuntimeError, subprocess.SubprocessError, ValueError) as exc:
            last_error = exc
            break
        captured = candidate
        transient = [
            item for item in (candidate.get("requestFailures") or [])
            if "ERR_CONNECTION" in str(item.get("error"))
            or "ERR_EMPTY_RESPONSE" in str(item.get("error"))
            or "ERR_NETWORK" in str(item.get("error"))
        ]
        if transient and attempts < 3:
            time.sleep(0.4 * attempts)
            continue
        break
    if captured is None:
        finish_note = finish_workbench_task()
        reason = (f"browser_capture_failed: {type(last_error).__name__}: {last_error}"
                  if last_error is not None else "browser_capture_failed: 未知原因")
        base.update({
            "availability": "unavailable",
            "reason": reason,
            "provenance": "unavailable",
            "age_s": None,
            "image": None,
            "image_path": None,
            "attempts": attempts,
            "browser": {"cli": str(tabbit_cli()), "finish": finish_note},
            "no_substitute": True,
            "schema_note": "工作台画面取不到时标 unavailable，不用旧图或其他渲染顶替。",
        })
        receipt_path = write_receipt(out_dir, "workbench", stamp, base)
        base["receipt_path"] = str(receipt_path)
        return base

    source_png = Path(captured["shot_path"])
    out_dir.mkdir(parents=True, exist_ok=True)
    image_path = out_dir / f"workbench_{stamp}.png"
    shutil.copyfile(source_png, image_path)
    finish_note = finish_workbench_task()
    view = captured.get("view") or {}
    panels = captured.get("panels") or {}
    shot_epoch = time.time()
    state_age = view.get("state_age_s")
    # Classify browser failures: a 404 for /api/strategy is the agreed graceful
    # degradation while Half A ships that endpoint, everything else is a defect.
    console_lines = captured.get("consoleErrors") or []
    http_errors = captured.get("httpErrors") or []
    request_failures = captured.get("requestFailures") or []
    expected_http = [item for item in http_errors
                     if urlparse(str(item.get("url"))).path == "/api/strategy" and item.get("status") == 404]
    unexpected_http = [item for item in http_errors if item not in expected_http]
    js_exceptions = [line for line in console_lines if not line.startswith("console:")]
    console_network_echo = [line for line in console_lines if "Failed to load resource" in line]
    console_other = [line for line in console_lines
                     if line not in js_exceptions and line not in console_network_echo]
    base.update({
        "availability": "ok",
        # age of the pose actually painted in the frame, straight from the state
        # frame stamp the workbench last received (not a guess from our own clock).
        "age_s": state_age if state_age is not None else round(max(0.0, shot_epoch - epoch), 3),
        "provenance": "glimpsed",
        "provenance_note": "浏览器渲染的工作台画面：源自工作台状态与合成雷达，是间接看到的画面（glimpsed），"
                           "不是相机帧，不可作为相机证据",
        "camera": view,
        "view": view_label(view),
        "pose_source": "window.__workbenchView + /api/state",
        "panels_snapshot": panels,
        "attempts": attempts,
        "console": {
            "js_exceptions": js_exceptions,
            "console_other": console_other,
            "raw_console_errors": console_lines,
            "network_echo_of_failures": console_network_echo,
            "http_errors": http_errors,
            "request_failures": request_failures,
            "expected_degradation": [
                {"url": item.get("url"), "status": item.get("status"),
                 "why": "/api/strategy 未部署时按约定降级（策略层未接入），不算缺陷"}
                for item in expected_http
            ],
            "unexpected_http": unexpected_http,
        },
        "image": {
            "path": str(image_path),
            "bytes": image_path.stat().st_size,
            "sha256": sha256_file(image_path),
            "source_artifact": str(source_png),
            "artifact_sha256": captured.get("shot_sha256"),
        },
        "image_path": str(image_path),
        "browser": {
            "cli": str(tabbit_cli()),
            "task": captured.get("_task"),
            "request_id": captured.get("_request_id"),
            "exit_code": captured.get("_exit_code"),
            "url": captured.get("url"),
            "finish": finish_note,
        },
        "no_substitute": False,
        "schema_note": "契约 §5：workbench 必须带 pose 与视角，并显式标注不是相机证据。",
    })
    receipt_path = write_receipt(out_dir, "workbench", stamp, base)
    base["receipt_path"] = str(receipt_path)
    return base


# --------------------------------------------------------------------- entry


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--kind", default="all",
                        choices=["all", "camera_front", "map_top", "workbench"])
    parser.add_argument("--sim", default=DEFAULT_SIM)
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT))
    parser.add_argument("--map-kind", default="truth", choices=["truth", "slam", "vehicle"])
    parser.add_argument("--no-browser", action="store_true",
                        help="工作台画面跳过浏览器采集（结果为 unavailable，不伪造）")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    sim = check_loopback(args.sim)
    out_dir = Path(args.out_dir)
    if not out_dir.is_absolute():
        out_dir = (REPO_ROOT / out_dir).resolve()
    stamp = datetime.now().strftime("%Y%m%dT%H%M%S")
    kinds = ["camera_front", "map_top", "workbench"] if args.kind == "all" else [args.kind]
    results: dict[str, dict] = {}
    for kind in kinds:
        if kind == "camera_front":
            results[kind] = capture_camera_front(sim, out_dir, stamp)
        elif kind == "map_top":
            results[kind] = capture_map_top(sim, out_dir, stamp, args.map_kind)
        else:
            results[kind] = capture_workbench(sim, out_dir, stamp, allow_browser=not args.no_browser)
    payload = {
        "ok": True,
        "sim_url": sim,
        "out_dir": str(out_dir),
        "stamp": stamp,
        # physical must be explicitly false everywhere (§7); a mixed set is a bug,
        # so it is reported as "MIXED" instead of being rounded to true/false.
        "physical": False if all(item.get("physical") is False for item in results.values()) else "MIXED",
        "captures": results,
    }
    print("CAPTURE=" + json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
    if not args.quiet:
        for kind, item in results.items():
            print(f"{kind}: availability={item.get('availability')} "
                  f"image={item.get('image_path') or '(无图片)'} receipt={item.get('receipt_path')}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (URLError, OSError, ValueError) as exc:
        print("CAPTURE=" + json.dumps({"ok": False, "error": f"{type(exc).__name__}: {exc}"}, ensure_ascii=False))
        raise SystemExit(1) from exc
