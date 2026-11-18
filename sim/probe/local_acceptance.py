"""Verify the local Coo -> robotd -> kinematic shadow driving loop plus the
docs/SIM-CONTRACT.md §3/§4/§5 observation surface.

Three things are judged here, each with its own receipts:

1. **driving** - the same Coo WebUI panel endpoints the desktop app uses: one
   ``forward 0.1 m`` goal must reach ``DONE`` and move the pose, then ``stop``
   must be accepted.  The physical robotd port (``18202``) must stay closed and
   every payload must say ``physical=false``.
2. **observation contract** - ``/api/sensors`` and ``/api/maps`` must carry the
   §3/§4 fields; every unavailable entry must give a ``reason``; no unavailable
   map may borrow a version/resolution from the truth map; a shadow backend
   (``physical=false``) may not claim ``source=ros`` anywhere.
3. **screenshot contract** - the three §5 exits from ``sim/probe/capture.py``
   are run and their receipts are attached: ``camera_front`` must be
   ``unavailable`` with a reason and **no image**, ``map_top`` must name
   ``map_kind`` + ``map_version``, ``workbench`` must carry pose and view and be
   stamped as not-camera-evidence.

Loopback only.  Exit 0 means every assertion held; any assertion failure prints
``{"ok": false, ...}`` and exits 1.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import socket
import sys
import time
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

PROBE_DIR = Path(__file__).resolve().parent
REPO_ROOT = PROBE_DIR.parents[1]

SENSOR_FIELDS = ("id", "label", "frame", "topic", "msg_type", "rate_hz", "mount",
                 "source", "substitute", "age_s", "availability", "reason", "controls")
SENSOR_AVAILABILITY = {"ok", "stale", "missing", "unavailable"}
SENSOR_SOURCES = {"sim", "ros", "unavailable"}
MAP_KINDS = ("truth", "slam", "vehicle")
MAP_AVAILABILITY = SENSOR_AVAILABILITY
PROVENANCE_TIERS = {"measured", "glimpsed", "inferred", "unknown"}


def fetch_json(url: str, *, method: str = "GET", payload: dict | None = None, timeout: float = 5.0,
               attempts: int = 3) -> dict:
    """Request JSON, retrying transient loopback refusals.

    The shadow backend is a plain ThreadingHTTPServer: under a burst of panel
    polls plus a capture it can refuse a connection for a moment even though it
    never restarted.  HTTP status errors are not retried.
    """
    body = None if payload is None else json.dumps(payload).encode()
    last: Exception | None = None
    for index in range(max(1, attempts)):
        request = Request(url, data=body, method=method, headers={"content-type": "application/json"})
        try:
            with urlopen(request, timeout=timeout) as response:  # noqa: S310 - fixed loopback URL from CLI
                return json.loads(response.read().decode())
        except HTTPError:
            raise
        except URLError as exc:
            last = exc
            time.sleep(0.15 * (index + 1))
    raise last if last is not None else RuntimeError(f"fetch failed: {url}")


def fetch_optional(url: str) -> tuple[int, dict | None]:
    """GET that tolerates 404 (an endpoint that is not deployed yet)."""
    try:
        with urlopen(url, timeout=5) as response:  # noqa: S310
            return response.status, json.loads(response.read().decode())
    except URLError as exc:
        status = getattr(exc, "code", None)
        if status == 404:
            return 404, None
        raise


def port_open(host: str, port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(0.5)
        return sock.connect_ex((host, port)) == 0


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def load_capture_module():
    spec = importlib.util.spec_from_file_location("xuegeros_capture", PROBE_DIR / "capture.py")
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader, "cannot load sim/probe/capture.py"
    spec.loader.exec_module(module)
    return module


# ------------------------------------------------------------------ §3 sensors


def check_sensors(payload: dict) -> dict:
    require(payload.get("physical") is False, {"sensors.physical": payload.get("physical")})
    require(isinstance(payload.get("generated_at"), (int, float)), "sensors 缺少 generated_at 时间戳")
    sensors = payload.get("sensors")
    require(isinstance(sensors, list) and sensors, "sensors 为空")
    by_id: dict[str, dict] = {}
    reasons = {}
    for sensor in sensors:
        missing = [field for field in SENSOR_FIELDS if field not in sensor]
        require(not missing, {"sensor_missing_fields": sensor.get("id"), "missing": missing})
        sid = sensor["id"]
        require(isinstance(sid, str) and sid, f"sensor id 非法: {sid!r}")
        require(sensor["availability"] in SENSOR_AVAILABILITY,
                {"sensor": sid, "availability": sensor["availability"]})
        require(sensor["source"] in SENSOR_SOURCES, {"sensor": sid, "source": sensor["source"]})
        require(isinstance(sensor["controls"], bool), {"sensor": sid, "controls": sensor["controls"]})
        require(isinstance(sensor["label"], str) and sensor["label"], {"sensor": sid, "label": sensor["label"]})
        # §3: missing fields must be reported as unavailable, and unavailable
        # entries must always say why.
        if sensor["availability"] == "unavailable":
            reason = sensor["reason"]
            require(isinstance(reason, str) and reason.strip(),
                    {"sensor": sid, "problem": "unavailable 但 reason 为空"})
            require(sensor["source"] == "unavailable",
                    {"sensor": sid, "problem": "unavailable 却声明了 source", "source": sensor["source"]})
            reasons[sid] = reason
        else:
            # §3 sample: reason is null while the sensor is fine, but whatever is
            # there must be a string (no numbers or objects leaking into the UI).
            require(sensor["reason"] is None or isinstance(sensor["reason"], str),
                    {"sensor": sid, "problem": f"{sensor['availability']} 但 reason 不是字符串",
                     "reason": sensor["reason"]})
        if sensor["availability"] == "ok":
            require(isinstance(sensor["frame"], str) and sensor["frame"],
                    {"sensor": sid, "problem": "ok 但没有 frame"})
            require(isinstance(sensor["topic"], str) and sensor["topic"].startswith("/"),
                    {"sensor": sid, "problem": "ok 但 topic 不是 ROS topic", "topic": sensor["topic"]})
            require(isinstance(sensor["age_s"], (int, float)) and sensor["age_s"] >= 0,
                    {"sensor": sid, "problem": "ok 但 age_s 非法", "age_s": sensor["age_s"]})
        # §7: shadow backend (physical=false) must never claim a real ROS source.
        require(sensor["source"] != "ros",
                {"sensor": sid, "problem": "physical=false 却声明 source=ros"})
        by_id[sid] = sensor

    require("lidar" in by_id, "sensors 未包含 lidar")
    light = by_id["lidar"]
    require(light["availability"] == "ok", {"lidar": light["availability"]})
    require(light["frame"] == "laser_frame", {"lidar.frame": light["frame"]})
    require(light["topic"] == "/scan", {"lidar.topic": light["topic"]})
    require(light["source"] == "sim" and light["substitute"],
            {"lidar": "合成雷达必须写清 source=sim 与 substitute", "source": light["source"]})

    # §3 + §7: the camera is the standing anti-substitution guard.  It must stay
    # unavailable with a reason - an appearance (camera.stl) or an MJPEG stream is
    # not a control-grade image source.
    require("camera" in by_id, "sensors 未包含 camera")
    camera = by_id["camera"]
    require(camera["availability"] == "unavailable",
            {"camera": "仿真侧没有相机，availability 必须是 unavailable", "got": camera["availability"]})
    require(camera["topic"] is None, {"camera.topic": camera["topic"]})
    require(camera["controls"] is False, {"camera.controls": camera["controls"]})
    return {
        "count": len(sensors),
        "ok": sorted(sid for sid, item in by_id.items() if item["availability"] == "ok"),
        "unavailable": sorted(reasons),
        "unavailable_reasons": reasons,
        "camera": {key: camera[key] for key in ("availability", "frame", "topic", "source", "controls", "reason")},
        "lidar": {key: light[key] for key in ("availability", "frame", "topic", "source", "substitute", "age_s")},
    }


# --------------------------------------------------------------------- §4 maps


def check_maps(payload: dict) -> dict:
    require(payload.get("physical") is False, {"maps.physical": payload.get("physical")})
    maps = payload.get("maps")
    require(isinstance(maps, dict), "maps 不是对象")
    compact = {}
    for kind in MAP_KINDS:
        require(kind in maps, f"maps 缺少 {kind} 图")
        entry = maps[kind]
        availability = entry.get("availability")
        require(availability in MAP_AVAILABILITY, {"map": kind, "availability": availability})
        if availability == "unavailable":
            reason = entry.get("reason")
            require(isinstance(reason, str) and reason.strip(),
                    {"map": kind, "problem": "unavailable 但 reason 为空"})
            # §4: an unavailable map must not inherit version/resolution/cells
            # from the truth map.
            for field in ("map_version", "known_cells", "resolution_m"):
                require(entry.get(field) is None,
                        {"map": kind, "problem": f"unavailable 却带了 {field}", "value": entry.get(field)})
        else:
            require(isinstance(entry.get("provenance"), str) and entry["provenance"],
                    {"map": kind, "problem": "可用地图必须有 provenance"})
            require(isinstance(entry.get("resolution_m"), (int, float)) and entry["resolution_m"] > 0,
                    {"map": kind, "problem": "可用地图必须有正分辨率"})
        compact[kind] = {
            "availability": availability,
            "provenance": entry.get("provenance"),
            "map_version": entry.get("map_version"),
            "resolution_m": entry.get("resolution_m"),
            "known_cells": entry.get("known_cells"),
            "scene": entry.get("scene"),
            "reason": entry.get("reason"),
        }
    require(maps["truth"]["availability"] == "ok", {"truth": maps["truth"].get("availability")})
    require(maps["truth"].get("provenance"), {"truth": "缺少 provenance"})
    # truth alone must exist today; slam/vehicle are unavailable by backend design
    # and must say so instead of borrowing truth.
    require(maps["slam"]["availability"] != "ok",
            {"slam": "本机影子后端不做 SLAM，不得报 ok", "got": maps["slam"].get("availability")})
    return compact


# ---------------------------------------------------------------- §5 captures


def check_captures(capture, sim: str, out_dir: Path, stamp: str, allow_browser: bool,
                   strict: bool) -> tuple[dict, list[str]]:
    warnings: list[str] = []
    receipts: dict[str, dict] = {}

    front = capture.capture_camera_front(sim, out_dir, stamp)
    require(front.get("availability") == "unavailable",
            {"camera_front": "仿真侧无相机，必须 unavailable", "got": front.get("availability")})
    require(isinstance(front.get("reason"), str) and front["reason"].strip(),
            "camera_front 缺 reason")
    require(front.get("image_path") is None and front.get("image") is None,
            {"camera_front": "不得写出图片（不得用工作台/MJPEG 顶替）", "image": front.get("image_path")})
    require(front.get("no_substitute") is True and front.get("workbench_not_used") is True
            and front.get("mjpeg_not_used") is True, "camera_front 缺少不顶替标记")
    require(front.get("physical") is False, {"camera_front.physical": front.get("physical")})
    require(front.get("provenance") == "unavailable",
            {"camera_front.provenance": "无图像帧时 provenance 必须是 unavailable 哨兵",
             "got": front.get("provenance")})
    require(front.get("frame") is None, {"camera_front.frame": front.get("frame")})
    require(front.get("captured_at") and "age_s" in front, "camera_front 缺 captured_at/age_s")
    receipts["camera_front"] = {
        "availability": front["availability"], "frame": front.get("frame"),
        "provenance": front.get("provenance"), "reason": front.get("reason"),
        "image_path": None, "receipt_path": front.get("receipt_path"),
        "pose": front.get("pose"), "age_s": front.get("age_s"), "physical": front.get("physical"),
    }

    top = capture.capture_map_top(sim, out_dir, stamp, "truth")
    require(top.get("availability") == "ok", {"map_top": top.get("availability"), "reason": top.get("reason")})
    require(top.get("map_kind") == "truth", {"map_top.map_kind": top.get("map_kind")})
    require(isinstance(top.get("map_version"), str) and top["map_version"].strip(),
            {"map_top": "map_version 缺失", "value": top.get("map_version")})
    require(top.get("provenance") in PROVENANCE_TIERS | {"unavailable"},
            {"map_top.provenance": top.get("provenance")})
    require(top.get("physical") is False, {"map_top.physical": top.get("physical")})
    require(isinstance(top.get("age_s"), (int, float)), {"map_top.age_s": top.get("age_s")})
    require(isinstance(top.get("pose"), dict), "map_top 缺 pose")
    image = top.get("image") or {}
    image_path = Path(top.get("image_path") or "")
    require(image_path.is_file(), {"map_top": "图片未落盘", "path": str(image_path)})
    require(capture.sha256_file(image_path) == image.get("sha256"),
            {"map_top": "图片 sha256 与元数据不一致"})
    receipts["map_top"] = {
        "availability": top["availability"], "map_kind": top.get("map_kind"),
        "map_version": top.get("map_version"), "map_version_source": top.get("map_version_source"),
        "world_sha256": top.get("world_sha256"), "resolution_m": top.get("resolution_m"),
        "known_cells": top.get("known_cells"), "provenance": top.get("provenance"),
        "age_s": top.get("age_s"), "frame": top.get("frame"), "pose": top.get("pose"),
        "image_path": str(image_path), "image_sha256": image.get("sha256"),
        "image_bytes": image.get("bytes"), "receipt_path": top.get("receipt_path"),
    }

    bench = capture.capture_workbench(sim, out_dir, stamp, allow_browser=allow_browser)
    if bench.get("availability") == "ok":
        require(isinstance(bench.get("pose"), dict) and {"x", "y", "yaw"} <= set(bench["pose"]),
                {"workbench.pose": bench.get("pose")})
        view = bench.get("view") or {}
        require(isinstance(view.get("azimuth_deg"), (int, float)) and isinstance(view.get("elevation_deg"), (int, float)),
                {"workbench.view": view})
        require(bench.get("workbench_not_camera_evidence") is True and bench.get("camera_evidence") is False,
                "workbench 未标注不是相机证据")
        require(bench.get("provenance") == "glimpsed",
                {"workbench.provenance": bench.get("provenance"), "expected": "glimpsed"})
        require(bench.get("physical") is False, {"workbench.physical": bench.get("physical")})
        require(isinstance(bench.get("age_s"), (int, float)), {"workbench.age_s": bench.get("age_s")})
        bench_image = Path(bench.get("image_path") or "")
        require(bench_image.is_file(), {"workbench": "图片未落盘", "path": str(bench_image)})
        require(capture.sha256_file(bench_image) == (bench.get("image") or {}).get("sha256"),
                {"workbench": "图片 sha256 与元数据不一致"})
        console = bench.get("console") or {}
        require(not console.get("js_exceptions"), {"workbench.js_exceptions": console.get("js_exceptions")})
        require(not console.get("unexpected_http"), {"workbench.unexpected_http": console.get("unexpected_http")})
        require(not console.get("request_failures"), {"workbench.request_failures": console.get("request_failures")})
        panels = bench.get("panels_snapshot") or {}
        sensors_panel = panels.get("sensors") or []
        maps_panel = panels.get("maps") or []
        require(len(sensors_panel) >= 4, {"workbench.sensors_panel": sensors_panel})
        require(len(maps_panel) == 3, {"workbench.maps_panel": maps_panel})
        receipts["workbench"] = {
            "availability": "ok", "frame": bench.get("frame"), "pose": bench.get("pose"),
            "provenance": bench.get("provenance"), "age_s": bench.get("age_s"),
            "view": view, "camera_position_m": (bench.get("camera") or {}).get("position_m"),
            "attempts": bench.get("attempts"),
            "workbench_not_camera_evidence": True,
            "image_path": str(bench_image), "image_sha256": (bench.get("image") or {}).get("sha256"),
            "image_bytes": (bench.get("image") or {}).get("bytes"),
            "receipt_path": bench.get("receipt_path"),
            "panels_snapshot": {
                "strategy": panels.get("strategy"),
                "goal_buttons": panels.get("goal_buttons"),
                "sensors": sensors_panel,
                "maps": maps_panel,
                "stamp": panels.get("stamp"),
                "physical": panels.get("physical"),
            },
            "console": {
                "js_exceptions": console.get("js_exceptions"),
                "unexpected_http": console.get("unexpected_http"),
                "request_failures": console.get("request_failures"),
                "expected_degradation_count": len(console.get("expected_degradation") or []),
            },
        }
    else:
        message = f"workbench 采集不可用：{bench.get('reason')}"
        if strict:
            raise AssertionError(message)
        warnings.append(message)
        receipts["workbench"] = {
            "availability": bench.get("availability"), "reason": bench.get("reason"),
            "image_path": None, "receipt_path": bench.get("receipt_path"),
            "physical": bench.get("physical"),
        }
    return receipts, warnings


# ---------------------------------------------------------------------- entry


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--coo", default="http://127.0.0.1:17788")
    parser.add_argument("--sim", default="http://127.0.0.1:18142")
    parser.add_argument("--physical-port", type=int, default=18202)
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--capture-dir", default=str(REPO_ROOT / "tmp" / "captures"))
    parser.add_argument("--no-captures", action="store_true", help="只验契约，不跑三种截图")
    parser.add_argument("--no-browser", action="store_true",
                        help="跳过工作台浏览器采集（结果为 unavailable，不算失败）")
    parser.add_argument("--strict-captures", action="store_true",
                        help="工作台截图不可用时按失败处理")
    args = parser.parse_args()

    panel = f"{args.coo}/api/webui/providers/world%3Axuegeros/panels/live"
    before = fetch_json(f"{panel}/state")
    sim_before = fetch_json(f"{args.sim}/api/state")
    require(before.get("deployment") == "simulation", {"deployment": before.get("deployment")})
    require(before.get("physical") is False, {"coo.physical": before.get("physical")})
    require(sim_before.get("physical") is False, {"sim.physical": sim_before.get("physical")})
    require(not port_open("127.0.0.1", args.physical_port), "physical robotd port is reachable")

    # §3/§4 observation contract, before any motion so the state is the idle one.
    sensors_receipt = check_sensors(fetch_json(f"{args.sim}/api/sensors"))
    maps_receipt = check_maps(fetch_json(f"{args.sim}/api/maps"))

    # The strategy layer may not be deployed yet: 404 means "not connected", and
    # a busy strategy would mean somebody else holds the goal (§1 mutex).
    strategy_status, strategy_payload = fetch_optional(f"{args.sim}/api/strategy")
    strategy_receipt: dict = {"status": strategy_status}
    if strategy_status == 200 and strategy_payload is not None:
        strategy_receipt.update({
            "state": strategy_payload.get("state"),
            "strategy": strategy_payload.get("strategy"),
            "phase": strategy_payload.get("phase"),
            "busy": strategy_payload.get("busy"),
            "events_cursor": strategy_payload.get("events_cursor"),
        })
        require(isinstance(strategy_payload.get("busy"), bool),
                {"strategy.busy": strategy_payload.get("busy")})
        require("facts" in strategy_payload, "strategy 回执缺少 facts")
        strategy_receipt["shape_ok"] = True

    goal = fetch_json(
        f"{panel}/goal",
        method="POST",
        payload={"args": [{"intent": "forward", "sector": 6, "distance_m": 0.1}]},
    )
    require(goal.get("dispatched") is True, goal)
    deadline = time.monotonic() + args.timeout
    after_goal = before
    while time.monotonic() < deadline:
        after_goal = fetch_json(f"{panel}/state")
        if after_goal.get("state") == "DONE" and after_goal.get("pose") != before.get("pose"):
            break
        time.sleep(0.1)
    require(after_goal.get("state") == "DONE", after_goal)
    require(after_goal.get("pose") != before.get("pose"),
            {"before": before.get("pose"), "after": after_goal.get("pose")})

    stop = fetch_json(f"{panel}/stop", method="POST", payload={"args": []})
    require(stop.get("accepted") is True, stop)
    sim_after = fetch_json(f"{args.sim}/api/state")
    require(sim_after.get("physical") is False, {"sim.physical": sim_after.get("physical")})
    require(not port_open("127.0.0.1", args.physical_port), "physical robotd port reopened")

    capture_receipts: dict = {}
    warnings: list[str] = []
    if not args.no_captures:
        capture = load_capture_module()
        sim_guarded = capture.check_loopback(args.sim)
        out_dir = Path(args.capture_dir)
        stamp = time.strftime("%Y%m%dT%H%M%S")
        capture_receipts, warnings = check_captures(
            capture, sim_guarded, out_dir, stamp,
            allow_browser=not args.no_browser, strict=args.strict_captures)

    receipt = {
        "ok": True,
        "deployment": before.get("deployment"),
        "physical": sim_after.get("physical"),
        "physical_port_open": port_open("127.0.0.1", args.physical_port),
        "pose_before": before.get("pose"),
        "pose_after": after_goal.get("pose"),
        "goal": goal,
        "stop": stop,
        "sensors": sensors_receipt,
        "maps": maps_receipt,
        "strategy": strategy_receipt,
        "captures": capture_receipts,
        "warnings": warnings,
    }
    print(json.dumps(receipt, ensure_ascii=False, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AssertionError, OSError, URLError, json.JSONDecodeError) as exc:
        detail = exc.args[0] if isinstance(exc, AssertionError) and exc.args else str(exc)
        print(json.dumps({"ok": False, "error": detail}, ensure_ascii=False, default=str))
        raise SystemExit(1)
