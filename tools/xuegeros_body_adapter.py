#!/usr/bin/env python3
"""Read-only HTTP adapter for the xuegeros-car-01 body."""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import cv2
import numpy as np
import requests
from ultralytics import YOLO


ROOT = Path(__file__).resolve().parents[1]
TOOL = Path(os.environ.get("XUEGEROS_TOOL", ROOT / "tools" / "xuegeros.ps1"))
CONFIG = Path(
    os.environ.get("XUEGEROS_CONFIG", ROOT / "config" / "xuegeros.env.ps1")
)
LISTEN_HOST = os.environ.get("XUEGEROS_ADAPTER_LISTEN_HOST", "127.0.0.1")
LISTEN_PORT = int(os.environ.get("XUEGEROS_ADAPTER_LISTEN_PORT", "18110"))
DEVICE_ID = os.environ.get("XUEGEROS_DEVICE_ID", "xuegeros-car-01")
TOOL_TIMEOUT_SECONDS = float(os.environ.get("XUEGEROS_TOOL_TIMEOUT_SECONDS", "45"))
STATUS_CACHE_SECONDS = float(os.environ.get("XUEGEROS_STATUS_CACHE_SECONDS", "3"))
MAPPING_CACHE_SECONDS = float(os.environ.get("XUEGEROS_MAPPING_CACHE_SECONDS", "5"))


def resolve_shell() -> str:
    """Pick the PowerShell host that actually exists on this machine.

    This host ships Windows PowerShell 5.1 only; `pwsh` (PowerShell 7) is not
    installed. Hardcoding `pwsh` made every status/mapping probe fail with
    OSError, which the world projection then reported as status_unavailable.
    An explicit XUEGEROS_SHELL always wins.
    """
    override = os.environ.get("XUEGEROS_SHELL", "").strip()
    if override:
        return override
    for candidate in ("pwsh", "powershell"):
        found = shutil.which(candidate)
        if found:
            return found
    return "powershell"


SHELL = resolve_shell()
LAST_TOOL_ERROR: str | None = None
def configured_camera_url() -> str:
    configured = os.environ.get("XUEGEROS_CAMERA_URL", "").strip()
    if configured:
        return configured
    if CONFIG.is_file():
        for line in CONFIG.read_text(encoding="utf-8").splitlines():
            match = re.match(r"\s*\$env:XUEGEROS_CAMERA_URL\s*=\s*'([^']*)'\s*$", line)
            if match:
                return match.group(1).strip()
    return ""


CAMERA_URL = configured_camera_url()
CAMERA_MAX_FRAME_BYTES = int(os.environ.get("XUEGEROS_CAMERA_MAX_FRAME_BYTES", "2000000"))
YOLO_MODEL = os.environ.get("XUEGEROS_YOLO_MODEL", "yolo11n.pt")
YOLO_IMAGE_SIZE = int(os.environ.get("XUEGEROS_YOLO_IMAGE_SIZE", "320"))
YOLO_CONFIDENCE = float(os.environ.get("XUEGEROS_YOLO_CONFIDENCE", "0.35"))
VISION_STALE_SECONDS = float(os.environ.get("XUEGEROS_VISION_STALE_SECONDS", "3"))
PREVIEW_ENABLED = os.environ.get("XUEGEROS_PREVIEW_ENABLE", "0") == "1"
PREVIEW_MAX_SECONDS = float(os.environ.get("XUEGEROS_PREVIEW_MAX_SECONDS", "300"))


class AdapterError(RuntimeError):
    pass


@dataclass
class CacheEntry:
    expires_at: float
    value: Any


class TtlCache:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._entries: dict[str, CacheEntry] = {}

    def get_or_create(self, key: str, ttl: float, factory):
        now = time.monotonic()
        with self._lock:
            entry = self._entries.get(key)
            if entry and entry.expires_at > now:
                return entry.value
        value = factory()
        with self._lock:
            self._entries[key] = CacheEntry(now + ttl, value)
        return value


CACHE = TtlCache()

class LatestFrame:
    """Single-slot volatile buffer; a newer camera frame replaces any unread one."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._frame: tuple[int, float, bytes] | None = None
        self._seq = 0
        self.dropped_frames = 0
        self.camera_state = "starting"
        self.last_error: str | None = None

    def publish(self, jpeg: bytes, observed_at: float | None = None) -> int:
        with self._condition:
            if self._frame is not None:
                self.dropped_frames += 1
            self._seq += 1
            self._frame = (self._seq, observed_at or time.time(), jpeg)
            self.camera_state = "streaming"
            self.last_error = None
            self._condition.notify_all()
            return self._seq

    def take_newest(self, after_seq: int, timeout: float = 1.0):
        with self._condition:
            self._condition.wait_for(
                lambda: self._frame is not None and self._frame[0] > after_seq,
                timeout=timeout,
            )
            if self._frame is None or self._frame[0] <= after_seq:
                return None
            frame = self._frame
            self._frame = None
            return frame

    def wait_frame(self, after_seq: int, timeout: float = 1.0):
        """Return the newest frame newer than after_seq without consuming it.

        The preview endpoint is a read-only fan-out of the same single-slot
        buffer: the YOLO worker still owns consumption, so this must not clear
        the slot. A preview reader dropping frames is expected and harmless.
        """
        with self._condition:
            self._condition.wait_for(
                lambda: self._frame is not None and self._frame[0] > after_seq,
                timeout=timeout,
            )
            if self._frame is None or self._frame[0] <= after_seq:
                return None
            return self._frame[0], self._frame[2]

    def set_state(self, state: str, error: str | None = None) -> None:
        with self._condition:
            self.camera_state = state
            self.last_error = error[:160] if error else None
            if state != "streaming":
                self._frame = None
            self._condition.notify_all()


class VisionState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._observation: dict[str, Any] = {
            "device_id": DEVICE_ID,
            "camera_state": "starting",
            "inference_state": "starting",
            "frame_seq": 0,
            "frame_age_ms": None,
            "dropped_frames": 0,
            "inference_latency_ms": None,
            "detection_count": 0,
            "detections": [],
            "observed_at": None,
        }

    def publish(self, observation: dict[str, Any]) -> None:
        with self._lock:
            self._observation = observation

    def set_inference_state(self, state: str) -> None:
        with self._lock:
            self._observation["inference_state"] = state

    def snapshot(self, camera: LatestFrame) -> dict[str, Any]:
        with self._lock:
            result = dict(self._observation)
        result["camera_state"] = camera.camera_state
        result["dropped_frames"] = camera.dropped_frames
        frame_timestamp = result.get("frame_timestamp")
        age_ms = (
            int(max(0.0, time.time() - frame_timestamp) * 1000)
            if frame_timestamp
            else None
        )
        result["frame_age_ms"] = age_ms
        if age_ms is None or age_ms > int(VISION_STALE_SECONDS * 1000) or camera.camera_state != "streaming":
            result["detections"] = []
            result["detection_count"] = 0
            if camera.camera_state != "streaming":
                result["frame_age_ms"] = None
        result["raw_frame_available"] = False
        result["raw_frame_persisted"] = False
        result["autonomous_motion_allowed"] = False
        return result


class WorldHealth:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._status: dict[str, Any] | None = None
        self._mapping: dict[str, Any] | None = None

    def publish(
        self, status: dict[str, Any] | None, mapping: dict[str, Any] | None
    ) -> None:
        with self._lock:
            self._status = status
            self._mapping = mapping

    def snapshot(self) -> tuple[dict[str, Any] | None, dict[str, Any] | None]:
        with self._lock:
            return self._status, self._mapping


LATEST_FRAME = LatestFrame()
VISION = VisionState()
WORLD_HEALTH = WorldHealth()


def extract_mjpeg_frames(response, max_frame_bytes: int = CAMERA_MAX_FRAME_BYTES):
    """Yield complete JPEG frames from an HTTP multipart or concatenated MJPEG stream."""
    frame = bytearray()
    in_frame = False
    previous = None
    for chunk in response.iter_content(chunk_size=8192):
        for byte in chunk:
            if not in_frame:
                if previous == 0xFF and byte == 0xD8:
                    frame = bytearray((0xFF, 0xD8))
                    in_frame = True
                previous = byte
                continue
            frame.append(byte)
            if len(frame) > max_frame_bytes:
                frame.clear()
                in_frame = False
                previous = None
                continue
            if previous == 0xFF and byte == 0xD9:
                yield bytes(frame)
                frame.clear()
                in_frame = False
            previous = byte


CAMERA_RECONNECT_MIN_SECONDS = float(
    os.environ.get("XUEGEROS_CAMERA_RECONNECT_MIN_SECONDS", "1")
)
CAMERA_RECONNECT_MAX_SECONDS = float(
    os.environ.get("XUEGEROS_CAMERA_RECONNECT_MAX_SECONDS", "30")
)
CAMERA_HEALTHY_RESET_SECONDS = float(
    os.environ.get("XUEGEROS_CAMERA_HEALTHY_RESET_SECONDS", "30")
)

# Set whenever a frame arrives; used to tell "the stream just broke" apart from
# "this connection was healthy for a long time", so the backoff can reset.
LAST_FRAME_AT = 0.0


def camera_capture_loop() -> None:
    """Pull the MJPEG stream, reconnecting with exponential backoff.

    The camera drops its Wi-Fi video session after long pulls (observed
    16:12-17:18 on 2026-10-23, ~3.5h into a stream). A fixed 1s retry hammered
    a camera that needs ~30s to release its previous session. Back off
    geometrically to a cap, and reset the delay once a stream has been healthy.
    """
    global LAST_FRAME_AT
    delay = CAMERA_RECONNECT_MIN_SECONDS
    while True:
        connect_started = time.monotonic()
        try:
            LATEST_FRAME.set_state("connecting")
            with requests.get(CAMERA_URL, stream=True, timeout=(3, 5)) as response:
                response.raise_for_status()
                found_frame = False
                for jpeg in extract_mjpeg_frames(response):
                    found_frame = True
                    LAST_FRAME_AT = time.monotonic()
                    LATEST_FRAME.publish(jpeg)
                if not found_frame:
                    raise AdapterError("camera stream ended without a JPEG frame")
                raise AdapterError("camera stream ended")
        except Exception as exc:
            LATEST_FRAME.set_state("disconnected", str(exc))
            healthy_for = time.monotonic() - connect_started
            if healthy_for >= CAMERA_HEALTHY_RESET_SECONDS:
                # The last connection worked for a while; treat this as a fresh
                # outage rather than a persistent failure.
                delay = CAMERA_RECONNECT_MIN_SECONDS
            time.sleep(delay)
            delay = min(delay * 2, CAMERA_RECONNECT_MAX_SECONDS)
        else:  # pragma: no cover - defensive; the loop only exits via exceptions
            delay = CAMERA_RECONNECT_MIN_SECONDS


def normalize_detections(result, width: int, height: int) -> list[dict[str, Any]]:
    detections = []
    boxes = getattr(result, "boxes", None)
    if boxes is None:
        return detections
    names = getattr(result, "names", {})
    for box in boxes:
        coords = box.xyxy[0].tolist()
        class_id = int(box.cls[0].item())
        confidence = float(box.conf[0].item())
        detections.append({
            "class": str(names.get(class_id, class_id)),
            "class_id": class_id,
            "confidence": round(max(0.0, min(1.0, confidence)), 4),
            "bbox": [
                round(max(0.0, min(1.0, float(coords[0]) / width)), 5),
                round(max(0.0, min(1.0, float(coords[1]) / height)), 5),
                round(max(0.0, min(1.0, float(coords[2]) / width)), 5),
                round(max(0.0, min(1.0, float(coords[3]) / height)), 5),
            ],
        })
        if len(detections) == 20:
            break
    return detections


def inference_loop(model=None, frame_source: LatestFrame = LATEST_FRAME, state: VisionState = VISION) -> None:
    try:
        if model is None:
            model = YOLO(YOLO_MODEL)
    except Exception:
        state.set_inference_state("model_unavailable")
        return
    last_seq = 0
    while True:
        item = frame_source.take_newest(last_seq, timeout=1.0)
        if item is None:
            continue
        seq, observed_at, jpeg = item
        last_seq = seq
        started = time.perf_counter()
        encoded = np.frombuffer(jpeg, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            state.set_inference_state("invalid_frame")
            continue
        height, width = image.shape[:2]
        try:
            result = model.predict(
                source=image,
                imgsz=YOLO_IMAGE_SIZE,
                conf=YOLO_CONFIDENCE,
                max_det=20,
                device="cpu",
                verbose=False,
                save=False,
                show=False,
            )[0]
        except Exception:
            state.set_inference_state("inference_failed")
            continue
        latency_ms = (time.perf_counter() - started) * 1000.0
        detections = normalize_detections(result, width, height)
        state.publish({
            "device_id": DEVICE_ID,
            "camera_state": "streaming",
            "inference_state": "ready",
            "frame_seq": seq,
            "frame_timestamp": observed_at,
            "observed_at": time.time(),
            "frame_age_ms": int(max(0.0, time.time() - observed_at) * 1000),
            "dropped_frames": frame_source.dropped_frames,
            "inference_latency_ms": round(latency_ms, 2),
            "inference_fps": round(1000.0 / latency_ms, 2) if latency_ms else None,
            "detection_count": len(detections),
            "detections": detections,
        })


def world_health_loop() -> None:
    while True:
        try:
            status = get_status()
        except AdapterError:
            status = None
        try:
            mapping = get_mapping()
        except AdapterError:
            mapping = None
        WORLD_HEALTH.publish(status, mapping)
        time.sleep(5.0)


def _run_tool(action: str, *extra_args: str) -> Any:
    if not TOOL.is_file():
        raise AdapterError(f"XuegerOS tool not found: {TOOL}")

    config_prefix = f'. "{CONFIG}"; ' if CONFIG.is_file() else ""
    quoted_args = " ".join(f'"{arg}"' for arg in extra_args)
    command = (
        f'{config_prefix}& "{TOOL}" {action} -Json {quoted_args}'
    ).strip()
    try:
        completed = subprocess.run(
            [SHELL, "-NoProfile", "-Command", command],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=TOOL_TIMEOUT_SECONDS,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        _note_tool_error(f"tool execution failed ({SHELL}): {exc}")
        raise AdapterError(f"tool execution failed: {exc}") from exc

    if completed.returncode != 0:
        detail = completed.stderr or completed.stdout or ""
        detail = detail.strip()
        _note_tool_error(f"tool exited with {completed.returncode}: {detail[:300]}")
        raise AdapterError(f"tool exited with {completed.returncode}: {detail[:500]}")

    output = completed.stdout.strip()
    if not output:
        _note_tool_error("tool returned no output")
        raise AdapterError("tool returned no output")
    try:
        payload = json.loads(output)
    except json.JSONDecodeError as exc:
        _note_tool_error(f"tool returned invalid JSON: {exc}")
        raise AdapterError(f"tool returned invalid JSON: {exc}") from exc
    _note_tool_error(None)
    return payload


def _note_tool_error(message: str | None) -> None:
    """Remember the last tool failure so /v1/world can explain a null probe."""
    global LAST_TOOL_ERROR
    LAST_TOOL_ERROR = message


def get_battery() -> dict[str, Any] | None:
    """Battery voltage/percentage via the guest probe, or None when unavailable.

    /battery_state is BEST_EFFORT QoS; the tool's probe subscribes with matching
    QoS, so a null here means "no reading", not "topic missing".
    """
    try:
        payload = _run_tool("battery")
    except AdapterError:
        return None
    if not isinstance(payload, dict) or not payload.get("available"):
        return None
    return {
        "voltage": payload.get("voltage"),
        "percentage": payload.get("percentage"),
    }


def _extract_guest_ip(guest_text: str) -> str | None:
    match = re.search(r"\b(\d{1,3}(?:\.\d{1,3}){3})/\d+\b", guest_text)
    return match.group(1) if match else None


def _guest_has_process(guest_text: str, process_name: str) -> bool:
    processes = guest_text.split("PROCESSES", 1)[-1].split("UDP", 1)[0]
    return process_name in processes


def normalize_status(payload: dict[str, Any]) -> dict[str, Any]:
    host = payload.get("host") or {}
    guest = str(payload.get("guest") or "")
    endpoints = host.get("udpEndpoints") or []
    udp_ports = {
        int(endpoint.get("LocalPort", 0))
        for endpoint in endpoints
        if isinstance(endpoint, dict)
    }
    agent_log = guest.split("AGENT_LOG", 1)[-1].lower()
    session_observed = bool(
        re.search(r"client_key:\s*0x0*[1-9a-f]", agent_log)
    ) or any(
        marker in agent_log
        for marker in ("session established", "session created", "create_session")
    )

    return {
        "device_id": DEVICE_ID,
        "observed_at": time.time(),
        "vm": {
            "running": bool(host.get("vmRunning")),
            "vm_count": host.get("vmCount"),
            "tools_state": host.get("toolsState"),
        },
        "network": {
            "guest_ip": _extract_guest_ip(guest),
            "udp_8888_listening": 8888 in udp_ports,
            "udp_8889_listening": 8889 in udp_ports,
        },
        "micro_ros": {
            "agent_running": _guest_has_process(guest, "micro_ros_agent"),
            "session_observed": session_observed,
        },
        "camera": {
            "ros_node_running": _guest_has_process(guest, "http_video_publisher"),
            "ros_topic": "/camera/image_raw/compressed",
        },
        "mapping": {
            "gmapping_running": _guest_has_process(guest, "slam_gmapping")
            or _guest_has_process(guest, "gmapping"),
        },
        "navigation": {
            "nav2_running": _guest_has_process(guest, "nav2"),
        },
        "battery": get_battery(),
        "autonomous_motion_allowed": False,
        "raw_logs_included": False,
    }


def summarize_mapping(payload: dict[str, Any]) -> dict[str, Any]:
    if isinstance(payload, str):
        scan_present = "/scan" in payload and "Unknown topic '/scan'" not in payload
        map_present = "/map" in payload and "Unknown topic '/map'" not in payload
        tf_present = "/tf" in payload and "Unknown topic '/tf'" not in payload
        gmapping_running = "slam_gmapping" in payload or "gmapping" in payload
    else:
        scan_present = bool(payload.get("scan_present"))
        map_present = bool(payload.get("map_present"))
        tf_present = bool(payload.get("tf_present"))
        gmapping_running = bool(payload.get("gmapping_running"))

    return {
        "device_id": DEVICE_ID,
        "observed_at": time.time(),
        "scan": {"present": scan_present},
        "map": {"present": map_present},
        "tf": {"present": tf_present},
        "gmapping_running": gmapping_running,
        "mapping_healthy": scan_present and map_present and tf_present,
        "raw_logs_included": False,
    }


def get_status() -> dict[str, Any]:
    return CACHE.get_or_create(
        "status",
        STATUS_CACHE_SECONDS,
        lambda: normalize_status(_run_tool("status")),
    )


def get_mapping() -> dict[str, Any]:
    return CACHE.get_or_create(
        "mapping",
        MAPPING_CACHE_SECONDS,
        lambda: summarize_mapping(_run_tool("mapping-status")),
    )


def summarize_navigation(
    status: dict[str, Any], mapping: dict[str, Any]
) -> dict[str, Any]:
    ready = bool(
        status["vm"]["running"]
        and status["micro_ros"]["session_observed"]
        and status["navigation"]["nav2_running"]
        and mapping["scan"]["present"]
        and mapping["map"]["present"]
        and mapping["tf"]["present"]
    )
    return {
        "device_id": DEVICE_ID,
        "observed_at": time.time(),
        "navigation_ready": ready,
        "autonomous_motion_allowed": False,
        "blockers": [
            blocker
            for blocker, blocked in (
                ("micro_ros_session_missing", not status["micro_ros"]["session_observed"]),
                ("nav2_not_running", not status["navigation"]["nav2_running"]),
                ("scan_missing", not mapping["scan"]["present"]),
                ("map_missing", not mapping["map"]["present"]),
                ("tf_missing", not mapping["tf"]["present"]),
            )
            if blocked
        ],
    }


def get_navigation() -> dict[str, Any]:
    return summarize_navigation(get_status(), get_mapping())


def get_observation() -> dict[str, Any]:
    return VISION.snapshot(LATEST_FRAME)


def build_world(status, mapping, vision: dict[str, Any]) -> dict[str, Any]:
    status_available = status is not None
    mapping_available = mapping is not None
    navigation = (
        summarize_navigation(status, mapping)
        if status_available and mapping_available
        else {
            "navigation_ready": False,
            "autonomous_motion_allowed": False,
            "blockers": [
                name
                for name, available in (
                    ("status_unavailable", status_available),
                    ("mapping_unavailable", mapping_available),
                )
                if not available
            ],
        }
    )
    return {
        "device_id": DEVICE_ID,
        "observed_at": time.time(),
        "read_only": True,
        "status_available": status_available,
        "mapping_available": mapping_available,
        "navigation_available": status_available and mapping_available,
        "shell": SHELL,
        "status_error": None if status_available else LAST_TOOL_ERROR,
        "status": status,
        "mapping": mapping,
        "vision": vision,
        "navigation": navigation,
        "autonomous_motion_allowed": False,
        "raw_frames_available": False,
    }


def get_world() -> dict[str, Any]:
    status, mapping = WORLD_HEALTH.snapshot()
    return build_world(status, mapping, get_observation())


class Handler(BaseHTTPRequestHandler):
    server_version = "XuegerosBodyAdapter/1.0"

    def _send_json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode(
            "utf-8"
        )
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        try:
            if path == "/health":
                self._send_json(200, {"ok": True, "device_id": DEVICE_ID})
                return
            if path == "/v1/status":
                self._send_json(200, get_status())
                return
            if path == "/v1/mapping":
                self._send_json(200, get_mapping())
                return
            if path == "/v1/navigation":
                self._send_json(200, get_navigation())
                return
            if path == "/v1/observations/latest":
                self._send_json(200, get_observation())
                return
            if path == "/v1/world":
                self._send_json(200, get_world())
                return
            if path == "/v1/preview.mjpeg":
                if not PREVIEW_ENABLED:
                    self._send_json(404, {"error": "preview_disabled"})
                    return
                self._stream_preview()
                return
            self._send_json(404, {"error": "not_found"})
        except AdapterError as exc:
            self._send_json(503, {"error": "adapter_unavailable", "detail": str(exc)[:500]})
        except Exception as exc:  # pragma: no cover - defensive boundary
            self._send_json(500, {"error": "adapter_error", "detail": str(exc)[:500]})

    def do_POST(self) -> None:  # noqa: N802
        self._send_json(
            405,
            {
                "error": "read_only_adapter",
                "detail": "Motion and configuration writes are not available here.",
            },
        )
    def _stream_preview(self) -> None:
        """Fan out the latest decoded JPEG as multipart.

        The camera only tolerates a small number of concurrent video sessions,
        so the browser cockpit reads this loopback stream instead of opening a
        second connection to the camera. Nothing is written to disk.
        """
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        seq = 0
        deadline = time.monotonic() + PREVIEW_MAX_SECONDS
        try:
            while time.monotonic() < deadline:
                item = LATEST_FRAME.wait_frame(seq, timeout=1.0)
                if item is None:
                    continue
                seq, jpeg = item
                self.wfile.write(
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(jpeg)).encode("ascii")
                    + b"\r\n\r\n"
                )
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            # A browser tab closing or reloading aborts the socket mid-write on
            # Windows with WinError 10053; that is a normal disconnect, not an
            # adapter failure.
            pass

    def log_message(self, fmt: str, *args) -> None:
        print(f"{self.address_string()} - {fmt % args}", flush=True)


def main() -> None:
    threading.Thread(target=camera_capture_loop, name="camera-capture", daemon=True).start()
    threading.Thread(target=inference_loop, name="vision-yolo", daemon=True).start()
    threading.Thread(target=world_health_loop, name="world-health", daemon=True).start()
    server = ThreadingHTTPServer((LISTEN_HOST, LISTEN_PORT), Handler)
    print(
        f"XuegerOS read-only adapter listening on http://{LISTEN_HOST}:{LISTEN_PORT}",
        flush=True,
    )
    server.serve_forever()


if __name__ == "__main__":
    main()
