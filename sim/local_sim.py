"""Lightweight XuegerOS simulator with optional LAN binding.

This is the local integration target for Coo while Gazebo/ROS is unavailable on
Windows. It reuses the production robotd protocol and Cerebellum, but replaces
the ROS base with a bounded differential-drive kinematic loop. No ROS imports,
SSH, physical gateway, or real-robot endpoint are used.

Besides the scene/state/sensors/maps endpoints, the HTTP side exposes the
long-task strategy from ``agent_control/strategy.py`` (contract §6)::

    GET  /api/strategy[?since=<cursor>]       current state + facts + events
    POST /api/strategy/start|pause|resume|cancel|stop|handover

A strategy run holds the single goal lease (contract §1): while it is live, a
single-step ``goal`` over the WebSocket answers ``{"accepted": false, "busy":
true, "error": "busy"}`` instead of driving the car behind the tour's back.

Run from the repository root::

    python -m sim.local_sim --port 18242

The Coo World can then use ``ws://127.0.0.1:18242/ws`` (panel workbench on
``http://127.0.0.1:18142/``). Port 18202 is the *physical* robotd socket and must
stay closed here: ``sim/probe/local_acceptance.py`` asserts it. The loopback
server defaults to 127.0.0.1; --host 0.0.0.0 enables LAN sharing.
"""

from __future__ import annotations

import argparse
import hashlib
import ipaddress
import json
import math
import mimetypes
import re
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Thread, RLock
from xml.etree import ElementTree
from urllib.parse import urlsplit

from agent_control.cerebellum import Cerebellum, CerebellumDecision, profile_scan
from agent_control.coverage import (
    DEFAULT_VARIANT_ID,
    VariantError,
    available_variants,
    load_variant,
)
from agent_control.explorer import GridMap
from agent_control.robotd import DaemonServer, RobotDaemon
from agent_control.strategy import STRATEGIES, StrategyPhase, StrategyRunner

ROOT = Path(__file__).resolve().parents[1]
SIM_ASSETS = ROOT / "sim" / "xuegeros_sim"
WEB_ROOT = ROOT / "sim" / "local_web"
THREE_ROOT = ROOT / "mcp-xuegeros" / "xuegeros_mcp" / "vendor" / "three"
STRATEGY_ACTIONS = ("start", "pause", "resume", "cancel", "stop", "handover")
SEEN_STRIDE = 1           # every actual lidar ray, no invented free-space fill
SEEN_STEP_M = 0.025       # <= half a cell: do not leave artificial map holes


@dataclass
class Scene:
    """2D collision and lidar projection of the selected Gazebo SDF world."""

    name: str
    source: str
    width: float
    height: float
    obstacles: list[dict[str, object]]
    robot_radius: float = math.hypot(0.0725, 0.081)

    @staticmethod
    def _pose(node: ElementTree.Element | None) -> tuple[float, float, float, float, float, float]:
        values = [float(value) for value in (node.text or "").split()] if node is not None else []
        values += [0.0] * (6 - len(values))
        return tuple(values[:6])  # type: ignore[return-value]

    @classmethod
    def from_sdf(cls, path: Path) -> "Scene":
        root = ElementTree.parse(path).getroot()
        world = root.find("world")
        if world is None:
            raise ValueError(f"SDF has no world: {path}")
        obstacles: list[dict[str, object]] = []
        bounds_x: list[float] = []
        bounds_y: list[float] = []
        for model in world.findall("model"):
            model_pose = cls._pose(model.find("pose"))
            for link in model.findall("link"):
                link_pose = cls._pose(link.find("pose"))
                for collision in link.findall("collision"):
                    collision_pose = cls._pose(collision.find("pose"))
                    geometry = collision.find("geometry")
                    if geometry is None:
                        continue
                    box = geometry.find("box/size")
                    cylinder = geometry.find("cylinder")
                    if box is not None:
                        dims = [float(value) for value in (box.text or "").split()]
                        if len(dims) != 3:
                            continue
                        shape = {"type": "box", "size": dims}
                    elif cylinder is not None and cylinder.find("radius") is not None:
                        radius = float(cylinder.findtext("radius", "0"))
                        length = float(cylinder.findtext("length", "0"))
                        shape = {"type": "cylinder", "radius": radius, "height": length}
                    else:
                        continue
                    x = model_pose[0] + link_pose[0] + collision_pose[0]
                    y = model_pose[1] + link_pose[1] + collision_pose[1]
                    z = model_pose[2] + link_pose[2] + collision_pose[2]
                    yaw = model_pose[5] + link_pose[5] + collision_pose[5]
                    angle = model_pose[5] + link_pose[5] + collision_pose[5]
                    if shape["type"] == "box":
                        sx, sy, _sz = shape["size"]
                        ex = abs(math.cos(angle)) * sx / 2 + abs(math.sin(angle)) * sy / 2
                        ey = abs(math.sin(angle)) * sx / 2 + abs(math.cos(angle)) * sy / 2
                    else:
                        ex = ey = float(shape["radius"])
                    # A ground slab is rendered as the workbench floor, not as
                    # a wall-sized obstacle in the 2D collision/lidar model.
                    if z + float(shape.get("size", [0, 0, shape.get("height", 0)])[2]) / 2 <= 0.04:
                        continue
                    bounds_x.extend((x - ex, x + ex))
                    bounds_y.extend((y - ey, y + ey))
                    obstacles.append({
                        "name": f"{model.get('name', 'model')}/{collision.get('name', 'collision')}",
                        **shape,
                        "x": x, "y": y, "z": z, "yaw": yaw,
                        "height": float(shape.get("size", [0, 0, shape.get("height", 0)])[2]) if shape["type"] == "box" else float(shape["height"]),
                        "color": cls._visual_color(model, link, collision),
                    })
        if not obstacles:
            raise ValueError(f"SDF has no supported collision geometry: {path}")
        return cls(path.stem, str(path.relative_to(ROOT)).replace("\\", "/"),
                   max(bounds_x) - min(bounds_x), max(bounds_y) - min(bounds_y), obstacles)

    @staticmethod
    def _visual_color(model: ElementTree.Element, link: ElementTree.Element, collision: ElementTree.Element) -> str:
        visual = link.find("visual")
        diffuse = visual.findtext("material/diffuse", "") if visual is not None else ""
        values = diffuse.split()
        if len(values) >= 3:
            try:
                rgb = [round(max(0, min(1, float(value))) * 255) for value in values[:3]]
                return "#" + "".join(f"{value:02x}" for value in rgb)
            except ValueError:
                pass
        colors = {"wall": "#66756b", "crate_a": "#a67543", "crate_b": "#40885c", "crate_c": "#71638f"}
        return colors.get(model.get("name", ""), "#68736c")

    @staticmethod
    def _local_xy(x: float, y: float, obstacle: dict[str, object]) -> tuple[float, float]:
        dx, dy = x - float(obstacle["x"]), y - float(obstacle["y"])
        yaw = float(obstacle["yaw"])
        return math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy

    @staticmethod
    def _ray_box(x: float, y: float, dx: float, dy: float, obstacle: dict[str, object]) -> float | None:
        lx, ly = Scene._local_xy(x, y, obstacle)
        yaw = float(obstacle["yaw"])
        ldx = math.cos(yaw) * dx + math.sin(yaw) * dy
        ldy = -math.sin(yaw) * dx + math.cos(yaw) * dy
        sx, sy, _ = obstacle["size"]
        low, high = -math.inf, math.inf
        for origin, direction, half in ((lx, ldx, sx / 2), (ly, ldy, sy / 2)):
            if abs(direction) < 1e-12:
                if abs(origin) > half:
                    return None
                continue
            first, second = (-half - origin) / direction, (half - origin) / direction
            low, high = max(low, min(first, second)), min(high, max(first, second))
            if low > high:
                return None
        hit = low if low > 1e-6 else high
        return hit if hit > 1e-6 else None

    @staticmethod
    def _ray_cylinder(x: float, y: float, dx: float, dy: float, obstacle: dict[str, object]) -> float | None:
        ox, oy = x - float(obstacle["x"]), y - float(obstacle["y"])
        radius = float(obstacle["radius"])
        b = ox * dx + oy * dy
        c = ox * ox + oy * oy - radius * radius
        discriminant = b * b - c
        if discriminant < 0:
            return None
        hit = -b - math.sqrt(discriminant)
        if hit <= 1e-6:
            hit = -b + math.sqrt(discriminant)
        return hit if hit > 1e-6 else None

    def distance(self, pose: tuple[float, float, float], angle: float) -> float:
        x, y, yaw = pose
        direction = yaw + angle
        dx, dy = math.cos(direction), math.sin(direction)
        hits: list[float] = []
        for obstacle in self.obstacles:
            height = float(obstacle["height"])
            if not (float(obstacle["z"]) - height / 2 <= 0.126 <= float(obstacle["z"]) + height / 2):
                continue
            hit = (self._ray_box(x - 0.0227 * math.cos(yaw), y - 0.0227 * math.sin(yaw), dx, dy, obstacle)
                   if obstacle["type"] == "box" else
                   self._ray_cylinder(x - 0.0227 * math.cos(yaw), y - 0.0227 * math.sin(yaw), dx, dy, obstacle))
            if hit is not None:
                hits.append(hit)
        return max(0.05, min(8.0, min(hits, default=8.0)))

    def scan(self, pose: tuple[float, float, float], count: int = 360) -> list[float]:
        return [self.distance(pose, -math.pi + i * (2 * math.pi / count)) for i in range(count)]

    def as_json(self) -> dict[str, object]:
        return {"name": self.name, "source": self.source, "width": self.width, "height": self.height,
                "obstacles": self.obstacles, "backend": "kinematic-shadow", "approximations": [
                    "平面差速运动学；不含 Gazebo 接触、质量、摩擦或动力学响应",
                    "SDF box/cylinder 共享给渲染、二维雷达和保守圆形底盘碰撞",
                ]}

    def collides(self, x: float, y: float, *, radius: float | None = None) -> bool:
        radius = self.robot_radius if radius is None else radius
        for obstacle in self.obstacles:
            lx, ly = self._local_xy(x, y, obstacle)
            if obstacle["type"] == "box":
                sx, sy, _ = obstacle["size"]
                qx, qy = max(abs(lx) - sx / 2, 0), max(abs(ly) - sy / 2, 0)
                if qx * qx + qy * qy <= radius * radius:
                    return True
            elif math.hypot(lx, ly) <= float(obstacle["radius"]) + radius:
                return True
        return False


class LocalSimulator:
    def __init__(self, daemon: RobotDaemon, scene: Scene, *, hz: float = 50.0) -> None:
        self.daemon = daemon
        from .control import ControlSettings
        self.control_lock = getattr(daemon, 'control_lock', RLock())
        self.controls = ControlSettings(self)
        self.scene = scene
        self.hz = max(10.0, float(hz))
        self.pose = (0.0, 0.0, 0.0)
        self.traveled_m = 0.0
        # "Known cells" here is *seen* truth cells, not a SLAM result (contract
        # §4). The strategy facts carry the provenance so nobody mistakes it for
        # /map; the denominator is the free-cell count of the same truth grid.
        self.grid: tuple[int, int, float, float, float] | None = None
        self.seen_cells: set[tuple[int, int]] = set()
        self.free_cells = 0
        self.truth: dict[str, object] = {}
        # Occupancy for map-based strategies (tour): -1 until a ray has seen the
        # cell, then the truth value. Cached until the seen set grows.
        self.truth_cells: list[int] = []
        self._gridmap: object | None = None
        self._gridmap_seen = -1
        # Ages are measured, not nominal: each cycle records when the frame it
        # used actually arrived, and every payload derives age from that stamp.
        self.last_scan_at: float | None = None
        self.last_odom_at: float | None = None
        # What the kinematic loop actually applied this cycle. The strategy tape
        # records what was handed over; this is the executor's real output, so the
        # two can be compared instead of assumed equal.
        self.last_applied: tuple[float, float] = (0.0, 0.0)
        self._world_hash: tuple[str, str] | None = None
        self._tick = 0

    @property
    def known_cells(self) -> int:
        return len(self.seen_cells)

    def known_cell_source(self) -> str:
        return "sim_truth_seen/v1"

    def as_gridmap(self) -> "GridMap | None":
        """Occupancy map for tour and point navigation.

        A cell is unknown (-1) until a lidar ray has seen it; after that it takes
        the *truth* value (0 free / 100 occupied). This is the simulation's stand-in
        for SLAM: the provenance stays ``sim_truth_seen``, never ``/map``. The
        frontier planner needs real unknown cells, which is exactly what the truth
        map alone cannot provide.
        """

        if self.grid is None or not self.truth_cells:
            return None
        width, height, resolution, origin_x, origin_y = self.grid
        # The robot is standing somewhere, so its own cell is known free. Without
        # this seed a freshly started map-based strategy sees an all-unknown grid,
        # finds no "unknown next to free" cell and reports COMPLETE immediately.
        here = (
            int((self.pose[0] - origin_x) // resolution),
            int((self.pose[1] - origin_y) // resolution),
        )
        if (
            self._gridmap is not None
            and self._gridmap_seen == (len(self.seen_cells), here)
        ):
            return self._gridmap  # type: ignore[return-value]
        data = [
            0
            if (index % width, index // width) == here
            else (
                int(self.truth_cells[index])
                if (index % width, index // width) in self.seen_cells
                else -1
            )
            for index in range(width * height)
        ]
        self._gridmap = GridMap(
            width=width,
            height=height,
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
            data=data,
        )
        self._gridmap_seen = len(self.seen_cells)
        return self._gridmap  # type: ignore[return-value]

    def world_sha256(self) -> str | None:
        """SHA-256 of the SDF this scene came from, recomputed on scene change."""

        candidates = [ROOT / self.scene.source, SIM_ASSETS / "worlds" / f"{self.scene.name}.world"]
        for path in candidates:
            if not path.is_file():
                continue
            key = str(path.resolve())
            if self._world_hash is not None and self._world_hash[0] == key:
                return self._world_hash[1]
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            self._world_hash = (key, digest)
            return digest
        self._world_hash = None
        return None

    def truth_summary(self) -> dict[str, object]:
        """Contract §4: the truth map names its own provenance and version."""

        world_sha256 = self.world_sha256()
        return {
            "availability": "ok",
            "provenance": "sdf_truth",
            "scene": self.scene.name,
            "source": self.scene.source,
            "world_sha256": world_sha256,
            "map_version": None if world_sha256 is None else f"truth/{world_sha256[:12]}",
            "known_cells": self.truth.get("known_cells"),
            "known_cells_basis": self.truth.get("basis"),
            "free_cells": self.truth.get("free_cells"),
            "occupied_cells": self.truth.get("occupied_cells"),
            "width_m": self.scene.width,
            "height_m": self.scene.height,
            "resolution_m": self.truth.get("resolution_m"),
            "note": "SDF 几何直接投影，仅用于评估与显示，不作为导航依据",
        }

    def measured_ages(self, now: float | None = None) -> tuple[float | None, float | None]:
        """``(scan_age_s, odom_age_s)`` from the last frame stamps, or None."""

        now = time.monotonic() if now is None else now
        scan_age = None if self.last_scan_at is None else round(max(0.0, now - self.last_scan_at), 3)
        odom_age = None if self.last_odom_at is None else round(max(0.0, now - self.last_odom_at), 3)
        return scan_age, odom_age

    def snapshot(self) -> dict[str, object]:
        return {
            "physical": False,
            "pose": {"x": self.pose[0], "y": self.pose[1], "yaw": self.pose[2]},
            "traveled_m": self.traveled_m,
            "state": self.daemon.state_frame(),
            "applied": [round(self.last_applied[0], 4), round(self.last_applied[1], 4)],
            "scene": self.scene.as_json(),
            "model": "sim/xuegeros_sim/urdf/xuegeros_car.urdf.xacro",
        }

    def update_world(self) -> tuple[object, list[float]]:
        ranges = self.scene.scan(self.pose)
        profile = profile_scan(ranges, -math.pi, 2 * math.pi / len(ranges),
                               sectors=self.daemon.cerebellum.sectors,
                               front_half_angle_rad=self.daemon.cerebellum.front_half_angle)
        return profile, ranges

    def mark_seen(self, ranges: list[float], angle_min: float, angle_increment: float) -> None:
        """Stamp the truth cells a lidar frame would have revealed.

        This is a *visibility* count used for acceptance facts, not a map and not
        SLAM: the source is stamped ``sim_truth_seen`` wherever it is reported.
        """

        if self.grid is None:
            return
        width, height, resolution, origin_x, origin_y = self.grid
        x, y, yaw = self.pose
        x -= 0.0227 * math.cos(yaw)
        y -= 0.0227 * math.sin(yaw)
        for index in range(0, len(ranges), SEEN_STRIDE):
            distance = ranges[index]
            if not math.isfinite(distance) or distance <= 0.0:
                continue
            angle = yaw + angle_min + index * angle_increment
            dx, dy = math.cos(angle), math.sin(angle)
            limit = min(distance, 8.0)
            travelled = 0.0
            while travelled <= limit:
                cell_x = math.floor((x + dx * travelled - origin_x) / resolution)
                cell_y = math.floor((y + dy * travelled - origin_y) / resolution)
                if 0 <= cell_x < width and 0 <= cell_y < height:
                    self.seen_cells.add((int(cell_x), int(cell_y)))
                travelled += SEEN_STEP_M

    def select_scene(self, scene: Scene) -> None:
        self.scene = scene
        self.daemon.scene_name = scene.name
        width = height = 120
        resolution = 0.05
        origin = (-3.0, -3.0)
        cells = [100 if scene.collides(origin[0] + (x + 0.5) * resolution,
                                       origin[1] + (y + 0.5) * resolution, radius=0.0) else 0
                 for y in range(height) for x in range(width)]
        self.daemon.set_map(
            width=width,
            height=height,
            resolution=resolution,
            origin=origin,
            data=cells,
        )
        self.grid = (width, height, resolution, origin[0], origin[1])
        self.truth_cells = cells
        self._gridmap = None
        self._gridmap_seen = -1
        self.seen_cells = set()
        free = sum(1 for value in cells if value == 0)
        occupied = sum(1 for value in cells if value == 100)
        self.free_cells = free
        self.truth = {
            "resolution_m": resolution,
            "free_cells": free,
            "occupied_cells": occupied,
            # The truth grid defines every cell: there is no unknown (-1) cell, so
            # "known" here is the whole 120x120 slab, not a coverage measurement.
            "known_cells": free + occupied,
            "basis": (
                f"SDF 真值栅格 {width}x{height} @ {resolution} m，全部有定义（无未知格）；"
                "known_cells = 非未知格数 = 自由格 + 占据格，分母 14400。"
                "这是评估口径，不是 SLAM 建图结果。"
            ),
        }
        self.last_scan_at = None
        self.last_odom_at = None
        self._world_hash = None

    def step(self, now: float, dt: float) -> None:
        with self.control_lock:
            self._step(now, dt)

    def _step(self, now: float, dt: float) -> None:
        """One production cycle, also usable with a deterministic test clock."""
        dt = min(0.1, max(0.0, dt))
        tick_with_strategy = getattr(self.daemon, "tick_with_strategy", None)
        # The age of the frame the executor is about to use is the time since
        # the previous frame arrived -- one measured stamp, no nominal number.
        scan_age, odom_age = self.measured_ages(now)
        profile, ranges = self.update_world()
        increment = 2 * math.pi / len(ranges)
        self.daemon.set_scan(-math.pi, increment, ranges, age_s=scan_age)
        self.last_scan_at = now
        strategy = getattr(self.daemon, "strategy", None)
        if strategy is not None and strategy.active and (
            self._tick % 4 == 0 or (strategy.strategy == "navigate" and strategy.phase == StrategyPhase.STARTING)
        ):
            # 12.5 Hz is enough for a coverage metric and cheap enough that
            # the 50 Hz control loop keeps its budget. A new destination needs
            # its first scan before its first planner tick, even in a fresh room.
            self.mark_seen(ranges, -math.pi, increment)
        self._tick += 1
        if tick_with_strategy is not None:
            decision = tick_with_strategy(
                now=now,
                profile=profile,
                pose=self.pose,
                ranges=ranges,
                angle_min=-math.pi,
                angle_increment=increment,
                scan_age_s=scan_age,
                odom_age_s=odom_age,
                known_cells=self.known_cells,
                known_cells_total=self.truth.get("known_cells"),
                known_cells_source=self.known_cell_source(),
                traveled_m=self.traveled_m,
                grid=self.as_gridmap(),
            )
        else:
            decision = self.daemon.tick(
                now=now,
                profile=profile,
                pose=self.pose,
                scan_age_s=scan_age,
                odom_age_s=odom_age,
            )
        # Integrate only the decision the executor produced. The simulator
        # never accepts a raw velocity command from a network client.
        linear = float(decision.linear_mps) if decision.publish else 0.0
        angular = float(decision.angular_rps) if decision.publish else 0.0
        self.last_applied = (round(linear, 4), round(angular, 4))
        x, y, yaw = self.pose
        next_x = x + linear * math.cos(yaw) * dt
        next_y = y + linear * math.sin(yaw) * dt
        if not self.scene.collides(next_x, next_y):
            self.traveled_m += math.hypot(next_x - x, next_y - y)
            x, y = next_x, next_y
        yaw = (yaw + angular * dt + math.pi) % (2 * math.pi) - math.pi
        self.pose = (x, y, yaw)
        self.last_odom_at = now

    def run(self) -> None:
        period = 1.0 / self.hz
        previous = time.monotonic()
        while True:
            now = time.monotonic()
            dt = now - previous
            previous = now
            self.step(now, dt)
            delay = period - (time.monotonic() - now)
            if delay > 0:
                time.sleep(delay)


class StrategyDaemon(RobotDaemon):
    """robotd for the local sim: owns the strategy lease and the execution bridge.

    ``RobotDaemon`` already refuses a single-step goal while a tour holds the
    lease; this subclass gives that answer the contract §1 name (``busy``), adds
    the ``pause`` / ``resume`` / ``cancel`` / ``handover`` lifecycle, and turns
    the strategy's handed-over proposal into the one decision the kinematic loop
    integrates. The strategy itself never publishes: this object does.
    """

    def __init__(self, cerebellum: Cerebellum, *, physical: bool = False,
                 strategy: StrategyRunner | None = None) -> None:
        super().__init__(cerebellum, physical=physical, tour=strategy)
        self.control_lock = RLock()
        self.strategy = strategy
        self._strategy_cursor = 0

    # -- lease --------------------------------------------------------------
    def _tour_summary(self):
        if self.strategy is None:
            return None
        status = self.strategy.status()
        backend = status.get("backend") or {}
        distance = (backend.get("return") or {}).get("dist_to_start_m")
        if distance is None:
            distance = backend.get("dist_to_home_m")
        return {"request_id": status.get("request_id"), "phase": status["phase"],
                "active": status["busy"], "elapsed_s": status.get("elapsed_s"),
                "dist_to_start_m": distance, "reason": status["reason"],
                "strategy": status["strategy"], "backend_phase": backend.get("phase"),
                "scene": getattr(self, "scene_name", None),
                "known_cells": status["facts"].get("known_cells"),
                "traveled_m": status["facts"].get("traveled_m"),
                "recovery": (backend.get("navigation") or {}).get("recovery"),
                "acceptance": status.get("acceptance_result"), "physical": False}

    @property
    def strategy_busy(self) -> bool:
        return self.strategy is not None and self.strategy.holds_goal_lease()

    def command(self, message):  # type: ignore[override]
        with self.control_lock:
            return self._command(message)

    def _command(self, message):
        kind = str(message.get("t") or message.get("type") or "").lower()
        if (kind in ("stop", "halt") and self._tour_active and self.strategy is not None
                and self.strategy.strategy == "navigate"):
            reason = str(message.get("reason") or "console_stop")[:120]
            self.strategy.stop(reason=reason)
            self.stop_motion(reason)
            return {"t": "answer", "request": "stop", "ok": True, "accepted": True, "state": "DONE", "reason": reason}
        if kind in ("goal", "propose", "execute") and self.strategy_busy:
            status = self.strategy.status()
            variant = status.get("variant") or {}
            return {
                "t": "answer",
                "request": kind,
                "dispatched": False,
                "accepted": False,
                "ok": False,
                "busy": True,
                "error": "busy",
                "reason": "strategy_active",
                "strategy": {
                    "phase": status["phase"],
                    "request_id": status.get("request_id"),
                    "variant_id": variant.get("variant_id"),
                },
            }
        return super().command(message)

    def _tour_command(self, message):  # type: ignore[override]
        action = str(message.get("action") or "status").lower()
        strategy = self.strategy
        if strategy is None:
            return {"t": "answer", "request": "tour", "accepted": False, "error": "strategy_unavailable"}
        source = str(message.get("source") or "console")[:32]
        reason = str(message.get("reason") or "")[:120]
        since = int(message.get("since") or 0)
        if action == "start":
            if self.physical and str(message.get("strategy") or "").strip().lower() == "navigate":
                return {"t": "answer", "request": "tour", "accepted": False, "error": "simulation_only"}
            answer = strategy.start(
                request_id=message.get("request_id"),
                source=source,
                variant_id=message.get("variant_id"),
                strategy=message.get("strategy"),
                target=message.get("target"),
            )
            if not answer.get("accepted"):
                return {"t": "answer", "request": "tour", "action": "start", **answer}
            # A new task owns the lease: a single-step goal still in flight must
            # not resume behind it (contract §1).
            self.cerebellum.release("strategy")
            self._suppress_state_event = True
            with self._lock:
                self._proposal = None
            self._tour_active = True
            status = strategy.status()
            self._tour_request_id = status.get("request_id")
            self.emit(
                "navigation_started" if status["strategy"] == "navigate" else "tour_started",
                request_id=self._tour_request_id,
                source=source,
                variant_id=(status.get("variant") or {}).get("variant_id"),
            )
            return {"t": "answer", "request": "tour", "action": "start", "accepted": True, "strategy": status}
        if action == "pause":
            answer = strategy.pause(reason=reason or "pause_requested")
            if answer.get("accepted"):
                self.stop_motion("strategy_paused", finish_tour=False)
            return {"t": "answer", "request": "tour", "action": "pause", **answer}
        if action == "resume":
            answer = strategy.resume(reason=reason or "resume_requested")
            return {"t": "answer", "request": "tour", "action": "resume", **answer}
        if action == "cancel":
            answer = strategy.cancel(reason=reason or "cancelled")
            if answer.get("accepted"):
                self.stop_motion("strategy_cancelled")
            return {"t": "answer", "request": "tour", "action": "cancel", **answer}
        if action in ("stop", "halt"):
            answer = strategy.stop(reason=reason or "stopped")
            if answer.get("accepted"):
                self.stop_motion("strategy_stopped")
            return {"t": "answer", "request": "tour", "action": "stop", **answer}
        if action == "handover":
            answer = strategy.handover(reason=reason or "handover_requested")
            if answer.get("accepted"):
                self.stop_motion("strategy_handover")
            return {"t": "answer", "request": "tour", "action": "handover", **answer}
        return {
            "t": "answer",
            "request": "tour",
            "action": "status",
            "accepted": True,
            "strategy": strategy.status(),
            "events": strategy.events_since(since),
        }

    def stop_motion(self, reason: str, *, finish_tour: bool = True) -> None:
        """Zero the executor and drop the lease: a stop is a receipt, not a hope."""

        self.cerebellum.cancel("stop")
        was_touring = self._tour_active
        if finish_tour:
            self._tour_active = False
        with self._lock:
            self._proposal = None
            self._last_result = {"request": "stop", "accepted": True, "reason": reason}
        self.emit("stop", reason=reason)
        if was_touring and finish_tour:
            kind = "navigation_done" if self.strategy is not None and self.strategy.strategy == "navigate" else "tour_done"
            self.emit(kind, reason=reason, request_id=self._tour_request_id)

    def _sync_tour(self) -> None:
        strategy = self.strategy
        if strategy is None:
            return
        for event in strategy.events_since(self._strategy_cursor):
            self._strategy_cursor = event["seq"]
            self.emit(
                "strategy",
                phase=event["to"],
                from_phase=event["from"],
                reason=event["reason"],
                cursor=event["seq"],
                at=event["at"],
            )
        if not self._tour_active or strategy.holds_goal_lease():
            return
        self._tour_active = False
        status = strategy.status()
        backend = status.get("backend") or {}
        self.emit(
            "navigation_done" if status["strategy"] == "navigate" else "tour_done",
            reason={"DONE": status["reason"], "CANCELLED": "cancelled", "TIMEOUT": "timeout",
                    "HANDOVER": "handover"}.get(status["phase"], "stopped"),
            request_id=status.get("request_id"),
            phase=status["phase"],
            elapsed_s=status.get("elapsed_s"),
            acceptance=status.get("acceptance_result"),
            dist_to_start_m=(backend.get("return") or {}).get("dist_to_start_m"),
            encoder_path_m=backend.get("encoder_path_m"),
            known_delta_start=(backend.get("map") or {}).get("known_delta_start"),
        )

    # -- execution bridge ---------------------------------------------------
    def _executor_refusal(self, scan_age_s: float | None, odom_age_s: float | None) -> str | None:
        if scan_age_s is None:
            return "scan_missing"
        if float(scan_age_s) > self.cerebellum.scan_timeout:
            return "scan_stale"
        if odom_age_s is None:
            return "odom_missing"
        if float(odom_age_s) > self.cerebellum.odom_timeout:
            return "odom_stale"
        return None

    def _record_strategy_decision(self, state: str, reason: str, linear: float, angular: float) -> None:
        with self._lock:
            self._decision = {
                "state": state,
                "reason": reason,
                "goal": "strategy",
                "publish": bool(linear or angular),
                "would_execute": {"linear": linear, "angular": angular},
                "strategy_phase": self.strategy.phase if self.strategy is not None else None,
            }

    def tick_with_strategy(
        self,
        *,
        now: float,
        profile,
        pose,
        ranges,
        angle_min: float,
        angle_increment: float,
        scan_age_s: float | None,
        odom_age_s: float | None,
        known_cells: int | None = None,
        known_cells_total: int | None = None,
        known_cells_source: str | None = None,
        traveled_m: float | None = None,
        grid: object | None = None,
    ) -> CerebellumDecision:
        decision = self.tick(
            now=now,
            profile=profile,
            pose=pose,
            scan_age_s=scan_age_s,
            odom_age_s=odom_age_s,
        )
        strategy = self.strategy
        if strategy is None or not strategy.holds_goal_lease():
            return decision

        proposal = strategy.tick(
            now=now,
            pose=pose,
            ranges=ranges,
            angle_min=angle_min,
            angle_increment=angle_increment,
            scan_age_s=scan_age_s,
            odom_age_s=odom_age_s,
            known_cells=known_cells,
            known_cells_total=known_cells_total,
            known_cells_source=known_cells_source,
            traveled_m=traveled_m,
            grid=grid,
        )
        phase = str(proposal["phase"])
        reason = str(proposal["reason"])
        if not proposal["publish"]:
            self._record_strategy_decision("IDLE", reason, 0.0, 0.0)
            return CerebellumDecision(
                linear_mps=0.0,
                angular_rps=0.0,
                state=phase,
                goal="strategy",
                goal_id=None,
                reason=reason,
                publish=False,
                detail={"strategy_phase": phase, "rationale": proposal.get("rationale")},
            )
        refusal = self._executor_refusal(scan_age_s, odom_age_s)
        if refusal is not None:
            strategy.note_refusal(refusal, now=now)
            self._record_strategy_decision("BLOCKED", refusal, 0.0, 0.0)
            return CerebellumDecision(
                linear_mps=0.0,
                angular_rps=0.0,
                state="BLOCKED",
                goal="strategy",
                goal_id=None,
                reason=refusal,
                publish=False,
                detail={"strategy_phase": phase, "refusal": refusal},
            )
        # Bounds stay the executor's: the strategy proposes, robotd decides.
        linear = max(-self.cerebellum.max_linear, min(self.cerebellum.max_linear, float(proposal["linear_mps"])))
        angular = max(-self.cerebellum.max_angular, min(self.cerebellum.max_angular, float(proposal["angular_rps"])))
        self._record_strategy_decision("ACTIVE", reason, linear, angular)
        return CerebellumDecision(
            linear_mps=linear,
            angular_rps=angular,
            state="ACTIVE",
            goal="strategy",
            goal_id=None,
            reason=reason,
            publish=True,
            detail={
                "strategy_phase": phase,
                "rationale": proposal.get("rationale"),
                "source": "strategy",
            },
        )


def variant_catalog() -> list[dict[str, object]]:
    """Every published variant, or nothing if the directory is missing."""

    catalog: list[dict[str, object]] = []
    for variant_id in available_variants():
        try:
            catalog.append(load_variant(variant_id).as_json())
        except VariantError:
            continue
    return catalog


def strategy_payload(sim: "LocalSimulator", *, since: int = 0) -> dict[str, object]:
    """GET /api/strategy: the fixed shape the workbench consumes (contract §6)."""

    daemon = sim.daemon
    strategy = getattr(daemon, "strategy", None)
    if strategy is None:
        return {
            "state": "IDLE",
            "reason": "strategy_unavailable",
            "strategy": None,
            "phase": "IDLE",
            "busy": False,
            "facts": {},
            "events_cursor": 0,
            "physical": False,
            "available": False,
            "variants": variant_catalog(),
            "scene": sim.scene.name,
        }
    status = strategy.status()
    return {
        # fixed shape
        "state": status["state"],
        "reason": status["reason"],
        "strategy": status["strategy"],
        "phase": status["phase"],
        "busy": status["busy"],
        "facts": status["facts"],
        "events_cursor": status["events_cursor"],
        # additive detail (contract §2 record + replay + workbench helpers)
        "physical": False,
        "available": True,
        "supported_strategies": list(STRATEGIES),
        "scene": sim.scene.name,
        "goal_lease": status["goal_lease"],
        "request_id": status["request_id"],
        "variant": status["variant"],
        "coverage": status["coverage"],
        "backend": status.get("backend"),
        "proposal": status["proposal"],
        "acceptance_result": status["acceptance_result"],
        "elapsed_s": status["elapsed_s"],
        "events": strategy.events_since(since),
        "variants": variant_catalog(),
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="HTTP and robotd IPv4 bind address; 0.0.0.0 enables LAN access")
    parser.add_argument("--port", type=int, default=18242)
    parser.add_argument("--http-port", type=int, default=18142)
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--config", type=Path, default=Path.home() / '.xuegeros' / 'simulation.json')
    parser.add_argument("--mesh-dir", type=Path, default=None)
    parser.add_argument("--token", default="", help="optional local-only robotd token")
    parser.add_argument("--world", default="room.world", choices=("room.world", "room_open.world"),
                        help="Gazebo SDF source used by the shadow scene")
    parser.add_argument("--variant", default=DEFAULT_VARIANT_ID,
                        help="coverage variant id (scene_family/variant/序号) used by POST /api/strategy/start")
    return parser


def model_metadata(mesh_dir: Path | None = None) -> dict[str, object]:
    xacro = SIM_ASSETS / "urdf" / "xuegeros_car.urdf.xacro"
    source = xacro.read_text(encoding="utf-8")
    meshes = re.findall(r'<xacro:real_mesh\s+file="([^"]+)"\s+material="([^"]+)"\s*/>', source)
    available = bool(meshes) and all(((mesh_dir or SIM_ASSETS / "meshes") / name).is_file() for name, _material in meshes)
    return {
        "source": "sim/xuegeros_sim/urdf/xuegeros_car.urdf.xacro",
        "dimensions_m": {"length": 0.145, "width": 0.095, "height": 0.125},
        "mesh_available": available,
        "meshes": [{"name": name, "material": material, "url": f"/assets/{name}"} for name, material in meshes] if available else [],
        "visual": "stl" if available else "procedural_fallback",
    }


def sensors_payload(sim: "LocalSimulator") -> dict[str, object]:
    """Per docs/SIM-CONTRACT.md §3.

    Anything this backend cannot actually supply is reported ``unavailable`` with a
    reason. Nothing is inferred from a mesh: ``camera.stl`` and ``ultrasonic.stl`` are
    visuals on ``base_link`` with no link, topic or plugin behind them.
    """
    state = sim.daemon.state_frame()
    # Ages come from the loop's own frame stamps, not from a nominal constant:
    # a frozen loop must show a growing age here (and go stale), not 0.05 forever.
    scan_age, odom_age = sim.measured_ages()
    reported = {
        "scan": (state.get("scan") or {}).get("age_s"),
        "odom": state.get("odom_age_s"),
    }

    def availability(age: object, limit: float) -> tuple[str, str | None]:
        if age is None:
            return "missing", "数据龄未知：循环还没有产出任何一帧"
        try:
            value = float(age)
        except (TypeError, ValueError):
            return "missing", "数据龄无法解析"
        if value > limit:
            return "stale", f"数据龄 {value:.2f}s 超过 {limit:.1f}s"
        return "ok", None

    lidar_availability, lidar_reason = availability(scan_age, 1.5)
    odom_availability, odom_reason = availability(odom_age, 1.0)
    return {
        "generated_at": time.time(),
        "physical": False,
        "age_source": "loop_frame_stamp",
        "sensors": [
            {
                "id": "lidar",
                "label": "雷达（合成射线）",
                "frame": "laser_frame",
                "topic": "/scan",
                "msg_type": "sensor_msgs/LaserScan",
                "rate_hz": 2.5,
                "mount": {"parent": "base_link", "xyz_m": [-0.0227, 0.000062, 0.0637]},
                "source": "sim",
                "substitute": "sim_sdf_raycast",
                "age_s": scan_age,
                "age_reported_s": reported["scan"],
                "availability": lidar_availability,
                "reason": lidar_reason,
                "controls": True,
            },
            {
                "id": "wheel_odom",
                "label": "轮式里程计（运动学积分）",
                "frame": "base_footprint",
                "topic": "/odom",
                "msg_type": "nav_msgs/Odometry",
                "rate_hz": sim.hz,
                "mount": {"parent": "world", "xyz_m": [0.0, 0.0, 0.0]},
                "source": "sim",
                "substitute": "kinematic_integration",
                "age_s": odom_age,
                "age_reported_s": reported["odom"],
                "availability": odom_availability,
                "reason": odom_reason,
                "controls": True,
            },
            {
                "id": "imu",
                "label": "IMU",
                "frame": "imu_link",
                "topic": "/imu",
                "msg_type": "sensor_msgs/Imu",
                "rate_hz": None,
                "mount": {"parent": "base_link", "xyz_m": [0.0, 0.0, 0.03]},
                "source": "unavailable",
                "substitute": None,
                "age_s": None,
                "availability": "unavailable",
                "reason": "URDF 有 imu_link，但本地运动学后端不产生 /imu；仿真里没有横滚/俯仰信息",
                "controls": False,
            },
            {
                "id": "camera",
                "label": "前视相机",
                "frame": None,
                "topic": None,
                "msg_type": None,
                "rate_hz": None,
                "mount": {"parent": "base_link", "derived": "camera.stl 包围盒 x≈+30…+61 mm"},
                "source": "unavailable",
                "substitute": None,
                "age_s": None,
                "availability": "unavailable",
                "reason": "URDF 没有 camera_link，仿真不渲染图像；真车 MJPEG 是画面流，不是可用于控制的 ROS 图像源",
                "controls": False,
            },
            {
                "id": "ultrasonic",
                "label": "前向避障模块",
                "frame": None,
                "topic": None,
                "msg_type": None,
                "rate_hz": None,
                "mount": {"parent": "base_link", "derived": "ultrasonic.stl 包围盒 x≈+34…+47 mm"},
                "source": "unavailable",
                "substitute": None,
                "age_s": None,
                "availability": "unavailable",
                "reason": "URDF 没有 ultrasonic link/topic，仿真里只有 visual；避障由雷达净空判定替代",
                "controls": False,
            },
        ],
    }


def maps_payload(sim: "LocalSimulator") -> dict[str, object]:
    """Per docs/SIM-CONTRACT.md §4: truth / slam / vehicle never share one field."""
    return {
        "generated_at": time.time(),
        "physical": False,
        "maps": {
            "truth": sim.truth_summary(),
            "slam": {
                "availability": "unavailable",
                "provenance": "none",
                "map_version": None,
                "known_cells": None,
                "resolution_m": None,
                "updated_at": None,
                "reason": "本机影子后端不做 SLAM：/scan 是合成射线，没有 map→odom 建图链",
            },
            "vehicle": {
                "availability": "unavailable",
                "provenance": "none",
                "map_version": None,
                "known_cells": None,
                "updated_at": None,
                "reason": "真车 /map 在 guest 的 domain 0 上，本机影子后端不接触真车",
            },
        },
    }


class SimHTTPServer(ThreadingHTTPServer):
    """Threaded loopback server with a backlog large enough for a polling GUI.

    The workbench polls ``/api/state`` every 2 s and captures screenshots on top
    of that, so several connections can arrive while the accept loop is busy. The
    stdlib default backlog is 5; a burst can then be refused (the observed
    ``ERR_CONNECTION_REFUSED``). Raising the backlog fixes the cause instead of
    merging endpoints.
    """

    daemon_threads = True
    allow_reuse_address = True
    request_queue_size = 64


def start_http(sim: LocalSimulator, port: int, model: dict[str, object], *, mesh_dir: Path | None = None, host: str = '127.0.0.1') -> ThreadingHTTPServer:
    from .observation import WorkbenchFrames
    frames = WorkbenchFrames()

    class Handler(BaseHTTPRequestHandler):
        def _json(self, payload: object, status: int = 200) -> None:
            body = json.dumps(payload, separators=(",", ":")).encode()
            self.send_response(status)
            self.send_header("content-type", "application/json; charset=utf-8")
            self.send_header("content-length", str(len(body)))
            self.send_header("cache-control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:
            path, _, query = self.path.partition("?")
            if path == '/api/control':
                self._json(sim.controls.snapshot())
                return
            if path == '/api/interfaces':
                hostname = urlsplit('http://' + self.headers.get('Host', '127.0.0.1')).hostname or '127.0.0.1'
                self._json({"http": "/", "robotd": f"ws://{hostname}:{getattr(sim, 'robotd_port', 18242)}/ws",
                            "get": ["/api/state", "/api/control", "/api/interfaces", "/api/model", "/api/sensors", "/api/maps", "/api/scenes", "/api/strategy", "/api/strategy/trace", "/api/map.png", "/api/workbench/latest", "/api/workbench/frames/<seq>.png"],
                            "post": ["/api/control", "/api/command", "/api/scene", "/api/sim/pose", "/api/workbench/frame"] + [f"/api/strategy/{a}" for a in STRATEGY_ACTIONS]})
                return
            if path == '/api/workbench/latest':
                self._json(frames.latest())
                return
            if path.startswith('/api/workbench/frames/'):
                try:
                    saved = frames.image(int(Path(path).stem))
                except ValueError:
                    saved = None
                if not saved:
                    self.send_error(404)
                    return
                body, _ = saved
                self.send_response(200)
                self.send_header('content-type', 'image/png')
                self.send_header('content-length', str(len(body)))
                self.send_header('cache-control', 'no-store')
                self.end_headers()
                self.wfile.write(body)
                return
            if path == '/api/map.png':
                from io import BytesIO
                try:
                    from .probe.capture import render_truth_map
                except SystemExit:
                    self._json({"ok": False, "error": "pillow_required", "install": "python -m pip install Pillow"}, 503)
                    return
                image = BytesIO()
                meta = dict(maps_payload(sim)['maps']['truth'], captured_at=time.time())
                x, y, yaw = sim.pose
                render_truth_map(sim.scene.as_json(), dict(x=x, y=y, yaw=yaw), meta, image)
                body = image.getvalue()
                self.send_response(200)
                self.send_header('content-type', 'image/png')
                self.send_header('content-length', str(len(body)))
                self.send_header('cache-control', 'no-store')
                self.end_headers()
                self.wfile.write(body)
                return
            if path == "/api/strategy":
                since = 0
                for item in query.split("&"):
                    key, _, value = item.partition("=")
                    if key == "since":
                        try:
                            since = int(value)
                        except ValueError:
                            since = 0
                self._json(strategy_payload(sim, since=since))
                return
            if path == "/api/state":
                body = json.dumps(sim.snapshot(), separators=(",", ":")).encode()
                content_type = "application/json; charset=utf-8"
            elif path == "/api/strategy/trace":
                # Diagnosis tape (contract §6): aim, proposal and handed-over output
                # per control cycle, so an oscillation can be attributed by evidence.
                strategy = getattr(sim.daemon, "strategy", None)
                limit = 256
                if "n=" in self.path:
                    try:
                        limit = max(1, min(4096, int(self.path.split("n=", 1)[1].split("&", 1)[0])))
                    except ValueError:
                        limit = 256
                trace = [] if strategy is None else strategy.trace(limit)
                body = json.dumps(
                    {"physical": False, "count": len(trace), "trace": trace},
                    separators=(",", ":"),
                ).encode()
                content_type = "application/json; charset=utf-8"
            elif path == "/api/model":
                body = json.dumps(model, separators=(",", ":")).encode()
                content_type = "application/json; charset=utf-8"
            elif path == "/api/sensors":
                body = json.dumps(sensors_payload(sim), separators=(",", ":")).encode()
                content_type = "application/json; charset=utf-8"
            elif path == "/api/maps":
                body = json.dumps(maps_payload(sim), separators=(",", ":")).encode()
                content_type = "application/json; charset=utf-8"
            elif path == "/api/scenes":
                scenes = [Scene.from_sdf(SIM_ASSETS / "worlds" / name).as_json() for name in ("room.world", "room_open.world")]
                body = json.dumps(scenes, separators=(",", ":")).encode()
                content_type = "application/json; charset=utf-8"
            elif path.startswith("/vendor/three/"):
                target = (THREE_ROOT / Path(path).name).resolve()
                if THREE_ROOT.resolve() not in target.parents or not target.is_file():
                    self.send_error(404)
                    return
                body = target.read_bytes()
                content_type = "text/javascript; charset=utf-8"
            elif path.startswith("/assets/"):
                asset_root = (mesh_dir or SIM_ASSETS / "meshes").resolve()
                target = (asset_root / Path(path).name).resolve()
                if asset_root not in target.parents or not target.is_file() or target.suffix.lower() != '.stl':
                    self.send_error(404)
                    return
                body = target.read_bytes()
                content_type = "model/stl"
            else:
                relative = "index.html" if path in ("/", "/index.html") else path.lstrip("/")
                target = (WEB_ROOT / relative).resolve()
                if WEB_ROOT.resolve() not in target.parents or not target.is_file():
                    self.send_error(404)
                    return
                body = target.read_bytes()
                content_type = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
                if content_type == "text/html":
                    content_type = "text/html; charset=utf-8"
            self.send_response(200)
            self.send_header("content-type", content_type)
            self.send_header("content-length", str(len(body)))
            self.send_header("cache-control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _body(self) -> dict[str, object]:
            length = int(self.headers.get("content-length", "0"))
            if length > 16384 or length < 0:
                raise ValueError("request_too_large")
            if length <= 0:
                return {}
            payload = json.loads(self.rfile.read(length) or b"{}")
            if not isinstance(payload, dict):
                raise ValueError("request body must be a JSON object")
            return payload

        def _strategy(self, action: str) -> None:
            daemon = sim.daemon
            if not hasattr(daemon, "strategy"):
                self._json({"ok": False, "action": action, "error": "strategy_unavailable"}, 503)
                return
            try:
                request = self._body()
            except (ValueError, TypeError, json.JSONDecodeError) as exc:
                self._json({"ok": False, "action": action, "error": "invalid_json", "message": str(exc)}, 400)
                return
            bare = [key for key in ("lane_length_m", "lane_width_m", "max_lanes") if key in request]
            if bare:
                # Same rule as CoverageController: a run is named by a variant.
                payload = strategy_payload(sim)
                payload.update({
                    "ok": False,
                    "action": action,
                    "error": "bare_geometry",
                    "message": "车道几何属于变体；请用 variant_id（见 variants）",
                    "rejected": bare,
                    "available_variants": available_variants(),
                })
                self._json(payload, 400)
                return
            answer = daemon.command(
                {
                    "t": "tour",
                    "action": action,
                    "source": "api",
                    "request_id": request.get("request_id"),
                    "variant_id": request.get("variant_id"),
                    "strategy": request.get("strategy"),
                    "target": request.get("target"),
                    "reason": request.get("reason"),
                    "since": request.get("since") or 0,
                }
            )
            error = answer.get("error")
            status = 200
            if error == "strategy_active" or error in ("not_active", "not_running", "not_paused"):
                status = 409
            elif error:
                status = 400
            payload = strategy_payload(sim, since=int(request.get("since") or 0))
            payload.update({"ok": bool(answer.get("accepted")), "action": action})
            if error:
                payload["error"] = error
                payload["message"] = answer.get("message") or error
            if "available" in answer:
                payload["available_variants"] = answer["available"]
            self._json(payload, status)

        def do_POST(self) -> None:
            path = self.path.split("?", 1)[0]
            if path in ('/api/control', '/api/command'):
                try:
                    request = self._body()
                    if path == '/api/control':
                        self._json(sim.controls.apply(request))
                    else:
                        if request.get('t') not in ('hello', 'observe', 'goal', 'stop', 'tour', 'propose', 'execute', 'priority'):
                            raise ValueError('unknown_command')
                        self._json(sim.daemon.command(request))
                except RuntimeError as exc:
                    self._json({"ok": False, "error": str(exc)}, 409)
                except (ValueError, TypeError) as exc:
                    self._json({"ok": False, "error": str(exc)}, 400)
                except OSError as exc:
                    self._json({"ok": False, "error": "settings_write_failed", "detail": str(exc)}, 500)
                return
            if path == '/api/workbench/frame':
                try:
                    length = int(self.headers.get('content-length', '0'))
                    if not 0 < length <= 2_800_000:
                        raise ValueError('image_too_large')
                    payload = json.loads(self.rfile.read(length))
                    meta = frames.publish(payload)
                    self._json(dict(ok=True, seq=meta['seq']))
                except (ValueError, TypeError, AttributeError) as exc:
                    self._json(dict(ok=False, error=str(exc)), 400)
                return
            match = re.match(r"^/api/strategy/([a-z]+)$", path)
            if match:
                action = match.group(1)
                if action not in STRATEGY_ACTIONS:
                    self._json({"ok": False, "error": "unknown_action", "allowed": list(STRATEGY_ACTIONS)}, 404)
                    return
                self._strategy(action)
                return
            if path == "/api/sim/pose":
                if getattr(sim.daemon, "strategy_busy", False):
                    self._json({"ok": False, "error": "strategy_active"}, 409)
                    return
                # Acceptance hook: place the robot at a known hard pose (e.g. beside
                # a crate) so a recovery can be reproduced instead of waited for.
                # Loopback only and physical is always false - this moves nothing real.
                try:
                    request = self._body()
                    x = float(request["x"])
                    y = float(request["y"])
                    yaw = float(request.get("yaw", 0.0))
                except (ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
                    self._json({"ok": False, "error": "invalid_pose", "message": str(exc)}, 400)
                    return
                sim.pose = (x, y, yaw)
                sim.traveled_m = 0.0
                self._json({"ok": True, "physical": False, "pose": [round(x, 3), round(y, 3), round(yaw, 3)]})
                return
            if path != "/api/scene":
                self.send_error(404)
                return
            if getattr(sim.daemon, "strategy_busy", False):
                self._json({"ok": False, "error": "strategy_active"}, 409)
                return
            try:
                length = min(int(self.headers.get("content-length", "0")), 4096)
                request = json.loads(self.rfile.read(length))
                name = request.get("name")
                if name not in ("room.world", "room_open.world"):
                    raise ValueError("unknown scene")
                sim.select_scene(Scene.from_sdf(SIM_ASSETS / "worlds" / name))
                body = json.dumps({"ok": True, "scene": sim.scene.as_json()}, separators=(",", ":")).encode()
                self.send_response(200)
                self.send_header("content-type", "application/json; charset=utf-8")
            except (ValueError, TypeError, json.JSONDecodeError) as exc:
                body = json.dumps({"error": str(exc)}).encode()
                self.send_response(400)
                self.send_header("content-type", "application/json; charset=utf-8")
            self.send_header("content-length", str(len(body)))
            self.send_header("cache-control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, _format: str, *_args: object) -> None:
            return

    server = SimHTTPServer((host, port), Handler)
    Thread(target=server.serve_forever, name="xuegeros-local-sim-http", daemon=True).start()
    return server


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not 1 <= args.hz <= 100:
        raise SystemExit("hz must be between 1 and 100")
    if args.host != 'localhost':
        try:
            ipaddress.IPv4Address(args.host)
        except ipaddress.AddressValueError:
            raise SystemExit('host must be localhost or an IPv4 bind address')
    control = Cerebellum()
    strategy = StrategyRunner(
        variant_id=args.variant,
        max_linear_mps=control.max_linear,
        max_angular_rps=control.max_angular,
        # Coverage must end a lane before the executor would refuse the advance,
        # so the two thresholds share one number instead of drifting apart.
        obstacle_stop_m=control.min_clearance,
        scan_timeout_s=control.scan_timeout,
        odom_timeout_s=control.odom_timeout,
        min_clearance_m=control.min_clearance,
        min_clearance_turn_m=control.min_clearance_turn,
    )
    daemon = StrategyDaemon(control, physical=False, strategy=strategy)
    scene = Scene.from_sdf(SIM_ASSETS / "worlds" / args.world)
    sim = LocalSimulator(daemon, scene, hz=args.hz)
    sim.robotd_port = args.port
    sim.controls.path = args.config
    sim.controls.load()
    http = start_http(sim, args.http_port, model_metadata(args.mesh_dir), mesh_dir=args.mesh_dir, host=args.host)
    sim.select_scene(scene)
    server = DaemonServer(
        daemon,
        socket_path=None,
        host=args.host,
        port=args.port,
        state_hz=args.hz,
        token=args.token,
    )
    server.start()
    print(f"xuegeros local sim: {server.url} physical=false", flush=True)
    print(f"xuegeros local sim view: http://127.0.0.1:{args.http_port}/", flush=True)
    print(
        f"xuegeros local sim strategy: variant={strategy.variant_id} "
        f"POST http://127.0.0.1:{args.http_port}/api/strategy/start",
        flush=True,
    )
    try:
        sim.run()
    except KeyboardInterrupt:
        return 0
    finally:
        server.stop()
        http.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
