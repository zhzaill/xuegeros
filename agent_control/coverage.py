"""Boustrophedon coverage: deterministic room sweep, then return home.

Pure Python, no ROS and no map dependency. The caller supplies the control
pose (yaw fused from the IMU) and the latest scan. The pattern is a classic
lawnmower: drive a fixed-length lane, turn 90 degrees, shift one lane width,
turn back, and repeat in the opposite direction.

Two layers live here, split the way ``docs/SIM-CONTRACT.md`` §1 splits them:

* ``CoverageController`` is the deterministic strategy. It turns scan geometry
  and pose into **one** ``ActionProposal`` (v, w). It never publishes a velocity,
  never sends a goal and never touches the scene.
* ``StrategyRunner`` is the long-task orchestration on top of it (contract §6):
  it owns the variant choice, the goal lease, the state machine, the event log
  and the acceptance verdict. It hands the proposal to the executor
  (``robotd`` / ``Cerebellum``) and records what happened; the executor is the
  only layer that produces motion.

Lane geometry is **not** a free parameter any more (contract §2/§8.3). A run is
identified by ``variant_id`` such as ``room/default/01``, which resolves to
``sim/contracts/variants/<scene_family>/<variant>/<序号>.json``. Passing a bare
``lane_length_m`` / ``lane_width_m`` raises ``BareGeometryRejected`` with the
list of variants that do exist, because a run nobody can name is not a run
anybody can compare.
"""

from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

from .protocol import ActionProposal

ROOT = Path(__file__).resolve().parents[1]
VARIANT_ROOT = ROOT / "sim" / "contracts" / "variants"
VARIANT_SCHEMA = "xuegeros.coverage.variant.v1"
DEFAULT_VARIANT_ID = "room/default/01"

IDLE = "IDLE"
LANE = "LANE"
TURN = "TURN"
SHIFT = "SHIFT"
RETURN = "RETURN"
DONE = "DONE"
STOPPED = "STOPPED"
LIMIT = "LIMIT"

ACTIVE_PHASES = (LANE, TURN, SHIFT, RETURN)
ZERO = ActionProposal(0.0, 0.0, 250, "coverage", "idle")


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


# -- variants (contract §2) ---------------------------------------------------
class VariantError(ValueError):
    """A run variant could not be resolved.

    ``code`` is a stable, machine-readable error code: ``invalid_variant_id``,
    ``unknown_variant``, ``invalid_variant`` or ``bare_geometry``. ``available``
    always lists the variants that *do* exist, so a caller that guessed wrong can
    correct itself instead of guessing again.
    """

    def __init__(self, code: str, message: str, *, available: Sequence[str] = ()) -> None:
        self.code = code
        self.available = list(available)
        detail = message
        if self.available:
            detail = f"{message}（可用变体：{', '.join(self.available)}）"
        super().__init__(detail)


class BareGeometryRejected(VariantError):
    """Raised when lane geometry is passed instead of a ``variant_id``."""

    def __init__(self, available: Sequence[str]) -> None:
        super().__init__(
            "bare_geometry",
            "coverage 不再接受裸的 lane_length_m/lane_width_m：车道几何属于变体配置，"
            "请用 variant_id 引用 sim/contracts/variants/ 下的变体",
            available=available,
        )


@dataclass(frozen=True)
class CoverageVariant:
    """One reproducible (scene family, lane plan, instruction, acceptance) set."""

    variant_id: str
    scene_family: str
    world: str
    world_sha256: str | None
    world_sha256_source: str
    instruction_id: str
    instruction_text: str
    seed: int | None
    start_pose: tuple[float, float, float]
    lane_length_m: float
    lane_width_m: float
    max_lanes: int
    acceptance: Mapping[str, Any]
    source: str

    def as_json(self) -> dict[str, Any]:
        return {
            "schema": VARIANT_SCHEMA,
            "variant_id": self.variant_id,
            "scene_family": self.scene_family,
            "world": self.world,
            "world_sha256": self.world_sha256,
            "world_sha256_source": self.world_sha256_source,
            "instruction_id": self.instruction_id,
            "instruction_text": self.instruction_text,
            "seed": self.seed,
            "start_pose": [round(value, 4) for value in self.start_pose],
            "lane": {
                "length_m": self.lane_length_m,
                "width_m": self.lane_width_m,
                "max_lanes": self.max_lanes,
            },
            "acceptance": dict(self.acceptance),
            "source": self.source,
        }


def _variant_root(root: str | Path | None) -> Path:
    return VARIANT_ROOT if root is None else Path(root)


def available_variants(*, root: str | Path | None = None) -> list[str]:
    """Every published variant id, derived from ``<family>/<variant>/<n>.json``."""

    base = _variant_root(root)
    if not base.is_dir():
        return []
    found: list[str] = []
    for path in sorted(base.glob("*/*/*.json")):
        relative = path.relative_to(base)
        found.append(f"{relative.parts[0]}/{relative.parts[1]}/{path.stem}")
    return found


def variant_parts(variant_id: Any, *, root: str | Path | None = None) -> tuple[str, str, str]:
    text = "" if variant_id is None else str(variant_id).strip()
    parts = [part for part in text.split("/") if part]
    unsafe = any(part in (".", "..") or "\\" in part or ":" in part for part in parts)
    if len(parts) != 3 or unsafe:
        raise VariantError(
            "invalid_variant_id",
            f"variant_id 必须是 scene_family/variant/序号 三段（收到 {text!r}）",
            available=available_variants(root=root),
        )
    return parts[0], parts[1], parts[2]


def variant_path(variant_id: Any, *, root: str | Path | None = None) -> Path:
    family, name, index = variant_parts(variant_id, root=root)
    return _variant_root(root) / family / name / f"{index}.json"


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _positive(value: Any, name: str, bad: Callable[[str], None], *, allow_zero: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        bad(f"{name} 必须是有限数字")
    number = float(value)
    if number < 0.0 or (number == 0.0 and not allow_zero):
        bad(f"{name} 必须 {'>= 0' if allow_zero else '> 0'}")
    return number


def _parse_variant(
    payload: Any,
    variant_id: str,
    path: Path,
    available: Sequence[str],
) -> CoverageVariant:
    def bad(message: str) -> None:
        raise VariantError("invalid_variant", f"变体 {variant_id} 不合法：{message}", available=available)

    if not isinstance(payload, Mapping):
        bad("顶层必须是 JSON 对象")
    if payload.get("schema") != VARIANT_SCHEMA:
        bad(f"schema 必须是 {VARIANT_SCHEMA!r}")
    if str(payload.get("variant_id")) != variant_id:
        bad(f"文件里的 variant_id={payload.get('variant_id')!r} 与路径不一致")
    family, _, _ = variant_parts(variant_id)
    scene_family = str(payload.get("scene_family") or "")
    if scene_family != family:
        bad(f"scene_family={scene_family!r} 必须与 variant_id 第一段 {family!r} 一致")
    world = str(payload.get("world") or "")
    if not world.endswith(".world"):
        bad("world 必须指向一个 .world 文件")
    lane = payload.get("lane")
    if not isinstance(lane, Mapping):
        bad("lane 必须是对象")
    length = _positive(lane.get("length_m"), "lane.length_m", bad)
    width = _positive(lane.get("width_m"), "lane.width_m", bad)
    lanes = lane.get("max_lanes")
    if isinstance(lanes, bool) or not isinstance(lanes, int) or lanes < 1:
        bad("lane.max_lanes 必须是 >= 1 的整数")
    instruction_id = str(payload.get("instruction_id") or "").strip()
    if not instruction_id:
        bad("instruction_id 不能为空")
    instruction_text = str(payload.get("instruction_text") or "").strip()
    acceptance = payload.get("acceptance")
    if not isinstance(acceptance, Mapping):
        bad("acceptance 必须是对象")
    for key in ("min_known_cells", "max_runtime_s", "return_distance_m"):
        if key not in acceptance:
            bad(f"acceptance 缺少 {key}")
        _positive(acceptance.get(key), f"acceptance.{key}", bad, allow_zero=key != "max_runtime_s")
    forbidden = acceptance.get("no_forbidden_state", [])
    if not isinstance(forbidden, (list, tuple)) or any(not isinstance(item, str) for item in forbidden):
        bad("acceptance.no_forbidden_state 必须是字符串数组")
    seed = payload.get("seed")
    if seed is not None and (isinstance(seed, bool) or not isinstance(seed, int)):
        bad("seed 必须是整数或 null")
    start_pose = payload.get("start_pose", [0.0, 0.0, 0.0])
    if not isinstance(start_pose, (list, tuple)) or len(start_pose) != 3:
        bad("start_pose 必须是 [x, y, yaw]")
    try:
        pose = tuple(float(value) for value in start_pose)
    except (TypeError, ValueError):
        bad("start_pose 必须是三个有限数字")
    if not all(math.isfinite(value) for value in pose):
        bad("start_pose 必须是三个有限数字")

    # The hash is measured here, never typed by hand: a world file that changed
    # under the same variant id would silently make two runs incomparable.
    recorded = payload.get("world_sha256")
    world_file = ROOT / world
    if world_file.is_file():
        world_sha256, source = _sha256_file(world_file), "computed"
    else:
        world_sha256 = recorded if isinstance(recorded, str) and recorded else None
        source = "unavailable" if world_sha256 is None else "recorded"
    try:
        source_path = str(path.relative_to(ROOT)).replace("\\", "/")
    except ValueError:
        source_path = str(path)
    return CoverageVariant(
        variant_id=variant_id,
        scene_family=scene_family,
        world=world,
        world_sha256=world_sha256,
        world_sha256_source=source,
        instruction_id=instruction_id,
        instruction_text=instruction_text,
        seed=None if seed is None else int(seed),
        start_pose=(pose[0], pose[1], pose[2]),
        lane_length_m=length,
        lane_width_m=width,
        max_lanes=int(lanes),
        acceptance={
            "min_known_cells": float(acceptance["min_known_cells"]),
            "max_runtime_s": float(acceptance["max_runtime_s"]),
            "return_distance_m": float(acceptance["return_distance_m"]),
            "no_forbidden_state": [str(item) for item in forbidden],
        },
        source=source_path,
    )


def load_variant(variant_id: Any, *, root: str | Path | None = None) -> CoverageVariant:
    """Resolve ``scene_family/variant/序号`` to one validated variant file."""

    base = _variant_root(root)
    available = available_variants(root=base)
    path = variant_path(variant_id, root=base)
    if not path.is_file():
        raise VariantError(
            "unknown_variant",
            f"没有变体 {variant_id}（期望文件 {path}）",
            available=available,
        )
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise VariantError(
            "invalid_variant", f"变体 {variant_id} 读不出来：{exc}", available=available
        ) from exc
    return _parse_variant(payload, str(variant_id), path, available)


class CoverageController:
    """Fixed-geometry coverage with scan-triggered lane changes.

    The lane plan comes from a variant (``variant_id``); a bare
    ``lane_length_m`` / ``lane_width_m`` is rejected. ``max_lanes`` stays an
    optional override because the long-task runner drives the variant value and
    a single test or a one-off probe may want one lane only -- but the default is
    always the variant's.
    """

    def __init__(
        self,
        *,
        variant_id: str | None = None,
        variant_root: str | Path | None = None,
        max_linear_mps: float = 0.09,
        max_angular_rps: float = 0.45,
        obstacle_stop_m: float = 0.22,
        lane_length_m: float | None = None,
        lane_width_m: float | None = None,
        max_lanes: int | None = None,
        turn_tolerance_rad: float = 0.08,
        turn_timeout_s: float = 6.0,
        home_tolerance_m: float = 0.15,
        max_runtime_s: float = 120.0,
        max_travel_m: float = 10.0,
        return_timeout_s: float = 60.0,
        no_progress_timeout_s: float = 20.0,
        min_valid_ranges: int = 12,
    ) -> None:
        if lane_length_m is not None or lane_width_m is not None:
            # A run whose lane plan is a loose number cannot be named, seeded or
            # compared with another run -- so it is not accepted at all.
            raise BareGeometryRejected(available_variants(root=variant_root))
        self.variant = load_variant(variant_id or DEFAULT_VARIANT_ID, root=variant_root)
        self.max_linear = float(max_linear_mps)
        self.max_angular = float(max_angular_rps)
        self.obstacle_stop = float(obstacle_stop_m)
        self.lane_length = self.variant.lane_length_m
        self.lane_width = self.variant.lane_width_m
        self.max_lanes = self.variant.max_lanes if max_lanes is None else max(1, int(max_lanes))
        self.turn_tolerance = float(turn_tolerance_rad)
        self.turn_timeout = float(turn_timeout_s)
        self.home_tolerance = float(home_tolerance_m)
        self.max_runtime = float(max_runtime_s)
        self.max_travel = float(max_travel_m)
        self.return_timeout = float(return_timeout_s)
        self.no_progress_timeout = float(no_progress_timeout_s)
        self.min_valid_ranges = int(min_valid_ranges)

        self.phase = IDLE
        self.request_id: str | None = None
        self.source: str | None = None
        self.reason: str | None = None

        self._started_at: float | None = None
        self._return_started: float | None = None
        self._home: tuple[float, float] | None = None
        self._axis_yaw = 0.0
        self._axis = (1.0, 0.0)
        self._normal = (0.0, 1.0)
        self._lane_index = 0
        self._lane_dir = 1
        self._lane_start: tuple[float, float] | None = None
        self._shift_start: tuple[float, float] | None = None
        self._turn_target = 0.0
        self._after_turn = ""
        self._turn_started = 0.0
        self._last_pose: tuple[float, float, float] | None = None
        self._encoder_path = 0.0
        self._yaw_path = 0.0
        self._last_progress_at = 0.0
        self._encoder_last = 0.0
        self._yaw_last = 0.0
        self._blocked_count = 0
        self._last_action = ZERO
        self._battery_v: float | None = None
        self._publishers: Mapping[str, int] = {}
        self._imu: dict[str, Any] = {"orientation_valid": False}
        self._actuator: tuple[float, float] = (0.0, 0.0)
        self._actuator_age: float | None = None
        self._encoder_twist: float | None = None

    @property
    def active(self) -> bool:
        return self.phase in ACTIVE_PHASES

    # -- lifecycle ----------------------------------------------------------
    def start(
        self,
        *,
        now: float | None = None,
        request_id: str,
        source: str,
        pose: tuple[float, float, float] | None = None,
        grid: Any = None,
        pose_map: tuple[float, float, float] | None = None,
        pose_odom: tuple[float, float, float] | None = None,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        battery_v: float | None = None,
        publishers: Mapping[str, int] | None = None,
        imu: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        pose = pose if pose is not None else pose_map
        if pose is None:
            raise ValueError("coverage needs a pose")
        now = time.monotonic() if now is None else now
        self.request_id = request_id
        self.source = source
        self.phase = LANE
        self.reason = None
        self._started_at = now
        self._return_started = None
        self._home = (pose[0], pose[1])
        self._axis_yaw = pose[2]
        self._axis = (math.cos(pose[2]), math.sin(pose[2]))
        self._normal = (-math.sin(pose[2]), math.cos(pose[2]))
        self._lane_index = 0
        self._lane_dir = 1
        self._lane_start = (pose[0], pose[1])
        self._shift_start = None
        self._last_pose = pose
        self._encoder_path = 0.0
        self._yaw_path = 0.0
        self._last_progress_at = now
        self._encoder_last = 0.0
        self._yaw_last = 0.0
        self._blocked_count = 0
        self._last_action = ZERO
        self._battery_v = battery_v
        self._publishers = dict(publishers or {})
        self._imu = dict(imu or {"orientation_valid": False})
        self._actuator = (0.0, 0.0)
        self._actuator_age = None
        self._encoder_twist = None
        return self.snapshot(now=now, pose=pose, ranges=ranges)

    def stop(self, *, reason: str = "stopped") -> dict[str, Any]:
        if self.active:
            self.phase = STOPPED
            self.reason = reason
            self._last_action = ZERO
        return self.snapshot()

    def rebase(self, *, now: float) -> None:
        """Restart the internal watchdogs after a pause or a blocked period.

        The no-progress and phase timeouts are wall-clock budgets; a task that was
        deliberately paused (or blocked on a stale scan the operator cleared) must
        not be timed out by the very clock it was not using. The lane plan itself
        is untouched: only the reference points are moved to ``now``.
        """

        self._last_progress_at = now
        self._last_pose = None
        self._encoder_last = self._encoder_path
        self._yaw_last = self._yaw_path
        if self._return_started is not None:
            self._return_started = now
        if self.phase == TURN:
            self._turn_started = now

    def observe(
        self,
        *,
        pose: tuple[float, float, float] | None = None,
        pose_odom: tuple[float, float, float] | None = None,
        battery_v: float | None = None,
        publishers: Mapping[str, int] | None = None,
        imu: Mapping[str, Any] | None = None,
        actuator: tuple[float, float] | None = None,
        actuator_age_s: float | None = None,
        encoder_twist_mps: float | None = None,
    ) -> None:
        pose = pose if pose is not None else pose_odom
        if pose is not None:
            if self._last_pose is not None:
                self._encoder_path += _distance(pose, self._last_pose)
                self._yaw_path += abs(_wrap(pose[2] - self._last_pose[2]))
            self._last_pose = pose
        if battery_v is not None:
            self._battery_v = battery_v
        if publishers is not None:
            self._publishers = dict(publishers)
        if imu is not None:
            self._imu = dict(imu)
        if actuator is not None:
            self._actuator = (float(actuator[0]), float(actuator[1]))
        if actuator_age_s is not None:
            self._actuator_age = float(actuator_age_s)
        if encoder_twist_mps is not None:
            self._encoder_twist = float(encoder_twist_mps)

    # -- decision -----------------------------------------------------------
    def step(
        self,
        *,
        now: float,
        pose: tuple[float, float, float] | None = None,
        pose_map: tuple[float, float, float] | None = None,
        grid: Any = None,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        facts_due: bool = False,
    ) -> tuple[ActionProposal, dict[str, Any]]:
        pose = pose if pose is not None else pose_map
        if pose is None:
            return ZERO, self.snapshot(now=now)
        if not self.active or self._started_at is None:
            return ZERO, self.snapshot(now=now, pose=pose, ranges=ranges)

        if now - self._started_at > self.max_runtime:
            return self._finish(LIMIT, "runtime_limit", now, pose, ranges)
        if self._encoder_path > self.max_travel:
            return self._finish(LIMIT, "travel_limit", now, pose, ranges)

        if self._encoder_path > self._encoder_last + 0.01 or self._yaw_path > self._yaw_last + 0.03:
            self._last_progress_at = now
            self._encoder_last = self._encoder_path
            self._yaw_last = self._yaw_path
        if now - self._last_progress_at > self.no_progress_timeout:
            return self._finish(LIMIT, "no_progress", now, pose, ranges)

        valid = sum(1 for value in ranges if math.isfinite(value) and value > 0)
        if valid < self.min_valid_ranges:
            self._last_action = ZERO
            return ZERO, self.snapshot(now=now, pose=pose, ranges=ranges)

        front = self._front_min(ranges, angle_min, angle_increment)

        if self.phase == LANE:
            action = self._lane_action(pose, front, now)
        elif self.phase == TURN:
            action = self._turn_action(pose, now)
        elif self.phase == SHIFT:
            action = self._shift_action(pose, front, now)
        elif self.phase == RETURN:
            action = self._return_action(pose, now)
        else:
            action = ZERO

        self._last_action = action
        if facts_due:
            pass
        return action, self.snapshot(now=now, pose=pose, ranges=ranges)

    def _finish(self, phase: str, reason: str, now: float, pose, ranges):
        self.phase = phase
        self.reason = reason
        self._last_action = ZERO
        return ZERO, self.snapshot(now=now, pose=pose, ranges=ranges)

    # -- phases -------------------------------------------------------------
    def _lane_action(self, pose, front: float, now: float) -> ActionProposal:
        assert self._lane_start is not None
        along = (
            (pose[0] - self._lane_start[0]) * self._axis[0]
            + (pose[1] - self._lane_start[1]) * self._axis[1]
        ) * self._lane_dir
        if along >= self.lane_length:
            self._begin_turn(now, self._axis_yaw + math.pi / 2, SHIFT)
            return ZERO
        if front < self.obstacle_stop:
            self._blocked_count += 1
            self._begin_turn(now, self._axis_yaw + math.pi / 2, SHIFT)
            return ZERO

        lookahead = min(self.lane_length, along + 0.5)
        target_x = self._lane_start[0] + self._axis[0] * self._lane_dir * lookahead
        target_y = self._lane_start[1] + self._axis[1] * self._lane_dir * lookahead
        return self._pursue(pose, target_x, target_y, front)

    def _turn_action(self, pose, now: float) -> ActionProposal:
        error = _wrap(self._turn_target - pose[2])
        if abs(error) <= self.turn_tolerance or now - self._turn_started > self.turn_timeout:
            if self._after_turn == SHIFT:
                self.phase = SHIFT
                self._shift_start = (pose[0], pose[1])
            elif self._after_turn == LANE:
                self.phase = LANE
                self._lane_start = (pose[0], pose[1])
            elif self._after_turn == RETURN:
                self.phase = RETURN
                self._return_started = now
            return ZERO
        angular = max(-self.max_angular, min(self.max_angular, error * 2.0))
        return ActionProposal(0.0, angular, 250, "coverage", "turn")

    def _shift_action(self, pose, front: float, now: float) -> ActionProposal:
        assert self._shift_start is not None
        if _distance(pose, self._shift_start) >= self.lane_width:
            self._lane_index += 1
            if self._lane_index >= self.max_lanes:
                self.phase = RETURN
                self._return_started = now
                return ZERO
            self._lane_dir *= -1
            target_yaw = self._axis_yaw if self._lane_dir > 0 else self._axis_yaw + math.pi
            self._begin_turn(now, target_yaw, LANE)
            return ZERO
        if front < self.obstacle_stop:
            self._blocked_count += 1
            self.phase = RETURN
            self._return_started = now
            return ZERO
        target_x = self._shift_start[0] + self._normal[0] * self.lane_width
        target_y = self._shift_start[1] + self._normal[1] * self.lane_width
        return self._pursue(pose, target_x, target_y, front)

    def _return_action(self, pose, now: float) -> ActionProposal:
        assert self._home is not None
        if _distance(pose, self._home) <= self.home_tolerance:
            self.phase = DONE
            self.reason = "returned_home"
            self._last_action = ZERO
            return ZERO
        if self._return_started is not None and now - self._return_started > self.return_timeout:
            self.phase = LIMIT
            self.reason = "return_timeout"
            self._last_action = ZERO
            return ZERO
        return self._pursue(pose, self._home[0], self._home[1], float("inf"))

    def _begin_turn(self, now: float, target_yaw: float, after: str) -> None:
        self.phase = TURN
        self._turn_started = now
        self._turn_target = _wrap(target_yaw)
        self._after_turn = after

    # -- control ------------------------------------------------------------
    def _pursue(self, pose, target_x: float, target_y: float, front: float) -> ActionProposal:
        distance = math.hypot(target_x - pose[0], target_y - pose[1])
        heading = math.atan2(target_y - pose[1], target_x - pose[0])
        error = _wrap(heading - pose[2])
        angular = max(-self.max_angular, min(self.max_angular, error * 1.5))
        linear = self.max_linear * max(0.0, math.cos(error))
        if abs(error) > 0.6:
            linear = 0.0
        if front < self.obstacle_stop:
            linear = 0.0
        if distance < 0.05:
            linear = 0.0
        return ActionProposal(linear, angular, 250, "coverage", "pursue")

    @staticmethod
    def _front_min(ranges: Sequence[float], angle_min: float, angle_increment: float) -> float:
        values = []
        for index, value in enumerate(ranges):
            if not math.isfinite(value) or value <= 0:
                continue
            angle = angle_min + index * angle_increment
            if abs(angle) <= math.radians(25):
                values.append(value)
        return min(values, default=float("inf"))

    # -- facts --------------------------------------------------------------
    def snapshot(
        self,
        *,
        now: float | None = None,
        pose: tuple[float, float, float] | None = None,
        ranges: Sequence[float] | None = None,
        grid: Any = None,
        signature: Sequence[float | None] | None = None,
    ) -> dict[str, Any]:
        now = time.monotonic() if now is None else now
        pose = pose or self._last_pose
        along = 0.0
        lateral = 0.0
        if pose is not None and self._lane_start is not None:
            dx = pose[0] - self._lane_start[0]
            dy = pose[1] - self._lane_start[1]
            along = (dx * self._axis[0] + dy * self._axis[1]) * self._lane_dir
            lateral = -dx * self._axis[1] + dy * self._axis[0]
        home_distance = None if pose is None or self._home is None else _distance(pose, self._home)
        return {
            "request_id": self.request_id,
            "source": self.source,
            "strategy": "coverage",
            "frame": "odom",
            "variant_id": self.variant.variant_id,
            "scene_family": self.variant.scene_family,
            "instruction_id": self.variant.instruction_id,
            "world": self.variant.world,
            "world_sha256": self.variant.world_sha256,
            "seed": self.variant.seed,
            "acceptance": dict(self.variant.acceptance),
            "phase": self.phase,
            "active": self.active,
            "reason": self.reason,
            "elapsed_s": None if self._started_at is None else round(now - self._started_at, 1),
            "pose": None if pose is None else [round(float(value), 3) for value in pose],
            "lane_index": self._lane_index,
            "lane_dir": self._lane_dir,
            "lane_progress_m": round(along, 3),
            "lane_lateral_m": round(lateral, 3),
            "lane_length_m": self.lane_length,
            "lane_width_m": self.lane_width,
            "blocked_count": self._blocked_count,
            "encoder_path_m": round(self._encoder_path, 3),
            "yaw_path_rad": round(self._yaw_path, 3),
            "dist_to_home_m": home_distance,
            "cmd": {
                "autonomy": [round(self._last_action.linear_mps, 4), round(self._last_action.angular_rps, 4)],
                "actuator": [round(self._actuator[0], 4), round(self._actuator[1], 4)],
                "actuator_age_s": self._actuator_age,
            },
            "publishers": dict(self._publishers),
            "imu": dict(self._imu),
            "envelope": {
                "runtime_s": None if self._started_at is None else round(now - self._started_at, 1),
                "runtime_limit_s": self.max_runtime,
                "travel_m": round(self._encoder_path, 3),
                "travel_limit_m": self.max_travel,
                "battery_v": self._battery_v,
            },
            "return": {
                "target": None if self._home is None else [round(self._home[0], 3), round(self._home[1], 3)],
                "started": self._return_started is not None,
                "elapsed_s": None
                if self._return_started is None
                else round(now - self._return_started, 1),
                "dist_to_home_m": home_distance,
                "timeout_s": self.return_timeout,
            },
        }
