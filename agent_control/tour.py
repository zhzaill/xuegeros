"""Tour controller: frontier exploration, then return to the start pose.

Pure Python, no ROS, same shape as ``cerebellum.py`` so the behaviour can be
driven by a synthetic loop in tests. The controller never claims success: it
reports the phase and the measurements behind it, and the persona decides
whether the car actually did what it said.
"""

from __future__ import annotations

import math
import time
from collections import deque
from typing import Any, Mapping, Sequence

from .explorer import FrontierExplorer, GridMap
from .protocol import ActionProposal
from .navigation import GridRouter, nearest_on_route, segment_is_free, smooth_route, waypoint

IDLE = "IDLE"
EXPLORE = "EXPLORE"
RETURN = "RETURN"
DONE = "DONE"
STOPPED = "STOPPED"
LIMIT = "LIMIT"

ACTIVE_PHASES = (EXPLORE, RETURN)
ZERO = ActionProposal(0.0, 0.0, 250, "tour", "idle")
SCAN_BINS = 36
SCAN_CHANGE_M = 0.10


def known_cells(grid: GridMap) -> int:
    return sum(1 for value in grid.data if value != -1)


def scan_signature(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    *,
    bins: int = SCAN_BINS,
) -> list[float | None]:
    """Per-bin minimum range: a cheap, orientation-preserving view of the world."""

    span = max(1e-6, angle_increment * max(1, len(ranges)))
    out: list[float | None] = [None] * bins
    for index, value in enumerate(ranges):
        if not math.isfinite(value) or value <= 0:
            continue
        angle = angle_min + index * angle_increment
        position = ((angle - angle_min) % span) / span
        bucket = min(bins - 1, max(0, int(position * bins)))
        current = out[bucket]
        out[bucket] = value if current is None else min(current, value)
    return [None if value is None else round(value, 3) for value in out]


def _signature_delta(current: Sequence[float | None], other: Sequence[float | None]) -> tuple[int, float, int]:
    changed = 0
    total = 0.0
    pairs = 0
    for left, right in zip(current, other):
        if left is None or right is None:
            continue
        delta = abs(left - right)
        total += delta
        pairs += 1
        if delta > SCAN_CHANGE_M:
            changed += 1
    return changed, (total / pairs if pairs else 0.0), pairs


def _distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def _angle_error(current: float, target: float) -> float:
    return (target - current + math.pi) % (2 * math.pi) - math.pi


class TourController:
    """Explore until saturated, then drive home. Reports facts, not verdicts."""

    def __init__(
        self,
        *,
        max_linear_mps: float = 0.04,
        max_angular_rps: float = 0.30,
        obstacle_stop_m: float = 0.32,
        target_reached_m: float = 0.22,
        replan_period_s: float = 3.0,
        max_runtime_s: float = 300.0,
        max_travel_m: float = 8.0,
        saturation_window_s: float = 20.0,
        growth_eps_cells: int = 10,
        no_frontier_samples: int = 3,
        return_timeout_s: float = 90.0,
        min_valid_ranges: int = 12,
        min_battery_v: float = 7.0,
        no_progress_timeout_s: float = 30.0,
        bootstrap_min_known: int = 200,
        stable_routes: bool = False,
    ) -> None:
        self.max_linear = float(max_linear_mps)
        self.max_angular = float(max_angular_rps)
        self.obstacle_stop = float(obstacle_stop_m)
        self.target_reached = float(target_reached_m)
        self.replan_period = float(replan_period_s)
        self.max_runtime = float(max_runtime_s)
        self.max_travel = float(max_travel_m)
        self.saturation_window = float(saturation_window_s)
        self.growth_eps = int(growth_eps_cells)
        self.no_frontier_limit = int(no_frontier_samples)
        self.return_timeout = float(return_timeout_s)
        self.min_valid_ranges = int(min_valid_ranges)
        self.min_battery = float(min_battery_v)
        self.no_progress_timeout = float(no_progress_timeout_s)
        # Below this many known cells the map cannot offer a usable frontier: the
        # only ones available sit inside the acceptance radius, so the planner
        # would answer COMPLETE and the tour would "finish" before looking around.
        # Bootstrap by sweeping in place; this is a map-sparsity floor, NOT a
        # relaxation of any safety threshold.
        self.bootstrap_min_known = max(1, int(bootstrap_min_known))
        self.stable_routes = stable_routes

        self.phase = IDLE
        self.request_id: str | None = None
        self.source: str | None = None
        self.reason: str | None = None
        self._explorer = self._new_explorer()
        self._started_at: float | None = None
        self._return_started: float | None = None
        self._start_pose_map: tuple[float, float] | None = None
        self._start_pose_odom: tuple[float, float] | None = None
        self._last_pose_map: tuple[float, float, float] | None = None
        self._last_pose_odom: tuple[float, float, float] | None = None
        self._tf_path = 0.0
        self._encoder_path = 0.0
        self._yaw_path = 0.0
        self._last_progress_at = 0.0
        self._known_last = 0
        self._encoder_last = 0.0
        self._yaw_last = 0.0
        self._known_start = 0
        self._known_history: deque[tuple[float, int]] = deque()
        self._no_frontier = 0
        self._scan_start: list[float | None] = []
        self._scan_prev: list[float | None] = []
        self._scan_prev_delta: tuple[int, float] = (0, 0.0)
        self._frontier_count: int | None = None
        self._growth_window: int | None = None
        self._last_action = ZERO
        self._battery_v: float | None = None
        self._publishers: Mapping[str, int] = {}
        self._imu: dict[str, Any] = {"orientation_valid": False}
        self._actuator: tuple[float, float] = (0.0, 0.0)
        self._actuator_age: float | None = None
        self._encoder_twist: float | None = None
        # Diagnosis fields (docs/SIM-CONTRACT.md §6): what was aimed at, and how
        # far off the nose it was, on the last decision.
        self._last_target: tuple[float, float] | None = None
        self._last_target_id: str | None = None
        self._last_bearing_error: float | None = None
        self._last_explorer_state: str | None = None
        self._last_explorer_reason: str | None = None
        self._route = []
        self._segments = []
        self._route_at = float('-inf')
        self._route_phase = None

    def _new_explorer(self) -> FrontierExplorer:
        return FrontierExplorer(
            max_linear_mps=self.max_linear,
            max_angular_rps=self.max_angular,
            obstacle_stop_m=self.obstacle_stop,
            target_reached_m=self.target_reached,
            replan_period_s=self.replan_period,
            max_runtime_s=self.max_runtime,
            max_travel_m=self.max_travel,
        )

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
        grid: GridMap,
        pose_map: tuple[float, float, float],
        pose_odom: tuple[float, float, float] | None,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        battery_v: float | None = None,
        publishers: Mapping[str, int] | None = None,
        imu: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        now = time.monotonic() if now is None else now
        self.request_id = request_id
        self.source = source
        self.phase = EXPLORE
        self.reason = None
        self._explorer = self._new_explorer()
        self._route = []
        self._segments = []
        self._route_at = float('-inf')
        self._route_phase = None
        self._started_at = now
        self._return_started = None
        self._start_pose_map = (pose_map[0], pose_map[1])
        self._start_pose_odom = None if pose_odom is None else (pose_odom[0], pose_odom[1])
        self._last_pose_map = pose_map
        self._last_pose_odom = pose_odom
        self._tf_path = 0.0
        self._encoder_path = 0.0
        self._yaw_path = 0.0
        self._last_progress_at = now
        self._known_last = known_cells(grid)
        self._encoder_last = 0.0
        self._yaw_last = 0.0
        self._known_start = self._known_last
        self._known_history = deque([(now, self._known_start)], maxlen=256)
        self._no_frontier = 0
        self._frontier_count = None
        self._growth_window = None
        self._scan_start = scan_signature(ranges, angle_min, angle_increment)
        self._scan_prev = list(self._scan_start)
        self._scan_prev_delta = (0, 0.0)
        self._last_action = ZERO
        self._battery_v = battery_v
        self._publishers = dict(publishers or {})
        self._imu = dict(imu or {"orientation_valid": False})
        self._actuator = (0.0, 0.0)
        self._actuator_age = None
        self._encoder_twist = None
        return self.snapshot(now=now, grid=grid, pose_map=pose_map)

    def stop(self, *, reason: str = "stopped") -> dict[str, Any]:
        if self.active:
            self.phase = STOPPED
            self.reason = reason
            self._last_action = ZERO
        return self.snapshot()

    # -- recovery hand-off ---------------------------------------------------
    def can_recover(self) -> bool:
        return self._explorer.can_recover()

    def force_recovery(self, *, now: float | None = None, reason: str = "gate_blocked", grid: GridMap | None = None) -> bool:
        """The executor refused the proposed motion; back out and re-plan."""

        now = time.monotonic() if now is None else float(now)
        return self._explorer.force_recovery(now, reason, grid)
    def rebase(self, *, now: float | None = None) -> None:
        """Restart the stall clocks after a pause or a blocked stretch.

        Without this a 40 s pause looks like 40 s without progress, and the
        ``no_progress`` envelope would fire the moment the task resumes. The
        same call exists on ``CoverageController`` so the orchestrator can treat
        both strategies alike.
        """

        now = time.monotonic() if now is None else float(now)
        self._last_progress_at = now
        self._encoder_last = self._encoder_path
        self._yaw_last = self._yaw_path
        # Recovery is distance driven. Restart its watchdog after an intentional pause.
        self._explorer.rebase(now)

    # -- inputs -------------------------------------------------------------
    def observe(
        self,
        *,
        pose_odom: tuple[float, float, float] | None = None,
        battery_v: float | None = None,
        publishers: Mapping[str, int] | None = None,
        imu: Mapping[str, Any] | None = None,
        actuator: tuple[float, float] | None = None,
        actuator_age_s: float | None = None,
        encoder_twist_mps: float | None = None,
    ) -> None:
        """Update the raw inputs the ROS adapter measured this cycle."""

        if pose_odom is not None:
            if self._last_pose_odom is not None:
                self._encoder_path += _distance(pose_odom, self._last_pose_odom)
                self._yaw_path += abs(_angle_error(self._last_pose_odom[2], pose_odom[2]))
            self._last_pose_odom = pose_odom
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
        grid: GridMap,
        pose_map: tuple[float, float, float],
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        facts_due: bool = False,
    ) -> tuple[ActionProposal, dict[str, Any]]:
        if not self.active or self._started_at is None:
            return ZERO, self.snapshot(now=now, grid=grid, pose_map=pose_map)

        if self._last_pose_map is not None:
            self._tf_path += _distance(pose_map, self._last_pose_map)
        self._last_pose_map = pose_map
        signature = scan_signature(ranges, angle_min, angle_increment)
        valid = sum(1 for value in ranges if math.isfinite(value) and value > 0)

        envelope_hit = None
        if now - self._started_at > self.max_runtime:
            envelope_hit = "runtime_limit"
        elif self._encoder_path > self.max_travel:
            envelope_hit = "travel_limit"
        if envelope_hit:
            self.phase = LIMIT
            self.reason = envelope_hit
            self._last_action = ZERO
            return ZERO, self.snapshot(now=now, grid=grid, pose_map=pose_map, signature=signature)

        if facts_due:
            self._update_saturation(now, grid, signature)

        if valid < self.min_valid_ranges:
            self._last_action = ZERO
            return ZERO, self.snapshot(now=now, grid=grid, pose_map=pose_map, signature=signature)

        # A map with no known cell has no frontier to find, and the frontier
        # detector would answer COMPLETE — which must never be read as "the room
        # is done". Sweep in place until the map is dense enough to plan on, toward
        # the freer side so the sweep itself is not the thing that gets boxed in.
        if known_cells(grid) < self.bootstrap_min_known:
            left = self._side_min(ranges, angle_min, angle_increment, 0.0, math.radians(90))
            right = self._side_min(ranges, angle_min, angle_increment, -math.radians(90), 0.0)
            direction = 1.0 if left >= right else -1.0
            self._last_explorer_state = "BOOTSTRAP"
            self._last_explorer_reason = "sparse_map"
            self._last_action = ActionProposal(
                0.0, direction * self.max_angular * 0.5, 250, "tour", "map_bootstrap"
            )
            return self._last_action, self.snapshot(
                now=now, grid=grid, pose_map=pose_map, signature=signature
            )

        # A tour that never moves must not be mistaken for a finished map.
        if facts_due:
            known_now = known_cells(grid)
            if known_now > self._known_last:
                self._last_progress_at = now
            self._known_last = known_now
        if self._encoder_path > self._encoder_last + 0.01 or self._yaw_path > self._yaw_last + 0.03:
            self._last_progress_at = now
            self._encoder_last = self._encoder_path
            self._yaw_last = self._yaw_path
        if now - self._last_progress_at > self.no_progress_timeout:
            self.phase = LIMIT
            self.reason = "no_progress"
            self._last_action = ZERO
            return ZERO, self.snapshot(now=now, grid=grid, pose_map=pose_map, signature=signature)

        if self.phase == EXPLORE:
            if self._saturated() or self._should_reserve_return(now, grid, pose_map):
                self._begin_return(now)
            else:
                target = self._route_target(now, grid, pose_map)
                decision = self._explorer.decide(grid, pose_map, ranges, angle_min, angle_increment,
                                                 now=now, navigation_target=target, route_required=True)
                # Record *what* was aimed at, so an oscillation can be attributed to
                # target switching versus a steering loop (docs/SIM-CONTRACT.md §6).
                self._last_explorer_state = decision.state
                self._last_explorer_reason = decision.reason
                self._last_target = decision.target
                self._last_target_id = self._target_id(decision.target, grid)
                self._last_bearing_error = self._bearing_to(decision.target, pose_map)
                # A single COMPLETE sample is not proof the room is covered;
                # saturation (repeated empty frontiers or flat map growth) is.
                self._last_action = ZERO if decision.state == "COMPLETE" else decision.proposal
                return self._last_action, self.snapshot(
                    now=now, grid=grid, pose_map=pose_map, signature=signature
                )

        if self.phase == RETURN:
            target = self._route_target(now, grid, pose_map)
            if target is None:
                action = ZERO
            else:
                decision = self._explorer.decide(grid, pose_map, ranges, angle_min, angle_increment,
                                                 now=now, navigation_target=target, route_required=True)
                action = decision.proposal
                self._last_explorer_state = decision.state
                self._last_explorer_reason = decision.reason
            self._last_action = action
            self._last_target = self._start_pose_map
            self._last_target_id = "home"
            self._last_bearing_error = (
                None
                if self._start_pose_map is None
                else round(
                    _angle_error(
                        pose_map[2],
                        math.atan2(
                            self._start_pose_map[1] - pose_map[1],
                            self._start_pose_map[0] - pose_map[0],
                        ),
                    ),
                    4,
                )
            )
            if self._start_pose_map is not None and _distance(
                pose_map, self._start_pose_map
            ) <= self.target_reached:
                self.phase = DONE
                self.reason = "returned_home"
                self._last_action = ZERO
            elif self._return_started is not None and now - self._return_started > self.return_timeout:
                self.phase = LIMIT
                self.reason = "return_timeout"
                self._last_action = ZERO
            return self._last_action, self.snapshot(now=now, grid=grid, pose_map=pose_map, signature=signature)

        return ZERO, self.snapshot(now=now, grid=grid, pose_map=pose_map, signature=signature)

    def _begin_return(self, now: float) -> None:
        if self.phase != EXPLORE:
            return
        self.phase = RETURN
        self._return_started = now
        self._last_action = ZERO

    def _route_target(self, now, grid, pose):
        if self._explorer.recovery_phase:
            self._route_at = float('-inf')
            self._route = []
            self._segments = []
            return self._last_target
        if now - self._route_at >= 1.0 or self._route_phase != self.phase:
            router = GridRouter(grid, pose)
            if not self._can_keep_route(router, pose):
                self._route = (router.home(self._start_pose_map) if self.phase == RETURN
                               else router.frontier(pose, self._explorer._blocked_targets))
                self._segments = smooth_route(router, [pose[:2], *self._route]) if self.stable_routes and self._route else []
            self._route_at = now
            self._route_phase = self.phase
        while len(self._segments) > 1 and _distance(self._segments[0], pose) <= 0.057:
            self._segments.pop(0)
        if self._segments:
            return self._segments[0]
        return waypoint(self._route, pose)

    def _can_keep_route(self, router, pose):
        if not self.stable_routes or not self._route or self._route_phase != self.phase:
            return False
        if _distance(self._route[-1], pose) <= 0.055:
            return False
        goal = router.grid.world_to_cell(*self._route[-1])
        if goal in self._explorer._blocked_targets:
            return False
        if self.phase == EXPLORE:
            # A previously selected frontier may already be seen from here.
            # Reuse the route only while the destination still offers new cells.
            x, y = goal
            if not any(router.grid.cell(x + dx, y + dy) == -1
                       for dx, dy in ((1, 0), (0, 1), (-1, 0), (0, -1))):
                return False
        if self._segments:
            points = [pose[:2], *self._segments]
            return all(segment_is_free(router.grid, router.free, left, right)
                       for left, right in zip(points, points[1:]))
        nearest = nearest_on_route(self._route, pose)
        if _distance(self._route[nearest], pose) > 2 * router.grid.resolution:
            return False
        cells = [router.grid.world_to_cell(*point) for point in self._route[nearest:]]
        return all(cell in router.free for cell in cells)

    def _should_reserve_return(self, now, grid, pose):
        # Check on planning ticks in the second half of the distance budget.
        # A long detour must start home before the existing travel limit can
        # stop it mid-return. Use the length of the known route around obstacles.
        if (not self.stable_routes or self._start_pose_map is None or self._explorer.recovery_phase
                or self._encoder_path < self.max_travel / 2 or now - self._route_at < 1.0):
            return False
        router = GridRouter(grid, pose)
        home = router.home(self._start_pose_map)
        if not home:
            return False
        start = pose[:2]
        route = [start, *(smooth_route(router, [start, *home]) or home)]
        return self._encoder_path + sum(math.dist(a, b) for a, b in zip(route, route[1:])) + 0.30 >= self.max_travel

    def _saturated(self) -> bool:
        if self._no_frontier >= self.no_frontier_limit:
            return True
        return (
            self._growth_window is not None
            and self._growth_window <= self.growth_eps
            and self._known_last > self._known_start
        )

    def _update_saturation(self, now: float, grid: GridMap, signature: Sequence[float | None]) -> None:
        # Backend ticks at 50 Hz; saturation is a time window, not 3 control ticks.
        if self._known_history and now - self._known_history[-1][0] < 0.9:
            return
        self._frontier_count = len(grid.frontier_candidates(stride=4))
        if self._frontier_count == 0:
            self._no_frontier += 1
        else:
            self._no_frontier = 0
        known = known_cells(grid)
        self._known_history.append((now, known))
        while self._known_history and now - self._known_history[0][0] > self.saturation_window:
            self._known_history.popleft()
        if len(self._known_history) >= 2 and now - self._known_history[0][0] >= self.saturation_window * 0.9:
            self._growth_window = known - self._known_history[0][1]
        else:
            self._growth_window = None
        changed, mean_abs, _ = _signature_delta(signature, self._scan_prev)
        self._scan_prev_delta = (changed, mean_abs)
        self._scan_prev = list(signature)

    @staticmethod
    def _target_id(target: tuple[float, float] | None, grid: GridMap | None) -> str | None:
        """Stable identity for a frontier target: its map cell, not a float pair."""

        if target is None:
            return None
        if grid is None:
            return f"xy:{target[0]:.2f},{target[1]:.2f}"
        cell = grid.world_to_cell(target[0], target[1])
        return f"cell:{cell[0]},{cell[1]}"

    @staticmethod
    def _bearing_to(target: tuple[float, float] | None, pose: Sequence[float]) -> float | None:
        """Target bearing error in the robot frame (positive = turn left)."""

        if target is None:
            return None
        heading = math.atan2(target[1] - pose[1], target[0] - pose[0])
        return round(_angle_error(pose[2], heading), 4)

    @staticmethod
    def _side_min(
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        low: float,
        high: float,
    ) -> float:
        values = []
        for index, value in enumerate(ranges):
            if not math.isfinite(value) or value <= 0:
                continue
            angle = angle_min + index * angle_increment
            if low <= angle <= high:
                values.append(value)
        return min(values, default=float("inf"))

    # -- facts --------------------------------------------------------------
    def snapshot(
        self,
        *,
        now: float | None = None,
        grid: GridMap | None = None,
        pose_map: tuple[float, float, float] | None = None,
        signature: Sequence[float | None] | None = None,
    ) -> dict[str, Any]:
        now = time.monotonic() if now is None else now
        pose_map = pose_map or self._last_pose_map
        pose_odom = self._last_pose_odom
        known = known_cells(grid) if grid is not None else self._known_start
        total = (grid.width * grid.height) if grid is not None else None
        growth = self._growth_window
        if growth is None and self._known_history:
            growth = known - self._known_history[0][1]
        current_sig = list(signature) if signature is not None else list(self._scan_prev)
        changed_prev, mean_prev = self._scan_prev_delta
        changed_start, mean_start, _ = _signature_delta(current_sig, self._scan_start)
        elapsed = None if self._started_at is None else round(now - self._started_at, 1)
        dist_home = (
            None
            if self._start_pose_map is None or pose_map is None
            else round(_distance(pose_map, self._start_pose_map), 3)
        )
        return {
            "request_id": self.request_id,
            "source": self.source,
            "strategy": "tour",
            "frame": "map",
            "phase": self.phase,
            "active": self.active,
            "reason": self.reason,
            "elapsed_s": elapsed,
            "pose_map": None if pose_map is None else [round(float(v), 3) for v in pose_map],
            "pose_odom": None if pose_odom is None else [round(float(v), 3) for v in pose_odom],
            "encoder_delta_m": None
            if self._start_pose_odom is None or pose_odom is None
            else round(_distance(pose_odom, self._start_pose_odom), 3),
            "encoder_path_m": round(self._encoder_path, 3),
            "encoder_twist_mps": self._encoder_twist,
            "tf_path_m": round(self._tf_path, 3),
            "tf_delta_m": dist_home,
            "scan_sig": {
                "bins": SCAN_BINS,
                "changed_bins_vs_prev": changed_prev,
                "mean_abs_delta_m": round(mean_prev, 3),
                "changed_bins_vs_start": changed_start,
                "mean_abs_delta_vs_start_m": round(mean_start, 3),
            },
            "map": {
                "known_cells": known,
                "known_delta_start": known - self._known_start,
                "known_growth_window": growth,
                "explored_ratio": None if not total else round(known / total, 4),
                "frontier_count": self._frontier_count,
            },
            "imu": dict(self._imu),
            # What the planner aimed at this cycle: an oscillation can then be
            # attributed to target switching vs a steering loop vs a veto.
            "navigation": {
                "target": None
                if self._last_target is None
                else [round(float(v), 3) for v in self._last_target],
                "target_id": self._last_target_id,
                "bearing_error_rad": self._last_bearing_error,
                "explorer_state": self._last_explorer_state,
                "explorer_reason": self._last_explorer_reason,
                "max_angular_rps": self.max_angular,
                "obstacle_stop_m": self.obstacle_stop,
                "route_cells": len(self._route),
                "route_goal_id": self._target_id(self._route[-1], grid) if self._route else None,
                "recovery": self._explorer.recovery_snapshot(),
            },
            "cmd": {
                "autonomy": [round(self._last_action.linear_mps, 4), round(self._last_action.angular_rps, 4)],
                "actuator": [round(self._actuator[0], 4), round(self._actuator[1], 4)],
                "actuator_age_s": self._actuator_age,
            },
            "publishers": dict(self._publishers),
            "saturation": {
                "no_frontier_samples": self._no_frontier,
                "no_frontier_limit": self.no_frontier_limit,
                "growth_window_cells": growth,
                "growth_eps_cells": self.growth_eps,
                "window_s": self.saturation_window,
            },
            "return": {
                "target": None if self._start_pose_map is None else [round(self._start_pose_map[0], 3), round(self._start_pose_map[1], 3)],
                "started": self._return_started is not None,
                "elapsed_s": None
                if self._return_started is None
                else round(now - self._return_started, 1),
                "dist_to_start_m": dist_home,
                "timeout_s": self.return_timeout,
            },
            "envelope": {
                "runtime_s": elapsed,
                "runtime_limit_s": self.max_runtime,
                "travel_m": round(self._encoder_path, 3),
                "travel_limit_m": self.max_travel,
                "battery_v": self._battery_v,
                "battery_min_v": self.min_battery,
            },
        }
