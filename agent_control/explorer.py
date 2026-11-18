"""Deterministic frontier explorer used by the ROS mapping node."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Sequence

from .protocol import ActionProposal
from .navigation import BODY_RADIUS_M, TURN_RANGE_M


@dataclass(frozen=True)
class GridMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: Sequence[int]

    def cell(self, x: int, y: int) -> int:
        if x < 0 or y < 0 or x >= self.width or y >= self.height:
            return 100
        return int(self.data[y * self.width + x])

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (math.floor((x - self.origin_x) / self.resolution), math.floor((y - self.origin_y) / self.resolution))

    def cell_to_world(self, x: int, y: int) -> tuple[float, float]:
        return (self.origin_x + (x + 0.5) * self.resolution, self.origin_y + (y + 0.5) * self.resolution)

    def frontier_candidates(self, stride: int = 2) -> list[tuple[float, float, int]]:
        candidates: list[tuple[float, float, int]] = []
        for y in range(1, self.height - 1, stride):
            for x in range(1, self.width - 1, stride):
                if self.cell(x, y) != -1:
                    continue
                neighbors = [self.cell(x + dx, y + dy) for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))]
                free_neighbors = sum(1 for value in neighbors if 0 <= value <= 25)
                if free_neighbors:
                    wx, wy = self.cell_to_world(x, y)
                    candidates.append((wx, wy, free_neighbors))
        return candidates


@dataclass(frozen=True)
class ExplorerDecision:
    proposal: ActionProposal
    state: str
    target: tuple[float, float] | None
    reason: str


class FrontierExplorer:
    """Small, deterministic local controller; it is not a replacement for Nav2."""

    def __init__(
        self,
        *,
        max_linear_mps: float = 0.04,
        max_angular_rps: float = 0.3,
        obstacle_stop_m: float = 0.32,
        target_reached_m: float = 0.22,
        max_runtime_s: float = 300.0,
        max_travel_m: float = 8.0,
        replan_period_s: float = 1.0,
        avoid_hold_s: float = 2.5,
        avoid_switch_limit: int = 1,
        avoid_worsen_m: float = 0.05,
        body_radius_m: float = BODY_RADIUS_M,
        sweep_margin_m: float = 0.05,
        reverse_speed_mps: float = 0.03,
        reverse_clearance_m: float = 0.30,
        reverse_step_m: float = 0.08,
        max_recoveries: int = 2,
        max_reverse_total_m: float = 0.20,
        pursuit_stall_s: float = 6.0,
    ) -> None:
        self.max_linear = max_linear_mps
        self.max_angular = max_angular_rps
        self.obstacle_stop = obstacle_stop_m
        self.target_reached = target_reached_m
        self.max_runtime = max_runtime_s
        self.max_travel = max_travel_m
        self.replan_period = replan_period_s
        self.started_at: float | None = None
        self.start_pose: tuple[float, float] | None = None
        self.target: tuple[float, float] | None = None
        self.last_plan_at = 0.0
        self._avoid_until = 0.0
        self._avoid_dir = 1.0
        self._avoid_active = False
        self._avoid_switches = 0
        # A hold window has to outlast the rotation that caused the block. At
        # max_angular 0.3 rad/s, 2.5 s is ~43 deg: enough for the obstacle to
        # leave the +-25 deg front cone instead of being pushed into the other
        # half-sector, which is what made the old 1 s window flip back.
        self.avoid_hold = max(0.2, float(avoid_hold_s))
        self.avoid_switch_limit = max(0, int(avoid_switch_limit))
        # Latched turns still have to be watched: in the field a latched direction
        # rotated a crate *into* the front cone until the executor's own close-range
        # gate blocked every motion. If clearance gets worse than it was when the
        # episode started, reverse immediately instead of waiting for the hold.
        self.avoid_worsen = max(0.0, float(avoid_worsen_m))
        self._avoid_front_at_start = float("inf")
        # A turn sweeps the body's corner, not just its centre line: the radius of
        # CAD half-width is 0.081 including the wheels, not 0.0475 chassis-only.
        # Scan-based sweep checks also account for the lidar's rearward offset.
        self.body_radius = max(0.02, float(body_radius_m))
        self.sweep_margin = max(0.0, float(sweep_margin_m))
        # Bounded reverse recovery: a boxed-in robot has no in-place escape left.
        self.reverse_speed = max(0.01, float(reverse_speed_mps))
        self.reverse_clearance = max(0.05, float(reverse_clearance_m))
        self.reverse_step = max(0.01, float(reverse_step_m))
        self.max_recoveries = max(0, int(max_recoveries))
        self.max_reverse_total = max(0.0, float(max_reverse_total_m))
        self._recoveries = 0
        self._reversed_total = 0.0
        self._recovery_id = 0
        self._recovery_reason: str | None = None
        # Pursuit that cannot advance (target behind an obstacle: the bearing never
        # closes because MOVE turns one way and AVOID turns back) must escalate to
        # recovery instead of holding that limit cycle forever.
        self.pursuit_stall = max(1.0, float(pursuit_stall_s))
        self._stall_since: float | None = None
        # A target that forced a recovery is not reachable from here right now:
        # without this the replan re-picks the same frontier and the recovery
        # budget is spent on the same deadlock.
        self._blocked_targets: dict[tuple[int, int], float] = {}
        self.target_block_s = 20.0
        self.recovery_phase = None
        self._last_pose = None
        self._recovery_pose = None
        self._recovery_started = 0.0
        self._recovery_heading = None
        self._recovery_distance = 0.0
        self._exit_pose = None
        self._stall_error = float('inf')

    @staticmethod
    def _angle_error(current: float, target: float) -> float:
        return (target - current + math.pi) % (2 * math.pi) - math.pi

    def _pick_target(
        self, grid: GridMap, pose: tuple[float, float, float], now: float | None = None
    ) -> tuple[float, float] | None:
        candidates = grid.frontier_candidates()
        if not candidates:
            return None
        if now is not None and self._blocked_targets:
            # Drop expired blocks, then ignore frontiers a recovery just failed on.
            self._blocked_targets = {
                cell: expiry for cell, expiry in self._blocked_targets.items() if expiry > now
            }
            if self._blocked_targets:
                candidates = [
                    item
                    for item in candidates
                    if grid.world_to_cell(item[0], item[1]) not in self._blocked_targets
                ]
                if not candidates:
                    return None
        x, y, _ = pose
        # A frontier inside the acceptance radius cannot be pursued: the
        # controller would declare it reached forever without moving.
        reachable = [
            item
            for item in candidates
            if math.hypot(item[0] - x, item[1] - y) > self.target_reached * 1.2
        ]
        if not reachable:
            return None
        reachable.sort(key=lambda item: ((item[0] - x) ** 2 + (item[1] - y) ** 2) / max(1, item[2]))
        return reachable[0][0], reachable[0][1]

    def decide(
        self,
        grid: GridMap,
        pose: tuple[float, float, float],
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        now: float | None = None,
        navigation_target: tuple[float, float] | None = None,
        route_required: bool = False,
    ) -> ExplorerDecision:
        now = time.monotonic() if now is None else now
        self._last_pose = pose
        if self.started_at is None:
            self.started_at = now
            self.start_pose = (pose[0], pose[1])
        elapsed = now - self.started_at
        travelled = math.hypot(pose[0] - self.start_pose[0], pose[1] - self.start_pose[1]) if self.start_pose else 0.0
        if elapsed > self.max_runtime:
            return ExplorerDecision(ActionProposal(0.0, 0.0, 250, "explorer", "runtime limit"), "LIMIT", self.target, "runtime_limit")
        if travelled > self.max_travel:
            return ExplorerDecision(ActionProposal(0.0, 0.0, 250, "explorer", "travel limit"), "LIMIT", self.target, "travel_limit")
        valid_ranges = [value for value in ranges if math.isfinite(value) and value > 0]
        if len(valid_ranges) < 12:
            return ExplorerDecision(ActionProposal(0.0, 0.0, 250, "explorer", "insufficient scan"), "BLOCKED", self.target, "scan_invalid")

        # A reverse recovery already under way finishes first: planning must not
        # interrupt it, and it stops the moment the rear closes.
        rear = min(self._sector_min(ranges, angle_min, angle_increment, 150.0, 180.0),
                   self._sector_min(ranges, angle_min, angle_increment, -180.0, -150.0))
        if self.recovery_phase is not None:
            return self._recover(now, pose, ranges, angle_min, angle_increment, rear)
        if route_required and navigation_target is None:
            # Never fall back to chasing an unknown frontier through an obstacle.
            # An initial pose inside the route's margin must first leave it.
            if min(valid_ranges) < 0.38:
                return self._start_reverse(now, rear, "route_clearance", grid)
            self.target = None
            return ExplorerDecision(ActionProposal(0.0, 0.0, 250, "explorer", "no reachable frontier"),
                                    "COMPLETE", None, "no_reachable_frontier")
        if navigation_target is not None:
            self.target = navigation_target
        elif self.target is None or now - self.last_plan_at >= self.replan_period:
            self.target = self._pick_target(grid, pose, now)
            self.last_plan_at = now
        if self.target is None:
            return ExplorerDecision(ActionProposal(0.0, 0.0, 250, "explorer", "no frontier"), "COMPLETE", None, "no_frontier")

        target_x, target_y = self.target
        distance = math.hypot(target_x - pose[0], target_y - pose[1])
        if distance <= (0.055 if navigation_target is not None else self.target_reached):
            self.target = None
            return ExplorerDecision(ActionProposal(0.0, 0.0, 250, "explorer", "frontier reached"), "REPLAN", None, "frontier_reached")

        front_values: list[float] = []
        for index, value in enumerate(ranges):
            if not math.isfinite(value) or value <= 0:
                continue
            angle = angle_min + index * angle_increment
            if abs(angle) <= math.radians(25):
                front_values.append(value)
        front = min(front_values, default=float("inf"))
        if front < self.obstacle_stop:
            # What a turn actually needs free is the body's swept corner on the
            # side we rotate toward -- not the raw 25..90 deg minimum, which would
            # forbid driving along a wall at a perfectly normal distance.
            need = max(TURN_RANGE_M, self.body_radius + self.sweep_margin)
            left_sweep = self._sector_min(ranges, angle_min, angle_increment, 5.0, 120.0)
            right_sweep = self._sector_min(ranges, angle_min, angle_increment, -120.0, -5.0)
            left_ok = left_sweep >= need
            right_ok = right_sweep >= need
            if not left_ok and not right_ok:
                # No in-place escape left: back out (bounded) or report it.
                return self._start_reverse(now, rear, "boxed_in", grid)
            if not self._avoid_active:
                # Latch the escape direction when the block starts. Re-choosing it
                # while the robot rotates is the bug that made the chassis wriggle:
                # each 20 deg of turn pushes the obstacle out of the front cone and
                # into the *other* half-sector, so the comparison flips and the
                # robot turns straight back.
                self._avoid_active = True
                self._avoid_switches = 0
                if left_ok and right_ok:
                    self._avoid_dir = 1.0 if left_sweep >= right_sweep else -1.0
                else:
                    self._avoid_dir = 1.0 if left_ok else -1.0
                self._avoid_until = now + self.avoid_hold
                self._avoid_front_at_start = front
            else:
                # A latched turn that closes on the obstacle is the wrong way.
                worsening = front < self._avoid_front_at_start - self.avoid_worsen
                latched_blocked = (
                    self._avoid_dir > 0 and not left_ok
                ) or (
                    self._avoid_dir < 0 and not right_ok
                )
                if worsening or latched_blocked or now >= self._avoid_until:
                    if self._avoid_switches >= self.avoid_switch_limit:
                        # Both directions cost clearance: recover backwards if the
                        # budget and the rear allow it, otherwise report stuck.
                        return self._start_reverse(now, rear, "turns_blocked", grid)
                    self._avoid_switches += 1
                    self._avoid_dir = -self._avoid_dir
                    self._avoid_until = now + self.avoid_hold
                    self._avoid_front_at_start = front
            return ExplorerDecision(
                ActionProposal(0.0, self._avoid_dir * self.max_angular, 250, "explorer", "obstacle clearance"),
                "AVOID",
                self.target,
                "obstacle",
            )

        # The front is clear again: leave the avoid episode and resume pursuit.
        self._avoid_active = False
        self._avoid_switches = 0

        heading = math.atan2(target_y - pose[1], target_x - pose[0])
        error = self._angle_error(pose[2], heading)
        angular = max(-self.max_angular, min(self.max_angular, error * 1.2))
        linear = min(self.max_linear, max(0.015, distance * 0.35))
        if abs(error) > 0.65:
            linear = 0.0
            # Turning toward a target it cannot face yet is only useful if it
            # eventually gets there. If it does not, back out and re-plan.
            if self._stall_since is None:
                self._stall_since = now
                self._stall_error = abs(error)
            elif abs(error) < self._stall_error - 0.05:
                self._stall_since = now
                self._stall_error = abs(error)
            elif now - self._stall_since >= self.pursuit_stall:
                self._stall_since = None
                return self._start_reverse(now, rear, "pursuit_stalled", grid)
        else:
            self._stall_since = None
        return ExplorerDecision(ActionProposal(linear, angular, 250, "explorer", "frontier pursuit"), "MOVE", self.target, "frontier_pursuit")

    # -- recovery -----------------------------------------------------------
    def _sector_min(
        self,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        low_deg: float,
        high_deg: float,
    ) -> float:
        low = math.radians(min(low_deg, high_deg))
        high = math.radians(max(low_deg, high_deg))
        values = []
        for index, value in enumerate(ranges):
            if not math.isfinite(value) or value <= 0:
                continue
            angle = angle_min + index * angle_increment
            if low <= angle <= high:
                values.append(value)
        return min(values, default=0.0)

    def can_recover(self) -> bool:
        """True while a bounded reverse is still affordable."""

        budget_left = self.max_reverse_total - self._reversed_total
        return self._recoveries < self.max_recoveries and budget_left >= self.reverse_step - 1e-9

    def force_recovery(self, now: float, reason: str, grid: GridMap | None = None) -> bool:
        """Start a reverse because the executor refused the proposed motion.

        The explorer sees the world through its own cones; the executor's gate can
        still refuse a turn (body sweep). This is the deterministic hand-off:
        trigger -> reverse -> re-observe -> replan, without freezing the task.
        """

        if not self.can_recover():
            return False
        self._recoveries += 1
        self._recovery_id += 1
        self._recovery_reason = reason
        if self.target is not None and grid is not None:
            cell = grid.world_to_cell(self.target[0], self.target[1])
            self._blocked_targets[cell] = now + self.target_block_s
        self.recovery_phase = "CHECK_REAR"
        self._recovery_started = now
        self._recovery_pose = self._last_pose
        self._recovery_distance = 0.0
        self._avoid_active = False
        self._avoid_switches = 0
        self._stall_since = None
        self.target = None
        self.last_plan_at = 0.0
        return True
    def _start_reverse(self, now: float, rear: float, reason: str, grid: GridMap | None = None) -> ExplorerDecision:
        """Back out of a boxed-in pose, bounded by count, distance and rear clearance.

        The rear is checked on entry (and every cycle while reversing, in
        ``decide``), so a blocked front never authorises driving backwards into
        something else. When the budget is spent the decision is ``BLOCKED`` and
        the orchestrator escalates to HANDOVER instead of spinning forever.
        """

        budget_left = self.max_reverse_total - self._reversed_total
        if (
            self._recoveries >= self.max_recoveries
            or budget_left < self.reverse_step - 1e-9
            or rear < self.reverse_clearance
        ):
            return ExplorerDecision(
                ActionProposal(0.0, 0.0, 250, "explorer", "stuck"),
                "BLOCKED",
                self.target,
                "obstacle_stuck",
            )
        self.force_recovery(now, reason, grid)
        self.recovery_phase = "REVERSE"
        return ExplorerDecision(
            ActionProposal(-self.reverse_speed, 0.0, 250, "explorer", "reverse recovery"),
            "RECOVER",
            self.target,
            "reverse",
        )

    def _recover(self, now, pose, ranges, angle_min, increment, rear):
        def result(v=0.0, w=0.0, reason=None, state="RECOVER"):
            return ExplorerDecision(ActionProposal(v, w, 250, "explorer", reason or self.recovery_phase),
                                    state, self.target, reason or self.recovery_phase.lower())

        def fail(reason):
            self.recovery_phase = "FAILED"
            self._recovery_reason = reason
            return result(reason=reason, state="BLOCKED")

        if self.recovery_phase == "FAILED":
            return result(reason=self._recovery_reason, state="BLOCKED")
        if now - self._recovery_started > 20.0:
            return fail("recovery_timeout")
        if self.recovery_phase == "CHECK_REAR":
            self._recovery_pose = pose
            self.recovery_phase = "REVERSE"
        if self.recovery_phase == "REVERSE":
            # Charge measured pose displacement, never requested speed x time.
            distance = math.dist(pose[:2], self._recovery_pose[:2]) if self._recovery_pose else 0.0
            self._reversed_total += max(0.0, distance - self._recovery_distance)
            self._recovery_distance = distance
            if rear < self.reverse_clearance:
                return fail("reverse_blocked")
            clearance = min(v for v in ranges if math.isfinite(v) and v > 0)
            if distance >= 0.04 and clearance >= TURN_RANGE_M:
                self.recovery_phase = "SELECT_EXIT"
                return result()
            if self._reversed_total >= self.max_reverse_total - 0.002:
                return fail("reverse_budget")
            if now - self._recovery_started > 3.0 and distance < 0.005:
                return fail("reverse_no_progress")
            return result(-self.reverse_speed, reason="reverse")
        if self.recovery_phase == "SELECT_EXIT":
            choices = []
            for degrees in range(-180, 180, 15):
                # Clearance over the whole forward cone, not one long ray.
                values = [v for i, v in enumerate(ranges) if math.isfinite(v) and v > 0
                          and abs(self._angle_error(math.radians(degrees), angle_min + i * increment)) <= math.radians(25)]
                if values and min(values) >= max(self.obstacle_stop + 0.10, 0.45):
                    choices.append((min(values), -abs(degrees), degrees))
            if not choices:
                return fail("no_safe_exit")
            self._recovery_heading = pose[2] + math.radians(max(choices)[2])
            self.recovery_phase = "TURN"
            return result()
        if self.recovery_phase == "TURN":
            if min(v for v in ranges if math.isfinite(v) and v > 0) < TURN_RANGE_M:
                return fail("turn_sweep_blocked")
            error = self._angle_error(pose[2], self._recovery_heading)
            if abs(error) < 0.08:
                self.recovery_phase = "EXIT"
                self._exit_pose = pose
                return result()
            return result(w=max(-self.max_angular, min(self.max_angular, error)))
        if self.recovery_phase == "EXIT":
            if self._sector_min(ranges, angle_min, increment, -25, 25) < self.obstacle_stop:
                return fail("exit_blocked")
            if math.dist(pose[:2], self._exit_pose[:2]) < 0.15:
                return result(min(self.max_linear, 0.04))
            self.recovery_phase = "REPLAN"
            return result()
        self.recovery_phase = None
        self.target = None
        self.last_plan_at = 0.0
        return result(reason="recovery_complete", state="REPLAN")

    def recovery_snapshot(self):
        return {"id": self._recovery_id, "stage": self.recovery_phase,
                "reason": self._recovery_reason, "reverse_m": round(self._recovery_distance, 4),
                "reverse_total_m": round(self._reversed_total, 4), "count": self._recoveries,
                "exit_heading_rad": self._recovery_heading}

    def rebase(self, now):
        """Pause time must not consume the recovery or pursuit watchdog."""
        if self.recovery_phase:
            self._recovery_started = now
        self._stall_since = None
