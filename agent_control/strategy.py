"""Long-task orchestration for a strategy run (docs/SIM-CONTRACT.md §6).

Three layers, and this module is the middle one:

* ``CoverageController`` (``coverage.py``) turns scene geometry into **one**
  ``ActionProposal`` (v, w). It never sends a goal and never publishes.
* ``StrategyRunner`` (here) owns the variant choice, the single goal lease, the
  long-task state machine, the transition event log and the acceptance verdict.
  It hands the proposal to the executor and records what the executor answered.
* ``robotd`` / ``Cerebellum`` execute: they validate skill vocabulary, clearance
  and sensor freshness and are the only layer that produces motion.

So an LLM may start/pause/resume/cancel a tour and explain the facts, but it
cannot put a speed into the control loop, and this module cannot either.

States (contract §6)::

    IDLE -> STARTING -> RUNNING <-> PAUSED
                         |- BLOCKED   (obstacle / stale sensor, with reason+clearance)
                         |- TIMEOUT   (acceptance envelope)
                         |- HANDOVER  (a human has to take over)
                         |- CANCELLED
                         `- DONE      (reason: reached | exhausted | stopped)

Every transition -- including ``pause`` / ``resume``, which may leave the robot
where it was -- appends one event ``{seq, at, from, to, reason, facts}`` with a
monotonic ``seq`` cursor so a console can replay the run without re-deriving it
from a 50 Hz stream.
"""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

from .coverage import (
    DEFAULT_VARIANT_ID,
    LIMIT as COVERAGE_LIMIT,
    STOPPED as COVERAGE_STOPPED,
    DONE as COVERAGE_DONE,
    CoverageController,
    CoverageVariant,
    VariantError,
    load_variant,
)
from .explorer import GridMap
from .protocol import ActionProposal
from .navigation import TURN_RANGE_M
from .tour import TourController

STRATEGY = "coverage"


class StrategyPhase:
    """Contract §6 states; the values are the wire strings."""

    IDLE = "IDLE"
    STARTING = "STARTING"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    BLOCKED = "BLOCKED"
    TIMEOUT = "TIMEOUT"
    HANDOVER = "HANDOVER"
    CANCELLED = "CANCELLED"
    DONE = "DONE"


# STARTING..BLOCKED hold the single goal lease (contract §1): while any of them
# is current a single-step goal must answer ``busy``.
LEASE_PHASES = (
    StrategyPhase.STARTING,
    StrategyPhase.RUNNING,
    StrategyPhase.PAUSED,
    StrategyPhase.BLOCKED,
)
TERMINAL_PHASES = (
    StrategyPhase.TIMEOUT,
    StrategyPhase.HANDOVER,
    StrategyPhase.CANCELLED,
    StrategyPhase.DONE,
)

# facts every transition must carry (contract §6); ``phase`` and ``strategy`` are
# added by the runner, the rest come from the measurements of that instant.
REQUIRED_FACT_KEYS = (
    "pose",
    "scan_age_s",
    "known_cells",
    "traveled_m",
    "strategy",
    "phase",
)


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


# Both strategies are pluggable behind the same five calls, so the state machine,
# the lease and the acceptance envelope do not know which one is running:
#
#     start(now, request_id, source, pose, ranges, angle_min, angle_increment, grid)
#     step(now, pose, ranges, angle_min, angle_increment, grid) -> (proposal, facts)
#     rebase(now)            reset the stall clocks after a pause / blocked stretch
#     stop(reason)           zero the action and report
#     snapshot()             the strategy's own facts (phase, reason, measurements)
#
# Neither backend publishes, and neither one may send a goal: the runner hands
# their proposal to robotd exactly like a single-step goal would be handed over.
STRATEGIES = ("coverage", "tour")
BACKEND_TERMINAL_PHASES = ("DONE", "LIMIT", "STOPPED")


def normalise_strategy(name: object) -> str:
    value = str(name or STRATEGY).strip().lower()
    if value not in STRATEGIES:
        raise ValueError(f"unknown strategy {name!r}; known: {', '.join(STRATEGIES)}")
    return value


class CoverageBackend:
    """The deterministic lane sweep behind the shared backend shape."""

    name = "coverage"

    def __init__(self, controller: CoverageController) -> None:
        self.controller = controller

    def start(self, *, now, request_id, source, pose, ranges, angle_min, angle_increment, grid=None):
        self.controller.start(
            now=now,
            request_id=request_id,
            source=source,
            pose=pose,
            ranges=ranges,
            angle_min=angle_min,
            angle_increment=angle_increment,
        )

    def step(self, *, now, pose, ranges, angle_min, angle_increment, grid=None):
        return self.controller.step(
            now=now,
            pose=pose,
            ranges=ranges,
            angle_min=angle_min,
            angle_increment=angle_increment,
        )

    def force_recovery(self, *, now, reason, grid=None):
        return False

    def rebase(self, *, now):
        self.controller.rebase(now=now)

    def stop(self, *, reason="stopped"):
        return self.controller.stop(reason=reason)

    def snapshot(self):
        return self.controller.snapshot()


class TourBackend:
    """The frontier tour behind the shared backend shape.

    A tour cannot start without a map and a pose, but the runner starts a task
    from the lease (no map yet). The first ``step`` that carries both performs
    the controller's ``start``; until then this returns zero motion with
    ``map_missing`` facts, which the runner surfaces as a normal zero proposal.
    """

    name = "tour"

    def __init__(self, controller: "TourController") -> None:
        self.controller = controller
        self._started = False
        self._pending: dict[str, Any] = {}

    def start(self, *, now, request_id, source, pose, ranges, angle_min, angle_increment, grid=None):
        self._started = False
        self._pending = {
            "request_id": request_id,
            "source": source,
            "pose": pose,
            "ranges": tuple(ranges),
            "angle_min": angle_min,
            "angle_increment": angle_increment,
            "grid": grid,
        }

    def _begin(self, *, now, pose, ranges, angle_min, angle_increment, grid) -> None:
        pending = self._pending
        self.controller.start(
            now=now,
            request_id=str(pending.get("request_id") or "strategy"),
            source=str(pending.get("source") or "strategy"),
            grid=grid,
            pose_map=pose,
            pose_odom=pose,
            ranges=ranges,
            angle_min=angle_min,
            angle_increment=angle_increment,
        )
        self._started = True

    def step(self, *, now, pose, ranges, angle_min, angle_increment, grid=None):
        if not self._started:
            pending = self._pending
            pose = pose if pose is not None else pending.get("pose")
            grid = grid if grid is not None else pending.get("grid")
            if pose is None or grid is None:
                return (
                    ActionProposal(0.0, 0.0, 250, "tour", "map_missing"),
                    {"phase": "IDLE", "reason": "map_missing"},
                )
            if not ranges:
                ranges = pending.get("ranges") or ()
            if not angle_increment and pending.get("angle_increment"):
                angle_min = pending["angle_min"]
                angle_increment = pending["angle_increment"]
            self._begin(
                now=now,
                pose=pose,
                ranges=ranges,
                angle_min=angle_min,
                angle_increment=angle_increment,
                grid=grid,
            )
        self.controller.observe(pose_odom=pose)
        return self.controller.step(
            now=now,
            grid=grid,
            pose_map=pose,
            ranges=ranges,
            angle_min=angle_min,
            angle_increment=angle_increment,
            facts_due=True,
        )

    def force_recovery(self, *, now, reason, grid=None):
        return bool(self.controller.force_recovery(now=now, reason=reason, grid=grid))

    def rebase(self, *, now):
        self.controller.rebase(now=now)

    def stop(self, *, reason="stopped"):
        return self.controller.stop(reason=reason)

    def snapshot(self):
        return self.controller.snapshot()



class StrategyRunner:
    """One long coverage task: lease, state machine, events, acceptance."""

    STRATEGY = STRATEGY

    def __init__(
        self,
        *,
        variant_id: str | None = None,
        variant_root: str | Path | None = None,
        strategy: str = STRATEGY,
        controller_factory: Callable[[CoverageVariant], CoverageController] | None = None,
        tour_factory: Callable[[], TourController] | None = None,
        max_linear_mps: float = 0.09,
        max_angular_rps: float = 0.45,
        obstacle_stop_m: float = 0.22,
        scan_timeout_s: float = 1.5,
        odom_timeout_s: float = 1.0,
        min_clearance_m: float = 0.25,
        min_clearance_turn_m: float = 0.12,
        min_reverse_clearance_m: float = 0.12,
        handover_after_s: float = 30.0,
        refusal_limit: int = 5,
        max_events: int = 256,
        clock: Callable[[], float] = time.time,
    ) -> None:
        self.variant_id = str(variant_id or DEFAULT_VARIANT_ID)
        self.variant_root = variant_root
        self.max_linear = float(max_linear_mps)
        self.max_angular = float(max_angular_rps)
        self.obstacle_stop = float(obstacle_stop_m)
        self.scan_timeout = float(scan_timeout_s)
        self.odom_timeout = float(odom_timeout_s)
        self.min_clearance = float(min_clearance_m)
        self.min_clearance_turn = float(min_clearance_turn_m)
        # A reverse is judged by the rear with its own threshold; the front rule
        # stays exactly as it was for forward and turn.
        self.min_reverse_clearance = float(min_reverse_clearance_m)
        self._recovering = False
        self._recovery_stage = None
        self.handover_after = float(handover_after_s)
        self.refusal_limit = max(1, int(refusal_limit))
        self.max_events = max(8, int(max_events))
        self.clock = clock
        self.strategy = normalise_strategy(strategy)
        self._factory = controller_factory or (
            lambda variant: CoverageController(
                variant_id=variant.variant_id,
                variant_root=variant_root,
                max_linear_mps=self.max_linear,
                max_angular_rps=self.max_angular,
                obstacle_stop_m=self.obstacle_stop,
            )
        )
        self._tour_factory = tour_factory or (
            lambda: TourController(
                max_linear_mps=self.max_linear,
                max_angular_rps=self.max_angular,
                obstacle_stop_m=max(self.obstacle_stop, 0.32),
                target_reached_m=0.18,
            )
        )

        self._lock = threading.RLock()
        self._phase = StrategyPhase.IDLE
        self._reason = "idle"
        self._variant: CoverageVariant | None = None
        self._backend: CoverageBackend | TourBackend | None = None
        self._backend_started = False
        self._request_id: str | None = None
        self._source: str | None = None
        self._events: list[dict[str, Any]] = []
        self._cursor = 0
        self._facts: dict[str, Any] = self._blank_facts()
        self._coverage_facts: dict[str, Any] | None = None
        self._coverage_phase: str | None = None
        self._proposal: dict[str, Any] = self._zero_proposal("idle")
        self._acceptance_result: dict[str, Any] | None = None
        self._started_mono: float | None = None
        self._started_wall: float | None = None
        # Per-tick trace for attributing an oscillation (contract §6 diagnosis):
        # target identity, bearing error, the strategy's proposal, and what the
        # executor was actually handed. Bounded ring buffer, oldest dropped first.
        self._trace: list[dict[str, Any]] = []
        self.trace_limit = 4096
        self._last_proposal: tuple[float, float] = (0.0, 0.0)
        self._paused_total = 0.0
        self._pause_started: float | None = None
        self._blocked_since: float | None = None
        self._refusals = 0
        self._forbidden_seen: list[str] = []
        self._traveled_start: float | None = None
        self._last_tick_mono: float | None = None
        self._last_scan_age: tuple[float, float] | None = None  # (age_s, measured_at)

    # -- introspection ------------------------------------------------------
    @property
    def phase(self) -> str:
        with self._lock:
            return self._phase

    @property
    def active(self) -> bool:
        with self._lock:
            return self._phase in LEASE_PHASES

    @property
    def variant(self) -> CoverageVariant | None:
        with self._lock:
            return self._variant

    def holds_goal_lease(self) -> bool:
        return self.active

    def controller_age_s(self) -> float | None:
        with self._lock:
            if self._last_tick_mono is None:
                return None
            return round(max(0.0, time.monotonic() - self._last_tick_mono), 3)

    # -- lifecycle ----------------------------------------------------------
    def start(
        self,
        *,
        request_id: str | None = None,
        source: str = "api",
        variant_id: str | None = None,
        strategy: str | None = None,
        now: float | None = None,
    ) -> dict[str, Any]:
        """Open a run; ``strategy`` picks coverage (default) or tour.

        Coverage needs a variant (lane geometry lives there). Tour does not: it
        reads the map itself, so a variant is optional and only used for the
        acceptance envelope when one is supplied.
        """

        with self._lock:
            if self._phase in LEASE_PHASES:
                return {
                    "accepted": False,
                    "error": "strategy_active",
                    "phase": self._phase,
                    "reason": self._reason,
                }
            try:
                target_strategy = normalise_strategy(strategy or self.strategy)
            except ValueError as exc:
                return {"accepted": False, "error": "unknown_strategy", "message": str(exc)}
            variant: CoverageVariant | None = None
            if target_strategy == "coverage" or variant_id:
                target = str(variant_id or self.variant_id)
                try:
                    variant = load_variant(target, root=self.variant_root)
                except VariantError as exc:
                    return {
                        "accepted": False,
                        "error": exc.code,
                        "available": list(exc.available),
                        "message": str(exc),
                    }
            mono = time.monotonic() if now is None else float(now)
            wall = self.clock()
            self.strategy = target_strategy
            if variant is not None:
                self.variant_id = variant.variant_id
            self._variant = variant
            self._backend = (
                CoverageBackend(self._factory(variant))
                if target_strategy == "coverage" and variant is not None
                else TourBackend(self._tour_factory())
            )
            self._backend_started = False
            self._request_id = str(request_id or f"strategy-{int(wall * 1000)}")[:64]
            self._source = str(source)[:32]
            self._started_mono = mono
            self._started_wall = wall
            self._paused_total = 0.0
            self._pause_started = None
            self._blocked_since = None
            self._refusals = 0
            self._forbidden_seen = []
            self._traveled_start = None
            self._last_tick_mono = mono
            self._last_scan_age = None
            self._coverage_facts = None
            self._coverage_phase = None
            self._recovery_stage = None
            self._acceptance_result = None
            self._proposal = self._zero_proposal("starting")
            self._facts = self._blank_facts()
            if variant is not None:
                self._facts["variant_id"] = variant.variant_id
            self._transition(StrategyPhase.STARTING, "requested", self._facts)
            return {"accepted": True, **self.status()}

    def pause(self, *, reason: str = "pause_requested", now: float | None = None) -> dict[str, Any]:
        """Stop now, keep the lease. The executor gets zero motion immediately."""

        with self._lock:
            if self._phase not in (StrategyPhase.STARTING, StrategyPhase.RUNNING, StrategyPhase.BLOCKED):
                return {"accepted": False, "error": "not_running", **self.status()}
            mono = time.monotonic() if now is None else float(now)
            if self._phase in (StrategyPhase.STARTING, StrategyPhase.RUNNING):
                self._pause_started = mono
            self._blocked_since = None
            self._proposal = self._zero_proposal("paused")
            self._transition(StrategyPhase.PAUSED, reason, self._facts)
            return {"accepted": True, **self.status()}

    def resume(self, *, reason: str = "resume_requested", now: float | None = None) -> dict[str, Any]:
        """Resume, but only into a fresh world: a stale scan stays BLOCKED."""

        with self._lock:
            if self._phase not in (StrategyPhase.PAUSED, StrategyPhase.BLOCKED):
                return {"accepted": False, "error": "not_paused", **self.status()}
            mono = time.monotonic() if now is None else float(now)
            if self._phase == StrategyPhase.PAUSED and self._pause_started is not None:
                self._paused_total += max(0.0, mono - self._pause_started)
                self._pause_started = None
            stale = self._staleness(self._effective_scan_age(mono), self._facts.get("odom_age_s"), mono)
            if stale is not None:
                self._blocked_since = mono
                self._proposal = self._zero_proposal("blocked")
                self._transition(StrategyPhase.BLOCKED, stale, self._facts)
                return {"accepted": False, "error": "blocked", **self.status()}
            self._blocked_since = None
            self._transition(StrategyPhase.RUNNING, reason, self._facts)
            if self._backend_started and self._backend is not None:
                self._backend.rebase(now=mono)
            return {"accepted": True, **self.status()}

    def cancel(self, *, reason: str = "cancelled") -> dict[str, Any]:
        with self._lock:
            if self._phase not in LEASE_PHASES:
                return {"accepted": False, "error": "not_active", **self.status()}
            self._proposal = self._zero_proposal("cancelled")
            self._acceptance_result = None
            self._transition(StrategyPhase.CANCELLED, reason, self._facts)
            return {"accepted": True, **self.status()}

    def stop(self, *, reason: str = "stopped") -> dict[str, Any]:
        """Graceful stop: DONE(reason=stopped), still a receipt and zero motion."""

        with self._lock:
            if self._phase not in LEASE_PHASES:
                return {"accepted": False, "error": "not_active", **self.status()}
            facts = dict(self._facts)
            facts["requested_reason"] = str(reason)
            self._proposal = self._zero_proposal("stopped")
            self._transition(StrategyPhase.DONE, "stopped", facts)
            return {"accepted": True, **self.status()}

    def handover(self, *, reason: str = "handover_requested") -> dict[str, Any]:
        """Give the task to a human; the lease is released either way."""

        with self._lock:
            if self._phase not in LEASE_PHASES:
                return {"accepted": False, "error": "not_active", **self.status()}
            self._proposal = self._zero_proposal("handover")
            self._transition(StrategyPhase.HANDOVER, reason, self._facts)
            return {"accepted": True, **self.status()}

    # -- per-tick decision --------------------------------------------------
    def tick(
        self,
        *,
        now: float,
        pose: tuple[float, float, float] | None = None,
        ranges: Sequence[float] = (),
        angle_min: float = 0.0,
        angle_increment: float = 0.0,
        scan_age_s: float | None = None,
        odom_age_s: float | None = None,
        known_cells: int | None = None,
        known_cells_total: int | None = None,
        known_cells_source: str | None = None,
        traveled_m: float | None = None,
        clearance_m: float | None = None,
        grid: GridMap | None = None,
    ) -> dict[str, Any]:
        """Advance the task by one control cycle and return the handover.

        The returned ``{"publish", "linear_mps", "angular_rps"}`` triple is a
        proposal for the executor, not a command: nothing here writes a topic.

        ``grid`` is only needed by strategies that plan against a map (tour); a
        coverage run ignores it, so existing callers stay valid.
        """

        now = float(now)
        with self._lock:
            if self._phase not in LEASE_PHASES:
                self._last_tick_mono = now
                return self._decision(now=now, publish=False, rationale="not_active")
            facts = self._measure(
                now=now,
                pose=pose,
                scan_age_s=scan_age_s,
                odom_age_s=odom_age_s,
                known_cells=known_cells,
                known_cells_total=known_cells_total,
                known_cells_source=known_cells_source,
                traveled_m=traveled_m,
                clearance_m=clearance_m,
            )
            recovery = ((self._coverage_facts or {}).get("navigation") or {}).get("recovery")
            if recovery:
                facts["recovery"] = dict(recovery)
            self._facts = facts
            self._last_tick_mono = now
            if scan_age_s is not None:
                self._last_scan_age = (float(scan_age_s), now)

            # 1. Acceptance envelope. A paused task is not running out of time.
            runtime = self._runtime(now)
            if (
                self._phase != StrategyPhase.PAUSED
                and self._variant is not None
                and runtime > self._variant.acceptance["max_runtime_s"]
            ):
                self._transition(StrategyPhase.TIMEOUT, "runtime_limit", facts)
                return self._decision(now=now, publish=False, rationale="timeout")

            # 2. Sensor freshness: the one thing that turns a run into BLOCKED
            #    without any obstacle at all.
            stale = self._staleness(scan_age_s, odom_age_s, now)
            if self._phase == StrategyPhase.BLOCKED:
                if self._blocked_since is not None and now - self._blocked_since > self.handover_after:
                    self._transition(StrategyPhase.HANDOVER, "blocked_timeout", facts)
                    return self._decision(now=now, publish=False, rationale="handover")
                return self._decision(now=now, publish=False, rationale=str(self._reason))
            if self._phase == StrategyPhase.PAUSED:
                return self._decision(now=now, publish=False, rationale="paused")
            if stale is not None:
                self._blocked_since = now
                self._proposal = self._zero_proposal("blocked")
                self._transition(StrategyPhase.BLOCKED, stale, facts)
                return self._decision(now=now, publish=False, rationale=stale)
            if self._phase == StrategyPhase.STARTING:
                self._transition(StrategyPhase.RUNNING, "first_valid_world", facts)

            # 3. The deterministic strategy proposes; this module only gates it.
            if self._backend is None:
                return self._decision(now=now, publish=False, rationale="no_backend")
            if not self._backend_started:
                # A strategy plan only exists once there is a pose to start from.
                if pose is None:
                    return self._decision(now=now, publish=False, rationale="pose_missing")
                self._backend.start(
                    now=now,
                    request_id=self._request_id or "strategy",
                    source=self._source or "strategy",
                    pose=pose,
                    ranges=ranges,
                    angle_min=angle_min,
                    angle_increment=angle_increment,
                    grid=grid,
                )
                self._backend_started = True
            proposal, backend_facts = self._backend.step(
                now=now,
                pose=pose,
                ranges=ranges,
                angle_min=angle_min,
                angle_increment=angle_increment,
                grid=grid,
            )
            self._coverage_facts = backend_facts
            self._last_proposal = (
                round(float(proposal.linear_mps), 4),
                round(float(proposal.angular_rps), 4),
            )
            backend_phase = str(backend_facts.get("phase") or "").upper()
            self._coverage_phase = backend_phase
            backend_reason = str(backend_facts.get("reason") or "")
            if backend_phase in BACKEND_TERMINAL_PHASES and backend_reason:
                if backend_reason in self._forbidden_states():
                    self._forbidden_seen.append(backend_reason)
            if backend_phase == "DONE":
                self._acceptance_result = self._evaluate_acceptance(
                    coverage_facts=backend_facts, known_cells=known_cells, runtime=runtime
                )
                reason = "reached" if self._acceptance_result["met"] else "exhausted"
                facts = dict(facts)
                facts["acceptance"] = self._acceptance_result
                self._transition(StrategyPhase.DONE, reason, facts)
                return self._decision(now=now, publish=False, rationale=reason)
            if backend_phase == "LIMIT":
                self._transition(StrategyPhase.TIMEOUT, backend_reason or f"{self.strategy}_limit", facts)
                return self._decision(now=now, publish=False, rationale="timeout")
            if backend_phase == "STOPPED":
                self._transition(StrategyPhase.DONE, "stopped", facts)
                return self._decision(now=now, publish=False, rationale="stopped")

            # 4. Clearance gate, judged by the direction of the proposed motion.
            #    A blocked front must not forbid the one escape that is clear, and
            #    a turn must clear the body corner it sweeps, not only the nose.
            front = clearance_m if clearance_m is not None else self._front_clearance(
                ranges, angle_min, angle_increment
            )
            linear = float(proposal.linear_mps)
            angular = float(proposal.angular_rps)

            # Recovery bookkeeping (contract §6): one event per recovery episode,
            # so "trigger -> reverse -> re-observe -> replan" is replayable.
            explorer_state = str((backend_facts.get("navigation") or {}).get("explorer_state") or "")
            recovery = (backend_facts.get("navigation") or {}).get("recovery") or {}
            facts["recovery"] = dict(recovery)
            stage = (recovery.get("id"), recovery.get("stage"))
            if stage != self._recovery_stage and stage[1] is not None:
                self._recovery_stage = stage
                self._note_event("recovery_stage", str(stage[1]), facts)
            if explorer_state == "RECOVER" and not self._recovering:
                self._recovering = True
                self._note_event(
                    "recovery_started",
                    str((backend_facts.get("navigation") or {}).get("explorer_reason") or "reverse"),
                    facts,
                )
            elif explorer_state != "RECOVER" and self._recovering:
                self._recovering = False
                self._note_event("recovery_finished", "failed" if explorer_state == "BLOCKED" else "replanned", facts)

            if explorer_state == "BLOCKED":
                reason = str((backend_facts.get("navigation") or {}).get("explorer_reason") or "obstacle_stuck")
                self._blocked_since = now
                self._proposal = self._zero_proposal(reason)
                self._transition(StrategyPhase.BLOCKED, reason, facts)
                return self._decision(now=now, publish=False, rationale=reason)

            if linear < 0.0:
                # Backing out is gated by the rear, never by the front cone.
                rear = self._rear_clearance(ranges, angle_min, angle_increment)
                if rear < self.min_reverse_clearance or not math.isfinite(rear):
                    facts = dict(facts)
                    facts["rear_clearance_m"] = round(rear, 3) if math.isfinite(rear) else None
                    facts["rear_threshold_m"] = self.min_reverse_clearance
                    self._blocked_since = now
                    self._proposal = self._zero_proposal("rear_blocked")
                    self._transition(StrategyPhase.BLOCKED, f"rear:{rear:.2f}", facts)
                    return self._decision(now=now, publish=False, rationale="rear_blocked")
            elif abs(linear) <= 1e-9 and abs(angular) > 1e-9:
                # Turning in place sweeps the body corner toward the turn side.
                side = self._turn_clearance(ranges, angle_min, angle_increment, angular)
                sweep = min(front, side)
                if (not math.isfinite(side) or front < self.min_clearance_turn
                        or side < max(self.min_clearance_turn, TURN_RANGE_M)):
                    facts = dict(facts)
                    facts["clearance_m"] = round(sweep, 3)
                    facts["clearance_threshold_m"] = self.min_clearance_turn
                    facts["clearance_side_m"] = round(side, 3)
                    self._blocked_since = now
                    self._proposal = self._zero_proposal("obstacle")
                    began = self._try_recovery(now=now, reason=f"turn_blocked:{sweep:.2f}", grid=grid, facts=facts)
                    if began is not None:
                        return began
                    self._transition(StrategyPhase.BLOCKED, f"obstacle:{sweep:.2f}", facts)
                    return self._decision(now=now, publish=False, rationale="obstacle")
            else:
                if front < self.min_clearance_turn:
                    facts = dict(facts)
                    facts["clearance_m"] = round(front, 3)
                    facts["clearance_threshold_m"] = self.min_clearance_turn
                    self._blocked_since = now
                    self._proposal = self._zero_proposal("obstacle")
                    began = self._try_recovery(now=now, reason=f"front_blocked:{front:.2f}", grid=grid, facts=facts)
                    if began is not None:
                        return began
                    self._transition(StrategyPhase.BLOCKED, f"obstacle:{front:.2f}", facts)
                    return self._decision(now=now, publish=False, rationale="obstacle")
                if linear > 0.0 and front < self.min_clearance:
                    linear = 0.0
            linear = max(-self.max_linear, min(self.max_linear, linear))
            angular = max(-self.max_angular, min(self.max_angular, angular))
            publish = abs(linear) > 1e-6 or abs(angular) > 1e-6
            if publish:
                self._refusals = 0
            self._proposal = {
                "publish": publish,
                "linear_mps": round(linear, 4),
                "angular_rps": round(angular, 4),
                "rationale": proposal.rationale,
                "source": proposal.source,
            }
            facts["clearance_m"] = round(front, 3) if math.isfinite(front) else None
            return self._decision(now=now, publish=publish, linear=linear, angular=angular)

    def note_refusal(self, reason: str, *, now: float | None = None) -> dict[str, Any] | None:
        """The executor refused the handed-over proposal (it is the only judge)."""

        with self._lock:
            if self._phase != StrategyPhase.RUNNING:
                return None
            self._refusals += 1
            if self._refusals < self.refusal_limit:
                return None
            mono = time.monotonic() if now is None else float(now)
            facts = dict(self._facts)
            facts["refusal_reason"] = str(reason)
            facts["refusal_streak"] = self._refusals
            self._blocked_since = mono
            self._proposal = self._zero_proposal("refused")
            self._transition(StrategyPhase.BLOCKED, f"executor_refused:{reason}", facts)
            return self.status()

    # -- records ------------------------------------------------------------
    def status(self) -> dict[str, Any]:
        """The fixed shape Half B consumes, plus the contract §2/§6 record."""

        with self._lock:
            phase = self._phase
            busy = phase in LEASE_PHASES
            variant = self._variant
            facts = dict(self._facts)
            facts.setdefault("strategy", self.strategy)
            facts.setdefault("phase", phase)
            return {
                # fixed shape (see the lead's contract for the workbench)
                "state": phase,
                "reason": self._reason,
                "strategy": self.strategy,
                "phase": phase,
                "busy": busy,
                "facts": facts,
                "events_cursor": self._cursor,
                # additive detail: identity, acceptance and the last proposal
                "active": busy,
                "request_id": self._request_id,
                "source": self._source,
                "physical": False,
                "elapsed_s": self._elapsed(),
                "goal_lease": {
                    "holder": "strategy" if busy else "none",
                    "since": self._started_wall,
                    "phases": list(LEASE_PHASES),
                },
                "variant": None if variant is None else variant.as_json(),
                "coverage": self._coverage_summary(),
                "backend": self._coverage_facts,
                "acceptance_result": self._acceptance_result,
                "proposal": dict(self._proposal),
            }

    def events_since(self, cursor: int = 0, *, limit: int = 32) -> list[dict[str, Any]]:
        with self._lock:
            try:
                start = int(cursor)
            except (TypeError, ValueError):
                start = 0
            return [event for event in self._events if event["seq"] > start][: max(1, int(limit))]

    def events(self) -> list[dict[str, Any]]:
        with self._lock:
            return [dict(event) for event in self._events]

    # -- internals ----------------------------------------------------------
    def _transition(self, to: str, reason: str, facts: Mapping[str, Any]) -> dict[str, Any]:
        from_phase = self._phase
        snapshot = dict(facts)
        snapshot["phase"] = to
        snapshot["strategy"] = self.strategy
        self._cursor += 1
        event = {
            "seq": self._cursor,
            "at": round(self.clock(), 3),
            "from": from_phase,
            "to": to,
            "reason": str(reason),
            "facts": snapshot,
        }
        self._events.append(event)
        if len(self._events) > self.max_events:
            del self._events[: len(self._events) - self.max_events]
        self._phase = to
        self._reason = str(reason)
        if to != StrategyPhase.RUNNING:
            self._proposal = self._zero_proposal(str(reason))
        self._facts = snapshot
        return event

    def _try_recovery(self, *, now, reason: str, grid, facts: Mapping[str, Any]) -> dict[str, Any] | None:
        """Hand a gate refusal to the strategy's bounded recovery, or give up.

        The executor refusing a turn must not freeze the task: the backend gets one
        deterministic chance to back out and re-plan. When its budget is spent this
        returns None and the caller transitions to BLOCKED as before.
        """

        backend = self._backend
        attempt = getattr(backend, "force_recovery", None)
        if attempt is None:
            return None
        if not attempt(now=now, reason=reason, grid=grid):
            return None
        self._recovering = True
        self._note_event("recovery_started", reason, facts)
        self._proposal = self._zero_proposal("recovery")
        return self._decision(now=now, publish=False, rationale="recovery")
    def _note_event(self, kind: str, reason: str, facts: Mapping[str, Any]) -> dict[str, Any]:
        """Record a non-transition event with the same monotonic cursor.

        Used for recovery episodes (``recovery_started`` / ``recovery_finished``):
        they change no phase, but "trigger -> reverse -> re-observe -> replan" has
        to be replayable next to the state machine's own transitions.
        """

        snapshot = dict(facts)
        snapshot["phase"] = self._phase
        snapshot["strategy"] = self.strategy
        self._cursor += 1
        event = {
            "seq": self._cursor,
            "at": round(self.clock(), 3),
            "from": self._phase,
            "to": self._phase,
            "kind": str(kind),
            "reason": str(reason),
            "facts": snapshot,
        }
        self._events.append(event)
        if len(self._events) > self.max_events:
            del self._events[: len(self._events) - self.max_events]
        return event

    def _blank_facts(self) -> dict[str, Any]:
        return {
            "strategy": self.strategy,
            "phase": self._phase,
            "pose": None,
            "scan_age_s": None,
            "odom_age_s": None,
            "known_cells": None,
            "known_cells_total": None,
            "known_cells_source": None,
            "traveled_m": 0.0,
            "coverage_phase": None,
            "elapsed_s": 0.0,
            "variant_id": None,
        }

    def _measure(
        self,
        *,
        now: float,
        pose: tuple[float, float, float] | None,
        scan_age_s: float | None,
        odom_age_s: float | None,
        known_cells: int | None,
        known_cells_total: int | None,
        known_cells_source: str | None,
        traveled_m: float | None,
        clearance_m: float | None,
    ) -> dict[str, Any]:
        if traveled_m is not None:
            if self._traveled_start is None:
                self._traveled_start = float(traveled_m)
            run_travel = max(0.0, float(traveled_m) - self._traveled_start)
        else:
            run_travel = 0.0
        return {
            "strategy": self.strategy,
            "phase": self._phase,
            "pose": None if pose is None else [round(float(value), 3) for value in pose],
            "scan_age_s": None if scan_age_s is None else round(float(scan_age_s), 3),
            "odom_age_s": None if odom_age_s is None else round(float(odom_age_s), 3),
            "known_cells": known_cells,
            "known_cells_total": known_cells_total,
            "known_cells_source": known_cells_source,
            "traveled_m": round(run_travel, 3),
            "traveled_total_m": None if traveled_m is None else round(float(traveled_m), 3),
            "elapsed_s": self._elapsed(now),
            "variant_id": None if self._variant is None else self._variant.variant_id,
            "request_id": self._request_id,
            "coverage_phase": self._coverage_phase,
            "clearance_m": None if clearance_m is None else round(float(clearance_m), 3),
        }

    def _staleness(self, scan_age_s: float | None, odom_age_s: float | None, now: float) -> str | None:
        if scan_age_s is None:
            return "scan_missing"
        if float(scan_age_s) > self.scan_timeout:
            return "scan_stale"
        if odom_age_s is not None and float(odom_age_s) > self.odom_timeout:
            return "odom_stale"
        return None

    def _effective_scan_age(self, now: float) -> float | None:
        """Last measured scan age plus the time that passed since we measured it."""

        if self._last_scan_age is None:
            return self._facts.get("scan_age_s")
        age, measured_at = self._last_scan_age
        return age + max(0.0, now - measured_at)

    def _runtime(self, now: float) -> float:
        if self._started_mono is None:
            return 0.0
        paused = self._paused_total
        if self._pause_started is not None:
            paused += max(0.0, now - self._pause_started)
        return max(0.0, now - self._started_mono - paused)

    def _elapsed(self, now: float | None = None) -> float:
        if now is not None:
            mono = float(now)
        elif self._last_tick_mono is not None:
            mono = self._last_tick_mono
        else:
            mono = time.monotonic()
        return round(self._runtime(mono), 1)

    def _forbidden_states(self) -> list[str]:
        if self._variant is None:
            return []
        return [str(item) for item in self._variant.acceptance.get("no_forbidden_state") or []]

    def _evaluate_acceptance(
        self,
        *,
        coverage_facts: Mapping[str, Any],
        known_cells: int | None,
        runtime: float,
    ) -> dict[str, Any]:
        acceptance = {} if self._variant is None else dict(self._variant.acceptance)
        return_section = coverage_facts.get("return") or {}
        home = return_section.get("dist_to_home_m")
        if home is None:
            home = coverage_facts.get("dist_to_home_m")
        if home is None:
            home = return_section.get("dist_to_start_m")
        checks = {
            "return_home": {
                "limit_m": acceptance.get("return_distance_m"),
                "actual_m": home,
                # An undeclared check cannot fail: a tour started without a
                # variant has no return-distance envelope to satisfy.
                "met": acceptance.get("return_distance_m") is None
                or (
                    home is not None
                    and float(home) <= float(acceptance["return_distance_m"])
                ),
            },
            "known_cells": {
                "minimum": acceptance.get("min_known_cells"),
                "actual": known_cells,
                "source": self._facts.get("known_cells_source"),
                "met": acceptance.get("min_known_cells") is None
                or (
                    known_cells is not None
                    and float(known_cells) >= float(acceptance["min_known_cells"])
                ),
            },
            "runtime": {
                "limit_s": acceptance.get("max_runtime_s"),
                "actual_s": round(runtime, 1),
                "met": acceptance.get("max_runtime_s") is None
                or runtime <= float(acceptance["max_runtime_s"]),
            },
            "forbidden_states": {
                "forbidden": self._forbidden_states(),
                "seen": list(self._forbidden_seen),
                "met": not any(item in self._forbidden_seen for item in self._forbidden_states()),
            },
        }
        return {"met": all(check["met"] for check in checks.values()), "checks": checks}

    def _coverage_summary(self) -> dict[str, Any] | None:
        if self._coverage_facts is None:
            return None
        return {
            key: self._coverage_facts.get(key)
            for key in (
                "phase",
                "reason",
                "lane_index",
                "lane_dir",
                "lane_progress_m",
                "lane_length_m",
                "lane_width_m",
                "blocked_count",
                "encoder_path_m",
                "dist_to_home_m",
            )
        }

    def _zero_proposal(self, rationale: str) -> dict[str, Any]:
        return {
            "publish": False,
            "linear_mps": 0.0,
            "angular_rps": 0.0,
            "rationale": rationale,
            "source": self.strategy,
        }

    def _decision(
        self,
        *,
        now: float,
        publish: bool,
        linear: float = 0.0,
        angular: float = 0.0,
        rationale: str = "",
    ) -> dict[str, Any]:
        with self._lock:
            facts = dict(self._facts)
            facts["phase"] = self._phase
            self._record_trace(now, publish=publish, linear=linear, angular=angular, rationale=rationale)
            return {
                "publish": bool(publish),
                "linear_mps": round(float(linear), 4) if publish else 0.0,
                "angular_rps": round(float(angular), 4) if publish else 0.0,
                "phase": self._phase,
                "reason": self._reason,
                "rationale": rationale,
                "time": round(float(now), 3),
                "facts": facts,
            }

    def _record_trace(
        self,
        now: float,
        *,
        publish: bool,
        linear: float,
        angular: float,
        rationale: str,
    ) -> None:
        """One line per control cycle: aim, proposal, and what was handed over.

        Enough to separate the three oscillation causes without guessing:
        a changing ``target_id`` is target switching, a stable target with a
        flipping ``proposal`` sign is a steering loop, and ``proposal`` non-zero
        with ``publish`` false is a veto by the runner's own gates.
        """

        backend = self._coverage_facts or {}
        navigation = backend.get("navigation") or {}
        started = self._started_mono
        self._trace.append(
            {
                "t": None if started is None else round(float(now) - started, 3),
                "phase": self._phase,
                "backend_phase": backend.get("phase"),
                "target_id": navigation.get("target_id"),
                "target": navigation.get("target"),
                "bearing_error_rad": navigation.get("bearing_error_rad"),
                "explorer_state": navigation.get("explorer_state"),
                "recovery": navigation.get("recovery"),
                "proposal": [self._last_proposal[0], self._last_proposal[1]],
                "out": [
                    round(float(linear), 4) if publish else 0.0,
                    round(float(angular), 4) if publish else 0.0,
                ],
                "publish": bool(publish),
                "rationale": rationale,
            }
        )
        if len(self._trace) > self.trace_limit:
            del self._trace[: len(self._trace) - self.trace_limit]

    def trace(self, limit: int = 256) -> list[dict[str, Any]]:
        """The last ``limit`` control cycles (newest last)."""

        with self._lock:
            count = max(1, int(limit))
            return [dict(entry) for entry in self._trace[-count:]]

    @staticmethod
    def _front_clearance(
        ranges: Sequence[float], angle_min: float, angle_increment: float
    ) -> float:
        best = float("inf")
        for index, value in enumerate(ranges):
            try:
                distance = float(value)
            except (TypeError, ValueError):
                continue
            if distance != distance or distance <= 0.0 or distance == float("inf"):
                continue
            angle = angle_min + index * angle_increment
            if abs(_wrap(angle)) <= 0.4363323129985824:  # ±25 deg
                best = min(best, distance)
        return best

    @staticmethod
    def _rear_clearance(
        ranges: Sequence[float], angle_min: float, angle_increment: float
    ) -> float:
        """Minimum range behind the car (|angle| >= 150 deg)."""

        best = float("inf")
        for index, value in enumerate(ranges):
            try:
                distance = float(value)
            except (TypeError, ValueError):
                continue
            if distance != distance or distance <= 0.0 or distance == float("inf"):
                continue
            angle = _wrap(angle_min + index * angle_increment)
            if abs(angle) >= 2.6179938779914944:  # 150 deg
                best = min(best, distance)
        return best

    @staticmethod
    def _turn_clearance(
        ranges: Sequence[float], angle_min: float, angle_increment: float, angular: float
    ) -> float:
        """Range over the half-plane a rotation sweeps into (5..120 deg)."""

        low, high = (
            (0.08726646259971647, 2.0943951023931953)
            if angular > 0
            else (-2.0943951023931953, -0.08726646259971647)
        )
        best = float("inf")
        for index, value in enumerate(ranges):
            try:
                distance = float(value)
            except (TypeError, ValueError):
                continue
            if distance != distance or distance <= 0.0 or distance == float("inf"):
                continue
            angle = _wrap(angle_min + index * angle_increment)
            if low <= angle <= high:
                best = min(best, distance)
        return best
