"""Long-task state machine tests (docs/SIM-CONTRACT.md §6).

Every transition is driven through the public API with injected measurements, so
the assertions are about behaviour, not about private fields.
"""

import json
import math
import tempfile
import unittest
from pathlib import Path

from agent_control.coverage import (
    DEFAULT_VARIANT_ID,
    BareGeometryRejected,
    CoverageController,
    VariantError,
)
from agent_control.protocol import ActionProposal
from agent_control.strategy import (
    LEASE_PHASES,
    REQUIRED_FACT_KEYS,
    TERMINAL_PHASES,
    StrategyPhase,
    StrategyRunner,
)

FULL_TURN = 2.0 * math.pi
VARIANT_SCHEMA = "xuegeros.coverage.variant.v1"


def scan_ranges(front=2.0, other=2.0, points=36):
    angle_min = -math.pi
    increment = FULL_TURN / points
    values = [
        front if abs(angle_min + index * increment) < math.radians(20) else other
        for index in range(points)
    ]
    return values, angle_min, increment


def write_variant(
    root,
    variant_id="room/test/01",
    *,
    lane=(1.0, 0.6, 1),
    acceptance=None,
    family=None,
):
    family = family or variant_id.split("/")[0]
    payload = {
        "schema": VARIANT_SCHEMA,
        "variant_id": variant_id,
        "scene_family": family,
        "world": "sim/xuegeros_sim/worlds/room.world",
        "world_sha256": None,
        "instruction_id": "sweep-room/zh/v1",
        "instruction_text": "把这个房间扫完，最后回到起点。",
        "seed": 7,
        "start_pose": [0.0, 0.0, 0.0],
        "lane": {"length_m": lane[0], "width_m": lane[1], "max_lanes": lane[2]},
        "acceptance": acceptance
        or {
            "min_known_cells": 0,
            "max_runtime_s": 300,
            "return_distance_m": 0.2,
            "no_forbidden_state": ["no_progress"],
        },
    }
    parts = variant_id.split("/")
    path = Path(root) / parts[0] / parts[1] / f"{parts[2]}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")
    return variant_id


class StrategyTestBase(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)
        self.variant = write_variant(self.root)

    def tearDown(self):
        self._tmp.cleanup()

    def runner(self, variant_id=None, **kwargs):
        return StrategyRunner(
            variant_id=variant_id or self.variant,
            variant_root=self.root,
            **kwargs,
        )

    def tick(self, runner, now, pose=(0.2, 0.0, 0.0), front=2.0, **kwargs):
        values, angle_min, increment = scan_ranges(front=front)
        kwargs.setdefault("scan_age_s", 0.05)
        kwargs.setdefault("odom_age_s", 0.03)
        return runner.tick(
            now=now,
            pose=pose,
            ranges=values,
            angle_min=angle_min,
            angle_increment=increment,
            **kwargs,
        )


class StrategyStartTests(StrategyTestBase):
    def test_idle_starting_then_running_on_first_valid_world(self):
        runner = self.runner()
        self.assertEqual(runner.phase, StrategyPhase.IDLE)
        self.assertFalse(runner.holds_goal_lease())

        receipt = runner.start(request_id="cov-1", source="tests", now=0.0)
        self.assertTrue(receipt["accepted"])
        self.assertEqual(runner.phase, StrategyPhase.STARTING)
        self.assertTrue(runner.holds_goal_lease())
        self.assertEqual(receipt["state"], StrategyPhase.STARTING)

        self.tick(runner, now=1.0)
        self.assertEqual(runner.phase, StrategyPhase.RUNNING)
        self.assertEqual(
            [(event["from"], event["to"]) for event in runner.events()],
            [
                (StrategyPhase.IDLE, StrategyPhase.STARTING),
                (StrategyPhase.STARTING, StrategyPhase.RUNNING),
            ],
        )

    def test_second_start_is_refused_while_the_lease_is_held(self):
        runner = self.runner()
        runner.start(now=0.0)
        answer = runner.start(now=0.1)
        self.assertFalse(answer["accepted"])
        self.assertEqual(answer["error"], "strategy_active")

    def test_start_rejects_an_unknown_variant_and_lists_what_exists(self):
        runner = self.runner()
        answer = runner.start(variant_id="room/nope/99", now=0.0)
        self.assertFalse(answer["accepted"])
        self.assertEqual(answer["error"], "unknown_variant")
        self.assertIn(self.variant, answer["available"])
        self.assertEqual(runner.phase, StrategyPhase.IDLE)

    def test_start_rejects_malformed_variant_id(self):
        runner = self.runner()
        answer = runner.start(variant_id="../../etc/passwd", now=0.0)
        self.assertFalse(answer["accepted"])
        self.assertEqual(answer["error"], "invalid_variant_id")

    def test_starting_blocked_when_the_first_world_is_stale(self):
        runner = self.runner()
        runner.start(now=0.0)
        values, angle_min, increment = scan_ranges()
        runner.tick(
            now=1.0,
            pose=(0.2, 0.0, 0.0),
            ranges=values,
            angle_min=angle_min,
            angle_increment=increment,
            scan_age_s=None,
            odom_age_s=0.03,
        )
        self.assertEqual(runner.phase, StrategyPhase.BLOCKED)
        self.assertEqual(runner.events()[-1]["reason"], "scan_missing")


class StrategyPauseResumeTests(StrategyTestBase):
    def running_runner(self):
        runner = self.runner()
        runner.start(now=0.0)
        self.tick(runner, now=1.0)
        self.assertEqual(runner.phase, StrategyPhase.RUNNING)
        return runner

    def test_pause_zeroes_the_proposal_within_one_cycle(self):
        runner = self.running_runner()
        before = self.tick(runner, now=2.0)
        self.assertTrue(before["publish"])
        self.assertGreater(before["linear_mps"], 0.0)

        receipt = runner.pause(now=2.1)
        self.assertTrue(receipt["accepted"])
        self.assertEqual(runner.phase, StrategyPhase.PAUSED)
        self.assertEqual(receipt["proposal"], {
            "publish": False,
            "linear_mps": 0.0,
            "angular_rps": 0.0,
            "rationale": "pause_requested",
            "source": "coverage",
        })

        # The very next control cycle hands the executor zeros.
        after = self.tick(runner, now=2.2)
        self.assertFalse(after["publish"])
        self.assertEqual(after["linear_mps"], 0.0)
        self.assertEqual(after["angular_rps"], 0.0)
        self.assertEqual(runner.phase, StrategyPhase.PAUSED)
        self.assertTrue(runner.holds_goal_lease())

    def test_resume_returns_to_running_with_a_fresh_scan(self):
        runner = self.running_runner()
        runner.pause(now=2.0)
        receipt = runner.resume(now=2.1)
        self.assertTrue(receipt["accepted"])
        self.assertEqual(runner.phase, StrategyPhase.RUNNING)
        self.assertEqual(
            [(event["from"], event["to"]) for event in runner.events()][-2:],
            [(StrategyPhase.RUNNING, StrategyPhase.PAUSED), (StrategyPhase.PAUSED, StrategyPhase.RUNNING)],
        )

    def test_resume_with_a_stale_scan_lands_in_blocked_not_running(self):
        runner = self.running_runner()
        runner.pause(now=2.0)
        # The last measured frame is 0.05 s old at t=1.0; at t=30 it is stale.
        receipt = runner.resume(now=30.0)
        self.assertFalse(receipt["accepted"])
        self.assertEqual(receipt["error"], "blocked")
        self.assertEqual(runner.phase, StrategyPhase.BLOCKED)
        self.assertEqual(runner.events()[-1]["reason"], "scan_stale")

    def test_paused_time_does_not_consume_the_runtime_envelope(self):
        runner = self.runner(
            variant_id=write_variant(
                self.root,
                "room/short/01",
                acceptance={
                    "min_known_cells": 0,
                    "max_runtime_s": 5.0,
                    "return_distance_m": 0.2,
                    "no_forbidden_state": ["no_progress"],
                },
            )
        )
        runner.start(now=0.0)
        self.tick(runner, now=1.0)
        runner.pause(now=1.5)
        for step in range(2, 40):
            self.tick(runner, now=float(step))
        self.assertEqual(runner.phase, StrategyPhase.PAUSED)
        runner.resume(now=40.0)
        self.assertEqual(runner.phase, StrategyPhase.RUNNING)
        # 40 s of wall clock, 1.5 s of work: still inside a 5 s envelope, and the
        # coverage watchdog is rebased so it does not time the pause out either.
        self.tick(runner, now=40.5)
        self.assertEqual(runner.phase, StrategyPhase.RUNNING)

    def test_a_long_pause_does_not_trip_the_coverage_no_progress_watchdog(self):
        runner = self.runner()
        runner.start(now=0.0)
        self.tick(runner, now=1.0)
        runner.pause(now=1.5)
        for step in range(2, 40):
            self.tick(runner, now=float(step))
        runner.resume(now=40.0)
        self.tick(runner, now=40.5)
        self.assertEqual(runner.phase, StrategyPhase.RUNNING)
        self.assertEqual(runner.status()["coverage"]["phase"], "LANE")


class StrategyBlockedTests(StrategyTestBase):
    def running_runner(self):
        runner = self.runner(handover_after_s=5.0)
        runner.start(now=0.0)
        self.tick(runner, now=1.0)
        return runner

    def test_stale_scan_turns_running_into_blocked_and_keeps_the_lease(self):
        runner = self.running_runner()
        self.tick(runner, now=2.0, scan_age_s=3.0)
        self.assertEqual(runner.phase, StrategyPhase.BLOCKED)
        self.assertEqual(runner.events()[-1]["reason"], "scan_stale")
        self.assertTrue(runner.holds_goal_lease())
        self.assertTrue(runner.status()["busy"])

    def test_obstacle_inside_the_turn_floor_blocks_with_clearance_in_facts(self):
        runner = self.running_runner()
        self.tick(runner, now=2.0, clearance_m=0.05)
        self.assertEqual(runner.phase, StrategyPhase.BLOCKED)
        event = runner.events()[-1]
        self.assertEqual(event["reason"], "obstacle:0.05")
        self.assertAlmostEqual(event["facts"]["clearance_m"], 0.05)
        self.assertAlmostEqual(event["facts"]["clearance_threshold_m"], 0.12)

    def test_advance_is_dropped_before_the_task_is_blocked(self):
        runner = self.running_runner()
        proposal = self.tick(runner, now=2.0, clearance_m=0.2)
        self.assertEqual(runner.phase, StrategyPhase.RUNNING)
        self.assertEqual(proposal["linear_mps"], 0.0)

    def test_blocked_resumes_to_running_once_the_scan_is_fresh_again(self):
        runner = self.running_runner()
        self.tick(runner, now=2.0, scan_age_s=3.0)
        self.assertEqual(runner.phase, StrategyPhase.BLOCKED)
        self.tick(runner, now=3.0, scan_age_s=0.05)
        self.assertEqual(runner.phase, StrategyPhase.BLOCKED)
        receipt = runner.resume(now=3.05)
        self.assertTrue(receipt["accepted"])
        self.assertEqual(runner.phase, StrategyPhase.RUNNING)

    def test_blocked_too_long_becomes_handover(self):
        runner = self.running_runner()
        self.tick(runner, now=2.0, scan_age_s=3.0)
        self.assertEqual(runner.phase, StrategyPhase.BLOCKED)
        self.tick(runner, now=8.0, scan_age_s=0.05)
        self.assertEqual(runner.phase, StrategyPhase.HANDOVER)
        self.assertEqual(runner.events()[-1]["reason"], "blocked_timeout")
        self.assertFalse(runner.holds_goal_lease())

    def test_repeated_executor_refusals_block_the_task(self):
        runner = self.running_runner()
        for _ in range(4):
            self.assertIsNone(runner.note_refusal("scan_stale", now=2.0))
        self.assertEqual(runner.phase, StrategyPhase.RUNNING)
        status = runner.note_refusal("scan_stale", now=2.0)
        self.assertIsNotNone(status)
        self.assertEqual(runner.phase, StrategyPhase.BLOCKED)
        self.assertEqual(runner.events()[-1]["reason"], "executor_refused:scan_stale")
        self.assertEqual(runner.events()[-1]["facts"]["refusal_streak"], 5)


class StrategyEnvelopeTests(StrategyTestBase):
    def test_runtime_limit_becomes_timeout(self):
        runner = self.runner(
            variant_id=write_variant(
                self.root,
                "room/tiny/01",
                acceptance={
                    "min_known_cells": 0,
                    "max_runtime_s": 1.0,
                    "return_distance_m": 0.2,
                    "no_forbidden_state": ["no_progress"],
                },
            )
        )
        runner.start(now=0.0)
        self.tick(runner, now=0.5)
        self.assertEqual(runner.phase, StrategyPhase.RUNNING)
        self.tick(runner, now=2.0)
        self.assertEqual(runner.phase, StrategyPhase.TIMEOUT)
        self.assertEqual(runner.events()[-1]["reason"], "runtime_limit")
        self.assertFalse(runner.holds_goal_lease())

    def test_no_progress_from_the_strategy_becomes_timeout(self):
        def factory(variant):
            return CoverageController(
                variant_id=variant.variant_id,
                variant_root=self.root,
                no_progress_timeout_s=1.0,
            )

        runner = self.runner(controller_factory=factory)
        runner.start(now=0.0)
        self.tick(runner, now=1.0, pose=(0.0, 0.0, 0.0))
        self.assertEqual(runner.phase, StrategyPhase.RUNNING)
        self.tick(runner, now=2.5, pose=(0.0, 0.0, 0.0))
        self.assertEqual(runner.phase, StrategyPhase.TIMEOUT)
        self.assertEqual(runner.events()[-1]["reason"], "no_progress")


class StrategyTerminationTests(StrategyTestBase):
    def running_runner(self, **kwargs):
        runner = self.runner(**kwargs)
        runner.start(now=0.0)
        # Start the lane plan at the origin so the lane geometry is arithmetic.
        self.tick(runner, now=1.0, pose=(0.0, 0.0, 0.0))
        self.assertEqual(runner.phase, StrategyPhase.RUNNING)
        return runner

    def test_cancel_lands_in_cancelled_and_releases_the_lease(self):
        runner = self.running_runner()
        receipt = runner.cancel(reason="operator")
        self.assertTrue(receipt["accepted"])
        self.assertEqual(runner.phase, StrategyPhase.CANCELLED)
        self.assertEqual(receipt["proposal"]["linear_mps"], 0.0)
        self.assertFalse(runner.holds_goal_lease())
        self.assertFalse(runner.status()["busy"])

    def test_stop_lands_in_done_with_reason_stopped(self):
        runner = self.running_runner()
        receipt = runner.stop(reason="api")
        self.assertTrue(receipt["accepted"])
        self.assertEqual(runner.phase, StrategyPhase.DONE)
        self.assertEqual(runner.events()[-1]["reason"], "stopped")
        self.assertEqual(runner.events()[-1]["facts"]["requested_reason"], "api")
        self.assertFalse(runner.holds_goal_lease())

    def test_handover_releases_the_lease(self):
        runner = self.running_runner()
        receipt = runner.handover(reason="operator")
        self.assertTrue(receipt["accepted"])
        self.assertEqual(runner.phase, StrategyPhase.HANDOVER)
        self.assertFalse(runner.holds_goal_lease())

    def test_terminal_phase_ignores_further_ticks(self):
        runner = self.running_runner()
        runner.cancel()
        proposal = self.tick(runner, now=30.0)
        self.assertFalse(proposal["publish"])
        self.assertEqual(runner.phase, StrategyPhase.CANCELLED)

    def test_a_new_run_can_start_after_a_terminal_phase(self):
        runner = self.running_runner()
        runner.stop()
        receipt = runner.start(request_id="cov-2", now=40.0)
        self.assertTrue(receipt["accepted"])
        self.assertEqual(runner.phase, StrategyPhase.STARTING)
        self.tick(runner, now=41.0)
        self.assertEqual(runner.phase, StrategyPhase.RUNNING)

    def test_coverage_reaching_home_is_done_reached_when_acceptance_holds(self):
        # max_lanes=1: one lane, one shift, then home -- the real controller's
        # own phase walk, observed through the state machine.
        runner = self.running_runner()
        self.tick(runner, now=2.0, pose=(1.05, 0.0, 0.0))
        self.tick(runner, now=3.0, pose=(1.05, 0.0, math.pi / 2))
        self.tick(runner, now=4.0, pose=(1.05, 0.6, math.pi / 2))
        self.tick(runner, now=5.0, pose=(0.05, 0.0, 0.0), known_cells=10)
        self.assertEqual(runner.phase, StrategyPhase.DONE)
        self.assertEqual(runner.events()[-1]["reason"], "reached")
        acceptance = runner.status()["acceptance_result"]
        self.assertTrue(acceptance["met"])
        self.assertTrue(acceptance["checks"]["return_home"]["met"])
        self.assertEqual(acceptance["checks"]["known_cells"]["actual"], 10)

    def test_coverage_reaching_home_with_a_missed_metric_is_done_exhausted(self):
        runner = self.runner(
            variant_id=write_variant(
                self.root,
                "room/hungry/01",
                acceptance={
                    "min_known_cells": 5000,
                    "max_runtime_s": 300,
                    "return_distance_m": 0.2,
                    "no_forbidden_state": ["no_progress"],
                },
            )
        )
        runner.start(now=0.0)
        self.tick(runner, now=1.0, pose=(0.0, 0.0, 0.0))
        self.tick(runner, now=2.0, pose=(1.05, 0.0, 0.0))
        self.tick(runner, now=3.0, pose=(1.05, 0.0, math.pi / 2))
        self.tick(runner, now=4.0, pose=(1.05, 0.6, math.pi / 2))
        self.tick(runner, now=5.0, pose=(0.05, 0.0, 0.0), known_cells=10)
        self.assertEqual(runner.phase, StrategyPhase.DONE)
        self.assertEqual(runner.events()[-1]["reason"], "exhausted")
        acceptance = runner.status()["acceptance_result"]
        self.assertFalse(acceptance["met"])
        self.assertFalse(acceptance["checks"]["known_cells"]["met"])


class StrategyEventTests(StrategyTestBase):
    def test_every_transition_writes_one_event_with_the_contract_facts(self):
        runner = self.runner(handover_after_s=5.0)
        runner.start(request_id="cov-9", source="tests", now=0.0)
        self.tick(runner, now=1.0)
        runner.pause(now=2.0)
        runner.resume(now=2.5)
        self.tick(runner, now=3.0, scan_age_s=4.0)
        runner.resume(now=3.5)
        runner.cancel(reason="operator")

        events = runner.events()
        self.assertGreaterEqual(len(events), 6)
        seqs = [event["seq"] for event in events]
        self.assertEqual(seqs, list(range(1, len(events) + 1)))
        for event in events:
            self.assertIsInstance(event["at"], float)
            self.assertIn(event["from"], [*LEASE_PHASES, *TERMINAL_PHASES, StrategyPhase.IDLE])
            self.assertIn(event["to"], [*LEASE_PHASES, *TERMINAL_PHASES])
            self.assertTrue(event["reason"])
            for key in REQUIRED_FACT_KEYS:
                self.assertIn(key, event["facts"], f"{key} missing from {event}")
            self.assertEqual(event["facts"]["strategy"], "coverage")
            self.assertEqual(event["facts"]["phase"], event["to"])
        self.assertEqual(runner.status()["events_cursor"], events[-1]["seq"])

    def test_pause_and_resume_events_are_recorded_even_though_nothing_moved(self):
        runner = self.runner()
        runner.start(now=0.0)
        self.tick(runner, now=1.0)
        runner.pause(now=2.0)
        runner.resume(now=2.1)
        tos = [event["to"] for event in runner.events()]
        self.assertEqual(
            tos,
            [StrategyPhase.STARTING, StrategyPhase.RUNNING, StrategyPhase.PAUSED, StrategyPhase.RUNNING],
        )

    def test_events_since_replays_from_a_cursor(self):
        runner = self.runner()
        runner.start(now=0.0)
        self.tick(runner, now=1.0)
        runner.pause(now=2.0)
        cursor = runner.status()["events_cursor"]
        runner.resume(now=2.1)
        new_events = runner.events_since(cursor)
        self.assertEqual([event["to"] for event in new_events], [StrategyPhase.RUNNING])
        self.assertTrue(all(event["seq"] > cursor for event in new_events))


class StrategyFactsTests(StrategyTestBase):
    def test_facts_carry_measured_travel_and_known_cells(self):
        runner = self.runner()
        runner.start(now=0.0)
        first = self.tick(runner, now=1.0, traveled_m=10.0, known_cells=42, known_cells_source="sim_truth_seen/v1")
        self.assertEqual(first["facts"]["traveled_m"], 0.0)
        second = self.tick(runner, now=2.0, traveled_m=10.5, known_cells=43, known_cells_source="sim_truth_seen/v1")
        self.assertAlmostEqual(second["facts"]["traveled_m"], 0.5)
        self.assertEqual(second["facts"]["known_cells"], 43)
        self.assertEqual(second["facts"]["known_cells_source"], "sim_truth_seen/v1")
        self.assertEqual(second["facts"]["variant_id"], self.variant)

    def test_proposal_is_clamped_to_the_executor_limits(self):
        runner = self.runner(max_linear_mps=0.05, max_angular_rps=0.3)
        runner.start(now=0.0)
        # The strategy may ask for more; the runner still clamps at handover.
        runner._backend.controller.max_linear = 0.5
        proposal = self.tick(runner, now=1.0)
        self.assertLessEqual(abs(proposal["linear_mps"]), 0.05)

    def test_status_has_the_fixed_shape_half_b_consumes(self):
        runner = self.runner()
        runner.start(now=0.0)
        status = runner.status()
        for key in ("state", "reason", "strategy", "phase", "busy", "facts", "events_cursor"):
            self.assertIn(key, status)
        self.assertEqual(status["state"], status["phase"])
        self.assertTrue(status["busy"])
        self.assertFalse(status["physical"])
        self.assertEqual(status["goal_lease"]["holder"], "strategy")

    def test_default_variant_of_the_repository_is_loadable(self):
        runner = StrategyRunner()
        self.assertEqual(runner.variant_id, DEFAULT_VARIANT_ID)
        receipt = runner.start(now=0.0)
        self.assertTrue(receipt["accepted"])
        self.assertEqual(receipt["variant"]["variant_id"], DEFAULT_VARIANT_ID)


class VariantRejectionTests(unittest.TestCase):
    def test_bare_lane_geometry_is_rejected_by_the_coverage_layer(self):
        with self.assertRaises(BareGeometryRejected) as caught:
            CoverageController(lane_length_m=1.0)
        self.assertEqual(caught.exception.code, "bare_geometry")
        self.assertIn("room/default/01", caught.exception.available)

    def test_bare_lane_width_alone_is_also_rejected(self):
        with self.assertRaises(VariantError):
            CoverageController(lane_width_m=0.5)

    def test_default_geometry_still_comes_from_the_default_variant(self):
        control = CoverageController()
        self.assertEqual(control.lane_length, 1.0)
        self.assertEqual(control.lane_width, 0.6)
        self.assertEqual(control.max_lanes, 4)


class TourStrategyTests(StrategyTestBase):
    """tour behind the same state machine (contract §1/§6).

    The point of these tests is that the orchestrator does not care which
    strategy runs: the lease, the transitions, the event cursor and the facts
    come out identical, and only the proposal differs.
    """

    @staticmethod
    def grid(radius_free=0.4, radius_unknown=0.6, size=40, resolution=0.05, origin=(-1.0, -1.0)):
        from agent_control.explorer import GridMap

        data = []
        for y in range(size):
            for x in range(size):
                cx = origin[0] + (x + 0.5) * resolution
                cy = origin[1] + (y + 0.5) * resolution
                distance = math.hypot(cx, cy)
                if distance < radius_free:
                    data.append(0)      # seen and free
                elif distance < radius_unknown:
                    data.append(-1)     # unknown ring: something to explore
                else:
                    data.append(100)    # wall
        return GridMap(
            width=size,
            height=size,
            resolution=resolution,
            origin_x=origin[0],
            origin_y=origin[1],
            data=data,
        )

    def tour_runner(self, **kwargs):
        return StrategyRunner(strategy="tour", **kwargs)

    def test_tour_uses_the_same_states_and_facts_as_coverage(self):
        runner = self.tour_runner()
        receipt = runner.start(request_id="tour-1", source="tests", now=0.0)
        self.assertTrue(receipt["accepted"])
        self.assertEqual(runner.phase, StrategyPhase.STARTING)
        self.assertEqual(receipt["strategy"], "tour")

        proposal = self.tick(runner, now=1.0, pose=(0.0, 0.0, 0.0), grid=self.grid())
        self.assertEqual(runner.phase, StrategyPhase.RUNNING)
        self.assertEqual(proposal["phase"], StrategyPhase.RUNNING)
        for key in REQUIRED_FACT_KEYS:
            self.assertIn(key, proposal["facts"])
        self.assertEqual(proposal["facts"]["strategy"], "tour")

        events = runner.events()
        self.assertEqual(
            [(event["from"], event["to"]) for event in events],
            [
                (StrategyPhase.IDLE, StrategyPhase.STARTING),
                (StrategyPhase.STARTING, StrategyPhase.RUNNING),
            ],
        )
        self.assertEqual([event["seq"] for event in events], [1, 2])
        self.assertEqual(runner.status()["strategy"], "tour")

    def test_tour_still_holds_the_single_goal_lease(self):
        runner = self.tour_runner()
        runner.start(now=0.0)
        self.assertTrue(runner.holds_goal_lease())
        second = runner.start(now=0.1)
        self.assertFalse(second["accepted"])
        self.assertEqual(second["error"], "strategy_active")

    def test_tour_blocked_when_the_scan_is_stale(self):
        runner = self.tour_runner()
        runner.start(now=0.0)
        self.tick(runner, now=1.0, pose=(0.0, 0.0, 0.0), grid=self.grid())
        self.tick(runner, now=20.0, pose=(0.0, 0.0, 0.0), scan_age_s=10.0, grid=self.grid())
        self.assertEqual(runner.phase, StrategyPhase.BLOCKED)
        self.assertTrue(runner.holds_goal_lease())

    def test_tour_pause_is_immediate_and_resume_needs_a_fresh_world(self):
        runner = self.tour_runner()
        runner.start(now=0.0)
        self.tick(runner, now=1.0, pose=(0.0, 0.0, 0.0), grid=self.grid())
        paused = runner.pause(now=2.0)
        self.assertTrue(paused["accepted"])
        # A paused task still consumes fresh frames; that stamp is what resume checks.
        self.assertFalse(
            self.tick(runner, now=2.5, pose=(0.0, 0.0, 0.0), grid=self.grid())["publish"]
        )
        resumed = runner.resume(now=3.0)
        self.assertTrue(resumed["accepted"])
        self.assertEqual(runner.phase, StrategyPhase.RUNNING)

    def test_tour_stop_is_done_stopped(self):
        runner = self.tour_runner()
        runner.start(now=0.0)
        self.tick(runner, now=1.0, pose=(0.0, 0.0, 0.0), grid=self.grid())
        answer = runner.stop(reason="operator")
        self.assertTrue(answer["accepted"])
        self.assertEqual(runner.phase, StrategyPhase.DONE)
        self.assertEqual(runner.events()[-1]["reason"], "stopped")
        self.assertFalse(runner.holds_goal_lease())

    def test_tour_terminal_phase_maps_to_the_runner_states(self):
        runner = self.tour_runner()
        runner.start(now=0.0)
        self.tick(runner, now=1.0, pose=(0.0, 0.0, 0.0), grid=self.grid())
        # The backend reports its own terminal phases; the runner owns the verdict.
        runner._backend.controller.phase = "DONE"
        runner._backend.controller.reason = "returned_home"
        self.tick(runner, now=2.0, pose=(0.0, 0.0, 0.0), grid=self.grid())
        self.assertEqual(runner.phase, StrategyPhase.DONE)
        self.assertIn(runner.events()[-1]["reason"], ("reached", "exhausted"))

    def test_unknown_strategy_name_is_refused_by_name(self):
        runner = StrategyRunner()
        answer = runner.start(strategy="greedy", now=0.0)
        self.assertFalse(answer["accepted"])
        self.assertEqual(answer["error"], "unknown_strategy")

    def test_coverage_is_still_the_default(self):
        runner = StrategyRunner(variant_id=self.variant, variant_root=self.root)
        self.assertEqual(runner.strategy, "coverage")
        runner.start(now=0.0)
        self.assertEqual(runner.status()["strategy"], "coverage")


class DirectionAwareGateTests(StrategyTestBase):
    """The executor gate must judge the proposed motion's direction.

    A blocked front used to forbid every motion, including the reverse that is the
    only remaining escape. The rule now is direction-dependent: forward and turns
    against the front/turn-side sweep, reverse against the rear - with the same
    thresholds as before, never looser.
    """

    class _StubController:
        def __init__(self, linear, angular):
            self.proposal = ActionProposal(linear, angular, 250, "stub", "test")
            self.phase = "LANE"
            self.reason = None

        def start(self, **_kwargs):
            return None

        def step(self, **_kwargs):
            return self.proposal, {"phase": self.phase, "reason": self.reason, "navigation": {}}

        def rebase(self, **_kwargs):
            return None

        def stop(self, *, reason="stopped"):
            return {"phase": "STOPPED", "reason": reason}

        def snapshot(self):
            return {"phase": self.phase}

    @staticmethod
    def situation(front=2.0, side=2.0, rear=2.0, points=180):
        angle_min = -math.pi
        increment = FULL_TURN / points
        values = []
        for index in range(points):
            angle = angle_min + index * increment
            magnitude = abs(angle)
            if magnitude <= math.radians(25):
                values.append(front)
            elif magnitude >= math.radians(150):
                values.append(rear)
            else:
                values.append(side)
        return values, angle_min, increment

    def runner_with(self, linear, angular, **kwargs):
        return self.runner(
            controller_factory=lambda _variant: self._StubController(linear, angular),
            **kwargs,
        )

    def tick_situation(self, runner, now, *, front=2.0, side=2.0, rear=2.0):
        values, angle_min, increment = self.situation(front=front, side=side, rear=rear)
        return runner.tick(
            now=now,
            pose=(0.5, 0.0, 0.0),
            ranges=values,
            angle_min=angle_min,
            angle_increment=increment,
            scan_age_s=0.05,
            odom_age_s=0.03,
        )

    def test_reverse_is_allowed_when_the_front_is_blocked_but_the_rear_is_clear(self):
        runner = self.runner_with(-0.03, 0.0)
        runner.start(now=0.0)
        decision = self.tick_situation(runner, 1.0, front=0.05, side=2.0, rear=0.5)
        self.assertTrue(decision["publish"])
        self.assertLess(decision["linear_mps"], 0.0)
        self.assertEqual(runner.phase, StrategyPhase.RUNNING)

    def test_reverse_is_blocked_when_the_rear_is_short(self):
        runner = self.runner_with(-0.03, 0.0)
        runner.start(now=0.0)
        decision = self.tick_situation(runner, 1.0, front=0.05, rear=0.05)
        self.assertFalse(decision["publish"])
        self.assertEqual(runner.phase, StrategyPhase.BLOCKED)
        self.assertTrue(runner.events()[-1]["reason"].startswith("rear:"))

    def test_turning_in_place_is_blocked_when_the_turn_side_sweep_is_short(self):
        # Front is clear, but the side the body rotates into is 0.08 m away.
        runner = self.runner_with(0.0, 0.2)
        runner.start(now=0.0)
        decision = self.tick_situation(runner, 1.0, front=2.0, side=0.08, rear=2.0)
        self.assertFalse(decision["publish"])
        self.assertEqual(runner.phase, StrategyPhase.BLOCKED)
        self.assertTrue(runner.events()[-1]["reason"].startswith("obstacle:"))

    def test_turning_in_place_is_allowed_when_the_sweep_is_clear(self):
        runner = self.runner_with(0.0, 0.2)
        runner.start(now=0.0)
        decision = self.tick_situation(runner, 1.0, front=2.0, side=0.5, rear=2.0)
        self.assertTrue(decision["publish"])
        self.assertGreater(decision["angular_rps"], 0.0)

    def test_forward_is_still_gated_by_the_front(self):
        runner = self.runner_with(0.04, 0.0)
        runner.start(now=0.0)
        decision = self.tick_situation(runner, 1.0, front=0.05, rear=2.0)
        self.assertFalse(decision["publish"])
        self.assertEqual(runner.phase, StrategyPhase.BLOCKED)


if __name__ == "__main__":
    unittest.main()
