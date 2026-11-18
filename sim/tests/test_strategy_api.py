"""HTTP/WS contract tests for the local-sim strategy endpoints.

These exercise the real handler (loopback, ephemeral port), the real
``StrategyDaemon`` and the real ``Cerebellum``: no real-robot port, no ROS, and
``physical`` stays false throughout.
"""

import json
import math
import unittest
import urllib.error
import time
import urllib.request

from agent_control.cerebellum import Cerebellum
from agent_control.strategy import StrategyPhase, StrategyRunner
from sim.local_sim import (
    SIM_ASSETS,
    LocalSimulator,
    Scene,
    StrategyDaemon,
    model_metadata,
    start_http,
)

FIXED_KEYS = ("state", "reason", "strategy", "phase", "busy", "facts", "events_cursor")
FORBIDDEN_PORTS = (18142, 18242, 18202)


class StrategyApiTestBase(unittest.TestCase):
    def setUp(self):
        self.control = Cerebellum()
        self.strategy = StrategyRunner(
            obstacle_stop_m=self.control.min_clearance,
            scan_timeout_s=self.control.scan_timeout,
            odom_timeout_s=self.control.odom_timeout,
            min_clearance_m=self.control.min_clearance,
            min_clearance_turn_m=self.control.min_clearance_turn,
        )
        self.daemon = StrategyDaemon(self.control, physical=False, strategy=self.strategy)
        self.scene = Scene.from_sdf(SIM_ASSETS / "worlds" / "room.world")
        self.sim = LocalSimulator(self.daemon, self.scene, hz=50.0)
        self.sim.select_scene(self.scene)
        self.http = start_http(self.sim, 0, model_metadata())
        self.port = self.http.server_address[1]
        self.base = f"http://127.0.0.1:{self.port}"

    def tearDown(self):
        self.http.shutdown()
        self.http.server_close()

    # -- helpers ------------------------------------------------------------
    def get(self, path):
        with urllib.request.urlopen(self.base + path, timeout=10) as response:
            return response.status, json.loads(response.read())

    def post(self, path, body=None):
        data = json.dumps(body or {}).encode()
        request = urllib.request.Request(
            self.base + path, data=data, headers={"content-type": "application/json"}
        )
        try:
            with urllib.request.urlopen(request, timeout=10) as response:
                return response.status, json.loads(response.read())
        except urllib.error.HTTPError as exc:
            return exc.code, json.loads(exc.read())

    def tick(self, now=None, pose=None, dt=0.02, integrate=False):
        """One control cycle exactly like ``LocalSimulator.run`` does it."""

        now = time.monotonic() if now is None else now
        pose = self.sim.pose if pose is None else pose
        ranges = self.scene.scan(pose)
        increment = 2 * math.pi / len(ranges)
        if self.daemon.strategy_busy:
            self.sim.mark_seen(ranges, -math.pi, increment)
        profile = None
        from agent_control.cerebellum import profile_scan

        profile = profile_scan(ranges, -math.pi, increment)
        decision = self.daemon.tick_with_strategy(
            now=now,
            profile=profile,
            pose=pose,
            ranges=ranges,
            angle_min=-math.pi,
            angle_increment=increment,
            scan_age_s=0.05,
            odom_age_s=0.03,
            known_cells=self.sim.known_cells,
            known_cells_total=self.sim.truth.get("known_cells"),
            known_cells_source=self.sim.known_cell_source(),
            traveled_m=self.sim.traveled_m,
        )
        if integrate:
            linear = float(decision.linear_mps) if decision.publish else 0.0
            angular = float(decision.angular_rps) if decision.publish else 0.0
            x, y, yaw = self.sim.pose
            next_x = x + linear * math.cos(yaw) * dt
            next_y = y + linear * math.sin(yaw) * dt
            if not self.scene.collides(next_x, next_y):
                self.sim.traveled_m += math.hypot(next_x - x, next_y - y)
                x, y = next_x, next_y
            yaw = (yaw + angular * dt + math.pi) % (2 * math.pi) - math.pi
            self.sim.pose = (x, y, yaw)
        return decision


class StrategyApiReadTests(StrategyApiTestBase):
    def test_the_test_server_never_binds_a_shared_or_physical_port(self):
        self.assertNotIn(self.port, FORBIDDEN_PORTS)
        self.assertEqual(self.http.server_address[0], "127.0.0.1")

    def test_get_strategy_returns_the_fixed_shape_while_idle(self):
        status, payload = self.get("/api/strategy")
        self.assertEqual(status, 200)
        for key in FIXED_KEYS:
            self.assertIn(key, payload)
        self.assertEqual(payload["state"], StrategyPhase.IDLE)
        self.assertEqual(payload["phase"], StrategyPhase.IDLE)
        self.assertFalse(payload["busy"])
        self.assertFalse(payload["physical"])
        self.assertGreaterEqual(len(payload["variants"]), 4)
        ids = [item["variant_id"] for item in payload["variants"]]
        self.assertIn("room/default/01", ids)
        self.assertEqual(payload["events_cursor"], 0)


class StrategyApiLifecycleTests(StrategyApiTestBase):
    def test_start_pause_resume_stop_round_trip(self):
        status, payload = self.post("/api/strategy/start", {"variant_id": "room/default/01"})
        self.assertEqual(status, 200)
        self.assertTrue(payload["ok"])
        self.assertEqual(payload["state"], StrategyPhase.STARTING)
        self.assertTrue(payload["busy"])
        self.assertEqual(payload["facts"]["variant_id"], "room/default/01")

        self.tick()
        _, payload = self.get("/api/strategy")
        self.assertEqual(payload["phase"], StrategyPhase.RUNNING)
        self.assertEqual(payload["coverage"]["phase"], "LANE")

        status, payload = self.post("/api/strategy/pause")
        self.assertEqual(status, 200)
        self.assertEqual(payload["state"], StrategyPhase.PAUSED)
        self.assertEqual(payload["proposal"]["linear_mps"], 0.0)

        status, payload = self.post("/api/strategy/resume")
        self.assertEqual(status, 200)
        self.assertEqual(payload["state"], StrategyPhase.RUNNING)

        status, payload = self.post("/api/strategy/stop")
        self.assertEqual(status, 200)
        self.assertEqual(payload["state"], StrategyPhase.DONE)
        self.assertEqual(payload["reason"], "stopped")
        self.assertFalse(payload["busy"])

    def test_the_strategy_actually_drives_the_car_through_the_executor(self):
        self.post("/api/strategy/start", {"variant_id": "room/default/01"})
        now = 0.0
        published = 0
        for _ in range(60):
            now += 0.02
            decision = self.tick(now, integrate=True)
            if decision.publish:
                published += 1
        self.assertGreater(published, 0)
        self.assertGreater(self.sim.traveled_m, 0.01)
        self.assertEqual(self.strategy.phase, StrategyPhase.RUNNING)
        _status, payload = self.get("/api/strategy")
        self.assertGreater(payload["facts"]["traveled_m"], 0.0)
        self.assertIsNotNone(payload["facts"]["pose"])
        self.assertGreater(payload["facts"]["known_cells"], 0)
        self.assertEqual(payload["facts"]["known_cells_total"], 120 * 120)
        self.assertEqual(payload["facts"]["known_cells_source"], "sim_truth_seen/v1")


class StrategyApiExclusionTests(StrategyApiTestBase):
    def test_single_step_goals_answer_busy_while_the_strategy_holds_the_lease(self):
        self.post("/api/strategy/start", {"variant_id": "room/default/01"})
        self.tick()
        for kind, message in (
            ("goal", {"t": "goal", "goal": "forward"}),
            ("propose", {"t": "propose", "goal": "forward"}),
            ("execute", {"t": "execute", "proposal_id": "prop-0"}),
        ):
            with self.subTest(kind=kind):
                answer = self.daemon.command(message)
                self.assertFalse(answer.get("accepted"))
                self.assertTrue(answer.get("busy"))
                self.assertEqual(answer.get("error"), "busy")
                self.assertEqual(answer.get("reason"), "strategy_active")

    def test_goals_are_allowed_again_after_the_lease_is_released(self):
        self.post("/api/strategy/start", {"variant_id": "room/default/01"})
        self.tick()
        self.post("/api/strategy/cancel")
        self.assertFalse(self.strategy.holds_goal_lease())
        self.daemon.set_scan(-math.pi, 2 * math.pi / 360, [2.0] * 360, age_s=0.05)
        answer = self.daemon.command({"t": "goal", "goal": "stop"})
        self.assertNotEqual(answer.get("error"), "busy")
        self.assertFalse(answer.get("busy"))

    def test_a_second_start_is_a_conflict_not_a_second_task(self):
        self.post("/api/strategy/start", {"variant_id": "room/default/01"})
        status, payload = self.post("/api/strategy/start", {"variant_id": "room/default/01"})
        self.assertEqual(status, 409)
        self.assertFalse(payload["ok"])
        self.assertEqual(payload["error"], "strategy_active")


class StrategyApiRejectionTests(StrategyApiTestBase):
    def test_bare_lane_geometry_is_rejected_with_the_variant_list(self):
        status, payload = self.post(
            "/api/strategy/start", {"variant_id": "room/default/01", "lane_length_m": 0.8}
        )
        self.assertEqual(status, 400)
        self.assertEqual(payload["error"], "bare_geometry")
        self.assertIn("room/default/01", payload["available_variants"])
        self.assertFalse(payload["busy"])

    def test_lane_width_and_max_lanes_are_rejected_too(self):
        for key, value in (("lane_width_m", 0.4), ("max_lanes", 3)):
            with self.subTest(key=key):
                status, payload = self.post("/api/strategy/start", {key: value})
                self.assertEqual(status, 400)
                self.assertEqual(payload["error"], "bare_geometry")

    def test_unknown_variant_is_rejected_with_the_variant_list(self):
        status, payload = self.post("/api/strategy/start", {"variant_id": "room/ghost/99"})
        self.assertEqual(status, 400)
        self.assertEqual(payload["error"], "unknown_variant")
        self.assertIn("room/default/01", payload["available_variants"])

    def test_unknown_action_is_not_a_silent_noop(self):
        status, payload = self.post("/api/strategy/teleport")
        self.assertEqual(status, 404)
        self.assertEqual(payload["error"], "unknown_action")

    def test_pause_before_a_run_is_a_conflict(self):
        status, payload = self.post("/api/strategy/pause")
        self.assertEqual(status, 409)
        self.assertEqual(payload["error"], "not_running")


class StrategyApiReplayTests(StrategyApiTestBase):
    def test_events_replay_from_a_cursor(self):
        self.post("/api/strategy/start", {"variant_id": "room/default/01"})
        self.tick()
        _status, payload = self.get("/api/strategy")
        cursor = payload["events_cursor"]
        self.assertGreaterEqual(cursor, 2)

        self.post("/api/strategy/pause")
        _status, payload = self.get(f"/api/strategy?since={cursor}")
        self.assertEqual([event["to"] for event in payload["events"]], [StrategyPhase.PAUSED])
        for event in payload["events"]:
            self.assertGreater(event["seq"], cursor)
            for key in ("at", "from", "to", "reason", "facts"):
                self.assertIn(key, event)
            for key in ("pose", "scan_age_s", "known_cells", "traveled_m", "strategy", "phase"):
                self.assertIn(key, event["facts"])

    def test_state_endpoint_still_serves_the_workbench(self):
        self.post("/api/strategy/start", {"variant_id": "room/default/01"})
        self.tick()
        status, payload = self.get("/api/state")
        self.assertEqual(status, 200)
        self.assertFalse(payload["physical"])
        self.assertIn("tour", payload["state"])
        self.assertEqual(payload["state"]["tour"]["phase"], StrategyPhase.RUNNING)


if __name__ == "__main__":
    unittest.main()
