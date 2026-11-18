import math
import unittest
from unittest.mock import mock_open, patch
from types import SimpleNamespace

from agent_control.explorer import FrontierExplorer, GridMap
from agent_control.policy import SafetyPolicy, physical_output_is_enabled
from agent_control.protocol import ActionProposal, WorldSnapshot
from agent_control.preflight import preflight
from agent_control.ros_preflight import evaluate_evidence, normalize_tf_age
from agent_control.runtime import mapping_controller_running


def healthy_world() -> WorldSnapshot:
    return WorldSnapshot(
        scan_age_s=0.05,
        odom_age_s=0.05,
        tf_age_s=0.05,
        map_age_s=0.05,
        map_present=True,
        mapping_healthy=True,
        battery_voltage=8.1,
        battery_age_s=0.05,
        micro_ros_session=True,
    )


class CoreTests(unittest.TestCase):
    def test_physical_output_requires_both_explicit_flags(self):
        self.assertFalse(physical_output_is_enabled(False, False))
        self.assertFalse(physical_output_is_enabled(True, False))
        self.assertFalse(physical_output_is_enabled(False, True))
        self.assertTrue(physical_output_is_enabled(True, True))

    def test_mapping_flow_refuses_an_existing_controller_process(self):
        with patch(
            "agent_control.runtime.subprocess.run",
            return_value=SimpleNamespace(
                stdout="123 /usr/bin/python3 /home/chuyun/agent_control/lib/agent_control/mapping_controller\n"
                "124 /usr/bin/python3 /home/chuyun/xuegeros_agent_flow.py mapping\n"
            ),
        ):
            with patch("builtins.open", mock_open(read_data=b"ROS_DOMAIN_ID=0\0")):
                self.assertTrue(mapping_controller_running("0"))

        with patch(
            "agent_control.runtime.subprocess.run",
            return_value=SimpleNamespace(stdout="125 /usr/bin/python3 ros2 topic echo /scan\n"),
        ):
            self.assertFalse(mapping_controller_running("0"))

        with patch(
            "agent_control.runtime.subprocess.run",
            return_value=SimpleNamespace(
                stdout="126 /usr/bin/python3 /home/chuyun/agent_control/lib/agent_control/mapping_controller\n"
            ),
        ), patch(
            "builtins.open",
            mock_open(read_data=b"ROS_DOMAIN_ID=42\0"),
        ):
            self.assertFalse(mapping_controller_running("0"))
            self.assertTrue(mapping_controller_running("42"))

    def test_preflight_missing_upstreams_is_not_ready(self):
        result = preflight(None, None)
        self.assertFalse(result["ok"])
        self.assertIn("world_url_missing", result["blockers"])
        self.assertIn("gateway_url_missing", result["blockers"])

    def test_ros_preflight_requires_single_live_sensor_graph(self):
        evidence = {
            "scan_publishers": 0,
            "odom_publishers": 1,
            "map_publishers": 1,
            "scan_valid_ranges": 0,
            "scan_age_s": None,
            "odom_age_s": 0.1,
            "map_age_s": 0.1,
            "tf_age_s": 0.1,
            "map_to_base_tf": True,
        }
        result = evaluate_evidence(evidence)
        self.assertFalse(result["ok"])
        self.assertIn("scan_publisher_count_0", result["blockers"])
        self.assertIn("scan_invalid_or_missing", result["blockers"])

    def test_ros_preflight_rejects_duplicate_scan_source(self):
        evidence = {
            "scan_publishers": 2,
            "odom_publishers": 1,
            "map_publishers": 1,
            "scan_valid_ranges": 360,
            "scan_age_s": 0.2,
            "odom_age_s": 0.2,
            "map_age_s": 0.2,
            "tf_age_s": 0.2,
            "map_to_base_tf": True,
        }
        result = evaluate_evidence(evidence)
        self.assertFalse(result["ok"])
        self.assertEqual(result["blockers"], ["scan_publisher_count_2"])

    def test_ros_preflight_allows_gmapping_tf_future_stamp_only_within_100ms(self):
        self.assertEqual(normalize_tf_age(-0.05), 0.0)
        self.assertEqual(normalize_tf_age(-0.2), -0.2)

    def test_policy_rejects_nonfinite(self):
        proposal = ActionProposal(math.nan, 0.0, 200)
        decision = SafetyPolicy(physical_enabled=True).evaluate(proposal, healthy_world())
        self.assertFalse(decision.allowed)
        self.assertIn("velocity_not_finite", decision.reasons)

    def test_policy_shadow_rejects_nonzero(self):
        proposal = ActionProposal(0.02, 0.0, 200)
        decision = SafetyPolicy(physical_enabled=False).evaluate(proposal, healthy_world())
        self.assertFalse(decision.allowed)
        self.assertIn("physical_motion_disabled", decision.reasons)

    def test_world_parser_does_not_invent_odom_freshness(self):
        world = WorldSnapshot.from_payload(
            {
                "observed_at": __import__("time").time(),
                "status": {"micro_ros": {"session_observed": True}},
                "mapping": {
                    "scan": {"present": True},
                    "map": {"present": True},
                    "tf": {"present": True},
                    "rates": {"scan_observed": True},
                },
            }
        )
        self.assertIsNone(world.odom_age_s)

    def test_world_parser_rejects_scan_presence_without_rate(self):
        world = WorldSnapshot.from_payload(
            {
                "observed_at": __import__("time").time(),
                "status": {"micro_ros": {"session_observed": True}},
                "mapping": {"scan": {"present": True}, "map": {"present": True}},
            }
        )
        self.assertIsNone(world.scan_age_s)

    def test_world_parser_does_not_infer_freshness_from_presence_or_rate(self):
        world = WorldSnapshot.from_payload(
            {
                "observed_at": __import__("time").time(),
                "mapping": {
                    "scan": {"present": True},
                    "odom": {"present": True},
                    "tf": {"present": True, "map_to_odom": True},
                    "map": {"present": True},
                    "rates": {"scan_observed": True},
                    "mapping_healthy": True,
                },
            }
        )
        self.assertIsNone(world.scan_age_s)
        self.assertIsNone(world.odom_age_s)
        self.assertIsNone(world.tf_age_s)
        self.assertIsNone(world.map_age_s)

    def test_policy_uses_per_sensor_age_limits(self):
        world = WorldSnapshot(
            scan_age_s=2.0,
            odom_age_s=1.1,
            tf_age_s=0.2,
            map_age_s=0.1,
            map_present=True,
            mapping_healthy=True,
            battery_voltage=8.0,
            micro_ros_session=True,
        )
        decision = SafetyPolicy(
            physical_enabled=True,
            require_map_freshness=True,
            max_scan_age_s=2.5,
            max_odom_age_s=1.0,
            max_tf_age_s=1.0,
        ).evaluate(ActionProposal(0.02, 0.0, 200), world)
        self.assertFalse(decision.allowed)
        self.assertIn("odom_stale", decision.reasons)
        self.assertNotIn("scan_stale", decision.reasons)

    def test_policy_rejects_unhealthy_mapping_even_if_map_exists(self):
        world = healthy_world()
        world = WorldSnapshot(**{**world.__dict__, "mapping_healthy": False})
        decision = SafetyPolicy(physical_enabled=True).evaluate(ActionProposal(0.02, 0.0, 200), world)
        self.assertFalse(decision.allowed)
        self.assertIn("mapping_unhealthy", decision.reasons)

    def test_policy_rejects_future_sensor_timestamp(self):
        world = WorldSnapshot(**{**healthy_world().__dict__, "tf_age_s": -0.1})
        decision = SafetyPolicy(physical_enabled=True).evaluate(ActionProposal(0.02, 0.0, 200), world)
        self.assertFalse(decision.allowed)
        self.assertIn("tf_freshness_invalid", decision.reasons)

    def test_world_parser_preserves_invalid_negative_source_age(self):
        world = WorldSnapshot.from_payload({"mapping": {"scan": {"age_s": -0.2}}})
        self.assertEqual(world.scan_age_s, -0.2)

    def test_policy_clamps_healthy_action(self):
        proposal = ActionProposal(0.2, 0.8, 200)
        decision = SafetyPolicy(physical_enabled=True).evaluate(proposal, healthy_world())
        self.assertTrue(decision.allowed)
        self.assertEqual(decision.clamped.linear_mps, 0.05)
        self.assertEqual(decision.clamped.angular_rps, 0.35)

    def test_physical_policy_rejects_unknown_battery(self):
        world = WorldSnapshot(scan_age_s=0, odom_age_s=0, tf_age_s=0, map_present=True, micro_ros_session=True)
        decision = SafetyPolicy(physical_enabled=True, require_battery=True).evaluate(ActionProposal(0.02, 0.0, 200), world)
        self.assertFalse(decision.allowed)
        self.assertIn("battery_unknown", decision.reasons)

    def test_frontier_controller_stops_on_obstacle(self):
        data = [0] * 100
        data[55] = -1
        grid = GridMap(10, 10, 0.1, 0.0, 0.0, data)
        explorer = FrontierExplorer()
        result = explorer.decide(grid, (0.25, 0.25, 0.0), [0.2] * 20, -1.0, 0.1, now=1.0)
        self.assertEqual(result.state, "AVOID")
        self.assertEqual(result.proposal.linear_mps, 0.0)

    def test_frontier_controller_blocks_empty_scan(self):
        data = [0] * 100
        data[55] = -1
        grid = GridMap(10, 10, 0.1, 0.0, 0.0, data)
        result = FrontierExplorer().decide(grid, (0.25, 0.25, 0.0), [], -1.0, 0.1, now=1.0)
        self.assertEqual(result.state, "BLOCKED")
        self.assertEqual(result.proposal.linear_mps, 0.0)

    def test_frontier_controller_finds_target(self):
        data = [0] * 100
        data[55] = -1
        grid = GridMap(10, 10, 0.1, 0.0, 0.0, data)
        explorer = FrontierExplorer()
        result = explorer.decide(grid, (0.25, 0.25, 0.0), [2.0] * 20, -1.0, 0.1, now=1.0)
        self.assertIn(result.state, {"MOVE", "REPLAN"})


if __name__ == "__main__":
    unittest.main()
