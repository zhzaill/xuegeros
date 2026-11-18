import json
import math
import os
import tempfile
import unittest

from agent_control import skills
from agent_control.cerebellum import (
    ACTIVE,
    BLOCKED,
    DONE,
    FORWARD,
    LEFT,
    OPEN_SPACE,
    RIGHT,
    STOP,
    VETO,
    Cerebellum,
    normalize_goal,
    parse_goal,
    profile_scan,
)

FULL_TURN = 2.0 * math.pi


def scan(front=None, left=None, right=None, back=2.0, points=360):
    """Build a synthetic 360-degree scan with per-quadrant ranges."""

    angle_min = -math.pi
    increment = FULL_TURN / points
    ranges = []
    for index in range(points):
        angle = angle_min + index * increment
        if abs(angle) <= math.radians(25) and front is not None:
            value = front
        elif math.radians(25) < angle <= math.pi / 2 and left is not None:
            value = left
        elif -math.pi / 2 <= angle < -math.radians(25) and right is not None:
            value = right
        else:
            value = back
        ranges.append(value)
    return ranges, angle_min, increment


def profile_for(front=None, left=None, right=None, back=2.0):
    ranges, angle_min, increment = scan(front, left, right, back)
    return profile_scan(ranges, angle_min, increment)


def pose(x=0.0, y=0.0, yaw=0.0):
    return (x, y, yaw)


class GoalParsingTests(unittest.TestCase):
    def test_accepts_english_and_chinese_goal_words(self):
        self.assertEqual(normalize_goal("forward"), FORWARD)
        self.assertEqual(normalize_goal("前进一小段"), FORWARD)
        self.assertEqual(normalize_goal("左转"), LEFT)
        self.assertEqual(normalize_goal("右转"), RIGHT)
        self.assertEqual(normalize_goal("停"), STOP)
        self.assertEqual(normalize_goal("去前方空旷处"), OPEN_SPACE)
        self.assertEqual(normalize_goal("  Turn_Left。"), LEFT)
        self.assertIsNone(normalize_goal("fly"))

    def test_parse_goal_bounds_overrides(self):
        self.assertEqual(parse_goal({"goal": "forward"}).kind, FORWARD)
        self.assertEqual(parse_goal({"goal": "forward", "duration_ms": 1500}).timeout_s, 1.5)
        self.assertEqual(parse_goal({"goal": "forward", "distance_m": 0.3}).distance_m, 0.3)
        self.assertEqual(parse_goal({"goal": "forward", "duration_ms": 10}).error, "duration_out_of_range")
        self.assertEqual(parse_goal({"goal": "forward", "distance_m": 5}).error, "distance_out_of_range")
        self.assertEqual(parse_goal({"goal": "nope"}).error, "unknown_goal")
        self.assertEqual(parse_goal({}).error, "unknown_goal")


class ScanProfileTests(unittest.TestCase):
    def test_front_left_right_minima(self):
        result = profile_for(front=0.4, left=0.9, right=0.6)
        self.assertAlmostEqual(result.front_min, 0.4, places=3)
        self.assertAlmostEqual(result.left_min, 0.9, places=3)
        self.assertAlmostEqual(result.right_min, 0.6, places=3)
        self.assertGreater(result.valid_count, 300)

    def test_invalid_and_out_of_range_returns_are_ignored(self):
        ranges = [float("nan"), float("inf"), -1.0, 0.0, 0.01, 20.0] + [1.0] * 360
        result = profile_scan(ranges, -math.pi, FULL_TURN / 366)
        self.assertEqual(result.valid_count, 360)
        self.assertAlmostEqual(result.front_min, 1.0, places=3)

    def test_empty_scan_reports_zero_valid_samples(self):
        result = profile_scan([], -math.pi, 0.01)
        self.assertEqual(result.valid_count, 0)
        self.assertTrue(math.isinf(result.front_min))
        self.assertEqual(result.blind_fraction, 1.0)

    def test_best_sector_prefers_clearance_then_smallest_angle(self):
        result = profile_for(front=0.6, left=0.3, right=2.0)
        index, angle, clearance = result.best_sector(math.pi / 2)
        self.assertAlmostEqual(clearance, 2.0, places=3)
        self.assertLess(angle, 0.0)
        # Sectors 3 and 4 both see the free right side; the tie resolves toward
        # the centre of sector 4 (-45 deg) because it is closer to straight ahead.
        self.assertEqual(index, 4)
        self.assertAlmostEqual(math.degrees(angle), -45.0, places=3)


class ForwardGoalTests(unittest.TestCase):
    def test_forward_drives_then_stops_after_distance(self):
        brain = Cerebellum(forward_distance_m=0.1, max_linear_mps=0.05)
        brain.set_goal({"goal": "forward"}, now=0.0)
        profile = profile_for(front=2.0)

        decision = brain.tick(now=0.1, profile=profile, pose=pose(), scan_age_s=0.1, odom_age_s=0.1)
        self.assertEqual(decision.state, ACTIVE)
        self.assertAlmostEqual(decision.linear_mps, 0.05)
        self.assertEqual(decision.angular_rps, 0.0)

        decision = brain.tick(now=0.2, profile=profile, pose=pose(x=0.06), scan_age_s=0.1, odom_age_s=0.1)
        self.assertEqual(decision.state, ACTIVE)

        decision = brain.tick(now=0.3, profile=profile, pose=pose(x=0.12), scan_age_s=0.1, odom_age_s=0.1)
        self.assertEqual(decision.state, DONE)
        self.assertEqual(decision.reason, "goal_reached")
        self.assertEqual(decision.linear_mps, 0.0)

    def test_forward_times_out(self):
        brain = Cerebellum(forward_distance_m=5.0, forward_timeout_s=1.0)
        brain.set_goal({"goal": "forward"}, now=0.0)
        decision = brain.tick(
            now=1.1, profile=profile_for(front=2.0), pose=pose(), scan_age_s=0.1, odom_age_s=0.1
        )
        self.assertEqual(decision.state, DONE)
        self.assertEqual(decision.reason, "timeout")

    def test_veto_cancels_goal_and_returns_zero(self):
        brain = Cerebellum(min_clearance_m=0.25)
        brain.set_goal({"goal": "forward"}, now=0.0)
        decision = brain.tick(
            now=0.1, profile=profile_for(front=0.2), pose=pose(), scan_age_s=0.1, odom_age_s=0.1
        )
        self.assertEqual(decision.state, VETO)
        self.assertEqual(decision.linear_mps, 0.0)
        self.assertTrue(decision.reason.startswith("veto:obstacle"))
        self.assertIsNone(brain.goal)

    def test_stale_scan_blocks(self):
        brain = Cerebellum(scan_timeout_s=1.0)
        brain.set_goal({"goal": "forward"}, now=0.0)
        decision = brain.tick(
            now=0.1, profile=profile_for(front=2.0), pose=pose(), scan_age_s=2.0, odom_age_s=0.1
        )
        self.assertEqual(decision.state, BLOCKED)
        self.assertEqual(decision.reason, "scan_stale")
        self.assertEqual(decision.linear_mps, 0.0)

    def test_stale_odom_blocks(self):
        brain = Cerebellum(odom_timeout_s=0.5)
        brain.set_goal({"goal": "forward"}, now=0.0)
        decision = brain.tick(
            now=0.1, profile=profile_for(front=2.0), pose=pose(), scan_age_s=0.1, odom_age_s=0.9
        )
        self.assertEqual(decision.state, BLOCKED)
        self.assertEqual(decision.reason, "odom_stale")

    def test_fresh_but_empty_scan_blocks(self):
        brain = Cerebellum(min_valid_samples=8)
        brain.set_goal({"goal": "forward"}, now=0.0)
        decision = brain.tick(
            now=0.1, profile=profile_scan([], -math.pi, 0.01), pose=pose(),
            scan_age_s=0.05, odom_age_s=0.05,
        )
        self.assertEqual(decision.state, BLOCKED)
        self.assertEqual(decision.reason, "scan_insufficient")


class TurnGoalTests(unittest.TestCase):
    def test_left_turn_uses_positive_yaw_rate_and_stops_at_angle(self):
        brain = Cerebellum(turn_rad=math.radians(30.0), max_angular_rps=0.35, turn_timeout_s=10.0)
        brain.set_goal({"goal": "left"}, now=0.0)
        profile = profile_for(front=2.0, left=2.0, right=2.0)

        decision = brain.tick(now=0.1, profile=profile, pose=pose(), scan_age_s=0.1, odom_age_s=0.1)
        self.assertEqual(decision.state, ACTIVE)
        self.assertGreater(decision.angular_rps, 0.0)
        self.assertEqual(decision.linear_mps, 0.0)

        decision = brain.tick(
            now=0.2, profile=profile, pose=pose(yaw=math.radians(20)),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        self.assertEqual(decision.state, ACTIVE)

        decision = brain.tick(
            now=0.3, profile=profile, pose=pose(yaw=math.radians(31)),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        self.assertEqual(decision.state, DONE)
        self.assertEqual(decision.angular_rps, 0.0)

    def test_right_turn_is_negative(self):
        brain = Cerebellum(turn_timeout_s=10.0)
        brain.set_goal({"goal": "right"}, now=0.0)
        decision = brain.tick(
            now=0.1, profile=profile_for(front=2.0, right=2.0), pose=pose(),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        self.assertLess(decision.angular_rps, 0.0)

    def test_turn_uses_tighter_clearance_threshold(self):
        brain = Cerebellum(min_clearance_turn_m=0.12, turn_timeout_s=10.0)
        brain.set_goal({"goal": "left"}, now=0.0)
        # 0.2 m on the swept side: below the driving threshold but above the
        # in-place-turn threshold, so the turn is allowed.
        decision = brain.tick(
            now=0.1, profile=profile_for(front=2.0, left=0.2), pose=pose(),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        self.assertEqual(decision.state, ACTIVE)


class OpenSpaceTests(unittest.TestCase):
    def test_chooses_the_freer_side(self):
        brain = Cerebellum(open_space_timeout_s=10.0)
        brain.set_goal({"goal": "open_space"}, now=0.0)
        decision = brain.tick(
            now=0.1, profile=profile_for(front=1.0, left=0.4, right=3.0), pose=pose(),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        self.assertEqual(decision.state, ACTIVE)
        self.assertLess(decision.angular_rps, 0.0)
        self.assertEqual(decision.detail["best_sector_clearance_m"], 3.0)

    def test_vetoes_when_all_directions_are_close(self):
        brain = Cerebellum(min_clearance_m=0.25)
        brain.set_goal({"goal": "open_space"}, now=0.0)
        decision = brain.tick(
            now=0.1, profile=profile_for(front=0.2, left=0.2, right=0.2, back=2.0),
            pose=pose(), scan_age_s=0.1, odom_age_s=0.1,
        )
        self.assertEqual(decision.state, VETO)

    def test_stops_after_distance(self):
        brain = Cerebellum(open_space_distance_m=0.05, open_space_timeout_s=10.0)
        brain.set_goal({"goal": "open_space"}, now=0.0)
        profile = profile_for(front=2.0, left=2.0, right=2.0)
        brain.tick(now=0.1, profile=profile, pose=pose(), scan_age_s=0.1, odom_age_s=0.1)
        decision = brain.tick(
            now=0.2, profile=profile, pose=pose(x=0.06, y=0.02),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        self.assertEqual(decision.state, DONE)


class StopAndIdleTests(unittest.TestCase):
    def test_stop_emits_a_zero_burst_then_goes_quiet(self):
        brain = Cerebellum(stop_burst=3)
        brain.set_goal({"goal": "forward"}, now=0.0)
        brain.set_goal({"goal": "stop"}, now=0.1)

        published = 0
        for step in range(5):
            decision = brain.tick(
                now=1.0 + step, profile=profile_for(front=2.0), pose=pose(),
                scan_age_s=0.1, odom_age_s=0.1,
            )
            if decision.publish:
                published += 1
                self.assertEqual(decision.linear_mps, 0.0)
                self.assertEqual(decision.angular_rps, 0.0)
        self.assertEqual(published, 3)
        self.assertFalse(decision.publish)
        self.assertIsNone(brain.goal)

    def test_idle_does_not_publish(self):
        brain = Cerebellum()
        decision = brain.tick(
            now=1.0, profile=profile_for(front=2.0), pose=pose(),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        self.assertFalse(decision.publish)
        self.assertEqual(decision.linear_mps, 0.0)

    def test_unknown_goal_is_rejected(self):
        brain = Cerebellum()
        result = brain.set_goal({"goal": "jump"}, now=0.0)
        self.assertFalse(result["ok"])
        self.assertEqual(result["error"], "unknown_goal")
        self.assertIn(FORWARD, result["accepted_goals"])


class GuidedPriorityTests(unittest.TestCase):
    """The L2 contract: veto + a 12-number preference, never wheel speeds."""

    def flat_priority(self, **overrides):
        values = [0.0] * 12
        for index, value in overrides.items():
            values[int(index[1:])] = value
        return values

    def test_priority_length_must_match_sectors(self):
        brain = Cerebellum(sectors=12)
        self.assertEqual(brain.set_priority({"priority": [0.5] * 11}, now=0.0)["error"], "priority_length")
        self.assertTrue(brain.set_priority({"priority": [0.5] * 12}, now=0.0)["ok"])

    def test_priority_values_are_clamped(self):
        brain = Cerebellum(sectors=12)
        result = brain.set_priority({"priority": [-5] + [9] * 11}, now=0.0)
        self.assertTrue(result["ok"])

    def test_priority_biases_an_otherwise_equal_choice(self):
        profile = profile_for(front=1.0, left=1.0, right=1.0)
        right_brain = Cerebellum()
        right_brain.set_priority({"priority": self.flat_priority(s4=1.0)}, now=0.0)
        decision = right_brain.tick(
            now=0.1, profile=profile, pose=pose(), scan_age_s=0.1, odom_age_s=0.1
        )
        self.assertEqual(decision.state, ACTIVE)
        self.assertLess(decision.angular_rps, 0.0)
        self.assertEqual(decision.detail["mode"], "guided")

        left_brain = Cerebellum()
        left_brain.set_priority({"priority": self.flat_priority(s7=1.0)}, now=0.0)
        decision = left_brain.tick(
            now=0.1, profile=profile, pose=pose(), scan_age_s=0.1, odom_age_s=0.1
        )
        self.assertGreater(decision.angular_rps, 0.0)

    def test_model_veto_stops_the_robot(self):
        brain = Cerebellum()
        brain.set_priority(
            {"veto": True, "reason": "前方是人", "priority": [0.5] * 12}, now=0.0
        )
        decision = brain.tick(
            now=0.1, profile=profile_for(front=2.0), pose=pose(),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        self.assertEqual(decision.state, VETO)
        self.assertTrue(decision.reason.startswith("veto:cortex:"))
        self.assertEqual(decision.linear_mps, 0.0)
        self.assertEqual(decision.angular_rps, 0.0)

    def test_geometry_beats_a_wrong_priority(self):
        # The model asks for the left side, but the left side is 0.1 m away.
        brain = Cerebellum(min_clearance_m=0.25)
        brain.set_priority({"priority": self.flat_priority(s7=1.0)}, now=0.0)
        decision = brain.tick(
            now=0.1, profile=profile_for(front=1.0, left=0.1, right=2.0), pose=pose(),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        # The fused score still prefers the open right side; the model cannot
        # steer the robot into the blocked one.
        self.assertEqual(decision.state, ACTIVE)
        self.assertLess(decision.angular_rps, 0.0)

    def test_fused_best_below_clearance_is_vetoed(self):
        brain = Cerebellum(min_clearance_m=0.25)
        brain.set_priority({"priority": [0.5] * 12}, now=0.0)
        decision = brain.tick(
            now=0.1, profile=profile_for(front=0.2, left=0.2, right=0.2), pose=pose(),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        self.assertEqual(decision.state, VETO)
        self.assertTrue(decision.reason.startswith("veto:obstacle"))

    def test_priority_expiry_stops_and_zeroes(self):
        brain = Cerebellum(stop_burst=2)
        brain.set_priority({"priority": [0.5] * 12, "ttl_ms": 300}, now=0.0)
        active = brain.tick(
            now=0.2, profile=profile_for(front=2.0), pose=pose(),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        self.assertEqual(active.state, ACTIVE)

        stopped = brain.tick(
            now=0.4, profile=profile_for(front=2.0), pose=pose(),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        self.assertEqual(stopped.state, DONE)
        self.assertEqual(stopped.reason, "priority_expired")
        self.assertTrue(stopped.publish)
        self.assertEqual(stopped.linear_mps, 0.0)

        quiet = brain.tick(
            now=0.5, profile=profile_for(front=2.0), pose=pose(),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        self.assertEqual(quiet.state, DONE)
        self.assertTrue(quiet.publish)  # second zero of the burst
        self.assertFalse(
            brain.tick(
                now=0.6, profile=profile_for(front=2.0), pose=pose(),
                scan_age_s=0.1, odom_age_s=0.1,
            ).publish
        )

    def test_model_veto_outranks_an_explicit_goal(self):
        brain = Cerebellum(turn_timeout_s=10.0)
        brain.set_goal({"goal": "left"}, now=0.0)
        brain.set_priority({"veto": True, "reason": "前方是人", "priority": [0.5] * 12}, now=0.1)
        decision = brain.tick(
            now=0.2, profile=profile_for(front=2.0, left=2.0), pose=pose(),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        self.assertEqual(decision.state, VETO)
        self.assertEqual(decision.angular_rps, 0.0)
        self.assertIsNone(brain.goal)
        # The veto is consumed once; a fresh goal is not silently blocked.
        brain.set_goal({"goal": "left"}, now=0.3)
        decision = brain.tick(
            now=0.4, profile=profile_for(front=2.0, left=2.0), pose=pose(),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        self.assertEqual(decision.state, ACTIVE)

    def test_guided_refuses_stale_scan(self):
        brain = Cerebellum(scan_timeout_s=1.0)
        brain.set_priority({"priority": [0.5] * 12}, now=0.0)
        decision = brain.tick(
            now=0.1, profile=profile_for(front=2.0), pose=pose(),
            scan_age_s=2.0, odom_age_s=0.1,
        )
        self.assertEqual(decision.state, BLOCKED)
        self.assertEqual(decision.reason, "scan_stale")


class SectorGoalTests(unittest.TestCase):
    """Sector goals are the executor half of the MCP ``forward(sector)`` /
    ``look(sector)`` vocabulary. The cortex names a sector; the geometry,
    the turn rate and the veto stay here."""

    def test_parse_goal_bounds_the_sector(self):
        self.assertEqual(parse_goal({"goal": "face", "sector": 7}).sector, 7)
        self.assertEqual(parse_goal({"goal": "forward", "sector": 7}).sector, 7)
        self.assertEqual(parse_goal({"goal": "look", "sector": 7}).sector, 7)
        self.assertEqual(parse_goal({"goal": "face", "sector": 64}).error, "invalid_sector")
        self.assertEqual(parse_goal({"goal": "face", "sector": "7"}).error, "invalid_sector")
        self.assertEqual(parse_goal({"goal": "face", "sector": True}).error, "invalid_sector")

    def test_face_requires_a_sector_and_checks_the_count(self):
        brain = Cerebellum()
        result = brain.set_goal({"goal": "face"}, now=0.0)
        self.assertFalse(result["accepted"])
        self.assertEqual(result["error"], "sector_required")
        result = brain.set_goal({"goal": "face", "sector": 12}, now=0.1)
        self.assertFalse(result["accepted"])
        self.assertEqual(result["error"], "sector_out_of_range")

    def test_face_turns_in_place_towards_a_left_sector(self):
        brain = Cerebellum()
        brain.set_goal({"goal": "face", "sector": 8}, now=0.0)  # +75 deg
        decision = brain.tick(
            now=0.1, profile=profile_for(front=2.0, left=2.0, right=2.0), pose=pose(),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        self.assertEqual(decision.state, ACTIVE)
        self.assertEqual(decision.linear_mps, 0.0)
        self.assertGreater(decision.angular_rps, 0.0)
        self.assertAlmostEqual(math.degrees(decision.detail["goal_sector_angle_rad"]), 75.0, places=3)
        self.assertEqual(decision.as_status()["goal_sector"], 8)

    def test_face_reaches_the_sector_and_stops(self):
        brain = Cerebellum()
        brain.set_goal({"goal": "face", "sector": 8}, now=0.0)
        brain.tick(
            now=0.1, profile=profile_for(front=2.0, left=2.0, right=2.0), pose=pose(),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        decision = brain.tick(
            now=0.2,
            profile=profile_for(front=2.0, left=2.0, right=2.0),
            pose=pose(yaw=math.radians(75.0)),
            scan_age_s=0.1,
            odom_age_s=0.1,
        )
        self.assertEqual(decision.state, DONE)
        self.assertEqual(decision.reason, "goal_reached")
        self.assertEqual(decision.linear_mps, 0.0)
        self.assertEqual(decision.angular_rps, 0.0)

    def test_face_vetoes_when_the_target_sector_is_blocked(self):
        brain = Cerebellum()
        brain.set_goal({"goal": "face", "sector": 8}, now=0.0)
        decision = brain.tick(
            now=0.1, profile=profile_for(front=2.0, left=0.1, right=2.0), pose=pose(),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        self.assertEqual(decision.state, VETO)
        self.assertTrue(decision.reason.startswith("veto:obstacle"))
        self.assertEqual(decision.angular_rps, 0.0)

    def test_forward_into_a_sector_steers_and_advances(self):
        brain = Cerebellum()
        brain.set_goal({"goal": "forward", "sector": 5, "distance_m": 0.1}, now=0.0)
        decision = brain.tick(
            now=0.1, profile=profile_for(front=2.0, left=2.0, right=2.0), pose=pose(),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        self.assertEqual(decision.state, ACTIVE)
        self.assertGreater(decision.linear_mps, 0.0)
        self.assertLess(decision.angular_rps, 0.0)  # sector 5 is -15 deg (right of centre)

    def test_forward_into_a_blocked_sector_is_vetoed(self):
        brain = Cerebellum()
        brain.set_goal({"goal": "forward", "sector": 7, "distance_m": 0.1}, now=0.0)
        decision = brain.tick(
            now=0.1, profile=profile_for(front=2.0, left=0.1, right=2.0), pose=pose(),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        self.assertEqual(decision.state, VETO)
        self.assertEqual(decision.linear_mps, 0.0)

    def test_face_times_out_instead_of_turning_forever(self):
        brain = Cerebellum(face_timeout_s=1.0)
        brain.set_goal({"goal": "face", "sector": 8}, now=0.0)
        brain.tick(
            now=0.1, profile=profile_for(front=2.0, left=2.0, right=2.0), pose=pose(),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        decision = brain.tick(
            now=1.5, profile=profile_for(front=2.0, left=2.0, right=2.0), pose=pose(yaw=0.1),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        self.assertEqual(decision.state, DONE)
        self.assertEqual(decision.reason, "timeout")
        self.assertEqual(decision.angular_rps, 0.0)


class SkillTableTests(unittest.TestCase):
    """The table is the vocabulary; these tests prove the file really drives it."""

    def table_without(self, skill: str):
        value = skills.builtin_table_json()
        value["skills"].pop(skill, None)
        return skills.parse_table(value, source="test")

    def test_unknown_skill_is_rejected_with_the_known_list(self):
        brain = Cerebellum(skill_table=self.table_without("left"))
        result = brain.set_goal({"goal": "left"}, now=0.0)
        self.assertFalse(result["accepted"])
        self.assertEqual(result["error"], "unknown_skill")
        self.assertNotIn("left", result["accepted_goals"])
        self.assertIn("forward", result["accepted_goals"])

    def test_distance_bound_comes_from_the_table(self):
        brain = Cerebellum()
        result = brain.set_goal({"goal": "forward", "sector": 6, "distance_m": 0.3}, now=0.0)
        self.assertFalse(result["accepted"])
        self.assertEqual(result["error"], "distance_m_out_of_range")
        result = brain.set_goal({"goal": "forward", "sector": 6, "distance_m": 0.15}, now=0.1)
        self.assertTrue(result["accepted"])
        self.assertEqual(result["primitive"], "advance")

    def test_face_without_sector_is_rejected_by_the_table(self):
        brain = Cerebellum()
        result = brain.set_goal({"goal": "face"}, now=0.0)
        self.assertFalse(result["accepted"])
        self.assertEqual(result["error"], "sector_required")

    def test_forward_turns_first_then_advances(self):
        brain = Cerebellum()
        brain.set_goal({"goal": "forward", "sector": 8, "distance_m": 0.12}, now=0.0)  # +75 deg
        first = brain.tick(
            now=0.1, profile=profile_for(front=2.0, left=2.0, right=2.0), pose=pose(),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        self.assertEqual(first.state, ACTIVE)
        self.assertEqual(first.detail["goal_phase"], "turn")
        self.assertEqual(first.linear_mps, 0.0)
        self.assertGreater(first.angular_rps, 0.0)

        second = brain.tick(
            now=0.2, profile=profile_for(front=2.0, left=2.0, right=2.0),
            pose=pose(yaw=math.radians(70.0)), scan_age_s=0.1, odom_age_s=0.1,
        )
        self.assertEqual(second.state, ACTIVE)
        self.assertEqual(second.detail["goal_phase"], "advance")
        self.assertGreater(second.linear_mps, 0.0)

    def test_forward_aligned_with_the_sector_advances_immediately(self):
        brain = Cerebellum()
        brain.set_goal({"goal": "forward", "sector": 6}, now=0.0)  # +15 deg
        decision = brain.tick(
            now=0.1, profile=profile_for(front=2.0, left=2.0, right=2.0), pose=pose(),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        self.assertEqual(decision.state, ACTIVE)
        self.assertEqual(decision.detail["goal_phase"], "advance")
        self.assertGreater(decision.linear_mps, 0.0)

    def test_forward_without_a_sector_holds_the_heading(self):
        """No sector means "along the current heading", not "aim at sector 6".

        A 12-sector scan has no sector centred straight ahead (they land on ±15 deg),
        so a default sector would make every plain "forward" veer 15 deg.
        """
        brain = Cerebellum()
        result = brain.set_goal({"goal": "forward", "distance_m": 0.05}, now=0.0)
        self.assertTrue(result["accepted"])
        decision = brain.tick(
            now=0.1, profile=profile_for(front=2.0, left=2.0, right=2.0), pose=pose(),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        self.assertEqual(decision.state, ACTIVE)
        self.assertNotIn("goal_phase", decision.detail)
        self.assertNotIn("goal_sector", decision.detail)
        self.assertGreater(decision.linear_mps, 0.0)
        self.assertEqual(decision.angular_rps, 0.0)

    def test_custom_table_skill_runs_without_a_code_change(self):
        value = skills.builtin_table_json()
        value["skills"]["swivel"] = {
            "primitive": "turn_in_place",
            "params": {"sector": {"type": "int", "min": 0, "max": 11, "required": True}},
            "preconditions": ["turn_clear"],
        }
        table = skills.parse_table(value, source="test")
        brain = Cerebellum(skill_table=table)
        result = brain.set_goal({"goal": "swivel", "sector": 10}, now=0.0)  # +135 deg
        self.assertTrue(result["accepted"])
        decision = brain.tick(
            now=0.1, profile=profile_for(front=2.0, left=2.0, right=2.0), pose=pose(),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        self.assertEqual(decision.state, ACTIVE)
        self.assertEqual(decision.linear_mps, 0.0)
        self.assertGreater(decision.angular_rps, 0.0)

    def test_zero_primitive_custom_skill_stops_like_stop(self):
        value = skills.builtin_table_json()
        value["skills"]["halt"] = {"primitive": "zero", "params": {}}
        brain = Cerebellum(skill_table=skills.parse_table(value, source="test"))
        brain.set_goal({"goal": "forward", "sector": 6}, now=0.0)
        result = brain.set_goal({"goal": "halt"}, now=0.1)
        self.assertTrue(result["accepted"])
        self.assertIsNone(brain.goal)
        self.assertEqual(brain.last_state, DONE)

    def test_active_skill_reports_its_primitive(self):
        brain = Cerebellum()
        brain.set_goal({"goal": "face", "sector": 8}, now=0.0)
        decision = brain.tick(
            now=0.1, profile=profile_for(front=2.0, left=2.0, right=2.0), pose=pose(),
            scan_age_s=0.1, odom_age_s=0.1,
        )
        status = decision.as_status()
        self.assertEqual(status["goal_primitive"], "turn_in_place")
        self.assertEqual(status["goal_sector"], 8)


if __name__ == "__main__":
    unittest.main()
