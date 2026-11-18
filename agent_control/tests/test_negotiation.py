import math
import unittest

from agent_control.cerebellum import DONE, Cerebellum, profile_scan

FULL_TURN = 2.0 * math.pi


def scan(front=2.0, back=2.0, left=2.0, right=2.0, points=360):
    angle_min = -math.pi
    increment = FULL_TURN / points
    ranges = []
    for index in range(points):
        angle = angle_min + index * increment
        if abs(angle) <= math.radians(25):
            value = front
        elif math.radians(25) < angle <= math.pi / 2:
            value = left
        elif -math.pi / 2 <= angle < -math.radians(25):
            value = right
        else:
            value = back
        ranges.append(value)
    return ranges, angle_min, increment


def world(front=2.0, back=2.0, left=2.0, right=2.0):
    ranges, angle_min, increment = scan(front, back, left, right)
    return profile_scan(ranges, angle_min, increment)


def pose(x=0.0, y=0.0, yaw=0.0):
    return (x, y, yaw)


def primed(front=2.0, back=2.0, left=2.0, right=2.0, **kwargs):
    control = Cerebellum(**kwargs)
    control.tick(now=1.0, profile=world(front, back, left, right), pose=pose(), scan_age_s=0.1, odom_age_s=0.1)
    return control


class ProposeTests(unittest.TestCase):
    def test_open_forward_is_accepted_with_a_bound(self):
        control = primed()
        answer = control.propose({"goal": "forward", "sector": 6, "distance_m": 0.12}, now=1.1)
        self.assertTrue(answer["accepted"], answer)
        self.assertAlmostEqual(answer["distance_m"], 0.12, places=3)
        self.assertEqual(answer["expires_in_ms"], 3000)
        self.assertTrue(answer["proposal_id"].startswith("prop-"))
        self.assertTrue(answer["alternatives"])

    def test_blocked_front_is_refused_with_alternatives(self):
        control = primed(front=0.2)
        answer = control.propose({"goal": "forward", "sector": 6}, now=1.1)
        self.assertFalse(answer["accepted"])
        self.assertTrue(answer["error"].startswith("obstacle:"), answer)
        self.assertIsInstance(answer["alternatives"], list)

    def test_stale_scan_is_refused(self):
        control = Cerebellum()
        control.tick(now=1.0, profile=world(), pose=pose(), scan_age_s=9.0, odom_age_s=0.1)
        answer = control.propose({"goal": "forward"}, now=1.1)
        self.assertEqual(answer["error"], "scan_stale")

    def test_unknown_skill_lists_the_table(self):
        answer = primed().propose({"goal": "teleport"}, now=1.1)
        self.assertEqual(answer["error"], "unknown_goal")
        self.assertIn("forward", answer["accepted_goals"])

    def test_stop_needs_no_execution(self):
        answer = primed().propose({"goal": "stop"}, now=1.1)
        self.assertTrue(answer["accepted"])
        self.assertFalse(answer["requires_execute"])
        self.assertEqual(answer["state"], DONE)

    def test_distance_is_clamped_to_headroom(self):
        control = primed(front=0.12, min_clearance_m=0.08)
        answer = control.propose({"goal": "forward", "sector": 6, "distance_m": 0.15}, now=1.1)
        self.assertTrue(answer["accepted"], answer)
        self.assertLess(answer["distance_m"], 0.15)
        self.assertAlmostEqual(answer["distance_m"], 0.07, places=3)


class ExecuteTests(unittest.TestCase):
    def test_execute_consumes_the_proposal_once(self):
        control = primed()
        answer = control.propose({"goal": "face", "sector": 8}, now=1.1)
        self.assertTrue(answer["accepted"])
        first = control.execute({"proposal_id": answer["proposal_id"]}, now=1.2)
        self.assertTrue(first["accepted"], first)
        self.assertEqual(first["goal"], "face")
        second = control.execute({"proposal_id": answer["proposal_id"]}, now=1.3)
        self.assertEqual(second["error"], "proposal_unknown_or_expired")

    def test_expired_proposal_is_refused(self):
        control = primed()
        answer = control.propose({"goal": "forward", "sector": 6}, now=1.1)
        result = control.execute({"proposal_id": answer["proposal_id"]}, now=5.0)
        self.assertEqual(result["error"], "proposal_unknown_or_expired")

    def test_execute_rechecks_the_world(self):
        control = primed()
        answer = control.propose({"goal": "forward", "sector": 6}, now=1.1)
        control.tick(now=1.2, profile=world(front=0.2), pose=pose(), scan_age_s=0.1, odom_age_s=0.1)
        result = control.execute({"proposal_id": answer["proposal_id"]}, now=1.3)
        self.assertFalse(result["accepted"])
        self.assertTrue(result["error"].startswith("obstacle:"), result)

    def test_back_measures_the_rear_sector(self):
        control = primed(front=2.0, back=0.1)
        answer = control.propose({"goal": "back", "distance_m": 0.10}, now=1.1)
        self.assertFalse(answer["accepted"])

        control = primed(front=0.1, back=2.0)
        answer = control.propose({"goal": "back", "distance_m": 0.10}, now=1.1)
        self.assertTrue(answer["accepted"], answer)
        result = control.execute({"proposal_id": answer["proposal_id"]}, now=1.2)
        self.assertTrue(result["accepted"], result)
        decision = control.tick(
            now=1.3, profile=world(front=0.1, back=2.0), pose=pose(), scan_age_s=0.1, odom_age_s=0.1
        )
        self.assertLess(decision.linear_mps, 0.0)


if __name__ == "__main__":
    unittest.main()
