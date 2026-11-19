import math
import unittest

from agent_control.explorer import FrontierExplorer, GridMap
from agent_control.navigation import BODY_RADIUS_M, TURN_RANGE_M, GridRouter, segment_is_free


class GridRouteTests(unittest.TestCase):
    def test_home_route_goes_around_a_wall_using_only_observed_free_cells(self):
        data = [0] * 900
        for y in range(5, 25):
            data[y * 30 + 15] = 100
        grid = GridMap(30, 30, .1, 0, 0, data)
        router = GridRouter(grid, (.8, 1.5, 0), clearance=.2)
        path = router.home((2.2, 1.5))
        self.assertTrue(path)
        self.assertTrue(any(y < .5 or y > 2.5 for _, y in path))
        self.assertTrue(all(grid.world_to_cell(*point) in router.free for point in path))

    def test_unknown_corridor_cannot_be_used_as_a_home_route(self):
        data = [0] * 400
        for y in range(20):
            data[y * 20 + 10] = -1
        grid = GridMap(20, 20, .1, 0, 0, data)
        self.assertFalse(GridRouter(grid, (.5, 1, 0)).home((1.5, 1)))

    def test_footprint_includes_outer_wheels_and_lidar_offset(self):
        self.assertAlmostEqual(BODY_RADIUS_M, math.hypot(.0725, .081))
        self.assertGreater(TURN_RANGE_M, .18)

    def test_a_smoothed_route_cannot_cut_past_an_unknown_corner(self):
        grid = GridMap(2, 2, 1.0, 0, 0, [0, -1, 0, 0])
        router = GridRouter(grid, (.5, .5, 0))
        self.assertTrue(router.path((1, 1)))
        self.assertFalse(segment_is_free(grid, router.free, (.5, .5), (1.5, 1.5)))
        self.assertFalse(segment_is_free(grid, router.free, (1.5, 1.5), (.5, .5)))


class MeasuredRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.explorer = FrontierExplorer()
        self.grid = GridMap(40, 40, .05, -1, -1, [0] * 1600)
        self.amin, self.inc = -math.pi, 2 * math.pi / 360

    def decide(self, pose, values, now):
        return self.explorer.decide(self.grid, pose, values, self.amin, self.inc, now=now)

    def test_negative_rear_obstacle_and_missing_rear_both_refuse_reverse(self):
        for rear in (.1, float('nan')):
            with self.subTest(rear=rear):
                self.explorer = FrontierExplorer()
                self.explorer._last_pose = (0, 0, 0)
                self.explorer.force_recovery(0, "test")
                scan = [2.] * 360
                scan[:30] = [rear] * 30
                scan[330:] = [rear] * 30
                decision = self.decide((0, 0, 0), scan, .1)
                self.assertEqual(decision.state, "BLOCKED")
                self.assertEqual(decision.reason, "reverse_blocked")
                self.assertEqual(decision.proposal.linear_mps, 0)

    def test_requested_reverse_without_pose_progress_spends_no_distance_and_stops(self):
        self.explorer._last_pose = (0, 0, 0)
        self.explorer.force_recovery(0, "test")
        scan = [2.] * 360
        scan[180] = .13
        self.decide((0, 0, 0), scan, .1)
        decision = self.decide((0, 0, 0), scan, 3.2)
        self.assertEqual(decision.reason, "reverse_no_progress")
        self.assertEqual(self.explorer.recovery_snapshot()["reverse_total_m"], 0)

    def test_exit_is_not_authorised_by_one_long_ray_in_a_narrow_gap(self):
        self.explorer.recovery_phase = "SELECT_EXIT"
        scan = [.2] * 360
        scan[180] = 3.
        decision = self.decide((0, 0, 0), scan, .1)
        self.assertEqual(decision.reason, "no_safe_exit")
        self.assertEqual(decision.proposal.linear_mps, 0)

    def test_new_obstacle_stops_an_exit_immediately(self):
        self.explorer.recovery_phase = "EXIT"
        self.explorer._exit_pose = (0, 0, 0)
        scan = [2.] * 360
        scan[180] = .2
        decision = self.decide((.03, 0, 0), scan, .1)
        self.assertEqual(decision.reason, "exit_blocked")
        self.assertEqual(decision.proposal.linear_mps, 0)


if __name__ == '__main__':
    unittest.main()
