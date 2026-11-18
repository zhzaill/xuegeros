import math
import unittest

from agent_control.explorer import GridMap
from agent_control.tour import (
    DONE,
    EXPLORE,
    LIMIT,
    RETURN,
    STOPPED,
    TourController,
    known_cells,
    scan_signature,
)

FULL_TURN = 2.0 * math.pi


def make_grid(width=20, height=20, *, frontier=False, unknown=0):
    data = [0] * (width * height)
    for index in range(min(unknown, width * height)):
        data[index] = -1
    if frontier:
        for y in range(height // 2 - 2, height // 2 + 2):
            for x in range(width - 6, width - 2):
                data[y * width + x] = -1
    return GridMap(width, height, 0.05, 0.0, 0.0, data)


def ranges(front=1.0, other=2.0, points=360):
    angle_min = -math.pi
    increment = FULL_TURN / points
    values = []
    for index in range(points):
        angle = angle_min + index * increment
        values.append(front if abs(angle) < math.radians(20) else other)
    return values, angle_min, increment


def start_tour(control, grid, pose=(0.0, 0.0, 0.0), now=0.0, request_id="tour-1"):
    values, angle_min, increment = ranges()
    return control.start(
        now=now,
        request_id=request_id,
        source="test",
        grid=grid,
        pose_map=pose,
        pose_odom=pose,
        ranges=values,
        angle_min=angle_min,
        angle_increment=increment,
    )


def step(control, grid, pose, now, *, facts_due=False):
    values, angle_min, increment = ranges()
    return control.step(
        now=now,
        grid=grid,
        pose_map=pose,
        ranges=values,
        angle_min=angle_min,
        angle_increment=increment,
        facts_due=facts_due,
    )


class TourLifecycleTests(unittest.TestCase):
    def test_start_reports_a_handle_and_facts(self):
        control = TourController()
        facts = start_tour(control, make_grid())
        self.assertEqual(control.phase, EXPLORE)
        self.assertTrue(control.active)
        self.assertEqual(facts["request_id"], "tour-1")
        self.assertEqual(facts["phase"], EXPLORE)
        self.assertEqual(facts["map"]["known_cells"], 400)
        self.assertEqual(facts["return"]["dist_to_start_m"], 0.0)

    def test_saturation_switches_to_return(self):
        control = TourController(no_frontier_samples=2)
        grid = make_grid()
        start_tour(control, grid)
        for index in range(2):
            step(control, grid, (0.5, 0.0, 0.0), 1.0 + index, facts_due=True)
        self.assertEqual(control.phase, RETURN)
        self.assertTrue(control.active)

    def test_returning_home_finishes(self):
        control = TourController(no_frontier_samples=1)
        grid = make_grid()
        start_tour(control, grid)
        step(control, grid, (0.5, 0.0, 0.0), 1.0, facts_due=True)
        self.assertEqual(control.phase, RETURN)
        action, facts = step(control, grid, (0.1, 0.0, 0.0), 2.0)
        self.assertEqual(control.phase, DONE)
        self.assertFalse(control.active)
        self.assertEqual(action.linear_mps, 0.0)
        self.assertTrue(facts["return"]["started"])

    def test_frontier_keeps_exploring(self):
        control = TourController(no_frontier_samples=1)
        grid = make_grid(frontier=True)
        start_tour(control, grid)
        action, facts = step(control, grid, (0.5, 0.5, 0.0), 1.0, facts_due=True)
        self.assertEqual(control.phase, EXPLORE)
        self.assertIsNotNone(facts["map"]["frontier_count"])
        self.assertGreater(facts["map"]["frontier_count"], 0)
        self.assertTrue(action.linear_mps >= 0.0)

    def test_runtime_limit_stops_the_tour(self):
        control = TourController(max_runtime_s=10.0)
        grid = make_grid(frontier=True)
        start_tour(control, grid)
        action, facts = step(control, grid, (0.0, 0.0, 0.0), 11.0)
        self.assertEqual(control.phase, LIMIT)
        self.assertEqual(action.linear_mps, 0.0)
        self.assertEqual(facts["reason"], "runtime_limit")
        self.assertFalse(control.active)

    def test_return_timeout_is_reported_as_limit(self):
        control = TourController(no_frontier_samples=1, return_timeout_s=5.0)
        grid = make_grid()
        start_tour(control, grid)
        step(control, grid, (0.5, 0.0, 0.0), 1.0, facts_due=True)
        self.assertEqual(control.phase, RETURN)
        step(control, grid, (0.5, 0.0, 0.0), 7.0)
        self.assertEqual(control.phase, LIMIT)
        self.assertEqual(control.reason, "return_timeout")

    def test_stop_ends_an_active_tour(self):
        control = TourController()
        start_tour(control, make_grid())
        facts = control.stop(reason="operator")
        self.assertEqual(control.phase, STOPPED)
        self.assertEqual(facts["reason"], "operator")
        self.assertFalse(control.active)

    def test_invalid_scan_holds_position_without_changing_phase(self):
        control = TourController()
        grid = make_grid(frontier=True)
        start_tour(control, grid)
        values = [float("nan")] * 360
        action, _ = control.step(
            now=1.0,
            grid=grid,
            pose_map=(0.0, 0.0, 0.0),
            ranges=values,
            angle_min=-math.pi,
            angle_increment=FULL_TURN / 360,
        )
        self.assertEqual(control.phase, EXPLORE)
        self.assertEqual(action.linear_mps, 0.0)
        self.assertEqual(action.angular_rps, 0.0)


class TourFactsTests(unittest.TestCase):
    def test_encoder_path_tracks_odometry_not_the_controller_claim(self):
        control = TourController(no_frontier_samples=99)
        grid = make_grid(frontier=True)
        start_tour(control, grid)
        control.observe(pose_odom=(0.3, 0.0, 0.0))
        _, facts = step(control, grid, (0.3, 0.0, 0.0), 1.0)
        self.assertAlmostEqual(facts["encoder_path_m"], 0.3, places=3)
        self.assertAlmostEqual(facts["encoder_delta_m"], 0.3, places=3)

    def test_scan_signature_shows_world_change(self):
        control = TourController(no_frontier_samples=99)
        grid = make_grid(frontier=True)
        start_tour(control, grid)
        changed, mean, pairs = __import__(
            "agent_control.tour", fromlist=["_signature_delta"]
        )._signature_delta(
            scan_signature(ranges(front=2.5)[0], -math.pi, FULL_TURN / 360),
            scan_signature(ranges(front=1.0)[0], -math.pi, FULL_TURN / 360),
        )
        self.assertGreater(changed, 0)
        self.assertGreater(mean, 0.1)
        self.assertGreater(pairs, 0)

    def test_actuator_and_autonomy_are_reported_separately(self):
        control = TourController(no_frontier_samples=99)
        grid = make_grid(frontier=True)
        start_tour(control, grid)
        control.observe(actuator=(0.0, 0.0), actuator_age_s=0.05, encoder_twist_mps=0.0)
        _, facts = step(control, grid, (0.0, 0.0, 0.0), 1.0)
        self.assertIn("autonomy", facts["cmd"])
        self.assertIn("actuator", facts["cmd"])
        self.assertEqual(facts["cmd"]["actuator"], [0.0, 0.0])
        self.assertEqual(facts["encoder_twist_mps"], 0.0)

    def test_growth_window_reports_map_saturation(self):
        control = TourController(
            no_frontier_samples=99, saturation_window_s=2.0, growth_eps_cells=10
        )
        sparse = make_grid(frontier=True, unknown=40)
        fuller = make_grid(frontier=True, unknown=20)
        start_tour(control, sparse, now=0.0)
        step(control, fuller, (0.0, 0.0, 0.0), 1.0, facts_due=True)
        step(control, fuller, (0.0, 0.0, 0.0), 2.0, facts_due=True)
        self.assertEqual(control.phase, EXPLORE)
        _, facts = step(control, fuller, (0.0, 0.0, 0.0), 3.0, facts_due=True)
        self.assertGreater(facts["map"]["known_delta_start"], 0)
        self.assertEqual(control.phase, DONE)
        self.assertEqual(control.reason, "returned_home")

    def test_no_progress_ends_the_tour_as_limit(self):
        control = TourController(no_progress_timeout_s=5.0)
        grid = make_grid(frontier=True)
        start_tour(control, grid, now=0.0)
        step(control, grid, (0.0, 0.0, 0.0), 1.0, facts_due=True)
        _, facts = step(control, grid, (0.0, 0.0, 0.0), 6.0, facts_due=True)
        self.assertEqual(control.phase, LIMIT)
        self.assertEqual(facts["reason"], "no_progress")
        self.assertEqual(facts["encoder_path_m"], 0.0)

    def test_an_empty_map_is_not_reported_as_complete(self):
        """No known cell means no frontier to evaluate, never "the room is done".

        The frontier detector looks for unknown cells next to free ones; on an
        all-unknown grid it finds none and answers COMPLETE, which used to end a
        freshly started tour with a false DONE(reached). The controller now
        bootstraps by sweeping in place until the map is dense enough to plan on.
        """

        control = TourController()
        grid = make_grid(unknown=20 * 20)
        self.assertEqual(known_cells(grid), 0)
        start_tour(control, grid, now=0.0)
        action, facts = step(control, grid, (0.0, 0.0, 0.0), 1.0, facts_due=True)
        self.assertNotEqual(control.phase, DONE)
        self.assertGreater(abs(action.angular_rps), 0.0)
        self.assertEqual(action.linear_mps, 0.0)
        self.assertEqual(facts["navigation"]["explorer_state"], "BOOTSTRAP")

    def test_a_sparse_map_does_not_saturate(self):
        control = TourController()
        data = [-1] * (20 * 20)
        data[20 * 10 + 10] = 0  # only the robot's own cell is known
        grid = GridMap(20, 20, 0.05, 0.0, 0.0, data)
        start_tour(control, grid, now=0.0)
        for index in range(6):
            step(control, grid, (0.5, 0.5, 0.0), 0.02 * index, facts_due=True)
        self.assertEqual(control.phase, EXPLORE)


if __name__ == "__main__":
    unittest.main()
