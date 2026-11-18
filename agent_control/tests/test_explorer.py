"""Frontier explorer avoidance behaviour.

These exist because of a measured field failure: in a room with crates the tour
turned in place, flipping direction about once per second, until ``no_progress``
ended the run. The trace (target id stable, proposal angular sign flipping, no
executor veto) pointed at the avoid-direction choice, not at the planner.
"""

import math
import unittest

from agent_control.explorer import FrontierExplorer, GridMap

FULL_TURN = 2.0 * math.pi


def make_grid(width=20, height=20):
    """Mostly free, with an unknown band on the far side so a frontier exists."""

    data = [0] * (width * height)
    for y in range(height // 2 - 1, height // 2 + 1):
        for x in range(width - 4, width - 1):
            data[y * width + x] = -1
    return GridMap(width, height, 0.05, 0.0, 0.0, data)


def ranges(front=0.2, other=3.0, points=360):
    angle_min = -math.pi
    increment = FULL_TURN / points
    values = [
        front if abs(angle_min + index * increment) < math.radians(15) else other
        for index in range(points)
    ]
    return values, angle_min, increment


def ranges_situation(front=0.2, side=0.1, rear=3.0, points=360):
    """Front cone, both side sweeps and the rear each get their own distance."""

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


class ExplorerRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.grid = make_grid()
        self.pose = (0.2, 0.2, 0.0)

    def test_boxed_robot_reverses_instead_of_rotating_into_the_obstacle(self):
        boxed, angle_min, increment = ranges_situation(front=0.2, side=0.10)
        explorer = FrontierExplorer(obstacle_stop_m=0.32)
        decision = explorer.decide(self.grid, self.pose, boxed, angle_min, increment, now=0.0)
        self.assertEqual(decision.state, "RECOVER")
        self.assertLess(decision.proposal.linear_mps, 0.0)
        self.assertEqual(decision.proposal.angular_rps, 0.0)

    def test_reverse_is_refused_when_the_rear_is_short(self):
        boxed, angle_min, increment = ranges_situation(front=0.2, side=0.10, rear=0.2)
        explorer = FrontierExplorer(obstacle_stop_m=0.32)
        decision = explorer.decide(self.grid, self.pose, boxed, angle_min, increment, now=0.0)
        self.assertEqual(decision.state, "BLOCKED")
        self.assertEqual(decision.reason, "obstacle_stuck")

    def test_a_blocked_rear_stops_an_ongoing_reverse(self):
        boxed, angle_min, increment = ranges_situation(front=0.2, side=0.10)
        explorer = FrontierExplorer(obstacle_stop_m=0.32)
        started = explorer.decide(self.grid, self.pose, boxed, angle_min, increment, now=0.0)
        self.assertEqual(started.state, "RECOVER")
        closing, angle_min, increment = ranges_situation(front=0.2, side=0.10, rear=0.2)
        stopped = explorer.decide(self.grid, self.pose, closing, angle_min, increment, now=0.5)
        self.assertEqual(stopped.state, "BLOCKED")
        self.assertEqual(stopped.reason, "reverse_blocked")

    def test_reverse_budget_is_cumulative(self):
        boxed, angle_min, increment = ranges_situation(front=0.2, side=0.10)
        explorer = FrontierExplorer(
            obstacle_stop_m=0.32,
            max_recoveries=3,
            max_reverse_total_m=0.08,
            reverse_step_m=0.08,
        )
        first = explorer.decide(self.grid, self.pose, boxed, angle_min, increment, now=0.0)
        self.assertEqual(first.state, "RECOVER")
        # The 0.08 m budget is spent: a second reverse must not be granted. The
        # decision reports that as stuck (or as "no frontier" once the failed
        # target is blacklisted) -- either way no motion appears.
        second = explorer.decide(self.grid, self.pose, boxed, angle_min, increment, now=10.0)
        self.assertNotEqual(second.state, "RECOVER")
        self.assertGreaterEqual(second.proposal.linear_mps, 0.0)
        self.assertEqual(explorer._recoveries, 1)

    def test_stalled_pursuit_escalates_to_a_reverse(self):
        """Target behind an obstacle: MOVE turns, AVOID turns back, nothing moves.

        The stall timer is what breaks that limit cycle: an unreachable bearing held
        for pursuit_stall_s becomes a bounded reverse plus a fresh plan.
        """

        explorer = FrontierExplorer(obstacle_stop_m=0.32, pursuit_stall_s=2.0)
        values, angle_min, increment = ranges(front=3.0)  # front clear, target behind
        pose = (0.1, 0.1, math.pi)  # facing away from the frontier
        first = explorer.decide(self.grid, pose, values, angle_min, increment, now=0.0)
        self.assertEqual(first.state, "MOVE")
        self.assertEqual(first.proposal.linear_mps, 0.0)  # bearing too far to drive
        explorer.decide(self.grid, pose, values, angle_min, increment, now=1.0)
        stalled = explorer.decide(self.grid, pose, values, angle_min, increment, now=2.1)
        self.assertEqual(stalled.state, "RECOVER")
        self.assertLess(stalled.proposal.linear_mps, 0.0)

    def test_a_normal_wall_alongside_does_not_trigger_avoidance(self):
        # Front clear, wall at 0.25 m on both sides: this is plain driving along a
        # wall, not an obstacle episode, so the lateral sweep must not fire.
        corridor, angle_min, increment = ranges_situation(front=3.0, side=0.25)
        explorer = FrontierExplorer(obstacle_stop_m=0.32)
        decision = explorer.decide(self.grid, self.pose, corridor, angle_min, increment, now=0.0)
        self.assertEqual(decision.state, "MOVE")


class ExplorerAvoidTests(unittest.TestCase):
    def setUp(self):
        self.grid = make_grid()
        self.pose = (0.2, 0.2, 0.0)
        self.explorer = FrontierExplorer(
            max_linear_mps=0.04,
            max_angular_rps=0.3,
            obstacle_stop_m=0.32,
            avoid_hold_s=2.5,
            avoid_switch_limit=1,
        )

    def test_blocked_front_latches_one_turn_direction(self):
        blocked, angle_min, increment = ranges(front=0.2)
        first = self.explorer.decide(self.grid, self.pose, blocked, angle_min, increment, now=0.0)
        self.assertEqual(first.state, "AVOID")
        self.assertEqual(first.proposal.linear_mps, 0.0)
        direction = first.proposal.angular_rps
        self.assertNotEqual(direction, 0.0)

        # Inside the hold window the choice must not be revisited: this is the
        # wriggle that the old 1 s window allowed.
        for now in (0.4, 0.8, 1.2, 1.6, 2.4):
            later = self.explorer.decide(self.grid, self.pose, blocked, angle_min, increment, now=now)
            self.assertEqual(later.state, "AVOID")
            self.assertEqual(later.proposal.angular_rps, direction)

        # One window elapsed: try the other side exactly once.
        flipped = self.explorer.decide(self.grid, self.pose, blocked, angle_min, increment, now=2.6)
        self.assertEqual(flipped.state, "AVOID")
        self.assertEqual(flipped.proposal.angular_rps, -direction)

        # Both directions failed: back out (the rear is clear in this scan) rather
        # than rotating into the obstacle until the executor blocks.
        recovering = self.explorer.decide(self.grid, self.pose, blocked, angle_min, increment, now=5.3)
        self.assertEqual(recovering.state, "RECOVER")
        self.assertLess(recovering.proposal.linear_mps, 0.0)
        self.assertEqual(recovering.proposal.angular_rps, 0.0)

    def test_front_clearing_releases_the_avoid_episode(self):
        blocked, angle_min, increment = ranges(front=0.2)
        self.explorer.decide(self.grid, self.pose, blocked, angle_min, increment, now=0.0)
        clear, angle_min, increment = ranges(front=2.0)
        released = self.explorer.decide(self.grid, self.pose, clear, angle_min, increment, now=1.0)
        self.assertEqual(released.state, "MOVE")
        self.assertGreater(released.proposal.linear_mps, 0.0)
        self.assertFalse(self.explorer._avoid_active)

    def test_a_latched_turn_that_closes_on_the_obstacle_reverses(self):
        """The field failure: a latched direction rotated a crate into the cone.

        Clearance went 0.30 -> 0.12 while the hold window was still open, and the
        executor's close-range gate then blocked every motion. A worsening front
        must reverse the turn before the window expires.
        """

        start, angle_min, increment = ranges(front=0.30)
        first = self.explorer.decide(self.grid, self.pose, start, angle_min, increment, now=0.0)
        self.assertEqual(first.state, "AVOID")
        direction = first.proposal.angular_rps

        worse, angle_min, increment = ranges(front=0.22)
        reversed_ = self.explorer.decide(self.grid, self.pose, worse, angle_min, increment, now=0.3)
        self.assertEqual(reversed_.state, "AVOID")
        self.assertEqual(reversed_.proposal.angular_rps, -direction)

    def test_avoid_hold_is_a_parameter_not_a_magic_number(self):
        blocked, angle_min, increment = ranges(front=0.2)
        explorer = FrontierExplorer(obstacle_stop_m=0.32, avoid_hold_s=0.4, avoid_switch_limit=1)
        explorer.decide(self.grid, self.pose, blocked, angle_min, increment, now=0.0)
        early = explorer.decide(self.grid, self.pose, blocked, angle_min, increment, now=0.2)
        self.assertEqual(early.proposal.angular_rps, 0.3)  # still the latched side
        late = explorer.decide(self.grid, self.pose, blocked, angle_min, increment, now=0.5)
        self.assertEqual(late.proposal.angular_rps, -0.3)  # swept after the short hold


if __name__ == "__main__":
    unittest.main()
