"""Fresh map, confirmed RUNNING, lifecycle through the real executor cycle."""
import unittest
import time

from sim.probe.tour_regression import make_sim


class TourLifecycleIntegrationTests(unittest.TestCase):
    def test_pause_resume_exclusion_and_stop_remain_still_on_later_ticks(self):
        sim, runner = make_sim()
        now = time.monotonic()
        sim.step(now, .0)
        sim.daemon.command({"t": "tour", "action": "start", "strategy": "tour", "variant_id": "room/default/01"})

        def ticks(count):
            nonlocal now
            for _ in range(count):
                now += .1
                sim.step(now, .1)

        ticks(25)
        self.assertEqual(runner.phase, "RUNNING")
        self.assertGreater(sim.traveled_m, .01)
        cursor = runner.status()["events_cursor"]
        refused = sim.daemon.command({"t": "goal", "goal": "forward"})
        self.assertEqual(refused["reason"], "strategy_active")
        paused_answer = sim.daemon.command({"t": "tour", "action": "pause"})
        self.assertTrue(paused_answer["accepted"])
        self.assertTrue(sim.daemon._tour_active)
        paused = sim.pose
        ticks(8)
        self.assertEqual(sim.pose, paused)
        self.assertTrue(sim.daemon.command({"t": "tour", "action": "resume"})["accepted"])
        ticks(15)
        self.assertGreater(runner.status()["events_cursor"], cursor)
        self.assertNotEqual(sim.pose, paused)
        stopped = sim.daemon.command({"t": "tour", "action": "stop"})
        self.assertTrue(stopped["accepted"])
        pose = sim.pose
        ticks(15)
        self.assertEqual(sim.pose, pose)
        self.assertEqual(sim.last_applied, (0., 0.))
        self.assertFalse(runner.holds_goal_lease())
        cursors = [event['seq'] for event in runner.events_since(0)]
        self.assertEqual(cursors, sorted(set(cursors)))


if __name__ == '__main__':
    unittest.main()
