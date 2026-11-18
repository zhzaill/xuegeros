import json
import tempfile
import time
import unittest
from pathlib import Path

from agent_control.cerebellum import Cerebellum
from agent_control.strategy import StrategyRunner
from sim.local_sim import LocalSimulator, Scene, StrategyDaemon, SIM_ASSETS


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.daemon = StrategyDaemon(Cerebellum(), strategy=StrategyRunner())
        scene = Scene.from_sdf(SIM_ASSETS / 'worlds' / 'room_open.world')
        self.sim = LocalSimulator(self.daemon, scene)
        self.sim.select_scene(scene)

    def test_changes_executor_and_runner_without_resetting_cursor(self):
        cursor = self.daemon.strategy.status()['events_cursor']
        self.sim.controls.apply({'max_linear_mps': .08, 'sectors': 24, 'geometry_weight': .8})
        self.assertEqual(self.daemon.cerebellum.max_linear, .08)
        self.assertEqual(self.daemon.strategy.max_linear, .08)
        self.assertAlmostEqual(self.daemon.cerebellum.priority_weight, .2)
        self.assertEqual(self.sim.update_world()[0].sector_count, 24)
        self.assertEqual(self.daemon.strategy.status()['events_cursor'], cursor)
        now = time.monotonic()
        self.sim.step(now, .02)
        self.sim.step(now+.02, .02)
        answer = self.daemon.command({'t':'goal', 'intent':'forward', 'distance_m':.1})
        self.assertTrue(answer['dispatched'])
        self.sim.step(now+.04, .02)
        self.assertAlmostEqual(self.sim.last_applied[0], .08)

    def test_rejects_invalid_values_atomically(self):
        old = self.sim.controls.snapshot()['values']
        for update in ({'max_linear_mps':.1, 'sectors':4.5}, {'scan_timeout_s':float('nan')},
                       {'max_linear_mps':True}, {'unknown':1}, {'geometry_weight':.1}):
            with self.assertRaises(ValueError): self.sim.controls.apply(update)
            self.assertEqual(old, self.sim.controls.snapshot()['values'])

    def test_busy_including_paused_strategy(self):
        self.daemon.strategy.start(strategy='tour')
        with self.assertRaises(RuntimeError): self.sim.controls.apply({'max_linear_mps':.1})
        self.daemon.strategy.pause()
        with self.assertRaises(RuntimeError): self.sim.controls.apply({'max_linear_mps':.1})

    def test_single_goal_blocks_changes(self):
        now = time.monotonic()
        self.sim.step(now, .02)
        self.sim.step(now+.02, .02)
        self.daemon.command({'t':'goal', 'intent':'forward', 'distance_m':.1})
        with self.assertRaises(RuntimeError): self.sim.controls.apply({'max_linear_mps':.1})

    def test_persistence_and_write_failure(self):
        with tempfile.TemporaryDirectory() as folder:
            self.sim.controls.path = Path(folder)/'control.json'
            self.sim.controls.apply({'max_angular_rps':.6})
            saved = json.loads(self.sim.controls.path.read_text())
            self.assertEqual(saved['max_angular_rps'], .6)
            self.daemon.cerebellum.max_angular = .2
            self.sim.controls.load()
            self.assertEqual(self.daemon.cerebellum.max_angular, .6)
            self.sim.controls.path = Path(folder)  # os.replace cannot replace a directory
            with self.assertRaises(OSError): self.sim.controls.apply({'max_angular_rps':.9})
            self.assertEqual(self.daemon.cerebellum.max_angular, .6)


if __name__ == '__main__': unittest.main()
