import hashlib
import math
import time
import unittest
from pathlib import Path

from agent_control.cerebellum import Cerebellum
from sim.local_sim import (
    SIM_ASSETS,
    LocalSimulator,
    Scene,
    StrategyDaemon,
    maps_payload,
    model_metadata,
    sensors_payload,
)


class SceneGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.room = Scene.from_sdf(SIM_ASSETS / "worlds" / "room.world")

    def test_floor_is_not_an_obstacle_or_scene_boundary(self):
        self.assertAlmostEqual(self.room.width, 6.2)
        self.assertAlmostEqual(self.room.height, 6.2)
        self.assertFalse(any(item["name"].startswith("ground/") for item in self.room.obstacles))
        self.assertFalse(self.room.collides(0.0, 0.0))

    def test_rotated_box_ray_requires_overlap_on_both_axes(self):
        # The ray points east and clears crate_a; the old slab test returned a
        # false hit because its x/y intervals never overlapped.
        self.assertAlmostEqual(self.room.distance((0.0, 0.0, 0.0), 0.0), 3.0227, places=3)

    def test_lidar_hits_crate_and_room_walls(self):
        angle_to_crate = math.atan2(1.1, 1.2)
        self.assertLess(self.room.distance((0.0, 0.0, 0.0), angle_to_crate), 1.7)
        self.assertAlmostEqual(self.room.distance((0.0, 0.0, 0.0), math.pi / 2), 3.0, places=2)

    def test_collision_uses_rotated_box_and_cylinder_footprints(self):
        self.assertTrue(self.room.collides(1.2, 1.1))
        self.assertTrue(self.room.collides(-1.4, 1.6))
        self.assertFalse(self.room.collides(0.0, 0.0))
        self.assertFalse(self.room.collides(0.5, 1.1))

    def test_shadow_metadata_does_not_claim_gazebo_physics(self):
        metadata = self.room.as_json()
        self.assertEqual(metadata["backend"], "kinematic-shadow")
        self.assertTrue(any("不含 Gazebo 接触、质量、摩擦或动力学响应" in item
                            for item in metadata["approximations"]))


class SimPayloadTests(unittest.TestCase):
    def setUp(self):
        self.control = Cerebellum()
        self.daemon = StrategyDaemon(self.control, physical=False)
        self.room = Scene.from_sdf(SIM_ASSETS / "worlds" / "room.world")
        self.sim = LocalSimulator(self.daemon, self.room, hz=50.0)
        self.sim.select_scene(self.room)

    def test_truth_map_names_its_world_hash_and_version(self):
        truth = maps_payload(self.sim)["maps"]["truth"]
        expected = hashlib.sha256((SIM_ASSETS / "worlds" / "room.world").read_bytes()).hexdigest()
        self.assertEqual(truth["world_sha256"], expected)
        self.assertEqual(truth["map_version"], f"truth/{expected[:12]}")
        self.assertEqual(truth["known_cells"], 120 * 120)
        self.assertIn("known_cells_basis", truth)
        self.assertTrue(truth["known_cells_basis"])
        self.assertEqual(truth["resolution_m"], 0.05)
        self.assertEqual(truth["availability"], "ok")

    def test_truth_version_changes_with_the_scene(self):
        before = maps_payload(self.sim)["maps"]["truth"]["map_version"]
        opened = Scene.from_sdf(SIM_ASSETS / "worlds" / "room_open.world")
        self.sim.select_scene(opened)
        after = maps_payload(self.sim)["maps"]["truth"]
        self.assertNotEqual(before, after["map_version"])
        expected = hashlib.sha256((SIM_ASSETS / "worlds" / "room_open.world").read_bytes()).hexdigest()
        self.assertEqual(after["world_sha256"], expected)

    def test_ages_come_from_frame_stamps_not_from_a_constant(self):
        payload = sensors_payload(self.sim)
        self.assertEqual(payload["age_source"], "loop_frame_stamp")
        lidar = next(item for item in payload["sensors"] if item["id"] == "lidar")
        self.assertIsNone(lidar["age_s"])
        self.assertEqual(lidar["availability"], "missing")

        self.sim.last_scan_at = time.monotonic()
        self.sim.last_odom_at = time.monotonic()
        fresh = sensors_payload(self.sim)
        lidar = next(item for item in fresh["sensors"] if item["id"] == "lidar")
        self.assertLess(lidar["age_s"], 0.5)
        self.assertEqual(lidar["availability"], "ok")

        time.sleep(0.2)
        later = sensors_payload(self.sim)
        lidar = next(item for item in later["sensors"] if item["id"] == "lidar")
        self.assertGreater(lidar["age_s"], fresh["sensors"][0]["age_s"])

    def test_a_stalled_loop_shows_up_as_stale(self):
        self.sim.last_scan_at = time.monotonic() - 5.0
        self.sim.last_odom_at = time.monotonic() - 5.0
        payload = sensors_payload(self.sim)
        by_id = {item["id"]: item for item in payload["sensors"]}
        self.assertEqual(by_id["lidar"]["availability"], "stale")
        self.assertEqual(by_id["wheel_odom"]["availability"], "stale")
        self.assertGreater(by_id["lidar"]["age_s"], 4.0)

    def test_http_server_backlog_is_raised(self):
        from sim.local_sim import SimHTTPServer

        self.assertGreaterEqual(SimHTTPServer.request_queue_size, 64)


if __name__ == "__main__":
    unittest.main()
