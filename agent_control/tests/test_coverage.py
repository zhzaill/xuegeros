import json
import math
import tempfile
import unittest
from pathlib import Path

from agent_control.coverage import (
    DONE,
    LANE,
    LIMIT,
    RETURN,
    SHIFT,
    STOPPED,
    TURN,
    VARIANT_SCHEMA,
    BareGeometryRejected,
    CoverageController,
    VariantError,
    available_variants,
    load_variant,
    variant_path,
)

FULL_TURN = 2.0 * math.pi


def ranges(front=2.0, other=2.0, points=360):
    angle_min = -math.pi
    increment = FULL_TURN / points
    values = []
    for index in range(points):
        angle = angle_min + index * increment
        values.append(front if abs(angle) < math.radians(20) else other)
    return values, angle_min, increment


def start(control, pose=(0.0, 0.0, 0.0), now=0.0):
    values, angle_min, increment = ranges()
    return control.start(
        now=now,
        request_id="cov-1",
        source="test",
        pose=pose,
        ranges=values,
        angle_min=angle_min,
        angle_increment=increment,
    )


def step(control, pose, now, front=2.0):
    values, angle_min, increment = ranges(front=front)
    return control.step(
        now=now,
        pose=pose,
        ranges=values,
        angle_min=angle_min,
        angle_increment=increment,
        facts_due=True,
    )


class CoverageTests(unittest.TestCase):
    def test_start_drives_forward_along_the_first_lane(self):
        control = CoverageController()
        start(control)
        action, facts = step(control, (0.2, 0.0, 0.02), 1.0)
        self.assertEqual(control.phase, LANE)
        self.assertGreater(action.linear_mps, 0.0)
        self.assertEqual(facts["lane_index"], 0)
        self.assertEqual(facts["lane_dir"], 1)

    def test_obstacle_ends_the_lane(self):
        control = CoverageController()
        start(control)
        _, facts = step(control, (0.1, 0.0, 0.0), 1.0, front=0.15)
        self.assertEqual(control.phase, TURN)
        self.assertEqual(facts["blocked_count"], 1)

    def test_turn_shift_and_reverse_lane(self):
        control = CoverageController()
        start(control)
        step(control, (1.0, 0.0, 0.0), 1.0)
        self.assertEqual(control.phase, TURN)

        step(control, (1.0, 0.0, math.pi / 2), 2.0)
        self.assertEqual(control.phase, SHIFT)

        step(control, (1.0, 0.6, math.pi / 2), 3.0)
        self.assertEqual(control.phase, TURN)
        self.assertEqual(control._lane_index, 1)
        self.assertEqual(control._lane_dir, -1)

        step(control, (1.0, 0.6, math.pi), 4.0)
        self.assertEqual(control.phase, LANE)

    def test_last_lane_returns_home(self):
        control = CoverageController(max_lanes=1)
        start(control)
        step(control, (1.0, 0.0, 0.0), 1.0)
        step(control, (1.0, 0.0, math.pi / 2), 2.0)
        step(control, (1.0, 0.6, math.pi / 2), 3.0)
        self.assertEqual(control.phase, RETURN)

        _, facts = step(control, (0.05, 0.0, 0.0), 4.0)
        self.assertEqual(control.phase, DONE)
        self.assertEqual(facts["reason"], "returned_home")

    def test_runtime_limit_stops_the_sweep(self):
        control = CoverageController(max_runtime_s=5.0)
        start(control)
        action, facts = step(control, (0.0, 0.0, 0.0), 6.0)
        self.assertEqual(control.phase, LIMIT)
        self.assertEqual(facts["reason"], "runtime_limit")
        self.assertEqual(action.linear_mps, 0.0)

    def test_no_progress_stops_the_sweep(self):
        control = CoverageController(no_progress_timeout_s=5.0)
        start(control)
        step(control, (0.0, 0.0, 0.0), 1.0)
        _, facts = step(control, (0.0, 0.0, 0.0), 6.0)
        self.assertEqual(control.phase, LIMIT)
        self.assertEqual(facts["reason"], "no_progress")

    def test_lane_progress_follows_the_odom_pose(self):
        # The adapter feeds coverage the odom x/y + IMU yaw pose; lane progress
        # must grow with that pose, not with the lagging map pose.
        control = CoverageController()
        start(control)
        _, first = step(control, (0.05, 0.0, 0.0), 1.0)
        _, second = step(control, (0.25, 0.0, 0.0), 2.0)
        self.assertEqual(first["frame"], "odom")
        self.assertGreater(second["lane_progress_m"], first["lane_progress_m"])

    def test_stop_ends_an_active_sweep(self):
        control = CoverageController()
        start(control)
        facts = control.stop(reason="operator")
        self.assertEqual(control.phase, STOPPED)
        self.assertEqual(facts["reason"], "operator")
        self.assertFalse(control.active)


def variant_document(variant_id="room/test/01", **overrides):
    parts = variant_id.split("/")
    document = {
        "schema": VARIANT_SCHEMA,
        "variant_id": variant_id,
        "scene_family": parts[0],
        "world": "sim/xuegeros_sim/worlds/room.world",
        "world_sha256": None,
        "instruction_id": "sweep-room/zh/v1",
        "instruction_text": "把这个房间扫完，最后回到起点。",
        "seed": 11,
        "start_pose": [0.0, 0.0, 0.0],
        "lane": {"length_m": 0.9, "width_m": 0.5, "max_lanes": 2},
        "acceptance": {
            "min_known_cells": 900,
            "max_runtime_s": 240,
            "return_distance_m": 0.2,
            "no_forbidden_state": ["no_progress"],
        },
    }
    document.update(overrides)
    return document


def write_document(root, document, variant_id="room/test/01"):
    parts = variant_id.split("/")
    path = Path(root) / parts[0] / parts[1] / f"{parts[2]}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document), encoding="utf-8")
    return path


class VariantTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def test_published_variants_are_listed_and_parse(self):
        ids = available_variants()
        self.assertIn("room/default/01", ids)
        self.assertIn("room_open/wide/01", ids)
        variant = load_variant("room/default/01")
        self.assertEqual(variant.variant_id, "room/default/01")
        self.assertEqual(variant.scene_family, "room")
        self.assertEqual(variant.lane_length_m, 1.0)
        self.assertEqual(variant.lane_width_m, 0.6)
        self.assertEqual(variant.max_lanes, 4)
        self.assertEqual(variant.instruction_id, "sweep-room/zh/v1")
        self.assertEqual(variant.source, "sim/contracts/variants/room/default/01.json")
        self.assertEqual(set(variant.acceptance), {
            "min_known_cells",
            "max_runtime_s",
            "return_distance_m",
            "no_forbidden_state",
        })

    def test_world_hash_is_measured_not_hand_written(self):
        variant = load_variant("room/default/01")
        self.assertEqual(variant.world_sha256_source, "computed")
        self.assertRegex(variant.world_sha256 or "", r"^[0-9a-f]{64}$")

    def test_variant_id_maps_to_a_file_under_sim_contracts(self):
        path = variant_path("room/crate-diagonal/01")
        self.assertEqual(
            path,
            Path(__file__).resolve().parents[2] / "sim" / "contracts" / "variants"
            / "room" / "crate-diagonal" / "01.json",
        )
        self.assertTrue(path.is_file())

    def test_unknown_variant_lists_the_available_ones(self):
        with self.assertRaises(VariantError) as caught:
            load_variant("room/ghost/99")
        self.assertEqual(caught.exception.code, "unknown_variant")
        self.assertIn("room/default/01", caught.exception.available)

    def test_variant_id_must_have_three_segments(self):
        for bad in ("room/default", "room/default/01/extra", "", "room/../01"):
            with self.subTest(bad=bad):
                with self.assertRaises(VariantError) as caught:
                    load_variant(bad)
                self.assertEqual(caught.exception.code, "invalid_variant_id")

    def test_variant_id_in_the_file_must_match_the_path(self):
        write_document(self.root, variant_document("room/other/02"))
        with self.assertRaises(VariantError) as caught:
            load_variant("room/test/01", root=self.root)
        self.assertEqual(caught.exception.code, "invalid_variant")
        self.assertIn("variant_id", str(caught.exception))

    def test_lane_geometry_must_be_positive(self):
        document = variant_document()
        document["lane"]["length_m"] = 0.0
        write_document(self.root, document)
        with self.assertRaises(VariantError) as caught:
            load_variant("room/test/01", root=self.root)
        self.assertIn("lane.length_m", str(caught.exception))

    def test_acceptance_must_declare_its_metrics(self):
        document = variant_document()
        del document["acceptance"]["return_distance_m"]
        write_document(self.root, document)
        with self.assertRaises(VariantError) as caught:
            load_variant("room/test/01", root=self.root)
        self.assertEqual(caught.exception.code, "invalid_variant")
        self.assertIn("return_distance_m", str(caught.exception))

    def test_schema_must_be_named(self):
        document = variant_document(schema="xuegeros.coverage.variant.v0")
        write_document(self.root, document)
        with self.assertRaises(VariantError):
            load_variant("room/test/01", root=self.root)

    def test_valid_temp_variant_drives_the_controller_geometry(self):
        write_document(self.root, variant_document())
        control = CoverageController(variant_id="room/test/01", variant_root=self.root)
        self.assertEqual(control.lane_length, 0.9)
        self.assertEqual(control.lane_width, 0.5)
        self.assertEqual(control.max_lanes, 2)
        facts = control.snapshot()
        self.assertEqual(facts["variant_id"], "room/test/01")
        self.assertEqual(facts["instruction_id"], "sweep-room/zh/v1")
        self.assertEqual(facts["acceptance"]["max_runtime_s"], 240.0)

    def test_bare_geometry_is_refused_with_the_variant_list(self):
        with self.assertRaises(BareGeometryRejected) as caught:
            CoverageController(lane_length_m=0.5, lane_width_m=0.4)
        self.assertEqual(caught.exception.code, "bare_geometry")
        self.assertIn("room/default/01", caught.exception.available)
        self.assertIn("variant_id", str(caught.exception))


if __name__ == "__main__":
    unittest.main()
