import json
import math
import os
import tempfile
import unittest

from agent_control import skills


class ShippedTableTests(unittest.TestCase):
    def test_shipped_table_loads_and_declares_the_v1_vocabulary(self):
        table = skills.load_table()
        self.assertEqual(table.version, "1")
        self.assertEqual(table.sectors, 12)
        self.assertTrue(table.source.endswith("skills.json"))
        self.assertEqual(table.names(), ["back", "face", "forward", "left", "open_space", "right", "stop"])
        self.assertEqual(table.primitive("back"), "retreat")
        self.assertEqual(table.primitive("forward"), "advance")
        self.assertEqual(table.primitive("face"), "turn_in_place")
        self.assertEqual(table.primitive("stop"), "zero")
        self.assertEqual(table.primitive("left"), "turn_fixed")
        self.assertEqual(table.primitive("open_space"), "best_open")

    def test_builtin_copy_matches_the_shipped_file(self):
        # The builtin is the fallback when the file is missing; if the two drift,
        # an installed package without data files would behave differently.
        # Notes are documentation, so they may differ; vocabulary and bounds may not.
        def strip_notes(value):
            for skill in value["skills"].values():
                skill.pop("note", None)
            return value

        builtin = strip_notes(skills.parse_table(skills.builtin_table_json(), source="builtin").as_dict())
        shipped = strip_notes(skills.load_table().as_dict())
        builtin.pop("source", None)
        shipped.pop("source", None)
        self.assertEqual(builtin, shipped)

    def test_forward_declares_turn_first_and_bounds(self):
        table = skills.load_table()
        forward = table.spec("forward")
        self.assertIsNotNone(forward)
        self.assertAlmostEqual(table.option("forward", "turn_first_deg", 99.0), 15.0)
        distance = forward.params["distance_m"]
        self.assertAlmostEqual(distance.minimum, 0.02)
        self.assertAlmostEqual(distance.maximum, 0.15)
        self.assertAlmostEqual(float(distance.default), 0.12)
        sector = forward.params["sector"]
        self.assertFalse(sector.required)
        # No default on purpose: a sector centre is never straight ahead in a
        # 12-sector scan, so defaulting to one silently veers the car.
        self.assertIsNone(sector.default)

    def test_face_requires_a_sector(self):
        table = skills.load_table()
        face = table.spec("face")
        self.assertTrue(face.params["sector"].required)
        self.assertIsNone(face.params["sector"].default)


class ParameterValidationTests(unittest.TestCase):
    def setUp(self):
        self.table = skills.load_table()

    def test_defaults_and_explicit_values(self):
        values, error = self.table.validate_params("forward", {})
        self.assertIsNone(error)
        self.assertNotIn("sector", values)  # no sector means "straight ahead"
        self.assertAlmostEqual(values["distance_m"], 0.12)

        values, error = self.table.validate_params("forward", {"sector": 9, "distance_m": 0.15})
        self.assertIsNone(error)
        self.assertEqual(values["sector"], 9)
        self.assertAlmostEqual(values["distance_m"], 0.15)

    def test_bounds_and_types(self):
        cases = [
            ({"sector": 12}, "sector_out_of_range"),
            ({"sector": -1}, "sector_out_of_range"),
            ({"sector": "9"}, "sector_invalid"),
            ({"sector": True}, "sector_invalid"),
            ({"distance_m": 0.2}, "distance_m_out_of_range"),
            ({"distance_m": 0.0}, "distance_m_out_of_range"),
            ({"distance_m": float("nan")}, "distance_m_invalid"),
            ({"distance_m": "0.1"}, "distance_m_invalid"),
        ]
        for payload, expected in cases:
            with self.subTest(payload=payload):
                _, error = self.table.validate_params("forward", payload)
                self.assertEqual(error, expected)

    def test_required_parameter(self):
        _, error = self.table.validate_params("face", {})
        self.assertEqual(error, "sector_required")
        values, error = self.table.validate_params("face", {"sector": 3})
        self.assertIsNone(error)
        self.assertEqual(values["sector"], 3)

    def test_unknown_skill_is_reported_not_guessed(self):
        values, error = self.table.validate_params("swivel", {})
        self.assertIsNone(values)
        self.assertEqual(error, "unknown_skill")


class TableErrorTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.path = os.path.join(self._tmp.name, "skills.json")

    def write(self, value):
        with open(self.path, "w", encoding="utf-8") as handle:
            json.dump(value, handle, ensure_ascii=False)

    def base(self):
        return {
            "schema": skills.SCHEMA,
            "version": "1",
            "sectors": 12,
            "skills": {"stop": {"primitive": "zero", "params": {}}},
        }

    def test_bad_schema(self):
        value = self.base()
        value["schema"] = "v0"
        self.write(value)
        with self.assertRaises(skills.SkillTableError) as raised:
            skills.load_table(self.path)
        self.assertIn("schema", str(raised.exception))

    def test_unknown_primitive(self):
        value = self.base()
        value["skills"]["stop"]["primitive"] = "teleport"
        self.write(value)
        with self.assertRaises(skills.SkillTableError) as raised:
            skills.load_table(self.path)
        self.assertIn("teleport", str(raised.exception))

    def test_bad_parameter_bounds(self):
        value = self.base()
        value["skills"]["stop"]["params"] = {"n": {"type": "int", "min": 10, "max": 1, "required": False}}
        self.write(value)
        with self.assertRaises(skills.SkillTableError) as raised:
            skills.load_table(self.path)
        self.assertIn("max < min", str(raised.exception))

    def test_default_outside_bounds(self):
        value = self.base()
        value["skills"]["stop"]["params"] = {"n": {"type": "int", "min": 0, "max": 1, "default": 5}}
        self.write(value)
        with self.assertRaises(skills.SkillTableError) as raised:
            skills.load_table(self.path)
        self.assertIn("default", str(raised.exception))

    def test_bad_json_is_loud(self):
        with open(self.path, "w", encoding="utf-8") as handle:
            handle.write("{oops")
        with self.assertRaises(skills.SkillTableError):
            skills.load_table(self.path)

    def test_missing_file_falls_back_to_builtin(self):
        table = skills.load_table(os.path.join(self._tmp.name, "absent.json"))
        self.assertEqual(table.source, "builtin")
        self.assertIn("forward", table.names())

    def test_minimal_table_is_accepted(self):
        self.write(self.base())
        table = skills.load_table(self.path)
        self.assertEqual(table.names(), ["stop"])
        self.assertEqual(table.source, self.path)


class CustomVocabularyTests(unittest.TestCase):
    """A skill the file adds must work without touching cerebellum.py."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.path = os.path.join(self._tmp.name, "skills.json")
        value = skills.builtin_table_json()
        value["skills"]["swivel"] = {
            "primitive": "turn_in_place",
            "params": {"sector": {"type": "int", "min": 0, "max": 11, "required": True}},
            "preconditions": ["turn_clear"],
            "note": "test-only alias of face",
        }
        with open(self.path, "w", encoding="utf-8") as handle:
            json.dump(value, handle, ensure_ascii=False)
        self.table = skills.load_table(self.path)

    def test_custom_skill_is_listed_and_validated(self):
        self.assertIn("swivel", self.table.names())
        values, error = self.table.validate_params("swivel", {"sector": 10})
        self.assertIsNone(error)
        self.assertEqual(values["sector"], 10)


if __name__ == "__main__":
    unittest.main()
