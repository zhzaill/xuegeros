"""Skill table: the file half of the cerebellum.

Plan P3 (``docs/PROJECT-PLAN-AND-CONTEXT.md`` P3) says behaviour must be a file,
not an if/else ladder. This module is that file's loader and validator:

* which skills exist (the vocabulary a goal may use),
* each skill's parameters, bounds and defaults,
* which **execution primitive** implements it,
* the preconditions it needs (documentation for the operator and the receipt;
  the geometry vetoes themselves live in ``Cerebellum.tick``).

Adding a skill that reuses an existing primitive is a file edit. An unknown
skill is refused with ``unknown_skill`` and the list of known ones, so a cortex
that guesses a new word gets a reason instead of silence.

The physical constants (max speed, clearances, timeouts) stay in
``cerebellum.py``: they are the vehicle's safety envelope, not policy. The table
only bounds what a request may ask for.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field
from typing import Any, Mapping

SCHEMA = "xuegeros.skills.v1"
PRIMITIVES = ("advance", "retreat", "turn_in_place", "turn_fixed", "best_open", "zero")
PARAM_KINDS = ("int", "float")


class SkillTableError(ValueError):
    """The table itself is malformed; the node should refuse to start."""


@dataclass(frozen=True)
class ParamSpec:
    name: str
    kind: str
    minimum: float
    maximum: float
    required: bool
    default: int | float | None

    def as_dict(self) -> dict[str, Any]:
        value: dict[str, Any] = {
            "type": self.kind,
            "min": self.minimum,
            "max": self.maximum,
            "required": self.required,
        }
        if self.default is not None:
            value["default"] = self.default
        return value


@dataclass(frozen=True)
class SkillSpec:
    name: str
    primitive: str
    params: Mapping[str, ParamSpec] = field(default_factory=dict)
    options: Mapping[str, float] = field(default_factory=dict)
    preconditions: tuple[str, ...] = ()
    note: str = ""

    def as_dict(self) -> dict[str, Any]:
        return {
            "primitive": self.primitive,
            "params": {name: spec.as_dict() for name, spec in self.params.items()},
            "options": dict(self.options),
            "preconditions": list(self.preconditions),
            "note": self.note,
        }


@dataclass(frozen=True)
class SkillTable:
    skills: Mapping[str, SkillSpec]
    sectors: int
    version: str
    source: str

    # -- vocabulary -------------------------------------------------------
    def names(self) -> list[str]:
        return sorted(self.skills)

    def spec(self, name: str) -> SkillSpec | None:
        return self.skills.get(name)

    def primitive(self, name: str) -> str | None:
        spec = self.skills.get(name)
        return spec.primitive if spec else None

    def option(self, name: str, key: str, default: float) -> float:
        spec = self.skills.get(name)
        if spec is None:
            return default
        value = spec.options.get(key, default)
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    # -- parameters -------------------------------------------------------
    def validate_params(self, name: str, payload: Mapping[str, Any]) -> tuple[dict[str, Any] | None, str | None]:
        """Return ``(values, None)`` or ``(None, error_code)``.

        Unknown names are *not* accepted here: ``unknown_skill`` must be checked
        first so the caller can report the known list.
        """

        spec = self.skills.get(name)
        if spec is None:
            return None, "unknown_skill"
        values: dict[str, Any] = {}
        for param_name, param in spec.params.items():
            raw = payload.get(param_name)
            if raw is None:
                if param.required:
                    return None, f"{param_name}_required"
                if param.default is not None:
                    values[param_name] = param.default
                continue
            if param.kind == "int":
                if isinstance(raw, bool) or not isinstance(raw, int):
                    return None, f"{param_name}_invalid"
                number: float = float(raw)
            else:
                if isinstance(raw, bool) or not isinstance(raw, (int, float)) or not math.isfinite(float(raw)):
                    return None, f"{param_name}_invalid"
                number = float(raw)
            if not param.minimum <= number <= param.maximum:
                return None, f"{param_name}_out_of_range"
            values[param_name] = int(number) if param.kind == "int" else float(number)
        return values, None

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema": SCHEMA,
            "version": self.version,
            "sectors": self.sectors,
            "source": self.source,
            "skills": {name: spec.as_dict() for name, spec in self.skills.items()},
        }


def default_path() -> str:
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "skills.json")


def _fail(message: str) -> "SkillTableError":
    return SkillTableError(message)


def _param_spec(name: str, raw: Any) -> ParamSpec:
    if not isinstance(raw, Mapping):
        raise _fail(f"skill param {name!r} must be an object")
    kind = str(raw.get("type", ""))
    if kind not in PARAM_KINDS:
        raise _fail(f"skill param {name!r} has unknown type {kind!r}")
    try:
        minimum = float(raw.get("min"))
        maximum = float(raw.get("max"))
    except (TypeError, ValueError):
        raise _fail(f"skill param {name!r} needs numeric min and max")
    if maximum < minimum:
        raise _fail(f"skill param {name!r} has max < min")
    required = bool(raw.get("required", False))
    default = raw.get("default")
    if default is not None:
        if isinstance(default, bool) or not isinstance(default, (int, float)):
            raise _fail(f"skill param {name!r} default must be a number")
        if not minimum <= float(default) <= maximum:
            raise _fail(f"skill param {name!r} default is outside [min, max]")
        default = int(default) if kind == "int" else float(default)
    return ParamSpec(
        name=name,
        kind=kind,
        minimum=minimum,
        maximum=maximum,
        required=required,
        default=default,
    )


def parse_table(value: Any, *, source: str) -> SkillTable:
    if not isinstance(value, Mapping):
        raise _fail("skill table must be a JSON object")
    if value.get("schema") != SCHEMA:
        raise _fail(f"schema must be {SCHEMA!r}")
    try:
        sectors = int(value.get("sectors", 12))
    except (TypeError, ValueError):
        raise _fail("sectors must be an integer")
    if not 1 <= sectors <= 64:
        raise _fail("sectors must be 1-64")
    skills_raw = value.get("skills")
    if not isinstance(skills_raw, Mapping) or not skills_raw:
        raise _fail("skills must be a non-empty object")

    skills: dict[str, SkillSpec] = {}
    for name, raw in skills_raw.items():
        if not isinstance(name, str) or not name:
            raise _fail("skill names must be non-empty strings")
        if not isinstance(raw, Mapping):
            raise _fail(f"skill {name!r} must be an object")
        primitive = str(raw.get("primitive", ""))
        if primitive not in PRIMITIVES:
            raise _fail(f"skill {name!r} has unknown primitive {primitive!r}")
        params_raw = raw.get("params", {})
        if not isinstance(params_raw, Mapping):
            raise _fail(f"skill {name!r} params must be an object")
        params = {param: _param_spec(param, spec) for param, spec in params_raw.items()}
        options_raw = raw.get("options", {})
        if not isinstance(options_raw, Mapping):
            raise _fail(f"skill {name!r} options must be an object")
        options: dict[str, float] = {}
        for key, option in options_raw.items():
            if isinstance(option, bool) or not isinstance(option, (int, float)) or not math.isfinite(float(option)):
                raise _fail(f"skill {name!r} option {key!r} must be a finite number")
            options[str(key)] = float(option)
        preconditions = raw.get("preconditions", [])
        if not isinstance(preconditions, list) or not all(isinstance(item, str) for item in preconditions):
            raise _fail(f"skill {name!r} preconditions must be a string list")
        skills[name] = SkillSpec(
            name=name,
            primitive=primitive,
            params=params,
            options=options,
            preconditions=tuple(preconditions),
            note=str(raw.get("note", ""))[:200],
        )
    return SkillTable(
        skills=skills,
        sectors=sectors,
        version=str(value.get("version", "1"))[:32],
        source=source,
    )


def load_table(path: str | None = None) -> SkillTable:
    """Load the table; a missing file falls back to the built-in copy.

    A *malformed* file is an error on purpose: silently running with defaults
    would make a typo in the skill table invisible.
    """

    target = path or default_path()
    try:
        with open(target, "r", encoding="utf-8") as handle:
            value = json.load(handle)
    except FileNotFoundError:
        return parse_table(builtin_table_json(), source="builtin")
    except (OSError, json.JSONDecodeError) as exc:
        raise SkillTableError(f"cannot read {target}: {exc}") from exc
    return parse_table(value, source=target)


def builtin_table_json() -> dict[str, Any]:
    """The table compiled in, used only when ``skills.json`` is absent.

    It is kept in sync with the shipped file by ``tests/test_skills.py``.
    """

    def param(kind: str, minimum: float, maximum: float, required: bool, default: float | None) -> dict[str, Any]:
        value: dict[str, Any] = {"type": kind, "min": minimum, "max": maximum, "required": required}
        if default is not None:
            value["default"] = default
        return value

    return {
        "schema": SCHEMA,
        "version": "1",
        "sectors": 12,
        "skills": {
            "forward": {
                "primitive": "advance",
                "params": {
                    "sector": param("int", 0, 11, False, None),
                    "distance_m": param("float", 0.02, 0.15, False, 0.12),
                },
                "options": {"turn_first_deg": 15.0},
                "preconditions": ["scan_fresh", "odom_fresh", "front_clear"],
                "note": "不给扇区就沿当前车头直行；给了扇区才先转到扇区中心再走（12 扇区中心落在 ±15°，没有正对车头的扇区，所以默认值不能是 6）",
            },
            "back": {
                "primitive": "retreat",
                "params": {
                    "sector": param("int", 0, 11, False, None),
                    "distance_m": param("float", 0.02, 0.15, False, 0.10),
                },
                "preconditions": ["scan_fresh", "odom_fresh", "rear_clear"],
                "note": "直线后退一小步；按请求扇区的反方向检查净空，不带转向",
            },
            "face": {
                "primitive": "turn_in_place",
                "params": {"sector": param("int", 0, 11, True, None)},
                "preconditions": ["scan_fresh", "odom_fresh", "turn_clear"],
                "note": "原地转到扇区中心",
            },
            "open_space": {
                "primitive": "best_open",
                "params": {"distance_m": param("float", 0.02, 0.20, False, 0.20)},
                "preconditions": ["scan_fresh", "odom_fresh", "front_clear"],
                "note": "挑最空的扇区前进",
            },
            "left": {"primitive": "turn_fixed", "params": {}, "preconditions": ["scan_fresh", "odom_fresh", "turn_clear"]},
            "right": {"primitive": "turn_fixed", "params": {}, "preconditions": ["scan_fresh", "odom_fresh", "turn_clear"]},
            "stop": {"primitive": "zero", "params": {}, "preconditions": []},
        },
    }
