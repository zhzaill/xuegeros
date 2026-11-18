"""Transactional, persisted tuning for the local robotd simulation."""
from __future__ import annotations

import json
import math
import os
import tempfile
from pathlib import Path

# key: (Cerebellum attribute, title, lower bound, upper bound, integer)
PARAMETERS = {
    "max_linear_mps": ("max_linear", "线速度上限 · m/s", 0.001, 1, False),
    "max_angular_rps": ("max_angular", "角速度上限 · rad/s", 0.01, 3, False),
    "min_clearance_m": ("min_clearance", "前向净空 · m", 0.05, 2, False),
    "min_clearance_turn_m": ("min_clearance_turn", "转向净空 · m", 0.05, 2, False),
    "sectors": ("sectors", "雷达扇区数", 4, 72, True),
    "front_half_angle_rad": ("front_half_angle", "前向半角 · rad", 0.05, math.pi / 2, False),
    "forward_distance_m": ("forward_distance", "默认前进距离 · m", 0.01, 2, False),
    "forward_timeout_s": ("forward_timeout", "前进超时 · s", 0.1, 120, False),
    "turn_rad": ("turn_rad", "默认转角 · rad", 0.01, math.pi, False),
    "turn_timeout_s": ("turn_timeout", "转向超时 · s", 0.1, 120, False),
    "face_timeout_s": ("face_timeout", "朝向超时 · s", 0.1, 120, False),
    "open_space_distance_m": ("open_space_distance", "开阔处距离 · m", 0.01, 2, False),
    "open_space_timeout_s": ("open_space_timeout", "开阔处超时 · s", 0.1, 120, False),
    "max_steer_rad": ("max_steer", "最大偏转 · rad", 0.05, math.pi, False),
    "scan_timeout_s": ("scan_timeout", "扫描新鲜度 · s", 0.1, 10, False),
    "odom_timeout_s": ("odom_timeout", "里程计新鲜度 · s", 0.1, 10, False),
    "min_valid_samples": ("min_valid_samples", "最少有效射线", 1, 360, True),
    "stop_burst": ("stop_burst", "停车零速周期", 0, 50, True),
    "priority_ttl_s": ("priority_ttl", "优先级有效期 · s", 0.3, 60, False),
    "geometry_weight": ("geometry_weight", "几何评分权重", 0.5, 1, False),
    "score_range_m": ("score_range", "评分距离 · m", 0.3, 12, False),
}
STRATEGY_PARAMETERS = {
    "min_reverse_clearance_m": ("min_reverse_clearance", "后退净空 · m", 0.05, 2, False),
    "handover_after_s": ("handover_after", "受阻接管时间 · s", 1, 300, False),
    "refusal_limit": ("refusal_limit", "拒绝次数上限", 1, 100, True),
}


class ControlSettings:
    def __init__(self, sim, path: Path | None = None):
        self.sim = sim
        self.path = path

    def snapshot(self):
        with self.sim.control_lock:
            values, schema = {}, {}
            groups = [(self.sim.daemon.cerebellum, PARAMETERS)]
            if getattr(self.sim.daemon, 'strategy', None):
                groups.append((self.sim.daemon.strategy, STRATEGY_PARAMETERS))
            for owner, parameters in groups:
                for key, (attr, title, low, high, integer) in parameters.items():
                    values[key] = getattr(owner, attr)
                    schema[key] = dict(title=title, minimum=low, maximum=high,
                                       type="integer" if integer else "number")
            return dict(ok=True, values=values, schema=schema, busy=self.busy())

    def busy(self):
        control = self.sim.daemon.cerebellum
        return bool(getattr(self.sim.daemon, 'strategy_busy', False) or control.goal or control.guide)

    def apply(self, updates, *, persist=True):
        with self.sim.control_lock:
            if self.busy():
                raise RuntimeError("control_busy")
            before = self.snapshot()["values"]
            if not isinstance(updates, dict) or not updates or set(updates) - before.keys():
                raise ValueError("unknown_or_empty_parameters")
            specs = {**PARAMETERS, **STRATEGY_PARAMETERS}
            for key, value in updates.items():
                _, _, low, high, integer = specs[key]
                if (isinstance(value, bool) or not isinstance(value, (int, float))
                        or not math.isfinite(value) or not low <= value <= high
                        or (integer and int(value) != value)):
                    raise ValueError(f"invalid_parameter:{key}")
            values = {**before, **updates}
            # Write/replace first: a persistence failure must not change the executor.
            if persist and self.path:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                fd, temporary = tempfile.mkstemp(dir=self.path.parent, prefix=".control-", suffix=".json")
                try:
                    with os.fdopen(fd, "w", encoding="utf-8") as stream:
                        json.dump(values, stream, indent=2, allow_nan=False)
                        stream.flush()
                        os.fsync(stream.fileno())
                    os.replace(temporary, self.path)
                finally:
                    if os.path.exists(temporary):
                        os.unlink(temporary)
            control = self.sim.daemon.cerebellum
            for key, (attr, _, _, _, integer) in PARAMETERS.items():
                setattr(control, attr, int(values[key]) if integer else float(values[key]))
            control.priority_weight = 1 - control.geometry_weight
            # Old offers were computed under the old limits; force renegotiation.
            control._proposals.clear()
            control._last_profile = None
            strategy = getattr(self.sim.daemon, 'strategy', None)
            if strategy:
                for attr in ("max_linear", "max_angular", "min_clearance", "min_clearance_turn", "scan_timeout", "odom_timeout"):
                    setattr(strategy, attr, getattr(control, attr))
                strategy.obstacle_stop = control.min_clearance
                for key, (attr, _, _, _, integer) in STRATEGY_PARAMETERS.items():
                    setattr(strategy, attr, int(values[key]) if integer else float(values[key]))
            return self.snapshot()

    def load(self):
        if self.path and self.path.exists():
            self.apply(json.loads(self.path.read_text(encoding="utf-8")), persist=False)
