"""Geometry-only cerebellum: a semantic goal becomes one bounded motion.

The cortex (any LLM, including a weak or local one) never sends wheel speeds.
It sends a short goal such as "forward" or "左转"; this module decides what that
means in terms of scan geometry and odometry, and the ROS adapter publishes it.

Two rules make the split safe:

* The veto is absolute. No goal can move the robot while the scan shows an
  obstacle inside ``min_clearance``; the goal is cancelled and the reason goes
  back to the cortex.
* Everything here is deterministic and ROS-free, so the whole decision surface
  is covered by offline tests. The ROS node is a thin adapter.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field, replace
from typing import Any, Mapping, Sequence

from .skills import SkillTable, load_table

STOP = "stop"
FORWARD = "forward"
LEFT = "left"
RIGHT = "right"
OPEN_SPACE = "open_space"# Turn in place until a lidar sector is centred. `look` is the *cortex* word for
# this; on the executor side it is named `face` so it can never be confused with
# the VLM "look at one frame" tool.
FACE = "face"

CANCEL_STOP = "stop"
CANCEL_VETO = "veto"
CANCEL_REACHED = "goal_reached"
CANCEL_TIMEOUT = "timeout"
CANCEL_STALE = "stale"
CANCEL_SUPERSEDED = "superseded"

IDLE = "IDLE"
ACTIVE = "ACTIVE"
DONE = "DONE"
VETO = "VETO"
BLOCKED = "BLOCKED"
REJECTED = "REJECTED"

# Aliases exist so a weak model can answer in plain Chinese or English without
# guessing a schema. Keep this table the single source of accepted names.
GOAL_ALIASES: dict[str, str] = {}
for _canonical, _names in (
    (
        FORWARD,
        ("forward", "forward_small", "advance", "go_forward", "move_forward",
         "前进", "前进一小段", "向前", "向前走", "往前", "直行"),
    ),
    (LEFT, ("left", "turn_left", "左转", "向左", "向左转", "左")),
    (RIGHT, ("right", "turn_right", "右转", "向右", "向右转", "右")),
    (STOP, ("stop", "halt", "stop_now", "停", "停止", "停车", "急停", "停下")),
    (
        OPEN_SPACE,
        ("open_space", "explore", "free_space", "go_open",
         "空旷", "去前方空旷处", "开阔处", "自由空间", "探索", "找空地方"),
    ),
    (
        FACE,
        ("face", "turn_to", "face_sector", "look", "look_that_way",
         "转向", "朝向", "转到", "转过去", "面朝"),
    ),
):
    for _name in _names:
        GOAL_ALIASES[_name] = _canonical
        GOAL_ALIASES[_name.replace("_", " ")] = _canonical
del _canonical, _names, _name

ACCEPTED_GOALS = (FORWARD, LEFT, RIGHT, STOP, OPEN_SPACE, FACE)

# Proportional turn gain for FACE: fast enough to close 30 deg in one bounded
# motion, gentle enough that the 30 deg sector grid does not overshoot.
FACE_GAIN = 1.5
FACE_TOLERANCE_RAD = math.radians(3.0)
# The 30 deg sector grid puts a sector centre exactly on the 15 deg turn-first
# boundary; without a tolerance the float noise flips the phase.
TURN_FIRST_EPS = 1e-6

# Negotiation, not a human timer: a proposal is the AI's ask, the cerebellum
# answers with a bounded counter-offer, and the answer is only valid while the
# scan that produced it is still fresh. Nothing here grants a standing window.
PROPOSAL_TTL_S = 3.0
STOP_MARGIN_M = 0.05
PROPOSAL_HISTORY = 32


def _rear_min(profile: "ScanProfile", sector: int | None) -> float:
    """Clearance behind the car, on the sector opposite the request."""

    if profile.sector_count <= 0:
        return float("inf")
    if sector is not None:
        return profile.sector_min[(int(sector) + profile.sector_count // 2) % profile.sector_count]
    # Sector 0 is -165 deg and sector count/2 is +15 deg, so the pair behind
    # the car straddles +-180 deg, not the front pair.
    center = profile.sector_count // 2
    rear_a = (center + profile.sector_count // 2) % profile.sector_count
    rear_b = (rear_a - 1) % profile.sector_count
    return min(profile.sector_min[rear_a], profile.sector_min[rear_b])


def normalize_goal(value: Any) -> str | None:
    """Return the canonical goal name, or None when the word is unknown."""

    if not isinstance(value, str):
        return None
    text = value.strip().strip("。.!！?？")
    if not text:
        return None
    key = text.lower()
    if key in GOAL_ALIASES:
        return GOAL_ALIASES[key]
    collapsed = " ".join(key.split())
    return GOAL_ALIASES.get(collapsed)


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _wrap_angle(value: float) -> float:
    """Fold an angle into (-pi, pi]; ±165 deg must not look like ±195 deg."""

    wrapped = (value + math.pi) % (2.0 * math.pi) - math.pi
    return wrapped


@dataclass(frozen=True)
class ScanProfile:
    """Per-sector minimum range plus the frontal clearance.

    A missing sector (no finite return) is reported as ``inf``. That is the
    normal case for open space; a dead lidar is caught by staleness instead.
    """

    sector_min: tuple[float, ...]
    sector_count: int
    front_min: float
    left_min: float
    right_min: float
    valid_count: int
    blind_fraction: float

    def sector_angle(self, index: int) -> float:
        """Centre angle of a sector in the laser frame (0 = straight ahead)."""

        width = 2.0 * math.pi / self.sector_count
        return -math.pi + (index + 0.5) * width

    def best_sector(self, max_angle_rad: float) -> tuple[int, float, float]:
        """Freest sector within ``max_angle_rad`` of straight ahead.

        Returns ``(index, centre_angle_rad, clearance_m)``. Ties resolve toward
        the smaller absolute angle, so the robot prefers the least deviation.
        """

        best_index = 0
        best_angle = 0.0
        best_clearance = -1.0
        for index in range(self.sector_count):
            angle = self.sector_angle(index)
            if abs(angle) > max_angle_rad:
                continue
            clearance = self.sector_min[index]
            if clearance > best_clearance or (
                clearance == best_clearance and abs(angle) < abs(best_angle)
            ):
                best_index, best_angle, best_clearance = index, angle, clearance
        return best_index, best_angle, best_clearance

    def as_list(self) -> list[float | None]:
        return [None if not math.isfinite(value) else round(value, 3) for value in self.sector_min]


def profile_scan(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    *,
    sectors: int = 12,
    front_half_angle_rad: float = math.radians(25.0),
    min_range_m: float = 0.05,
    max_range_m: float = 12.0,
) -> ScanProfile:
    """Fold a LaserScan into per-sector minima.

    Returns an empty profile (all ``inf``, zero valid samples) when no range is
    usable; callers treat that as "no geometry" and stay stopped.
    """

    if sectors < 1:
        raise ValueError("sectors must be >= 1")
    sector_min = [float("inf")] * sectors
    front_min = float("inf")
    left_min = float("inf")
    right_min = float("inf")
    valid_count = 0
    width = 2.0 * math.pi / sectors

    for index, raw in enumerate(ranges):
        if not _finite(raw):
            continue
        value = float(raw)
        if value < min_range_m or value > max_range_m:
            continue
        angle = angle_min + index * angle_increment
        angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
        valid_count += 1
        sector = int(math.floor((angle + math.pi) / width))
        if sector >= sectors:
            sector = sectors - 1
        if value < sector_min[sector]:
            sector_min[sector] = value
        if abs(angle) <= front_half_angle_rad:
            front_min = min(front_min, value)
        elif 0.0 < angle <= math.pi / 2.0:
            left_min = min(left_min, value)
        elif -math.pi / 2.0 <= angle < 0.0:
            right_min = min(right_min, value)

    blind = sum(1 for value in sector_min if not math.isfinite(value))
    return ScanProfile(
        sector_min=tuple(sector_min),
        sector_count=sectors,
        front_min=front_min,
        left_min=left_min,
        right_min=right_min,
        valid_count=valid_count,
        blind_fraction=blind / sectors,
    )


@dataclass(frozen=True)
class GoalSpec:
    kind: str
    goal_id: str
    note: str
    distance_m: float
    timeout_s: float
    turn_rad: float
    requested_at: float
    sector: int | None = None
    # "advance" skills may start with a turn phase; "" means "not decided yet".
    phase: str = ""


@dataclass(frozen=True)
class ParsedGoal:
    kind: str
    distance_m: float | None = None
    timeout_s: float | None = None
    note: str = ""
    sector: int | None = None
    error: str | None = None


def parse_goal(payload: Mapping[str, Any]) -> ParsedGoal:
    """Validate a goal request arriving over ``/xuegeros/goal``.

    Only shape is validated here; bounds are re-applied by ``Cerebellum``.
    ``duration_ms`` and ``distance_m`` are optional overrides, not requirements.
    """

    if not isinstance(payload, Mapping):
        return ParsedGoal(kind="", error="goal_must_be_object")
    kind = normalize_goal(payload.get("goal", payload.get("command")))
    if kind is None:
        return ParsedGoal(kind="", error="unknown_goal")
    distance: float | None = None
    if payload.get("distance_m") is not None:
        value = payload.get("distance_m")
        if not _finite(value) or not 0.02 <= float(value) <= 0.5:
            return ParsedGoal(kind=kind, error="distance_out_of_range")
        distance = float(value)
    timeout: float | None = None
    if payload.get("duration_ms") is not None:
        value = payload.get("duration_ms")
        if not _finite(value) or not 100 <= float(value) <= 8000:
            return ParsedGoal(kind=kind, error="duration_out_of_range")
        timeout = float(value) / 1000.0
    sector: int | None = None
    if payload.get("sector") is not None:
        value = payload.get("sector")
        if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 63:
            return ParsedGoal(kind=kind, error="invalid_sector")
        sector = int(value)
    note = str(payload.get("note", ""))[:120]
    return ParsedGoal(kind=kind, distance_m=distance, timeout_s=timeout, note=note, sector=sector)


@dataclass(frozen=True)
class Proposal:
    """A bounded counter-offer the AI may execute once, while it stays fresh."""

    proposal_id: str
    skill: str
    sector: int | None
    requested_distance_m: float
    distance_m: float
    created_at: float
    expires_at: float
    payload: dict[str, Any]


@dataclass(frozen=True)
class PriorityGuide:
    """A short-lived L2 opinion: veto plus a per-sector preference.

    The weak model never says "turn left at 0.3 rad/s". It says "these sectors
    look good, that one does not, and stop if X". The geometry layer decides
    the actual wheel commands and keeps its own absolute veto.
    """

    veto: bool
    reason: str
    priority: tuple[float, ...]
    mode: str
    set_at: float
    expires_at: float


@dataclass(frozen=True)
class CerebellumDecision:
    linear_mps: float
    angular_rps: float
    state: str
    goal: str | None
    goal_id: str | None
    reason: str
    publish: bool
    detail: Mapping[str, Any] = field(default_factory=dict)

    def as_status(self) -> dict[str, Any]:
        return {
            "state": self.state,
            "goal": self.goal,
            "goal_id": self.goal_id,
            "reason": self.reason,
            "publish": self.publish,
            **dict(self.detail),
        }


class Cerebellum:
    """Goal state machine over scan geometry and odometry.

    The class owns no ROS objects and no threads: ``tick`` is a pure function of
    (time, scan profile, odom pose, previous state), which is what the tests
    exercise.
    """

    def __init__(
        self,
        *,
        max_linear_mps: float = 0.05,
        max_angular_rps: float = 0.35,
        min_clearance_m: float = 0.25,
        min_clearance_turn_m: float = 0.12,
        sectors: int = 12,
        front_half_angle_rad: float = math.radians(25.0),
        forward_distance_m: float = 0.12,
        # Turn-first makes a sector forward a two-phase motion (turn, then go),
        # so the budget covers a 165 deg turn plus the advance.
        forward_timeout_s: float = 8.0,
        turn_rad: float = math.radians(30.0),
        turn_timeout_s: float = 2.5,
        face_timeout_s: float = 8.0,
        open_space_distance_m: float = 0.20,
        open_space_timeout_s: float = 4.0,
        max_steer_rad: float = math.pi / 2.0,
        scan_timeout_s: float = 1.5,
        odom_timeout_s: float = 1.0,
        min_valid_samples: int = 8,
        stop_burst: int = 3,
        priority_ttl_s: float = 3.0,
        geometry_weight: float = 0.7,
        score_range_m: float = 2.0,
        skill_table: SkillTable | None = None,
    ) -> None:
        # The file is the vocabulary; a malformed table raises at construction
        # so the node refuses to start instead of running without skills.
        self.skills: SkillTable = skill_table if skill_table is not None else load_table()
        self.max_linear = max(0.0, float(max_linear_mps))
        self.max_angular = max(0.0, float(max_angular_rps))
        self.min_clearance = float(min_clearance_m)
        self.min_clearance_turn = float(min_clearance_turn_m)
        self.sectors = int(sectors)
        self.front_half_angle = float(front_half_angle_rad)
        self.forward_distance = float(forward_distance_m)
        self.forward_timeout = float(forward_timeout_s)
        self.turn_rad = float(turn_rad)
        self.turn_timeout = float(turn_timeout_s)
        # A 165 deg face takes ~4.7 s at 0.35 rad/s; 8 s leaves room for the
        # proportional tail without letting a stuck turn run forever.
        self.face_timeout = float(face_timeout_s)
        self.open_space_distance = float(open_space_distance_m)
        self.open_space_timeout = float(open_space_timeout_s)
        self.max_steer = float(max_steer_rad)
        self.scan_timeout = float(scan_timeout_s)
        self.odom_timeout = float(odom_timeout_s)
        self.min_valid_samples = max(1, int(min_valid_samples))
        self.stop_burst = max(0, int(stop_burst))
        self.priority_ttl = max(0.3, float(priority_ttl_s))
        # Geometry must dominate: a wrong L2 opinion can bias the choice, never
        # overrule a measured distance.
        self.geometry_weight = min(1.0, max(0.5, float(geometry_weight)))
        self.priority_weight = 1.0 - self.geometry_weight
        self.score_range = max(0.3, float(score_range_m))

        self._goal: GoalSpec | None = None
        self._guide: PriorityGuide | None = None
        self._start_pose: tuple[float, float, float] | None = None
        self._last_pose: tuple[float, float, float] | None = None
        self._traveled = 0.0
        self._yaw_delta = 0.0
        self._pending_zero = 0
        self._last_state = IDLE
        self._last_reason = "idle"
        self._last_detail: dict[str, Any] = {}
        # Negotiation state: the last measured world is what a proposal is
        # evaluated against, so it must be captured by tick rather than asked
        # for again by the caller.
        self._last_profile: ScanProfile | None = None
        self._world_pose: tuple[float, float, float] | None = None
        self._last_scan_age: float | None = None
        self._last_odom_age: float | None = None
        self._proposals: dict[str, Proposal] = {}
        self._proposal_ttl = PROPOSAL_TTL_S

    @property
    def goal(self) -> GoalSpec | None:
        return self._goal

    @property
    def guide(self) -> PriorityGuide | None:
        return self._guide

    @property
    def last_state(self) -> str:
        return self._last_state

    @property
    def traveled_m(self) -> float:
        return round(self._traveled, 3)

    # --- negotiation --------------------------------------------------------
    def _expire_proposals(self, now: float) -> None:
        for proposal_id in [pid for pid, item in self._proposals.items() if item.expires_at <= now]:
            self._proposals.pop(proposal_id, None)
        while len(self._proposals) > PROPOSAL_HISTORY:
            oldest = min(self._proposals.values(), key=lambda item: item.created_at)
            self._proposals.pop(oldest.proposal_id, None)

    def _world_evidence(self) -> dict[str, Any]:
        """The measurements a refusal is based on, so the answer is explainable."""

        profile = self._last_profile
        front = None
        valid = None
        if profile is not None:
            valid = profile.valid_count
            if math.isfinite(profile.front_min):
                front = round(profile.front_min, 3)
        return {
            "scan_age_s": None if self._last_scan_age is None else round(self._last_scan_age, 3),
            "scan_limit_s": self.scan_timeout,
            "odom_age_s": None if self._last_odom_age is None else round(self._last_odom_age, 3),
            "odom_limit_s": self.odom_timeout,
            "valid_samples": valid,
            "min_valid_samples": self.min_valid_samples,
            "front_m": front,
            "min_clearance_m": self.min_clearance,
            "min_clearance_turn_m": self.min_clearance_turn,
        }

    def _proposal_refusal(self, error: str, **extra: Any) -> dict[str, Any]:
        base = error.split(":", 1)[0]
        hint = {
            "scan_stale": "雷达没有新帧；查 /scan 发布者与 micro-ROS 会话",
            "scan_missing": "还没收到任何 /scan；查雷达与 agent",
            "scan_age_unknown": "扫描帧没有有效时间戳；查 /scan header",
            "odom_stale": "里程计没有新帧；查 /odom 与 agent 会话",
            "odom_missing": "还没收到 /odom；查 bringup",
            "odom_age_unknown": "里程计没有有效时间戳；查 /odom header",
            "scan_insufficient": "扫描帧是新的，但有效距离太少；查遮挡、供电或雷达朝向",
            "obstacle": "净空低于阈值；换一个扇区，或先原地转向",
            "no_headroom": "净空扣掉停车余量后不够走一步；先转向",
            "clearance_unknown": "该方向没有有效测量，不能当作空",
            "proposal_unknown_or_expired": "提议已过期或被消费；重新 propose",
            "sector_out_of_range": f"扇区必须在 0–{self.sectors - 1}",
        }.get(base)
        return {
            "ok": False,
            "accepted": False,
            "error": error,
            **self._world_evidence(),
            **({"hint": hint} if hint else {}),
            **extra,
        }
    def propose(self, payload: Mapping[str, Any], *, now: float | None = None) -> dict[str, Any]:
        """Evaluate an ask against the last measured world and answer with a bound.

        This replaces a human-held timer: the AI asks, the cerebellum checks the
        scan it already has, and either refuses with a named reason or returns a
        proposal with a *negotiated* distance and a short freshness window. The
        proposal must be executed before it expires; executing re-checks the
        world, so a stale answer cannot become a stale command.
        """

        now = time.monotonic() if now is None else now
        self._expire_proposals(now)
        if not isinstance(payload, Mapping):
            return self._proposal_refusal("goal_must_be_object")

        parsed = parse_goal(payload)
        if parsed.error == "unknown_goal":
            raw = payload.get("goal", payload.get("intent", payload.get("skill")))
            if isinstance(raw, str) and raw.strip().lower() in self.skills.skills:
                parsed = ParsedGoal(kind=raw.strip().lower(), note=str(payload.get("note", ""))[:120])
        if parsed.error:
            return self._proposal_refusal(parsed.error, accepted_goals=self.skills.names())

        primitive = self.skills.primitive(parsed.kind)
        if primitive is None:
            return self._proposal_refusal(
                "unknown_skill", skill=parsed.kind, accepted_goals=self.skills.names()
            )
        values, error = self.skills.validate_params(parsed.kind, payload)
        if error:
            return self._proposal_refusal(error, skill=parsed.kind)
        sector = values.get("sector") if payload.get("sector") is not None else None
        if sector is not None and not 0 <= int(sector) < self.sectors:
            return self._proposal_refusal("sector_out_of_range", allowed=[0, self.sectors - 1])

        if primitive == "zero":
            return {
                "ok": True,
                "accepted": True,
                "skill": parsed.kind,
                "requires_execute": False,
                "reason": "stop_is_immediate",
                "state": DONE,
            }

        profile = self._last_profile
        if profile is None:
            return self._proposal_refusal("scan_missing")
        if self._last_scan_age is None:
            return self._proposal_refusal("scan_age_unknown")
        if self._last_scan_age > self.scan_timeout:
            return self._proposal_refusal(
                "scan_stale", scan_age_s=round(self._last_scan_age, 3), limit_s=self.scan_timeout
            )
        if self._world_pose is None:
            return self._proposal_refusal("odom_missing")
        if self._last_odom_age is None:
            return self._proposal_refusal("odom_age_unknown")
        if self._last_odom_age > self.odom_timeout:
            return self._proposal_refusal(
                "odom_stale", odom_age_s=round(self._last_odom_age, 3), limit_s=self.odom_timeout
            )
        if profile.valid_count < self.min_valid_samples:
            return self._proposal_refusal(
                "scan_insufficient",
                valid_samples=profile.valid_count,
                min_valid_samples=self.min_valid_samples,
            )

        requested = float(values.get("distance_m") or 0.0)
        if primitive == "best_open" and requested <= 0.0:
            requested = self.open_space_distance
        elif primitive in ("advance", "retreat") and requested <= 0.0:
            requested = self.forward_distance

        if primitive == "best_open":
            index, angle, clearance = profile.best_sector(self.max_steer)
            sector = int(index)
            detail_angle = angle
        elif primitive == "retreat":
            clearance = _rear_min(profile, sector)
            detail_angle = None
        elif primitive in ("turn_in_place", "turn_fixed") and sector is not None:
            clearance = min(profile.sector_min[int(sector)], profile.front_min)
            detail_angle = profile.sector_angle(int(sector))
        elif sector is not None:
            clearance = min(profile.sector_min[int(sector)], profile.front_min)
            detail_angle = profile.sector_angle(int(sector))
        else:
            clearance = profile.front_min
            detail_angle = 0.0

        turning = primitive in ("turn_in_place", "turn_fixed")
        threshold = self.min_clearance_turn if turning else self.min_clearance
        if not math.isfinite(clearance) or clearance < threshold:
            return self._proposal_refusal(
                f"obstacle:{clearance:.2f}" if math.isfinite(clearance) else "clearance_unknown",
                skill=parsed.kind,
                sector=sector,
                clearance_m=None if not math.isfinite(clearance) else round(clearance, 3),
                threshold_m=threshold,
                alternatives=self._open_sectors(profile, limit=5),
            )

        distance = requested
        if primitive in ("advance", "retreat") and sector is not None:
            headroom = max(0.0, clearance - STOP_MARGIN_M)
            distance = min(requested, headroom)
            if distance < 0.02:
                return self._proposal_refusal(
                    "no_headroom",
                    skill=parsed.kind,
                    sector=sector,
                    clearance_m=round(clearance, 3),
                    alternatives=self._open_sectors(profile, limit=5),
                )

        proposal_id = f"prop-{int(now * 1000)}-{len(self._proposals):02d}"
        body: dict[str, Any] = {"goal": parsed.kind, "note": parsed.note}
        if sector is not None:
            body["sector"] = int(sector)
        if primitive in ("advance", "retreat", "best_open"):
            body["distance_m"] = round(distance, 3)
        if parsed.timeout_s:
            body["duration_ms"] = int(parsed.timeout_s * 1000)
        self._proposals[proposal_id] = Proposal(
            proposal_id=proposal_id,
            skill=parsed.kind,
            sector=None if sector is None else int(sector),
            requested_distance_m=round(requested, 3),
            distance_m=round(distance, 3),
            created_at=now,
            expires_at=now + self._proposal_ttl,
            payload=body,
        )
        return {
            "ok": True,
            "accepted": True,
            "proposal_id": proposal_id,
            "skill": parsed.kind,
            "primitive": primitive,
            "sector": None if sector is None else int(sector),
            "sector_angle_rad": None if detail_angle is None else round(detail_angle, 3),
            "requested_distance_m": round(requested, 3),
            "distance_m": round(distance, 3),
            "clearance_m": None if not math.isfinite(clearance) else round(clearance, 3),
            "expires_in_ms": int(self._proposal_ttl * 1000),
            "alternatives": self._open_sectors(profile, limit=5),
            "state": "proposed",
            **self._world_evidence(),
            "min_clearance_for_this_skill_m": threshold,
        }

    def proposal(self, proposal_id: str) -> Proposal | None:
        """Return a live proposal, or None once it expired or was consumed."""

        return self._proposals.get(str(proposal_id))

    def _open_sectors(self, profile: ScanProfile, *, limit: int = 5) -> list[dict[str, Any]]:
        rows = []
        for index, distance in enumerate(profile.sector_min):
            if not math.isfinite(distance) or distance < self.min_clearance:
                continue
            rows.append(
                {
                    "sector": index,
                    "deg": round(math.degrees(profile.sector_angle(index)), 1),
                    "clearance_m": round(float(distance), 3),
                }
            )
        rows.sort(key=lambda row: row["clearance_m"], reverse=True)
        return rows[:limit]

    def execute(self, payload: Mapping[str, Any], *, now: float | None = None) -> dict[str, Any]:
        """Execute one live proposal, re-checking the world first."""

        now = time.monotonic() if now is None else now
        self._expire_proposals(now)
        proposal_id = payload.get("proposal_id") if isinstance(payload, Mapping) else None
        if not isinstance(proposal_id, str) or proposal_id not in self._proposals:
            return {"ok": False, "accepted": False, "error": "proposal_unknown_or_expired"}

        proposal = self._proposals.pop(proposal_id)
        profile = self._last_profile
        if profile is None or self._last_scan_age is None or self._last_scan_age > self.scan_timeout:
            return {"ok": False, "accepted": False, "error": "scan_stale", "proposal_id": proposal_id}
        if (
            self._world_pose is None
            or self._last_odom_age is None
            or self._last_odom_age > self.odom_timeout
        ):
            return {"ok": False, "accepted": False, "error": "odom_stale", "proposal_id": proposal_id}

        primitive = self.skills.primitive(proposal.skill)
        if primitive == "retreat":
            clearance = _rear_min(profile, proposal.sector)
            threshold = self.min_clearance
        elif proposal.sector is not None:
            clearance = min(profile.sector_min[proposal.sector], profile.front_min)
            threshold = self.min_clearance_turn if primitive in ("turn_in_place", "turn_fixed") else self.min_clearance
        else:
            clearance = profile.front_min
            threshold = self.min_clearance
        if not math.isfinite(clearance) or clearance < threshold:
            reason = f"obstacle:{clearance:.2f}" if math.isfinite(clearance) else "clearance_unknown"
            return {
                "ok": False,
                "accepted": False,
                "error": reason,
                "proposal_id": proposal_id,
                "clearance_m": None if not math.isfinite(clearance) else round(clearance, 3),
            }

        result = self.set_goal(proposal.payload, now=now)
        result["proposal_id"] = proposal_id
        result["negotiated_distance_m"] = proposal.distance_m
        return result

    def set_goal(self, payload: Mapping[str, Any], *, now: float | None = None) -> dict[str, Any]:
        """Accept a goal request. Returns an immediate, JSON-safe answer.

        The *vocabulary and the parameter bounds* come from ``skills.json``:
        a known alias that the table does not define is ``unknown_skill`` (with
        the known list), and a parameter outside the table's range is
        ``<param>_out_of_range``. The table is the contract; this method only
        applies it.
        """

        now = time.monotonic() if now is None else now
        parsed = parse_goal(payload)
        if parsed.error == "unknown_goal":
            # A skill that exists only in the table has no alias entry here; its
            # own name is canonical. This is what makes "add a skill = edit the
            # file" true instead of aspirational.
            raw = payload.get("goal", payload.get("intent", payload.get("skill")))
            if isinstance(raw, str) and raw.strip().lower() in self.skills.skills:
                parsed = ParsedGoal(kind=raw.strip().lower(), note=str(payload.get("note", ""))[:120])
        if parsed.error:
            self._last_state = REJECTED
            self._last_reason = parsed.error
            return {
                "ok": False,
                "accepted": False,
                "error": parsed.error,
                "accepted_goals": self.skills.names(),
            }

        primitive = self.skills.primitive(parsed.kind)
        if primitive is None:
            self._last_state = REJECTED
            self._last_reason = "unknown_skill"
            return {
                "ok": False,
                "accepted": False,
                "error": "unknown_skill",
                "skill": parsed.kind,
                "accepted_goals": self.skills.names(),
            }

        values, error = self.skills.validate_params(parsed.kind, payload)
        if error:
            self._last_state = REJECTED
            self._last_reason = error
            spec = self.skills.spec(parsed.kind)
            return {
                "ok": False,
                "accepted": False,
                "error": error,
                "skill": parsed.kind,
                "params": {name: param.as_dict() for name, param in (spec.params.items() if spec else [])},
            }

        # A table default fills a *required* parameter, but forwarding without a
        # sector means "straight ahead" (legacy contract). Only an explicit
        # sector in the payload makes this a sector-targeted skill.
        sector = values.get("sector") if payload.get("sector") is not None else None
        if sector is not None and not 0 <= int(sector) < self.sectors:
            self._last_state = REJECTED
            self._last_reason = "sector_out_of_range"
            return {
                "ok": False,
                "accepted": False,
                "error": "sector_out_of_range",
                "allowed": [0, self.sectors - 1],
            }
        # A table default only fills what the caller left out; an explicit
        # distance went through the table's bounds above.
        distance = float(values["distance_m"]) if payload.get("distance_m") is not None else 0.0

        previous = self._goal.kind if self._goal else None
        goal_id = str(payload.get("goal_id") or f"goal-{int(now * 1000)}")[:64]
        if primitive == "zero":
            self._goal = None
            self._guide = None
            self._start_pose = None
            self._last_pose = None
            self._pending_zero = self.stop_burst
            self._last_state = DONE
            self._last_reason = CANCEL_STOP
            self._last_detail = {"traveled_m": 0.0, "elapsed_s": 0.0, "skill": parsed.kind}
            return {
                "ok": True,
                "accepted": True,
                "goal": parsed.kind,
                "goal_id": goal_id,
                "state": DONE,
                "reason": CANCEL_STOP,
                "previous": previous,
            }

        self._goal = GoalSpec(
            kind=parsed.kind,
            goal_id=goal_id,
            note=parsed.note,
            distance_m=distance,
            timeout_s=parsed.timeout_s or 0.0,
            turn_rad=self.turn_rad,
            requested_at=now,
            sector=None if sector is None else int(sector),
            phase="",
        )
        self._start_pose = None
        self._last_pose = None
        self._traveled = 0.0
        self._yaw_delta = 0.0
        self._pending_zero = 0
        self._last_state = ACTIVE
        self._last_reason = "goal_accepted"
        return {
            "ok": True,
            "accepted": True,
            "goal": parsed.kind,
            "goal_id": goal_id,
            "sector": None if sector is None else int(sector),
            "primitive": primitive,
            "state": ACTIVE,
            "reason": CANCEL_SUPERSEDED if previous else "goal_accepted",
            "previous": previous,
        }

    def cancel(self, reason: str = CANCEL_STOP) -> None:
        self._goal = None
        self._start_pose = None
        self._last_pose = None
        self._pending_zero = self.stop_burst
        self._last_state = DONE if reason == CANCEL_STOP else BLOCKED
        self._last_reason = reason

    def release(self, reason: str = "tour") -> None:
        """Drop a single-step goal without reporting a fault (tour handover)."""

        self._goal = None
        self._start_pose = None
        self._last_pose = None
        self._pending_zero = self.stop_burst
        self._last_state = DONE
        self._last_reason = reason

    def set_priority(self, payload: Mapping[str, Any], *, now: float | None = None) -> dict[str, Any]:
        """Accept an L2 opinion: ``{veto, reason, priority[sectors], mode, ttl_ms}``.

        The vector is bounded to ``[0, 1]`` per sector and expires; when it
        expires the robot stops. A model that stops thinking cannot leave the
        robot driving.
        """

        now = time.monotonic() if now is None else now
        priority = payload.get("priority")
        if not isinstance(priority, (list, tuple)) or len(priority) != self.sectors:
            return {
                "ok": False,
                "error": "priority_length",
                "expected": self.sectors,
            }
        values: list[float] = []
        for item in priority:
            if not _finite(item):
                return {"ok": False, "error": "priority_not_finite"}
            values.append(min(1.0, max(0.0, float(item))))
        ttl_ms = payload.get("ttl_ms")
        if ttl_ms is None:
            ttl = self.priority_ttl
        else:
            if not _finite(ttl_ms) or not 300 <= float(ttl_ms) <= 10000:
                return {"ok": False, "error": "ttl_out_of_range", "allowed_ms": [300, 10000]}
            ttl = float(ttl_ms) / 1000.0
        guide = PriorityGuide(
            veto=bool(payload.get("veto", False)),
            reason=str(payload.get("reason", ""))[:120],
            priority=tuple(values),
            mode=str(payload.get("mode", ""))[:24],
            set_at=now,
            expires_at=now + ttl,
        )
        self._guide = guide
        return {
            "ok": True,
            "accepted": True,
            "veto": guide.veto,
            "mode": guide.mode,
            "expires_in_ms": int(ttl * 1000),
        }

    def clear_priority(self, reason: str = "priority_cleared") -> None:
        self._guide = None
        if reason:
            self._last_reason = reason

    def _fresh_guide(self, now: float) -> PriorityGuide | None:
        guide = self._guide
        if guide is None or guide.expires_at <= now:
            return None
        return guide

    def _fused_sector(
        self, profile: ScanProfile, guide: PriorityGuide
    ) -> tuple[int, float, float, float]:
        """Pick a direction from geometry fused with the L2 preference."""

        best_score = -1.0
        best_angle = 0.0
        best_index = 0
        best_clearance = -1.0
        for index in range(self.sectors):
            angle = profile.sector_angle(index)
            if abs(angle) > self.max_steer:
                continue
            clearance = profile.sector_min[index]
            geometric = (
                1.0
                if not math.isfinite(clearance)
                else min(1.0, max(0.0, clearance / self.score_range))
            )
            score = self.geometry_weight * geometric + self.priority_weight * guide.priority[index]
            if score > best_score or (score == best_score and abs(angle) < abs(best_angle)):
                best_score, best_angle, best_index, best_clearance = (
                    score, angle, index, clearance,
                )
        return best_index, best_angle, best_clearance, best_score

    def _finish(self, state: str, reason: str, now: float) -> None:
        if self._goal is not None:
            self._last_detail = {
                "last_goal": self._goal.kind,
                "last_goal_id": self._goal.goal_id,
                "last_goal_sector": self._goal.sector,
                "last_phase": self._goal.phase,
                "traveled_m": round(self._traveled, 4),
                "elapsed_s": round(max(0.0, now - self._goal.requested_at), 3),
                "yaw_deg": round(math.degrees(self._yaw_delta), 1),
            }
        self._goal = None
        self._start_pose = None
        self._pending_zero = self.stop_burst
        self._last_state = state
        self._last_reason = reason

    def _advance_odometry(self, pose: tuple[float, float, float]) -> None:
        x, y, yaw = pose
        if self._start_pose is None:
            self._start_pose = pose
            self._last_pose = pose
            return
        last = self._last_pose or pose
        self._traveled += math.hypot(x - last[0], y - last[1])
        delta = (yaw - last[2] + math.pi) % (2.0 * math.pi) - math.pi
        self._yaw_delta += delta
        self._last_pose = pose

    def _abort(self, state: str, reason: str, now: float, goal: GoalSpec | None, mode: str = "") -> None:
        """End whatever was running, keeping the zero-burst invariant."""

        if goal is not None:
            self._finish(state, reason, now)
            return
        self._guide = None
        self._pending_zero = self.stop_burst
        self._last_state = state
        self._last_reason = reason
        self._last_detail = {"last_goal": "guided", "mode": mode}

    def tick(
        self,
        *,
        now: float,
        profile: ScanProfile,
        pose: tuple[float, float, float] | None,
        scan_age_s: float | None,
        odom_age_s: float | None,
    ) -> CerebellumDecision:
        detail: dict[str, Any] = {
            "front_m": None if not math.isfinite(profile.front_min) else round(profile.front_min, 3),
            "left_m": None if not math.isfinite(profile.left_min) else round(profile.left_min, 3),
            "right_m": None if not math.isfinite(profile.right_min) else round(profile.right_min, 3),
            "sectors": profile.as_list(),
            "valid_samples": profile.valid_count,
            "blind_fraction": round(profile.blind_fraction, 3),
            "scan_age_s": None if scan_age_s is None else round(scan_age_s, 3),
            "odom_age_s": None if odom_age_s is None else round(odom_age_s, 3),
        }
        self._last_profile = profile
        self._world_pose = pose
        self._last_scan_age = scan_age_s
        self._last_odom_age = odom_age_s

        goal = self._goal
        guide: PriorityGuide | None = None

        # An L2 veto outranks everything, including an explicit goal. It is
        # consumed once so a later goal is not silently blocked by a stale veto.
        fresh = self._fresh_guide(now)
        if fresh is not None and fresh.veto:
            mode = fresh.mode or "guided"
            reason = f"veto:cortex:{fresh.reason or 'veto'}"
            self._guide = None
            detail["mode"] = mode
            self._abort(VETO, reason, now, goal, mode)
            return CerebellumDecision(
                0.0, 0.0, VETO, goal.kind if goal else None, goal.goal_id if goal else None,
                reason, True, {**detail, **self._last_detail},
            )

        if goal is None:
            guide = fresh
            if guide is None:
                if self._guide is not None:
                    # The L2 opinion expired. Stop on its own: a model that
                    # stops thinking must not leave the robot driving.
                    self._guide = None
                    self._last_state = DONE
                    self._last_reason = "priority_expired"
                    self._last_detail = {"last_goal": "guided"}
                    self._pending_zero = self.stop_burst
                if self._pending_zero > 0:
                    self._pending_zero -= 1
                    return CerebellumDecision(
                        0.0, 0.0, self._last_state, None, None, self._last_reason, True,
                        {**detail, **self._last_detail},
                    )
                return CerebellumDecision(
                    0.0, 0.0, self._last_state, None, None, self._last_reason, False,
                    {**detail, **self._last_detail},
                )
            mode = guide.mode or "guided"
            detail["mode"] = mode
            detail["priority_age_s"] = round(max(0.0, now - guide.set_at), 3)

        elapsed = max(0.0, now - goal.requested_at) if goal is not None else 0.0
        kind = goal.kind if goal is not None else "guided"
        goal_id = goal.goal_id if goal is not None else None

        if scan_age_s is None or scan_age_s < 0 or scan_age_s > self.scan_timeout:
            detail["blocked_goal"] = kind
            self._abort(BLOCKED, "scan_stale", now, goal, guide.mode if guide else "")
            return CerebellumDecision(0.0, 0.0, BLOCKED, kind, goal_id, "scan_stale", True, detail)

        if pose is None or odom_age_s is None or odom_age_s < 0 or odom_age_s > self.odom_timeout:
            detail["blocked_goal"] = kind
            self._abort(BLOCKED, "odom_stale", now, goal, guide.mode if guide else "")
            return CerebellumDecision(0.0, 0.0, BLOCKED, kind, goal_id, "odom_stale", True, detail)

        # A fresh timestamp with almost no usable returns means the lidar is not
        # telling us about the world (blocked, power issue, wrong frame). Never
        # drive on an empty scan just because its header is new.
        if profile.valid_count < self.min_valid_samples:
            detail["blocked_goal"] = kind
            detail["valid_samples"] = profile.valid_count
            self._abort(BLOCKED, "scan_insufficient", now, goal, guide.mode if guide else "")
            return CerebellumDecision(0.0, 0.0, BLOCKED, kind, goal_id, "scan_insufficient", True, detail)

        self._advance_odometry(pose)

        if goal is not None:
            primitive = self.skills.primitive(goal.kind) or ""
            sector_index = goal.sector
            sector_angle = profile.sector_angle(sector_index) if sector_index is not None else 0.0
            remaining = _wrap_angle(sector_angle - self._yaw_delta) if sector_index is not None else 0.0
            sector_clearance = float("inf")
            if sector_index is not None and 0 <= sector_index < profile.sector_count:
                sector_clearance = profile.sector_min[sector_index]
                detail["goal_sector"] = sector_index
                detail["goal_sector_angle_rad"] = round(sector_angle, 3)
                detail["goal_sector_clearance_m"] = (
                    None if not math.isfinite(sector_clearance) else round(sector_clearance, 3)
                )
            detail["goal_primitive"] = primitive

            # Sector advances are two-phase: turn towards the sector centre first
            # (plan P3), then drive. The phase is decided before the clearance
            # gate because a turn is judged by the turn clearance and the advance
            # by the forward one.
            phase = goal.phase
            if primitive == "advance" and sector_index is not None:
                turn_first_rad = math.radians(self.skills.option(goal.kind, "turn_first_deg", 15.0))
                if not phase:
                    phase = "turn" if abs(remaining) > turn_first_rad + TURN_FIRST_EPS else "advance"
                    self._goal = replace(goal, phase=phase)
                elif phase == "advance" and abs(remaining) > math.radians(60.0):
                    # Drifted far off the chosen direction: turn again instead of
                    # arcing blindly across the room.
                    phase = "turn"
                    self._goal = replace(goal, phase=phase)
                elif phase == "turn" and abs(remaining) <= turn_first_rad + TURN_FIRST_EPS:
                    phase = "advance"
                    self._goal = replace(goal, phase=phase)
                detail["goal_phase"] = phase
                detail["goal_remaining_deg"] = round(math.degrees(remaining), 1)

            turning_now = primitive in ("turn_in_place", "turn_fixed") or (
                primitive == "advance" and sector_index is not None and phase == "turn"
            )
            if turning_now:
                if primitive == "turn_fixed":
                    swept = profile.left_min if goal.kind == LEFT else profile.right_min
                    clearance = min(swept, profile.front_min)
                else:
                    clearance = min(sector_clearance, profile.front_min)
                threshold = self.min_clearance_turn
            elif primitive in ("advance", "best_open") and sector_index is not None:
                # A sector goal is vetoed by the *target* direction and by what is
                # straight ahead: driving into a clear sector still crosses the
                # near field first.
                clearance = min(sector_clearance, profile.front_min)
                threshold = self.min_clearance
            elif primitive == "retreat":
                # The body moves backwards, so the meaningful direction is the
                # one opposite the request (or the rear pair when no sector).
                clearance = _rear_min(profile, sector_index)
                threshold = self.min_clearance
            else:
                clearance = profile.front_min
                threshold = self.min_clearance
            if clearance < threshold:
                reason = f"veto:obstacle:{clearance:.2f}"
                detail["clearance_m"] = round(clearance, 3)
                detail["veto_threshold_m"] = threshold
                self._finish(VETO, reason, now)
                return CerebellumDecision(0.0, 0.0, VETO, goal.kind, goal.goal_id, reason, True, detail)

            if primitive == "turn_in_place":
                timeout = goal.timeout_s or self.face_timeout
                detail["goal_remaining_deg"] = round(math.degrees(remaining), 1)
                if abs(remaining) <= FACE_TOLERANCE_RAD:
                    self._finish(DONE, CANCEL_REACHED, now)
                    return CerebellumDecision(0.0, 0.0, DONE, goal.kind, goal.goal_id, CANCEL_REACHED, True, detail)
                if elapsed >= timeout:
                    self._finish(DONE, CANCEL_TIMEOUT, now)
                    return CerebellumDecision(0.0, 0.0, DONE, goal.kind, goal.goal_id, CANCEL_TIMEOUT, True, detail)
                linear = 0.0
                angular = _clamp(FACE_GAIN * remaining, self.max_angular)
            elif primitive == "advance":
                distance = goal.distance_m or self.forward_distance
                timeout = goal.timeout_s or self.forward_timeout
                if phase == "turn":
                    if elapsed >= timeout:
                        self._finish(DONE, CANCEL_TIMEOUT, now)
                        return CerebellumDecision(0.0, 0.0, DONE, goal.kind, goal.goal_id, CANCEL_TIMEOUT, True, detail)
                    linear = 0.0
                    angular = _clamp(FACE_GAIN * remaining, self.max_angular)
                else:
                    if self._traveled >= distance:
                        self._finish(DONE, CANCEL_REACHED, now)
                        return CerebellumDecision(0.0, 0.0, DONE, goal.kind, goal.goal_id, CANCEL_REACHED, True, detail)
                    if elapsed >= timeout:
                        self._finish(DONE, CANCEL_TIMEOUT, now)
                        return CerebellumDecision(0.0, 0.0, DONE, goal.kind, goal.goal_id, CANCEL_TIMEOUT, True, detail)
                    if sector_index is not None:
                        linear, angular = self._steer_from_sector(remaining)
                    else:
                        linear, angular = self.max_linear, 0.0
            elif primitive == "retreat":
                distance = goal.distance_m or self.forward_distance
                timeout = goal.timeout_s or self.forward_timeout
                if self._traveled >= distance:
                    self._finish(DONE, CANCEL_REACHED, now)
                    return CerebellumDecision(0.0, 0.0, DONE, goal.kind, goal.goal_id, CANCEL_REACHED, True, detail)
                if elapsed >= timeout:
                    self._finish(DONE, CANCEL_TIMEOUT, now)
                    return CerebellumDecision(0.0, 0.0, DONE, goal.kind, goal.goal_id, CANCEL_TIMEOUT, True, detail)
                linear = -self.max_linear
                angular = 0.0
            elif primitive == "turn_fixed":
                timeout = goal.timeout_s or self.turn_timeout
                limit = goal.turn_rad or self.turn_rad
                if abs(self._yaw_delta) >= limit:
                    self._finish(DONE, CANCEL_REACHED, now)
                    return CerebellumDecision(0.0, 0.0, DONE, goal.kind, goal.goal_id, CANCEL_REACHED, True, detail)
                if elapsed >= timeout:
                    self._finish(DONE, CANCEL_TIMEOUT, now)
                    return CerebellumDecision(0.0, 0.0, DONE, goal.kind, goal.goal_id, CANCEL_TIMEOUT, True, detail)
                linear = 0.0
                if goal.kind == LEFT:
                    direction = 1.0
                elif goal.kind == RIGHT:
                    direction = -1.0
                else:
                    direction = 1.0 if remaining >= 0 else -1.0
                angular = direction * self.max_angular
            elif primitive == "best_open":
                distance = goal.distance_m or self.open_space_distance
                timeout = goal.timeout_s or self.open_space_timeout
                if self._traveled >= distance:
                    self._finish(DONE, CANCEL_REACHED, now)
                    return CerebellumDecision(0.0, 0.0, DONE, goal.kind, goal.goal_id, CANCEL_REACHED, True, detail)
                if elapsed >= timeout:
                    self._finish(DONE, CANCEL_TIMEOUT, now)
                    return CerebellumDecision(0.0, 0.0, DONE, goal.kind, goal.goal_id, CANCEL_TIMEOUT, True, detail)
                index, angle, sector_clearance = profile.best_sector(self.max_steer)
                detail["best_sector"] = index
                detail["best_sector_angle_rad"] = round(angle, 3)
                detail["best_sector_clearance_m"] = round(sector_clearance, 3)
                if sector_clearance < self.min_clearance:
                    reason = f"veto:obstacle:{sector_clearance:.2f}"
                    self._finish(VETO, reason, now)
                    return CerebellumDecision(0.0, 0.0, VETO, goal.kind, goal.goal_id, reason, True, detail)
                linear, angular = self._steer_from_sector(angle)
            else:
                # A skill in the file whose primitive has no implementation in
                # this build: refuse loudly instead of guessing a motion.
                self._finish(REJECTED, "unsupported_primitive", now)
                return CerebellumDecision(
                    0.0, 0.0, REJECTED, goal.kind, goal.goal_id, "unsupported_primitive", True, detail
                )
        else:
            # Guided: geometry fused with the L2 preference vector.
            index, angle, sector_clearance, score = self._fused_sector(profile, guide)
            detail.update(
                {
                    "best_sector": index,
                    "best_sector_angle_rad": round(angle, 3),
                    "best_sector_clearance_m": None
                    if not math.isfinite(sector_clearance)
                    else round(sector_clearance, 3),
                    "fused_score": round(score, 3),
                }
            )
            # Escape rule: never move with anything inside the turn clearance,
            # but allow rotating away from a near obstacle the strict forward
            # threshold would freeze on.
            if profile.front_min < self.min_clearance_turn:
                reason = f"veto:obstacle:{profile.front_min:.2f}"
                self._abort(VETO, reason, now, None, detail["mode"])
                return CerebellumDecision(0.0, 0.0, VETO, None, None, reason, True, detail)
            turning_hard = abs(angle) > math.radians(45.0)
            threshold = self.min_clearance_turn if turning_hard else self.min_clearance
            effective = (
                sector_clearance
                if turning_hard
                else min(sector_clearance, profile.front_min)
            )
            if effective < threshold:
                reason = f"veto:obstacle:{effective:.2f}"
                self._abort(VETO, reason, now, None, detail["mode"])
                return CerebellumDecision(0.0, 0.0, VETO, None, None, reason, True, detail)
            linear, angular = self._steer_from_sector(angle)

        linear = _clamp(linear, self.max_linear)
        angular = _clamp(angular, self.max_angular)
        detail.update(
            {
                "traveled_m": round(self._traveled, 4),
                "elapsed_s": round(elapsed, 3),
                "yaw_deg": round(math.degrees(self._yaw_delta), 1),
                "would_execute": {"linear": round(linear, 4), "angular": round(angular, 4)},
            }
        )
        self._last_state = ACTIVE
        self._last_reason = "guided_active" if goal is None else "goal_active"
        return CerebellumDecision(
            linear, angular, ACTIVE, kind, goal_id, self._last_reason, True, detail
        )

    def _steer_from_sector(self, angle: float) -> tuple[float, float]:
        angular = _clamp(1.2 * angle, self.max_angular)
        if abs(angle) > math.radians(60.0):
            linear = 0.0
        else:
            linear = self.max_linear * max(0.35, 1.0 - abs(angle) / self.max_steer)
        return linear, angular
