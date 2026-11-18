"""robotd: the car-side daemon that owns the control loop and the live socket.

The daemon is deliberately split in two:

* ``RobotDaemon`` is pure Python. It holds a ``Cerebellum``, the latest scan
  profile, pose and map, and answers negotiation commands. It imports no ROS,
  which is what lets the same object be driven by a synthetic loop offline.
* ``DaemonServer`` is the transport. It listens on a Unix socket (the real
  deployment) or TCP (the offline mock) and speaks JSON over WebSocket:
  a 50 Hz ``state`` stream, a slower ``scan`` and ``map`` stream, and
  ``goal`` / ``propose`` / ``execute`` / ``stop`` commands answered with
  ``answer``. ``goal`` is the console's one-shot command; ``propose`` and
  ``execute`` stay for the AI, which wants to evaluate before it acts.

Discrete events describe what the source did (``command``, ``motion_started``,
``motion_done``, ``motion_vetoed``, ``motion_blocked``), not an internal
negotiation.

The local console reaches the Unix socket through ``ssh -L``; robotd never opens
a network port in the real deployment.
"""

from __future__ import annotations

import base64
import logging
import os
import socket
import threading
import time
from collections import deque
from typing import Any, Iterable, Mapping

from . import wsproto
from .cerebellum import Cerebellum, ScanProfile, profile_scan

LOGGER = logging.getLogger("xuegeros.robotd")
DEFAULT_SOCKET = "/run/user/1000/xuegeros-robotd.sock"
STATE_HZ = 50.0
SCAN_EVERY = 5   # 10 Hz at a 50 Hz loop
MAP_EVERY = 50   # 1 Hz
SCAN_POINTS = 180
MAP_MAX_SIDE = 400


def _downsample(ranges: Iterable[float], target: int) -> list[float | None]:
    values = list(ranges)
    if not values:
        return []
    step = max(1, len(values) // target)
    picked = values[::step]
    cleaned: list[float | None] = []
    for value in picked:
        try:
            number = float(value)
        except (TypeError, ValueError):
            cleaned.append(None)
            continue
        cleaned.append(number if number == number and number not in (float("inf"), float("-inf")) else None)
    return cleaned


class RobotDaemon:
    def __init__(self, cerebellum: Cerebellum, *, physical: bool = False, tour: Any = None) -> None:
        self.cerebellum = cerebellum
        self.physical = bool(physical)
        self.tour = tour
        self._tour_active = False
        self._tour_request_id: str | None = None
        self._tour_source: str | None = None
        self._suppress_state_event = False
        self._lock = threading.Lock()
        self._profile: ScanProfile | None = None
        self._pose: tuple[float, float, float] | None = None
        self._scan_age: float | None = None
        self._odom_age: float | None = None
        self._decision: dict[str, Any] | None = None
        self._scan_frame: dict[str, Any] | None = None
        self._map_frame: dict[str, Any] | None = None
        self._proposal: dict[str, Any] | None = None
        self._last_result: dict[str, Any] = {}
        self._started_at = time.time()
        # Discrete events are what a log panel can show without re-deriving
        # meaning from a 50 Hz state stream.
        self._events: deque[dict[str, Any]] = deque(maxlen=256)
        self._events_lock = threading.Lock()
        self._event_seq = 0
        self._state_seq = 0
        self._last_state_key: tuple[str, str] | None = None
        self._last_block: dict[str, Any] | None = None
        self._last_event_key: str | None = None
        self._last_event_at = 0.0

    def emit(self, kind: str, **fields: Any) -> dict[str, Any]:
        now = time.time()
        # A repeated identical refusal is one fact, not 43 lines. Collapse it
        # into a count on the existing event; a new event is emitted once the
        # reason changes or five seconds pass.
        key = f"{kind}:{fields.get('error') or fields.get('reason') or fields.get('state') or ''}"
        with self._events_lock:
            if (
                kind in ("motion_vetoed", "motion_blocked", "command_failed", "command_error")
                and key == self._last_event_key
                and now - self._last_event_at < 5.0
                and self._events
            ):
                self._events[-1]["count"] = int(self._events[-1].get("count", 1)) + 1
                return self._events[-1]
            self._event_seq += 1
            event = {"seq": self._event_seq, "ts": now, "kind": kind, **fields}
            self._events.append(event)
            self._last_event_key = key
            self._last_event_at = now
            return event

    def events_since(self, seq: int) -> list[dict[str, Any]]:
        with self._events_lock:
            return [event for event in self._events if event["seq"] > seq]

    @property
    def event_seq(self) -> int:
        with self._events_lock:
            return self._event_seq

    def _next_state_seq(self) -> int:
        # A monotonic frame counter is the console's heartbeat: a page that sees
        # the number stop advancing knows the stream died even if the socket is
        # still open.
        with self._events_lock:
            self._state_seq += 1
            return self._state_seq

    # -- inputs -------------------------------------------------------------
    def tick(
        self,
        *,
        now: float,
        profile: ScanProfile,
        pose: tuple[float, float, float] | None,
        scan_age_s: float | None,
        odom_age_s: float | None,
    ) -> Any:
        decision = self.cerebellum.tick(
            now=now, profile=profile, pose=pose, scan_age_s=scan_age_s, odom_age_s=odom_age_s
        )
        with self._lock:
            self._profile = profile
            self._pose = pose
            self._scan_age = scan_age_s
            self._odom_age = odom_age_s
            self._decision = decision.as_status()
            self._decision["state"] = decision.state
            self._decision["reason"] = decision.reason
            self._decision["would_execute"] = {
                "linear": decision.linear_mps,
                "angular": decision.angular_rps,
            }
        key = (decision.state, decision.reason)
        if key != self._last_state_key:
            self._last_state_key = key
            if self._suppress_state_event:
                self._suppress_state_event = False
            else:
                self._emit_motion_state(decision)
        self._sync_tour()
        return decision

    def _emit_motion_state(self, decision: Any) -> None:
        """Report what the motion source did, using only measured facts."""

        detail = dict(decision.detail or {})
        state = decision.state
        if state == "ACTIVE":
            self.emit(
                "motion_started",
                goal=decision.goal or "guided",
                sector=detail.get("goal_sector"),
            )
        elif state == "DONE":
            self.emit(
                "motion_done",
                goal=decision.goal,
                reason=decision.reason,
                traveled_m=detail.get("traveled_m", self.cerebellum.traveled_m),
            )
        elif state == "VETO":
            reason = str(decision.reason).replace("veto:", "")
            with self._lock:
                self._last_block = {
                    "reason": reason,
                    "skill": decision.goal,
                    "sector": detail.get("goal_sector"),
                    "clearance_m": detail.get("clearance_m"),
                    "threshold_m": detail.get("veto_threshold_m"),
                    "at": time.time(),
                }
            self.emit(
                "motion_vetoed",
                goal=decision.goal,
                sector=detail.get("goal_sector"),
                reason=reason,
                clearance_m=detail.get("clearance_m"),
                threshold_m=detail.get("veto_threshold_m"),
            )
        elif state == "BLOCKED":
            with self._lock:
                self._last_block = {
                    "reason": decision.reason,
                    "skill": decision.goal,
                    "scan_age_s": detail.get("scan_age_s"),
                    "scan_limit_s": self.cerebellum.scan_timeout,
                    "odom_age_s": detail.get("odom_age_s"),
                    "odom_limit_s": self.cerebellum.odom_timeout,
                    "at": time.time(),
                }
            self.emit(
                "motion_blocked",
                goal=decision.goal,
                reason=decision.reason,
                scan_age_s=detail.get("scan_age_s"),
                scan_limit_s=self.cerebellum.scan_timeout,
                odom_age_s=detail.get("odom_age_s"),
                odom_limit_s=self.cerebellum.odom_timeout,
            )
        elif state == "REJECTED":
            self.emit("command_failed", goal=decision.goal, reason=decision.reason)

    def set_scan(self, angle_min: float, angle_increment: float, ranges: Iterable[float], *, age_s: float | None) -> None:
        values = list(ranges)
        step = max(1, len(values) // SCAN_POINTS)
        with self._lock:
            self._scan_frame = {
                "t": "scan",
                "angle_min": float(angle_min),
                "angle_increment": float(angle_increment) * step,
                "ranges": _downsample(values, SCAN_POINTS),
                "age_s": age_s,
                "ts": time.time(),
            }

    def set_scan_frame(self, frame: Mapping[str, Any]) -> None:
        with self._lock:
            self._scan_frame = dict(frame)

    def set_map(
        self,
        *,
        width: int,
        height: int,
        resolution: float,
        origin: tuple[float, float],
        data: Iterable[int],
    ) -> None:
        values = list(data)
        stride = max(1, max(width, height) // MAP_MAX_SIDE)
        out_w = max(1, width // stride)
        out_h = max(1, height // stride)
        cells = bytearray()
        for y in range(out_h):
            row = y * stride
            source = row * width
            for x in range(out_w):
                value = int(values[source + x * stride])
                cells.append(value & 0xFF)
        with self._lock:
            self._map_frame = {
                "t": "map",
                "width": out_w,
                "height": out_h,
                "resolution": float(resolution) * stride,
                "origin": {"x": float(origin[0]), "y": float(origin[1])},
                "data_b64": base64.b64encode(bytes(cells)).decode("ascii"),
                "ts": time.time(),
            }

    # -- outputs ------------------------------------------------------------
    def state_frame(self, now: float | None = None) -> dict[str, Any]:
        now = time.time() if now is None else now
        with self._lock:
            profile = self._profile
            decision = self._decision or {}
            proposal = self._proposal
        if proposal is not None and proposal.get("expires_at", 0.0) <= now:
            proposal = None
        sectors: list[dict[str, Any]] = []
        if profile is not None:
            for index, distance in enumerate(profile.sector_min):
                sectors.append(
                    {
                        "sector": index,
                        "deg": round((index * 360.0 / profile.sector_count) - 165.0, 1),
                        "min_m": round(float(distance), 3) if distance == distance and distance != float("inf") else None,
                    }
                )
        return {
            "t": "state",
            "ts": now,
            "seq": self._next_state_seq(),
            "uptime_s": round(now - self._started_at, 1),
            "physical": self.physical,
            "state": decision.get("state", "IDLE"),
            "reason": decision.get("reason", "idle"),
            "goal": decision.get("goal"),
            "goal_sector": decision.get("goal_sector"),
            "goal_phase": decision.get("goal_phase"),
            "traveled_m": decision.get("traveled_m"),
            "pose": None if self._pose is None else [round(v, 3) for v in self._pose],
            "scan": {
                "age_s": None if self._scan_age is None else round(self._scan_age, 3),
                "valid": decision.get("valid_samples"),
                "front_m": decision.get("front_m"),
                "left_m": decision.get("left_m"),
                "right_m": decision.get("right_m"),
            },
            "odom_age_s": None if self._odom_age is None else round(self._odom_age, 3),
            "sectors": sectors,
            "proposal": None
            if proposal is None
            else {
                "proposal_id": proposal["proposal_id"],
                "skill": proposal["skill"],
                "sector": proposal["sector"],
                "distance_m": proposal["distance_m"],
                "expires_in_ms": max(0, int((proposal["expires_at"] - now) * 1000)),
            },
            "last_result": self._last_result,
            "last_block": self._last_block,
            "tour": self._tour_summary(),
            "skills": self.cerebellum.skills.names(),
        }

    def heartbeat_frame(self, now: float | None = None) -> dict[str, Any]:
        """One second of truth for consumers: are the sensors still arriving?"""

        now = time.time() if now is None else now
        with self._lock:
            scan_age = self._scan_age
            odom_age = self._odom_age
            decision = self._decision or {}
            block = self._last_block
        scan_ok = scan_age is not None and scan_age <= self.cerebellum.scan_timeout
        odom_ok = odom_age is not None and odom_age <= self.cerebellum.odom_timeout
        return {
            "t": "heartbeat",
            "ts": now,
            "state_seq": self._state_seq,
            "uptime_s": round(now - self._started_at, 1),
            "state": decision.get("state", "IDLE"),
            "scan_age_s": None if scan_age is None else round(scan_age, 3),
            "scan_limit_s": self.cerebellum.scan_timeout,
            "scan_ok": scan_ok,
            "odom_age_s": None if odom_age is None else round(odom_age, 3),
            "odom_limit_s": self.cerebellum.odom_timeout,
            "odom_ok": odom_ok,
            "stale": not (scan_ok and odom_ok),
            "last_block": block,
            "tour_controller_age_s": None
            if self.tour is None or not hasattr(self.tour, "controller_age_s")
            else self.tour.controller_age_s(),
        }

    def scan_frame(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._scan_frame or {"t": "scan", "ranges": [], "ts": time.time()})

    def map_frame(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._map_frame or {"t": "map", "error": "no_map"})

    # -- commands -----------------------------------------------------------
    def tour_frame(self) -> dict[str, Any] | None:
        if self.tour is None or not hasattr(self.tour, "status"):
            return None
        facts = self.tour.status()
        if not facts:
            return None
        return {"t": "tour", "ts": time.time(), "tour": facts}

    def _tour_summary(self) -> dict[str, Any] | None:
        if self.tour is None:
            return None
        facts = self.tour.status() if hasattr(self.tour, "status") else None
        if not facts:
            return {"phase": "UNAVAILABLE", "active": self._tour_active}
        back = facts.get("return") or {}
        return {
            "request_id": facts.get("request_id"),
            "phase": facts.get("phase"),
            "active": bool(self._tour_active),
            "elapsed_s": facts.get("elapsed_s"),
            "dist_to_start_m": back.get("dist_to_start_m"),
            "physical": self.physical,
        }

    def _sync_tour(self) -> None:
        """Turn the controller's own phase into one lifecycle event."""

        if not self._tour_active or self.tour is None:
            return
        facts = self.tour.status()
        if not facts:
            return
        phase = facts.get("phase")
        if phase not in ("DONE", "LIMIT", "STOPPED"):
            return
        self._tour_active = False
        back = facts.get("return") or {}
        grid = facts.get("map") or {}
        self.emit(
            "tour_done",
            reason={"DONE": "complete", "LIMIT": "limit", "STOPPED": "stopped"}[phase],
            request_id=self._tour_request_id,
            elapsed_s=facts.get("elapsed_s"),
            dist_to_start_m=back.get("dist_to_start_m"),
            encoder_path_m=facts.get("encoder_path_m"),
            known_delta_start=grid.get("known_delta_start"),
        )

    def _tour_command(self, message: Mapping[str, Any]) -> dict[str, Any]:
        action = str(message.get("action") or "status").lower()
        if self.tour is None:
            return {"t": "answer", "request": "tour", "accepted": False, "error": "tour_unavailable"}
        if action == "start":
            if self._tour_active:
                return {"t": "answer", "request": "tour", "action": "start", "accepted": False, "error": "tour_active"}
            self.cerebellum.release("tour")
            self._suppress_state_event = True
            with self._lock:
                self._proposal = None
                self._last_result = {"request": "tour", "accepted": True}
            request_id = str(message.get("request_id") or f"tour-{int(time.time() * 1000)}")[:64]
            source = str(message.get("source") or "console")[:32]
            answer = self.tour.start(request_id=request_id, source=source)
            if answer.get("accepted", True):
                self._tour_active = True
                self._tour_request_id = request_id
                self._tour_source = source
                self.emit("tour_started", request_id=request_id, source=source)
            return {"t": "answer", "request": "tour", "action": "start", **answer}
        if action in ("stop", "halt"):
            self.tour.stop(reason="requested")
            was_active = self._tour_active
            self._tour_active = False
            if was_active:
                self.emit("tour_done", reason="stopped", request_id=self._tour_request_id)
            return {"t": "answer", "request": "tour", "action": "stop", "accepted": True}
        facts = self.tour.status()
        return {
            "t": "answer",
            "request": "tour",
            "action": "status",
            "accepted": True,
            "available": facts is not None,
            "facts": facts,
        }

    def _record_block(self, answer: Mapping[str, Any]) -> None:
        """Keep the measurements behind the last thing that did not dispatch."""

        with self._lock:
            self._last_block = {
                "reason": answer.get("error"),
                "skill": answer.get("skill"),
                "sector": answer.get("sector"),
                "clearance_m": answer.get("clearance_m"),
                "threshold_m": answer.get("threshold_m") or answer.get("min_clearance_for_this_skill_m"),
                "scan_age_s": answer.get("scan_age_s"),
                "scan_limit_s": answer.get("scan_limit_s"),
                "odom_age_s": answer.get("odom_age_s"),
                "odom_limit_s": answer.get("odom_limit_s"),
                "at": time.time(),
            }

    def _goal(self, message: Mapping[str, Any]) -> dict[str, Any]:
        """Evaluate and dispatch in one step: this is what a console button sends."""

        source = str(message.get("source") or "console")[:32]
        answer = self.cerebellum.propose(message)
        requested = message.get("goal") or message.get("intent")
        if not answer.get("accepted"):
            self._record_block(answer)
            self.emit(
                "command_failed",
                skill=answer.get("skill") or requested,
                sector=answer.get("sector"),
                reason=answer.get("error"),
                clearance_m=answer.get("clearance_m"),
                threshold_m=answer.get("threshold_m"),
                scan_age_s=answer.get("scan_age_s"),
                scan_limit_s=answer.get("scan_limit_s"),
                odom_age_s=answer.get("odom_age_s"),
                odom_limit_s=answer.get("odom_limit_s"),
            )
            return {
                "t": "answer",
                "request": "goal",
                "dispatched": False,
                "accepted": False,
                **answer,
                "reason": answer.get("error"),
            }

        if not answer.get("requires_execute", True):
            result = self.cerebellum.set_goal(message)
            with self._lock:
                self._last_block = None
                self._last_result = {
                    "request": "goal",
                    "accepted": bool(result.get("accepted")),
                    "skill": answer.get("skill"),
                }
            self.emit("command", skill=answer.get("skill"), source=source)
            return {
                "t": "answer",
                "request": "goal",
                "dispatched": bool(result.get("accepted")),
                "accepted": bool(result.get("accepted")),
                "skill": answer.get("skill"),
                "state": result.get("state"),
            }

        executed = self.cerebellum.execute({"proposal_id": answer["proposal_id"]})
        with self._lock:
            self._proposal = None
            self._last_result = {
                "request": "goal",
                "accepted": bool(executed.get("accepted")),
                "error": executed.get("error"),
                "skill": answer.get("skill"),
            }
            if executed.get("accepted"):
                self._last_block = None
        if not executed.get("accepted"):
            merged = {**answer, **executed, "error": executed.get("error")}
            self._record_block(merged)
            self.emit(
                "command_failed",
                skill=answer.get("skill"),
                sector=answer.get("sector"),
                reason=executed.get("error"),
                clearance_m=executed.get("clearance_m"),
            )
            return {
                "t": "answer",
                "request": "goal",
                "dispatched": False,
                "accepted": False,
                **answer,
                "error": executed.get("error"),
                "reason": executed.get("error"),
                "clearance_m": executed.get("clearance_m", answer.get("clearance_m")),
            }

        self.emit(
            "command",
            skill=answer.get("skill"),
            sector=answer.get("sector"),
            distance_m=answer.get("distance_m"),
            source=source,
        )
        return {
            "t": "answer",
            "request": "goal",
            "dispatched": True,
            "accepted": True,
            "skill": answer.get("skill"),
            "sector": answer.get("sector"),
            "distance_m": answer.get("distance_m"),
            "clearance_m": answer.get("clearance_m"),
            "threshold_m": answer.get("min_clearance_for_this_skill_m"),
            "state": executed.get("state"),
            "scan_age_s": answer.get("scan_age_s"),
            "scan_limit_s": answer.get("scan_limit_s"),
            "odom_age_s": answer.get("odom_age_s"),
            "odom_limit_s": answer.get("odom_limit_s"),
        }

    def command(self, message: Mapping[str, Any]) -> dict[str, Any]:
        kind = str(message.get("t") or message.get("type") or "").lower()
        if kind == "hello":
            return {"t": "hello", "ok": True, "state_hz": STATE_HZ, "skills": self.cerebellum.skills.names()}
        if kind == "tour":
            return self._tour_command(message)
        if kind == "goal":
            if self._tour_active:
                return {
                    "t": "answer",
                    "request": "goal",
                    "dispatched": False,
                    "accepted": False,
                    "error": "tour_active",
                    "reason": "tour_active",
                }
            return self._goal(message)
        if kind == "propose":
            if self._tour_active:
                return {"t": "answer", "request": "propose", "accepted": False, "ok": False, "error": "tour_active"}
            answer = self.cerebellum.propose(message)
            if answer.get("accepted") and answer.get("proposal_id"):
                proposal = self.cerebellum.proposal(answer["proposal_id"])
                with self._lock:
                    self._proposal = (
                        None
                        if proposal is None
                        else {
                            "proposal_id": proposal.proposal_id,
                            "skill": proposal.skill,
                            "sector": proposal.sector,
                            "distance_m": proposal.distance_m,
                            "expires_at": proposal.expires_at,
                        }
                    )
                    self._last_block = None
            else:
                self._record_block(answer)
            return {"t": "answer", "request": "propose", **answer}
        if kind == "execute":
            if self._tour_active:
                return {"t": "answer", "request": "execute", "accepted": False, "ok": False, "error": "tour_active"}
            answer = self.cerebellum.execute(message)
            with self._lock:
                self._proposal = None
                self._last_result = {
                    "request": "execute",
                    "accepted": bool(answer.get("accepted")),
                    "error": answer.get("error"),
                    "goal": answer.get("goal"),
                }
                if answer.get("accepted"):
                    self._last_block = None
            return {"t": "answer", "request": "execute", **answer}
        if kind in ("stop", "halt"):
            reason = str(message.get("reason") or "console_stop")[:120]
            self.cerebellum.cancel("stop")
            was_touring = self._tour_active
            if self.tour is not None:
                self.tour.stop(reason=reason)
            self._tour_active = False
            with self._lock:
                self._proposal = None
                self._last_result = {"request": "stop", "accepted": True, "reason": reason}
            self.emit("stop", reason=reason)
            if was_touring:
                self.emit("tour_done", reason="stopped", request_id=self._tour_request_id)
            return {"t": "answer", "request": "stop", "ok": True, "accepted": True, "state": "DONE", "reason": reason}
        self.emit("command_error", detail=kind[:40])
        return {"t": "error", "error": "unknown_command", "detail": kind[:40]}


class DaemonServer:
    """Accept WebSocket clients and stream at ``state_hz``."""

    def __init__(
        self,
        daemon: RobotDaemon,
        *,
        socket_path: str | None = DEFAULT_SOCKET,
        host: str = "127.0.0.1",
        port: int = 0,
        state_hz: float = STATE_HZ,
        token: str = "",
    ) -> None:
        self.daemon = daemon
        self.socket_path = socket_path
        self.host = host
        self.port = int(port)
        self.state_hz = max(1.0, float(state_hz))
        self.token = token or ""
        self._listener: socket.socket | None = None
        self._threads: list[threading.Thread] = []
        self._clients: dict[socket.socket, dict[str, Any]] = {}
        self._clients_lock = threading.Lock()
        self._running = False
        self._ticks = 0

    @property
    def url(self) -> str:
        if self.socket_path:
            return f"ws+unix://{self.socket_path}"
        return f"ws://{self.host}:{self.port}/ws"

    def start(self) -> None:
        if self._listener is not None:
            return
        if self.socket_path:
            if os.path.exists(self.socket_path):
                os.unlink(self.socket_path)
            os.makedirs(os.path.dirname(self.socket_path), exist_ok=True)
            listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            listener.bind(self.socket_path)
            os.chmod(self.socket_path, 0o600)
        else:
            listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind((self.host, self.port))
            self.port = listener.getsockname()[1]
        listener.listen(8)
        listener.settimeout(0.5)
        self._listener = listener
        self._running = True
        for target, name in ((self._accept_loop, "robotd-accept"), (self._broadcast_loop, "robotd-broadcast")):
            thread = threading.Thread(target=target, name=name, daemon=True)
            thread.start()
            self._threads.append(thread)
        LOGGER.info("robotd listening on %s at %.1f Hz", self.url, self.state_hz)

    def stop(self) -> None:
        self._running = False
        listener = self._listener
        self._listener = None
        if listener is not None:
            try:
                listener.close()
            except OSError:
                pass
        with self._clients_lock:
            for client in list(self._clients):
                try:
                    client.close()
                except OSError:
                    pass
            self._clients.clear()
        if self.socket_path and os.path.exists(self.socket_path):
            try:
                os.unlink(self.socket_path)
            except OSError:
                pass

    def _accept_loop(self) -> None:
        listener = self._listener
        while self._running and listener is not None:
            try:
                client, _address = listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            thread = threading.Thread(target=self._serve_client, args=(client,), name="robotd-client", daemon=True)
            thread.start()
            self._threads.append(thread)

    def _serve_client(self, client: socket.socket) -> None:
        client.settimeout(30.0)
        try:
            wsproto.handshake(client, expected_token=self.token or None)
        except (OSError, wsproto.WebSocketError):
            client.close()
            return
        lock = threading.Lock()
        client_state = {"lock": lock, "event_seq": max(0, self.daemon.event_seq - 50)}
        with self._clients_lock:
            self._clients[client] = client_state
        try:
            with lock:
                wsproto.send_json(
                    client,
                    {"t": "hello", "ok": True, "state_hz": self.state_hz, "skills": self.daemon.cerebellum.skills.names()},
                )
                backlog = self.daemon.events_since(client_state["event_seq"])
                if backlog:
                    wsproto.send_json(client, {"t": "events", "items": backlog})
                wsproto.send_json(client, self.daemon.heartbeat_frame())
                client_state["event_seq"] = self.daemon.event_seq
            while self._running:
                try:
                    message = wsproto.recv_json(client)
                except socket.timeout:
                    # A tour holds the socket open for minutes and may send no
                    # commands at all; an idle read is not a dead client.
                    continue
                answer = self.daemon.command(message)
                with lock:
                    wsproto.send_json(client, answer)
        except (OSError, wsproto.WebSocketError, ValueError):
            pass
        finally:
            with self._clients_lock:
                self._clients.pop(client, None)
            try:
                client.close()
            except OSError:
                pass

    def _broadcast_loop(self) -> None:
        period = 1.0 / self.state_hz
        next_at = time.monotonic()
        while self._running:
            next_at += period
            delay = next_at - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_at = time.monotonic()
            self._ticks += 1
            frames = [self.daemon.state_frame()]
            if self._ticks % SCAN_EVERY == 0:
                frames.append(self.daemon.scan_frame())
            if self._ticks % MAP_EVERY == 0:
                frames.append(self.daemon.map_frame())
            if self._ticks % max(1, int(round(self.state_hz))) == 0:
                frames.append(self.daemon.heartbeat_frame())
                tour = self.daemon.tour_frame()
                if tour is not None:
                    frames.append(tour)
            with self._clients_lock:
                clients = list(self._clients.items())
            for client, client_state in clients:
                events = self.daemon.events_since(client_state["event_seq"])
                try:
                    with client_state["lock"]:
                        for frame in frames:
                            wsproto.send_json(client, frame)
                        if events:
                            wsproto.send_json(client, {"t": "events", "items": events})
                    if events:
                        client_state["event_seq"] = events[-1]["seq"]
                except (OSError, wsproto.WebSocketError):
                    with self._clients_lock:
                        self._clients.pop(client, None)
                    try:
                        client.close()
                    except OSError:
                        pass


def main(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description="robotd offline mock (synthetic loop, TCP WebSocket)")
    parser.add_argument("--tcp", action="store_true", help="listen on TCP instead of a Unix socket")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18202)
    parser.add_argument("--hz", type=float, default=STATE_HZ)
    parser.add_argument("--token", default="")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    import math

    control = Cerebellum()
    daemon = RobotDaemon(control, physical=False)
    width = height = 120
    cells = []
    for y in range(height):
        for x in range(width):
            edge = x in (0, width - 1) or y in (0, height - 1)
            wall = 20 <= x <= 22 and 20 <= y <= 90
            cells.append(100 if edge or wall else 0)
    daemon.set_map(width=width, height=height, resolution=0.05, origin=(-3.0, -3.0), data=cells)
    ranges = [2.0] * 360
    for index in range(360):
        angle = -math.pi + index * (2 * math.pi / 360)
        if abs(angle) < math.radians(20):
            ranges[index] = 0.9
    daemon.set_scan(-math.pi, 2 * math.pi / 360, ranges, age_s=0.1)

    server = DaemonServer(
        daemon,
        socket_path=None,
        host=args.host,
        port=args.port,
        state_hz=args.hz,
        token=args.token,
    )
    if not args.tcp:
        parser.error("use --tcp for the offline mock; the real deployment uses a Unix socket")
    server.start()
    print(f"robotd mock: {server.url}", flush=True)

    profile = profile_scan(ranges, -math.pi, 2 * math.pi / 360)
    started = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            daemon.tick(now=now, profile=profile, pose=(0.0, 0.0, 0.0), scan_age_s=0.1, odom_age_s=0.05)
            time.sleep(max(0.0, 1.0 / args.hz - (time.monotonic() - now)))
            if time.monotonic() - started > 3600:
                break
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
