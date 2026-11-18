import json
import math
import os
import socket
import tempfile
import time
import unittest

from agent_control import wsproto
from agent_control.cerebellum import Cerebellum, profile_scan
from agent_control.robotd import DaemonServer, RobotDaemon

FULL_TURN = 2.0 * math.pi


def synthetic_ranges(front=0.9, other=2.0, points=360):
    ranges = []
    angle_min = -math.pi
    increment = FULL_TURN / points
    for index in range(points):
        angle = angle_min + index * increment
        ranges.append(front if abs(angle) < math.radians(20) else other)
    return ranges, angle_min, increment


def make_daemon(tour=None):
    control = Cerebellum()
    daemon = RobotDaemon(control, tour=tour)
    ranges, angle_min, increment = synthetic_ranges()
    daemon.set_scan(angle_min, increment, ranges, age_s=0.1)
    daemon.set_map(
        width=40,
        height=40,
        resolution=0.05,
        origin=(-1.0, -1.0),
        data=[0] * 1600,
    )
    profile = profile_scan(ranges, angle_min, increment)
    daemon.tick(
        now=time.monotonic(),
        profile=profile,
        pose=(0.0, 0.0, 0.0),
        scan_age_s=0.1,
        odom_age_s=0.05,
    )
    return daemon


class FakeTour:
    def __init__(self):
        self.facts = {
            "request_id": None,
            "phase": "IDLE",
            "active": False,
            "elapsed_s": None,
            "return": {"dist_to_start_m": None},
            "map": {"known_delta_start": 0},
            "encoder_path_m": 0.0,
        }
        self.started = False
        self.stopped = False

    def status(self):
        return dict(self.facts)

    def controller_age_s(self):
        return 0.4

    def start(self, *, request_id, source):
        self.started = True
        self.facts = {
            "request_id": request_id,
            "phase": "EXPLORE",
            "active": True,
            "elapsed_s": 0.0,
            "return": {"dist_to_start_m": 1.2},
            "map": {"known_delta_start": 0},
            "encoder_path_m": 0.0,
        }
        return {"accepted": True, "request_id": request_id, "physical": False}

    def stop(self, *, reason="requested"):
        self.stopped = True
        self.facts = {**self.facts, "phase": "STOPPED", "active": False}


class TourCommandTests(unittest.TestCase):
    def test_start_returns_a_handle_and_emits_one_event(self):
        daemon = make_daemon(FakeTour())
        answer = daemon.command({"t": "tour", "action": "start", "source": "test", "request_id": "t-1"})
        self.assertTrue(answer["accepted"], answer)
        self.assertEqual(answer["request"], "tour")
        self.assertEqual(answer["request_id"], "t-1")
        kinds = [event["kind"] for event in daemon.events_since(0)]
        self.assertIn("tour_started", kinds)
        self.assertIn("tour", daemon.state_frame())

    def test_goal_is_refused_while_touring(self):
        daemon = make_daemon(FakeTour())
        daemon.command({"t": "tour", "action": "start"})
        answer = daemon.command({"t": "goal", "goal": "forward", "sector": 6})
        self.assertFalse(answer["dispatched"], answer)
        self.assertEqual(answer["error"], "tour_active")

    def test_stop_ends_the_tour(self):
        tour = FakeTour()
        daemon = make_daemon(tour)
        daemon.command({"t": "tour", "action": "start"})
        daemon.command({"t": "stop", "reason": "operator"})
        self.assertTrue(tour.stopped)
        kinds = [event["kind"] for event in daemon.events_since(0)]
        self.assertIn("tour_done", kinds)

    def test_finished_phase_becomes_one_tour_done_event(self):
        tour = FakeTour()
        daemon = make_daemon(tour)
        daemon.command({"t": "tour", "action": "start"})
        tour.facts = {
            **tour.facts,
            "phase": "DONE",
            "active": False,
            "elapsed_s": 12.0,
            "return": {"dist_to_start_m": 0.1},
            "map": {"known_delta_start": 120},
            "encoder_path_m": 2.0,
        }
        ranges, angle_min, increment = synthetic_ranges()
        daemon.tick(
            now=time.monotonic(),
            profile=profile_scan(ranges, angle_min, increment),
            pose=(0.0, 0.0, 0.0),
            scan_age_s=0.1,
            odom_age_s=0.05,
        )
        done = [event for event in daemon.events_since(0) if event["kind"] == "tour_done"]
        self.assertEqual(len(done), 1)
        self.assertEqual(done[0]["reason"], "complete")
        self.assertEqual(done[0]["known_delta_start"], 120)

    def test_status_is_unavailable_without_a_link(self):
        daemon = make_daemon()
        answer = daemon.command({"t": "tour", "action": "status"})
        self.assertFalse(answer["accepted"])
        self.assertEqual(answer["error"], "tour_unavailable")


def recv_until(sock, predicate, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        opcode, payload = wsproto.read_frame(sock)
        if opcode != wsproto.OP_TEXT:
            continue
        value = json.loads(payload.decode("utf-8"))
        if predicate(value):
            return value
    raise AssertionError("no matching frame")


class WebSocketDaemonTests(unittest.TestCase):
    def setUp(self):
        self.server = DaemonServer(make_daemon(), socket_path=None, host="127.0.0.1", port=0)
        self.server.start()
        self.client = socket.create_connection(("127.0.0.1", self.server.port), timeout=2.0)
        wsproto.client_handshake(self.client, host=f"127.0.0.1:{self.server.port}")

    def tearDown(self):
        self.client.close()
        self.server.stop()

    def test_streams_state_at_fifty_hertz(self):
        started = time.monotonic()
        frames = 0
        while time.monotonic() - started < 0.4:
            opcode, payload = wsproto.read_frame(self.client)
            if opcode == wsproto.OP_TEXT:
                value = json.loads(payload.decode("utf-8"))
                if value.get("t") == "state":
                    frames += 1
        self.assertGreaterEqual(frames, 12, f"only {frames} state frames in 0.4 s")

    def test_propose_then_execute(self):
        wsproto.send_client_json(self.client, {"t": "propose", "goal": "forward", "sector": 6, "distance_m": 0.12})
        answer = recv_until(self.client, lambda value: value.get("t") == "answer")
        self.assertEqual(answer["request"], "propose")
        self.assertTrue(answer["accepted"], answer)
        proposal_id = answer["proposal_id"]

        wsproto.send_client_json(self.client, {"t": "execute", "proposal_id": proposal_id})
        result = recv_until(
            self.client,
            lambda value: value.get("t") == "answer" and value.get("request") == "execute",
        )
        self.assertTrue(result["accepted"], result)
        self.assertEqual(result["goal"], "forward")

    def test_refused_proposal_names_the_reason(self):
        blocked = make_daemon()
        ranges, angle_min, increment = synthetic_ranges(front=0.2)
        blocked.set_scan(angle_min, increment, ranges, age_s=0.1)
        blocked.tick(
            now=time.monotonic(),
            profile=profile_scan(ranges, angle_min, increment),
            pose=(0.0, 0.0, 0.0),
            scan_age_s=0.1,
            odom_age_s=0.05,
        )
        answer = blocked.command({"t": "propose", "goal": "forward", "sector": 6})
        self.assertFalse(answer["accepted"])
        self.assertTrue(answer["error"].startswith("obstacle:"), answer)

    def test_goal_dispatches_in_one_command(self):
        answer = self.server.daemon.command(
            {"t": "goal", "goal": "forward", "sector": 6, "distance_m": 0.12, "source": "console"}
        )
        self.assertTrue(answer["dispatched"], answer)
        self.assertEqual(answer["request"], "goal")
        self.assertEqual(answer["skill"], "forward")
        self.assertNotIn("proposal_id", answer)
        kinds = [event["kind"] for event in self.server.daemon.events_since(0)]
        self.assertIn("command", kinds)

    def test_goal_names_the_measurement_that_stopped_it(self):
        blocked = make_daemon()
        ranges, angle_min, increment = synthetic_ranges(front=0.2)
        blocked.set_scan(angle_min, increment, ranges, age_s=0.1)
        blocked.tick(
            now=time.monotonic(),
            profile=profile_scan(ranges, angle_min, increment),
            pose=(0.0, 0.0, 0.0),
            scan_age_s=0.1,
            odom_age_s=0.05,
        )
        answer = blocked.command({"t": "goal", "goal": "forward", "sector": 6})
        self.assertFalse(answer["dispatched"], answer)
        self.assertTrue(answer["reason"].startswith("obstacle:"), answer)
        self.assertLess(answer["clearance_m"], answer["threshold_m"])
        kinds = [event["kind"] for event in blocked.events_since(0)]
        self.assertIn("command_failed", kinds)

    def test_heartbeat_reports_sensor_health(self):
        heartbeat = recv_until(self.client, lambda value: value.get("t") == "heartbeat", timeout=3.0)
        self.assertIn("scan_ok", heartbeat)
        self.assertIn("scan_limit_s", heartbeat)
        self.assertIn("odom_ok", heartbeat)
        self.assertTrue(heartbeat["scan_ok"])
        self.assertFalse(heartbeat["stale"])

    def test_repeated_identical_blocks_collapse(self):
        daemon = make_daemon()
        for _ in range(3):
            daemon.emit("motion_blocked", skill="forward", reason="scan_stale")
        events = [event for event in daemon.events_since(0) if event["kind"] == "motion_blocked"]
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["count"], 3)

    def test_stop_is_accepted_without_a_proposal(self):
        answer = self.server.daemon.command({"t": "stop", "reason": "test"})
        self.assertTrue(answer["accepted"])
        self.assertEqual(answer["state"], "DONE")

    def test_event_stream_carries_discrete_events(self):
        wsproto.send_client_json(self.client, {"t": "stop", "reason": "event_test"})
        kinds = []
        end = time.monotonic() + 2.0
        while time.monotonic() < end and "stop" not in kinds:
            opcode, payload = wsproto.read_frame(self.client)
            if opcode != wsproto.OP_TEXT:
                continue
            message = json.loads(payload.decode("utf-8"))
            if message.get("t") == "events":
                kinds.extend(item.get("kind") for item in message.get("items", []))
        self.assertIn("stop", kinds)
        self.assertNotIn("proposal_refused", kinds)


class TokenTests(unittest.TestCase):
    def test_token_is_required_when_configured(self):
        server = DaemonServer(make_daemon(), socket_path=None, host="127.0.0.1", port=0, token="s3cret")
        server.start()
        try:
            denied = socket.create_connection(("127.0.0.1", server.port), timeout=2.0)
            with self.assertRaises(wsproto.WebSocketError):
                wsproto.client_handshake(denied, host="127.0.0.1", path="/ws")
            denied.close()

            allowed = socket.create_connection(("127.0.0.1", server.port), timeout=2.0)
            wsproto.client_handshake(allowed, host="127.0.0.1", path="/ws?token=s3cret")
            state = recv_until(allowed, lambda value: value.get("t") == "state")
            self.assertEqual(state["t"], "state")
            allowed.close()
        finally:
            server.stop()


class UnixSocketTests(unittest.TestCase):
    @unittest.skipUnless(hasattr(socket, "AF_UNIX"), "Unix sockets are not available on this platform")
    def test_daemon_speaks_websocket_over_a_unix_socket(self):
        with tempfile.TemporaryDirectory() as folder:
            path = os.path.join(folder, "robotd.sock")
            server = DaemonServer(make_daemon(), socket_path=path)
            server.start()
            try:
                client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                client.settimeout(2.0)
                client.connect(path)
                wsproto.client_handshake(client, host="localhost")
                state = recv_until(client, lambda value: value.get("t") == "state")
                self.assertEqual(state["t"], "state")
                self.assertTrue(state["sectors"])
                client.close()
                self.assertEqual(os.stat(path).st_mode & 0o777, 0o600)
            finally:
                server.stop()


if __name__ == "__main__":
    unittest.main()
