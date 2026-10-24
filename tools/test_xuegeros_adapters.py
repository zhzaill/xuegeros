from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parent))

from xuegeros_body_adapter import (
    LatestFrame,
    VisionState,
    build_world,
    extract_mjpeg_frames,
    get_observation,
    normalize_status,
    normalize_detections,
    summarize_mapping,
    summarize_navigation,
)
from xuegeros_motion_gateway import MotionState


class BodyAdapterTests(unittest.TestCase):
    def test_latest_frame_overwrites_old_frame_and_counts_drops(self) -> None:
        frames = LatestFrame()
        first = frames.publish(b"first", observed_at=10.0)
        second = frames.publish(b"second", observed_at=11.0)
        self.assertEqual(second, first + 1)
        self.assertEqual(frames.dropped_frames, 1)
        self.assertEqual(frames.take_newest(0, timeout=0), (second, 11.0, b"second"))
        frames.publish(b"third", observed_at=12.0)
        self.assertEqual(frames.dropped_frames, 1)

    def test_mjpeg_parser_yields_only_complete_bounded_jpegs(self) -> None:
        class Stream:
            def iter_content(self, chunk_size):
                yield b"boundary\xff\xd8frame\xff\xd9tail\xff\xd8unfinished"

        self.assertEqual(list(extract_mjpeg_frames(Stream(), max_frame_bytes=32)), [b"\xff\xd8frame\xff\xd9"])

    def test_detections_are_normalized_and_capped_at_twenty(self) -> None:
        class Value:
            def __init__(self, value):
                self.value = value

            def tolist(self):
                return self.value

            def item(self):
                return self.value

        class Box:
            def __init__(self, index):
                self.xyxy = [Value([-10, 20, 50, 120])]
                self.cls = [Value(index)]
                self.conf = [Value(0.9)]

        class Result:
            boxes = [Box(index) for index in range(25)]
            names = {index: f"class-{index}" for index in range(25)}

        detections = normalize_detections(Result(), width=100, height=100)
        self.assertEqual(len(detections), 20)
        self.assertEqual(detections[0]["bbox"], [0.0, 0.2, 0.5, 1.0])
        self.assertEqual(detections[0]["class"], "class-0")

    def test_disconnected_camera_fails_closed_without_stale_detections(self) -> None:
        frames = LatestFrame()
        state = VisionState()
        state.publish({
            "device_id": "xuegeros-car-01",
            "camera_state": "streaming",
            "frame_seq": 1,
            "frame_timestamp": time.time() - 10,
            "observed_at": time.time(),
            "detection_count": 1,
            "detections": [{"class": "person"}],
        })
        frames.set_state("disconnected", "offline")
        result = state.snapshot(frames)
        self.assertEqual(result["camera_state"], "disconnected")
        self.assertEqual(result["detections"], [])
        self.assertFalse(result["raw_frame_available"])
        self.assertFalse(result["autonomous_motion_allowed"])

    def test_old_frame_timestamp_fails_closed_even_after_recent_inference(self) -> None:
        frames = LatestFrame()
        frames.set_state("streaming")
        state = VisionState()
        state.publish({
            "device_id": "xuegeros-car-01",
            "camera_state": "streaming",
            "frame_seq": 2,
            "frame_timestamp": time.time() - 10,
            "observed_at": time.time(),
            "detection_count": 1,
            "detections": [{"class": "person"}],
        })
        result = state.snapshot(frames)
        self.assertGreater(result["frame_age_ms"], 3000)
        self.assertEqual(result["detections"], [])
        self.assertEqual(result["detection_count"], 0)

    def test_status_is_bounded_and_marks_motion_disallowed(self) -> None:
        payload = {
            "host": {
                "vmRunning": True,
                "vmCount": 1,
                "toolsState": "running",
                "udpEndpoints": [
                    {"LocalPort": 8888},
                    {"LocalPort": 8889},
                ],
            },
            "guest": (
                "NETWORK\nens33 inet 192.168.253.128/24\n"
                "PROCESSES\n2055 micro_ros_agent\n2500 http_video_publisher\n"
                "UDP\n0.0.0.0:8888\n"
                "AGENT_LOG\nrunning port 8888"
            ),
        }
        status = normalize_status(payload)
        self.assertEqual(status["device_id"], "xuegeros-car-01")
        self.assertTrue(status["vm"]["running"])
        self.assertTrue(status["network"]["udp_8888_listening"])
        self.assertTrue(status["micro_ros"]["agent_running"])
        self.assertTrue(status["camera"]["ros_node_running"])
        self.assertFalse(status["autonomous_motion_allowed"])
        self.assertFalse(status["raw_logs_included"])

    def test_mapping_requires_scan_map_and_tf(self) -> None:
        summary = summarize_mapping(
            {
                "scan_present": True,
                "map_present": True,
                "tf_present": False,
                "gmapping_running": True,
                "raw": "bounded",
            }
        )
        self.assertTrue(summary["scan"]["present"])
        self.assertTrue(summary["map"]["present"])
        self.assertFalse(summary["tf"]["present"])
        self.assertFalse(summary["mapping_healthy"])
        self.assertNotIn("summary", summary)

    def test_nonzero_micro_ros_client_key_marks_session_observed(self) -> None:
        payload = {
            "host": {"vmRunning": True, "vmCount": 1, "udpEndpoints": []},
            "guest": (
                "PROCESSES\nmicro_ros_agent\n"
                "AGENT_LOG\nclient_key: 0x272A4A6B, len: 332"
            ),
        }
        status = normalize_status(payload)
        self.assertTrue(status["micro_ros"]["session_observed"])

    def test_navigation_requires_nav2_even_when_sensor_graph_is_healthy(self) -> None:
        status = {
            "vm": {"running": True},
            "micro_ros": {"session_observed": True},
            "navigation": {"nav2_running": False},
        }
        mapping = {
            "scan": {"present": True},
            "map": {"present": True},
            "tf": {"present": True},
        }
        result = summarize_navigation(status, mapping)
        self.assertFalse(result["navigation_ready"])
        self.assertIn("nav2_not_running", result["blockers"])

    def test_world_keeps_vision_available_when_robot_status_is_offline(self) -> None:
        vision = {
            "device_id": "xuegeros-car-01",
            "camera_state": "streaming",
            "detection_count": 1,
            "detections": [{"class": "person"}],
            "raw_frame_available": False,
        }
        world = build_world(None, None, vision)
        self.assertFalse(world["status_available"])
        self.assertFalse(world["mapping_available"])
        self.assertEqual(world["vision"]["detections"], [{"class": "person"}])
        self.assertFalse(world["autonomous_motion_allowed"])


class MotionGatewayTests(unittest.TestCase):
    def make_state(self) -> MotionState:
        return MotionState(
            enabled=True,
            shadow_mode=True,
            operator_token="human-only-token",
            lease_max_ms=1000,
            watchdog_timeout_ms=200,
            max_linear=0.1,
            max_angular=0.5,
        )

    def test_operator_token_is_required(self) -> None:
        state = self.make_state()
        self.assertFalse(state.authorized(None))
        self.assertFalse(state.authorized("wrong"))
        self.assertTrue(state.authorized("human-only-token"))

    def test_only_one_short_lease_can_be_active(self) -> None:
        state = self.make_state()
        first = state.issue_lease(500, "test", now=10.0)
        self.assertTrue(first["ok"])
        second = state.issue_lease(500, "test", now=10.1)
        self.assertFalse(second["ok"])
        self.assertEqual(second["error"], "lease_already_active")

    def test_velocity_limits_and_shadow_mode(self) -> None:
        state = self.make_state()
        lease = state.issue_lease(500, "test", now=20.0)
        lease_id = lease["lease_id"]
        too_fast = state.command(lease_id, 0.2, 0.0, now=20.1)
        self.assertFalse(too_fast["ok"])
        self.assertEqual(too_fast["error"], "velocity_limit")
        accepted = state.command(lease_id, 0.05, 0.1, now=20.2)
        self.assertTrue(accepted["ok"])
        self.assertFalse(accepted["published"])
        self.assertTrue(accepted["shadow_mode"])

    def test_watchdog_clears_stale_command(self) -> None:
        state = self.make_state()
        lease = state.issue_lease(1000, "test", now=30.0)
        lease_id = lease["lease_id"]
        self.assertTrue(state.command(lease_id, 0.05, 0.0, now=30.05)["ok"])
        self.assertFalse(state.tick(now=30.20))
        self.assertTrue(state.tick(now=30.26))
        status = state.status(now=30.3)
        self.assertIsNone(status["active_lease"])
        self.assertEqual(status["safe_stop_events"], 1)


if __name__ == "__main__":
    unittest.main()
