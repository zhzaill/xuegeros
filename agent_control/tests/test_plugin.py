import json
import threading
import unittest
import urllib.error
import urllib.request
from unittest.mock import patch
from http.server import ThreadingHTTPServer

from agent_control import plugin_server


def local_opener():
    """Bypass the Windows system proxy.

    Measured 2026-11-12: ``urllib.request.getproxies()`` returns
    ``http://127.0.0.1:26561`` from the WinINET registry, so a plain
    ``urlopen`` to a loopback test server is sent to the proxy and answers an
    empty 404. Every target here is loopback, so the proxy is never wanted.
    """

    return urllib.request.build_opener(urllib.request.ProxyHandler({}))


class PluginTests(unittest.TestCase):
    def setUp(self):
        self.old_world = plugin_server.get_world
        self.old_token = plugin_server.PLUGIN_TOKEN
        self.old_physical = plugin_server.PHYSICAL_ENABLED
        plugin_server.PLUGIN_TOKEN = "test-token"
        plugin_server.PHYSICAL_ENABLED = False
        plugin_server.get_world = lambda: {
            "observed_at": __import__("time").time(),
            "status": {"micro_ros": {"session_observed": True}, "battery": {"voltage": 8.1}},
            "mapping": {
                "scan": {"present": True, "age_s": 0.05},
                "odom": {"present": True, "age_s": 0.05},
                "map": {"present": True, "age_s": 0.05},
                "tf": {"present": True, "map_to_odom": True, "age_s": 0.05},
                "rates": {"scan_observed": True},
                "mapping_healthy": True,
            },
        }
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), plugin_server.PluginHandler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        plugin_server.get_world = self.old_world
        plugin_server.PLUGIN_TOKEN = self.old_token
        plugin_server.PHYSICAL_ENABLED = self.old_physical

    def post(self, path, body):
        request = urllib.request.Request(
            f"http://127.0.0.1:{self.server.server_port}{path}",
            data=json.dumps(body).encode(),
            headers={"content-type": "application/json", "X-Agent-Token": "test-token"},
            method="POST",
        )
        with local_opener().open(request) as response:
            return json.loads(response.read())

    def test_shadow_proposal_never_calls_gateway(self):
        result = self.post("/v1/propose", {"session_id": "s1", "turn_id": "t1", "proposal": {"linear_mps": 0.02, "angular_rps": 0, "duration_ms": 200}})
        self.assertTrue(result["ok"])
        self.assertTrue(result["shadow"])
        self.assertFalse(result["decision"]["clamped"] is None)
        self.assertTrue(result["event_id"])

    def test_missing_plugin_token_denies_requests(self):
        plugin_server.PLUGIN_TOKEN = ""
        request = urllib.request.Request(f"http://127.0.0.1:{self.server.server_port}/health")
        with self.assertRaises(urllib.error.HTTPError) as raised:
            local_opener().open(request)
        self.assertEqual(raised.exception.code, 403)

    def test_physical_gateway_pulse_refuses_shadow_gateway(self):
        status = {
            "enabled": True,
            "shadow_mode": True,
            "operator_token_configured": True,
            "physical_backend_available": True,
        }
        with patch.object(plugin_server, "_json_request", return_value=status) as request:
            result = plugin_server.gateway_pulse(
                plugin_server.ActionProposal(0.02, 0.0, 200)
            )
        self.assertEqual(result["error"], "motion_gateway_shadow_mode")
        request.assert_called_once_with(plugin_server.GATEWAY_URL.rstrip("/") + "/v1/status")


if __name__ == "__main__":
    unittest.main()
