"""Local Coo panel RPC -> World.tour -> robotd -> executor, with receipts."""

import json
import socket
import time
from pathlib import Path
from urllib.request import Request, urlopen

BASE = "http://127.0.0.1:17788/api/webui/providers/world%3Axuegeros/panels/live/"
SIM = "http://127.0.0.1:18142"


def request(url, body=None):
    data = None if body is None else json.dumps(body).encode()
    with urlopen(Request(url, data=data, headers={"Content-Type": "application/json"}), timeout=8) as response:
        return json.load(response)


def tour(action, **args):
    return request(BASE + "tour", {"args": [{"action": action, **args}]})


def state():
    result = request(SIM + "/api/state")
    assert result["physical"] is False
    return result


def fresh():
    request(SIM + "/api/scene", {"name": "room.world"})
    request(SIM + "/api/sim/pose", {"x": 0, "y": 0, "yaw": 0})
    time.sleep(.3)


def run():
    with socket.socket() as sock:
        assert sock.connect_ex(("127.0.0.1", 18202)) != 0, "physical port is open"
    receipts = {"physical": False, "samples": []}
    path = Path("tmp/coo-tour-acceptance.json")
    try:
        fresh()
        receipts["lifecycle_start"] = tour("start", variant_id="room/default/01")
        assert receipts["lifecycle_start"]["accepted"]
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            status = tour("status")["strategy"]
            if status["phase"] == "RUNNING" and state()["traveled_m"] > .02:
                break
            time.sleep(.1)
        assert status["phase"] == "RUNNING"
        receipts["exclusion"] = request(BASE + "goal", {"args": [{"intent": "forward"}]})
        assert receipts["exclusion"]["reason"] == "strategy_active"
        receipts["pause"] = tour("pause")
        assert receipts["pause"]["accepted"]
        # Let any in-flight integration cycle finish, then watch multiple later cycles.
        time.sleep(.15)
        paused = state()["pose"]
        for _ in range(5):
            time.sleep(.1)
            assert state()["pose"] == paused
        receipts["resume"] = tour("resume")
        assert receipts["resume"]["accepted"]
        time.sleep(.5)
        receipts["stop"] = request(BASE + "stop", {"args": []})
        assert receipts["stop"]["accepted"]
        time.sleep(.15)
        stopped = state()["pose"]
        for _ in range(5):
            time.sleep(.1)
            assert state()["pose"] == stopped
        receipts["stop_pose"] = stopped
        assert tour("status")["strategy"]["busy"] is False
        fresh()
        receipts["start"] = tour("start", variant_id="room/default/01")
        assert receipts["start"]["accepted"]
        assert receipts["start"]["strategy"]["strategy"] == "tour"
        deadline = time.monotonic() + 320
        cursor = 0
        while time.monotonic() < deadline:
            answer = tour("status")
            status = answer["strategy"]
            assert status["physical"] is False
            assert status["events_cursor"] >= cursor
            cursor = status["events_cursor"]
            receipts["samples"].append(status)
            print(json.dumps({"phase": status["phase"], "elapsed": status["elapsed_s"],
                              "known": status["facts"].get("known_cells"), "cursor": cursor}), flush=True)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(receipts, indent=2), encoding="utf-8")
            if not status["busy"]:
                break
            time.sleep(1)
        receipts["result"] = answer
        assert status["phase"] == "DONE" and status["reason"] == "reached", status["reason"]
        assert status["acceptance_result"]["met"] is True
        time.sleep(.3)
        receipts["coo_state"] = request(BASE + "state")
        assert receipts["coo_state"]["tour"]["phase"] == "DONE"
        receipts["ok"] = True
        return receipts
    except Exception as exc:
        receipts["ok"] = False
        receipts["error"] = str(exc)
        try:
            receipts["failure_stop"] = tour("stop")
        except Exception:
            pass
        raise
    finally:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(receipts, indent=2), encoding="utf-8")


if __name__ == "__main__":
    result = run()
    print(json.dumps({"ok": result["ok"], "result": result["result"]["strategy"]["reason"]}))
