"""Deterministic clock, real production simulator/executor cycle, no sockets."""

import argparse
import json
import math
from pathlib import Path

from agent_control.cerebellum import Cerebellum
from agent_control.strategy import StrategyRunner
from sim.local_sim import LocalSimulator, Scene, StrategyDaemon, SIM_ASSETS


def make_sim(world="room", pose=(0.0, 0.0, 0.0)):
    control = Cerebellum()
    runner = StrategyRunner(obstacle_stop_m=control.min_clearance,
                            max_linear_mps=control.max_linear, max_angular_rps=control.max_angular,
                            scan_timeout_s=control.scan_timeout, odom_timeout_s=control.odom_timeout,
                            min_clearance_m=control.min_clearance,
                            min_clearance_turn_m=control.min_clearance_turn)
    daemon = StrategyDaemon(control, physical=False, strategy=runner)
    scene = Scene.from_sdf(SIM_ASSETS / "worlds" / f"{world}.world")
    sim = LocalSimulator(daemon, scene)
    sim.select_scene(scene)
    sim.pose = pose
    return sim, runner


def run(world="room", pose=(0.0, 0.0, 0.0), dt=0.1):
    sim, runner = make_sim(world, pose)
    sim.step(0.0, 0.0)  # sensors arrive before the task, as in the hosted service
    runner.start(strategy="tour", variant_id="room/default/01" if world == "room" else "room_open/wide/01", now=0)
    trace = []
    last = None
    collisions = 0
    for index in range(int(310 / dt)):
        now = (index + 1) * dt
        before = sim.pose
        sim.step(now, dt)
        status = runner.status()
        navigation = (status.get("tour") or {}).get("navigation") or {}
        # status puts backend facts under coverage for backwards compatibility.
        if not navigation:
            navigation = (status.get("backend") or status.get("coverage") or {}).get("navigation") or {}
        key = (status["phase"], navigation.get("explorer_state"),
               (navigation.get("recovery") or {}).get("stage"))
        if key != last or index % max(1, round(10 / dt)) == 0:
            row = {"t": round(now, 2), "state": status["phase"], "reason": status["reason"],
                   "pose": [round(v, 4) for v in sim.pose], "known": sim.known_cells,
                   "traveled": round(sim.traveled_m, 4), "applied": sim.last_applied, "nav": navigation}
            trace.append(row)
            print(json.dumps(row), flush=True)
            last = key
        if sim.last_applied[0] and sim.pose[:2] == before[:2]:
            collisions += 1
        if not status["busy"]:
            break
    home = math.dist(sim.pose[:2], pose[:2])
    ok = status["phase"] == "DONE" and status["reason"] == "reached" and home <= 0.22 and collisions == 0
    return {"ok": ok, "world": world, "start": pose, "dt": dt, "physical": False,
            "home_distance": home, "collision_rejections": collisions,
            "status": status, "trace": trace}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", default="room")
    parser.add_argument("--pose", nargs=3, type=float, default=[0., 0., 0.])
    parser.add_argument("--dt", type=float, default=.1)
    parser.add_argument("--output", default="tmp/tour-regression.json")
    args = parser.parse_args()
    result = run(args.world, tuple(args.pose), args.dt)
    path = Path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps({k: result[k] for k in ("ok", "home_distance", "collision_rejections")}))
    raise SystemExit(0 if result["ok"] else 1)
