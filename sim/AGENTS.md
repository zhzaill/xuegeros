# Local simulation tool handoff

This file covers `local_sim.py`, `control.py`, `observation.py` and
`local_web/{app.js,control.js,index.html}`. Existing `guest/AGENTS.md` covers ROS.

Entry: `python -m sim.local_sim --http-port 18142 --port 18242` from the repo
or the npm runtime tree. Python 3.10+; Pillow is optional for `/api/map.png`.
HTTP and WS bind loopback by default; `--host 0.0.0.0` enables LAN. `physical=false`. Actions always use StrategyDaemon /
robotd and Cerebellum; never add a raw velocity HTTP endpoint.

`GET /api/interfaces` lists the routes and actual WS port. `POST /api/command`
forwards the existing named robotd commands. `GET/POST /api/control` reads/writes
24 numeric parameters. `--config` defaults to `~/.xuegeros/simulation.json`.
The config is validated in full, atomically replaced on disk, then applied under
the shared reentrant lock used by command and simulation step. Goal, guide and
strategy leases (including pause) reject tuning with 409. Runner is preserved:
events/cursor do not reset. Shared speed/freshness/clearance values are kept in
sync; running controller instances cannot be retuned mid-task. Offers computed
under old limits are invalidated. The schema in `control.py` is the UI source.

`--mesh-dir` chooses external STL files, used consistently by metadata and asset
serving. Without meshes the workbench uses the procedural renderer. The npm
runtime builder deliberately excludes CAD, model weights and ROS dependencies.
Workbenches discover their socket from `/api/interfaces`, including custom ports.
Coo uses its own RPC to edit these same values; the 3D view remains independent.

Checks: `python -m unittest discover -s sim/tests` (no `-t .`),
`python -m unittest discover -s agent_control/tests -t .`. For package changes,
build and install the tarball in a clean folder, start with distinct ports and
config, verify actual bounded motion and stop, then shut down that test process.

Limits: kinematic / 2D geometry, not contact dynamics or SLAM. Scope of tour
acceptance remains the fixed local room; cameras, ultrasound and IMU are absent.
The scheduled local launcher enables LAN by request. Service discovery derives
the WS hostname from the HTTP Host, with the actual configured robotd port.
Windows Firewall changes require administrator rights. Changing config cannot
tune arbitrary backend internals.
