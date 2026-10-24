# Xuegeros Operation Log

Date: 2026-10-22 (Asia/Shanghai)

## Server And VM

- Server: `192.168.1.166`
- VM: `F:\Studio\LEAP_ROS\7.虚拟机镜像\LEAP_ROSV2.1\LEAP_ROSV2.1.vmx`
- VM state: running
- Guest OS: Ubuntu, kernel `6.8.0-136-generic`
- Guest IP: `192.168.253.128/24`
- NAT gateway: `192.168.253.2`
- Guest SSH: `192.168.253.128:22`, reachable from the server

The supplied account `chuxue` does not exist in the guest. The existing
desktop user is `chuyun`; the camera Agent is running under `chuyun`.

NetworkManager currently fails because `/lib/x86_64-linux-gnu/libssh.so.4`
has an invalid ELF header. The network was brought up manually with DHCP on
`ens33`, so networking is working for the current boot but is not reboot-safe.

## micro-ROS Agent

- Binary: `/home/chuyun/microros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent`
- Runtime user: `chuyun`
- Transport: `udp4`
- Port: `8888`
- Process IDs at verification: 2053 (launcher), 2055 (Agent)
- Listen state: `0.0.0.0:8888/udp`
- Log: `/home/chuyun/microros_ws/log/micro_ros_agent.log`
- Agent log: startup succeeded; no micro-ROS client session has been observed

The Agent is launched with:

```text
LD_LIBRARY_PATH=/home/chuyun/microros_ws/install/micro_ros_agent/lib:/home/chuyun/microros_ws/install/micro_ros_msgs/lib:/opt/ros/humble/lib
/home/chuyun/microros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent udp4 --port 8888 -v6
```

The extra `micro_ros_msgs/lib` path is required for
`libmicro_ros_msgs__rosidl_typesupport_cpp.so`.

## Host Port Forwarding

- `192.168.1.166:8888/udp` -> `192.168.253.128:8888`
- `192.168.1.166:8889/udp` -> `192.168.253.128:8889`
- VMware NAT listens on both UDP ports
- Windows inbound firewall rule: `Codex micro-ROS Agent UDP`
- NAT configuration backup:
  `C:\ProgramData\VMware\vmnetnat.conf.bak-codex-20261022-145627`

## Camera

- Camera: Maturo CAM `1CDBD49799F4`
- Wi-Fi: `大肥鱼连这个`
- Mode: STA
- Camera LAN IP: `192.168.1.189`
- Configuration page: `http://192.168.1.189/`
- Video stream: `http://192.168.1.189:81/`
- Stream type: `multipart/x-mixed-replace; boundary=frame`
- Frame capture: `D:\11\Projects\xuegeros\tmp\camera-frame.jpg`

The MJPEG stream was fetched successfully and a valid JPEG frame was
extracted. The frame shows the live camera scene, so video visibility is
verified.

## Local ROS Source And Quick Commands

- Local source directory: `D:\11\Ayxi\ros`
- Quick command notes: `D:\11\Ayxi\ros\README.md` and the VM Desktop copy of
  `demo.txt`
- Vehicle-side file index: `D:\11\Ayxi\ros\xuegecar_车端文件索引.md`
- The camera publisher config must be changed from its old hard-coded address
  to `http://192.168.1.189:81/`.

Recorded vehicle command sequence:

```text
ros2 launch micro_ros_agent micro_ros_agent_launch.py
ros2 launch xuegecar_bringup xuegecar_bringup.launch.py
ros2 launch xuegecar_camera http_video_publisher.launch.py
ros2 launch xuegecar_navigation2 slam_nav2.launch.py
ros2 launch slam_gmapping slam_gmapping.launch.py
rqt_service_caller
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.3}}"
```

The vehicle-side notes state that the ESP32 lower controller publishes `/odom`,
`/imu`, `/scan`, `/battery_state`, and `/ultrasonic`, and subscribes to
`/cmd_vel` through micro-ROS UDP `8888`.

The quick command set is not permission to move the car. Before publishing
`/cmd_vel`, confirm the micro-ROS client session, lift the drive wheels, use a
small speed command, and make sure no Nav2 or GUI publisher is also writing
`/cmd_vel`.

## Robot Control Status

Video visibility: verified.

Robot control: not verified and not yet available.

Reasons:

- The micro-ROS Agent is listening, but its log shows no client session.
- No robot micro-ROS client handshake has been observed.
- ROS topics `/cmd_vel`, `/odom`, and `/scan` have not been verified against a
  live robot connection.
- Port forwarding alone does not establish the robot-side micro-ROS session.

Do not report the car as controllable until the micro-ROS client connects and
the expected ROS topics are present.

## Remaining Work

- Repair the guest `libssh.so.4` package so NetworkManager starts normally.
- Make the guest DHCP configuration and micro-ROS Agent startup persistent.
- Connect the robot-side micro-ROS UDP client to `192.168.1.166:8888`.
- Verify ROS topics and publish a controlled `/cmd_vel` test only after the
  robot is confirmed connected and safely supported.

## Tooling Validation

Created a reusable command-line tool and runbook:

- `README.md`: complete operating guide and safety checklist
- `tools/xuegeros.ps1`: VM, guest, Agent, camera, mapping, and safety actions
- `config/xuegeros.env.example.ps1`: environment template without committed
  credentials

Validated actions:

- `help`: passed
- `snapshot`: passed, produced a valid JPEG from the live MJPEG stream
- `status`: passed after fixing PowerShell 5.1 JSON/path handling
- `start-vm`: idempotent when the VM is already running
- `check-forwarding`: passed, host UDP `8888/8889` are listening
- `start-camera`: passed, `http_video_publisher` connects to
  `http://192.168.1.189:81/`
- `mapping-status`: passed

Current validated ROS graph after starting the camera:

```text
/camera/image_raw/compressed
/parameter_events
/rosout
```

Current mapping blockers:

```text
/scan: not published
/map: not published
/tf: not published
```

No `/cmd_vel` motion command was sent during tool validation.

## Integration Boundary Repair

The implementation now matches the documented `xuegeros-car-01` Body adapter
boundary.

### Credential handling

- Removed server and guest password parameters from the PowerShell CLI.
- Local server access uses `id_loopros`.
- Guest access uses a local OpenSSH ProxyCommand and `dsh_vm_key`.
- A temporary server-side guest key was used only to prove the path, then
  removed after the local ProxyCommand path passed validation.
- No password is required in process arguments or environment variables.

### Read-only Body Adapter

Added `tools/xuegeros_body_adapter.py`.

Endpoints:

```text
GET /health
GET /v1/status
GET /v1/observations/latest
GET /v1/observations/camera/<short-lived-token>.jpg
GET /v1/mapping
GET /v1/navigation
```

Verified:

- `/health`: HTTP 200
- `/v1/status`: HTTP 200, bounded status, `autonomous_motion_allowed=false`
- `/v1/mapping`: HTTP 200, mapping unhealthy
- `/v1/navigation`: HTTP 200, blocked by missing session/scan/map/TF
- `POST /v1/status`: HTTP 405, read-only boundary enforced

The camera reference returned HTTP 503 during the final smoke test because the
camera MJPEG endpoint stopped producing data. The adapter correctly failed
closed instead of exposing stale media.

### Motion Gateway

Added `tools/xuegeros_motion_gateway.py`.

- CLI no longer contains `publish-cmd-vel` or `stop-motion`.
- Motion Gateway is disabled by default.
- Requires `XUEGEROS_MOTION_ENABLE=I_UNDERSTAND_PHYSICAL_RISK`.
- Requires an operator token in `X-Operator-Token`.
- Defaults to shadow mode.
- Has no physical command backend.
- Enforces one active lease.
- Lease maximum is 1000 ms.
- Watchdog timeout is 200 ms.
- Velocity limits are 0.1 m/s and 0.5 rad/s.

Shadow-mode smoke test:

- lease issued
- command accepted with `published=false`
- lease automatically cleared after the 200 ms stale-command timeout
- `safe_stop_events` incremented
- unauthenticated stop returned HTTP 403

### ROS-side safety

Added `ros2_safety/xuegeros_safety`:

- `twist_mux` priority configuration
- `cmd_vel_watchdog`
- 200 ms zero-velocity fallback
- velocity clamping
- `/motion/lease` parsing
- `~/disarm` service

The package is prepared but not deployed or started.

### Current robot state

- micro-ROS Agent is listening.
- The USB Wi-Fi adapter is passed through to the VM.
- `Maturo_E072A1B4F368` was provisioned to `大肥鱼连这个`.
- Its current wired-LAN address is `192.168.1.106`.
- Its runtime was changed from `192.168.2.229:8888` to
  `192.168.1.166:8888`.
- The robot-side micro-ROS client session is established.
- `/odom`, `/imu`, `/scan`, `/battery_state`, and `/ultrasonic` are live.
- `/scan` has one publisher and runs at approximately 9.8 Hz.
- `xuegecar_bringup` is running and `/tf` is live.
- Gmapping is running, `/map` is present, and map width is 384.
- `/tf` has three publishers.
- Autonomous motion remains forbidden.
- A physical motion pulse was published through the new safety path during the
  movement test.

## UDP Relay And Motion Test

The VMware NAT UDP mapping for port 8888 was unreliable for bidirectional
micro-ROS traffic. It was replaced with a server-hosted relay:

```text
192.168.1.166:8888 -> 192.168.253.128:8888
```

Deployment:

- Script: `C:\Users\zhzaill\xuegeros_udp_proxy.ps1`
- Log: `C:\Users\zhzaill\xuegeros-udp-proxy.log`
- Scheduled task: `XuegerosUdpProxy`
- Runner: `SYSTEM`
- UDP 8888 VMware NAT rule removed
- UDP 8889 NAT rule retained

After restarting the Agent and device, the complete endpoint graph became
stable:

- `/cmd_vel`: one publisher, one robot subscription
- `/odom`: one robot publisher
- `/scan`: one robot publisher

Safety stack:

- `twist_mux` 4.3 extracted to a user-owned prefix
- `/cmd_vel_autonomy -> /cmd_vel_safe_raw -> /cmd_vel`
- 200 ms watchdog active
- short lease accepted and disarmed

Movement test results:

- First pulse: `0.03 m/s`, `0.5 s`
- Stronger pulse: `0.1 m/s`, `0.8 s`
- `/cmd_vel_autonomy`: nonzero messages observed
- `/cmd_vel_safe_raw`: nonzero messages observed
- `/cmd_vel`: nonzero messages observed
- ESP32 `target_vx`: reached `50 mm/s` and `100 mm/s`
- ESP32 `source`: `6` (`MOTION_SRC_MICROROS`)
- Wheel feedback: remained `0.00 / 0.00 mm/s`
- `/odom`: x/y/yaw remained unchanged
- Battery telemetry during test: approximately `3.22-3.27 V`, `0%`

Conclusion:

The ROS2, relay, micro-ROS, mux, watchdog, and ESP32 command-ingress path are
working. Physical movement is blocked at the motor power/actuator layer, most
likely because the 2S motor battery is disconnected, switched off, or depleted.
Do not repeat motor pulses until battery power is restored and the wheels are
lifted.

### Battery-restored movement test

The motor battery was connected after the initial blocked test.

- Battery voltage rose to about `6.947-7.149 V`
- Battery estimate rose to `39-48%`
- Safety path remained ready for every pulse

Measured movement:

- Forward `0.05 m/s` for `0.5 s`: odom x increased from approximately `0` to
  `0.0388 m`
- Backward `-0.04 m/s` for `0.5 s`: odom x decreased by approximately
  `0.00264 m`
- Left turn `0.2 rad/s` for `0.6 s`: yaw changed by approximately `+0.0361 rad`
- Right arc `0.04 m/s`, `-0.1 rad/s` for `0.5 s`: x increased approximately
  `0.0112 m` and yaw changed approximately `-0.0186 rad`
- Final target velocity and wheel velocity returned to zero
- Watchdog disarm succeeded

Residual issue:

- A pure reverse angular command (`0 rad/s linear`, `-0.2 rad/s angular`) was
  observed on `/cmd_vel_autonomy`, but did not appear on `/cmd_vel_safe_raw`
  in two attempts.
- Rightward rotation did work when combined with a small forward command.
- Do not expose turn-only autonomy until this intermittent mux path is fixed and
  repeated.

## 2026-10-23 Connectivity Recovery

The VM stayed powered on with VMware Tools healthy, but `ens33` had no address
after boot. Running `dhclient -v ens33` restored `192.168.253.128/24` and the
default route via `192.168.253.2`; guest SSH then became reachable.

The healthy ROS state recorded on 2026-10-22 was real but depended on a
manually running Windows UDP relay. The `XuegerosUdpProxy` scheduled task was
later disabled. Its previous default wildcard bind also failed under the
installed PowerShell/.NET runtime because the script constructed an IPv4
endpoint from `0.0.0.0` and resolved it as IPv6. The relay script now uses the
`UdpClient(port)` constructor for this wildcard case. The scheduled task was
re-enabled and started; robot endpoint `192.168.1.106:8888` was observed, and
the Agent exchanged micro-ROS traffic.

The current `Codex micro-ROS Agent UDP` firewall rule is enabled with action
`Allow`; the earlier report of `Block` was stale. The ROS graph recovered with
`/battery_state`, `/cmd_vel`, `/imu`, `/odom`, `/scan`, and `/ultrasonic`.
`xuegecar_bringup` and Gmapping were started; `/map`, `/tf`, and
`/slam_gmapping` are present. A follow-up observed `/scan` at about 2.9 Hz,
`/map` width 384, and three `/tf` publishers. ROS reports two
`/leap_low_driver` nodes/publishers, which needs investigation; the scan rate
is below the approximately 9.8 Hz recorded on 2026-10-22. Nav2 was not started
and no motion command was published.

Remaining reboot-safety work: guest DHCP still relies on manual `dhclient`
because NetworkManager is broken by the invalid `libssh.so.4`; Agent, bringup,
and Gmapping were started manually and are not persistent guest services.
Verify relay startup on the next planned host reboot.

## 2026-10-23 DSH Cockpit And Motion Bridge

A DeepSeek Harness bundle (`dsh-xuegeros`) now exposes the car in the session:
a right-sidebar cockpit with the live camera, YOLO boxes, telemetry and bounded
jog, plus model tools `xuegeros_observe`, `xuegeros_drive`, `xuegeros_stop`.

Host half:

- polls the read-only adapter at `127.0.0.1:18110/v1/world`;
- serves `/sync/xuegeros/{state,frame,move,stop,ai,summon,gateway/start}`;
- injects a throttled world brief with `agent.inject()` and can summon the
  current session with `agent.followup()`;
- starts the motion gateway on request and keeps an SSH tunnel
  `localhost:18200 -> guest:18200` only while live mode is enabled.

Guest side:

- `tools/xuegeros_cmd_bridge.py` (HTTP, 127.0.0.1:18200, bridge token) publishes
  `/motion/lease` and 20 Hz `/cmd_vel_autonomy`, then zero velocity; hard limits
  0.1 m/s, 0.5 rad/s, 100-1000 ms.
- `xuegeros_safety` was started with the user-owned `twist_mux` prefix
  (`/home/chuyun/ros_local/twist_mux/opt/ros/humble` added to
  `AMENT_PREFIX_PATH`, `LD_LIBRARY_PATH`, `PYTHONPATH`). Without that prefix the
  launch fails with `package 'twist_mux' not found`.

Motion gateway:

- now supports `XUEGEROS_MOTION_BACKEND_URL` and `XUEGEROS_BRIDGE_TOKEN`;
  non-finite velocities are rejected, `duration_ms` is validated, and shadow
  mode still publishes nothing.
- verified end to end with a zero-velocity pulse: gateway `published=true`,
  bridge `published_messages=4`, watchdog logged `motion lease accepted`.

Camera outage:

- MJPEG stream and the adapter lost the camera between about 16:12 and 17:18.
  The camera stayed in STA mode with the same address `192.168.1.189`; its
  config page still advertised `http://192.168.1.189:81/`. After the camera
  restarted, the adapter reconnected on its own. The long-running adapter pull
  (about 3.5 hours before the drop) remains the main suspicion for the device
  dropping its Wi-Fi session.

Verification:

- `xuegeros_observe` returned live detections, healthy `scan/map/tf` and an
  online micro-ROS session.
- The client bundle self-check registers one `sidebar.right.pane.tab` entry and
  one guide entry.
- The client bundle is not visible until DSH is restarted (Harness discovers
  client bundles at host startup).

Camera port is exclusive:

- `http://192.168.1.189:81/` accepts only one video session. The adapter holds
  it for YOLO, so any second stream (browser tab) hangs or resets; the camera's
  port 80 config page is also reset while the video session is active.
- Verification: after stopping the adapter, the config page returned 200 and the
  MJPEG stream yielded 2,998,378 bytes in four seconds; the adapter needed about
  30 seconds after restart before the camera accepted it again.
- Fix: the adapter exposes a loopback-only preview `GET /v1/preview.mjpeg`
  (`XUEGEROS_PREVIEW_ENABLE=1`, enabled by `xuegeros_body_adapter.cmd`) that
  fans out `LATEST_FRAME` without consuming the YOLO slot and without writing to
  disk. The DSH frame route prefers it and only falls back to the camera URL.

## 2026-10-23 Evening Session: Link Recovery, Real Bug Fixes, Physical Motion

### Robot link was dead; root cause was the relay task's power settings

On arrival the whole guest ROS graph was silently degraded: the micro-ROS agent
was alive but its log had stopped growing, `/scan`, `/odom` had no live data,
and the DSH adapter reported `status_available=false`.

Root cause (proven, not guessed): the server-side UDP relay
(`XuegerosUdpProxy` on DESKTOP-6PVSJFV) had exited and was refusing to start,
because the scheduled task carried the default laptop power policy:

```text
DisallowStartIfOnBatteries : true
StopIfGoingOnBatteries     : true
```

The relay log's last line was `relay listening ...` with no subsequent stats,
and `netstat -an` showed nothing bound to UDP 8888 anywhere on the server.

Fix (server `DESKTOP-6PVSJFV`): both battery settings set to `false`,
`ExecutionTimeLimit=PT0S`, `RestartCount=999` / `RestartInterval=PT1M`.
After restart the robot session re-established and topics went live again.

### The adapter silently lost status and mapping

`tools/xuegeros_body_adapter.py` shelled out to `pwsh`, which does not exist on
this host (Windows PowerShell 5.1 only). Every status/mapping probe therefore
failed with `OSError`, and the code degraded that to `null`, so the UI and the
model tools reported healthy hardware as unavailable — with no error surfaced.

Fix: `resolve_shell()` prefers `pwsh` but falls back to the `powershell` that
exists (`XUEGEROS_SHELL` overrides), and the world projection now carries
`shell` plus `status_error` so a null probe explains itself.

### Battery was permanently null

`/battery_state` is published with **BEST_EFFORT** QoS, so a default RELIABLE
subscriber receives nothing (the same trap as `/scan`, and the reason
`ros2 topic echo`/`hz` returned empty for robot topics). The plugin's
`batteryOf()` reads `status.battery`, which the adapter never populated.

Fix: new `battery` action in `tools/xuegeros.ps1` subscribing with matching
BEST_EFFORT QoS, surfaced through `normalize_status` as `status.battery`.
Measured 8.136-8.183 V, 89-91%.

### Stale "physical mode" claim across restarts

Persisted `state.json` held `liveMotion: true` and `gatewayStartedHere: true`
from a gateway process that died with the previous DSH session. The cockpit
would render "physical mode on" while the reachable gateway was shadow-only.

Fix: on startup the plugin clears `liveMotion` unless a live gateway with
`shadow_mode === false` is actually reachable (fail closed). The stale file was
also corrected by hand; a backup is at `state.json.bak-20261023-live-reset`.

### Guest services are now reboot-persistent

Added `ros2_safety/install_guest_systemd.sh`, which installs and enables five
units: `xuegeros-micro-ros-agent`, `xuegeros-bringup`, `xuegeros-cmd-bridge`,
`xuegeros-safety`, `xuegeros-gmapping`. All run as `chuyun`, all enabled in
`multi-user.target.wants`, all restart on failure. Verified: all five
`enabled`+`active`, and a `systemctl restart xuegeros-cmd-bridge` brings the
bridge back with `/health` answering.

### Camera reconnect backoff

The capture loop retried every fixed 1.0 s forever, hammering a camera that
needs ~30 s to release its previous session. Now exponential backoff
1s -> 30s cap, reset when a connection was healthy for >=30 s.

### Physical motion verified for the first time

Operator confirmed wheels unblocked, battery connected, no other `/cmd_vel`
publisher. Path: live gateway -> SSH tunnel 18200 -> guest bridge -> twist_mux
-> watchdog -> `/cmd_vel`.

Single pulse, `0.05 m/s` for `300 ms`:

- gateway: `published: true`, `shadow_mode: false`
- odom x: `-0.00323` -> `+0.01599` (about **1.9 cm forward**)
- yaw: `0.07421` -> `0.11430`
- `/cmd_vel` returned to exactly zero; watchdog logged
  `motion lease accepted` then `zero velocity active: watchdog` /
  `zero velocity active: lease`
- battery 8.17 V

The system was then restored to the safe default: gateway back in shadow mode,
physical backend removed, tunnel closed.

### Gateway boundary re-verified

`velocity_not_finite` (NaN and Infinity), `velocity_limit` (linear 5.0),
`invalid_duration` (50 ms), `lease_not_found`, and HTTP 403 without the
operator token all behave as designed.

## 2026-10-23 Late Session: The Relay Suicide Bug (root cause of "the car broke after restart")

### Symptom

After the DSH restart the car appeared dead: nothing bound to UDP 8888 on the
server, no adapter, no gateway, no tunnel on the host. The guest ROS graph,
however, was completely healthy the whole time (all five `xuegeros-*` systemd
units `active running`). Mapping was live and correct, contrary to the working
assumption that it had never worked.

### Root cause (proven, not guessed)

`xuegeros_udp_proxy.ps1` treated one transient socket error as fatal.

A UDP socket learns its peer is gone only asynchronously: when the ESP32
reboots or drops Wi-Fi, the OS delivers an ICMP port-unreachable and **.NET
surfaces it on a later `Receive()` call** as a `SocketException`. The script's
`$ErrorActionPreference = 'Stop'` plus a `Receive()` inside a single
try/catch meant that exception unwound to the outer `finally`, disposed both
sockets, and **exited the process**. Nothing rebound the port.

Server evidence (relay ran fine for ~11 minutes, then died and stayed dead):

```text
2026-10-23T22:45:32  stats client=192.168.1.106:8888 upstream=29554 client=37769
2026-10-23T22:46:32  stats client=192.168.1.106:8888 upstream=32573 client=41534
2026-10-23T22:46:52  FATAL ... Receive ... SocketException（远程主机强迫关闭了一个现有的连接）
```

`LastTaskResult=1`, and `netstat` then showed **no UDP 8888 listener at all**.
The guest VM and its ROS stack were never the problem.

Reproduced deliberately by running the old script in the foreground: it served
traffic, then threw the same `System.Management.Automation.MethodInvocationException`
wrapping `System.Net.Sockets.SocketException` from `Receive`, and terminated.

### Second, independent defect: the log everyone was reading was the wrong file

The scheduled task runs as `SYSTEM` (`LogonType=ServiceAccount`,
`RunLevel=Highest`), so `$HOME` resolves to
`C:\Windows\System32\config\systemprofile\`. The default `LogPath` is
`"$HOME\xuegeros-udp-proxy.log"`, so the relay has always written to

```text
C:\Windows\System32\config\systemprofile\xuegeros-udp-proxy.log
```

while every operator and prior investigation read

```text
C:\Users\zhzaill\xuegeros-udp-proxy.log
```

which only received lines when the script was run **interactively** as
`zhzaill`. This is why the 2026-10-23 morning session saw "last line =
`relay listening`, then nothing" and why the relay's own death appeared
silent: the task's real log (50 KB) showed the FATAL line plainly.

### Fix

`tools/xuegeros_udp_proxy.ps1` rewritten (deployed to the server):

- a reconnect loop around the whole socket lifecycle; `SocketException` on
  `Receive()` no longer terminates the process — it resets the elected client,
  waits 200 ms, and continues;
- outer retry with capped exponential backoff (1 s -> 15 s) for bind failures;
- error lines are rate-limited to one per 5 s so a fast failure loop cannot
  flood the log;
- logging itself is wrapped so a log failure can never take the relay down;
- explicit `relay starting:` / `relay listening on:` lines, and the existing
  `stats` heartbeat retained to prove liveness.

### Verification (live)

```text
22:49:19  relay starting: 0.0.0.0:8888 -> 192.168.253.128:8888
22:49:19  relay listening on 0.0.0.0:8888 -> 192.168.253.128:8888
22:49:19  client selected: 192.168.1.106:8888
22:49:49  stats client=192.168.1.106:8888 upstream=1544 client=1865
22:50:49  stats client=192.168.1.106:8888 upstream=4471 client=5583
```

Guest link re-verified through the repaired relay: `/odom` live, `/scan` at
about 0.9-1.0 Hz, `/map` at about 0.44 Hz, `map -> odom` TF resolving.

### Physical motion re-verified end to end

Full path live gateway -> SSH tunnel 18200 -> guest bridge -> twist_mux ->
watchdog -> `/cmd_vel`. Single pulse `0.05 m/s` for `300 ms`:

- gateway `published=true`, `shadow_mode=false`
- odom x: `0.0` -> `0.0105156` (about **1.05 cm forward**), wheels lifted
- `/cmd_vel` returned to exactly zero afterwards
- battery 9.949 V

System restored to safe defaults: gateway stopped, tunnel closed, `/cmd_vel`
zero.

### Correction to the record

Mapping is **not** unfinished. It is running and correct: `/map` publishing at
about 0.44-0.52 Hz, 384x384 at 0.05 m/px, `mapping.healthy=true` with
`scan/map/tf` all present, and `map -> odom` TF resolving at
`translation [0.652, -0.080, 0]`, yaw -14.09 deg. The earlier assumption that
mapping had never worked was wrong; the thing that was broken was the UDP
relay underneath everything.

### New finding, unresolved: /scan is far below its documented rate

`/scan` now reports about **0.9-1.0 Hz**, against the **9.8 Hz** recorded on
2026-10-22 and the "about 2.9 Hz" noted on 2026-10-23 morning. Robust
pathological cause is not yet established; the earlier session also flagged two
`/leap_low_driver` nodes/publishers as needing investigation. This is the next
thing to chase, because Gmapping quality and any future Nav2 depend on it.

### RESOLVED: the duplicate /scan publishers were stale agent sessions

The long-standing "two `/leap_low_driver` nodes/publishers" oddity is now
explained, and it was never a duplicate process. Only one bringup process
exists (PID 86714, a single clean systemd cgroup). The duplication was inside
the micro-ROS session: `/scan` had **3 publishers all named `leap_low_driver`**
whose DDS **GIDs shared the identical client key prefix**
`01.0f.17.54.b8.52.6a.ae`, differing only in a later byte. That is one micro-ROS
client session registering the same endpoint three times - stale sessions left
behind while the relay was repeatedly dying underneath it.

Restarting the agent cleared them immediately:

| | before | after |
| --- | --- | --- |
| `/scan` publishers | 3 | 1 |
| `leap_low_driver` nodes | 3 | 1 |

Note this was *not* the cause of the low rate (see below), so it is a separate
correctness win: three duplicate RELIABLE publishers of the same sensor stream
is exactly the kind of thing that silently ruins timing later.

### Open: all robot topics are slow, and the relay is not the bottleneck

After the duplicate endpoints were cleared, rates stayed low:

- `/scan` about 0.75 Hz (documented 9.8 Hz)
- `/odom` about 4.7 Hz
- `/imu` about 4.3 Hz

The relay is demonstrably healthy at the same time: roughly **1440-1485 packets
per 30 s (about 48 pkt/s)**, steady, with no packet loss in the counters, and it
self-healed through two `ConnectionReset` events at 23:03:41 and 23:03:59. The
guest VM is small (2 CPUs, load average about 2.2), which may contribute, but
packets are flowing fine end to end. The remaining suspicion is the ESP32 side
publishing these topics slowly, which needs physical access to that device to
confirm. Do not assume the low rate is a host or VM problem.

### The relay fix is proven in production, not just by inspection

Same log, same failure signature, two different outcomes:

```text
16:13:25  FATAL ... SocketException      <- old script, relay dead
20:57:02  FATAL ... SocketException      <- old script, relay dead
21:28:47  FATAL ... SocketException      <- old script, relay dead
23:03:41  socket error (ConnectionReset); rebinding   <- new script, survived
23:03:59  socket error (ConnectionReset); rebinding   <- new script, survived
```

Three fatal terminations under the old code, then two identical events handled
and recovered under the new code.

### Plugin: two real defects found and fixed in the DSH cockpit bundle

Both were found while building `dsh-xuegeros/tools/selftest.mjs` (a mock-cordis
harness in the style of the 2026-10-16 DSH-TTS one). It now runs 11 checks
green and repeatable.

1. **Late-registration leak.** The host can dispose the plugin before
   `stateReady` resolves, because state is read from disk first while route and
   timer registration live in that continuation. Teardown would splice an empty
   `disposers` array, and then the continuation would mount 7 route handlers and
   a poll timer that were never unmounted - they leak on the shared web server
   on every plugin reload. Fixed with a `disposed` flag plus a `trackDisposer`
   helper that disposes immediately if teardown already ran, and an early return
   in the `stateReady` continuation. Regression test added.

2. **`agent.followup()` is session-corrupting and is now proven banned.** On
   this DSH build `followup` appends a `user/message` event without `id`/`role`;
   the session loader then rejects the whole store on restart and the session
   disappears (two sessions were lost this way). Summon now uses
   `agent.inject()` only, and the self-test asserts the inject happened *and*
   that a `followup` that throws is never reached.

Three flaws that had been making the self-test tell lies were also fixed, since
a test that reports success while doing nothing is worse than no test:

- `mockReq` only emitted its body when the test called `.settle()`. Handlers
  `await readBody(req)`, which never resolved; the event loop drained and
  **node exited 0 with the remaining assertions unrun**. A silent false pass.
- the mock's `ctx.effect(cb)` stored the callback without invoking it, so
  teardown disposed nothing and every disposal assertion was meaningless.
- `agents.list()` returned a fresh object per call while the plugin re-lists on
  every pick, so recorded injects landed on a throwaway object.

The suite now also carries a 60 s watchdog and an exact check count, so a hang
or a skipped assertion fails loudly instead of passing quietly.

### Still outstanding

- The relay now survives transient socket errors, but has **not** been tested
  across a real host reboot or a real ESP32 power cycle.
- `libssh.so.4` on the guest is still corrupt (`invalid ELF header`), which
  breaks `curl` and NetworkManager there. Guest networking and all probe
  scripts work around it; the package repair remains undone. Note this also
  means "use curl to check the bridge" fails for reasons unrelated to the
  bridge - use python3 `urllib` instead.
- `/scan` rate regression above.
- The relay's real log path should be pinned explicitly (for example via
  `XUEGEROS_UDP_PROXY_LOG`) rather than depending on `$HOME` resolving
  differently for SYSTEM, so operators stop reading the wrong file.

## 2026-10-23 Final Session: "stop failed: HTTP 403 operator_authorization_required"

### Symptom

The cockpit's stop button returned
`HTTP 403: {"error":"operator_authorization_required"}`. Critically, the car was
**not** left moving - the motion pulses had already returned to zero and
`/cmd_vel` was zero throughout - but the operator could not command a stop,
which is the one control that must never fail.

### Root cause: a gateway that was not ours

The plugin always sends `X-Operator-Token: state.operatorToken` (its own
auto-generated token, `lib/index.js` `gatewayHeaders()`), and it only passes
that token to a gateway **it spawns itself** (`startGateway()` sets
`XUEGEROS_OPERATOR_TOKEN: state.operatorToken` in the child env).

Earlier in this same day, for an end-to-end motion test, the gateway had been
started **by hand** with `XUEGEROS_OPERATOR_TOKEN` set to the *bridge* token
instead. So:

| | token |
| --- | --- |
| gateway enforcing | `yCD9Ie5m…` (bridge token; full value lives only in config/xuegeros.env.ps1, redacted here before publishing) |
| cockpit sending | `swYbdqvz…` (plugin `state.operatorToken`, example prefix only) |

Proven directly: stop with the bridge token returned
`{"ok":true,"published":true}`, while stop with the cockpit token returned 403.
This was self-inflicted by the earlier manual gateway start, not a pre-existing
defect.

### The real defect this exposed (now fixed)

`startGateway()` began with:

```js
const existing = await refreshGateway(true);
if (existing) return { ok: true, already: true };
```

`/v1/status` is **public by design** and carries no identity, so *any* gateway on
port 18111 - including one enforcing a foreign token - looked healthy and was
reported as "already started". The plugin then never corrected the mismatch, and
every subsequent stop or move failed 403 with **no recovery path from the UI**.
That is a genuine availability defect: the cockpit could not stop the car and
could not tell the operator why.

Fix in `lib/index.js`:

- new `gatewayAcceptsOurToken()` probes authorization with the most harmless
  authorized action - a stop with zero motion. Stopping is idempotent and is the
  direction we want to fail in: if we cannot authorize a stop, the cockpit must
  know rather than believe it can halt the car.
- a 403 is treated as the token verdict; any other failure (timeout, 5xx) is
  **not** claimed as a token mismatch.
- `startGateway()` now returns
  `{ ok: false, error: 'gateway_token_mismatch', detail: ... }` instead of a
  silent `already: true`, and warns in the log.

### Verification

- Reproduced the exact production condition: started a gateway with a
  deliberately foreign token. It answered `/v1/status` 200 (the trap) but
  rejected stop with 403.
- Confirmed the detection logic against that live foreign gateway:
  `status 200` / `stop with our token 403` / `stop with foreign token 200`, so
  the 403 is sufficient evidence of "not ours".
- Regression test added to `tools/selftest.mjs`: a stub server that answers
  `/v1/status` but 403s everything else, asserting `gateway/start` now returns
  `error: gateway_token_mismatch`.
- Suite is now **12 checks, all green, repeatable**.
- After restarting the gateway *through the cockpit route* (so the tokens match),
  stop succeeded via both paths with `published: true`:
  - `POST /v1/stop` -> `{"ok":true,"published":true,"shadow_mode":false}`
  - `POST /sync/xuegeros/stop` -> same result wrapped by the plugin
- Guest `/cmd_vel` confirmed exactly zero afterwards.

Environment restored to safe defaults: `ai_motion=false`, `live_motion=false`,
gateway plugin-owned, `/cmd_vel` zero.

### Lesson worth keeping

A hand-started gateway bypasses the plugin's token wiring and silently breaks
the cockpit's authority. **Always start the gateway through
`/sync/xuegeros/gateway/start`**, never by hand, when the cockpit must be able to
drive. The new mismatch error now makes that mistake loud instead of invisible.



## 2026-10-24 NoMaD VLA Service Deployment (Server)

Deployed the NoMaD inference service on DESKTOP-6PVSJFV per
`docs/VLA-INTEGRATION-PLAN.md`. Everything lives in a new `E:\vla\` tree;
the ComfyUI venv is used read-only and its site-packages were not modified
(dependencies installed with `--no-deps --target E:\vla\pydeps`).

- Service: `E:\vla\nomad\vla_server.py`, port 18300, endpoints
  `GET /health`, `POST /goal`, `POST /infer`, `POST /reset`.
- Control: `E:\vla\vla-ctl.ps1 start|stop|status|log`. Started via
  `Win32_Process.Create` — a plain `Start-Process` over SSH dies with the
  plink session (verified: the service vanished right after its first
  health check when started that way).
- Vendored code (MIT): `vint_train` models + a minimal `diffusion_policy`
  subset, fetched from GitHub raw; archived in this repo under
  `tools/vla_vendor/`.
- Dependencies: efficientnet_pytorch, einops, diffusers 0.34.0,
  huggingface_hub 0.36.2 (both downgraded: hub 2.x needs httpx, diffusers
  0.42 needs hub 2.x).
- Verified on server and cross-host from 192.168.1.178: `/health` 200;
  `/goal` + 12x `/infer` smoke test with synthetic JPEGs; velocity
  proposals clamp to v<=0.08 / |w|<=0.4; p50 latency 164 ms (first call
  ~4 s CUDA warmup); VRAM 85 MB fp32 on the P104-100.
- Model facts: the released NoMaD is EfficientNet-b0 based, 3.85M params,
  goal-image conditioned (language conditioning was not released), 96x96
  input, 10-step DDPM, 8 waypoints + PD controller.
- Weights `nomad.pth` are NOT yet on the server: the only official source
  is Google Drive, unreachable from this network (probed 000). HF
  (incl. full-text search), GitHub forks/releases/issues, Zenodo, Gitee,
  OpenI and ModelScope were all checked on 2026-10-24 with no mirror
  found. The service runs random-init and flags `weights_loaded:false`
  in `/health` and every `/infer` response. Drop-in slot:
  `E:\vla\nomad\weights\nomad.pth`, then `vla-ctl.ps1 stop/start`.

## 2026-10-24: VLA (NoMaD) Update Verified, and a Relay Reconnect Defect Fixed

### VLA update accepted (v0.2.0 plugin + server-side NoMaD)

Server service http://192.168.1.166:18300 is live: `device=cuda`, fp32,
`nomad.pth` 76,473,631 B, sha256 `70f79b8...a333`, `missing_keys=0 /
unexpected_keys=0`, `vram_mb=84.8`, EfficientNet-b0 encoder at 96x96 with a
4-frame context. Measured latency 199-337 ms (about 3-5 Hz), a little above the
plan's p50 of 164 ms. Proposals are clamped inside the service to
`v=0.08 / |w|<=0.4`, confirmed on every sampled frame.

P1 shadow chain verified end to end with `tools/vla_shadow_probe.py`, which
pulls frames from the adapter loopback and never opens a second camera session.

### The safety invariant holds in runtime, not just on paper

Six consecutive VLA proposals were generated while `live_motion=true` AND
`ai_motion=true` - i.e. physical mode, able to drive the motors. Result:

- `/cmd_vel` exactly zero
- `/motion/lease` empty (no lease ever issued)
- odom `x=0.000022`, unchanged across the whole run

Code audit agrees: only three model tools exist (`xuegeros_observe`,
`xuegeros_drive`, `xuegeros_stop`) - there is no VLA tool at all - and
`shadowProposal` flows only into `vlaSummary()` for display. There is no code
path from a proposal into `drive()` or any gateway command.

### v0.2.0 fixes the "screen froze" problem left open on 2026-10-23

Last session established the backend was healthy and the fault was in the
browser, but stopped there. Now fixed in two places:

- `lib/client.js`: camera `<img>` freeze watchdog. An 8x8 canvas checksum runs
  against the live element; 10 s without a change while the adapter still says
  `streaming` triggers a cache-busted reconnect (8 s debounce). Tainted-canvas
  and decode races are swallowed rather than reported as freezes.
- `lib/index.js`: `proxyStream` idle guard. Every chunk re-arms a 20 s timer; a
  stalled upstream is aborted so the client watchdog can recover instead of the
  panel hanging forever. A healthy stream is untouched - measured 1,675,756 B
  over 72 chunks in 4 s with `200 multipart/x-mixed-replace`.

### Real defect found in my own previous relay fix (socket poisoning)

During testing the robot's DDS endpoints vanished: `/scan` and `/odom` both
reported **Publisher count: 0** and no `leap_low_driver` node, while the relay
process was alive, UDP 8888 remained bound, and its counters were still
climbing. The adapter kept reporting `micro_ros.session_observed=true`. A
half-open link: counters advance, nothing reaches the guest.

How it was narrowed down, layer by layer:

1. Relay alive and it **survived four `ConnectionReset` events** - the
   self-healing added on 2026-10-23 works; the old script would have died.
2. `ping` to the ESP32 and to the guest: 0% loss, 1-4 ms / <1 ms. Not the
   network.
3. Guest agent: `ss -ulnp` shows `0.0.0.0:8888` bound (pid 795), unit active.
4. Restarting the agent did not bring publishers back, so the fault had to be
   the relay-to-guest session rather than a missing agent.

Root cause: the previous version's socket-exception handler did
`$clientEndpoint = $null; continue`, **reusing the same socket**. It logged
"rebinding" while rebinding nothing. UDP failure is reported late: when a peer
disappears the ICMP port-unreachable surfaces on a *later* `Receive()`, and the
socket can stay poisoned - every later call fails immediately and nothing ever
reaches the peer again. So once the guest agent restarted (recreating its UDP
socket), the relay's upstream socket was poisoned and could never forward the
ESP32's packets again.

Fix: `break` on `SocketException` so the outer reconnect loop **recreates both
sockets**, plus a 200 ms pause before rebinding (lets the OS release UDP 8888
and keeps a flapping peer from turning reconnect into a hot loop). Log wording
changed to `recreating both sockets` so it stops claiming something it does not
do.

Verification (deployed to the server, repo copy byte-identical, 7240 B):

| | before | intermediate | after clean reset |
| --- | --- | --- | --- |
| `/scan` publishers | 0 | 2 | **1** |
| `/odom` publishers | 0 | 1 | **1** |
| `leap_low_driver` nodes | 0 | 2 | **1** |
| odom x | 0.000022 | - | 0.000022 |

The intermediate 2 is the same stale-session artifact documented on 2026-10-23
(identical DDS GID client prefix); one more agent restart cleared it to 1.

### Other acceptance results

- `tools/selftest.mjs` rewritten: **14/14 green**, including two new VLA
  assertions (state route carries health + shadow proposal; VLA disabled stays
  offline with an explicit reason).
- `node --check` passes for both `lib/index.js` and `lib/client.js`; the client
  registers `sidebar.right.pane.tab` and renders the VLA card.
- Gateway starts through the plugin route; the 403 stop-authorization fix from
  2026-10-23 still holds (stop with `state.json`'s `operatorToken` returns
  `ok:true`).
- Zero-velocity pulse through the full chain: `published:true`,
  `shadow_mode:false`; subsequent stop also `published:true`.

### Outstanding after this session

- The socket-recreate fix has **not** been tested across a real host reboot or a
  real ESP32 power cycle.
- ESP32 session stability is still not solved: both agent restarts in this
  session produced duplicate DDS endpoints, so "restart agent -> stale session"
  reproduces reliably and can only be cleared by restarting the agent again.
  Worth making observable (run the agent with `-v6` and surface session logs via
  the adapter) or detecting guest port recreation in the relay.
- `/scan` has not returned to the documented 9.8 Hz; `ros2 topic hz` timed out
  repeatedly this session while a python subscriber still received messages,
  i.e. the rate is very low. The link between flapping and low rate is not
  quantified yet.
- adapter-side VLA is still unimplemented (`/v1/frames/latest.jpg` -> 404,
  `/v1/world` has no `vla` section). This matches plan section 5 marking those
  as pending, so it is **not** a regression; the plugin talks to the VLA service
  directly.
- `config/xuegeros.env.ps1` still lacks `XUEGEROS_VLA_URL` / `XUEGEROS_VLA_ENABLE`
  (the hardcoded `DEFAULTS.vlaUrl` masks this).
- Guest `libssh.so.4` still corrupt; use `python3 urllib`, not `curl`.
- The guest appears to have rebooted twice today (14:10:58, 14:18:49); systemd
  persistence evidently works under a real reboot but has no formal acceptance
  record yet.

### 收尾补充观察（15:58–16:00）：relay 已自愈，但链路仍在抖

部署修复后整链路仍在抖动，`ConnectionReset` 约每 5 秒一次，但 relay 每次都能恢复：

```text
15:58:22  client selected: 192.168.1.106:8888
15:58:27  socket error (ConnectionReset); recreating both sockets
15:58:27  relay listening on 0.0.0.0:8888        <- 约 230 ms 后重回服务
15:58:27  client selected: 192.168.1.106:8888    <- ESP32 立刻重新选中
```

对比老版本：同样的事件会让进程退出（10-23）或让 socket 永久毒化（本会话修复前）。
**修复的价值是「不再死/不再哑」，不是「消除抖动」。**

抖动本身仍在损害上层：收尾复查时 `/scan` publisher = 1 而 `/odom` publisher = 0，
`leap_low_driver` 节点 = 1，`/cmd_vel` 为零（安全）。即 ESP32 侧会话在反复重建，
DDS 端点随之闪断。这是**基础设施问题，需接触 ESP32 侧才能根治**，本机只能观测。
