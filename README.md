# XuegerOS Car 01 Body Adapter

XuegerOS is the physical robot runtime for `xuegeros-car-01`. It is not a Mind,
not a second Hub, and not a generic terminal exposed to Persona.

Current boundary:

```text
camera + robot observations
             |
             v
   read-only Body Adapter
             |
             v
        Absolute Solver
      Perception / World

motion request
             |
             v
 human-only Motion Gateway
             |
             v
  twist_mux + cmd_vel watchdog
             |
             v
        physical robot
```

The PowerShell maintenance CLI cannot publish `/cmd_vel`.

## Current State

| Capability | State |
| --- | --- |
| VM lifecycle | Verified |
| Guest network | Verified for the current boot |
| Camera MJPEG | Verified |
| Camera ROS topic `/camera/image_raw/compressed` | Verified |
| micro-ROS Agent UDP `8888` | Listening |
| Robot micro-ROS client session | Connected |
| `/odom`, `/imu`, `/scan` | Verified |
| `/map` and mapping TF | Verified |
| Motion command path | Verified through mux and watchdog |
| Physical motor response | Verified after restoring 2S battery power |
| Autonomous motion | Forbidden |

The robot is observable and mapping. Autonomous motion remains disabled until
the ROS safety package is deployed and tested.

## Repository Layout

```text
.
├── README.md
├── OPERATION_LOG.md
├── config/
│   ├── xuegeros.env.example.ps1
│   └── xuegeros.env.ps1        local only, gitignored
├── tools/
│   ├── xuegeros.ps1            maintenance and observation CLI
│   ├── xuegeros.cmd
│   ├── xuegeros_body_adapter.py
│   ├── xuegeros_body_adapter.cmd
│   ├── xuegeros_motion_gateway.py
│   ├── xuegeros_motion_gateway.cmd
│   └── test_xuegeros_adapters.py
└── ros2_safety/
    └── xuegeros_safety/        twist_mux + 200 ms watchdog
```

## Key-Only Authentication

The current toolchain does not accept server or guest passwords.

Local key layout:

```text
C:\Users\<user>\.ssh\id_loopros       server SSH key
C:\Users\<user>\.ssh\dsh_vm_key       guest SSH key
```

Guest access uses a local OpenSSH `ProxyCommand`:

```text
local SSH -> server SSH TCP forwarding -> guest SSH
```

No password is placed in process arguments, environment variables, or logs.

The one-time bootstrap that installed guest public keys is complete. Future
tooling should add keys through a controlled provisioning step, not by putting
passwords in command lines.

## Configuration

Create the local configuration once:

```powershell
Copy-Item .\config\xuegeros.env.example.ps1 .\config\xuegeros.env.ps1
notepad .\config\xuegeros.env.ps1
```

The local file is gitignored. It contains hostnames, local key paths, VM paths,
and topics. It must not contain passwords.

Required variables:

```text
XUEGEROS_SERVER_HOST
XUEGEROS_SERVER_USER
XUEGEROS_SERVER_KEY
XUEGEROS_VMRUN_PATH
XUEGEROS_VMX_PATH
XUEGEROS_GUEST_IP
XUEGEROS_GUEST_KEY
XUEGEROS_GUEST_RUNTIME_USER
XUEGEROS_GUEST_WORKSPACE
XUEGEROS_AGENT_BINARY
XUEGEROS_AGENT_LIBRARY_PATH
XUEGEROS_AGENT_LOG
```

Optional:

```text
XUEGEROS_SSH_PATH
XUEGEROS_CAMERA_URL
```

Load the configuration:

```powershell
. .\config\xuegeros.env.ps1
```

## Maintenance CLI

The CLI is read/write for local VM lifecycle and ROS process startup, but it has
no physical motion action.

```powershell
.\tools\xuegeros.cmd help
.\tools\xuegeros.cmd status
.\tools\xuegeros.cmd status -Json
.\tools\xuegeros.cmd start-vm
.\tools\xuegeros.cmd ensure-network
.\tools\xuegeros.cmd start-agent
.\tools\xuegeros.cmd agent-log
.\tools\xuegeros.cmd start-bringup
.\tools\xuegeros.cmd start-camera
.\tools\xuegeros.cmd start-mapping
.\tools\xuegeros.cmd start-nav2
.\tools\xuegeros.cmd mapping-status
.\tools\xuegeros.cmd check-forwarding
.\tools\xuegeros.cmd snapshot
```

The CLI must not be registered as a Persona tool. Persona may receive bounded
observations from the Body Adapter, not shell access.

### Status

`status` checks:

- VMware VM count and Tools state
- host UDP `8888/8889`
- guest `ens33` address
- Agent, camera, Gmapping, and Nav2 process presence
- Agent startup/log state

### Camera

The camera is independent of micro-ROS:

```text
http://<camera-ip>:81/ -> xuegecar_camera -> /camera/image_raw/compressed
```

`snapshot` captures one JPEG directly from the MJPEG stream.

### Mapping

`mapping-status` returns structured fields:

```text
scan_present
scan_rate_observed
map_present
tf_present
gmapping_running
```

Mapping is healthy only when all of the following are true:

1. The robot-side micro-ROS client has a session with the Agent.
2. `/scan` has fresh data.
3. Gmapping is running.
4. `/map` is updating.
5. `map -> odom` TF exists.

Starting Gmapping before `/scan` exists is not a valid mapping state.

## Read-Only Body Adapter

Start:

```powershell
.\tools\xuegeros_body_adapter.cmd
```

Default listen address:

```text
http://127.0.0.1:18110
```

Endpoints:

```text
GET /health
GET /v1/status
GET /v1/observations/latest
GET /v1/observations/camera/<short-lived-token>.jpg
GET /v1/mapping
GET /v1/navigation
```

The adapter:

- has no POST or write endpoint
- never exposes raw camera or lidar streams to Persona
- returns bounded status and mapping metadata
- returns a short-lived JPEG reference
- always reports `autonomous_motion_allowed: false`
- does not expose the maintenance CLI as a tool

Start with a different address:

```powershell
$env:XUEGEROS_ADAPTER_LISTEN_HOST = '127.0.0.1'
$env:XUEGEROS_ADAPTER_LISTEN_PORT = '18110'
.\tools\xuegeros_body_adapter.cmd
```

## Motion Gateway

Start the shadow-mode gateway:

```powershell
$env:XUEGEROS_OPERATOR_TOKEN = '<human-only-secret>'
.\tools\xuegeros_motion_gateway.cmd
```

Default listen address:

```text
http://127.0.0.1:18111
```

The gateway is disabled unless:

```text
XUEGEROS_MOTION_ENABLE=I_UNDERSTAND_PHYSICAL_RISK
```

It runs in shadow mode by default:

```text
XUEGEROS_MOTION_SHADOW=1
```

Shadow mode validates:

- human operator token
- short lease duration
- one active lease
- 200 ms command watchdog
- linear and angular limits
- explicit stop receipt

Shadow mode never publishes to the robot. A physical backend is intentionally
not configured.

API:

```text
GET  /v1/status
POST /v1/leases
POST /v1/commands
POST /v1/stop
```

State-changing requests require:

```text
X-Operator-Token: <human-only-secret>
```

The operator token is intended for a human-controlled Tool Gateway. It must not
be placed in Persona prompts, URL parameters, logs, or source control.

Default limits:

```text
lease <= 1000 ms
watchdog timeout = 200 ms
|linear| <= 0.1 m/s
|angular| <= 0.5 rad/s
```

## UDP Relay

VMware NAT UDP forwarding was not stable for micro-ROS request/response traffic.
The 8888 NAT rule was removed and replaced with a server-side relay:

```text
server 192.168.1.166:8888 -> guest 192.168.253.128:8888
```

Source:

```text
tools/xuegeros_udp_proxy.ps1
```

Server deployment:

```text
C:\Users\zhzaill\xuegeros_udp_proxy.ps1
C:\Users\zhzaill\xuegeros-udp-proxy.log
Scheduled task: XuegerosUdpProxy
```

The relay preserves a stable upstream UDP session and returns Agent responses to
the robot. Do not run it alongside the old 8888 VMware NAT rule.

## ROS-Side Safety

The package under `ros2_safety/xuegeros_safety` contains:

- `twist_mux` priorities for teleop, Nav2, and autonomy
- `cmd_vel_watchdog`
- 200 ms zero-velocity fallback
- velocity clamping
- short-lived lease requirement on `/motion/lease`
- `~/disarm` service

Data path:

```text
/cmd_vel_teleop     priority 100
/cmd_vel_nav        priority 50
/cmd_vel_autonomy   priority 20
        |
        v
   twist_mux
        |
        v
/cmd_vel_safe_raw
        |
        v
xuegeros_cmd_vel_watchdog
        |
        v
      /cmd_vel
```

The package is not deployed automatically. Review and build it in the VM only
after the robot observation graph is healthy.

## VLA Service (NoMaD, server-side)

Deployed 2026-10-24 on DESKTOP-6PVSJFV (`E:\vla\`). See
`docs/VLA-INTEGRATION-PLAN.md` for design and evidence. The service is a
**proposal service only** — it never talks to ROS or the car; every physical
pulse still goes through the motion gateway.

```text
E:\vla\nomad\vla_server.py     service (port 18300)
E:\vla\nomad\weights\          nomad.pth deployed (76,473,631 B,
                                sha256 70f79b82..., missing_keys=0)
E:\vla\pydeps\                 isolated python deps (ComfyUI venv untouched)
E:\vla\vla-ctl.ps1             start | stop | status | log
```

Operations from the local machine:

```powershell
# service control runs on the server:
ssh: powershell -File E:\vla\vla-ctl.ps1 status      # via plink (ComfyUI access/ssh.md)

# health from anywhere on the LAN:
curl http://192.168.1.166:18300/health

# weight drop-in (one command: upload, restart, sha256 + missing_keys check, smoke):
powershell -File tools\vla-weights-deploy.ps1 -Path <nomad.pth>
# (or drop the file into Downloads/Inbox and the watcher automation deploys it)
```

Acceptance rule: `/health` shows `weights.loaded: true` and
`missing_keys: 0` (currently satisfied; downloaded 2026-10-24 from the
official Google Drive via the server's Cloudflare WARP tunnel, byte-exact
against Content-Length).

API: `POST /goal {image_b64}` set goal frame · `POST /infer {image_b64,
goal_image_b64?, num_samples?, waypoint?}` → 8 waypoints + chosen waypoint +
`velocity_proposal {v,w}` (clamped v≤0.08, |w|≤0.4) · `POST /reset` clears the
rolling context. Measured on the P104-100 (fp32): p50 164 ms, p95 166 ms.

## Required Bring-Up Order

1. Confirm the VM and guest network.
2. Confirm Agent on `192.168.1.166:8888/udp`.
3. Point the ESP32 micro-ROS client to the Agent.
4. Confirm an Agent session.
5. Confirm one source each for `/odom`, `/imu`, and `/scan`.
6. Check data freshness and wheel-track calibration.
7. Start Gmapping.
8. Verify `/map` and `map -> odom`.
9. Build and inspect the ROS safety package.
10. Restore motor battery power, then run lifted-wheel tests with a human operator.

Motion remains blocked before step 10.

## Security Invariants

- No password arguments.
- No shell or PowerShell CLI exposure to Persona.
- No motion endpoint on the read-only Body Adapter.
- No physical backend in the shadow motion gateway.
- No raw high-frequency sensor stream in Persona context.
- No autonomous motion while `/odom`, `/imu`, or `/scan` is missing or stale.
- No `stop-motion` claim without a successful watchdog/stop receipt.
- Physical emergency stop remains mandatory.

## Tests

```powershell
python -m unittest -v tools/test_xuegeros_adapters.py
python -m py_compile `
  tools/xuegeros_body_adapter.py `
  tools/xuegeros_motion_gateway.py `
  tools/test_xuegeros_adapters.py
```

CLI syntax:

```powershell
$tokens = $null
$errors = $null
[System.Management.Automation.Language.Parser]::ParseFile(
  '.\tools\xuegeros.ps1',
  [ref]$tokens,
  [ref]$errors
) | Out-Null
$errors
```

## Known Limits

- Guest NetworkManager remains broken until `libssh.so.4` is repaired.
- Guest networking currently depends on manual DHCP for the active boot.
- NAT forwarding targets the DHCP address `192.168.253.128`.
- The Agent is started with `nohup`, not systemd.
- The robot-side micro-ROS session is live.
- Live `/scan`, `/map`, and mapping TF were verified.
- Movement commands reach the ESP32 (`target_vx` changes), but wheel velocity is
  zero when the 2S motor battery is disconnected or depleted.
- Low-speed forward, backward, left turn, and right arc were verified after
  battery power was restored.
- A pure negative angular command intermittently failed to appear on
  `/cmd_vel_safe_raw`, while a right arc command passed. Keep validating this
  before exposing turn-only motion.
- The Motion Gateway still has no physical backend. The operator-only
  `xuegeros_motion_backend.py` test path is not exposed to Persona.
