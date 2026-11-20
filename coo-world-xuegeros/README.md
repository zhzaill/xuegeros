# XuegerOS — Coo 小车插件与本机仿真工作台

独立安装需要 Node.js 22+ 和 Python 3.10+，不需要 ROS、Gazebo 或原工作区。

```sh
npm install -g cortico-world-xuegeros
xuegeros doctor
xuegeros sim
```

或者 `npx --package cortico-world-xuegeros xuegeros sim`。
浏览器打开 http://127.0.0.1:18142/ 。Ctrl+C 结束服务。
地图 PNG 功能另需 `python -m pip install Pillow`；3D、控制和状态接口不依赖 Pillow。
通过 `XUEGEROS_PYTHON` 可指定 Python 可执行文件。

工作台的“中控与小脑”可编辑 24 项执行参数；同一 API 是
`GET /api/control` 与 `POST /api/control`（body 为参数名到数值的 JSON 对象）。
空闲时一次保存并生效，默认保存在 `~/.xuegeros/simulation.json`。
`--config <文件>` 选择另一个配置；不会写入 npm 安装目录。

仿真工作台的「点选导航」可以在地面选点或输入 X、Y，再点「前往目标」。车辆沿雷达已观测的空闲路线行驶；路线不可用时显示原因并停车。暂停、继续、取消和停车与巡游共用。
`GET /api/interfaces` 列出全部 HTTP 接口和 robotd WebSocket 地址。
`POST /api/command` 接受 robotd 的 goal / stop / tour / propose / execute 等现有命令。
例如 `{"t":"goal","goal":"forward","distance_m":0.1}`，停车为 `{"t":"stop"}`。
动作统一经 robotd 与小脑执行。默认监听本机回环；局域网共享：

```sh
xuegeros sim --host 0.0.0.0
```

其他电脑浏览器打开 `http://<运行仿真的电脑IP>:18142/`，工作台自动使用相同主机的
WebSocket。远端 Coo 把 `simulationHttpUrl` 和 `simulationRobotdUrl` 分别设为
`http://<IP>:18142` 与 `ws://<IP>:18242/ws`。Windows 防火墙需放行两个 TCP 端口；
在管理员 PowerShell 执行 `xuegeros firewall`（只允许 LocalSubnet）。

```sh
xuegeros sim --world room_open.world --http-port 18143 --port 18243
xuegeros sim --mesh-dir /path/to/your/meshes --config /path/to/control.json
```

包内提供程序化小车；本机已有的 STL 可通过 `--mesh-dir` 加载。
公开包不附带许可未确立的 CAD 或模型权重。

## Coo / Cortico

将 `cortico-world-xuegeros` 安装到 Coo 的扩展依赖目录，重启并启用 World。
新安装默认使用仿真。在「动作面板」点「启动仿真工作台」，等「打开仿真工作台」出现后
在浏览器查看 3D 场景。请先安装 Python 3.10+。Coo 会复用已运行的仿真，
或从随包运行时启动一个本机进程。关闭 World 会结束由它启动的进程。
独立运行用 `xuegeros sim`。默认 robotd 地址 `ws://127.0.0.1:18242/ws`、
`simulationHttpUrl=http://127.0.0.1:18142`。改端口时两处一并修改。
局域网服务由仿真主机运行 `xuegeros sim --host 0.0.0.0` 启动。
已有 World 保留保存过的运行目标，可在设置里改成 `simulation`。
Coo 面板已移除旧的「控制窗」「地图/雷达」「3D 模型」快捷入口。
Coo 面板提供动作、巡游、参数编辑及独立工作台入口，不自动刷图。
所有现有连接地址、超时与事件参数都可在 World 设置中编辑。
工具增加 `xuegeros_control` 与 `xuegeros_interfaces`，原有观察、动作、巡游、地图、
截图、ROS、源码工具继续可用。截图取自打开的独立工作台。

本包是运动学仿真与插件，不包含真实 ROS 部署。现有本机场景巡游验收与
真车能力分别记录，参数开放不会把仿真变成物理仿真。

## 仿真入口打不开

| 遇到的情况 | 怎么恢复 |
| --- | --- |
| Coo 里找不到 XuegerOS 小车，或动作面板没有仿真入口 | 在「World 扩展」启用 XuegerOS，再将「运行目标」设为 `simulation`。旧安装会保留上次保存的目标。保存后重新打开动作面板。如果面板还显示「物理车端」，重启 Coo 后再试。 |
| 仍看到「控制窗」「地图/雷达」「3D 模型」，或面板提示 `Failed to fetch dynamically imported module` | 在 Coo 的扩展列表核对已加载的 `cortico-world-xuegeros` 版本。若还在用旧版本或旧的 `file:` / `link:` 安装，切换到 npm 的 `0.3.7`。完全退出并重启 Coo，再打开动作面板。 |
| Coo 启动工作台失败，或浏览器打不开 `18142` | 先运行下面的 `doctor` 命令，确认已安装 Python 3.10+ 且 `runtime` 为 `true`。再从终端启动工作台。 |
| 在 Coo 卸载时看到 `ERR_PNPM_UNEXPECTED_STORE` | 这是 Coo 管理依赖时使用了不同的 pnpm 存储目录。把扩展版本和第一条报错交给 Coo 安装维护者，工作台可先通过下面的命令独立运行。 |

终端里依次运行以下命令。保留第二条命令的终端窗口，然后打开 [本机仿真工作台](http://127.0.0.1:18142/) 。如果默认端口被占用，可在第二条命令末尾加 `--http-port 18143 --port 18243`，浏览器改用 `18143`。要让 Coo 控制同一个仿真，Coo 设置里的 HTTP 和 WebSocket 端口也要一起改。

```sh
npx --yes --package=cortico-world-xuegeros@0.3.7 xuegeros doctor
npx --yes --package=cortico-world-xuegeros@0.3.7 xuegeros sim
```


## 开发与历史部署说明

# XuegerOS World（Cortico / Coo）

把车上 robotd 的动作契约接进 Coo：工具、事件、环境提示词和动作面板。

## 拓扑

```text
Coo（Electron 扩展进程）
  └─ XuegerosWorld ── WebSocket ──> 127.0.0.1:18202 ── ssh -L ──> robotd.sock
       ├─ tools: observe / propose / goal / stop / look
       ├─ events: xuegeros.motion（拒绝、否决、阻塞、命令错误）
       └─ JSONL: <dataDir>/xuegeros-events.jsonl（cursor 补投）
```

World 不拥有控制环，也不发速度；单步意图统一走 robotd 的 `goal`，预检用 `propose`。

## 安装

```powershell
# 1. 隧道（robotd 与模型/遥测）
ssh -N -L 18202:/run/user/1000/xuegeros-robotd.sock -L 18201:127.0.0.1:18201 chuyun@<guest>
# 2. 可选的独立真车中控
python tools\robot_console.py --port 18112 --token <robotd-token>
# 3. Coo 扩展
#    data/extensions/package.json 里加 "cortico-world-xuegeros": "link:<本目录>"
#    重启 Coo，在 World 设置里启用 worlds.xuegeros
```

`robotdToken` 必须与 robotd 的 `console_token`/`--token` 一致；它只用于这条
World 连接，不是 operator token。

### 本机轻量仿真

仿真使用独立的回环端口，避免误连车端隧道。将 World 配置为
`deployment: "simulation"` 后，它只会连接 `simulationRobotdUrl`
（默认 `ws://127.0.0.1:18242/ws`，也可填写局域网主机）；服务端还必须在状态帧中明确报告
`physical: false`，否则动作会被拒绝。不要把仿真端口改成车端的 `18202`。

```jsonc
{
  "worlds": {
    "xuegeros": {
      "deployment": "simulation",
      "simulationRobotdUrl": "ws://127.0.0.1:18242/ws",
      "robotdToken": ""
    }
  }
}
```

这只是控制端配置；发布前仍需通过本机仿真的模型、场景、实际位姿变化和停车验收。

## 工具

| 工具 | 作用 |
|---|---|
| `xuegeros_observe` | 状态、位姿、扫描龄、12 扇区、巡游 phase 与可用动作 |
| `xuegeros_propose` | 只预检：返回限幅距离、净空、可用扇区 |
| `xuegeros_goal` | 直接下发单步意图；未下发时返回测量与原因 |
| `xuegeros_stop` | 立即停车，不需要预检 |
| `xuegeros_look` | 单帧视觉问答（相机 + VLM），不用于控制 |
| `xuegeros_tour` | 房间巡游 start/stop/status；返回 phase 与原始 facts |
| `xuegeros_map` | 地图 PNG 链接与元信息 |
| `xuegeros_screenshot` | 按需截一张相机帧，返回图片链接 |
| `xuegeros_ros` | 原样透传 `/v1/state` 与 `/v1/graph` |
| `xuegeros_source` | 只读查看白名单目录内的源码（≤200 KB） |

## 事件

只有这些会唤醒人格（`xuegeros.motion`）：

- `motion_vetoed` → `interrupt`
- `motion_blocked` / `command_failed` / `command_error` / `tour_done` → `flush`

普通 `state`（IDLE/ACTIVE/DONE）、`motion_started`、`motion_done` 只落盘、不推送。
事件先落盘，宿主确认接收后记录持久化送达凭据；重启补投与 robotd 历史补发共用去重。
身份包含序号、源时间戳与事件种类，robotd 重启后序号复用不会吞掉新通知。
`pushEvent` 失败仍保留待投事件。升级会迁移旧 cursor；无法保证进程恰在宿主接收与凭据落盘之间崩溃时的恰好一次投递。

## 测试

```powershell
node --test coo-world-xuegeros/test/
```

## 已知限制

- 只支持 `ws://`（明文）；Unix socket 依赖 `ssh -L`，没有原生 UDS 支持。
- `xuegeros_look` 依赖相机与 VLM；二者离线时必然失败，这是事实。
- 本机 Coo → World → robotd → 仿真位姿变化与停车已验收；真实车辆路径另行验收。
