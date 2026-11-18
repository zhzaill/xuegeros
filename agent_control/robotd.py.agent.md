# Handoff: robotd.py

## 用途 / 入口

robotd 是车端唯一控制进程：`ros_cerebellum.py` 的 ROS 节点创建
`RobotDaemon` + `DaemonServer`，控制环、协商和中控数据都出自同一对象。
`python -m agent_control.robotd --tcp --port 18202` 是离线 mock（合成世界，
不接 ROS）。

## 结构

- `RobotDaemon`：纯 Python，持有 `Cerebellum`、最近的扫描/位姿/地图、提议、
  事件环形缓冲；`tick()` 由 ROS 定时器调用；`command()` 处理
  `propose/execute/stop`。
- `DaemonServer`：WebSocket 传输。真实部署用 Unix socket（默认
  `/run/user/1000/xuegeros-robotd.sock`，`0600`），mock 用 TCP。广播循环
  50 Hz 发 `state`，每 5 tick 发 `scan`（10 Hz），每 50 tick 发 `map`（1 Hz），
  有离散事件就发 `events`。设了 `token`（ROS 参数 `console_token` / mock 的
  `--token`）后，握手必须带 `?token=...`，否则 401。

## 协议

见 `docs/ROBOTD-CONSOLE.md`。改动帧字段时同步更新该文档和
`agent_control/console.html`。

## 协商语义（不要退回限时授权）

- `propose` 用**最近一次 tick 的扫描**评估，返回 `proposal_id`、被限幅的
  `distance_m`、`expires_in_ms`（3 s）和可用扇区。
- `execute` 消费提议前**重新检查世界**；过期或世界变了就拒绝。
- `stop` 不需要提议。
- 提议窗口是新鲜度界限，不是人工授权；不要在机器人侧加“保持最后提议”。

## 前置条件与依赖

- guest Python 3.10 + rclpy（真实）；本机 Python 3.11 只能跑 mock/TCP。
- 真实 socket 目录 `/run/user/1000` 必须属于运行用户；否则节点日志报
  `console socket unavailable`，控制环继续跑（这是有意的降级）。

## 验证

```powershell
python -m unittest agent_control.tests.test_robotd agent_control.tests.test_negotiation
python -m agent_control.robotd --tcp --port 18202
```

测试覆盖：50 Hz 状态帧、propose/execute、拒绝原因、stop、离散事件、
Unix socket 上完成 WebSocket 握手（Windows 跳过）。

## 已知限制 / 未决项

- `scan` 降到 180 点、`map` 降到 ≤400×400；只用于显示。
- 事件缓冲 256 条；连接时补发最近 50 条。
- 未在实车上验证；车离线期间只做了 mock 与单元测试。
- 没有鉴权时安全边界是 loopback + SSH + `0600`；生产应设 `console_token`，
  World 与本地控制台各持一份，不复用 operator token。
