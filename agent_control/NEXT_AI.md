# 留给下一个 AI 的说明

这是标准化车辆控制流程的第一版实现。请先读 `docs/STANDARD-AGENT-CONTROL-FLOW.md`，再读 `protocol.py`、`policy.py`、`explorer.py` 和 `ros_mapping_controller.py`。

## robotd + 中控（2026-11-13）

- **一个进程**：`ros_cerebellum.py` 现在同时是 robotd 控制环和 WebSocket 服务
  （`robotd.py` 的 `RobotDaemon` + `DaemonServer`）。入口改名为 `robotd`。
- **拓扑**：本地 `tools/robot_console.py` 只发静态页；浏览器经
  `ssh -L 18202:/run/user/1000/xuegeros-robotd.sock` 直连 robotd，50 Hz `state`、
  10 Hz `scan`、1 Hz `map`、离散 `events`。细节见 `docs/ROBOTD-CONSOLE.md`。
- **没有限时授权**：`propose` 用最近扫描评估并给出限幅距离与 3 s 新鲜窗，
  `execute` 前再查一次世界；`stop` 不需要提议。不要加回“人工开 10 分钟”。
- **新增 `back` 技能**（原语 `retreat`），按请求扇区的反方向检查净空；
  `skills.json` 与 `skills.py` 的内置表必须同步，`test_skills.py` 会比对。
- 离线 mock：`python -m agent_control.robotd --tcp --port 18202`；本机没有
  `AF_UNIX`，UDS 只能在 guest 上验收。

## 小脑（2026-11-11 新增）

- `cerebellum.py` 是纯逻辑，无 ROS 依赖；`ros_cerebellum.py` 只是适配器。改行为优先改前者并补 `tests/test_cerebellum.py`。
- 模型接口是**意见，不是速度**：`/xuegeros/priority` 收 `{veto, priority[12], mode, reason, ttl_ms}`。
  `priority` 必须正好 `sectors`（默认 12）个，超出 `[0,1]` 会被夹紧；长度不符直接拒绝。
- 融合公式 `score = geometry_weight*geom + priority_weight*priority`，`geometry_weight` 下限 0.5。
  **不要把模型权重调高到能压过几何**——那是把否决权交回给不可靠的模型。
- 意见会过期（默认 3 s），过期后自动停车并上报 `priority_expired`。这是核心安全性质，不要改成「保持最后一个意见」。
- `veto=true` 优先于显式 goal，且只消费一次。
- 小脑写 `/cmd_vel_nav`（twist_mux 优先级 50），不是 `/cmd_vel_autonomy`。不要改回同一个槽位。
- 影子模式对命令话题完全静默；`decision.publish` 为真也不代表已发布，只有 `physical=true` 才发。
- 启动：`ros2 run agent_control robotd --ros-args -p physical_enable:=true -p confirm_physical:=true`。
  默认两个参数都是 false。`tools/deploy-agent-control.ps1` 会递归复制整个 `agent_control/`，新文件无需改部署脚本。
- **扇区目标 + 技能表（2026-11-12，未部署）**：`/xuegeros/goal` 接受 `sector`（0–63，再按
  `sectors` 校验）。词表、参数边界、执行原语由 `agent_control/skills.json` 决定
  （`skills.py` 加载/校验，表坏了节点拒绝启动）：`forward` 是两段式（偏差 >15° 先原地转，
  再前进），`face` 原地转到扇区中心，`stop` 零速；未知技能回 `unknown_skill` + 已知列表。
  `look` 是 MCP 侧的词，落到车上叫 `face`。状态里多了 `skills_version/skills_source/`
  `goal_sector/goal_phase/goal_primitive/goal_remaining_deg`。
  已过 85 项包测试（含技能表 15 项），但 **guest overlay 未构建、实车未验证**。

ROS executable 安装位置由 `setup.cfg` 固定为 `lib/agent_control`；如果新增入口但 `ros2 pkg executables` 看不到，先检查标准 ament 安装布局。

- 默认是影子模式；不要为了“验证代码”直接打开物理参数，也不要向真实车辆发送速度。
- 真实固件仍未确认是否带 MCU 命令超时；服务器/ROS watchdog 不能替代急停。
- 当前固件可能发布 BEST_EFFORT 电池话题，ROS 控制器已使用 sensor-data QoS。
- 物理模式要求能读到有效电池电压；未知电压只能继续影子运行。
- 不要把 `WorldSnapshot` 的 adapter 快照新鲜度误写成传感器硬件时间戳；实车验收时需要记录 `/scan`、`/odom`、TF 的实际消息时间。
- `/map` 也必须有明确更新证据；仅有 `map.present` 不代表地图正在变化。
- 扫描超时默认 2.5 s，是针对历史 0.7–1.1 Hz 实测的临时窗口；重新测得稳定频率后再收紧。
- 规划器要求每帧至少 12 个有限正 range；不要为了“让车动起来”放宽无效扫描门槛。
- 在改动动作接口前，保持唯一写入路径：`/cmd_vel_autonomy -> twist_mux -> watchdog -> /cmd_vel`。
- 下一步优先做仿真/录包回放、唯一 `/scan` 验证、固件版本/NVS 读取和 MCU 超时验证。
- 插件可用 `XUEGEROS_AGENT_EVENT_LOG` 写 JSONL 事件；用 `event_id/session_id/turn_id`
  对齐动作与传感器记录，不要把旧 VLA 影子日志直接当训练标签。
- Guest 源码测试要设置 `PYTHONPATH=/home/chuyun`；正式 ROS 运行应使用 colcon install overlay。
- `ros_shadow_smoke.py` 是 domain 42 的隔离 ROS 集成验收：合成 scan/odom/map/TF/battery，验证 preflight、控制器状态和零速输出；不要在 domain 0 或实车上运行它。
- Guest smoke 已通过：preflight OK、控制器进入 MOVE、11 条 autonomy 消息全部为零；运行时先 `source /opt/ros/humble/setup.bash` 和安装 overlay，再执行脚本，不能用 `PYTHONPATH=/home/chuyun` 覆盖 ROS 路径。
- 2026-11-05：`mapping` 先执行 `python -m agent_control.ros_preflight`，必须确认唯一且有新时间戳的 `/scan`、`/odom`、`/map` publisher 和 map->base TF；物理模式还检查最近电池消息。当前服务器有车端 `/scan`/`/odom` 和 relay client，但 GMapping/bringup 进程不在 ROS graph，门禁应失败。GMapping 的 map->odom 时间戳按源码会比 ROS now 超前 50 ms；只允许 100 ms 内的 TF 前置，不可把普通未来传感器时间戳一并钳成新鲜。
- AI 插件的物理提案每次都要确认 Gateway `enabled`、`shadow_mode=false`、operator token 配置与 physical backend；最终命令还必须报告 `published=true`，只 accepted 不算执行成功。

## 2026-11-13 夜：直接 goal 与巡游桥（断电前状态）

- **已部署并在实机影子验证**：`{t:"goal"}` 一次下发，事件只有
  `command / motion_started / motion_done / motion_vetoed / motion_blocked / command_failed / stop`；
  心跳 `last_block`。实机结果：`forward sector 6` 净空 0.259 m 接受（阈值 0.25），
  8 s 后 `motion_done reason=timeout traveled_m=0.0`、位姿不动——控制器说法与外部事实已经分开。
- **已部署并在实机影子验证（2026-11-14）**：
  - 断电后 VM 未自启，`vmrun list` 为 0；用 `xuegeros.ps1 start-vm` 拉起。
  - 重启后出现 DDS 图分裂：新节点能收 `/scan`/`/odom`，收不到 `/map`、看不到 map 坐标系；
    重启 bringup + gmapping 后恢复。`ros2` CLI daemon 也坏了（`!rclpy.ok()`），
    `ros2 daemon stop/start` 修复。
  - 巡游链路实机影子通过：`tour_started` → `EXPLORE`，facts 含地图 1419 格 / 24 前沿、
    IMU roll 2.6° pitch 3.9°、发布者数、命令链；规划角速度 0.3、执行器 0；
    30 s 后 `LIMIT no_progress`，`encoder_path_m=0`、`known_delta_start=0`。
  - 本轮修掉的真问题：30 s 空闲客户端被 robotd 断开；静态地图被误判 `complete`；
    最近的未探索前沿落在 0.22 m 内导致永远零速；重启后旧 facts 被当成新 tour 的完成；
    `cancel("tour")` 产生假的 `motion_blocked`。
- **仍未做**：物理开关保持 false，实车巡游未跑；World 五个工具只过了离线测试；
  adapter 的 `/v1/map.png` 与相机帧未在本次实机联调（adapter 未启动）。
- 恢复后的第一步：部署 → `colcon build --packages-select agent_control` → 重启 robotd →
  在 workspace 启动 `ros2 run agent_control mapping_controller --ros-args -p physical_enable:=false`，
  然后发 `{t:"tour",action:"start"}` 看 `/agent/tour_facts` 1 Hz 是否到 robotd。
- **2026-11-14 实车两步（World 流程）**：子代理伪装 Cortico，通过
  `xuegeros_goal` 两次 `{forward, sector 4, 0.08 m}`。第 1 步净空 0.263/0.25 m、
  实走 0.0823 m；第 2 步净空 0.352/0.25 m、实走 0.0843 m；pose 从
  `[0.001,0,-0.015,-0.015]` 到 `[0.065,-0.155,-1.616]`，无否决/阻塞事件。
  注意：扇区 4 是 -45°，但最终 yaw 约 -92.6°，两段式转向似乎在第二个目标里多转了一段，
  下次实车先只发一个带扇区的前进并记录 odom yaw 与 map yaw 的差。
  测试后已把 robotd 重新切回 `physical=False`。
- 物理开关仍在启动参数；tour 命令不能自行 arm。开启前先确认前方净空与电池读数
  （本次 facts 报 `battery_v=9.949`，高于 2S 包上限 8.6，疑似仍取到非电池轨，先查清）。

## 2026-11-14 夜：覆盖式扫掠（新策略，未上车）

- **为什么换**：贪心前沿没有世界记忆，近处来回换目标；轮式 odom yaw 比 IMU 慢约 10 倍；
  watchdog 对非零命令只按 1 Hz 刷新，车实际只走约 1/4 时间。继续调阈值没有意义。
- **新模块** `agent_control/coverage.py`：纯 Python、不依赖 `/map`。固定车道几何：
  `LANE → TURN(90°) → SHIFT(一条车道宽) → TURN(反向) → LANE`，到底或遇到障碍就换道，
  `max_lanes` 后回起点；带 `no_progress` / `runtime` / `travel` 包络。
  yaw 一律用调用方传入的 IMU 融合值，不看 odom yaw。
- 已接进 `ros_mapping_controller`：`strategy:=tour|coverage`，coverage 参数
  `lane_length_m/lane_width_m/max_lanes`。131 项包测试通过（coverage 7 项）。
- **未部署、未上车**。下一步顺序：
  1. 部署 `cmd_vel_watchdog` 的 keepalive 修复（非零命令 10 Hz，零命令不刷屏），
     `colcon build --packages-select xuegeros_safety`，`systemctl restart xuegeros-safety`。
  2. 执行标定：小角速度 vs IMU、小线速度 vs 扫描/编码器，得到比例与死区。
  3. 用 `strategy:=coverage lane_length_m:=0.8 lane_width_m:=0.5 max_lanes:=3`
     跑一次短扫掠；先影子，再物理。
- **注意**：`/map` 的 DDS 图分裂会自动复发；bringup/gmapping 是 systemd 服务
  （`xuegeros-bringup.service`、`xuegeros-gmapping.service`），不要手起进程叠实例；
  用 `GuestRootSsh 'systemctl restart ...'` 按 bringup→gmapping 顺序重启。

## 2026-11-15：3D 模型加载（已修）

- `model3d.html` 以前只应用每个 `<visual>` 的 origin，**没有走 `<joint>` 树**，9 个 STL
  各自被关节偏移错位。现在解析 joint（parent/child/origin），递归算 link 世界矩阵，再套
  visual origin；rpy 按固定轴 `Rz*Ry*Rx` 合成四元数。页面 meta 应为
  `9 网格 / 14 链 / 13 关节`。
- 另有一个坐标系错误：**URDF/Gazebo 是 Z-up，three.js 是 Y-up**。整辆模型必须
  `robot.rotation.x = -Math.PI / 2`，否则相对地面网格是躺倒/歪的（2026-11-15 用户现场指出）。
- 数据链路本身没问题：`/v1/model` 与 `/v1/meshes/<name>` 均 200，9 个 STL 齐全；
  3D 页需要 `ssh -L 18201:127.0.0.1:18201`。
- Gazebo 侧同样换成这 9 个 STL（统一 `(0.00225,0,-0.00925) Rz90°`），collision 仍用简化
  几何；原生窗口用 `tools/sim.ps1 up showcase.world -Gui`。
