# XuegerOS 仿真数据契约（场景 / 传感器 / 截图 / 长任务）

> 状态：**草案 v0.1（2026-11-17）**。本文件是 `sim/local_sim.py`、`sim/local_web/`、
> `coo-world-xuegeros/` 与将来的 `tour` 策略共同遵循的**唯一口径**。
> 任何"看起来像在跑"的能力，只要不满足本契约的字段与判据，就必须报告为不可用。

## 0. 为什么需要它

工作台已经能跑，但"能跑"不等于"已验证"。三类混淆会直接导致假闭环：

1. **把外观当配置**：看到车身上有个圆盘就当雷达可用；看到 MJPEG 就当 ROS 图像源可用。
2. **把真值当地图**：SDF 几何渲染出的房间、雷达扫描建出的图、真车 `/map`，三者混用一个 `map` 字段。
3. **把长任务当一个 `ACTIVE`**：没有暂停/恢复/取消/超时/阻塞/接管的语义。

本契约把这三件事各自固定成独立字段与判据。

## 1. 职责边界（谁可以产生动作）

| 层 | 职责 | 可以做的事 | 禁止 |
|---|---|---|---|
| **tour** | 任务编排与恢复 | 选择场景族、解释指令变体、决定开始/暂停/恢复/取消；记录 cursor 供回放 | 直接下发速度；直接发 goal；自己做避障 |
| **coverage** | 场景族上的确定性策略 | 依据场景几何产出**一个动作提案**（v, w） | 自己发 goal；抢 tour 的控制权；修改场景 |
| **robotd / Cerebellum** | **唯一动作执行层** | 校验技能词表、净空、新鲜度，产出 `/cmd_vel_*` | 被绕过（任何速度必须经它） |
| **workbench（local_web）** | 观察与人机输入 | 显示状态、发单步 goal、显示传感器与地图来源 | 自行积分位姿；自行判定传感器可用 |
| **Coo 面板** | 人与 persona 的入口 | 与 workbench 同一 `goal`/`stop` 协议 | 直接写速度 |

**互斥规则**：同一时刻只有一个 goal 持有者。tour 运行期间，workbench 的单步按钮必须
置灰（或返回 `busy`），避免两个来源各自发 goal 抢控制权。

## 2. 场景族与指令变体（可复现的最小单元）

一次可比较的实验由下面这个元组唯一确定，缺一项就不可复现：

```jsonc
{
  "scene_family": "room",              // 场景族：room / room_open / <新增>
  "world": "sim/xuegeros_sim/worlds/room.world",
  "world_sha256": "<文件内容哈希>",     // 世界内容变了就不再是同一族
  "variant_id": "room/crate-diagonal/01",
  "seed": 20261117,                    // 位姿抖动、噪声、随机化都用它
  "start_pose": [0.0, 0.0, 0.0],
  "map_version": "truth/v1",           // 见 §4，必须指名是哪种地图
  "instruction_id": "sweep-room/zh/v1",// 指令文本的版本号，不是文本本身
  "instruction_text": "把房间扫完并回到起点",
  "acceptance": {                      // 见 §6
    "min_known_cells": 2000,
    "max_runtime_s": 300,
    "return_distance_m": 0.2,
    "no_forbidden_state": ["no_progress"]
  }
}
```

规则：

- **参数不进指令**：`coverage` 的车道长度/宽度不写进指令文本，而是由 `variant_id` 指定的
  变体配置提供；指令文本只说意图（"扫完这个房间"），由策略决定怎么做。
- **变体 ID 必须能定位到文件**：`scene_family/variant/序号`，变体描述文件放在
  `sim/contracts/variants/`，与 `world` 哈希一同进入运行记录。
- **seed 必须写进运行记录**，否则"同一个覆盖任务"两次不可比较。

## 3. 传感器契约

每个传感器必须有下面这组字段；缺一就该报 `unavailable`，**不得**由外观推断。

```jsonc
{
  "id": "lidar",
  "frame": "laser_frame",                 // URDF frame 名
  "topic": "/scan",                       // 真实 ROS topic；仿真替代物另注
  "msg_type": "sensor_msgs/LaserScan",
  "rate_hz": 2.5,
  "mount": {"parent": "base_link", "xyz_m": [-0.0227, 0.000062, 0.0637]},
  "source": "sim|ros|unavailable",        // 这一帧的数据到底从哪来
  "substitute": "sim_sdf_raycast",        // 若 source=sim，说清替代物
  "age_s": 0.05,
  "availability": "ok|stale|missing|unavailable",
  "reason": null                          // 不可用时必须给原因
}
```

**当前实测口径（2026-11-17，以 URDF 为准）**

| id | frame | topic | 仿真侧 | 真车侧 |
|---|---|---|---|---|
| `lidar` | `laser_frame` | `/scan` | `source=sim`，`substitute=sim_sdf_raycast`，360 束 / 2.5 Hz / 0.05–8 m | `source=ros`（`/scan` 实测 2.5 Hz） |
| `imu` | `imu_link` | `/imu` | **`unavailable`**：本地运动学模型没有 IMU 模型 | `source=ros`（约 4 Hz） |
| `camera` | **URDF 无 `camera_link`** | 无 | **`unavailable`**：`camera.stl` 只是视觉件 | MJPEG 画面流可用，但 **ROS 相机节点未运行**（`camera.ros_node_running=false`），故 `source=unavailable` 作为控制图像源 |
| `ultrasonic` | **URDF 无 link** | 无 | **`unavailable`**：`ultrasonic.stl` 只是视觉件 | 避障模块存在，但未暴露 ROS topic |
| `wheel_odom` | `base_footprint` | `/odom` | `source=sim`，`substitute=kinematic_integration` | `source=ros`（13 Hz） |

推论（必须写进 UI）：

- 仿真**不能**宣称"相机已验证"或"视觉避障已验证"；它只有合成雷达。
- MJPEG 是**画面流**，不是可用于控制的 ROS 图像源；两者不可互换。
- 圆形外观不构成雷达判据；`laser_frame` + `/scan` + URDF 挂载点 + 实际 topic 四方对齐才算。

## 4. 地图契约：三张图不能混用一个字段

| 字段 | 含义 | 来源 | 判据 |
|---|---|---|---|
| `maps.truth` | 场景真值 | SDF/世界几何直接投影 | 与 `world_sha256` 绑定；用于评估，不用于导航 |
| `maps.slam` | 雷达建图结果 | 由 `/scan` + TF 建出 | 必须有 `map_version`、`known_cells`、`resolution`、`updated_at` |
| `maps.vehicle` | 真车 `/map` | 车端 gmapping | 必须与真车 `map_version` 对齐 |

规则：

- 三者**字段分离、各自带 provenance**；任何"地图"读数必须说明是哪一张。
- 覆盖率类指标必须声明分母来自哪张图（`truth` 与 `slam` 的分母不同，结果不可互比）。
- 仿真没有 SLAM 时，`maps.slam.availability = "unavailable"`，不得用 `truth` 顶替。

## 5. 截图契约：至少三种，各带元数据

| 种类 | 来源 | 必须有 |
|---|---|---|
| `camera_front` | 前视相机 | `timestamp`、`frame`、`pose`、`source`（真实 ROS 图像 / MJPEG / **unavailable**） |
| `map_top` | 地图俯视 | `map_kind`（truth/slam/vehicle）、`map_version`、`resolution`、`known_cells`、`timestamp` |
| `workbench` | 3D 工作台 | `pose`、`camera`（视角）、`backend`、`physical=false` |

共同字段：`captured_at`、`age_s`、`provenance`（measured/glimpsed/inferred）、`frame`。
**工作台截图不得作为相机证据**；相机不可用时截图必须显式标 `unavailable`，不得用渲染画面补齐。

## 6. 长任务状态机

长任务（tour / coverage / 本机点选导航）至少暴露这些状态与事件，单靠 `ACTIVE` 不合格：

```text
IDLE → STARTING → RUNNING ⇄ PAUSED
                     ├→ BLOCKED      (障碍/传感器陈旧，带 reason 与 clearance)
                     ├→ TIMEOUT      (包络超时)
                     ├→ HANDOVER     (需要人工接管)
                     ├→ CANCELLED
                     └→ DONE         (reason: reached | exhausted | stopped)
```

每个状态转换必须写一条事件：`{at, from, to, reason, facts}`；`facts` 至少含
`pose`、`scan_age_s`、`known_cells`、`traveled_m`、`strategy`、`phase`。
`STOPPED` 与 `CANCELLED` 都必须落到零速并回执。

### 6.1 语义（2026-11-17 定稿）

| 动作 | 结果 | 硬要求 |
|---|---|---|
| `pause` | → `PAUSED` | **一个控制周期内**落到零速，不是"不再推进" |
| `resume` | `PAUSED` → `RUNNING` | 先重查 `scan_fresh`；陈旧则转 `BLOCKED`，不续跑 |
| `cancel` | → `CANCELLED` | 落零速 + 回执 |
| `stop` | → `DONE(reason="stopped")` | 落零速 + 回执；与 `cancel` 区分语义 |
| `timeout` | → `TIMEOUT` | 由 runtime/travel/phase 包络触发 |

- 事件必须有**单调 cursor**，供回放对齐；`pause/resume` 这类"无位姿变化"的转换也要记。
- 观察接口固定形状（UI 直接消费，不猜）：
  ```jsonc
  {"state":"RUNNING","reason":"goal_active","strategy":"coverage","phase":"LANE",
   "busy":true,"facts":{"pose":[0,0,0],"scan_age_s":0.05,"known_cells":0,
   "traveled_m":0.12,"strategy":"coverage","phase":"LANE"},"events_cursor":42}
  ```
  `busy` 在 `STARTING/RUNNING/PAUSED/BLOCKED` 为 `true`。
- 策略持有租约期间，单步 `goal`/`propose`/`execute` 一律返回
  `{"accepted":false,"busy":true,"error":"busy","reason":"strategy_active"}`；
  UI 必须原样显示这条回执，不得吞掉。

## 7. 隔离与措辞

- 场景族/指令变体测试**只允许** `127.0.0.1` 的仿真端口（`18142` HTTP / `18242` WS）；
  不得指向真车、不得指向 `18202`（物理 robotd 口）。
- 所有状态必须显式 `physical=false`；物理端口必须保持关闭（`local_acceptance.py` 已在断言）。
- 措辞纪律：`仿真可运行` ≠ `传感器链路已验证`；`画面流可用` ≠ `图像源可用`；
  `SDF 真值地图` ≠ `建图结果`。

## 8. 待办（按依赖顺序）

1. ✅ `local_sim`：新增 `GET /api/sensors` 与 `GET /api/maps`，按 §3/§4 输出，缺失即 `unavailable`
   （2026-11-17 已实现并实测：lidar/odom = ok，imu/camera/ultrasonic = unavailable 且带原因；
   maps.truth = ok，maps.slam / maps.vehicle = unavailable）。
2. 🚧 `local_web`：显示传感器清单（frame + 可用性 + 原因）与地图来源；相机不可用时明确标红
   （代码已写，视觉验收归 Half B / `task-2`）。
3. ✅ `coverage`：参数改为引用 `variant_id`，不再接受裸的 `lane_length_m/lane_width_m`（Half A / `task-1`）。
4. ✅ `tour`：已接入 §6 状态机（2026-11-18）。两个策略共用同一后端接口
   `start / step / rebase / stop / snapshot`，由 `StrategyRunner` 统一持有租约、状态、事件与验收；
   `strategy=coverage|tour` 在 `POST /api/strategy/start` 选择。仿真实测
   `room_open`：`IDLE→STARTING→RUNNING→DONE(reached)`，known 14390/14400、回到起点、
   acceptance `met=true`、cursor 单调；`room`（有箱体）：`RUNNING→TIMEOUT(no_progress)`，
   即策略自己的包络正确接管。**历史缺陷（本机修复见 §10）**：`room` 里 tour 会在箱体附近进入
   原地左右摆动的局部极小（yaw ±0.15 rad、约 1 Hz、位置不动），需修控制律并补闭环回归，
   不属于接入问题。
5. 🚧 截图：三个端点/命令，带 §5 元数据（Half B / `task-2`）。

## 9. 工作台与仿真的运行方式（2026-11-17 起）

- 影子仿真由**计划任务** `XuegerosLocalSim` 托管（`pythonw.exe D:\11\Projects\xuegeros\tools\local_sim_launch.py`，
  无时间限制、允许电池供电）。从工具会话里用 `Win32_Process.Create` 或 `Start-Process` 直接起会被
  随调用结束回收（日志里表现为 `^C`），所以固定走计划任务。
- 重启：`Stop-ScheduledTask -TaskName XuegerosLocalSim; Start-ScheduledTask -TaskName XuegerosLocalSim`。
- 端口：HTTP `127.0.0.1:18142`、WS `127.0.0.1:18242`；**18202 是物理 robotd 口，必须保持关闭**。

## 10. 本机巡游收口（2026-11-18）

含轮轮廓、已观测自由格路径、分阶段有界恢复与 Coo 入口对齐后，带箱体房间已完成
`DONE(reached)`。恢复按实际位姿计量；每阶段的 `recovery_stage` 使用同一 cursor，
facts 带恢复 ID、阶段、实际后退与累计距离；失败不记作“已重规划成功”。
饱和统计约 1 Hz 采样，避免将三个控制 tick 当成三次独立观察。

运行方法、原始证据路径与局限见 [LOCAL-TOUR-DELIVERY.md](LOCAL-TOUR-DELIVERY.md)。
本节只收口本机影子仿真；guest、真车、SLAM 与真实传感器不在本次交付内。

## 11. 本机点选导航

`POST /api/strategy/start` 接收 `{"strategy":"navigate","target":[1.025,1.675]}`，坐标为 `map` 系中的 X/Y 米数，不使用场景变体。`GET /api/strategy` 的 `backend.navigation` 提供固定目标 `goal`、剩余折线 `route`、距目标 `distance_m`。进入 0.06 m 目标范围并停车时才计为到达。目标未观测、被占用、净空不足或无可通行路径时，任务进入 `BLOCKED` 并停止运动。

路线使用 `sim_truth_seen/v1` 的已观测空闲格，避开膨胀后的占据格；折线只在经检查后合并，不切未知区或障碍角。点选导航和巡游共用任务租约、净空检查与生命周期，仿真不向真车转发该模式。本机巡游也复用折线检查，保留可用路线；追踪记录分别提供短路标 `target_id` 和探索路线终点 `route_goal_id`。
