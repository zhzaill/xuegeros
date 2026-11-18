# 本机巡游交付（2026-11-18）

本轮收口范围：一个带箱体的房间，雷达观察、确定性巡游、受限脱困、返航、停车，
由 Coo 插件发起并读取原始结果。全部使用本机运动学影子仿真，`physical=false`。
备份完成、远端可复现、功能完成分别记录。后续 npm 发布与源码公开交付见 [NPM-RELEASE.md](NPM-RELEASE.md)；没有真车部署。

## 使用

本机已有计划任务 `XuegerosLocalSim`，启动后打开 <http://127.0.0.1:18142/>。
需要重启时在 PowerShell 执行：

```powershell
Stop-ScheduledTask -TaskName XuegerosLocalSim
Start-ScheduledTask -TaskName XuegerosLocalSim
```

干净副本不依赖这个计划任务，在副本根目录运行：

```powershell
python -m sim.local_sim --world room.world --http-port 18142 --port 18242
```

两种启动方式任选一种。Coo 已由旧链接扩展切换为 npm registry 精确版本安装；World 配置需为
`deployment=simulation`、`simulationRobotdUrl=ws://127.0.0.1:18242/ws`。
Coo 使用 `xuegeros_tour`：

```json
{"action":"start","variant_id":"room/default/01"}
```

同一工具支持 `status / pause / resume / cancel / stop`。start 明确选择 `tour`，
不再误用后端的 coverage 缺省。`xuegeros_stop` 随时停车；运行期间单步 goal 被租约拒绝。
完成事件 `tour_done` 给 Coo 返回原因、返航距离、实际里程、地图新增格与验收结果。
`DONE(stopped)` 是人工停车，`DONE(exhausted)` 是未满足验收，只有 `DONE(reached)` 表示本轮指标达标。

## 修复与边界

- 轮廓含轮：半长 72.5 mm、含轮半宽 81 mm，保守圆半径约 109 mm；
  雷达后偏 22.7 mm 也计入转向净空。地图占据几何与运动碰撞轮廓分离，避免重复膨胀。
- 已观测自由格上规划前沿接近路径和返航路径；未知格不作为可通行走廊。
  每秒重规划，地图由实际射线揭示；修正射线原点，并提高栅格采样密度。
- 恢复阶段：检查后方 → 实际后退到可转向位置 → 选宽出口 → 转向 → 短距离离开 → 重规划。
  后退仍受后方 0.30 m、累计 0.20 m、最多两次约束，按实际位姿计费。
  无位移、后方受阻、无宽出口、转向/离开时出现新障碍会停止，随后可进入 HANDOVER。
- Runner 继续独占租约、状态机、cursor 和验收；后端只交提案，唯一执行入口未改变。
  `backend.navigation.recovery` 与 `recovery_stage` 事件记录阶段和实际后退距离。
- 修复 tour 的里程输入遗漏、返航距离字段不一致、饱和采样频率和暂停误发完成事件。

这不是全场景导航能力。建图仍为 `sim_truth_seen/v1`，不是 SLAM；相机、超声、IMU 与
真实 ROS 路径继续未验证。没有提高速度、降低前向阈值或延长无进展包络。

## 验收与复现

回执汇总在 `D:\11\Projects\xuegeros\docs\receipts\local-tour-20261118.json`，原始轨迹绝对路径列在黑板。
实际 Coo → World → robotd → 执行层运行：167.9 s，`DONE(reached)`，已观测 14117 格，
里程 6.924 m，返航距离 0.180 m，验收 `met=true`。新地图上确认 RUNNING 后，
暂停/恢复、单步互斥、停车后五次间隔观察均通过；完成事件已在 Coo JSONL 找到同一 request_id。

当前界面已按用户反馈撤下自动刷新图片，“眼睛”不再周期闪烁；完整 3D 工作台独立打开。
Coo 面板保留动作、状态、按需截图与地图，并提供与工作台相同的 24 项中控/小脑参数。
早期 0.2.10 PNG 回执只保留为历史证据，不代表当前界面。安装、局域网与最新验收见
[NPM-RELEASE.md](NPM-RELEASE.md)。

确定性时钟使用相同的 `LocalSimulator.step` 与执行器，便于重复困难位姿；它与实时运行分开记账：

```powershell
python -m sim.probe.tour_regression --pose 0.72 0.86 0.99 --dt 0.05
python -m sim.probe.tour_regression --world room_open --dt 0.02
python -m sim.probe.coo_tour_acceptance
```

最后一条需要 Coo 与本机仿真已启动，会重置仿真场景并实际执行任务。
普通测试：

```powershell
python -m unittest discover -s agent_control/tests -t .
python -m unittest discover -s sim/tests
python -m unittest tools.test_xuegeros_adapters tools.test_xuegeros_ros_graph
python -m unittest discover -s mcp-xuegeros/tests
node --test coo-world-xuegeros/test/world.test.mjs coo-world-xuegeros/test/client.test.mjs coo-world-xuegeros/test/events.test.mjs
```

注意 `sim/tests` 没有包初始化文件，不能加 `-t .`。
后方缺测/受阻、无实际后退、窄出口、恢复中新障碍分别有故障安全单测；这些单测不冒充独立场景闭环。

## 资产与发布

- 网格目录：`D:\11\Projects\xuegeros\sim\xuegeros_sim\meshes`，来源清单 `SOURCE.md`。
- 场景：`D:\11\Projects\xuegeros\sim\xuegeros_sim\worlds\room.world`、`room_open.world`。
- URDF：`D:\11\Projects\xuegeros\sim\xuegeros_sim\urdf\xuegeros_car.urdf.xacro`。
- 插件：`D:\11\Projects\xuegeros\coo-world-xuegeros`。

STL 许可未确立；YOLO 来源与 AGPL 义务见 `docs/ASSET-PROVENANCE.md`。origin 仍是旧版本，
本轮干净副本复现指本地提交的克隆。不得将这次本机功能交付写成对外发布或远端复现完成。
