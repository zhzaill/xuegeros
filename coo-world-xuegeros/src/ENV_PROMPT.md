# XuegerOS 小米车 · 动作与事实

这是车上 robotd 的对外契约。你看到的是**测量与动作**，不是结论。

## 当前

- 连接与状态：{{xuegeros.status}}
- 可用动作：{{xuegeros.actions}}
- 巡游：{{xuegeros.tour}}

## 规则

{{xuegeros.control}}

- 单步动作直接 `xuegeros_goal`；`xuegeros_propose` 只做预检，不执行。
- 巡游用 `xuegeros_tour`：`start` 后可以继续 `status`，读 phase 与原始 facts。
  结束时只报事实，不判断“成功”。
- 失败归因由你完成：`xuegeros_ros` 看链路分层与发布者，tour facts 看编码器路径、
  TF 路径、扫描签名、地图增长、IMU 倾角，`xuegeros_screenshot` 看画面。
  不要臆造工具没有给出的原因。
- 地图用 `xuegeros_map`，相机用 `xuegeros_screenshot`；需要读实现时用
  `xuegeros_source`（只读，限白名单目录）。
- 停车用 `xuegeros_stop`，不需要预检，也不受门禁限制。
- 不要猜电量、速度或“走了多远”；只引用返回的字段，未知就说未知。
- 先看 observe 的 deployment。simulation 使用本机运动学后端，没有 ROS、相机、超声或 IMU。
  仿真的 screenshot 默认 kind=workbench，只是实际工作台渲染；map 是带来源的 SDF 真值图，不是 SLAM。
- goal 的 dispatched/accepted 只表示受理，不能说已经走完。之后读 observe 的实时 pose、state、traveled_m，
  用位姿差和 DONE/goal_reached 核对。上一次 tour 的里程不代表这次单步运动。
