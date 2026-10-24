# XuegerOS 总汇报

日期：2026-10-22  
设备：`Maturo_E072A1B4F368` + `Maturo_CAM_1CDBD49799F4`  
服务器：`192.168.1.166`

## 结论

本轮完成了两套设备的接入、ROS2 建图、安全层改造和实体移动测试。

最终结果：

- 摄像头正常接入局域网。
- 机器人接入 Agent 并建立稳定 micro-ROS session。
- `/odom`、`/imu`、`/scan`、`/battery_state`、`/ultrasonic` 正常。
- Gmapping 正常运行，`/map` 和 TF 正常。
- 前进、后退、左转、右弧线均产生可验证的里程计变化。
- 移动命令经过 `twist_mux + 200 ms watchdog + lease`。
- 服务器测试运行态已清空。

当前服务器状态：

- VMware 虚拟机：已关闭
- `vmware-vmx`：无进程
- micro-ROS Agent：已停止
- ROS2、camera、bringup、mapping、safety：已停止
- `XuegerosUdpProxy`：任务已禁用
- UDP `8888/8889`：无监听
- VMware NAT 自定义 UDP 映射：已移除

项目配置、日志、脚本和记录保留，后续可以恢复。

## 设备与网络

| 项目 | 值 |
| --- | --- |
| 机器人设备 | `Maturo_E072A1B4F368` |
| 摄像头设备 | `Maturo_CAM_1CDBD49799F4` |
| 机器人局域网 IP | `192.168.1.106` |
| 摄像头 IP | `192.168.1.189` |
| 摄像头视频 | `http://192.168.1.189:81/` |
| 目标 Wi-Fi | `大肥鱼连这个` |
| Agent 地址 | `192.168.1.166:8888` |
| Agent guest 地址 | `192.168.253.128:8888` |
| ROS2 | Humble |

摄像头和机器人都在服务器同一有线局域网内，不需要让服务器长期连接目标
Wi-Fi。设备自身进入 STA 模式后即可直接访问。

## 接入过程

### 1. 摄像头接入

最初连接摄像头热点：

```text
Maturo CAM 1CDBD49799F4
```

设备 AP 配网页实际为：

```text
http://192.168.1.88/
```

随后摄像头配置到 `大肥鱼连这个`，进入 STA 模式并取得：

```text
192.168.1.189
```

MJPEG 视频和 ROS camera topic 均验证成功。

### 2. 机器人设备接入

服务器 USB 无线网卡直通给 VM。VM 连接：

```text
Maturo_E072A1B4F368
```

设备 AP 地址：

```text
192.168.4.1
```

读取到原通信配置：

```text
comm_mode: micro_ros
agent: 192.168.2.229:8888
```

修改为：

```text
comm_mode: micro_ros
agent: 192.168.1.166:8888
```

设备随后加入 `大肥鱼连这个`，获得有线局域网地址：

```text
192.168.1.106
```

USB Wi-Fi 在配网完成后已断开，VM 只保留 `ens33` 管理网。

### 3. micro-ROS 会话

设备重启后完成 Agent 握手，以下话题恢复：

```text
/odom
/imu
/scan
/battery_state
/ultrasonic
/cmd_vel
```

`/scan` 单一发布者，实测约 `9.8 Hz`。

### 4. 建图

启动：

```text
ros2 launch xuegecar_bringup xuegecar_bringup.launch.py
ros2 launch slam_gmapping slam_gmapping.launch.py
```

验证结果：

- `/tf` 和 `/tf_static` 正常
- TF 发布者数量为 3
- `/map` 正常发布
- 地图宽度 `384`
- `/scan -> /map` 链路正常

## micro-ROS UDP 稳定性修复

VMware NAT UDP 8888 在双向通信一段时间后不稳定，表现为：

- 机器人偶发丢失响应
- Agent 删除 micro-ROS objects
- `/odom`、`/scan`、`/cmd_vel` 订阅间歇消失

最终处理：

1. 移除 VMware NAT 的 8888 映射。
2. 增加服务器侧 UDP relay：

```text
192.168.1.166:8888 -> 192.168.253.128:8888
```

3. 使用 Windows 计划任务 `XuegerosUdpProxy` 托管。
4. 重启 Agent 和设备。
5. 验证 `/cmd_vel`、`/odom`、`/scan` 同时具有稳定端点。

最终清理时该任务已禁用。

## 安全层改造

### 维护 CLI

`tools/xuegeros.ps1` 已改为 key-only：

- 不再接受服务器或 guest 密码
- 使用 `id_loopros + dsh_vm_key`
- 使用 OpenSSH ProxyCommand
- 不再包含 `/cmd_vel` 发布动作

### 只读 Body Adapter

`tools/xuegeros_body_adapter.py`：

```text
GET /health
GET /v1/status
GET /v1/observations/latest
GET /v1/mapping
GET /v1/navigation
```

特点：

- 没有写入端点
- 不向 Persona 暴露原始高频流
- 返回有界状态和短期 JPEG 引用
- 始终报告 `autonomous_motion_allowed=false`

### Motion Gateway

`tools/xuegeros_motion_gateway.py`：

- 默认禁用
- 默认 shadow mode
- 需要人工 operator token
- 单一短期 lease
- 最长 lease `1000 ms`
- watchdog `200 ms`
- 无人值守时不会产生物理运动

### ROS 安全包

`ros2_safety/xuegeros_safety`：

- `twist_mux` 优先级
- 200 ms `/cmd_vel` watchdog
- 速度钳制
- `/motion/lease`
- `~/disarm`

数据路径：

```text
/cmd_vel_teleop
/cmd_vel_nav
/cmd_vel_autonomy
        |
        v
    twist_mux
        |
        v
/cmd_vel_safe_raw
        |
        v
cmd_vel_watchdog
        |
        v
     /cmd_vel
```

## 移动测试

### 第一次失败

电池未接入时：

- `/cmd_vel` 到达 ESP32
- `target_vx` 可变为 `50/100 mm/s`
- 轮速保持 `0`
- 电池仅约 `3.2 V / 0%`

结论：ESP32 逻辑由 USB 供电可运行，但电机驱动缺少主电池供电。

### 电池恢复后

电池状态：

```text
6.947-7.149 V
39-48%
```

实测运动：

| 测试 | 命令 | 结果 |
| --- | --- | --- |
| 前进 | `0.05 m/s × 0.5 s` | x 增加约 `0.0388 m` |
| 后退 | `-0.04 m/s × 0.5 s` | x 减少约 `0.00264 m` |
| 左转 | `0.2 rad/s × 0.6 s` | yaw 增加约 `0.0361 rad` |
| 右弧线 | `0.04 m/s + -0.1 rad/s × 0.5 s` | x 增加约 `0.0112 m`，yaw 减少约 `0.0186 rad` |

每次测试结果：

- 三层 `/cmd_vel` 路径均有非零消息
- lease 被接受
- watchdog 正常工作
- 测试结束后零速
- disarm 成功

最终设备状态：

```text
target_vx = 0
target_wz = 0
wheel velocity = 0 / 0
emergency_stop = false
motion_busy = false
battery = 7.149 V / 48%
```

## 剩余问题

### 1. 纯右转的 mux 异常

纯负角速度命令：

```text
linear.x = 0
angular.z = -0.2
```

输入端 `/cmd_vel_autonomy` 有消息，但两次测试均未出现在
`/cmd_vel_safe_raw`。

右弧线：

```text
linear.x = 0.04
angular.z = -0.1
```

正常通过。

因此不能把“纯右转”标记为完全通过。需要继续检查 `twist_mux` 的负角速度路径。

### 2. 固件无最终 deadman

当前 200 ms watchdog 位于 ROS2 侧。如果 VM、网络或 Agent 整体失效，固件仍可能保留最后速度。

生产接入前必须在固件内增加最终命令超时停车。

### 3. Guest NetworkManager 损坏

`libssh.so.4` 和部分系统包数据仍损坏。当前网络通过手动 DHCP 和 key-only
SSH 维护，不是重启持久化状态。

### 4. Nav2 未启动

建图和传感器图健康，但 Nav2 尚未启动。自主导航不是当前可用状态。

### 5. Persona 边界

PowerShell CLI 和维护能力不能暴露给 Persona。Persona 只能通过只读 Body
Adapter 和单独的 Tool Gateway 接入。

## 当前文件

```text
README.md
OPERATION_LOG.md
FINAL_REPORT.md
tools/xuegeros.ps1
tools/xuegeros_body_adapter.py
tools/xuegeros_motion_gateway.py
tools/xuegeros_motion_backend.py
tools/xuegeros_udp_proxy.ps1
ros2_safety/xuegeros_safety
```

## 恢复顺序

如果需要重新运行：

1. 启用并启动 `XuegerosUdpProxy`。
2. 启动 VM。
3. 恢复 guest 网络。
4. 启动 micro-ROS Agent。
5. 启动 bringup。
6. 启动 Gmapping。
7. 验证 `/scan`、`/odom`、`/map`、TF。
8. 启动 safety launch。
9. 验证 `/cmd_vel` 的 publisher/subscription。
10. 确认电池和轮子架空后再做运动测试。

## 最终判定

- 设备接入：完成
- 摄像头：完成
- micro-ROS：完成
- 建图：完成
- 安全链：完成
- 基础移动：完成
- 纯右转：仍有间歇异常
- 自主导航：未开放
- 服务器运行态：已清空
