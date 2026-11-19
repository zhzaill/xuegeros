# Coo 插件与轻量仿真发布

本文档记录 npm 0.3.5 的安装与验收，以及源码 0.3.6 的导航更新。Node.js 22+、Python 3.10+；不需要 ROS 或 Gazebo。

## 安装和启动

```sh
npm install -g cortico-world-xuegeros@0.3.5
xuegeros doctor
xuegeros sim
```

打开 http://127.0.0.1:18142/ 。地图 PNG 另需 `python -m pip install Pillow`。
Coo 扩展依赖中安装同名包并启用 xuegeros World。新装默认选择仿真。
在动作面板点「启动仿真工作台」，等待服务就绪后点「打开仿真工作台」。
这一入口能启动随包安装的 Python 仿真，也能复用已启动的仿真。已配置真车的旧安装可在设置中切换到 simulation。
默认 simulationHttpUrl=http://127.0.0.1:18142，simulationRobotdUrl=ws://127.0.0.1:18242/ws。
Coo 面板提供动作、巡游、原始状态与 24 项参数；3D 工作台独立打开，不再自动刷截图。
0.3.5 已移除 Coo 的旧「控制窗」「地图/雷达」「3D 模型」快捷入口。
旧链接安装应先卸载，再从 npm 安装；重启 Core 后重新打开设置窗口，更新版本化产物清单。

## 0.3.6 点选导航

在工作台展开「点选导航」，在场景地面选点或输入 X/Y 坐标，再点「前往目标」。路线只经过雷达已观测的空闲区域；目标不可达时停住并显示原因。到达判据为目标距离不超过 0.06 m。暂停、继续、取消与停车和巡游共用。

本机巡游保留可用路线，合并经过净空检查的路段，并为返航预留里程。带箱场景的对比窗口内，有效转向换向从 3 次降至 0 次；贴箱起步、绕箱导航、返航与停车均在影子仿真验证。

## 局域网

```sh
xuegeros sim --host 0.0.0.0
```

其他电脑访问 http://运行电脑IP:18142/ 。WebSocket 自动使用同一主机的 18242。
远端 Coo 的两个地址也改成运行电脑 IP。可用 --http-port / --port 换端口。
Windows 管理员终端执行 `xuegeros firewall`，放行 LocalSubnet 的 TCP 18142 和 18242。
本轮普通账户不能添加防火墙规则；已验证通配监听和本机 LAN IP 发现，尚未跨电脑验证入站。

## 参数与接口

GET /api/control 返回 schema 与 values；POST /api/control 接受参数名到数值的对象。
中控、小脑和工作台使用同一 schema：24 项，包含速度、净空、新鲜度与有界动作参数。
必须空闲才能修改；持有 goal、guide 或策略租约（含暂停）返回 409。
整批验证后原子落盘，默认 ~/.xuegeros/simulation.json，可由 --config 改路径。
NaN、bool、未知键与越界值拒绝；保留 Runner cursor，并失效旧参数的 offers。

GET /api/interfaces 列出完整 HTTP 路由与实际 WS 地址。
POST /api/command 复用 robotd 的命名命令，例如：

```json
{"t":"goal","goal":"forward","distance_m":0.1}
```

停车：`{"t":"stop"}`。动作仍统一经过 robotd、小脑和任务租约。
接口支持局域网可达；现有执行校验仍生效，没有原始速度旁路。

## 重启重复通知修复

0.3.3 为事件记录持久化送达凭据，在串行发送队列内去重。
本地历史回放与 robotd 连接补发不再重复通知；从旧 cursor 自动迁移。
身份包含 seq、源时间戳 ts 和 kind，因此 daemon 重启后复用 seq 不会吞掉新事件。
宿主发送失败仍可重试；宿主接收与落盘之间进程崩溃的窄窗口仍可能重试一次。
回归覆盖 16 条历史、重复补发、两次重启、新 daemon 同序号新事件和发送失败。

## 来源与源码发布线

npm 不含 STL、权重、凭据、日志或 Git 历史。默认使用程序化车模；有权使用的 CAD
可用 --mesh-dir 加载。本机计划任务优先使用 npm 安装的 Python runtime，外部 STL
仍从本地资产目录读取。源码发布线从旧 origin 独立推进，不包含私有研发分支的 CAD 历史。
旧 origin 已含 yolo11n.pt，随附 AGPL 文本和来源说明；npm 包不包含它。
Three.js r128 附上游 MIT 文本。详见 ASSET-PROVENANCE.md。

源码构建：在 coo-world-xuegeros 下运行 npm test、npm pack。
prepack 通过显式清单构建 runtime；公开源码包含此清单的全部源文件。

## 验收范围

完整记录见 receipts/npm-release-20261118.json。
干净包安装已实际执行 0.10 m 前进、按参数限速、停车后位姿稳定，Coo World 到 robotd
也执行 0.05 m。此为本机运动学仿真；不是接触动力学、SLAM 或真车验收。
真实相机、超声和 IMU 仍不可用；地图是 sim_truth_seen/v1。

registry 新装后实际 Coo 完整 tour：164.4 s / 6.967 m / 已观测 14112 格 / 返航 0.18 m，
DONE(reached)、acceptance.met=true；新图暂停、恢复、单步互斥与停车后位姿稳定通过。
