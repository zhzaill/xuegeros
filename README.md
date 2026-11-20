# XuegerOS：Coo 小车插件与轻量仿真

Coo 新装默认进入仿真。在动作面板点击「启动仿真工作台」，服务就绪后点击「打开仿真工作台」。需要 Python 3.10+。

独立本机仿真与 Coo World，Node.js 22+、Python 3.10+，无需 ROS/Gazebo。

```sh
npm install -g cortico-world-xuegeros@0.3.7
xuegeros doctor
xuegeros sim
```

打开 http://127.0.0.1:18142/ 。局域网用 `xuegeros sim --host 0.0.0.0`。
Windows 管理员终端 `xuegeros firewall` 放行本地子网的 18142/18242。
地图 PNG 可选依赖：`python -m pip install Pillow`。

Coo 扩展安装 `cortico-world-xuegeros@0.3.7`，启用 xuegeros，deployment=simulation。
面板保留动作、巡游、24 项中控/小脑参数；3D 工作台独立打开，撤掉自动刷新“眼睛”。
0.3.3 修复重启时历史“巡游结束”重复通知。

0.3.6 为本机仿真工作台增加「点选导航」，并减少巡游来回修正。地面选点或输入坐标后点「前往目标」，可查看路线、剩余距离及停车回执。

- [安装、接口、参数与局域网](docs/NPM-RELEASE.md)
- [仿真入口打不开](coo-world-xuegeros/README.md#仿真入口打不开)
- [插件开发说明](coo-world-xuegeros/README.md)
- [仿真契约](docs/SIM-CONTRACT.md)
- [本机巡游验收](docs/LOCAL-TOUR-DELIVERY.md)
- [资产来源与许可](docs/ASSET-PROVENANCE.md)

## 开发

```sh
python -m unittest discover -s agent_control/tests -t .
python -m unittest discover -s sim/tests
cd coo-world-xuegeros
npm test
npm pack
```

此公开发布线从旧 origin/master 独立推进，保留原有 Body Adapter/VLA 文件；
新增本机控制核心、仿真、插件与可重建 npm runtime。完整本地研发历史另行保留，
未将许可未确立的 STL 或含 STL 的提交合并到本公开历史。
公开包用程序化车模；本机有权使用的资产通过 --mesh-dir 指定。

这是运动学影子仿真。地图是 sim_truth_seen/v1，不是 SLAM；相机/超声/IMU
与真实 ROS 路径没有通过本轮验收。动作统一经 robotd、小脑和确定性任务状态机。

## 许可

新发布插件与本机运行源码（agent_control、sim、coo-world-xuegeros、
tools/enable_sim_lan.ps1）采用 [MIT](coo-world-xuegeros/LICENSE)。
Three.js 保留自己的 MIT 文本。此许可不覆盖历史第三方工具、CAD 或权重。
历史文件 yolo11n.pt 来自 Ultralytics，按上游 AGPL-3.0 记录，随附
[AGPL 文本](LICENSES/AGPL-3.0.txt) 与来源说明；npm 包不分发权重。
