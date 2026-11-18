# 公开交付黑板（2026-11-18）

- npm cortico-world-xuegeros@0.3.3；Coo 已卸载旧链接、清空对应安装目录、从 registry 精确重装。
- 面板撤下自动图片，独立 3D 工作台保留；24 项参数与接口目录开放，支持 LAN。
- 新装 Coo 全巡游 DONE(reached)：164.4 s、6.967 m、known 14112、返航 0.18 m。
  新图暂停、恢复、互斥、停车稳定通过。实际 Core 连续重启两次，历史通知计数不增长。
- 公开源码重建 tgz 与 npm 发布包 SHA-256 一致；不含 STL 或含 STL 的本地提交历史。
- 来源与许可证见 ASSET-PROVENANCE.md；旧 YOLO 权重附 AGPL 文本，npm 不含权重。
- 使用与边界见 NPM-RELEASE.md；证据见 receipts/npm-release-20261118.json。
- 本机资产绝对路径：D:/11/Projects/xuegeros/sim/xuegeros_sim/meshes（仅本地）；
  场景 D:/11/Projects/xuegeros/sim/xuegeros_sim/worlds/room.world；
  模型 D:/11/Projects/xuegeros/sim/xuegeros_sim/urdf/xuegeros_car.urdf.xacro。
- 公开工作区 D:/11/Worktrees/xuegeros-public-032 从旧 origin/master 独立推进。
  本地完整 master 保留私有资产，禁止整段合并/推送到公开线。
- LAN 本机通配监听/地址发现已验；跨电脑入站未验，Windows 放行需管理员。
- 真实相机/超声/IMU/ROS 与接触动力学不在本轮交付范围。
