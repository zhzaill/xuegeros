# XuegerOS VLA 接入方案（替代 YOLO 决策链）

日期：2026-10-24 · 状态：**服务器侧已部署，权重已上线（missing_keys=0 验收通过）**

## 0. 一句话结论

**用 NoMaD 系（视觉导航模型，直接输出差速 (v, ω)）替换「YOLO 检测 → LLM 符号决策」这条链，
但 VLA 永远不直接碰 `/cmd_vel`**——它的提案走现有运动网关的同一套 lease/限幅/看门狗。
YOLO 先降级为独立安全观察者（person 近距否决），验证后再退役。

> **2026-10-24 部署修正**：发布版 `nomad.pth` 是 **EfficientNet-b0 编码器**（全模型仅
> **385 万参数**，服务器日志实测 `number of parameters: 3.849538e+06`），不是 ViT；
> 条件方式是 **goal-image（目标帧）**而非纯语言（官方 deployment 循环以拓扑图目标帧
> 为条件，语言版未随权重发布）。"语言目标"仍可由 LLM 从目标帧库中选择来近似。

## 1. 为什么是 VLA、替换的是什么

### 现状链路（2026-10-23 已验证）

```text
相机 MJPEG(独占会话) → body adapter(本机 CPU, YOLO11n 320px ~11-23FPS)
  → /v1/world(检测框+建图健康+电池) → DSH 插件 inject 世界简报
  → LLM 读简报 → xuegeros_drive(有界脉冲) → 网关 lease → 隧道 → bridge
  → twist_mux → watchdog → /cmd_vel
```

YOLO 在链里承担两个角色：**感知摘要**（给 LLM 的符号世界）与 **fail-closed 清框**。
决策本体是 LLM，靠文字简报闭环——每步延迟 = 观察 + 模型往返，指令语义粒度是"脉冲"。

### VLA 替换后的链路

```text
同一相机帧 ──┬─→ YOLO(过渡期保留：安全观察者，person 近距否决)
             └─→ VLA 推理服务(GPU)：帧 + 语言指令 → (v, ω) 提案流
                   → 运动网关（lease ≤1000ms、|v|≤0.08、|ω|≤0.4、看门狗）→ 同一条物理路径
LLM 的角色：下达语言目标("去门口"/"停")、监督、异常接管——不再是逐脉冲决策者
```

收益：闭环延迟从「LLM 往返」降到「推理一拍」；指令从脉冲变成目标；感知不再被 COCO 80 类绑架。

## 2. 选型（受硬件直接约束，已按实测排除）

| 硬件 | 事实 | 来源 |
| --- | --- | --- |
| 服务器 GPU | P104-100 **8 GB，Pascal sm_61**：无 fp8/fp4 内核，无 flash-attn2 | ComfyUI-Workspace/environment/server.md |
| 本机 GPU | RTX 5060（Blackwell）——但"本机不装"铁律目前只针对 ComfyUI，**是否允许本机跑 VLA 需用户拍板** | models.md L79 |
| 现有推理服务先例 | ComfyUI（GPU 常驻）、llama.cpp（4B q8 双槽 7.7 GB 可行） | server.md L196-197 |

排除与候选：

| 模型 | 判定 | 理由 |
| --- | --- | --- |
| OpenVLA (7B) | ❌ | 8 GB 装不下 bf16；sm_61 无量化内核；动作空间是机械臂 EE/joint，与本车不符 |
| π0 / π0-FAST (3B+) | ❌ | PaliGemma 底座依赖 flash-attn2（Ampere+）；8 GB Pascal 不可行 |
| Octo (93M) | ⚠️ 备选 | 够小，但动作头需按车微调，JAX 生态在服务器上链路成本高 |
| **NoMaD (EfficientNet-b0, 3.85M)** | ✅ **已部署** | **动作空间就是差速 (v, ω)**；goal-image 条件；96×96 输入；fp32 推理实测 p50 **164 ms**、显存 **85 MB**（P104-100） |
| SmolVLA (450M) | ✅ 二阶段 | LeRobot 生态、PyTorch 原生；用自己车的数据微调动作块；先 NoMaD 后再上 |

推理速度预估（Pascal ViT-B，未实测）：5–15 Hz。对 0.08 m/s 的有界脉冲足够。
验证方法：部署后用 `/system_stats` 同款探针测 `latency_ms` 分布（adapter 的 `inference_latency_ms` 字段已有先例）。

## 3. 部署形态：已落地（2026-10-24）

```text
[本机] body adapter ──/v1/preview.mjpeg(不消耗 YOLO 帧槽)──► [服务器 192.168.1.166]
                                                              E:\vla\nomad\vla_server.py
                                                              GET  /health   POST /goal  POST /infer  POST /reset
                                                              端口 18300，0.0.0.0（本机跨机实测可达）
```

服务器侧现状（全部已验证）：

| 项 | 值 |
| --- | --- |
| 位置 | `E:\vla\nomad\`（代码/权重槽位/日志），`E:\vla\pydeps\`（依赖隔离目录，未动 ComfyUI venv） |
| 解释器 | `E:\ComfyUI\venv\Scripts\python.exe`（torch 2.7.1+cu126 只读使用） |
| 依赖 | efficientnet_pytorch / einops / diffusers 0.34.0 / huggingface_hub 0.36.2（`--no-deps` 装入 pydeps；hub 2.x 需 httpx 故降级；diffusers 0.42 需 hub 2.x 故降级） |
| 保活 | `E:\vla\vla-ctl.ps1`，经 `Win32_Process.Create` 脱离 SSH 会话（直接 Start-Process 会随 plink 断开而死，已实测） |
| 性能 | fp32、CUDA：p50 **164 ms**/帧（首帧 ~4 s 为 CUDA 预热），显存 85 MB，约 6 Hz 闭环能力 |
| 权重 | `nomad.pth` **已部署**：76,473,631 B，sha256 `70f79b8262527e20e56ced64a3e3d7ef91855bc9e7c3fa348d78edcb83c6a333`，`missing_keys=0 / unexpected_keys=0`。下载通道：服务器 Cloudflare WARP（MASQUE，Always On）→ Google Drive，Content-Length 逐字节校验相等；同目录还有 gnm.pth / vint.pth 可备用 |
| 钳制 | 提案在服务内即钳制 v≤0.08、|w|≤0.4（随机权重冒烟已验证钳制路径生效） |

### nomad.pth 获取记录（已解决，2026-10-24 晚）

最初误判为"被墙不可达"——实际**服务器上的 Cloudflare WARP（MASQUE，Always On）可直达
Google**，且 UDP 8888 relay 不受影响。从 Drive 文件夹解析出文件 ID 后直接在服务器下载
（46% 断流一次，断点续传补齐，Content-Length 逐字节相等）。若日后需 gnm.pth / vint.pth，
同一通道同一文件夹。本地 `tools/vla-weights-deploy.ps1` 仍可用于从本机侧一键重部署。

## 4. 分阶段落地（每步可回退）

| 阶段 | 内容 | 退出判据 |
| --- | --- | --- |
| **P0 数据** | adapter 加录制器：2–5 Hz 存 (jpg, cmd_vel, odom, 指令) 成 LeRobot 兼容 episode；驾驶舱点动/agent 脉冲天然就是数据源 | 累计 ≥30 分钟有效驾驶片段 |
| **P1 影子** | vla_server 上线，adapter 只上报提案不执行；与实际动作并排落日志 | 提案-实动作方向一致率、轨迹误差有基线数字 |
| **P2 辅助** | 侧边栏「VLA 提案」模式：人按住 dead-man 才连续出脉冲；YOLO person 否决在线 | 架空+场地各跑通，异常一键停 |
| **P3 有界自主** | agent 下语言目标，VLA 闭环；LLM 只监督+可停 | YOLO 从决策链退役（可留否决） |

P1 期间零控制风险；P2 引入的每个新开关默认关，沿用 `ai_motion` 的二次确认交互。

## 5. 本机侧待接的文件（服务器侧已完成）

| 位置 | 改动 | 状态 |
| --- | --- | --- |
| `tools/vla_server.py` | 服务器推理服务（NoMaD 加载、/health、/goal、/infer、/reset） | ✅ 已部署 `E:\vla\nomad\`，项目留档 `tools/` |
| `tools/vla-ctl.ps1` / `tools/vla_smoke_test.py` | 启停与冒烟 | ✅ 已部署 `E:\vla\`，项目留档 `tools/` |
| `tools/vla_vendor/` | 上游代码闭包（vint_train + diffusion_policy 最小子集，MIT） | ✅ 已推服务器并留档 |
| `tools/xuegeros_body_adapter.py` | +`/v1/frames/latest.jpg` 单帧端点、+`vla_worker`、world 加 `vla` 段、+P0 录制器 | ⬜ 待做 |
| `dsh-xuegeros/lib/index.js` | +`xuegeros_vla_*` 工具/路由（goal/stop/mode），提案→有界脉冲转换 | ⬜ 待做 |
| `dsh-xuegeros/lib/client.js` | 侧边栏：VLA 模式开关（默认关+二次确认）、dead-man、提案可视化 | ⬜ 待做 |
| `config/xuegeros.env.ps1` | +`XUEGEROS_VLA_URL=http://192.168.1.166:18300`、`XUEGEROS_VLA_ENABLE` | ⬜ 待做 |

**帧源规则不变**：相机单视频会话归 adapter；本机侧只从 adapter 回环取帧转 POST `/infer`，
VLA 服务绝不直连 `192.168.1.189:81`。提案消费仍走运动网关（lease ≤1000ms、脉冲 ≤800ms），
bridge/watchdog/twist_mux 一行不改。任何一级失联 → 零速。

安全边界不变量（回归测试沿用 selftest 模式）：VLA 模式关 = 现状；开 = 仍受 ai_motion 开关、
网关 lease、速度/时长钳制、看门狗约束。

## 6. 未决（需要用户/后续会话拍板）

1. ~~nomad.pth 到手~~ ✅ 2026-10-24 晚经服务器 Cloudflare WARP 下载并验收（见 §3）。
2. **本机 RTX 5060 是否允许跑 VLA**（延迟更低，但违背"本机不装"的字面习惯——该铁律原文只限 ComfyUI）。
3. VLA 服务**开机自启**未配置（当前手动 `vla-ctl.ps1 start`；参考 llama/XuegerosUdpProxy 的计划任务模式需管理员）。
4. P2 的 dead-man 交互形态（按住按钮 vs 保持开关+超时）。
5. `/scan` 0.75 Hz 嫌疑在 ESP32 侧未解决——VLA 若要激光避障需先解决，否则 P2 起纯视觉。
