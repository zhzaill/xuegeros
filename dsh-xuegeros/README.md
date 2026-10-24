# dsh-xuegeros

XuegerOS 小车在 DeepSeek Harness 里的驾驶舱：右侧栏一页 **小车**，实时相机 + YOLO
检测框 + 遥测 + 有界点动，同时把小车世界通过 Cordis 注入当前会话，让 Agent 用
`xuegeros_observe` / `xuegeros_drive` / `xuegeros_stop` 接管。

## 组成

| 文件 | 角色 |
|---|---|
| `lib/index.js` | Host 半侧：轮询 `127.0.0.1:18110/v1/world`，注册模型工具，托管 `/sync/xuegeros/*` 路由，按需拉起运动网关与 SSH 隧道；state 路由内嵌 **VLA 段**（NoMaD 服务健康 + 影子提案，读 `tmp/vla-shadow-log.jsonl` 尾部）；frame 代理带 **20s 空闲超时**（上游挂死不再冻结页面）。 |
| `lib/client.js` | Client 半侧：向 `sidebar.right.pane.tab` 注册 `xuegeros` 页，使用 DSH 主题 token，不碰宿主 DOM；相机 `<img>` 带**冻结看门狗**（8×8 canvas 校验和，10s 无变化自动重连——修 2026-10-23「画面不动」）；新增 **VLA 导航卡片**（服务/权重/延迟/最近提案，影子提示）。 |
| `cordis.patch.yml` | Bundle 补丁：把 `xuegeros` 行插进 Loader。 |

## 数据流

```text
Maturo 相机 (MJPEG 800x600)
        │  body adapter 拉流 + YOLO11n (CPU)
        ▼
/v1/world ──► Host 插件 ──► /sync/xuegeros/state ──► 右侧栏（1.5s 轮询）
   │                │
   │                └──► agent.inject()（节流，世界变化时；「接管」也走 inject——
   │                     禁用 agent.followup()：它在 desktop 0.2.0-rc.2 上会写入
   │                     缺 id/role 的 user/message 事件，DSH 重启后整个会话被判
   │                     corrupt 而消失，2026-10-23 因此丢过两个会话）
   │
   └──► xuegeros_observe / xuegeros_drive / xuegeros_stop（模型工具）

手动点动 / AI 脉冲
        ▼
Motion Gateway 127.0.0.1:18111（lease ≤1000ms、命令 ≤800ms、速度钳制）
        ▼（shadow 默认 / live 才走下一跳）
SSH 隧道 18200 ──► guest cmd_bridge ──► /cmd_vel_autonomy
                                         │ twist_mux (priority 20)
                                         ▼
                               /cmd_vel_safe_raw ──► watchdog ──► /cmd_vel
```

## 安全边界

- 网关默认 **shadow**：只校验 lease/限速/看门狗，不发布物理命令。
- 侧边栏里「允许 AI 运动」默认关闭；打开必须二次确认。
- 「物理模式」打开后才会建立 SSH 隧道并让网关转发到 guest bridge；关闭立即停车。
- 每条命令都是有界脉冲：`|linear| ≤ 0.08 m/s`、`|angular| ≤ 0.4 rad/s`、`100–800 ms`。
- Bridge 在 guest 侧再次校验 finite、范围、时长，并用 `/motion/lease` 与 watchdog 对齐。
- 相机原始帧只走本机回环代理给页面，不写盘、不进会话。

## 安装

```powershell
dsh plugin --profile desktop add link:D:/11/Projects/xuegeros/dsh-xuegeros
```

`dsh plugin` 会自动把包加进 profile 的 `dependencies` 与 `dsh.profile.bundles`。
首次安装后 **重启 DSH**，client bundle 才会进入启动图。

## 配置

`cordis.patch.yml` 里可覆盖：

```yaml
- id: xuegeros
  config:
    projectDir: D:\11\Projects\xuegeros
    adapterUrl: http://127.0.0.1:18110
    gatewayUrl: http://127.0.0.1:18111
```

> 旧版 `cordis.patch.yml` 里的路径必须保持单行 plain scalar，反斜杠原样保留。

## 运行前提

1. Body Adapter 在 `127.0.0.1:18110`（`tools/xuegeros_body_adapter.cmd`）。
2. 相机 `http://192.168.1.189:81/` 在线（地址在 `config/xuegeros.env.ps1`）。
3. 点「启动运动网关」后，`127.0.0.1:18111` 才上线；影子模式不需要 ROS。
4. 物理模式还需要：guest 上 `xuegeros_cmd_bridge.py` 在跑、`xuegeros_safety` 已启动。

**相机端口是独占的。** 该相机只接受一条视频会话；adapter 是它唯一的消费者
（持续拉流给 YOLO），再开浏览器 `:81` 会被相机重置。侧边栏的画面因此走
adapter 的回环预览 `GET /v1/preview.mjpeg`（`XUEGEROS_PREVIEW_ENABLE=1`，
`xuegeros_body_adapter.cmd` 已默认开启），不抢相机的第二个连接。

## YOLO 参数（实测）

模型 `yolo11n.pt`，CPU，`max_det=20`，检测框归一化为 `[x1,y1,x2,y2]`（0–1）。
同一帧参数扫描（800×600 输入，12 次平均）：

| imgsz | conf | 延迟 | 等效 FPS | 检测 |
|---|---|---|---|---|
| 256 | 0.35 | 74.1 ms | 13.5 | person 0.759 / 0.457 |
| 320 | 0.35 | 88.4 ms | 11.3 | person 0.764 / 0.486 |
| 416 | 0.35 | 115.0 ms | 8.7 | person 0.821 |

当前取 **320 / 0.35**：比 256 多保留一个小目标，比 416 快约 30%，FPS 仍在 10 以上。
相机断流或帧龄超过 3 秒时 `detections` 立即清空（fail closed），不会把陈旧框喂给界面或模型。
类别名是 COCO 英文（`person`、`cup`、`chair`…），中文映射由客户端负责。


## 验证记录（2026-10-23）

- `xuegeros_observe` 返回实时世界：`person`/`cup` 检测、`scan/map/tf` 健康、micro-ROS 会话在线。
- gateway 启动 → shadow 脉冲 `published=false`；切 live 后 `physical_backend_available=true`，
  零速脉冲经隧道到 bridge 返回 `published=true`，watchdog 日志出现 `motion lease accepted`。
- guest bridge：`/health` 200、无 token 命令 403、零速命令 `published_messages=4`。
- 相机曾于 16:12–17:18 掉线（STA 断连，IP 未变），恢复后 adapter 自动重连。
- 物理位移复测通过：`0.05 m/s × 300 ms`，odom x `0 → 0.0105156`（约 1.05 cm）。

## 自测：mock cordis context（2026-10-23）

```powershell
node tools/selftest.mjs
```

不需要真实 DSH、不需要 adapter、不需要 gateway；全部跑在一个一次性的
`DSH_HOME` 临时目录里，任何断言失败即非零退出。

覆盖 11 项：注册的 3 个模型工具与 7 条驾驶舱路由、状态文件与安全默认值、
`/state` 的 world 块（**要么是实时投影、要么是显式错误，绝不静默为空**）、
`ai_motion` 关闭时 `xuegeros_drive` 必须被拒、开启运动缺 confirm 必须 400、
无活跃会话时 summon 返回 `no_live_agent`、teardown 恰好回收 10 个注册、
**提前 teardown 不得泄漏路由**、启动清理陈旧 `liveMotion`（fail closed）而保留
`aiMotion`/token，以及 **summon 只走 `inject`、绝不调用 `followup`**。

三个曾经让这个自测说假话的坑，已修好并保留注释：

1. `mockReq` 原先只有测试主动调 `.settle()` 才会发 `data`/`end`。处理器 `await
   readBody(req)` 永不 resolve，事件循环排空后 node **以 0 退出**，后面的断言
   从未运行 —— 一次静默的假通过。现在请求体自动投递，并加了 60 s 看门狗与
   最终检查计数，任何挂起都必须是响亮的失败。
2. mock 的 `ctx.effect(cb)` 只存回调、不调用它。真实 cordis 会**立刻调用**回调
   并保留其返回的清理函数。原写法下 teardown 实际什么都没做，所有「回收了几个」
   的断言都无意义。现在按真实语义执行。
3. `agents.list()` 每次返回**新对象**，而插件每次选 agent 都重新 `list()` ——
   inject 落在临时对象上，断言读到的是 `undefined`。现在每个 agent 只包装一次、
   反复返回同一个对象。

另外 `gatewayUrl` 属于插件**配置**而非持久化状态，REST 探活也依赖机器上恰好跑没跑
网关；相关用例现在显式传 `{ gatewayUrl: 'http://127.0.0.1:9' }`，断言的才是
fail-closed 逻辑本身，而不是本机的偶然环境。

