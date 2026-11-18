# AGENTS · coo-world-xuegeros

这是 Coo/Cortico 的 XuegerOS World 扩展。入口：`src/definition.js`，控制逻辑在
`src/world.js`，协议在 `src/robotd_client.js`，事件持久化在 `src/events.js`。
本说明也覆盖 `src/webui/client.js` / `style.css` 与 `dist/webui.js`。
Also covers `bin/xuegeros.mjs` and `scripts/build-runtime.mjs`.
Also covers `src/workbench.js`. The panel invokes `startWorkbench` only in
simulation mode. It accepts the configured HTTP and WebSocket endpoints,
reuses running simulators, and launches the bundled Python module for
loopback endpoints through Python 3.10+. The created child belongs to the
World. World shutdown leaves independent CLI services alone. Test with
separate free ports and no simulator process, then assert HTTP and WS
readiness, simulator motion and teardown. Start LAN services from the CLI.

## 不能改的四条

1. **动作只走 robotd**：单步用 `goal`，房间巡游用 `tour`；`propose/execute`
   只给 AI 预检，控制台和 persona 都不走两步协商。不要在 World 里发速度或绕到旧网关。
2. **没有人工计时授权**。不要加回 `arm.json` 或“开 N 分钟”窗口；小脑的几何否决、
   传感器新鲜度和 watchdog 就是安全壳。物理开关只在车端启动参数里。
3. **只有源码事实事件唤醒人格**：`motion_vetoed` / `motion_blocked` /
   `command_failed` / `tour_done`。普通 `state` 与 50 Hz 流只落盘，不 `pushEvent`。
   不在这里写“网络坏了/车动不了/没放好”的分类结论——把 facts 交给 AI。
4. **token 是 World 专用的受限 token**，不复用 operator token，也不写进仓库。

## 验证

```powershell
node --test coo-world-xuegeros/test/world.test.mjs coo-world-xuegeros/test/client.test.mjs coo-world-xuegeros/test/events.test.mjs
```

车下线时用 `python -m agent_control.robotd --tcp --port 18202` 的合成世界；
真实部署见 `README.md` 的拓扑与 `docs/ROBOTD-CONSOLE.md`。

## 工具边界

- `xuegeros_tour` 是策略入口（start/stop/status），只返回 phase 与原始 facts。
- `xuegeros_map` / `xuegeros_screenshot` 只给可打开的 URL，不在 World 里重画或缓存图片。
- `xuegeros_ros` 原样透传 Body Adapter 的 `/v1/state` 与 `/v1/graph`。
- `xuegeros_source` 只读：白名单目录、单次 200 KB、拒绝路径穿越与凭据类文件。
  不要放宽成可写或全仓库可读。
- 面板不自动刷图；3D 工作台独立打开。按需截图与有界图片 RPC 仍保留。
  HTTP 地址由 `simulationHttpUrl` 配置，支持本机或局域网 HTTP origin。图片 PNG 上限 2 MB，
  禁重定向，不接受任意路径；不改 Electron 权限。宿主对版本化产物设置 immutable，
  修改已加载的 bundle/CSS 后必须递增包版本，重载 World 并重新打开设置窗口。
- `xuegeros_control` / 面板 `control` 调用同一仿真 `/api/control`，保存时需要空闲；
  `xuegeros_interfaces` 读取接口清单。全部 DEFAULTS 都列入 CONFIG_GROUP。
  构建：在本目录 `npm run build`；校验 `node --check src/webui/client.js`，再运行上面的测试。

## 已知

- 不支持 `wss://` 与原生 Unix socket；UDS 经 `ssh -L`。
- 本机仿真已在 Coo 加载；真车路径未验收。面板/截图不代表车载相机或 SLAM。

## Standalone npm runtime

`npm pack` runs prepack: build panel, then stage an explicit file allowlist under
`runtime/`. Generated runtime and tarballs are ignored by Git. Sources live in
the parent repository; installed users need no build. Never broaden staging to
copy the whole repository: CAD licenses are unresolved and weights are excluded.
Three.js r128 ships with its upstream MIT license. Package license is MIT.

CLI: `xuegeros doctor`, `xuegeros sim`, Windows administrator `xuegeros firewall`;
the firewall command runs the bundled `tools/enable_sim_lan.ps1` (requires elevation).
Node 22+, Python 3.10+. Python selection
can be overridden with XUEGEROS_PYTHON. No global pip installation is performed.
Pillow is optional; doctor prints a dependency command when missing. Simulation
forwards CLI arguments and exits on Ctrl+C. Config writes to the user's home,
not the installed package. Verify a fresh tgz install, custom ports, persistence,
actual bounded motion and stop. Release exact tested tarball, excluding secrets.
Keep npm auth only in the publishing process and remove temporary auth files.

Event delivery uses `.delivered.jsonl` receipts keyed by seq/ts/kind, migrated from the old cursor.
Both startup replay and server backlog check receipts inside the serialized delivery queue.
Host failures stay pending. Stop drains delivery before detaching the host. Test duplicate backlog,
two restarts, daemon sequence reuse and failed delivery via `npm test`. A crash between host
acceptance and receipt persistence can still cause one retry (no host idempotency API).
