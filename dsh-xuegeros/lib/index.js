/**
 * dsh-xuegeros — host half.
 *
 * Bridges the XuegerOS car body (127.0.0.1:18110) and the operator-only
 * motion gateway (127.0.0.1:18111) into the current DSH session:
 *
 *  - polls `/v1/world` (camera YOLO detections, battery, scan/map health),
 *  - injects a compact world brief into running agents through `agent.inject()`
 *    (throttled; never wakes an idle agent; also used by the summon route -
 *    `agent.followup()` is banned here, it corrupts the session store),
 *  - registers model-facing tools `xuegeros_observe` / `xuegeros_drive` /
 *    `xuegeros_stop`,
 *  - serves the sidebar cockpit: `/sync/xuegeros/{state,frame,move,stop,ai,summon}`.
 *
 * Motion is guarded twice: the cockpit must enable AI motion explicitly, and
 * the gateway itself stays in shadow mode unless a physical backend is
 * configured. `xuegeros_drive` never sends an unbounded command.
 */

import { promises as fs, createWriteStream } from 'node:fs';
import { dirname, join } from 'node:path';
import { homedir } from 'node:os';
import { randomBytes } from 'node:crypto';
import { spawn } from 'node:child_process';

export const name = 'dsh-xuegeros';
export const inject = ['tools'];

const DEFAULTS = {
  projectDir: 'D:\\11\\Projects\\xuegeros',
  adapterUrl: 'http://127.0.0.1:18110',
  gatewayUrl: 'http://127.0.0.1:18111',
  pollMs: 2000,
  injectIntervalMs: 15000,
  maxLinear: 0.08,
  maxAngular: 0.4,
  maxPulseMs: 800,
  shadowDefault: true,
};

const STATE_VERSION = 1;

function dshHome() {
  return process.env.DSH_HOME ?? join(homedir(), '.dsh');
}

function dataDir() {
  return join(dshHome(), 'data', 'dsh-xuegeros');
}

function stateFile() {
  return join(dataDir(), 'state.json');
}

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

async function readJson(file, fallback) {
  try {
    const text = await fs.readFile(file, 'utf8');
    return JSON.parse(text.replace(/^\uFEFF/, ''));
  } catch {
    return fallback;
  }
}

async function writeJson(file, value) {
  await fs.mkdir(dirname(file), { recursive: true });
  const tmp = `${file}.tmp`;
  await fs.writeFile(tmp, JSON.stringify(value, null, 2), { encoding: 'utf8' });
  await fs.rename(tmp, file);
}

function parseCameraUrlFromConfig(projectDir) {
  // Read-only peek at the existing config; the camera URL is not a secret.
  return fs
    .readFile(join(projectDir, 'config', 'xuegeros.env.ps1'), 'utf8')
    .then((text) => {
      const match = text.match(/\$env:XUEGEROS_CAMERA_URL\s*=\s*'([^']*)'/);
      return match ? match[1].trim() : '';
    })
    .catch(() => '');
}

function readBody(req, limit = 16 * 1024) {
  return new Promise((resolve, reject) => {
    let text = '';
    req.setEncoding('utf8');
    req.on('data', (chunk) => {
      text += chunk;
      if (text.length > limit) {
        reject(new Error('body too large'));
        req.destroy();
      }
    });
    req.on('end', () => resolve(text));
    req.on('error', reject);
  });
}

function sendJson(res, status, value) {
  const body = JSON.stringify(value);
  res.writeHead(status, {
    'content-type': 'application/json; charset=utf-8',
    'cache-control': 'no-store',
    'content-length': Buffer.byteLength(body),
  });
  res.end(body);
}

function shortError(error) {
  if (error === undefined || error === null) return 'unknown error';
  if (error.name === 'AbortError') return 'timeout';
  if (error.cause && error.cause.code) return `${error.message} (${error.cause.code})`;
  return String(error.message ?? error).slice(0, 240);
}

async function fetchJson(url, init = {}, timeoutMs = 4000) {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), timeoutMs);
  try {
    const response = await fetch(url, { ...init, signal: controller.signal });
    const text = await response.text();
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}: ${text.slice(0, 160)}`);
    }
    return text ? JSON.parse(text) : {};
  } finally {
    clearTimeout(timer);
  }
}

export function apply(ctx, entryConfig) {
  const config = { ...DEFAULTS, ...(entryConfig && typeof entryConfig === 'object' ? entryConfig : {}) };
  const log = (...args) => ctx.logger?.info?.('[xuegeros]', ...args);
  const warn = (...args) => ctx.logger?.warn?.('[xuegeros]', ...args);

  let state = {
    version: STATE_VERSION,
    aiMotion: false,
    injectWorld: false,
    operatorToken: '',
    liveMotion: false,
    gatewayStartedHere: false,
  };

  let world = null;
  let worldAt = 0;
  let worldError = null;
  let gateway = null;
  let gatewayAt = 0;
  let gatewayError = null;
  let gatewayChild = null;
  let lastInjectAt = 0;
  let lastInjectSig = '';
  let cameraFallback = '';
  let envConfig = null;
  let tunnelChild = null;

  const disposers = [];
  // The host can dispose this plugin before `stateReady` resolves (it loads
  // state from disk first). Routes and the poll timer are registered inside
  // that continuation, so without this flag teardown would splice an empty
  // array, then the continuation would mount 7 routes and a timer that never
  // unmount. On a plugin reload those handlers leak on the shared web server.
  let disposed = false;
  const trackDisposer = (fn) => {
    if (disposed) {
      try {
        fn();
      } catch {
        /* ignore */
      }
      return;
    }
    disposers.push(fn);
  };

  // --- persistence -------------------------------------------------------
  const stateReady = readJson(stateFile(), null).then((saved) => {
    if (saved && typeof saved === 'object') {
      state = { ...state, ...saved, version: STATE_VERSION };
      if (!state.operatorToken) state.operatorToken = randomBytes(18).toString('base64url');
    } else {
      state.operatorToken = randomBytes(18).toString('base64url');
    }
    return writeJson(stateFile(), state).catch((error) => warn('state write failed:', shortError(error)));
  });

  const persist = () => writeJson(stateFile(), state).catch((error) => warn('state write failed:', shortError(error)));

  async function readEnvConfig() {
    if (envConfig) return envConfig;
    const file = join(config.projectDir, 'config', 'xuegeros.env.ps1');
    let text = '';
    try {
      text = await fs.readFile(file, 'utf8');
    } catch {
      envConfig = {};
      return envConfig;
    }
    const out = {};
    for (const line of text.split(/\r?\n/)) {
      const match = line.match(/^\s*\$env:([A-Z0-9_]+)\s*=\s*(['"])(.*?)\2\s*$/);
      if (match) out[match[1]] = match[3].replace(/\$HOME/gi, homedir());
    }
    envConfig = out;
    return envConfig;
  }

  async function cameraUrl() {
    if (config.cameraUrl) return config.cameraUrl;
    if (process.env.XUEGEROS_CAMERA_URL) return process.env.XUEGEROS_CAMERA_URL;
    if (!cameraFallback) {
      const env = await readEnvConfig();
      cameraFallback = env.XUEGEROS_CAMERA_URL ?? '';
    }
    return cameraFallback;
  }

  async function bridgeToken() {
    const env = await readEnvConfig();
    return env.XUEGEROS_BRIDGE_TOKEN ?? '';
  }

  /** Keep one SSH tunnel: localhost:18200 -> guest cmd_bridge (ROS side). */
  async function ensureTunnel() {
    if (tunnelChild) return { ok: true, already: true };
    const env = await readEnvConfig();
    const serverHost = env.XUEGEROS_SERVER_HOST || '192.168.1.166';
    const serverUser = env.XUEGEROS_SERVER_USER || 'zhzaill';
    const serverKey = env.XUEGEROS_SERVER_KEY || join(homedir(), '.ssh', 'id_loopros');
    const guestIp = env.XUEGEROS_GUEST_IP || '192.168.253.128';
    const guestKey = env.XUEGEROS_GUEST_KEY || join(homedir(), '.ssh', 'dsh_vm_key');
    const proxy = `ssh.exe -i "${serverKey}" -o BatchMode=yes -o StrictHostKeyChecking=yes -W %h:%p ${serverUser}@${serverHost}`;
    const child = spawn(
      'ssh.exe',
      [
        '-N',
        '-L', '18200:127.0.0.1:18200',
        '-o', `ProxyCommand=${proxy}`,
        '-o', 'BatchMode=yes',
        '-o', 'ConnectTimeout=8',
        '-o', 'StrictHostKeyChecking=yes',
        '-o', 'ExitOnForwardFailure=yes',
        '-i', guestKey,
        `root@${guestIp}`,
      ],
      { windowsHide: true, stdio: ['ignore', 'ignore', 'pipe'] },
    );
    child.stderr.on('data', () => {});
    child.on('exit', () => {
      if (tunnelChild === child) tunnelChild = null;
    });
    tunnelChild = child;
    for (let i = 0; i < 20; i += 1) {
      await sleep(500);
      try {
        const health = await fetchJson('http://127.0.0.1:18200/health', {}, 1500);
        if (health && health.ok) return { ok: true, started: true };
      } catch {
        /* retry */
      }
    }
    stopTunnel();
    return { ok: false, error: 'tunnel_not_ready' };
  }

  function stopTunnel() {
    if (tunnelChild && tunnelChild.pid) {
      try {
        tunnelChild.kill();
      } catch {
        /* ignore */
      }
      tunnelChild = null;
    }
  }

  // --- world + gateway observation --------------------------------------
  async function refreshWorld(force = false) {
    const now = Date.now();
    if (!force && world && now - worldAt < config.pollMs) return world;
    try {
      world = await fetchJson(`${config.adapterUrl.replace(/\/$/, '')}/v1/world`, {}, 6000);
      worldError = null;
    } catch (error) {
      worldError = shortError(error);
    }
    worldAt = now;
    return world;
  }

  async function refreshGateway(force = false) {
    const now = Date.now();
    if (!force && gateway && now - gatewayAt < 1500) return gateway;
    try {
      gateway = await fetchJson(`${config.gatewayUrl.replace(/\/$/, '')}/v1/status`, {}, 1200);
      gatewayError = null;
    } catch (error) {
      gateway = null;
      gatewayError = shortError(error);
    }
    gatewayAt = now;
    return gateway;
  }

  // Prove the listening gateway actually honours our operator token.
  //
  // A state-changing call is the only honest probe: /v1/status is public by
  // design, so a gateway with a foreign token still answers it. The gateway has
  // no "who am I" endpoint, so probe with the most harmless authorized action -
  // a stop with zero motion. Stopping is idempotent and is the direction we
  // want to fail in anyway: if we cannot authorize a stop, the cockpit must
  // know that rather than believe it can halt the car.
  async function gatewayAcceptsOurToken() {
    try {
      await fetchJson(
        `${config.gatewayUrl.replace(/\/$/, '')}/v1/stop`,
        {
          method: 'POST',
          headers: gatewayHeaders(),
          body: JSON.stringify({ reason: 'cockpit authorization probe' }),
        },
        1500,
      );
      return true;
    } catch (error) {
      const text = String(error?.message ?? error);
      // 403 is the token verdict. Any other failure (timeout, 5xx) is not proof
      // of a token mismatch, so do not claim one.
      return !/403|operator_authorization_required/.test(text);
    }
  }

  function visionOf(snapshot) {
    return (snapshot && snapshot.vision) || null;
  }

  function batteryOf(snapshot) {
    const status = snapshot && snapshot.status;
    if (!status) return null;
    // The adapter currently exposes battery state only through the ROS graph;
    // keep this tolerant for future fields without inventing a value.
    return status.battery ?? status.battery_state ?? null;
  }

  function summarizeWorld(snapshot, detailed) {
    if (!snapshot) {
      return { ok: false, error: worldError ?? 'world_unavailable' };
    }
    const vision = visionOf(snapshot) ?? {};
    const mapping = snapshot.mapping ?? {};
    const status = snapshot.status ?? {};
    const navigation = snapshot.navigation ?? {};
    const detections = Array.isArray(vision.detections) ? vision.detections : [];
    const counts = new Map();
    for (const d of detections) {
      const key = String(d.class ?? 'object');
      counts.set(key, (counts.get(key) ?? 0) + 1);
    }
    const brief = {
      device_id: snapshot.device_id ?? 'xuegeros-car-01',
      camera: {
        state: vision.camera_state ?? 'unknown',
        fps: vision.inference_fps ?? null,
        latency_ms: vision.inference_latency_ms ?? null,
        frame_age_ms: vision.frame_age_ms ?? null,
        detections: detections.map((d) => ({
          class: d.class,
          confidence: d.confidence,
          bbox: d.bbox,
        })),
      },
      objects: Object.fromEntries(counts),
      mapping: {
        healthy: Boolean(mapping.mapping_healthy),
        scan: Boolean(mapping.scan?.present),
        map: Boolean(mapping.map?.present),
        tf: Boolean(mapping.tf?.present),
      },
      micro_ros: {
        agent: Boolean(status.micro_ros?.agent_running),
        session: Boolean(status.micro_ros?.session_observed),
      },
      navigation_ready: Boolean(navigation.navigation_ready),
      battery: batteryOf(snapshot),
      observed_at: snapshot.observed_at,
      autonomous_motion_allowed: Boolean(snapshot.autonomous_motion_allowed),
    };
    if (!detailed) return brief;
    return { ...brief, raw: snapshot };
  }

  function worldBriefText(snapshot) {
    const s = summarizeWorld(snapshot, false);
    if (!s.ok && s.error) return `【XuegerOS】车体世界不可用：${s.error}`;
    const objects = Object.entries(s.objects ?? {})
      .map(([k, v]) => `${k}×${v}`)
      .join('、');
    const bits = [
      `相机 ${s.camera.state}`,
      s.camera.fps ? `${s.camera.fps} FPS` : null,
      objects ? `目标：${objects}` : '画面中无识别目标',
      `建图 ${s.mapping.healthy ? '正常' : '不健康'}（scan ${s.mapping.scan ? '有' : '无'} / map ${s.mapping.map ? '有' : '无'} / TF ${s.mapping.tf ? '有' : '无'}）`,
      s.micro_ros.session ? 'micro-ROS 会话在线' : 'micro-ROS 会话缺失',
      gateway?.enabled
        ? gateway.shadow_mode
          ? '运动网关：影子模式（不驱动物理电机）'
          : '运动网关：物理模式'
        : '运动网关：未连接',
    ].filter(Boolean);
    return `【XuegerOS 世界状态】${bits.join('；')}。`;
  }

  // --- agents ------------------------------------------------------------
  function liveAgents() {
    const registry = ctx.get('agents');
    if (!registry || typeof registry.list !== 'function') return [];
    try {
      return registry.list() ?? [];
    } catch {
      return [];
    }
  }

  function pickAgent(sessionId) {
    const all = liveAgents();
    if (!all.length) return undefined;
    if (sessionId) {
      const exact = all.find((agent) => String(agent.id) === String(sessionId));
      if (exact) return exact;
    }
    const running = all.find((agent) => agent.status && agent.status !== 'idle');
    return running ?? all[0];
  }

  async function injectWorld(snapshot) {
    if (!state.injectWorld) return { ok: false, error: 'inject_disabled' };
    const agent = pickAgent();
    if (!agent || typeof agent.inject !== 'function') return { ok: false, error: 'no_live_agent' };
    const text = worldBriefText(snapshot);
    await agent.inject({
      content: [{ type: 'text', text }],
      source: { kind: 'plugin', plugin: 'dsh-xuegeros' },
    });
    return { ok: true, text };
  }

  async function summonAgent(snapshot, sessionId) {
    const agent = pickAgent(sessionId);
    if (!agent || typeof agent.inject !== 'function') {
      return { ok: false, error: 'no_live_agent' };
    }
    // Do NOT use agent.followup() here. On this DSH build (desktop 0.2.0-rc.2)
    // followup appends a `user/message` event without `id`/`role`; the session
    // loader then rejects the whole store on restart ("lacks an identified
    // message") and the session disappears. Two sessions were lost this way on
    // 2026-10-23. inject() produces a well-formed `agent/inbox/spliced` event
    // and is the only summon mechanism verified not to corrupt the store.
    // Caveat: inject splices into the next turn; it does not wake an idle
    // agent - the operator must nudge the conversation once.
    const text = [
      worldBriefText(snapshot),
      '你现在通过 DSH 侧边栏控制台接管这台 XuegerOS 小车。',
      '可用工具：xuegeros_observe 读取世界状态（YOLO 检测、建图、网关），xuegeros_drive 发送有界短脉冲（默认影子模式，需要人工在控制台打开「允许 AI 运动」），xuegeros_stop 立即停车。',
      '动作规则：先 observe，再以不超过 0.5 秒的单次脉冲移动；每步之间重新观察；遇到目标靠近、建图异常或网关影子模式时停止并说明。不要一次发送连续运动。',
    ].join('\n');
    await agent.inject({
      content: [{ type: 'text', text }],
      source: { kind: 'plugin', plugin: 'dsh-xuegeros' },
    });
    return { ok: true, session_id: agent.id, mechanism: 'inject', text };
  }

  // --- motion ------------------------------------------------------------
  function gatewayHeaders() {
    return {
      'X-Operator-Token': state.operatorToken,
      'content-type': 'application/json',
    };
  }

  function backendUrl() {
    return config.physicalBackendUrl ?? process.env.XUEGEROS_MOTION_BACKEND_URL ?? '';
  }

  async function startGateway() {
    await stateReady;
    const existing = await refreshGateway(true);
    if (existing) {
      // A gateway already listening is not necessarily OURS. If it was started
      // with a different operator token (a hand-run gateway, or one left over
      // from another profile), it answers /v1/status fine but rejects every
      // stop and move with 403 operator_authorization_required. Treating that
      // as "already started" left the cockpit permanently unable to stop the
      // car, with no way out from the UI. Probe authorization before claiming
      // success so the failure is named instead of silent.
      const authorized = await gatewayAcceptsOurToken();
      if (authorized) return { ok: true, already: true };
      // An unauthorized stop-safe call is still worth making: in shadow mode it
      // is inert, and if we cannot authorize we cannot vouch for motion state.
      warn('gateway on', config.gatewayUrl, 'rejected our operator token; it is not ours to drive');
      return {
        ok: false,
        error: 'gateway_token_mismatch',
        detail:
          'A motion gateway is listening but rejects this cockpit\'s operator token. ' +
          'Stop that process and start the gateway from the cockpit instead.',
      };
    }
    let backend = backendUrl();
    if (state.liveMotion && !backend) {
      const tunnel = await ensureTunnel();
      if (!tunnel.ok) return { ok: false, error: 'tunnel_failed', detail: tunnel };
      backend = 'http://127.0.0.1:18200';
    }
    const python = config.pythonPath ?? process.env.XUEGEROS_PYTHON ?? 'python';
    const script = join(config.projectDir, 'tools', 'xuegeros_motion_gateway.py');
    try {
      await fs.access(script);
    } catch {
      return { ok: false, error: 'gateway_script_missing', detail: script };
    }
    await fs.mkdir(dataDir(), { recursive: true });
    const logStream = createWriteStream(join(dataDir(), 'gateway.log'), { flags: 'a' });
    const child = spawn(python, [script], {
      cwd: config.projectDir,
      env: {
        ...process.env,
        XUEGEROS_MOTION_ENABLE: 'I_UNDERSTAND_PHYSICAL_RISK',
        XUEGEROS_MOTION_SHADOW: state.liveMotion ? '0' : '1',
        XUEGEROS_OPERATOR_TOKEN: state.operatorToken,
        XUEGEROS_BRIDGE_TOKEN: await bridgeToken(),
        ...(backend ? { XUEGEROS_MOTION_BACKEND_URL: backend } : {}),
      },
      windowsHide: true,
      stdio: ['ignore', 'pipe', 'pipe'],
    });
    child.stdout.pipe(logStream);
    child.stderr.pipe(logStream);
    child.on('exit', () => {
      if (gatewayChild === child) gatewayChild = null;
    });
    gatewayChild = child;
    state.gatewayStartedHere = true;
    await persist();
    for (let i = 0; i < 10; i += 1) {
      await sleep(300);
      if (await refreshGateway(true)) {
        return { ok: true, started: true, pid: child.pid, live: state.liveMotion };
      }
    }
    return { ok: false, error: 'gateway_did_not_start', pid: child.pid };
  }

  async function stopGateway() {
    const existing = await refreshGateway(true);
    // The gateway has no shutdown endpoint by design; only stop a process we
    // started ourselves, and only by its exact PID.
    if (gatewayChild && gatewayChild.pid) {
      try {
        gatewayChild.kill();
      } catch {
        /* ignore */
      }
      gatewayChild = null;
    }
    if (!state.liveMotion) stopTunnel();
    return { ok: true, already: !existing };
  }

  /** Re-spawn a self-owned gateway after the shadow/live choice changed. */
  async function restartGateway() {
    const existing = await refreshGateway(true);
    if (existing && gatewayChild && gatewayChild.pid) {
      try {
        gatewayChild.kill();
      } catch {
        /* ignore */
      }
      gatewayChild = null;
      for (let i = 0; i < 10; i += 1) {
        await sleep(200);
        if (!(await refreshGateway(true))) break;
      }
    }
    return startGateway();
  }

  async function drive({ linear, angular, durationMs, reason, targetSession }) {
    const lim = (value, max) => Math.max(-max, Math.min(max, Number(value) || 0));
    const lin = lim(linear, config.maxLinear);
    const ang = lim(angular, config.maxAngular);
    const duration = Math.max(100, Math.min(config.maxPulseMs, Math.round(durationMs || 300)));
    const gw = await refreshGateway(true);
    if (!gw) {
      return { ok: false, error: 'gateway_offline', detail: gatewayError ?? undefined };
    }
    if (!gw.enabled) {
      return { ok: false, error: 'motion_disabled', hint: 'start the gateway from the cockpit' };
    }
    let lease;
    try {
      lease = await fetchJson(
        `${config.gatewayUrl.replace(/\/$/, '')}/v1/leases`,
        {
          method: 'POST',
          headers: gatewayHeaders(),
          body: JSON.stringify({ duration_ms: Math.min(1000, duration + 250), reason: String(reason ?? 'xuegeros drive') }),
        },
        3000,
      );
    } catch (error) {
      return { ok: false, error: 'lease_failed', detail: shortError(error) };
    }
    if (!lease || lease.ok === false) {
      return { ok: false, error: lease?.error ?? 'lease_failed', detail: lease };
    }
    let command;
    try {
      command = await fetchJson(
        `${config.gatewayUrl.replace(/\/$/, '')}/v1/commands`,
        {
          method: 'POST',
          headers: gatewayHeaders(),
          body: JSON.stringify({ lease_id: lease.lease_id, linear: lin, angular: ang }),
        },
        3000,
      );
    } catch (error) {
      return { ok: false, error: 'command_failed', detail: shortError(error) };
    }
    await sleep(duration);
    await stop({ reason: 'pulse_complete' }).catch(() => {});
    return {
      ok: Boolean(command && command.ok),
      linear: lin,
      angular: ang,
      duration_ms: duration,
      shadow: Boolean(command?.shadow_mode ?? gw.shadow_mode),
      published: Boolean(command?.published),
      gateway: { enabled: gw.enabled, shadow_mode: gw.shadow_mode },
      note: command?.published ? 'physical backend accepted the pulse' : 'shadow mode: validated but not published',
    };
  }

  async function stop({ reason }) {
    const gw = await refreshGateway(true);
    if (!gw) return { ok: false, error: 'gateway_offline' };
    try {
      const result = await fetchJson(
        `${config.gatewayUrl.replace(/\/$/, '')}/v1/stop`,
        { method: 'POST', headers: gatewayHeaders(), body: JSON.stringify({ reason: String(reason ?? 'cockpit_stop') }) },
        2500,
      );
      return { ok: Boolean(result?.ok), result };
    } catch (error) {
      return { ok: false, error: shortError(error) };
    }
  }

  // --- tools -------------------------------------------------------------
  const toolRender = (value) => [{ type: 'text', text: typeof value === 'string' ? value : JSON.stringify(value) }];

  const toolObjects = [
    {
      name: 'xuegeros_observe',
      description:
        'Read the XuegerOS car world: camera state and YOLO detections, battery, scan/map/TF health, micro-ROS session, and motion-gateway mode. Call before and after any driving step.',
      parameters: {
        type: 'object',
        properties: {
          detail: { type: 'string', enum: ['brief', 'full'], description: 'brief (default) or full raw world projection' },
        },
      },
      output: { schema: { type: 'string' }, render: (_args, value) => toolRender(value) },
      async execute(args) {
        const snapshot = await refreshWorld(true);
        const summary = summarizeWorld(snapshot, args?.detail === 'full');
        return JSON.stringify(summary);
      },
    },
    {
      name: 'xuegeros_drive',
      description:
        'Pulse the XuegerOS car for a bounded time (100-800 ms). Requires the human cockpit to enable "allow AI motion"; the gateway may still be in shadow mode (validated, not published). Limits: |linear| <= 0.08 m/s, |angular| <= 0.4 rad/s.',
      parameters: {
        type: 'object',
        properties: {
          linear: { type: 'number', description: 'Forward velocity in m/s; negative moves backward' },
          angular: { type: 'number', description: 'Yaw rate in rad/s; positive turns left' },
          duration_ms: { type: 'integer', description: 'Pulse length in ms (100-800)' },
          reason: { type: 'string', description: 'Short human-readable purpose of this pulse' },
        },
        required: ['duration_ms'],
      },
      output: { schema: { type: 'string' }, render: (_args, value) => toolRender(value) },
      async execute(args) {
        if (!state.aiMotion) {
          return JSON.stringify({
            ok: false,
            error: 'ai_motion_disabled',
            hint: 'The human operator must enable AI motion in the XuegerOS sidebar first.',
          });
        }
        const result = await drive({
          linear: args?.linear ?? 0,
          angular: args?.angular ?? 0,
          durationMs: args?.duration_ms ?? 300,
          reason: args?.reason ?? 'agent drive',
        });
        return JSON.stringify(result);
      },
    },
    {
      name: 'xuegeros_stop',
      description: 'Stop the XuegerOS car immediately (clears the motion lease; publishes zero velocity when a physical backend is connected).',
      parameters: { type: 'object', properties: { reason: { type: 'string' } } },
      output: { schema: { type: 'string' }, render: (_args, value) => toolRender(value) },
      async execute(args) {
        return JSON.stringify(await stop({ reason: args?.reason ?? 'agent stop' }));
      },
    },
  ];

  const unregister = [];
  try {
    for (const definition of toolObjects) {
      const dispose = ctx.tools.register(definition);
      if (typeof dispose === 'function') unregister.push(dispose);
    }
  } catch (error) {
    warn('tool registration failed:', shortError(error));
  }
  trackDisposer(() => {
    for (const dispose of unregister.splice(0)) {
      try {
        dispose();
      } catch {
        /* ignore */
      }
    }
  });

  // --- polling loop ------------------------------------------------------
  const pollTimer = setInterval(async () => {
    try {
      const snapshot = await refreshWorld();
      await refreshGateway();
      if (!snapshot || !state.injectWorld) return;
      const summary = summarizeWorld(snapshot, false);
      const signature = JSON.stringify([summary.objects, summary.mapping, summary.micro_ros, summary.camera.state, gateway?.shadow_mode ?? null]);
      const now = Date.now();
      if (signature === lastInjectSig) return;
      if (now - lastInjectAt < config.injectIntervalMs) return;
      const running = liveAgents().filter((agent) => agent.status && agent.status !== 'idle');
      if (!running.length) return;
      lastInjectAt = now;
      lastInjectSig = signature;
      await injectWorld(snapshot);
    } catch (error) {
      log('poll tick failed:', shortError(error));
    }
  }, config.pollMs);
  pollTimer.unref?.();

  trackDisposer(() => clearInterval(pollTimer));

  // --- cockpit HTTP routes ----------------------------------------------
  const mount = () => {
    const webServer = ctx.get('webServer') ?? ctx.get('webserver');
    if (!webServer || typeof webServer.register !== 'function') {
      warn('web server service unavailable; cockpit routes not mounted');
      return;
    }
    const routes = [];

    /** Proxy one MJPEG stream; report whether any header was already sent. */
    const proxyStream = async (req, res, url) => {
      const controller = new AbortController();
      req.on('close', () => controller.abort());
      try {
        const upstream = await fetch(url, { signal: controller.signal });
        if (!upstream.ok || !upstream.body) {
          return { ok: false, started: false, status: upstream.status, error: `HTTP ${upstream.status}` };
        }
        res.writeHead(200, {
          'content-type': upstream.headers.get('content-type') || 'multipart/x-mixed-replace; boundary=frame',
          'cache-control': 'no-store',
          connection: 'close',
        });
        const reader = upstream.body.getReader();
        for (;;) {
          const { done, value } = await reader.read();
          if (done) break;
          if (!res.write(Buffer.from(value))) {
            await new Promise((resolve) => res.once('drain', resolve));
          }
        }
        res.end();
        return { ok: true, started: true };
      } catch (error) {
        if (!res.headersSent) return { ok: false, started: false, error: shortError(error) };
        try {
          res.end();
        } catch {
          /* ignore */
        }
        return { ok: false, started: true, error: shortError(error) };
      }
    };

    const stateRoute = {
      kind: 'exact',
      path: '/sync/xuegeros/state',
      handler: async (req, res) => {
        if (req.method !== 'GET') return sendJson(res, 405, { ok: false, error: 'method_not_allowed' });
        const snapshot = await refreshWorld();
        const gw = await refreshGateway();
        sendJson(res, 200, {
          ok: true,
          device_id: snapshot?.device_id ?? 'xuegeros-car-01',
          world: summarizeWorld(snapshot, true),
          gateway: gw ?? { online: false, error: gatewayError },
          control: {
            ai_motion: state.aiMotion,
            inject_world: state.injectWorld,
            live_motion: state.liveMotion,
            shadow: gw ? Boolean(gw.shadow_mode) : true,
            camera_proxy: '/sync/xuegeros/frame',
          },
          agents: liveAgents().map((agent) => ({ id: agent.id, status: agent.status ?? null })),
        });
      },
    };

    const frameRoute = {
      kind: 'exact',
      path: '/sync/xuegeros/frame',
      handler: async (req, res) => {
        if (req.method !== 'GET') return sendJson(res, 405, { ok: false, error: 'method_not_allowed' });

        // The camera tolerates only a small number of video sessions, so the
        // adapter stays its single consumer and the cockpit reads the adapter's
        // loopback preview. Direct camera streaming remains a fallback for
        // hosts whose adapter predates the preview endpoint.
        const preview = `${config.adapterUrl.replace(/\/$/, '')}/v1/preview.mjpeg`;
        const first = await proxyStream(req, res, preview);
        if (first.ok || first.started) return;

        // Only an adapter that predates the preview endpoint (404) justifies
        // touching the camera directly. When the adapter is merely down, do not
        // open a second camera session: the browser would then hold the camera's
        // single video slot and starve the adapter after it restarts.
        if (first.status !== 404) {
          res.writeHead(503, { 'content-type': 'text/plain; charset=utf-8' });
          res.end(`camera preview unavailable: ${first.error ?? 'adapter offline'}`);
          return;
        }

        const url = await cameraUrl();
        if (!url) {
          res.writeHead(503, { 'content-type': 'text/plain; charset=utf-8' });
          res.end('camera preview unavailable; camera url is not configured');
          return;
        }
        const second = await proxyStream(req, res, url);
        if (!second.ok && !second.started) {
          res.writeHead(503, { 'content-type': 'text/plain; charset=utf-8' });
          res.end(`camera stream failed: ${second.error ?? 'unknown'}`);
        }
      },
    };

    const moveRoute = {
      kind: 'exact',
      path: '/sync/xuegeros/move',
      handler: async (req, res) => {
        if (req.method !== 'POST') return sendJson(res, 405, { ok: false, error: 'method_not_allowed' });
        try {
          const body = JSON.parse((await readBody(req)) || '{}');
          const result = await drive({
            linear: Number(body.linear ?? 0),
            angular: Number(body.angular ?? 0),
            durationMs: Number(body.duration_ms ?? 300),
            reason: String(body.reason ?? 'cockpit move'),
          });
          return sendJson(res, 200, result);
        } catch (error) {
          return sendJson(res, 400, { ok: false, error: shortError(error) });
        }
      },
    };

    const stopRoute = {
      kind: 'exact',
      path: '/sync/xuegeros/stop',
      handler: async (req, res) => {
        if (req.method !== 'POST') return sendJson(res, 405, { ok: false, error: 'method_not_allowed' });
        let reason = 'cockpit stop';
        try {
          const body = JSON.parse((await readBody(req)) || '{}');
          if (body.reason) reason = String(body.reason);
        } catch {
          /* keep default */
        }
        return sendJson(res, 200, await stop({ reason }));
      },
    };

    const aiRoute = {
      kind: 'exact',
      path: '/sync/xuegeros/ai',
      handler: async (req, res) => {
        if (req.method !== 'POST') return sendJson(res, 405, { ok: false, error: 'method_not_allowed' });
        let body;
        try {
          body = JSON.parse((await readBody(req)) || '{}');
        } catch (error) {
          return sendJson(res, 400, { ok: false, error: shortError(error) });
        }
        if (typeof body.motion === 'boolean') {
          if (body.motion && body.confirm !== true) {
            return sendJson(res, 400, { ok: false, error: 'confirmation_required' });
          }
          state.aiMotion = body.motion;
          if (!body.motion) await stop({ reason: 'ai_motion_disabled' }).catch(() => {});
        }
        if (typeof body.inject === 'boolean') state.injectWorld = body.inject;
        let gatewayResult = null;
        if (typeof body.live === 'boolean' && body.live !== state.liveMotion) {
          const wasOnline = Boolean(await refreshGateway(true));
          await stop({ reason: 'live_motion_changed' }).catch(() => {});
          state.liveMotion = body.live;
          await persist();
          if (wasOnline || gatewayChild) gatewayResult = await restartGateway();
        }
        await persist();
        return sendJson(res, 200, {
          ok: true,
          control: { ai_motion: state.aiMotion, inject_world: state.injectWorld, live_motion: state.liveMotion },
          gateway: gatewayResult,
        });
      },
    };

    const summonRoute = {
      kind: 'exact',
      path: '/sync/xuegeros/summon',
      handler: async (req, res) => {
        if (req.method !== 'POST') return sendJson(res, 405, { ok: false, error: 'method_not_allowed' });
        let body = {};
        try {
          body = JSON.parse((await readBody(req)) || '{}');
        } catch {
          /* default */
        }
        const snapshot = await refreshWorld(true);
        try {
          return sendJson(res, 200, await summonAgent(snapshot, body.session_id));
        } catch (error) {
          return sendJson(res, 500, { ok: false, error: shortError(error) });
        }
      },
    };

    const gatewayRoute = {
      kind: 'exact',
      path: '/sync/xuegeros/gateway/start',
      handler: async (req, res) => {
        if (req.method !== 'POST') return sendJson(res, 405, { ok: false, error: 'method_not_allowed' });
        return sendJson(res, 200, await startGateway());
      },
    };

    for (const route of [stateRoute, frameRoute, moveRoute, stopRoute, aiRoute, summonRoute, gatewayRoute]) {
      try {
        const dispose = webServer.register(route);
        if (typeof dispose === 'function') routes.push(dispose);
      } catch (error) {
        warn(`route ${route.path} not mounted:`, shortError(error));
      }
    }
    trackDisposer(() => {
      for (const dispose of routes.splice(0)) {
        try {
          dispose();
        } catch {
          /* ignore */
        }
      }
    });
  };

  stateReady.then(async () => {
    // Teardown may already have run while state was loading. Registering
    // routes or starting refreshes now would leak both past disposal.
    if (disposed) return;
    // Physical mode cannot survive a host restart. `liveMotion` and
    // `gatewayStartedHere` describe a gateway *process*; when DSH exits that
    // process dies with it, but the persisted flag does not. Left alone, the
    // cockpit would render "physical mode on" while the gateway it talks to is
    // a shadow-mode one that publishes nothing - a safety-relevant lie.
    //
    // Fail closed: clear the claim unless a live gateway is actually reachable
    // AND enforcing physical mode.
    if (state.liveMotion) {
      const existing = await refreshGateway(true);
      const liveBacked = Boolean(existing && existing.shadow_mode === false);
      if (!liveBacked) {
        state.liveMotion = false;
        state.gatewayStartedHere = false;
        await persist();
        log('cleared stale live_motion: no physical gateway survived the restart');
      }
    } else {
      state.gatewayStartedHere = false;
      await persist();
    }

    mount();
    refreshWorld(true).catch(() => {});
    refreshGateway(true).catch(() => {});
    log('cockpit ready at /sync/xuegeros/*');
  });

  ctx.effect(() => () => {
    // Set before draining: any continuation still pending on `stateReady`
    // must not register new routes or timers after this point.
    disposed = true;
    for (const dispose of disposers.splice(0)) {
      try {
        dispose();
      } catch {
        /* ignore */
      }
    }
    if (gatewayChild && gatewayChild.pid && state.gatewayStartedHere) {
      try {
        gatewayChild.kill();
      } catch {
        /* ignore */
      }
      gatewayChild = null;
    }
    stopTunnel();
  }, 'dsh-xuegeros: teardown');
}
