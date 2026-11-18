import os from 'node:os';
import { existsSync } from 'node:fs';
import path from 'node:path';
import fs from 'node:fs/promises';
import { fileURLToPath } from 'node:url';

import { CONFIG_GROUP, DEFAULTS, ID, LABEL, WAKE_KINDS, WAKE_STATES } from './config.js';
import { EventLog } from './events.js';
import { RobotdClient } from './robotd_client.js';

const ENV_PROMPT_FILE = fileURLToPath(new URL('./ENV_PROMPT.md', import.meta.url));
const BUNDLED_ROOT = fileURLToPath(new URL('../runtime', import.meta.url));
const CHECKOUT_ROOT = fileURLToPath(new URL('../..', import.meta.url));
const REPO_ROOT = existsSync(path.join(CHECKOUT_ROOT, 'agent_control')) ? CHECKOUT_ROOT : BUNDLED_ROOT;
const SOURCE_PREFIXES = [
  'agent_control',
  'sim',
  'coo-world-xuegeros/src',
  'docs',
  'ros2_safety/xuegeros_safety',
  'tools',
];
const SOURCE_DENY = [
  /(^|\/)access(\/|$)/i,
  /\.(pem|key|token)$/i,
  /(^|\/)\.env/i,
  /(^|\/)\.git(\/|$)/,
  /(^|\/)node_modules(\/|$)/,
  /(^|\/)logs?(\/|$)/i,
  /secret|credential/i,
];
const SOURCE_MAX_BYTES = 200_000;

function corticoTool(handler) {
  return async (args) => {
    const result = await handler(args ?? {});
    return { text: typeof result === 'string' ? result : JSON.stringify(result ?? null) };
  };
}

function finite(value) {
  return typeof value === 'number' && Number.isFinite(value);
}

export class XuegerosWorld {
  id = ID;

  constructor(cfg, ctx = {}, deps = {}) {
    this.cfg = { ...DEFAULTS, ...(cfg && typeof cfg === 'object' ? cfg : {}) };
    this.ctx = ctx && typeof ctx === 'object' ? ctx : {};
    this.deps = deps && typeof deps === 'object' ? deps : {};
    this.host = null;
    this.client = null;
    this.log = null;
    this.state = null;
    this.heartbeat = null;
    this.lastHeartbeatAt = 0;
    this.lastError = null;
    this.lastStateAt = 0;
    this.eventsSeen = 0;
    this.stopping = false;
    this._wasStale = null;
    this._delivery = Promise.resolve();
  }

  // --- lifecycle -----------------------------------------------------------
  async start(host) {
    this.host = host;
    this.stopping = false;
    this.log = this.deps.createLog
      ? this.deps.createLog(this.logPath(), { maxReplay: this.cfg.maxReplay })
      : new EventLog(this.logPath(), { maxReplay: this.cfg.maxReplay });
    const clientOptions = {
      url: this.robotdUrl(),
      token: this.cfg.robotdToken,
      requestTimeoutMs: this.cfg.requestTimeoutMs,
    };
    this.client = this.deps.createClient ? this.deps.createClient(clientOptions) : new RobotdClient(clientOptions);
    this.client.on('state', (frame) => {
      this.state = frame;
      this.lastStateAt = Date.now();
      this.lastError = null;
    });
    this.client.on('open', () => {
      host?.log?.info?.('[xuegeros] robotd connected');
      this.replayMissed();
    });
    this.client.on('close', () => {
      host?.log?.warn?.('[xuegeros] robotd disconnected');
    });
    this.client.on('error', (error) => {
      this.lastError = String(error?.message || error);
    });
    this.client.on('frame', (frame) => {
      if (frame?.t === 'events') this.onEvents(frame.items || []);
      else if (frame?.t === 'heartbeat') this.onHeartbeat(frame);
    });
    this.client.start();
    // A failed connection must not unmount the World: it stays degraded and
    // reconnects, which is what the operator asked for.
  }

  robotdUrl() {
    return this.cfg.deployment === 'simulation'
      ? this.cfg.simulationRobotdUrl
      : this.cfg.robotdUrl;
  }

  async stop() {
    this.stopping = true;
    this.client?.stop();
    await this._delivery;
    this.client = null;
    this.host = null;
  }

  logPath() {
    const configured = String(this.cfg.eventLogPath || 'xuegeros-events.jsonl');
    if (path.isAbsolute(configured)) return configured;
    const base = this.ctx.dataDir || path.join(os.homedir(), '.xuegeros');
    return path.join(base, configured);
  }

  // --- events --------------------------------------------------------------
  onEvents(items) {
    if (!Array.isArray(items) || items.length === 0) return;
    items = items.filter((event) => event && typeof event === 'object');
    for (const event of items) {
      this.eventsSeen += 1;
      try {
        this.log?.append(event);
      } catch (error) {
        this.host?.log?.warn?.(`[xuegeros] event log append failed: ${error?.message ?? error}`);
      }
    }
    const waking = items.filter((event) => this.wakeReason(event));
    if (waking.length) this._delivery = this._delivery.then(() => this.deliver(waking));
  }

  onHeartbeat(frame) {
    this.heartbeat = frame;
    this.lastHeartbeatAt = Date.now();
    const stale = Boolean(frame.stale);
    if (this._wasStale === null) {
      this._wasStale = stale;
      return;
    }
    if (stale !== this._wasStale) {
      this._wasStale = stale;
      const text = stale
        ? `传感器链路降级：scan ${frame.scan_age_s ?? '未知'}/${frame.scan_limit_s ?? '?'} s，` +
          `odom ${frame.odom_age_s ?? '未知'}/${frame.odom_limit_s ?? '?'} s。先恢复 /scan、/odom。`
        : `传感器链路恢复：scan ${frame.scan_age_s ?? '未知'} s，odom ${frame.odom_age_s ?? '未知'} s。`;
      const payload = { type: 'xuegeros.fault', text, meta: { stale, scan_age_s: frame.scan_age_s } };
      this._delivery = this._delivery.then(async () => {
        try {
          if (this.host?.pushEvent) await this.host.pushEvent(payload, { trigger: 'flush' });
        } catch (error) {
          this.host?.log?.warn?.(`[xuegeros] fault push failed: ${error?.message ?? error}`);
        }
      });
    }
  }

  async replayMissed() {
    if (!this.log) return;
    const missed = this.log.pending().filter((event) => this.wakeReason(event));
    if (missed.length) this._delivery = this._delivery.then(() => this.deliver(missed));
  }

  wakeReason(event) {
    if (!event || typeof event !== 'object') return null;
    if (WAKE_KINDS.has(event.kind)) return event.kind;
    if (event.kind === 'state' && WAKE_STATES.has(event.state)) return `state:${event.state}`;
    return null;
  }

  async deliver(events) {
    for (const event of events) {
      if (this.stopping) return;
      const reason = this.wakeReason(event);
      if (!reason) continue;
      if (this.log?.wasDelivered(event)) continue;
      const trigger = reason === 'motion_vetoed' || reason.includes('VETO') || reason.includes('BLOCKED')
        ? 'interrupt'
        : 'flush';
      const payload = {
        type: 'xuegeros.motion',
        text: this.renderEvent(event, reason),
        meta: { seq: event.seq, kind: event.kind },
      };
      try {
        if (!this.host?.pushEvent) return;
        await this.host.pushEvent(payload, { trigger });
        this.log?.markDelivered(event);
        this.log?.writeCursor(Number(event.seq) || 0);
      } catch (error) {
        this.host?.log?.warn?.(`[xuegeros] pushEvent failed: ${error?.message ?? error}`);
        return;
      }
    }
  }

  renderEvent(event, reason) {
    if (event.kind === 'tour_done') {
      const bits = [];
      if (event.reason) bits.push(event.reason);
      if (event.elapsed_s != null) bits.push(`${event.elapsed_s} s`);
      if (event.dist_to_start_m != null) bits.push(`离起点 ${event.dist_to_start_m} m`);
      if (event.encoder_path_m != null) bits.push(`编码器 ${event.encoder_path_m} m`);
      if (event.known_delta_start != null) bits.push(`地图新增 ${event.known_delta_start} 格`);
      return `巡游结束（${bits.join(' · ') || '无细节'}）。`;
    }
    const what = [event.skill || event.goal, event.sector == null ? '' : `扇区 ${event.sector}`]
      .filter(Boolean).join(' ');
    if (event.kind === 'motion_vetoed' || event.state === 'VETO') {
      const clearance = event.clearance_m ?? '未知';
      const threshold = event.threshold_m ?? '未知';
      return `小脑否决${what ? `（${what}）` : ''}：净空 ${clearance} m < 阈值 ${threshold} m，车已停。`;
    }
    if (event.kind === 'motion_blocked' || event.state === 'BLOCKED') {
      const sensor = String(event.reason || '').startsWith('odom') ? '里程计' : '雷达';
      const age = String(event.reason || '').startsWith('odom') ? event.odom_age_s : event.scan_age_s;
      const limit = String(event.reason || '').startsWith('odom') ? event.odom_limit_s : event.scan_limit_s;
      return `${sensor}陈旧 ${age ?? '未知'} s（阈值 ${limit ?? '未知'} s），未下发${what || '动作'}。`;
    }
    if (event.kind === 'command_failed') {
      return `未下发${what || '动作'}：${event.reason || event.error || '未知原因'}。`;
    }
    if (event.kind === 'command_error') {
      return `控制台命令格式错误：${event.detail || '未知命令'}。`;
    }
    if (event.kind === 'proposal_refused' || event.kind === 'execute_refused') {
      return `未下发${what || '动作'}：${event.error || '未知原因'}。`;
    }
    return `XuegerOS 事件：${reason}`;
  }

  // --- observation ---------------------------------------------------------
  online() {
    return Boolean(this.client?.connected && this.state);
  }

  ageMs() {
    return this.lastStateAt ? Date.now() - this.lastStateAt : null;
  }

  sectorRows() {
    return Array.isArray(this.state?.sectors) ? this.state.sectors : [];
  }

  actions() {
    const rows = this.sectorRows();
    if (!rows.length) return { forward: [], look: [], reason: 'sector_distances_unavailable' };
    const forward = rows
      .filter((row) => finite(row.min_m) && row.min_m > 0.30)
      .map((row) => row.sector);
    const look = rows
      .filter((row) => finite(row.min_m) && row.min_m > 0.15)
      .map((row) => row.sector);
    return { forward, look };
  }

  envPromptVars() {
    const state = this.state;
    const online = this.online();
    const hb = this.heartbeat;
    const health = hb
      ? `scan ${hb.scan_age_s ?? '未知'}/${hb.scan_limit_s ?? '?'} s，` +
        `odom ${hb.odom_age_s ?? '未知'}/${hb.odom_limit_s ?? '?'} s，` +
        (hb.stale ? '链路降级' : '链路正常')
      : '心跳未到';
    const block = hb?.last_block;
    const status = online
      ? `在线，状态 ${state.state}，seq ${state.seq}，${health}`
      : `不可用：${this.lastError || '尚未连接 robotd'}`;
    let actions;
    if (!online) {
      actions = '当前没有可用动作：先恢复 robotd 与 ssh -L。stop 仍可通过 xuegeros_stop 尝试。';
    } else {
      const { forward, look } = this.actions();
      actions = forward.length
        ? `可前进扇区：${forward.join(', ')}；可原地转向扇区：${look.join(', ') || '无'}。` +
          `距离由主控限幅。`
        : `没有可前进扇区（可用转向：${look.join(', ') || '无'}）。先转身或等待扫描更新。`;
      const explained = this.describeBlock(block);
      if (explained) actions += ` 上次未下发：${explained}。`;
    }
    const tour = state?.tour;
    const tourLine = tour
      ? `巡游 ${tour.phase}${tour.active ? '（进行中）' : ''}` +
        (tour.elapsed_s != null ? `，已运行 ${tour.elapsed_s} s` : '') +
        (tour.dist_to_start_m != null ? `，离起点 ${tour.dist_to_start_m} m` : '') +
        '。'
      : '没有巡游记录。';
    return {
      'xuegeros.status': status,
      'xuegeros.actions': actions,
      'xuegeros.tour': tourLine,
      'xuegeros.control':
        '动作直接调用 xuegeros_goal；房间巡游用 xuegeros_tour（start/stop/status），它只返回 phase 与原始测量。' +
        '判断链路、车轮或摆放时，用 xuegeros_ros 读 ROS 分层、用 xuegeros_tour status 读 facts、用 xuegeros_source 只读源码；不要等一个现成的结论。' +
        (this.cfg.deployment === 'simulation'
          ? '当前为本机仿真：screenshot 默认工作台渲染，map 是真值图，ROS/相机不存在。'
          : '地图用 xuegeros_map，相机用 xuegeros_screenshot。') +
        'dispatched 仅表示受理，后续 observe 的 pose/state/traveled_m 才能证明运动。主控用雷达几何否决；stop 不受限制。',
    };
  }

  describeBlock(block) {
    if (!block) return null;
    const reason = String(block.reason || '');
    const base = reason.split(':')[0];
    const measured = reason.split(':')[1];
    if (base === 'obstacle') {
      return `净空 ${block.clearance_m ?? measured ?? '?'} m < 阈值 ${block.threshold_m ?? '?'} m` +
        (block.sector != null ? `（扇区 ${block.sector}）` : '');
    }
    if (base === 'no_headroom') return `净空 ${block.clearance_m ?? '?'} m 扣掉停车余量后不够一步`;
    if (base === 'scan_stale') return `雷达陈旧 ${block.scan_age_s ?? '?'} s > 阈值 ${block.scan_limit_s ?? '?'} s`;
    if (base === 'odom_stale') return `里程计陈旧 ${block.odom_age_s ?? '?'} s > 阈值 ${block.odom_limit_s ?? '?'} s`;
    if (base === 'scan_missing') return '还没收到 /scan';
    if (base === 'odom_missing') return '还没收到 /odom';
    if (base === 'scan_insufficient') return `有效距离点太少（${block.valid_samples ?? '?'}）`;
    if (base === 'clearance_unknown') return '该方向没有有效测量';
    if (base === 'invalid_sector' || base === 'sector_out_of_range') return '扇区无效';
    return reason || '未知';
  }

  // --- tools ---------------------------------------------------------------
  tools() {
    return [
      { name:'xuegeros_control', tags:['configuration'], description:'读取或保存本机中控与小脑参数；空闲时生效。', parameters:{type:'object', properties:{values:{type:'object', additionalProperties:{type:'number'}}}}, handler:corticoTool(args => this.controlSettings(args)) },
      { name:'xuegeros_interfaces', tags:['read'], description:'读取仿真接口清单与连接地址。', parameters:{type:'object',properties:{}}, handler:corticoTool(() => this.simRead('/api/interfaces')) },
      {
        name: 'xuegeros_observe',
        tags: ['read'],
        description: '读取小车当前状态：运行状态、位姿、扫描龄、12 扇区距离、当前提议与最近事件。只读。',
        parameters: {
          type: 'object',
          additionalProperties: false,
          properties: {
            detail: { type: 'string', enum: ['brief', 'full'], description: 'brief 默认；full 附完整扇区与提议 JSON' },
          },
        },
        handler: corticoTool((args) => this.observe(args.detail)),
      },
      {
        name: 'xuegeros_propose',
        tags: ['motion'],
        description: '只预检不执行：让车上小脑按它手里的扫描评估 forward/look/stop，返回限幅距离、扇区与净空。',
        parameters: {
          type: 'object',
          additionalProperties: false,
          properties: {
            intent: { type: 'string', enum: ['forward', 'left', 'right', 'look', 'face', 'open_space', 'stop'] },
            sector: { type: 'integer', minimum: 0, maximum: 71 },
            distance_m: { type: 'number', minimum: 0.02, maximum: 0.15 },
          },
          required: ['intent'],
        },
        handler: corticoTool((args) => this.propose(args)),
      },
      {
        name: 'xuegeros_goal',
        tags: ['motion'],
        description: '直接下发一个有界意图：forward / look / stop。主控当场评估并执行，返回 dispatched；未下发时给出净空、阈值与传感器年龄。',
        parameters: {
          type: 'object',
          additionalProperties: false,
          properties: {
            intent: { type: 'string', enum: ['forward', 'left', 'right', 'look', 'face', 'open_space', 'stop'] },
            sector: { type: 'integer', minimum: 0, maximum: 71 },
            distance_m: { type: 'number', minimum: 0.02, maximum: 0.15 },
          },
          required: ['intent'],
        },
        handler: corticoTool((args) => this.goal(args)),
      },
      {
        name: 'xuegeros_stop',
        tags: ['motion'],
        description: '立即停车。不需要提议，也不依赖世界健康。',
        parameters: {
          type: 'object',
          additionalProperties: false,
          properties: { reason: { type: 'string', maxLength: 200 } },
        },
        handler: corticoTool((args) => this.stopMotion(args.reason)),
      },
      {
        name: 'xuegeros_look',
        tags: ['read'],
        description: '让视觉模型看一眼当前相机帧并回答一个具体问题（单次 2-6 秒）。只用于理解画面，不用于控制。',
        parameters: {
          type: 'object',
          additionalProperties: false,
          properties: { question: { type: 'string', maxLength: 1000 } },
          required: ['question'],
        },
        handler: corticoTool((args) => this.look(args.question)),
      },
      {
        name: 'xuegeros_tour',
        tags: ['motion'],
        description: '房间巡游：start 开始前沿探索并自动回家，stop 取消，status 读最新一次原始测量。返回的是事实，不是结论。',
        parameters: {
          type: 'object',
          additionalProperties: false,
          properties: {
            action: { type: 'string', enum: ['start', 'pause', 'resume', 'cancel', 'stop', 'status'], description: '默认 status' },
            variant_id: { type: 'string', description: '仿真场景变体；省略沿用后端缺省' },
          },
        },
        handler: corticoTool((args) => this.tour(args)),
      },
      {
        name: 'xuegeros_map',
        tags: ['read'],
        description: '取当前地图 PNG 的链接与原始元信息（尺寸、分辨率、已探索比例、年龄）。只读。',
        parameters: { type: 'object', additionalProperties: false, properties: {} },
        handler: corticoTool(() => this.mapArtifact()),
      },
      {
        name: 'xuegeros_screenshot',
        tags: ['read'],
        description: '取当前图片与来源。仿真默认 workbench（实际工作台渲染，不是相机）；真车默认 camera_front。缺失不会用其他来源替代。',
        parameters: { type: 'object', additionalProperties: false, properties: {
          kind: { type: 'string', enum: ['workbench', 'camera_front'] },
        } },
        handler: corticoTool((args) => this.screenshot(args)),
      },
      {
        name: 'xuegeros_ros',
        tags: ['read'],
        description: '原样返回 Body Adapter 的 /v1/state 与 /v1/graph：VM/网络/agent/gmapping 分层、节点与话题数、无发布者或 QoS 冲突。链路判断由你来做。',
        parameters: { type: 'object', additionalProperties: false, properties: {} },
        handler: corticoTool(() => this.rosFacts()),
      },
      {
        name: 'xuegeros_source',
        tags: ['read'],
        description: '只读查看本仓库源码：path 相对仓库根，限定 agent_control / coo-world-xuegeros/src / docs / ros2_safety/xuegeros_safety / tools。上限 200 KB，凭据类文件拒绝。',
        parameters: {
          type: 'object',
          additionalProperties: false,
          properties: {
            path: { type: 'string', description: '例如 agent_control/tour.py' },
            max_bytes: { type: 'integer', minimum: 1, maximum: 200000 },
          },
          required: ['path'],
        },
        handler: corticoTool((args) => this.sourceFile(args)),
      },
    ];
  }

  async request(message) {
    if (!this.client) throw new Error('robotd_unavailable');
    return this.client.request(message);
  }

  async observe(detail = 'brief') {
    const rows = this.sectorRows();
    const summary = {
      deployment: this.cfg.deployment,
      backend_url: this.robotdUrl(),
      physical: this.state?.physical ?? null,
      online: this.online(),
      state: this.state?.state ?? null,
      reason: this.state?.reason ?? null,
      seq: this.state?.seq ?? null,
      pose: this.state?.pose ?? null,
      scan_age_s: this.state?.scan?.age_s ?? null,
      odom_age_s: this.state?.odom_age_s ?? null,
      traveled_m: this.state?.traveled_m ?? null,
      proposal: this.state?.proposal ?? null,
      tour: this.state?.tour ?? null,
      health: {
        age_ms: this.lastHeartbeatAt ? Date.now() - this.lastHeartbeatAt : null,
        stale: this.heartbeat?.stale ?? null,
        scan_age_s: this.heartbeat?.scan_age_s ?? null,
        scan_limit_s: this.heartbeat?.scan_limit_s ?? null,
        odom_age_s: this.heartbeat?.odom_age_s ?? null,
        last_block: this.heartbeat?.last_block ?? null,
        tour_controller_age_s: this.heartbeat?.tour_controller_age_s ?? null,
      },
      actions: this.actions(),
      sectors: rows.map((row) => ({ sector: row.sector, min_m: row.min_m })),
      events_seen: this.eventsSeen,
      error: this.lastError,
    };
    if (detail === 'full') return summary;
    return {
      online: summary.online,
      deployment: summary.deployment,
      backend_url: summary.backend_url,
      physical: summary.physical,
      pose: summary.pose,
      seq: summary.seq,
      traveled_m: summary.traveled_m,
      state: summary.state,
      reason: summary.reason,
      scan_age_s: this.heartbeat?.scan_age_s ?? summary.scan_age_s,
      scan_limit_s: this.heartbeat?.scan_limit_s ?? null,
      front_m: this.state?.scan?.front_m ?? null,
      health: summary.health,
      actions: summary.actions,
      proposal: summary.proposal,
      tour: summary.tour,
      error: summary.error,
    };
  }

  async propose(args) {
    try {
      return await this.request(this.commandFrom(args, 'propose'));
    } catch (error) {
      return { ok: false, accepted: false, error: 'robotd_unreachable', detail: String(error?.message || error) };
    }
  }

  async goal(args) {
    if (String(args.intent).toLowerCase() === 'stop') return this.stopMotion('goal:stop');
    if (this.cfg.deployment === 'simulation' && this.state?.physical !== false) {
      return { ok: false, dispatched: false, error: 'simulation_not_confirmed' };
    }
    try {
      const answer = await this.request(this.commandFrom(args, 'goal'));
      return { ok: Boolean(answer?.dispatched), ...answer,
        completion_verified: false, backend_url: this.robotdUrl(),
        note: 'dispatched 只表示已受理；实际行驶与完成需读取 observe 的 pose/state/traveled_m。' };
    } catch (error) {
      return { ok: false, dispatched: false, error: 'robotd_unreachable', detail: String(error?.message || error) };
    }
  }

  commandFrom(args, type = 'propose') {
    const intent = String(args.intent || '').toLowerCase();
    const command = { t: type, goal: intent === 'look' ? 'face' : intent };
    if (type === 'goal') command.source = 'ai';
    if (args.sector != null) command.sector = Number(args.sector);
    if (args.distance_m != null && intent !== 'look') command.distance_m = Number(args.distance_m);
    return command;
  }

  async stopMotion(reason = 'world_stop') {
    try {
      return await this.request({ t: 'stop', reason: String(reason).slice(0, 120) });
    } catch (error) {
      return { ok: false, error: 'robotd_unreachable', detail: String(error?.message || error) };
    }
  }

  async look(question) {
    if (this.cfg.deployment === 'simulation') return this.simUnavailable('camera_front');
    const text = String(question || '').trim();
    if (!text) return { ok: false, error: 'empty_question' };
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), this.cfg.lookTimeoutMs);
    try {
      const frame = await fetch(this.cfg.frameUrl, { signal: controller.signal, redirect: 'error' });
      if (!frame.ok) throw new Error(`frame HTTP ${frame.status}`);
      const bytes = Buffer.from(await frame.arrayBuffer());
      const response = await fetch(`${this.cfg.vlmUrl.replace(/\/$/, '')}/v1/chat/completions`, {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        signal: controller.signal,
        body: JSON.stringify({
          model: 'local',
          max_tokens: 220,
          messages: [
            {
              role: 'user',
              content: [
                { type: 'text', text: `${text}\n只回答看到的内容；看不清就说看不清。` },
                { type: 'image_url', image_url: { url: `data:image/jpeg;base64,${bytes.toString('base64')}` } },
              ],
            },
          ],
        }),
      });
      if (!response.ok) throw new Error(`vlm HTTP ${response.status}`);
      const payload = await response.json();
      const answer = payload?.choices?.[0]?.message?.content;
      return { ok: Boolean(answer), answer: answer || null };
    } catch (error) {
      return { ok: false, error: 'look_failed', detail: String(error?.message || error) };
    } finally {
      clearTimeout(timer);
    }
  }

  // --- strategy and raw facts ---------------------------------------------
  async tour(args = {}) {
    const action = String(args.action || 'status').toLowerCase();
    if (['start', 'resume'].includes(action) && this.cfg.deployment === 'simulation' && this.state?.physical !== false) {
      return { ok: false, accepted: false, error: 'simulation_not_confirmed' };
    }
    try {
      return await this.request({ t: 'tour', action, source: 'ai',
        ...(action === 'start' ? { strategy: 'tour', ...(args.variant_id ? { variant_id: args.variant_id } : {}) } : {}),
      });
    } catch (error) {
      return { ok: false, accepted: false, error: 'robotd_unreachable', detail: String(error?.message || error) };
    }
  }

  async mapArtifact() {
    if (this.cfg.deployment === 'simulation') {
      const payload = await this.simRead('/api/maps');
      if (!payload.ok) return payload;
      return { ok: true, kind: 'truth', url: `${this.simHttpUrl()}/api/map.png`,
        meta: payload.data.maps?.truth, physical: false,
        note: 'SDF 真值图，仅用于显示与评估，不是 SLAM 或真车 /map。' };
    }
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), 5000);
    try {
      const response = await fetch(this.cfg.mapMetaUrl, { signal: controller.signal, redirect: 'error' });
      const meta = response.ok ? await response.json() : null;
      return response.ok
        ? { ok: true, url: this.cfg.mapUrl, meta }
        : { ok: false, url: this.cfg.mapUrl, error: `map_http_${response.status}` };
    } catch (error) {
      return { ok: false, url: this.cfg.mapUrl, error: 'map_unavailable', detail: String(error?.message || error) };
    } finally {
      clearTimeout(timer);
    }
  }

  simUnavailable(kind) {
    return { ok: false, availability: 'unavailable', kind, deployment: 'simulation', physical: false,
      reason: kind === 'ros' ? '本机运动学影子后端没有 ROS /v1/state 或 /v1/graph；用 observe 读实际仿真状态。'
        : '本机运动学影子后端没有相机帧；工作台截图请使用 kind=workbench。',
      error: kind === 'ros' ? 'ros_unavailable_in_simulation' : 'camera_unavailable_in_simulation' };
  }

  simHttpUrl() {
    const url = new URL(this.cfg.simulationHttpUrl);
    if (url.protocol !== 'http:' || !url.hostname || url.username || url.password
        || url.pathname !== '/' || url.search || url.hash) {
      throw new Error('simulationHttpUrl must be an HTTP origin');
    }
    return url.origin;
  }

  async controlSettings(args = {}) {
    if (this.cfg.deployment !== 'simulation') return {ok:false, error:'simulation_only'};
    const writing = args.values !== undefined;
    try {
      const response = await fetch(`${this.simHttpUrl()}/api/control`, {
        method: writing ? 'POST' : 'GET', redirect:'error',
        signal:AbortSignal.timeout(this.cfg.requestTimeoutMs),
        ...(writing ? {headers:{'content-type':'application/json'}, body:JSON.stringify(args.values)} : {}),
      });
      return await response.json();
    } catch (e) { return {ok:false, error:String(e.message || e)}; }
  }

  async simRead(path) {
    try {
      const response = await fetch(`${this.simHttpUrl()}${path}`, { signal: AbortSignal.timeout(5000), redirect: 'error' });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      return { ok: true, data: await response.json() };
    } catch (error) {
      return { ok: false, error: 'simulation_observation_unavailable', detail: String(error?.message || error), url: `${this.simHttpUrl()}${path}` };
    }
  }

  async screenshot(args = {}) {
    if (this.cfg.deployment === 'simulation') {
      if (args.kind === 'camera_front') return this.simUnavailable('camera_front');
      const payload = await this.simRead('/api/workbench/latest');
      if (!payload.ok) return payload;
      return { ...payload.data, url: payload.data.url ? `${this.simHttpUrl()}${payload.data.url}` : null };
    }
    if (args.kind === 'workbench') return { ok: false, error: 'workbench_requires_simulation' };
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), 5000);
    try {
      const response = await fetch(this.cfg.frameUrl, { signal: controller.signal, redirect: 'error' });
      if (!response.ok) throw new Error(`frame HTTP ${response.status}`);
      const seq = response.headers.get('x-frame-seq');
      const bytes = (await response.arrayBuffer()).byteLength;
      const origin = new URL(this.cfg.frameUrl).origin;
      return {
        ok: true,
        url: seq ? `${origin}/v1/frames/${seq}.jpg` : this.cfg.frameUrl,
        seq: seq ? Number(seq) : null,
        bytes,
        captured_at: Math.round(Date.now() / 1000),
      };
    } catch (error) {
      return { ok: false, error: 'frame_unavailable', detail: String(error?.message || error) };
    } finally {
      clearTimeout(timer);
    }
  }

  // Electron's settings partition forwards only Core/pet HTTP origins. Deliver
  // simulation PNGs through the host's binary panel RPC, never an arbitrary URL.
  async simulationImage(args = {}) {
    if (this.cfg.deployment !== 'simulation') throw new Error('image_requires_simulation');
    let endpoint;
    if (args.kind === 'truth') endpoint = '/api/map.png';
    else if (args.kind === 'workbench' && Number.isSafeInteger(args.seq) && args.seq > 0) {
      endpoint = `/api/workbench/frames/${args.seq}.png`;
    } else throw new Error('invalid_simulation_image');
    const response = await fetch(`${this.simHttpUrl()}${endpoint}`, {
      signal: AbortSignal.timeout(5000), redirect: 'error',
    });
    if (!response.ok) throw new Error(`image_http_${response.status}`);
    if (response.headers.get('content-type')?.split(';')[0] !== 'image/png') throw new Error('image_not_png');
    const limit = 2_000_000;
    const reader = response.body.getReader();
    const chunks = [];
    let size = 0;
    try {
      while (true) {
        const { done, value } = await reader.read();
        if (done) break;
        size += value.byteLength;
        if (size > limit) throw new Error('image_too_large');
        chunks.push(value);
      }
    } finally {
      await reader.cancel();
    }
    const bytes = Buffer.concat(chunks, size);
    if (!bytes.subarray(0, 8).equals(Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]))) throw new Error('image_not_png');
    return { $binary: { mime: 'image/png', base64: bytes.toString('base64') } };
  }

  async rosFacts() {
    if (this.cfg.deployment === 'simulation') return this.simUnavailable('ros');
    const read = async (url) => {
      const controller = new AbortController();
      const timer = setTimeout(() => controller.abort(), 5000);
      try {
        const response = await fetch(url, { signal: controller.signal, redirect: 'error' });
        const text = await response.text();
        return {
          ok: response.ok,
          status: response.status,
          body: text.length > 12000 ? `${text.slice(0, 12000)}…` : text,
        };
      } catch (error) {
        return { ok: false, error: String(error?.message || error) };
      } finally {
        clearTimeout(timer);
      }
    };
    const [state, graph] = await Promise.all([read(this.cfg.stateUrl), read(this.cfg.graphUrl)]);
    return { ok: state.ok || graph.ok, state, graph };
  }

  async sourceFile(args = {}) {
    const rel = String(args.path || '').trim();
    if (!rel || rel.startsWith('/') || /^[a-zA-Z]:/.test(rel) || rel.split(/[\\/]/).includes('..')) {
      return { ok: false, error: 'path_rejected' };
    }
    if (SOURCE_DENY.some((pattern) => pattern.test(rel))) return { ok: false, error: 'path_rejected' };
    const root = path.resolve(this.cfg.sourceRoot || REPO_ROOT);
    let rootReal;
    let targetReal;
    try {
      rootReal = await fs.realpath(root);
      targetReal = await fs.realpath(path.resolve(root, rel));
    } catch {
      return { ok: false, error: 'not_found' };
    }
    const allowed = SOURCE_PREFIXES.some((prefix) => {
      const dir = path.join(rootReal, prefix);
      return targetReal === dir || targetReal.startsWith(`${dir}${path.sep}`);
    });
    if (!allowed) return { ok: false, error: 'path_rejected' };
    const stat = await fs.stat(targetReal).catch(() => null);
    if (!stat || !stat.isFile()) return { ok: false, error: 'not_a_file' };
    const limit = Math.min(Math.max(Number(args.max_bytes) || SOURCE_MAX_BYTES, 1), SOURCE_MAX_BYTES);
    const handle = await fs.open(targetReal, 'r');
    try {
      const buffer = Buffer.alloc(limit);
      const { bytesRead } = await handle.read(buffer, 0, limit, 0);
      return {
        ok: true,
        path: rel,
        bytes: stat.size,
        truncated: stat.size > bytesRead,
        text: buffer.subarray(0, bytesRead).toString('utf8'),
      };
    } finally {
      await handle.close();
    }
  }

  // --- web UI --------------------------------------------------------------
  webUi(language = 'zh') {
    const isEn = language === 'en';
    const online = this.online();
    return {
      label: isEn ? 'XuegerOS Car' : LABEL,
      config: [CONFIG_GROUP],
      lamps: [
        {
          label: isEn ? 'robotd' : 'robotd',
          state: online ? 'online' : this.lastError ? 'error' : 'offline',
          hint: online ? `seq ${this.state?.seq}` : this.lastError || (isEn ? 'not connected' : '未连接'),
        },
        {
          label: isEn ? 'State' : '状态',
          state: this.state?.state === 'ACTIVE' ? 'online' : 'offline',
          hint: this.state?.state || '—',
        },
      ],
      badges: [
        { label: isEn ? 'Events' : '事件', value: this.eventsSeen, tone: 'plain' },
        { label: isEn ? 'Scan' : '扫描龄', value: this.state?.scan?.age_s ?? '—', tone: 'plain' },
      ],
      links: this.cfg.deployment === 'simulation' ? [
        { label: isEn ? 'Simulation workbench :18142' : '仿真工作台 :18142', href: `${this.simHttpUrl()}/` },
      ] : [
        { label: isEn ? 'Control' : '控制窗', href: `${this.cfg.localConsoleUrl}/`, inheritTheme: true },
        { label: isEn ? 'Map / lidar' : '地图/雷达', href: `${this.cfg.localConsoleUrl}/map`, inheritTheme: true },
        { label: isEn ? '3D model' : '3D 模型', href: `${this.cfg.localConsoleUrl}/3d`, inheritTheme: true },
      ],
      panels: [
        {
          id: 'live',
          title: isEn ? 'Motion console' : '动作面板',
          description: isEn
            ? 'Status, available sectors and the last command that did not dispatch. Buttons in the local window send the same goal directly.'
            : '状态、可用扇区与最近一次未下发原因。本地控制窗的按钮走同一条 goal。',
          getMethods: ['state', 'screenshot', 'map', 'ros', 'control', 'interfaces'],
          postMethods: this.cfg.deployment === 'simulation' ? ['goal', 'stop', 'tour', 'image', 'control'] : [],
        },
      ],
      invoke: async (panel, method, rawArgs) => {
        if (panel !== 'live') throw new Error(`unknown panel ${panel}`);
        if (method === 'state') return this.webUiState();
        if (method === 'screenshot') return this.screenshot();
        if (method === 'map') return this.mapArtifact();
        if (method === 'ros') return this.rosFacts();
        if (method === 'stop') return this.stopMotion('coo_panel');
        const args = Array.isArray(rawArgs) ? rawArgs[0] : rawArgs;
        if (method === 'control') return this.controlSettings(args ?? {});
        if (method === 'interfaces') return this.simRead('/api/interfaces');
        if (method === 'image') return this.simulationImage(args ?? {});
        if (method === 'goal' && this.cfg.deployment === 'simulation') return this.goal(args ?? {});
        if (method === 'tour' && this.cfg.deployment === 'simulation') return this.tour(args ?? {});
        throw new Error(`unknown method ${method}`);
      },
      promptDocs: [
        {
          key: 'worlds.xuegeros.envPrompt',
          title: isEn ? 'XuegerOS motion contract' : 'XuegerOS 小车 · 动作契约',
          description: isEn
            ? 'How to read the action options and negotiate a bounded intent.'
            : '怎么读动作选项、怎么与主控协商一个有界动作。',
          path: ENV_PROMPT_FILE,
          role: 'envPrompt',
          vars: [
            { name: 'xuegeros.status', description: isEn ? 'robotd link and state' : 'robotd 连接与状态' },
            { name: 'xuegeros.actions', description: isEn ? 'Sectors open to forward/turn' : '可前进/可转向的扇区' },
            { name: 'xuegeros.tour', description: isEn ? 'Tour phase and raw return distance' : '巡游阶段与离起点距离' },
            { name: 'xuegeros.control', description: isEn ? 'Action and evidence rules' : '动作与证据规则' },
          ],
        },
      ],
    };
  }

  webUiState() {
    return {
      online: this.online(),
      deployment: this.cfg.deployment,
      physical: typeof this.state?.physical === 'boolean' ? this.state.physical : null,
      state: this.state?.state ?? null,
      seq: this.state?.seq ?? null,
      state_ts: this.state?.ts ?? null,
      traveled_m: this.state?.traveled_m ?? null,
      goal: this.state?.goal ?? null,
      scan_age_s: this.state?.scan?.age_s ?? null,
      pose: this.state?.pose ?? null,
      actions: this.actions(),
      proposal: this.state?.proposal ?? null,
      tour: this.state?.tour ?? null,
      events_seen: this.eventsSeen,
      error: this.lastError,
      links: {
        control: this.cfg.deployment === 'simulation' ? `${this.simHttpUrl()}/` : `${this.cfg.localConsoleUrl}/`,
        map: this.cfg.deployment === 'simulation' ? `${this.simHttpUrl()}/` : `${this.cfg.localConsoleUrl}/map`,
        model3d: this.cfg.deployment === 'simulation' ? `${this.simHttpUrl()}/` : `${this.cfg.localConsoleUrl}/3d`,
      },
    };
  }
}

export { LABEL };
