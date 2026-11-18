// XuegerOS motion panel for the current Coo WebUI contract.
// The host provides ctx.root, ctx.ui, ctx.invoke and ctx.invokeBinary.
// The 3D workbench opens independently; the panel contains controls and tuning.

const COPY = {
  zh: {
    title: 'XuegerOS 动作面板', status: '状态', seq: 'seq', scan: '扫描龄',
    pose: '位姿', target: '运行目标', physical: '物理输出', events: '事件',
    online: '在线', offline: '未连接', simulation: '本机仿真', physicalTarget: '物理车端',
    forward: '前进 0.10 m', left: '左转', right: '右转', stop: '立即停车',
    sent: '已发送', failed: '动作失败', refresh: '刷新', loading: '读取中…',
    tour: '房间巡游', start: '开始巡游', pause: '暂停', resume: '继续', cancel: '取消任务',
    phase: '任务', stage: '阶段', known: '已观测格', traveled: '任务里程', home: '距起点', elapsed: '用时',
    accepted: '验收通过', exhausted: '指标未达成', stopped: '已停车', desc: '巡游、返航与停车',
  },
  en: {
    title: 'XuegerOS motion panel', status: 'State', seq: 'seq', scan: 'Scan age',
    pose: 'Pose', target: 'Target', physical: 'Physical output', events: 'Events',
    online: 'online', offline: 'offline', simulation: 'local simulation', physicalTarget: 'physical car',
    forward: 'Forward 0.10 m', left: 'Turn left', right: 'Turn right', stop: 'Stop now',
    sent: 'sent', failed: 'action failed', refresh: 'Refresh', loading: 'Loading…',
    tour: 'Room tour', start: 'Start tour', pause: 'Pause', resume: 'Resume', cancel: 'Cancel task',
    phase: 'Task', stage: 'Stage', known: 'Observed cells', traveled: 'Task distance', home: 'Distance home', elapsed: 'Elapsed',
    accepted: 'Acceptance passed', exhausted: 'Acceptance not met', stopped: 'Stopped', desc: 'Explore, return home and stop',
  },
};

function text(language) { return String(language || '').startsWith('en') ? COPY.en : COPY.zh; }
function message(error) { return String(error?.message || error); }

function mount(ctx) {
  const t = text(ctx.language);
  const sheet = ctx.ui.sheet({ title: t.title, desc: t.desc });
  ctx.root.appendChild(sheet.el);
  sheet.el.classList.add('xuegeros-panel');
  const body = sheet.body;
  // Keep the scene inside Coo without an external-browser protocol handoff.
  // Mount only after confirming this World targets local simulation.
  const preview = ctx.ui.h('div', 'xuegeros-preview');
  preview.hidden = true;
  const previewLinks = ctx.ui.h('div', 'xuegeros-view-links');
  const workbenchLink = ctx.ui.h('a', 'btn sm', 'http://127.0.0.1:18142/');
  workbenchLink.href = 'http://127.0.0.1:18142/';
  workbenchLink.target = '_blank';
  workbenchLink.rel = 'noopener noreferrer';
  workbenchLink.title = '打开完整仿真工作台（端口 18142）';
  previewLinks.append(workbenchLink);
  const artifactLink = ctx.ui.h('a', 'btn sm', '');
  artifactLink.target = '_blank';
  artifactLink.rel = 'noopener noreferrer';
  artifactLink.hidden = true;
  async function artifact(method) {
    const result = await ctx.invoke(method, []);
    artifactLink.hidden = !result?.ok || !result?.url;
    if (result?.ok && result?.url) {
      artifactLink.href = result.url;
      artifactLink.textContent = method === 'map' ? '打开真值地图' : '打开工作台截图';
      artifactLink.title = artifactLink.textContent;
    } else receipt.textContent = result?.reason || result?.error || '读取失败';
  }
  previewLinks.append(ctx.ui.button('工作台截图', {size:'sm', onClick:() => void artifact('screenshot').catch(e => { receipt.textContent = message(e); })}),
    ctx.ui.button('真值地图', {size:'sm', onClick:() => void artifact('map').catch(e => { receipt.textContent = message(e); })}), artifactLink);
  preview.append(previewLinks);
  body.appendChild(preview);
  const stats = ctx.ui.statgrid();
  body.appendChild(stats);
  const actions = ctx.ui.actions();
  actions.classList.add('xuegeros-actions');
  const tourActions = actions;
  body.insertBefore(tourActions, preview);
  body.insertBefore(actions, preview);
  const liveReadout = ctx.ui.msgline('');
  body.insertBefore(liveReadout, preview);
  const note = ctx.ui.msgline('');
  body.appendChild(note);
  const receipt = ctx.ui.msgline('');
  receipt.setAttribute('role', 'status');
  receipt.setAttribute('aria-live', 'polite');
  body.appendChild(receipt);
  const controls = {};
  const stepButtons = [];
  let pending = false;
  let latest = null;
  const tuning = ctx.ui.h('details', 'xuegeros-tuning');
  const summary = ctx.ui.h('summary', '', '中控与小脑');
  summary.title = '编辑执行参数';
  const fields = ctx.ui.h('div', 'xuegeros-fields');
  const tuningStatus = ctx.ui.msgline('');
  const inputs = new Map();
  async function loadControl() {
    try {
      const result = await ctx.invoke('control', []);
      if (!result.ok) throw new Error(result.error || '读取失败');
      fields.replaceChildren(); inputs.clear();
      for (const [key, spec] of Object.entries(result.schema)) {
        const label = ctx.ui.h('label', '', spec.title);
        const input = ctx.ui.h('input');
        input.type = 'number'; input.min = spec.minimum; input.max = spec.maximum;
        input.step = spec.type === 'integer' ? '1' : 'any';
        input.value = result.values[key]; input.title = key;
        input.setAttribute('aria-label', spec.title);
        label.append(input); fields.append(label); inputs.set(key, input);
      }
      tuningStatus.textContent = result.busy ? '任务结束或停车后可保存' : '';
    } catch (e) { tuningStatus.textContent = message(e); }
  }
  const saveControl = ctx.ui.button('保存参数', {size:'sm', onClick:async () => {
    const values = {};
    for (const [key, input] of inputs) {
      if (!input.value.trim() || !input.checkValidity()) { input.reportValidity(); return; }
      values[key] = Number(input.value);
    }
    try {
      const result = await ctx.invoke('control', [{values}]);
      tuningStatus.textContent = result.ok ? '已保存并生效' : result.error || '保存失败';
    } catch (e) { tuningStatus.textContent = message(e); }
  }});
  tuning.append(summary, fields, saveControl, ctx.ui.button('重新读取', {size:'sm', onClick:loadControl}), tuningStatus);
  tuning.addEventListener('toggle', () => { if (tuning.open && !inputs.size) void loadControl(); });
  body.append(tuning);

  let requestSerial = 0;
  const value = (v, fallback = '—') => v == null ? fallback : String(v);

  function render(data) {
    if (!data || typeof data !== 'object') return;
    const sim = data.deployment === 'simulation';
    preview.hidden = !sim;
    latest = data;
    tuning.hidden = !sim;
    const href = data.links?.control;
    if (href) { workbenchLink.href = href; workbenchLink.textContent = '打开仿真工作台'; }
    const tour = data.tour || {};
    const pose = Array.isArray(data.pose) ? data.pose.map(n => Number(n).toFixed(3)).join(' / ') : '—';
    liveReadout.textContent = `实时 ${data.state || '—'} · x/y/yaw ${pose} · 单步 ${Number(data.traveled_m || 0).toFixed(3)} m · #${data.seq ?? '—'}`;
    const rawStage = tour.recovery?.stage || tour.backend_phase || '—';
    const zh = !String(ctx.language || '').startsWith('en');
    const labels = zh ? {IDLE:'未开始', STARTING:'准备中', RUNNING:'巡游中', PAUSED:'已暂停', BLOCKED:'受阻，已停车', TIMEOUT:'任务超时', HANDOVER:'需要接管', CANCELLED:'已取消', EXPLORE:'探索房间', RETURN:'返回起点', CHECK_REAR:'检查后方', REVERSE:'后退脱困', SELECT_EXIT:'选择出口', TURN:'转向出口', EXIT:'离开障碍', REPLAN:'重新规划', FAILED:'脱困失败', DONE:'完成'} : {};
    const stage = labels[rawStage] || rawStage;
    const phase = tour.phase === 'DONE'
      ? (tour.reason === 'reached' ? t.accepted : tour.reason === 'exhausted' ? t.exhausted : t.stopped)
      : labels[tour.phase] || value(tour.phase, zh ? '未开始' : 'Idle');
    stats.replaceChildren(
      ctx.ui.stat({ k: tour.active ? t.phase : (zh ? '上次巡游' : 'Last tour'), v: phase }),
      ctx.ui.stat({ k: t.stage, v: value(stage) }),
      ctx.ui.stat({ k: t.elapsed, v: tour.elapsed_s == null ? '—' : `${Number(tour.elapsed_s).toFixed(1)} s` }),
      ctx.ui.stat({ k: t.home, v: tour.dist_to_start_m == null ? '—' : `${Number(tour.dist_to_start_m).toFixed(3)} m` }),
      ctx.ui.stat({ k: t.known, v: value(tour.known_cells) }),
      ctx.ui.stat({ k: t.traveled, v: tour.traveled_m == null ? '—' : `${Number(tour.traveled_m).toFixed(3)} m` }),
    );
    const safe = sim && data.online && data.physical === false;
    const busy = tour.active === true;
    for (const button of stepButtons) button.disabled = !safe || busy || pending;
    controls.start.disabled = !safe || busy || pending;
    controls.pause.disabled = !safe || tour.phase !== 'RUNNING' || pending;
    controls.resume.disabled = !safe || !['PAUSED', 'BLOCKED'].includes(tour.phase) || pending;
    controls.cancel.disabled = !safe || !busy || pending;
    controls.stop.disabled = !data.online;
    note.textContent = data.error
      ? `${t.failed}: ${data.error}`
      : `${data.online ? t.online : t.offline} · ${sim ? t.simulation : t.physicalTarget}${tour.scene ? ` · ${tour.scene}` : ''} · ${t.physical}: ${value(data.physical)}`;
  }

  async function refresh() {
    const serial = ++requestSerial;
    try {
      const data = await ctx.invoke('state', []);
      if (serial === requestSerial && !ctx.signal.aborted) render(data);
    } catch (error) {
      if (serial === requestSerial && latest) render({ ...latest, online: false, error: message(error) });
    }
  }

  async function command(method, args) {
    pending = true;
    if (latest) render(latest);
    const release = ctx.ui.busy(t.loading, t.refresh);
    try {
      const result = await ctx.invoke(method, [args]);
      receipt.textContent = result?.accepted || result?.dispatched ? t.sent : `${t.failed}: ${result?.error || result?.reason || 'rejected'}`;
      await refresh();
    } catch (error) {
      receipt.textContent = `${t.failed}: ${message(error)}`;
    } finally {
      release.dispose();
      pending = false;
      if (latest) render(latest);
    }
  }

  function button(label, options) {
    const element = ctx.ui.button(label, { size: 'sm', ...options });
    element.title = label;
    element.setAttribute('aria-label', label);
    element.disabled = true;
    return element;
  }
  for (const action of ['start', 'pause', 'resume', 'cancel']) {
    controls[action] = button(t[action], { variant: action === 'start' ? 'primary' : 'plain',
      onClick: () => command('tour', { action, ...(action === 'start' && latest?.tour?.scene
        ? { variant_id: latest.tour.scene === 'room_open' ? 'room_open/wide/01' : 'room/default/01' } : {}) }) });
    tourActions.append(controls[action]);
  }
  for (const intent of ['forward', 'left', 'right']) {
    const element = button(t[intent], { onClick: () => command('goal', { intent, ...(intent === 'forward' ? { distance_m: .1 } : {}) }) });
    stepButtons.push(element);
    actions.append(element);
  }
  controls.stop = button(t.stop, { variant: 'danger', onClick: () => command('stop', {}) });
  const refreshButton = button(t.refresh, { onClick: refresh });
  refreshButton.disabled = false;
  actions.append(controls.stop, refreshButton);
  void refresh();
  ctx.interval(refresh, 1000);
}

export default { panels: { live: { mount } } };
