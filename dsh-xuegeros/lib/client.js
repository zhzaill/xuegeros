/**
 * dsh-xuegeros — client half.
 *
 * Registers the "小车" tab in the right sidebar. The cockpit is a normal DSH
 * panel: theme tokens, no DOM outside the slot, all data through same-origin
 * `/sync/xuegeros/*` routes served by the host half.
 *
 * Visual model: camera frame (MJPEG proxy) + normalized YOLO boxes drawn on a
 * canvas overlay + bounded D-pad pulses + the AI handoff switch that lets the
 * session's agent drive through `xuegeros_drive`.
 */
window.__ModuleLoader__.load({
  id: 'dsh-xuegeros',
  factory: (require) => {
    var module = { exports: {} };
    var exports = module.exports;

    const React = require('react');

    const ID = 'dsh-xuegeros';
    const KIND = 'xuegeros';
    const STATE_URL = '/sync/xuegeros/state';
    const FRAME_URL = '/sync/xuegeros/frame';
    const MOVE_URL = '/sync/xuegeros/move';
    const STOP_URL = '/sync/xuegeros/stop';
    const AI_URL = '/sync/xuegeros/ai';
    const SUMMON_URL = '/sync/xuegeros/summon';
    const GATEWAY_URL = '/sync/xuegeros/gateway/start';
    const POLL_MS = 1500;

    // --- styles ----------------------------------------------------------
    const CSS_TAG = 'dsh-xuegeros/cockpit.css';
    const CSS = [
      '.xz-root{--xz-ok:var(--dsw-alias-state-business-primary);--xz-danger:var(--dsw-alias-state-error-primary);--xz-border:var(--dsw-alias-border-l1);--xz-border-strong:var(--dsw-alias-border-l2);container:xz / inline-size;display:flex;flex-direction:column;height:100%;min-height:0;font:var(--dsw-font-xs-13);color:var(--dsw-alias-label-primary);background:var(--dsw-alias-bg-layer-1)}',
      '.xz-head{flex:none;display:flex;align-items:center;gap:8px;min-height:44px;padding:8px 12px;border-bottom:1px solid var(--xz-border-strong);background:var(--dsw-specific-sidebar-fill)}',
      '.xz-dot{flex:none;width:8px;height:8px;border-radius:50%;background:var(--xz-danger)}',
      '.xz-dot[data-ok="1"]{background:var(--xz-ok)}',
      '.xz-title{font-weight:600;line-height:16px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}',
      '.xz-sub{color:var(--dsw-alias-label-tertiary);font:var(--dsw-font-xxs-12);line-height:15px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}',
      '.xz-grow{flex:1;min-width:0}',
      '.xz-badge,.xz-status{display:inline-flex;align-items:center;gap:5px;flex:none;border:1px solid var(--xz-border-strong);border-radius:999px;padding:3px 8px;font:var(--dsw-font-xxs-12);line-height:14px;color:var(--dsw-alias-label-secondary);white-space:nowrap}',
      '.xz-badge[data-state="ok"]{border-color:var(--xz-ok);color:var(--xz-ok)}',
      '.xz-badge[data-state="warn"]{border-color:var(--xz-border-strong);color:var(--dsw-alias-label-secondary)}',
      '.xz-badge[data-state="bad"]{border-color:var(--xz-danger);color:var(--xz-danger)}',
      '.xz-body{flex:1;min-height:0;overflow:auto;padding:10px 12px 18px;display:flex;flex-direction:column;gap:10px}',
      '.xz-overview{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:6px}',
      '.xz-overview-item{min-width:0;padding:7px 8px;border:1px solid var(--xz-border);border-radius:8px;background:var(--dsw-alias-bg-layer-2)}',
      '.xz-overview-label{display:block;color:var(--dsw-alias-label-tertiary);font:var(--dsw-font-xxs-12);line-height:14px}',
      '.xz-overview-value{display:block;margin-top:2px;font-weight:600;font-variant-numeric:tabular-nums;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}',
      '.xz-card{border:1px solid var(--xz-border);border-radius:10px;background:var(--dsw-alias-bg-layer-2);padding:10px;display:flex;flex-direction:column;gap:8px}',
      '.xz-cardhead{display:flex;align-items:center;justify-content:space-between;gap:8px;color:var(--dsw-alias-label-secondary);font:var(--dsw-font-xxs-12);font-weight:600;line-height:16px}',
      '.xz-cardtitle{display:flex;align-items:center;gap:6px}',
      '.xz-cardtitle::before{content:"";width:3px;height:14px;border-radius:2px;background:var(--xz-ok)}',
      '.xz-frame{position:relative;border:1px solid var(--xz-border-strong);border-radius:8px;overflow:hidden;background:var(--dsw-specific-sidebar-fill);aspect-ratio:4/3}',
      '.xz-frame img{display:block;width:100%;height:100%;object-fit:contain;background:var(--dsw-specific-sidebar-fill)}',
      '.xz-frame canvas{position:absolute;inset:0;width:100%;height:100%;pointer-events:none}',
      '.xz-frame .xz-offline{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;color:var(--dsw-alias-label-tertiary);font:var(--dsw-font-xxs-12);text-align:center;padding:12px}',
      '.xz-hud{position:absolute;left:8px;bottom:8px;display:flex;gap:6px;font:var(--dsw-font-xxs-12);color:var(--dsw-alias-label-primary);pointer-events:none}',
      '.xz-hud span{padding:3px 6px;border:1px solid var(--xz-border-strong);border-radius:6px;background:var(--dsw-specific-sidebar-fill)}',
      '.xz-grid{display:grid;grid-template-columns:1fr 1fr;gap:6px}',
      '.xz-kv{display:flex;justify-content:space-between;gap:8px;font:var(--dsw-font-xs-13);line-height:18px}',
      '.xz-k{color:var(--dsw-alias-label-tertiary)}',
      '.xz-v{color:var(--dsw-alias-label-primary);text-align:right;font-variant-numeric:tabular-nums}',
      '.xz-v[data-bad="1"]{color:var(--xz-danger)}',
      '.xz-detlist{display:flex;flex-wrap:wrap;gap:4px}',
      '.xz-det{display:inline-flex;align-items:center;gap:5px;border:1px solid var(--xz-border-strong);border-radius:999px;padding:2px 7px;font:var(--dsw-font-xxs-12);color:var(--dsw-alias-label-secondary)}',
      '.xz-det i{width:6px;height:6px;border-radius:50%;background:var(--xz-ok);font-style:normal}',
      '.xz-empty{padding:7px 8px;border:1px dashed var(--xz-border-strong);border-radius:7px;color:var(--dsw-alias-label-tertiary);font:var(--dsw-font-xxs-12)}',
      '.xz-pad{display:grid;grid-template-columns:repeat(3,40px);grid-template-rows:repeat(3,40px);gap:6px;justify-content:center;align-self:center}',
      '.xz-key{display:flex;align-items:center;justify-content:center;border:1px solid var(--xz-border-strong);border-radius:8px;background:var(--dsw-alias-bg-layer-1);color:var(--dsw-alias-label-secondary);font:inherit;font-size:15px;cursor:pointer;user-select:none;touch-action:none;transition:background 160ms ease,border-color 160ms ease,color 160ms ease,transform 160ms ease}',
      '.xz-key:hover{border-color:var(--xz-ok);color:var(--dsw-alias-label-primary);background:var(--dsw-alias-interactive-bg-hover)}',
      '.xz-key:active{transform:translateY(1px);background:var(--dsw-alias-interactive-bg-hover)}',
      '.xz-key:focus-visible,.xz-btn:focus-visible,.xz-track:focus-visible{outline:2px solid var(--dsw-alias-state-business-primary);outline-offset:2px}',
      '.xz-key[data-stop="1"]{border-color:var(--xz-danger);color:var(--xz-danger);font:var(--dsw-font-xxs-12);font-weight:600}',
      '.xz-key[disabled],.xz-btn[disabled]{opacity:.45;cursor:not-allowed;transform:none}',
      '.xz-row{display:flex;align-items:center;gap:8px}',
      '.xz-row label{flex:none;color:var(--dsw-alias-label-tertiary);font:var(--dsw-font-xxs-12);min-width:42px}',
      '.xz-row input[type="range"]{flex:1;min-width:0;accent-color:var(--xz-ok)}',
      '.xz-row .xz-num{flex:none;width:44px;text-align:right;font-variant-numeric:tabular-nums;color:var(--dsw-alias-label-secondary);font:var(--dsw-font-xxs-12)}',
      '.xz-btn{display:inline-flex;align-items:center;justify-content:center;gap:6px;min-height:30px;border:1px solid var(--xz-border-strong);border-radius:8px;background:var(--dsw-alias-bg-layer-1);color:var(--dsw-alias-label-secondary);font:var(--dsw-font-xs-13);padding:5px 10px;cursor:pointer;transition:background 160ms ease,border-color 160ms ease,color 160ms ease}',
      '.xz-btn:hover{border-color:var(--dsw-alias-label-secondary);color:var(--dsw-alias-label-primary);background:var(--dsw-alias-interactive-bg-hover)}',
      '.xz-btn[data-primary="1"]{border-color:var(--xz-ok);color:var(--xz-ok)}',
      '.xz-btn[data-danger="1"]{border-color:var(--xz-danger);color:var(--xz-danger)}',
      '.xz-switch{display:flex;align-items:center;justify-content:space-between;gap:8px;min-height:28px;font:var(--dsw-font-xs-13);color:var(--dsw-alias-label-secondary)}',
      '.xz-switch[data-critical="1"]{padding:6px 8px;border:1px solid color-mix(in srgb,var(--xz-danger) 45%,var(--xz-border));border-radius:8px;color:var(--dsw-alias-label-primary)}',
      '.xz-switch .xz-track{flex:none;width:34px;height:20px;border-radius:999px;border:1px solid var(--xz-border-strong);position:relative;cursor:pointer;background:var(--dsw-alias-bg-layer-1);transition:border-color 160ms ease,background 160ms ease}',
      '.xz-switch .xz-track::after{content:"";position:absolute;top:2px;left:2px;width:14px;height:14px;border-radius:50%;background:var(--dsw-alias-label-tertiary);transition:transform 160ms ease,background 160ms ease}',
      '.xz-switch .xz-track[data-on="1"]{border-color:var(--xz-ok);background:var(--xz-ok)}',
      '.xz-switch .xz-track[data-on="1"]::after{transform:translateX(14px);background:var(--dsw-alias-bg-layer-1)}',
      '.xz-note{font:var(--dsw-font-xxs-12);line-height:16px;color:var(--dsw-alias-label-tertiary);word-break:break-word}',
      '.xz-note[data-bad="1"]{color:var(--xz-danger)}',
      '.xz-inline-status{display:flex;align-items:center;gap:5px;color:var(--dsw-alias-label-tertiary);font:var(--dsw-font-xxs-12)}',
      '.xz-inline-status::before{content:"";width:6px;height:6px;border-radius:50%;background:var(--xz-ok)}',
      '.xz-inline-status[data-bad="1"]::before{background:var(--xz-danger)}',
      '.xz-modal-backdrop{position:fixed;inset:0;z-index:1000;display:flex;align-items:center;justify-content:center;padding:16px;background:var(--dsw-alias-bg-layer-1)}',
      '.xz-modal{width:min(330px,100%);border:1px solid var(--xz-border-strong);border-radius:10px;background:var(--dsw-alias-bg-layer-2);padding:14px;display:flex;flex-direction:column;gap:10px}',
      '.xz-modal-title{font:var(--dsw-font-xs-13);font-weight:600;color:var(--dsw-alias-label-primary)}',
      '.xz-modal-copy{font:var(--dsw-font-xxs-12);line-height:17px;color:var(--dsw-alias-label-secondary)}',
      '.xz-modal-actions{display:flex;justify-content:flex-end;gap:6px}',
      '.xz-center{display:flex;align-items:center;justify-content:center;height:100%;color:var(--dsw-alias-label-tertiary);font:var(--dsw-font-xs-13);padding:24px;text-align:center}',
      '@container xz (min-width:760px){.xz-body{display:grid;grid-template-columns:minmax(0,1.35fr) minmax(250px,.65fr);align-content:start}.xz-body>.xz-overview,.xz-body>.xz-card:last-child,.xz-body>.xz-note{grid-column:1/-1}.xz-body>.xz-frame-card{grid-row:span 2}.xz-body>.xz-card{min-width:0}}',
      '@container xz (max-width:320px){.xz-body{padding-left:8px;padding-right:8px}.xz-overview{gap:4px}.xz-overview-item{padding:6px}.xz-card{padding:8px}.xz-pad{gap:5px}}',
      '@media (prefers-reduced-motion:reduce){.xz-key,.xz-btn,.xz-track,.xz-track::after{transition:none}}',
    ].join('');

    if (typeof document !== 'undefined' && document.querySelector(`style[data-plugin-css="${CSS_TAG}"]`) === null) {
      const tag = document.createElement('style');
      tag.dataset.plugin = ID;
      tag.dataset.pluginCss = CSS_TAG;
      tag.textContent = CSS;
      document.head.appendChild(tag);
    }

    async function getJson(url, init) {
      const response = await fetch(url, { cache: 'no-store', ...init });
      const text = await response.text();
      let value;
      try {
        value = text ? JSON.parse(text) : {};
      } catch {
        value = { ok: false, error: text.slice(0, 200) };
      }
      if (!response.ok) throw new Error(value?.error ?? `HTTP ${response.status}`);
      return value;
    }

    function fmt(value, fallback = '—') {
      return value === null || value === undefined ? fallback : String(value);
    }

    /** Draw normalized YOLO boxes over the displayed frame. */
    function DetectionLayer({ detections, offline }) {
      const canvasRef = React.useRef(null);
      const wrapRef = React.useRef(null);
      const boxes = Array.isArray(detections) ? detections : [];

      const draw = React.useCallback(() => {
        const canvas = canvasRef.current;
        const wrap = wrapRef.current;
        if (!canvas || !wrap) return;
        const width = wrap.clientWidth;
        const height = wrap.clientHeight;
        const ratio = window.devicePixelRatio || 1;
        canvas.width = Math.max(1, Math.round(width * ratio));
        canvas.height = Math.max(1, Math.round(height * ratio));
        const g = canvas.getContext('2d');
        if (!g) return;
        g.setTransform(ratio, 0, 0, ratio, 0, 0);
        g.clearRect(0, 0, width, height);
        g.font = '600 10px system-ui';
        g.textBaseline = 'bottom';
        const styles = getComputedStyle(wrap);
        const ok = (styles.getPropertyValue('--xz-ok') || styles.getPropertyValue('--dsw-alias-state-business-primary') || '').trim() || 'CanvasText';
        const danger = (styles.getPropertyValue('--xz-danger') || styles.getPropertyValue('--dsw-alias-state-error-primary') || '').trim() || 'CanvasText';
        const labelBackground = (styles.getPropertyValue('--dsw-specific-sidebar-fill') || '').trim() || 'Canvas';
        for (const det of boxes) {
          const box = det && det.bbox;
          if (!Array.isArray(box) || box.length !== 4) continue;
          const x = box[0] * width;
          const y = box[1] * height;
          const w = Math.max(1, (box[2] - box[0]) * width);
          const h = Math.max(1, (box[3] - box[1]) * height);
          const hot = det.class === 'person';
          const stroke = hot ? danger : ok;
          g.strokeStyle = stroke;
          g.lineWidth = 1.5;
          g.strokeRect(x, y, w, h);
          const label = `${det.class} ${Math.round((det.confidence ?? 0) * 100)}%`;
          const pad = 3;
          const tw = g.measureText(label).width + pad * 2;
          const ty = Math.max(12, y - 4);
          g.fillStyle = labelBackground;
          g.fillRect(x, ty - 12, tw, 14);
          g.fillStyle = stroke;
          g.fillText(label, x + pad, ty);
        }
      }, [boxes]);

      React.useEffect(() => {
        draw();
        const wrap = wrapRef.current;
        if (!wrap || typeof ResizeObserver === 'undefined') return undefined;
        const observer = new ResizeObserver(() => draw());
        observer.observe(wrap);
        const timer = setInterval(draw, 400);
        return () => {
          observer.disconnect();
          clearInterval(timer);
        };
      }, [draw]);

      return React.createElement(
        'div',
        { className: 'xz-frame', ref: wrapRef },
        React.createElement('img', { src: FRAME_URL, alt: 'XuegerOS camera', draggable: false }),
        React.createElement('canvas', { ref: canvasRef }),
        offline ? React.createElement('div', { className: 'xz-offline' }, '相机未连接 · 等待视频流') : null,
      );
    }

    function Cockpit(props) {
      const sessionId = props && props.sessionId;
      const [data, setData] = React.useState(null);
      const [error, setError] = React.useState(null);
      const [note, setNote] = React.useState('');
      const [noteBad, setNoteBad] = React.useState(false);
      const [sending, setSending] = React.useState(false);
      const [linear, setLinear] = React.useState(0.05);
      const [angular, setAngular] = React.useState(0.25);
      const [aiConfirmOpen, setAiConfirmOpen] = React.useState(false);
      const sendingRef = React.useRef(false);
      const holdRef = React.useRef(null);
      const confirmRef = React.useRef(null);

      React.useEffect(() => {
        if (!aiConfirmOpen) return undefined;
        const onKey = (event) => {
          if (event.key === 'Escape') {
            event.preventDefault();
            setAiConfirmOpen(false);
          }
        };
        document.addEventListener('keydown', onKey);
        const timer = setTimeout(() => {
          if (confirmRef.current) confirmRef.current.focus();
        }, 0);
        return () => {
          document.removeEventListener('keydown', onKey);
          clearTimeout(timer);
        };
      }, [aiConfirmOpen]);

      React.useEffect(() => {
        let alive = true;
        let timer;
        const tick = async () => {
          try {
            const next = await getJson(STATE_URL);
            if (!alive) return;
            setData(next);
            setError(null);
          } catch (err) {
            if (!alive) return;
            setError(String(err.message ?? err));
          }
          timer = setTimeout(tick, POLL_MS);
        };
        tick();
        return () => {
          alive = false;
          if (timer) clearTimeout(timer);
        };
      }, []);

      const control = (data && data.control) || {};
      const world = (data && data.world) || {};
      const camera = world.camera || {};
      const mapping = world.mapping || {};
      const micro = world.micro_ros || {};
      const gateway = (data && data.gateway) || {};
      const online = Boolean(data && data.ok && !world.error);
      const gatewayOnline = Boolean(gateway && gateway.enabled === true);
      const shadow = control.shadow !== false;

      const say = (text, bad) => {
        setNote(text);
        setNoteBad(Boolean(bad));
      };

      const pulse = async (lin, ang) => {
        if (sendingRef.current) return;
        sendingRef.current = true;
        setSending(true);
        try {
          const result = await getJson(MOVE_URL, {
            method: 'POST',
            headers: { 'content-type': 'application/json' },
            body: JSON.stringify({ linear: lin, angular: ang, duration_ms: 320, reason: 'cockpit dpad' }),
          });
          if (result.ok) {
            say(result.published ? '已发送到物理后端。' : '影子模式：命令已校验，未驱动物理电机。');
          } else {
            say(`未发送：${result.error || 'unknown'}`, true);
          }
        } catch (err) {
          say(`控制请求失败：${String(err.message ?? err)}`, true);
        } finally {
          sendingRef.current = false;
          setSending(false);
        }
      };

      const stop = async () => {
        try {
          const result = await getJson(STOP_URL, {
            method: 'POST',
            headers: { 'content-type': 'application/json' },
            body: JSON.stringify({ reason: 'cockpit stop' }),
          });
          say(result.ok ? '已停车。' : `停车失败：${result.error || 'unknown'}`, !result.ok);
        } catch (err) {
          say(`停车失败：${String(err.message ?? err)}`, true);
        }
      };

      const beginHold = (lin, ang) => (event) => {
        event.preventDefault();
        holdRef.current = setInterval(() => pulse(lin, ang), 420);
        pulse(lin, ang);
      };

      const endHold = () => {
        const wasHolding = Boolean(holdRef.current);
        if (holdRef.current) {
          clearInterval(holdRef.current);
          holdRef.current = null;
        }
        if (wasHolding) stop();
      };

      React.useEffect(() => () => endHold(), []);

      const setAi = async (patch) => {
        try {
          const body = { ...patch };
          if (patch.motion === true) body.confirm = true;
          const result = await getJson(AI_URL, {
            method: 'POST',
            headers: { 'content-type': 'application/json' },
            body: JSON.stringify(body),
          });
          if (result.ok) say('控制台设置已更新。');
          else say(`设置失败：${result.error || 'unknown'}`, true);
        } catch (err) {
          say(`设置失败：${String(err.message ?? err)}`, true);
        }
      };

      const requestAiMotion = () => {
        if (control.ai_motion) {
          setAi({ motion: false });
          return;
        }
        setAiConfirmOpen(true);
      };

      const confirmAiMotion = async () => {
        setAiConfirmOpen(false);
        await setAi({ motion: true });
      };

      const summon = async () => {
        try {
          const result = await getJson(SUMMON_URL, {
            method: 'POST',
            headers: { 'content-type': 'application/json' },
            body: JSON.stringify(sessionId ? { session_id: sessionId } : {}),
          });
          if (result.ok) say('已把小车世界状态交给 Angela，等待她在对话里接管。');
          else say(`唤醒失败：${result.error || 'unknown'}`, true);
        } catch (err) {
          say(`唤醒失败：${String(err.message ?? err)}`, true);
        }
      };

      const startGateway = async () => {
        try {
          say('正在启动运动网关…');
          const result = await getJson(GATEWAY_URL, { method: 'POST', headers: { 'content-type': 'application/json' }, body: '{}' });
          say(result.ok ? '运动网关已就绪。' : `网关启动失败：${result.error || 'unknown'}`, !result.ok);
        } catch (err) {
          say(`网关启动失败：${String(err.message ?? err)}`, true);
        }
      };

      const toggle = (on, onClick, label) =>
        React.createElement(
          'span',
          {
            className: 'xz-track',
            'data-on': on ? '1' : '0',
            role: 'switch',
            'aria-checked': on ? 'true' : 'false',
            'aria-label': label,
            tabIndex: 0,
            onClick,
            onKeyDown: (event) => {
              if (event.key === 'Enter' || event.key === ' ') {
                event.preventDefault();
                onClick();
              }
            },
          },
          null,
        );

      const objects = world.objects || {};
      const detcount = Object.values(objects).reduce((sum, n) => sum + (Number(n) || 0), 0);
      const cameraStreaming = online && camera.state === 'streaming';
      const cameraLabel = cameraStreaming
        ? 'Streaming'
        : (camera.state === 'connecting' ? '连接中' : (camera.state === 'not_configured' ? '未配置' : '断流'));
      const gatewayLabel = gatewayOnline ? (shadow ? '影子模式' : '物理模式') : '网关离线';
      const overallLabel = !data ? '加载中' : (!online ? '全离线' : (cameraStreaming && gatewayOnline ? '正常' : '部分降级'));
      const inferenceLabel = camera.latency_ms != null ? '就绪' : (camera.state === 'connecting' ? '启动中' : '不可用');

      return React.createElement(
        'div',
        { className: 'xz-root' },
        React.createElement(
          'div',
          { className: 'xz-head' },
          React.createElement('span', { className: 'xz-dot', 'data-ok': cameraStreaming ? '1' : '0' }),
          React.createElement('div', { className: 'xz-grow' },
            React.createElement('div', { className: 'xz-title' }, 'XuegerOS · 车 01'),
            React.createElement('div', { className: 'xz-sub' },
              online ? `${cameraLabel} · ${fmt(camera.fps)} FPS · ${detcount} 个目标` : '车体世界离线'),
          ),
          React.createElement('span', {
            className: 'xz-badge',
            'data-state': overallLabel === '正常' ? 'ok' : (overallLabel === '全离线' ? 'bad' : 'warn'),
            title: gatewayOnline ? `运动网关：${shadow ? '影子' : '物理'}模式` : '运动网关未启动（相机与车体仍可在线）',
          }, overallLabel),
        ),
        React.createElement(
          'div',
          { className: 'xz-body' },
          React.createElement(
            'div',
            { className: 'xz-overview' },
            React.createElement('div', { className: 'xz-overview-item' }, React.createElement('span', { className: 'xz-overview-label' }, '整体'), React.createElement('span', { className: 'xz-overview-value' }, overallLabel)),
            React.createElement('div', { className: 'xz-overview-item' }, React.createElement('span', { className: 'xz-overview-label' }, '相机'), React.createElement('span', { className: 'xz-overview-value' }, cameraLabel)),
            React.createElement('div', { className: 'xz-overview-item' }, React.createElement('span', { className: 'xz-overview-label' }, '网关'), React.createElement('span', { className: 'xz-overview-value' }, gatewayLabel)),
          ),
          React.createElement(
            'div',
            { className: 'xz-card xz-frame-card' },
            React.createElement('div', { className: 'xz-cardhead' }, React.createElement('span', { className: 'xz-cardtitle' }, '实时相机'), React.createElement('span', { className: 'xz-inline-status', 'data-bad': cameraStreaming ? '0' : '1' }, cameraStreaming ? `${fmt(camera.fps)} FPS` : cameraLabel)),
            React.createElement(DetectionLayer, { detections: camera.detections, offline: !cameraStreaming }),
            React.createElement('div', { className: 'xz-hud' }, React.createElement('span', null, `${detcount} 个目标`), React.createElement('span', null, `帧龄 ${fmt(camera.frame_age_ms)} ms`)),
          ),
          React.createElement(
            'div',
            { className: 'xz-card' },
            React.createElement('div', { className: 'xz-cardhead' }, React.createElement('span', { className: 'xz-cardtitle' }, '视觉 / YOLO'), React.createElement('span', { className: 'xz-inline-status', 'data-bad': inferenceLabel === '就绪' ? '0' : '1' }, inferenceLabel)),
            React.createElement('div', { className: 'xz-detlist' },
              detcount === 0
                ? React.createElement('span', { className: 'xz-empty' }, cameraStreaming ? '画面中暂无识别目标' : '相机断流，检测已暂停')
                : Object.entries(objects).map(([label, count]) =>
                    React.createElement('span', { className: 'xz-det', key: label }, React.createElement('i'), `${label} ×${count}`)),
            ),
            React.createElement('div', { className: 'xz-grid' },
              React.createElement('div', { className: 'xz-kv' }, React.createElement('span', { className: 'xz-k' }, '推理延迟'), React.createElement('span', { className: 'xz-v' }, `${fmt(camera.latency_ms)} ms`)),
              React.createElement('div', { className: 'xz-kv' }, React.createElement('span', { className: 'xz-k' }, '帧龄'), React.createElement('span', { className: 'xz-v' }, `${fmt(camera.frame_age_ms)} ms`)),
            ),
          ),
          React.createElement(
            'div',
            { className: 'xz-card' },
            React.createElement('div', { className: 'xz-cardhead' }, React.createElement('span', { className: 'xz-cardtitle' }, '车体'), React.createElement('span', { className: 'xz-inline-status', 'data-bad': micro.session ? '0' : '1' }, micro.session ? '健康' : '降级')),
            React.createElement('div', { className: 'xz-grid' },
              React.createElement('div', { className: 'xz-kv' }, React.createElement('span', { className: 'xz-k' }, 'micro-ROS'), React.createElement('span', { className: 'xz-v', 'data-bad': micro.session ? '0' : '1' }, micro.session ? '会话在线' : '缺失')),
              React.createElement('div', { className: 'xz-kv' }, React.createElement('span', { className: 'xz-k' }, '建图'), React.createElement('span', { className: 'xz-v', 'data-bad': mapping.healthy ? '0' : '1' }, mapping.healthy ? '健康' : '不健康')),
              React.createElement('div', { className: 'xz-kv' }, React.createElement('span', { className: 'xz-k' }, '/scan'), React.createElement('span', { className: 'xz-v', 'data-bad': mapping.scan ? '0' : '1' }, mapping.scan ? '有' : '无')),
              React.createElement('div', { className: 'xz-kv' }, React.createElement('span', { className: 'xz-k' }, '/map · TF'), React.createElement('span', { className: 'xz-v', 'data-bad': mapping.map && mapping.tf ? '0' : '1' }, `${mapping.map ? '有' : '无'} · ${mapping.tf ? '有' : '无'}`)),
            ),
            world.battery
              ? React.createElement('div', { className: 'xz-kv' }, React.createElement('span', { className: 'xz-k' }, '电池'), React.createElement('span', { className: 'xz-v' }, JSON.stringify(world.battery)))
              : null,
          ),
          React.createElement(
            'div',
            { className: 'xz-card' },
            React.createElement('div', { className: 'xz-cardhead' }, React.createElement('span', { className: 'xz-cardtitle' }, '手动点动'), React.createElement('span', { className: 'xz-inline-status', 'data-bad': gatewayOnline ? '0' : '1' }, gatewayOnline ? '可用' : '不可用')),
            React.createElement('div', { className: 'xz-pad' },
              React.createElement('span', null),
              React.createElement('button', { type: 'button', className: 'xz-key', disabled: !gatewayOnline || sending, onPointerDown: beginHold(linear, 0), onPointerUp: endHold, onPointerCancel: endHold, onPointerLeave: endHold, onKeyDown: (event) => { if (!event.repeat) beginHold(linear, 0)(event); }, onKeyUp: endHold, onBlur: endHold, title: '前进（短脉冲）', 'aria-label': '前进' }, '▲'),
              React.createElement('span', null),
              React.createElement('button', { type: 'button', className: 'xz-key', disabled: !gatewayOnline || sending, onPointerDown: beginHold(0, angular), onPointerUp: endHold, onPointerCancel: endHold, onPointerLeave: endHold, onKeyDown: (event) => { if (!event.repeat) beginHold(0, angular)(event); }, onKeyUp: endHold, onBlur: endHold, title: '左转（短脉冲）', 'aria-label': '左转' }, '◀'),
              React.createElement('button', { type: 'button', className: 'xz-key', 'data-stop': '1', disabled: !gatewayOnline, onClick: stop, title: '立即停车', 'aria-label': '立即停车' }, '停车'),
              React.createElement('button', { type: 'button', className: 'xz-key', disabled: !gatewayOnline || sending, onPointerDown: beginHold(0, -angular), onPointerUp: endHold, onPointerCancel: endHold, onPointerLeave: endHold, onKeyDown: (event) => { if (!event.repeat) beginHold(0, -angular)(event); }, onKeyUp: endHold, onBlur: endHold, title: '右转（短脉冲）', 'aria-label': '右转' }, '▶'),
              React.createElement('span', null),
              React.createElement('button', { type: 'button', className: 'xz-key', disabled: !gatewayOnline || sending, onPointerDown: beginHold(-linear, 0), onPointerUp: endHold, onPointerCancel: endHold, onPointerLeave: endHold, onKeyDown: (event) => { if (!event.repeat) beginHold(-linear, 0)(event); }, onKeyUp: endHold, onBlur: endHold, title: '后退（短脉冲）', 'aria-label': '后退' }, '▼'),
              React.createElement('span', null),
            ),
            React.createElement('div', { className: 'xz-row' },
              React.createElement('label', null, '线速'),
              React.createElement('input', { type: 'range', min: 0.03, max: 0.08, step: 0.01, value: linear, 'aria-label': '线速度', onChange: (e) => setLinear(Number(e.target.value)) }),
              React.createElement('span', { className: 'xz-num' }, `${linear.toFixed(2)}`),
            ),
            React.createElement('div', { className: 'xz-row' },
              React.createElement('label', null, '角速'),
              React.createElement('input', { type: 'range', min: 0.1, max: 0.4, step: 0.05, value: angular, 'aria-label': '角速度', onChange: (e) => setAngular(Number(e.target.value)) }),
              React.createElement('span', { className: 'xz-num' }, `${angular.toFixed(2)}`),
            ),
            !gatewayOnline
              ? React.createElement('button', { type: 'button', className: 'xz-btn', onClick: startGateway }, '启动运动网关')
              : null,
            React.createElement('div', { className: 'xz-note' },
              gatewayOnline
                ? `网关 ${shadow ? '影子模式' : '物理模式'} · ${gateway.operator_token_configured ? '操作员令牌已配置' : '令牌缺失'} · lease ${fmt(gateway.lease_max_ms)} ms`
                : '运动网关离线。启动后仍默认影子模式。'),
          ),
          React.createElement(
            'div',
            { className: 'xz-card' },
            React.createElement('div', { className: 'xz-cardhead' }, React.createElement('span', { className: 'xz-cardtitle' }, 'AI 接管'), React.createElement('span', { className: 'xz-inline-status', 'data-bad': control.ai_motion ? '1' : '0' }, control.ai_motion ? '已授权' : '待授权')),
            React.createElement('div', { className: 'xz-switch', 'data-critical': control.ai_motion ? '1' : '0' },
              React.createElement('span', null, '允许 AI 运动'),
              toggle(control.ai_motion, requestAiMotion, '允许 AI 运动'),
            ),
            React.createElement('div', { className: 'xz-switch' },
              React.createElement('span', null, '物理模式（需后端）'),
              toggle(control.live_motion, () => setAi({ live: !control.live_motion }), '物理模式'),
            ),
            React.createElement('div', { className: 'xz-switch' },
              React.createElement('span', null, '注入世界状态到对话'),
              toggle(control.inject_world, () => setAi({ inject: !control.inject_world }), '注入世界状态到对话'),
            ),
            React.createElement('button', { type: 'button', className: 'xz-btn', 'data-primary': '1', onClick: summon, title: '让当前会话里的 Angela 读取车体状态并接管' }, '让 Angela 接管'),
            React.createElement('div', { className: 'xz-note' },
              '召唤会让当前会话的 Angela 读取世界状态并发起一次接管回合；她只能使用有界短脉冲，且影子模式下不会真的驱动电机。'),
          ),
          note
            ? React.createElement('div', { className: 'xz-note', 'data-bad': noteBad ? '1' : '0' }, note)
            : null,
          error
            ? React.createElement('div', { className: 'xz-note', 'data-bad': '1' }, `控制台后端未就绪：${error}`)
            : null,
          aiConfirmOpen
            ? React.createElement(
                'div',
                { className: 'xz-modal-backdrop', role: 'presentation', onClick: () => setAiConfirmOpen(false) },
                React.createElement(
                  'div',
                  { className: 'xz-modal', role: 'dialog', 'aria-modal': 'true', 'aria-labelledby': 'xz-ai-confirm-title', onClick: (event) => event.stopPropagation() },
                  React.createElement('div', { className: 'xz-modal-title', id: 'xz-ai-confirm-title' }, '允许 AI 运动？'),
                  React.createElement('div', { className: 'xz-modal-copy' }, 'Angela 将可以通过有界短脉冲驱动车体。请确认周围无人、轮子可按需架空，并由你在旁边盯守。'),
                  React.createElement('div', { className: 'xz-modal-actions' },
                    React.createElement('button', { type: 'button', className: 'xz-btn', onClick: () => setAiConfirmOpen(false) }, '取消'),
                    React.createElement('button', { type: 'button', className: 'xz-btn', 'data-danger': '1', ref: confirmRef, onClick: confirmAiMotion }, '确认并开启'),
                  ),
                ),
              )
            : null,
        ),
      );
    }

    function apply(ctx) {
      ctx.effect(() => ctx.sidebarRightTabs.register({
        id: ID,
        kind: KIND,
        title: () => '小车',
        guide: [{
          id: 'xuegeros-cockpit',
          title: () => '小车控制台',
          description: () => '实时相机、YOLO 检测与有界运动控制',
        }],
      }), 'dsh-xuegeros: tab type');

      ctx.effect(() => ctx.slots.inject('sidebar.right.pane.tab', () => ctx.slots.register({
        name: 'sidebar.right.pane.tab',
        key: ID,
      }, Cockpit)), 'dsh-xuegeros: cockpit pane');
    }

    exports.inject = ['slots', 'sidebarRightTabs'];
    exports.apply = apply;
    return module.exports;
  },
});
