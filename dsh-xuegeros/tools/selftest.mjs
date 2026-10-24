/**
 * dsh-xuegeros selftest — apply() against a mock cordis context.
 *
 * Pattern borrowed from dsh-tts/tools/selftest.mjs (2026-10-16). Run:
 *
 *     node tools/selftest.mjs
 *
 * Everything runs against a throwaway DSH_HOME, no adapter, no gateway, no
 * real DSH. Exits non-zero on the first failed assertion.
 */

import { strict as assert } from 'node:assert';
import { mkdtempSync, rmSync, readFileSync, existsSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { apply } from '../lib/index.js';

const PASS = [];
const fail = (name, error) => {
  console.error(`FAIL ${name}: ${error?.message ?? error}`);
  process.exit(1);
};
const ok = (name) => {
  PASS.push(name);
  console.log(`ok   ${name}`);
};

// --- mock cordis context ---------------------------------------------------

function mockRes() {
  return {
    statusCode: null,
    headers: null,
    body: null,
    ended: false,
    writeHead(status, headers) {
      this.statusCode = status;
      this.headers = headers;
    },
    end(body) {
      this.ended = true;
      if (body) this.body = this.body ? this.body + body : body;
    },
    write() {
      return true;
    },
    once() {},
    on() {},
  };
}

function mockReq(method, body) {
  const listeners = {};
  const req = {
    method,
    setEncoding() {},
    on(event, fn) {
      listeners[event] = fn;
    },
    async settle() {
      if (body !== undefined) listeners.data?.(body);
      listeners.end?.();
    },
  };
  // A real HTTP request delivers its body on its own, without the caller
  // driving it. Handlers await readBody(req), which resolves on 'end' — so if
  // the test forgets settle(), the handler suspends forever, the event loop
  // drains and node exits 0 with the remaining assertions silently unrun.
  // Auto-settling on the next tick mirrors real IO and keeps a forgotten
  // settle() from turning into a false pass. settle() stays idempotent.
  let settled = false;
  const autoSettle = () => {
    if (settled) return;
    settled = true;
    void req.settle();
  };
  if (body !== undefined) setImmediate(autoSettle);
  else setImmediate(() => listeners.end?.());
  return req;
}

function buildCtx({ agents = [], dshHome } = {}) {
  const tools = [];
  const routes = [];
  const effects = [];
  const warnings = [];
  // One stable wrapper per agent, created once (see list() below). Spread the
  // caller's fixture first so the tracking fields always win.
  const wrappers = agents.map((a) => ({
    ...a,
    followupCalls: 0,
    injects: [],
    inject(...args) {
      this.injects.push(args);
      return Promise.resolve();
    },
  }));
  let disposed = 0;
  const ctx = {
    logger: {
      info: () => {},
      warn: (...a) => warnings.push(a.join(' ')),
    },
    get(name) {
      if (name === 'agents') {
        return {
          // Build each wrapper ONCE and hand out the same object on every
          // list() call. The plugin calls list() afresh each time it picks an
          // agent, so a per-call clone would swallow the recorded inject and
          // the assertions would read a throwaway object.
          list: () => wrappers,
        };
      }
      if (name === 'webServer') {
        return {
          register(route) {
            routes.push(route);
            return () => {
              disposed += 1;
            };
          },
        };
      }
      return null;
    },
    tools: {
      register(def) {
        tools.push(def);
        return () => {
          disposed += 1;
        };
      },
    },
    // cordis semantics: effect(callback) invokes the callback immediately and
    // keeps the disposer it RETURNS. The earlier mock only stored the callback,
    // so calling it produced a disposer nobody ran - teardown silently did
    // nothing and every disposal assertion was meaningless.
    effect(callback, name) {
      const disposer = callback();
      effects.push({ fn: callback, disposer, name });
    },
  };
  // Run every registered effect disposer, like a host reload would.
  const disposeAll = () => {
    let ran = 0;
    for (const { disposer } of effects) {
      if (typeof disposer === 'function') {
        disposer();
        ran += 1;
      }
    }
    return ran;
  };
  return { ctx, tools, routes, effects, warnings, wrappers, disposeAll, disposeCount: () => disposed };
}

async function flush(times = 4) {
  for (let i = 0; i < times; i += 1) await new Promise((r) => setTimeout(r, 25));
}

async function run() {
  // Watchdog: a suspended await drains the event loop and node exits 0 with
  // assertions unrun. Any hang must be a loud failure, never a quiet pass.
  const watchdog = setTimeout(() => {
    fail('watchdog', new Error(`selftest hung after ${PASS.length} checks (last: ${PASS.at(-1)})`));
  }, 60_000);
  watchdog.unref?.();

  // 1. fresh install: tools, routes, state file, operator token
  {
    const home = mkdtempSync(join(tmpdir(), 'dsh-xuegeros-test-'));
    process.env.DSH_HOME = home;
    const harness = buildCtx();
    apply(harness.ctx, {});
    await flush(8);

    assert.equal(harness.tools.length, 3, 'expected 3 model tools');
    assert.deepEqual(
      harness.tools.map((t) => t.name),
      ['xuegeros_observe', 'xuegeros_drive', 'xuegeros_stop'],
    );
    ok('fresh: 3 model tools registered');

    const paths = harness.routes.map((r) => r.path);
    for (const p of [
      '/sync/xuegeros/state',
      '/sync/xuegeros/frame',
      '/sync/xuegeros/move',
      '/sync/xuegeros/stop',
      '/sync/xuegeros/ai',
      '/sync/xuegeros/summon',
      '/sync/xuegeros/gateway/start',
    ]) {
      assert.ok(paths.includes(p), `route ${p} missing`);
    }
    ok('fresh: 7 cockpit routes mounted');

    const stateFile = join(home, 'data', 'dsh-xuegeros', 'state.json');
    assert.ok(existsSync(stateFile), 'state.json not persisted');
    const persisted = JSON.parse(readFileSync(stateFile, 'utf8'));
    assert.ok(persisted.operatorToken, 'operator token missing');
    assert.equal(persisted.aiMotion, false);
    assert.equal(persisted.liveMotion, false);
    ok('fresh: state persisted with token, safe defaults');

    // state route answers whether or not the adapter is up. The world block is
    // either a live projection (has device_id) or an explicit error object —
    // what must never happen is a silent empty/null world.
    const stateRoute = harness.routes.find((r) => r.path === '/sync/xuegeros/state');
    const res = mockRes();
    await stateRoute.handler(mockReq('GET'), res);
    const body = JSON.parse(res.body);
    assert.equal(res.statusCode, 200);
    assert.equal(body.ok, true);
    assert.equal(body.control.ai_motion, false);
    assert.ok(body.world && typeof body.world === 'object', 'world block missing');
    assert.ok(
      body.world.error || body.world.device_id,
      'world is neither a live projection nor an explicit error (swallowed failure)',
    );
    ok('state route: 200, world either live or explicitly errored (never silent)');

    // drive blocked without ai_motion
    const driveTool = harness.tools.find((t) => t.name === 'xuegeros_drive');
    const driveResult = JSON.parse(await driveTool.execute({ duration_ms: 300 }));
    assert.equal(driveResult.ok, false);
    assert.equal(driveResult.error, 'ai_motion_disabled');
    ok('xuegeros_drive: blocked while ai_motion disabled');

    // ai route requires confirmation to enable motion
    const aiRoute = harness.routes.find((r) => r.path === '/sync/xuegeros/ai');
    const resNoConfirm = mockRes();
    await aiRoute.handler(mockReq('POST', JSON.stringify({ motion: true })), resNoConfirm);
    assert.equal(resNoConfirm.statusCode, 400);
    assert.equal(JSON.parse(resNoConfirm.body).error, 'confirmation_required');
    ok('ai route: enabling motion without confirm rejected (400)');

    // summon without agents fails cleanly
    const summonRoute = harness.routes.find((r) => r.path === '/sync/xuegeros/summon');
    const resSummon = mockRes();
    await summonRoute.handler(mockReq('POST', '{}'), resSummon);
    assert.equal(JSON.parse(resSummon.body).error, 'no_live_agent');
    ok('summon: no_live_agent when registry empty');

    // teardown disposes everything: 3 tools + 7 routes = 10 registrations
    const beforeTeardown = harness.disposeCount();
    harness.disposeAll();
    const disposed = harness.disposeCount() - beforeTeardown;
    assert.equal(disposed, 10, `expected 10 disposals (3 tools + 7 routes), got ${disposed}`);
    ok(`teardown: disposers ran (${disposed} disposals)`);

    rmSync(home, { recursive: true, force: true });
  }

  // 1b. teardown BEFORE state resolves must not leak routes or timers.
  //     The host disposes a plugin as soon as it unloads; state is read from
  //     disk first, so registration happens in a continuation. Without a
  //     disposed guard, teardown splices an empty array and the continuation
  //     then mounts 7 route handlers that are never unmounted (they leak on
  //     the shared web server across every plugin reload).
  {
    const home = mkdtempSync(join(tmpdir(), 'dsh-xuegeros-test-'));
    process.env.DSH_HOME = home;
    const harness = buildCtx();
    apply(harness.ctx, {});
    // Dispose immediately, before the async state load can finish.
    harness.disposeAll();
    const afterTeardown = harness.routes.length;
    await flush(8);
    assert.equal(
      harness.routes.length,
      afterTeardown,
      'routes mounted after teardown (late-registration leak)',
    );
    assert.equal(
      harness.routes.length,
      0,
      `expected no routes after early teardown, got ${harness.routes.length}`,
    );
    ok('early teardown: no routes leak past disposal');
    rmSync(home, { recursive: true, force: true });
  }

  // 2. stale liveMotion must be cleared at startup (fail closed) when no
  //    physical gateway is reachable — the safety lie that lost its process.
  //    Point the plugin at a closed port: this test must assert the fail-closed
  //    LOGIC, not whatever gateway happens to be running on the dev machine.
  {
    const home = mkdtempSync(join(tmpdir(), 'dsh-xuegeros-test-'));
    process.env.DSH_HOME = home;
    const dataDir = join(home, 'data', 'dsh-xuegeros');
    const { mkdirSync, writeFileSync } = await import('node:fs');
    mkdirSync(dataDir, { recursive: true });
    writeFileSync(
      join(dataDir, 'state.json'),
      JSON.stringify({
        version: 1,
        aiMotion: true,
        injectWorld: true,
        operatorToken: 'seeded-token',
        liveMotion: true,
        gatewayStartedHere: true,
      }),
    );
    const harness = buildCtx();
    // gatewayUrl is plugin config, not persisted state; point it at a closed
    // port so this asserts the fail-closed logic rather than whichever gateway
    // happens to be running on this machine.
    apply(harness.ctx, { gatewayUrl: 'http://127.0.0.1:9' });
    await flush(8);
    const persisted = JSON.parse(readFileSync(join(dataDir, 'state.json'), 'utf8'));
    assert.equal(persisted.liveMotion, false, 'stale liveMotion survived restart');
    assert.equal(persisted.gatewayStartedHere, false, 'stale gatewayStartedHere survived restart');
    assert.equal(persisted.aiMotion, true, 'aiMotion is a cockpit choice and must survive');
    assert.equal(persisted.operatorToken, 'seeded-token', 'token must be preserved');
    ok('startup: stale liveMotion cleared fail-closed, cockpit choices preserved');

    // summon against a mock agent must NOT call followup (session-corrupting API)
    const agent = { id: 'sess-1', status: 'running', followup: async () => { throw new Error('followup must not be called'); } };
    const harness2 = buildCtx({ agents: [agent], dshHome: home });
    // re-apply a fresh instance on the same home
    apply(harness2.ctx, { gatewayUrl: 'http://127.0.0.1:9' });
    await flush(8);
    const summonRoute2 = harness2.routes.find((r) => r.path === '/sync/xuegeros/summon');
    const res2 = mockRes();
    await summonRoute2.handler(mockReq('POST', JSON.stringify({ session_id: 'sess-1' })), res2);
    const body2 = JSON.parse(res2.body);
    assert.equal(body2.ok, true, `summon failed: ${body2.error}`);
    assert.equal(body2.mechanism, 'inject', 'summon must use inject, not followup');
    const recorded = harness2.wrappers[0];
    assert.equal(recorded.injects.length, 1, 'agent did not receive the summon inject');
    // Adapter is down in this test, so the brief renders the degraded variant
    // (【XuegerOS】车体世界不可用…) — both variants must carry the bracket prefix.
    assert.match(recorded.injects[0][0].content[0].text, /【XuegerOS】/);
    ok('summon: uses inject only (followup banned, session corruption prevented)');

    rmSync(home, { recursive: true, force: true });
  }

  // 3. A listening gateway that rejects our operator token must NOT be reported
  //    as "already started". That is how the cockpit became permanently unable
  //    to stop the car: /v1/status is public, so a foreign-token gateway looked
  //    healthy, startGateway() returned already:true, and every stop then
  //    failed 403 with no path to recovery from the UI.
  {
    const home = mkdtempSync(join(tmpdir(), 'dsh-xuegeros-test-'));
    process.env.DSH_HOME = home;
    const { createServer } = await import('node:http');
    // Stand in for a gateway with someone else's token: status answers, stop 403s.
    const foreign = createServer((req, res) => {
      if (req.url === '/v1/status') {
        res.writeHead(200, { 'content-type': 'application/json' });
        res.end(JSON.stringify({ enabled: true, shadow_mode: true, operator_token_configured: true }));
        return;
      }
      res.writeHead(403, { 'content-type': 'application/json' });
      res.end(JSON.stringify({ error: 'operator_authorization_required' }));
    });
    await new Promise((r) => foreign.listen(0, '127.0.0.1', r));
    const port = foreign.address().port;

    const harness = buildCtx();
    apply(harness.ctx, { gatewayUrl: `http://127.0.0.1:${port}` });
    await flush(8);

    const startRoute = harness.routes.find((r) => r.path === '/sync/xuegeros/gateway/start');
    const res = mockRes();
    await startRoute.handler(mockReq('POST', '{}'), res);
    const body = JSON.parse(res.body);
    assert.equal(
      body.ok,
      false,
      'foreign-token gateway wrongly reported as already started (silent 403 later)',
    );
    assert.equal(body.error, 'gateway_token_mismatch', `unexpected error: ${body.error}`);
    ok('gateway start: foreign-token gateway named, not silently accepted');

    await new Promise((r) => foreign.close(r));
    rmSync(home, { recursive: true, force: true });
  }

  clearTimeout(watchdog);
  // An early drain means some await never resolved; refuse to report success.
  const REQUIRED = 12;
  assert.equal(PASS.length, REQUIRED, `only ${PASS.length}/${REQUIRED} checks ran`);
  console.log(`\n${PASS.length} checks passed`);
}

run().catch((e) => fail('selftest', e));
