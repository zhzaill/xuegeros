import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import test from 'node:test';
import { EventEmitter } from 'node:events';

import { EventLog } from '../src/events.js';
import { XUEGEROS } from '../src/definition.js';
import { XuegerosWorld } from '../src/world.js';

class FakeClient extends EventEmitter {
  constructor() {
    super();
    this.connected = false;
    this.state = null;
    this.requests = [];
    this.answers = [];
  }

  start() {
    this.connected = true;
    this.emit('open');
  }

  stop() {
    this.connected = false;
    this.emit('close');
  }

  async request(message) {
    this.requests.push(message);
    if (!this.answers.length) throw new Error('unexpected request');
    const answer = this.answers.shift();
    if (answer instanceof Error) throw answer;
    return answer;
  }

  pushState(state) {
    this.state = state;
    this.lastState = state;
    this.emit('state', state);
  }

  pushEvents(items) {
    this.emit('frame', { t: 'events', items });
  }
}

function workspace() {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'xuegeros-world-'));
  return dir;
}

function makeWorld(dir, client) {
  const world = new XuegerosWorld(
    { deployment: 'physical', robotdUrl: 'ws://test', eventLogPath: 'events.jsonl', maxReplay: 50, localConsoleUrl: 'http://127.0.0.1:18112' },
    { dataDir: dir },
    { createClient: () => client },
  );
  return world;
}

const flush = () => new Promise((resolve) => setTimeout(resolve, 5));

test('replay and server backlog notify once across two World restarts', async () => {
  const dir = workspace();
  const events = Array.from({length:16}, (_, i) => ({seq:i+1, ts:100+i, kind:'tour_done', reason:'reached'}));
  const log = new EventLog(path.join(dir, 'events.jsonl'));
  events.forEach(event => log.append(event));
  const pushed = [];
  const host = {pushEvent:async event => pushed.push(event)};
  for (let restart=0; restart<3; restart++) {
    const client = new FakeClient();
    const world = makeWorld(dir, client);
    await world.start(host);
    client.pushEvents(events);
    client.pushEvents(events);
    await world._delivery;
    assert.equal(pushed.length, 16);
    await world.stop();
  }
  const client = new FakeClient();
  const world = makeWorld(dir, client);
  await world.start(host);
  // New daemon: same sequence number but a different timestamp is new work.
  client.pushEvents([{seq:1, ts:1000, kind:'tour_done', reason:'reached'}]);
  await world._delivery;
  assert.equal(pushed.length, 17);
  await world.stop();
});

test('a failed host notification remains pending and can be retried', async () => {
  const dir = workspace();
  const client = new FakeClient();
  const world = makeWorld(dir, client);
  await world.start({pushEvent:async () => {throw new Error('offline');}});
  client.pushEvents([{seq:1, ts:42, kind:'tour_done'}]);
  await world._delivery;
  await world.stop();
  const pushed=[];
  const next = makeWorld(dir, new FakeClient());
  await next.start({pushEvent:async event => pushed.push(event)});
  await next._delivery;
  assert.equal(pushed.length,1);
  await next.stop();
});

test('simulation preflight accepts local and LAN WebSocket and rejects other protocols', () => {
  assert.doesNotThrow(() => XUEGEROS.preflight({ cfg: {
    robotdUrl: 'ws://127.0.0.1:18202/ws',
    deployment: 'simulation',
    simulationRobotdUrl: 'ws://127.0.0.1:18242/ws',
  } }));
  assert.doesNotThrow(() => XUEGEROS.preflight({ cfg: {
    robotdUrl: 'ws://127.0.0.1:18202/ws',
    deployment: 'simulation',
    simulationRobotdUrl: 'ws://192.168.67.166:18242/ws',
  } }));
  assert.throws(() => XUEGEROS.preflight({cfg:{robotdUrl:'ws://127.0.0.1:18202/ws', deployment:'simulation', simulationRobotdUrl:'http://192.168.67.166:18242/ws'}}), /必须是 ws/);
});

test('replays only waking events since the cursor', async () => {
  const dir = workspace();
  const log = new EventLog(path.join(dir, 'events.jsonl'));
  log.append({
    seq: 1,
    kind: 'motion_blocked',
    goal: 'forward',
    reason: 'scan_stale',
    scan_age_s: 2.4,
    scan_limit_s: 1.5,
  });
  log.append({ seq: 2, kind: 'motion_started', goal: 'forward' });
  log.writeCursor(0);

  const pushed = [];
  const host = { pushEvent: async (event, opts) => pushed.push({ event, opts }), log: { info() {}, warn() {} } };
  const world = makeWorld(dir, new FakeClient());
  await world.start(host);
  await flush();

  assert.equal(pushed.length, 1);
  assert.equal(pushed[0].event.type, 'xuegeros.motion');
  assert.match(pushed[0].event.text, /雷达陈旧 2\.4 s/);
  assert.equal(new EventLog(path.join(dir, 'events.jsonl')).readCursor(), 1);
});

test('veto wakes with interrupt, routine motion stays silent', async () => {
  const pushed = [];
  const host = { pushEvent: async (event, opts) => pushed.push({ event, opts }), log: { info() {}, warn() {} } };
  const client = new FakeClient();
  const world = makeWorld(workspace(), client);
  await world.start(host);

  client.pushEvents([
    {
      seq: 10,
      kind: 'motion_vetoed',
      goal: 'forward',
      sector: 6,
      reason: 'obstacle:0.18',
      clearance_m: 0.18,
      threshold_m: 0.25,
    },
    { seq: 11, kind: 'motion_started', goal: 'forward' },
    { seq: 12, kind: 'motion_done', goal: 'forward', reason: 'goal_reached', traveled_m: 0.12 },
  ]);
  await flush();

  assert.equal(pushed.length, 1);
  assert.equal(pushed[0].opts.trigger, 'interrupt');
  assert.match(pushed[0].event.text, /净空 0\.18 m < 阈值 0\.25 m/);
});

test('goal dispatches in one request', async () => {
  const client = new FakeClient();
  const world = makeWorld(workspace(), client);
  client.answers.push({ dispatched: true, skill: 'forward', sector: 6, distance_m: 0.12 });
  await world.start({ log: { info() {}, warn() {} } });

  const result = await world.goal({ intent: 'forward', sector: 6, distance_m: 0.12 });
  assert.equal(result.ok, true);
  assert.equal(result.dispatched, true);
  assert.equal(client.requests.length, 1);
  assert.equal(client.requests[0].t, 'goal');
  assert.equal(client.requests[0].skill ?? client.requests[0].goal, 'forward');
  assert.equal(client.requests[0].source, 'ai');
});

test('simulation deployment uses its isolated loopback endpoint', async () => {
  const client = new FakeClient();
  const world = new XuegerosWorld(
    {
      deployment: 'simulation',
      robotdUrl: 'ws://127.0.0.1:18202/ws',
      simulationRobotdUrl: 'ws://127.0.0.1:18242/ws',
      eventLogPath: 'events.jsonl',
    },
    { dataDir: workspace() },
    { createClient: (options) => { assert.equal(options.url, 'ws://127.0.0.1:18242/ws'); return client; } },
  );
  await world.start({ log: { info() {}, warn() {} } });
  client.pushState({ physical: false, state: 'IDLE', seq: 1, scan: { age_s: 0.1 } });
  client.answers.push({ dispatched: true, physical: false });
  const result = await world.goal({ intent: 'forward', sector: 6, distance_m: 0.1 });
  assert.equal(result.ok, true);
});

test('simulation deployment refuses an unknown or physical endpoint state', async () => {
  const client = new FakeClient();
  const world = new XuegerosWorld(
    { deployment: 'simulation', simulationRobotdUrl: 'ws://127.0.0.1:18242/ws', eventLogPath: 'events.jsonl' },
    { dataDir: workspace() },
    { createClient: () => client },
  );
  await world.start({ log: { info() {}, warn() {} } });
  const result = await world.goal({ intent: 'forward', sector: 6, distance_m: 0.1 });
  assert.equal(result.ok, false);
  assert.equal(result.error, 'simulation_not_confirmed');
  assert.equal(client.requests.length, 0);
});

test('simulation live panel unwraps the WebUI positional argument array', async () => {
  const client = new FakeClient();
  const world = new XuegerosWorld(
    { deployment: 'simulation', simulationRobotdUrl: 'ws://127.0.0.1:18242/ws', eventLogPath: 'events.jsonl' },
    { dataDir: workspace() },
    { createClient: () => client },
  );
  await world.start({ log: { info() {}, warn() {} } });
  client.pushState({ physical: false, state: 'IDLE', seq: 1, scan: { age_s: 0.1 } });
  client.answers.push({ dispatched: true, physical: false });

  const result = await world.webUi().invoke('live', 'goal', [{ intent: 'forward', sector: 6, distance_m: 0.1 }]);

  assert.equal(result.dispatched, true);
  assert.deepEqual(client.requests[0], {
    t: 'goal', goal: 'forward', source: 'ai', sector: 6, distance_m: 0.1,
  });
});

test('a goal that did not dispatch is reported with its reason', async () => {
  const client = new FakeClient();
  const world = makeWorld(workspace(), client);
  client.answers.push({
    dispatched: false,
    reason: 'obstacle:0.2',
    error: 'obstacle:0.2',
    clearance_m: 0.2,
    threshold_m: 0.25,
  });
  await world.start({ log: { info() {}, warn() {} } });

  const result = await world.goal({ intent: 'look', sector: 8 });
  assert.equal(result.ok, false);
  assert.equal(result.dispatched, false);
  assert.equal(result.reason, 'obstacle:0.2');
  assert.equal(client.requests.length, 1);
});

test('env prompt exposes action options, not prose', async () => {
  const client = new FakeClient();
  const world = makeWorld(workspace(), client);
  await world.start({ log: { info() {}, warn() {} } });
  client.pushState({
    seq: 7,
    state: 'IDLE',
    scan: { age_s: 0.2 },
    sectors: [
      { sector: 0, min_m: 2.0 },
      { sector: 6, min_m: 0.9 },
      { sector: 7, min_m: 0.2 },
    ],
  });
  const vars = world.envPromptVars();
  assert.match(vars['xuegeros.status'], /seq 7/);
  assert.match(vars['xuegeros.actions'], /可前进扇区：0, 6/);
  assert.match(vars['xuegeros.actions'], /可原地转向扇区：0, 6/);
});

test('heartbeat stale transition pushes one fault event', async () => {
  const pushed = [];
  const host = { pushEvent: async (event, opts) => pushed.push({ event, opts }), log: { info() {}, warn() {} } };
  const client = new FakeClient();
  const world = makeWorld(workspace(), client);
  await world.start(host);

  const beat = (stale, scan) => client.emit('frame', {
    t: 'heartbeat',
    stale,
    scan_age_s: scan,
    scan_limit_s: 1.5,
    odom_age_s: 0.1,
    odom_limit_s: 1.0,
  });
  beat(false, 0.1);
  await flush();
  assert.equal(pushed.length, 0, 'first heartbeat only initializes');

  beat(true, 2.4);
  await flush();
  assert.equal(pushed.length, 1);
  assert.equal(pushed[0].event.type, 'xuegeros.fault');
  assert.match(pushed[0].event.text, /降级/);

  beat(true, 2.5);
  await flush();
  assert.equal(pushed.length, 1, 'still stale must not repeat');

  beat(false, 0.2);
  await flush();
  assert.equal(pushed.length, 2);
  assert.match(pushed[1].event.text, /恢复/);
});

test('panel stop does not need a proposal', async () => {
  const client = new FakeClient();
  const world = makeWorld(workspace(), client);
  client.answers.push({ accepted: true, state: 'DONE' });
  await world.start({ log: { info() {}, warn() {} } });
  const result = await world.webUi().invoke('live', 'stop');
  assert.equal(result.accepted, true);
  assert.equal(client.requests[0].t, 'stop');
});

test('tour tool sends a strategy command', async () => {
  const client = new FakeClient();
  const world = makeWorld(workspace(), client);
  client.answers.push({ accepted: true, request_id: 't1', physical: false });
  await world.start({ log: { info() {}, warn() {} } });

  const result = await world.tour({ action: 'start' });
  assert.equal(result.accepted, true);
  assert.deepEqual(client.requests[0], { t: 'tour', action: 'start', source: 'ai', strategy: 'tour' });
});

test('simulation tour requires a confirmed shadow and forwards the selected variant', async () => {
  const client = new FakeClient();
  const world = makeWorld(workspace(), client);
  world.cfg.deployment = 'simulation';
  await world.start({ log: { info() {}, warn() {} } });
  for (const action of ['start', 'resume']) {
    const refused = await world.tour({ action });
    assert.equal(refused.error, 'simulation_not_confirmed');
  }
  assert.equal(client.requests.length, 0);
  client.pushState({ physical: false });
  client.answers.push({ accepted: true });
  const answer = await world.webUi().invoke('live', 'tour', [{ action: 'start', variant_id: 'room/default/01' }]);
  assert.equal(answer.accepted, true);
  assert.deepEqual(client.requests[0], { t: 'tour', action: 'start', source: 'ai', strategy: 'tour', variant_id: 'room/default/01' });
});

test('tour_done wakes with raw numbers', async () => {
  const pushed = [];
  const host = { pushEvent: async (event, opts) => pushed.push({ event, opts }), log: { info() {}, warn() {} } };
  const client = new FakeClient();
  const world = makeWorld(workspace(), client);
  await world.start(host);

  client.pushEvents([
    {
      seq: 20,
      kind: 'tour_done',
      reason: 'complete',
      elapsed_s: 42.1,
      dist_to_start_m: 0.1,
      encoder_path_m: 2.4,
      known_delta_start: 180,
    },
  ]);
  await flush();

  assert.equal(pushed.length, 1);
  assert.equal(pushed[0].opts.trigger, 'flush');
  assert.match(pushed[0].event.text, /42\.1 s/);
  assert.match(pushed[0].event.text, /离起点 0\.1 m/);
});

test('source tool rejects traversal and reads an allowed file', async () => {
  const world = makeWorld(workspace(), new FakeClient());
  const denied = await world.sourceFile({ path: '../config/xuegeros.env.ps1' });
  assert.equal(denied.ok, false);
  assert.equal(denied.error, 'path_rejected');

  const allowed = await world.sourceFile({ path: 'agent_control/tour.py' });
  assert.equal(allowed.ok, true);
  assert.match(allowed.text, /TourController/);
});

test('map and screenshot return adapter URLs', async () => {
  const original = globalThis.fetch;
  globalThis.fetch = async (url) => {
    if (String(url).endsWith('/v1/map')) {
      return new Response(JSON.stringify({ explored_ratio: 0.4, width: 384 }), { status: 200 });
    }
    if (String(url).endsWith('/v1/frames/latest.jpg')) {
      return new Response(new Uint8Array([1, 2, 3]), { status: 200, headers: { 'x-frame-seq': '77' } });
    }
    return new Response('{}', { status: 200 });
  };
  try {
    const world = makeWorld(workspace(), new FakeClient());
    const map = await world.mapArtifact();
    assert.equal(map.ok, true);
    assert.match(map.url, /map\.png$/);
    assert.equal(map.meta.explored_ratio, 0.4);

    const shot = await world.screenshot();
    assert.equal(shot.ok, true);
    assert.equal(shot.seq, 77);
    assert.match(shot.url, /\/v1\/frames\/77\.jpg$/);
  } finally {
    globalThis.fetch = original;
  }
});

test('simulation observations use sim images and never contact the physical adapter', async () => {
  const original = globalThis.fetch;
  const urls = [];
  globalThis.fetch = async (url) => {
    urls.push(String(url));
    assert.match(String(url), /^http:\/\/127\.0\.0\.1:18142\//);
    return new Response(JSON.stringify(String(url).endsWith('/api/maps')
      ? {maps:{truth:{provenance:'sdf_truth'}}}
      : {ok:true, kind:'workbench', camera_evidence:false, url:'/api/workbench/frames/12.png'}));
  };
  try {
    const world = makeWorld(workspace(), new FakeClient());
    world.cfg.deployment = 'simulation';
    const map = await world.mapArtifact();
    assert.equal(map.meta.provenance, 'sdf_truth');
    assert.equal(map.url, 'http://127.0.0.1:18142/api/map.png');
    const shot = await world.tools().find(t => t.name === 'xuegeros_screenshot').handler({});
    assert.equal(JSON.parse(shot.text).url, 'http://127.0.0.1:18142/api/workbench/frames/12.png');
    assert.equal((await world.screenshot({kind:'camera_front'})).error, 'camera_unavailable_in_simulation');
    assert.equal((await world.rosFacts()).error, 'ros_unavailable_in_simulation');
    assert.equal((await world.look('看前方')).error, 'camera_unavailable_in_simulation');
    assert.equal(urls.length, 2);
  } finally { globalThis.fetch = original; }
});

test('dispatch is not completion and brief observe includes live movement facts', async () => {
  const client = new FakeClient();
  const world = makeWorld(workspace(), client);
  await world.start({log:{info(){},warn(){}}});
  client.pushState({pose:[.1,.2,0], seq:17, traveled_m:.1, physical:false});
  client.answers.push({dispatched:true});
  assert.equal((await world.goal({intent:'forward',distance_m:.1})).completion_verified, false);
  const observed = await world.observe();
  assert.deepEqual(observed.pose, [.1,.2,0]);
  assert.equal(observed.seq, 17);
});

test('panel images use bounded binary RPC and fixed simulation endpoints', async () => {
  const original = globalThis.fetch;
  const urls = [];
  const png = Buffer.from('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVQIHWP4z8DwHwAFgAI/ScLbtAAAAABJRU5ErkJggg==', 'base64');
  globalThis.fetch = async (url) => {
    urls.push(String(url));
    return new Response(png, {headers:{'content-type':'image/png'}});
  };
  try {
    const world = makeWorld(workspace(), new FakeClient());
    world.cfg.deployment = 'simulation';
    const ui = world.webUi();
    assert.ok(ui.panels[0].postMethods.includes('image'));
    for (const args of [{kind:'workbench',seq:12}, {kind:'truth'}]) {
      const result = await ui.invoke('live', 'image', [args]);
      assert.equal(result.$binary.mime, 'image/png');
      assert.deepEqual(Buffer.from(result.$binary.base64, 'base64'), png);
    }
    assert.deepEqual(urls, ['http://127.0.0.1:18142/api/workbench/frames/12.png', 'http://127.0.0.1:18142/api/map.png']);
    await assert.rejects(ui.invoke('live','image',[{kind:'workbench',seq:'../../secret'}]), /invalid_simulation_image/);
    await assert.rejects(ui.invoke('live','image',[{url:'http://127.0.0.1:18202/'}]), /invalid_simulation_image/);
    world.cfg.deployment = 'physical';
    await assert.rejects(ui.invoke('live','image',[{kind:'truth'}]), /image_requires_simulation/);
    assert.equal(urls.length, 2);
  } finally { globalThis.fetch = original; }
});

test('panel image bridge refuses failed, invalid and oversized image responses', async () => {
  const original = globalThis.fetch;
  const world = makeWorld(workspace(), new FakeClient());
  world.cfg.deployment = 'simulation';
  try {
    for (const [response, reason] of [
      [new Response('', {status:404}), /image_http_404/],
      [new Response('<html>', {headers:{'content-type':'text/html'}}), /image_not_png/],
      [new Response('not a PNG', {headers:{'content-type':'image/png'}}), /image_not_png/],
      [new Response(new Uint8Array(2_000_001), {headers:{'content-type':'image/png'}}), /image_too_large/],
    ]) {
      globalThis.fetch = async () => response;
      await assert.rejects(world.simulationImage({kind:'truth'}), reason);
    }
  } finally { globalThis.fetch = original; }
});
