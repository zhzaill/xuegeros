import test from 'node:test';
import assert from 'node:assert/strict';
import {XuegerosWorld} from '../src/world.js';
import {CONFIG_GROUP, DEFAULTS} from '../src/config.js';

test('every setting is exposed and a custom HTTP origin is used by tools and panel', async () => {
  for (const key of Object.keys(DEFAULTS)) assert.ok(CONFIG_GROUP.schema.properties[`worlds.xuegeros.${key}`], key);
  const world = new XuegerosWorld({deployment:'simulation', simulationHttpUrl:'http://127.0.0.1:19142'});
  const saved = globalThis.fetch;
  const calls = [];
  globalThis.fetch = async (url, options) => {
    calls.push([String(url), options]);
    return Response.json({ok:true, values:{max_linear_mps:.08}, schema:{}});
  };
  try {
    const panel = world.webUi();
    assert.deepEqual(panel.links, []);
    assert.ok(panel.panels[0].postMethods.includes('startWorkbench'));
    assert.equal(world.webUiState().links.control, 'http://127.0.0.1:19142/');
    await panel.invoke('live', 'control', [{values:{max_linear_mps:.08}}]);
    assert.equal(calls[0][0], 'http://127.0.0.1:19142/api/control');
    assert.equal(calls[0][1].method, 'POST');
    assert.deepEqual(JSON.parse(calls[0][1].body), {max_linear_mps:.08});
    await panel.invoke('live', 'control', []);
    assert.equal(calls[1][1].method, 'GET');
    assert.ok(world.tools().find(t => t.name === 'xuegeros_interfaces'));
  } finally {globalThis.fetch = saved;}
});

test('simulation HTTP configuration accepts LAN but rejects credentials and non-origin paths', () => {
  assert.equal(new XuegerosWorld({simulationHttpUrl:'http://192.168.67.178:18142'}).simHttpUrl(), 'http://192.168.67.178:18142');
  for (const url of ['file:///tmp/sim', 'http://user:pw@127.0.0.1:18142', 'http://127.0.0.1:18142/other']) {
    const world = new XuegerosWorld({simulationHttpUrl:url});
    assert.throws(() => world.simHttpUrl(), /HTTP origin/);
  }
});

test('a physical World does not advertise legacy console shortcuts or simulator launch', async () => {
  const world = new XuegerosWorld({deployment: 'physical', localConsoleUrl: 'http://127.0.0.1:18112'});
  const ui = world.webUi();
  assert.deepEqual(ui.links, []);
  assert.equal(ui.panels[0].postMethods.includes('startWorkbench'), false);
  assert.deepEqual(await ui.invoke('live', 'startWorkbench', []), {ok:false, error:'simulation_only'});
});
