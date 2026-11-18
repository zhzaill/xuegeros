import fs from 'node:fs/promises';
import path from 'node:path';
import {fileURLToPath} from 'node:url';

const pkg = fileURLToPath(new URL('..', import.meta.url));
const repo = path.resolve(pkg, '..');
const out = path.join(pkg, 'runtime');
const files = [
  ...['__init__.py','policy.py','cerebellum.py','coverage.py','explorer.py','navigation.py',
    'tour.py','protocol.py','robotd.py','wsproto.py','skills.py','skills.json','strategy.py'].map(f => `agent_control/${f}`),
  'sim/__init__.py', 'sim/local_sim.py', 'sim/control.py', 'sim/observation.py',
  'sim/probe/capture.py',
  'tools/enable_sim_lan.ps1',
  'sim/local_web/app.js', 'sim/local_web/index.html', 'sim/local_web/control.js',
  'sim/xuegeros_sim/worlds/room.world', 'sim/xuegeros_sim/worlds/room_open.world',
  'sim/xuegeros_sim/urdf/xuegeros_car.urdf.xacro',
  ...['three.module.js','OrbitControls.js','STLLoader.js','LICENSE'].map(f => `mcp-xuegeros/xuegeros_mcp/vendor/three/${f}`),
];
async function variants(dir) {
  for (const item of await fs.readdir(path.join(repo,dir), {withFileTypes:true})) {
    const name = `${dir}/${item.name}`;
    if (item.isDirectory()) await variants(name);
    else if (item.name.endsWith('.json')) files.push(name);
  }
}
await variants('sim/contracts/variants');
await fs.mkdir(out, {recursive:true});
// Fresh staging prevents an old file from leaking into a later release.
await fs.rm(out, {recursive:true, force:true});
for (const file of files) {
  const target = path.join(out,file);
  await fs.mkdir(path.dirname(target), {recursive:true});
  await fs.copyFile(path.join(repo,file),target);
}
await fs.writeFile(path.join(out,'MANIFEST.json'), JSON.stringify(files.sort(), null, 2) + '\n');
console.log(`Staged ${files.length} runtime files (external meshes and model weights excluded).`);
