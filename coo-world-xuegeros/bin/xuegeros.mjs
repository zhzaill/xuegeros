#!/usr/bin/env node
import {spawn, spawnSync} from 'node:child_process';
import {fileURLToPath} from 'node:url';
import {existsSync} from 'node:fs';

const runtime = fileURLToPath(new URL('../runtime', import.meta.url));
const candidates = process.env.XUEGEROS_PYTHON ? [[process.env.XUEGEROS_PYTHON, []]]
  : process.platform === 'win32' ? [['py', ['-3']], ['python', []]] : [['python3', []], ['python', []]];
let python;
for (const [exe, prefix] of candidates) {
  const result = spawnSync(exe, [...prefix, '-c', 'import sys; sys.exit(0 if sys.version_info >= (3,10) else 1)'], {stdio:'ignore'});
  if (result.status === 0) { python = [exe, prefix]; break; }
}
const [command = 'help', ...args] = process.argv.slice(2);
if (command === 'help' || command === '--help' || command === '-h') {
  console.log('xuegeros sim [--host 0.0.0.0 --http-port 18142 --port 18242 --world room.world --config <file> --mesh-dir <folder>]\nxuegeros doctor\nxuegeros firewall (Windows Administrator shell)\nRequires Python 3.10+. Set XUEGEROS_PYTHON to select an interpreter.');
} else if (command === 'firewall') {
  if (process.platform !== 'win32') { console.error('This helper is for Windows. Allow TCP 18142/18242 in your local firewall.'); process.exitCode=1; }
  else {
    const result=spawnSync('powershell.exe', ['-NoProfile','-ExecutionPolicy','Bypass','-File',fileURLToPath(new URL('../runtime/tools/enable_sim_lan.ps1', import.meta.url))], {stdio:'inherit'});
    process.exitCode=result.status ?? 1;
  }
} else if (!python) {
  console.error('Python 3.10+ is required. Install Python, then run xuegeros doctor.'); process.exitCode = 1;
} else if (command === 'doctor') {
  const [exe, prefix] = python;
  const pillow = spawnSync(exe, [...prefix, '-c', 'import PIL'], {stdio:'ignore'}).status === 0;
  const ready = existsSync(new URL('../runtime/sim/local_sim.py', import.meta.url));
  console.log(JSON.stringify({python:exe, runtime:ready, mapImages:pillow,
    ...(!pillow ? {installMapImages:`${exe} ${prefix.join(' ')} -m pip install Pillow`} : {})}, null, 2));
  process.exitCode = ready ? 0 : 1;
} else if (command === 'sim') {
  const [exe, prefix] = python;
  const child = spawn(exe, [...prefix, '-u', '-m', 'sim.local_sim', ...args], {cwd:runtime, stdio:'inherit'});
  child.on('error', e => {console.error(e.message); process.exitCode = 1;});
  child.on('exit', code => {process.exitCode = code ?? 0;});
  for (const signal of ['SIGINT','SIGTERM']) process.on(signal, () => { if (!child.killed) child.kill(signal); });
} else { console.error(`Unknown command: ${command}`); process.exitCode = 1; }
