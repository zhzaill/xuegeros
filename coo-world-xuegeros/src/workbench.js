import {execFile, spawn} from 'node:child_process';
import {existsSync} from 'node:fs';
import path from 'node:path';
import {setTimeout as sleep} from 'node:timers/promises';
import {promisify} from 'node:util';

const run = promisify(execFile);
const loopback = new Set(['localhost', '127.0.0.1']);

async function findPython() {
  const candidates = process.env.XUEGEROS_PYTHON ? [[process.env.XUEGEROS_PYTHON]]
    : process.platform === 'win32' ? [['py', '-3'], ['python']] : [['python3'], ['python']];
  for (const [command, ...prefix] of candidates) {
    try {
      const {stdout} = await run(command, [...prefix, '-E', '-c',
        'import sys; sys.exit(1) if sys.version_info < (3, 10) else print(sys.executable)'],
      {timeout: 2500, windowsHide: true});
      const executable = stdout.trim();
      if (path.isAbsolute(executable) && existsSync(executable)) return executable;
    } catch {
      // Try the next platform interpreter.
    }
  }
  return null;
}

async function hasWorkbench(http, ws) {
  try {
    const response = await fetch(new URL('/api/interfaces', http), {
      signal: AbortSignal.timeout(750), redirect: 'error',
    });
    if (!response.ok) return false;
    const api = await response.json();
    const advertised = new URL(api.robotd);
    return api.get?.includes('/api/control') && advertised.protocol === 'ws:'
      && advertised.port === ws.port && advertised.pathname === ws.pathname;
  } catch {
    return false;
  }
}

export class Workbench {
  constructor() {
    this.child = null;
    this.pending = null;
    this.lifetime = new AbortController();
  }

  start(options) {
    if (!this.pending) {
      this.pending = this.prepare(options).catch(() => ({ok: false, error: 'workbench_start_failed'}))
        .finally(() => { this.pending = null; });
    }
    return this.pending;
  }

  async prepare({http, ws, runtime, connected}) {
    const signal = this.lifetime.signal;
    const ready = () => ({ok: true, url: http.href});
    if (connected() && await hasWorkbench(http, ws)) return ready();
    const local = loopback.has(http.hostname) && loopback.has(ws.hostname);
    if (!local) return {ok: false, error: 'remote_workbench_offline'};
    if (!http.port || !ws.port || ws.protocol !== 'ws:' || ws.pathname !== '/ws'
        || ws.username || ws.password || ws.search || ws.hash
        || http.port === ws.port || [http.port, ws.port].includes('18202')) {
      return {ok: false, error: 'workbench_address_invalid'};
    }
    if (!existsSync(path.join(runtime, 'sim', 'local_sim.py'))) {
      return {ok: false, error: 'workbench_runtime_missing'};
    }

    // Probe before starting so an existing CLI or scheduled instance stays
    // independent of this World, even while its robotd connection is coming up.
    const existing = await hasWorkbench(http, ws);
    if (!existing && !this.child) {
      const python = await findPython();
      if (signal.aborted) return {ok: false, error: 'workbench_start_failed'};
      if (!python) return {ok: false, error: 'python_required'};
      const child = spawn(python, ['-E', '-m', 'sim.local_sim', '--host', '127.0.0.1',
        '--http-port', http.port, '--port', ws.port], {
        cwd: runtime, stdio: 'ignore', windowsHide: true,
      });
      this.child = child;
      const clear = () => { if (this.child === child) this.child = null; };
      child.once('error', clear);
      child.once('exit', clear);
    }

    const deadline = Date.now() + 12000;
    while (Date.now() < deadline) {
      if (connected() && await hasWorkbench(http, ws)) return ready();
      if (!existing && !this.child) return {ok: false, error: 'workbench_port_or_start_failed'};
      await sleep(200, undefined, {signal});
    }
    return {ok: false, error: 'workbench_timeout'};
  }

  stop() {
    this.lifetime.abort();
    this.child?.kill();
  }
}
