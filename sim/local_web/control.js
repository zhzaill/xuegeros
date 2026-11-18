const host = document.getElementById('control-fields');
const status = document.getElementById('control-status');
const inputs = new Map();
async function load() {
  try {
    const response = await fetch('/api/control');
    const data = await response.json();
    if (!response.ok || !data.ok) throw new Error(data.error || response.status);
    host.replaceChildren(); inputs.clear();
    for (const [key, spec] of Object.entries(data.schema)) {
      const label = document.createElement('label'); label.textContent = spec.title;
      const input = document.createElement('input');
      input.type = 'number'; input.min = spec.minimum; input.max = spec.maximum;
      input.step = spec.type === 'integer' ? '1' : 'any'; input.value = data.values[key];
      input.title = key; input.setAttribute('aria-label', spec.title);
      label.append(input); host.append(label); inputs.set(key, input);
    }
    status.textContent = data.busy ? '任务结束或停车后可保存' : '';
  } catch (e) { status.textContent = String(e.message || e); }
}
document.getElementById('control-save').addEventListener('click', async () => {
  const values = {};
  for (const [key, input] of inputs) {
    if (!input.value.trim() || !input.checkValidity()) { input.reportValidity(); return; }
    values[key] = Number(input.value);
  }
  try {
    const response = await fetch('/api/control', {method:'POST', headers:{'content-type':'application/json'}, body:JSON.stringify(values)});
    const data = await response.json();
    status.textContent = data.ok ? '已保存并生效' : data.error;
  } catch (e) { status.textContent = String(e.message || e); }
});
document.getElementById('control-reload').addEventListener('click', load);
document.getElementById('control-panel').addEventListener('toggle', function () {
  if (this.open && !inputs.size) void load();
});
fetch('/api/interfaces').then(r => r.json()).then(data => {
  const list = document.getElementById('interfaces');
  for (const route of data.get) {
    const a = document.createElement('a'); a.href = route; a.textContent = `GET ${route}`;
    a.target = '_blank'; a.rel = 'noopener'; a.title = route; list.append(a);
  }
  for (const route of data.post) {
    const code = document.createElement('code'); code.textContent = `POST ${route}`; list.append(code);
  }
  const code = document.createElement('code'); code.textContent = data.robotd; list.append(code);
}).catch(() => {});
