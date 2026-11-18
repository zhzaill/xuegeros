import * as THREE from "three";
import { OrbitControls } from "/vendor/three/OrbitControls.js";
import { STLLoader } from "/vendor/three/STLLoader.js";

const canvas = document.getElementById("view");
const embedded = new URLSearchParams(location.search).get('embed') === '1';
document.body.classList.toggle('embedded', embedded);
let following = false;
let viewMode = 'near';
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
renderer.setClearColor(0x151d18, 1);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x151d18);
scene.fog = new THREE.Fog(0x151d18, 9, 18);
const camera = new THREE.PerspectiveCamera(42, 1, 0.02, 40);
camera.position.set(4.5, 5.0, 5.5);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.target.set(0, 0, 0);
scene.add(new THREE.HemisphereLight(0xd7eadc, 0x252d29, 2.2));
const key = new THREE.DirectionalLight(0xfff3d8, 2.1);
key.position.set(-3, 7, 4);
key.castShadow = true;
key.shadow.mapSize.set(1024, 1024);
scene.add(key);

let ground;
let grid;
let room = new THREE.Group();
scene.add(room);
function renderScene(description) {
  while (room.children.length) {
    const child = room.children.pop();
    child.geometry?.dispose();
    if (Array.isArray(child.material)) child.material.forEach((material) => material.dispose());
    else child.material?.dispose();
  }
  ground?.geometry.dispose();
  ground?.material.dispose();
  if (grid) scene.remove(grid);
  const width = Number(description.width || 6);
  const height = Number(description.height || 6);
  ground = new THREE.Mesh(new THREE.BoxGeometry(width, 0.02, height), new THREE.MeshStandardMaterial({ color:0x657269, roughness:0.96 }));
  ground.position.y = -0.015;
  ground.receiveShadow = true;
  scene.add(ground);
  grid = new THREE.GridHelper(Math.max(width, height), Math.ceil(Math.max(width, height) * 5), 0x9baa9e, 0x66736a);
  grid.position.y = -0.003;
  scene.add(grid);
  for (const item of description.obstacles || []) {
    const color = new THREE.Color(item.color || "#68736c");
    let mesh;
    if (item.type === "box") {
      const [sx, sz, sy] = item.size;
      mesh = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), new THREE.MeshStandardMaterial({ color, roughness:0.82 }));
      mesh.rotation.y = Number(item.yaw || 0);
    } else if (item.type === "cylinder") {
      mesh = new THREE.Mesh(new THREE.CylinderGeometry(item.radius, item.radius, item.height, 32), new THREE.MeshStandardMaterial({ color, roughness:0.76 }));
    } else continue;
    mesh.name = item.name;
    mesh.position.set(item.x, item.z, -item.y);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    room.add(mesh);
  }
  camera.position.set(width * 0.72, Math.max(width, height) * 0.8, height * 0.9);
  controls.target.set(0, 0.15, 0);
  controls.update();
  if (viewMode === 'near') setNearView();
  else if (following) setView(true);
  document.getElementById("limits").textContent = description.approximations?.join("；") || "后端未提供能力说明";
  document.getElementById("backend").textContent = description.backend === "kinematic-shadow" ? "运动学影子后端 · 非 Gazebo 物理" : description.backend;
}

const robot = new THREE.Group();
const fallbackModel = new THREE.Group();
robot.add(fallbackModel);
// The nine visuals hang off base_link (mesh_xyz in URDF millimetres), while this
// group lives in base_footprint (ground) coordinates: the base_link height has to be
// added or the whole body renders 62.05 mm too low, sunk into the floor with the
// wheel meshes poking up past the chassis.
const BASE_Z = 0.06205; // base_footprint -> base_link
const robotModel = new THREE.Group();
robotModel.rotation.x = -Math.PI / 2; // URDF is z-up, three.js is y-up
robotModel.position.set(0.00225, BASE_Z - 0.00925, 0);
// meshes are rebuilt per load so a scene/model swap cannot stack duplicates
let meshFrame = null;
// axle line in the *mesh* frame (metres): URDF puts it at base_link (x=-50.75,
// z=-29.55) mm, which maps to mesh (y=53, z=-20.3) mm after the -90 deg yaw and the
// mesh_xyz offset. The wheel cylinder axis is the mesh x axis.
let wheelPivot = null;
const WHEEL_AXLE = { y: 0.053, z: -0.0203 };
const WHEEL_RADIUS = 0.0325;
const chassis = new THREE.Mesh(new THREE.BoxGeometry(0.145, 0.055, 0.095), new THREE.MeshStandardMaterial({ color:0x2775df, metalness:0.16, roughness:0.52 }));
chassis.position.y = 0.055;
fallbackModel.add(chassis);
const top = new THREE.Mesh(new THREE.BoxGeometry(0.078, 0.018, 0.062), new THREE.MeshStandardMaterial({ color:0x56636a, metalness:0.42, roughness:0.48 }));
top.position.set(-0.004, 0.092, 0);
fallbackModel.add(top);
const lidar = new THREE.Mesh(new THREE.CylinderGeometry(0.021, 0.021, 0.026, 20), new THREE.MeshStandardMaterial({ color:0x171d20, metalness:0.3 }));
lidar.position.set(-0.023, 0.112, 0);
fallbackModel.add(lidar);
const cameraBlock = new THREE.Mesh(new THREE.BoxGeometry(0.024, 0.018, 0.022), new THREE.MeshStandardMaterial({ color:0x171d20 }));
cameraBlock.position.set(0.061, 0.085, 0);
fallbackModel.add(cameraBlock);
// Fallback wheels only, used when the STL assets are missing. Once the real
// drive_wheels.stl loads this rig is hidden and must not be trusted for placement:
// the URDF joint origins (±0.081) sit at the axle *ends*, while the wheel mesh's
// lateral mass is centred near ±0.066.
const wheelPivots = [];
const wheelRig = new THREE.Group();
robot.add(wheelRig);
for (const side of [-1, 1]) {
  const pivot = new THREE.Group();
  // URDF axis (0,1,0) maps to Three's local z axis after the z-up conversion.
  pivot.position.set(-0.05075, 0.06205 - 0.02955, -side * 0.081);
  const wheel = new THREE.Mesh(new THREE.CylinderGeometry(0.0325, 0.0325, 0.02, 28), new THREE.MeshStandardMaterial({ color:0x202322, roughness:0.94 }));
  wheel.rotation.x = Math.PI / 2;
  pivot.add(wheel);
  pivot.traverse((object) => { if (object.isMesh) object.castShadow = true; });
  wheelRig.add(pivot);
  wheelPivots.push(pivot);
}
const front = new THREE.ArrowHelper(new THREE.Vector3(1,0,0), new THREE.Vector3(0,0.08,0), 0.2, 0xffce65, 0.045, 0.028);
fallbackModel.add(front);
robot.add(robotModel);
scene.add(robot);

const rays = new THREE.Group();
scene.add(rays);
const origin = new THREE.Vector3(0, 0.012, 0);
function paintScan(values = []) {
  rays.clear();
  const material = new THREE.LineBasicMaterial({ color:0xf0c65d, transparent:true, opacity:0.35 });
  for (let i = 0; i < values.length; i += 4) {
    const raw = values[i];
    if (!(typeof raw === "number" && Number.isFinite(raw))) continue;
    const angle = -Math.PI + i * 2 * Math.PI / values.length;
    const local = new THREE.Vector3(Math.cos(angle) * Math.min(raw, 1.8), 0.126, -Math.sin(angle) * Math.min(raw, 1.8));
    local.applyAxisAngle(new THREE.Vector3(0,1,0), robot.rotation.y);
    const sensor = new THREE.Vector3(-0.0227, 0.126, 0)
      .applyAxisAngle(new THREE.Vector3(0,1,0), robot.rotation.y).add(robot.position);
    const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints([sensor, sensor.clone().add(local)]), material);
    rays.add(line);
  }
}

function resize() {
  const w = canvas.clientWidth || innerWidth;
  const h = canvas.clientHeight || innerHeight * 0.65;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
addEventListener("resize", resize);
resize();

const ui = {
  health:document.getElementById("health"), state:document.getElementById("state"), pose:document.getElementById("pose"),
  yaw:document.getElementById("yaw"), distance:document.getElementById("distance"), physical:document.getElementById("physical"),
  model:document.getElementById("model"), mesh:document.getElementById("mesh"), result:document.getElementById("result"),
};
let lastPose = null;
let lastStateSeq = null;
function setNearView() {
  setView(true);
  following = false;
  viewMode = 'near';
  document.getElementById('follow-car').setAttribute('aria-pressed', 'false');
}
function setView(follow) {
  following = follow;
  viewMode = follow ? 'follow' : 'room';
  document.getElementById('follow-car').setAttribute('aria-pressed', String(follow));
  if (follow) {
    controls.target.copy(robot.position).add(new THREE.Vector3(0, .06, 0));
    camera.position.copy(controls.target).add(new THREE.Vector3(.7, .65, .8));
  } else {
    camera.position.set(4.5, 5, 5.5);
    controls.target.set(0, .15, 0);
  }
  controls.update();
}
document.getElementById('follow-car').addEventListener('click', () => setView(true));
document.getElementById('near-car').addEventListener('click', setNearView);
document.getElementById('room-view').addEventListener('click', () => setView(false));
new ResizeObserver(resize).observe(canvas);
// Backend state frame stamp (epoch seconds); exposed so a screenshot receipt can
// state the real age of the pose it shows instead of guessing.
let lastStateTs = null;
let socket;
const pending = new Map();
let seq = 0;
async function connect() {
  let endpoint;
  try {
    endpoint = (await (await fetch('/api/interfaces')).json()).robotd;
  } catch { ui.health.textContent = '连接接口读取失败'; setTimeout(connect, 1500); return; }
  socket = new WebSocket(endpoint);
  socket.addEventListener("open", () => { ui.health.textContent = "本机 robotd 已连接"; ui.health.dataset.online = "true"; });
  socket.addEventListener("close", () => { ui.health.textContent = "等待本机 robotd"; ui.health.dataset.online = "false"; setTimeout(connect, 1000); });
  socket.addEventListener("error", () => socket.close());
  socket.addEventListener("message", (event) => {
    let frame;
    try { frame = JSON.parse(event.data); } catch { return; }
    if (frame.t === "state") paint(frame);
    if (frame.t === "scan") paintScan(frame.ranges);
    if (frame.t === "answer" && frame.request && pending.has(frame.request)) {
      const queue = pending.get(frame.request);
      const resolve = queue.shift();
      if (!queue.length) pending.delete(frame.request);
      resolve(frame);
    }
  });
}
function request(message) {
  return new Promise((resolve, reject) => {
    const key = message.t;
    const queue = pending.get(key) || [];
    queue.push(resolve); pending.set(key, queue);
    if (!socket || socket.readyState !== WebSocket.OPEN) { reject(new Error("robotd_unavailable")); return; }
    socket.send(JSON.stringify(message));
    setTimeout(() => reject(new Error(`timeout:${key}`)), 4500);
  });
}
function paint(frame) {
  if (lastStateTs && Number(frame.ts) < lastStateTs) return;
  const pose = Array.isArray(frame.pose) ? frame.pose : [0,0,0];
  lastStateSeq = frame.seq;
  if (!lastPose && viewMode === 'near') {
    robot.position.set(pose[0], 0, -pose[1]);
    setNearView();
  }
  lastStateTs = Number.isFinite(Number(frame.ts)) ? Number(frame.ts) : lastStateTs;
  // Sim frames are ROS-like (x forward, y left, yaw CCW about +z). Three.js is y-up,
  // so the ground plane maps (x, y) -> (x, 0, -y) — the same mirror the room and the
  // scan rays already use. Using +y here flipped the car against its own heading, so
  // it visually drove sideways while the odometry said it went forward.
  robot.position.set(pose[0], 0, -pose[1]);
  if (following) {
    if (!lastPose) setView(true);
    else {
      const delta = new THREE.Vector3(pose[0] - lastPose[0], 0, -(pose[1] - lastPose[1]));
      camera.position.add(delta);
      controls.target.add(delta);
    }
  }
  robot.rotation.y = pose[2];
  if (lastPose) {
    const dx = pose[0] - lastPose[0], dy = pose[1] - lastPose[1];
    const signedDistance = dx * Math.cos(lastPose[2]) + dy * Math.sin(lastPose[2]);
    // Cosmetic only: the real wheel mesh spins about its own axle line.
    if (wheelPivot) wheelPivot.rotation.x -= signedDistance / WHEEL_RADIUS;
    for (const pivot of wheelPivots) pivot.rotation.z += signedDistance / WHEEL_RADIUS;
  }
  lastPose = pose;
  ui.state.textContent = `${frame.state || "—"} · ${frame.reason || ""}`;
  ui.pose.textContent = `${pose[0].toFixed(3)} / ${pose[1].toFixed(3)} m`;
  ui.yaw.textContent = `${(pose[2] * 180 / Math.PI).toFixed(1)}°`;
  ui.distance.textContent = `${Number(frame.traveled_m || 0).toFixed(3)} m`;
  ui.physical.textContent = String(frame.physical);
  ui.physical.style.color = frame.physical === false ? "#75d29b" : "#ff817d";
  document.getElementById('live-pose').textContent = `x ${pose[0].toFixed(3)} · y ${pose[1].toFixed(3)} m · ${(pose[2]*180/Math.PI).toFixed(1)}° · ${frame.state} · 单步 ${Number(frame.traveled_m || 0).toFixed(3)} m · #${frame.seq}`;
}
async function loadModel() {
  const response = await fetch("/api/model");
  const model = await response.json();
  ui.model.textContent = "URDF 145×95 mm";
  if (!model.mesh_available) {
    ui.mesh.textContent = "程序化外形（仓库未含 STL）";
    return;
  }
  // Rebuild both groups so a reload cannot stack duplicate meshes.
  if (meshFrame) robotModel.remove(meshFrame);
  meshFrame = new THREE.Group();
  // The exports carry their length along +Y while base_link points +X; the xacro
  // compensates with mesh_rpy="0 0 1.5708" on every <mesh>.
  meshFrame.rotation.z = Math.PI / 2;
  robotModel.add(meshFrame);
  wheelPivot = new THREE.Group();
  wheelPivot.position.set(0, WHEEL_AXLE.y, WHEEL_AXLE.z);
  const wheelOffset = new THREE.Group();
  wheelOffset.position.set(0, -WHEEL_AXLE.y, -WHEEL_AXLE.z);
  wheelPivot.add(wheelOffset);
  meshFrame.add(wheelPivot);
  wheelRig.visible = false;
  fallbackModel.visible = false;
  const loader = new STLLoader();
  const colors = { blue:0x2775df, matte_black:0x202322, dark_gray:0x343b3e, silver:0xaeb6bb };
  for (const mesh of model.meshes) {
    const geometry = await new Promise((resolve, reject) => loader.load(mesh.url, resolve, undefined, reject));
    // The exports are in millimetres and share one assembly frame, which is why the
    // xacro declares scale="0.001 0.001 0.001" on every <mesh>. Without this the car
    // renders a thousand times too large and the camera ends up inside its interior.
    geometry.scale(0.001, 0.001, 0.001);
    geometry.computeBoundingSphere();
    const visual = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({color:colors[mesh.material] ?? 0x9aa4b2, metalness:0.18, roughness:0.68}));
    visual.castShadow = true;
    visual.receiveShadow = true;
    // The wheel mesh is the real geometry: never replace it with cylinders whose
    // spacing comes from the URDF axle ends rather than the wheel centres.
    if (mesh.name === "drive_wheels.stl") wheelOffset.add(visual);
    else meshFrame.add(visual);
  }
  ui.mesh.textContent = `${model.meshes.length} 个 STL（含车轮网格）`;
}
// docs/SIM-CONTRACT.md §1: while a strategy (tour/coverage) holds the goal, the
// single-step buttons must go grey instead of racing it. The判据 is the backend's
// own busy flag, not a guess — and while /api/strategy is not deployed yet (404)
// this degrades to "no strategy running" with one muted line, never a red error.
let strategyBusy = false;
let strategyAvailable = false;
let strategyPending = false;
let strategySnapshot = null;
const phaseLabels = { IDLE:"未开始", STARTING:"准备中", RUNNING:"巡游中", PAUSED:"已暂停", BLOCKED:"受阻，已停车", TIMEOUT:"任务超时", HANDOVER:"需要接管", CANCELLED:"已取消" };
const stageLabels = { EXPLORE:"探索房间", RETURN:"返回起点", CHECK_REAR:"检查后方", REVERSE:"后退脱困", SELECT_EXIT:"选择出口", TURN:"转向出口", EXIT:"离开障碍", REPLAN:"重新规划", FAILED:"脱困失败", DONE:"完成" };
function goalButtons() {
  return [...document.querySelectorAll("[data-goal]")];
}
function applyGoalButtons() {
  for (const button of goalButtons()) button.disabled = strategyBusy || strategyPending;
  document.getElementById("scene-select").disabled = strategyBusy || strategyPending;
  for (const button of document.querySelectorAll("[data-tour]")) {
    const action = button.dataset.tour;
    const phase = strategySnapshot?.phase;
    button.disabled = !strategyAvailable || strategyPending || (
      action === "start" ? strategyBusy : action === "pause" ? phase !== "RUNNING" :
      action === "resume" ? !["PAUSED", "BLOCKED"].includes(phase) : !strategyBusy);
  }
}
function setStrategyLine(text, level = "muted") {
  const line = document.getElementById("strategy");
  if (!line) return;
  line.textContent = text;
  line.dataset.level = level;
}
async function refreshStrategy() {
  let response;
  try {
    response = await fetch("/api/strategy");
  } catch (error) {
    strategyAvailable = false;
    strategyBusy = true;
    applyGoalButtons();
    setStrategyLine("任务连接中断；请等待重连或停车", "warn");
    return;
  }
  if (response.status === 404) {
    strategyBusy = false;
    strategyAvailable = false;
    applyGoalButtons();
    setStrategyLine("策略层未接入（/api/strategy 404）：单步按钮可用", "muted");
    return;
  }
  if (!response.ok) {
    strategyAvailable = false;
    strategyBusy = true;
    applyGoalButtons();
    setStrategyLine(`任务状态读取失败：${response.status}`, "warn");
    return;
  }
  let payload;
  try {
    payload = await response.json();
  } catch (error) {
    strategyAvailable = false;
    strategyBusy = true;
    applyGoalButtons();
    setStrategyLine("任务回执无法解析；请刷新或停车", "warn");
    return;
  }
  strategyBusy = payload.busy === true;
  strategyAvailable = payload.physical === false && payload.available !== false;
  strategySnapshot = payload;
  applyGoalButtons();
  const backend = payload.backend || {};
  const stage = backend.navigation?.recovery?.stage || backend.phase;
  const home = backend.return?.dist_to_start_m ?? backend.dist_to_home_m;
  document.getElementById("tour-stage").textContent = stageLabels[stage] || stage || "—";
  document.getElementById("tour-time").textContent = `${Number(payload.elapsed_s || 0).toFixed(1)} s`;
  document.getElementById("tour-home").textContent = home == null ? "—" : `${Number(home).toFixed(3)} m`;
  document.getElementById("tour-known").textContent = payload.facts?.known_cells == null ? "—" : `${payload.facts.known_cells}（射线已观测）`;
  document.getElementById("tour-travel").textContent = `${Number(payload.facts?.traveled_m || 0).toFixed(3)} m`;
  const label = payload.phase === "DONE" ? (payload.reason === "reached" ? "巡游完成 · 已返回起点 · 验收通过" : payload.reason === "exhausted" ? "任务结束 · 指标未达成" : "已停车") : phaseLabels[payload.phase] || payload.phase;
  setStrategyLine(`${label}${["BLOCKED", "TIMEOUT", "HANDOVER"].includes(payload.phase) ? ` · ${payload.reason}` : ""}`, strategyBusy ? "busy" : "idle");
}
refreshStrategy();
setInterval(refreshStrategy, 1000);

for (const button of document.querySelectorAll("[data-tour]")) {
  button.addEventListener("click", async () => {
    strategyPending = true;
    applyGoalButtons();
    try {
      const action = button.dataset.tour;
      const scene = sceneSelect.value;
      const body = action === "start" ? {strategy:"tour", variant_id:scene === "room_open" ? "room_open/wide/01" : "room/default/01"} : {};
      const response = await fetch(`/api/strategy/${action}`, {method:"POST", headers:{"content-type":"application/json"}, body:JSON.stringify(body)});
      const result = await response.json();
      ui.result.textContent = response.ok && result.accepted !== false ? "操作已接收" : `操作未执行：${result.error || result.reason || response.status}`;
      ui.result.dataset.state = response.ok && result.accepted !== false ? "ok" : "rejected";
    } catch (error) { ui.result.textContent = `操作失败：${error}`; ui.result.dataset.state = "error"; }
    finally { strategyPending = false; await refreshStrategy(); }
  });
}

for (const button of document.querySelectorAll("[data-goal]")) {
  button.addEventListener("click", async () => {
    const intent = button.dataset.goal;
    button.disabled = true;
    try {
      // Only bind a sector when the button names one. "forward" with a sector aims
      // at that sector centre, and a 12-sector grid has none straight ahead, so
      // sending sector 6 here made every press veer 15 degrees to the left.
      const message = { t:"goal", goal:intent, source:"sim-console" };
      if (button.dataset.sector) message.sector = Number(button.dataset.sector);
      if (intent === "forward") message.distance_m = 0.1;
      const result = await request(message);
      // A rejected goal (e.g. {"accepted":false,"busy":true,"reason":"strategy_active"})
      // is shown verbatim: swallowing it would look like a silent no-op.
      ui.result.textContent = JSON.stringify(result);
      ui.result.dataset.state = result.accepted === false ? "rejected" : "ok";
    } catch (error) { ui.result.textContent = String(error); ui.result.dataset.state = "error"; }
    finally { applyGoalButtons(); }
  });
}
document.getElementById("stop").addEventListener("click", async () => {
  try { ui.result.textContent = JSON.stringify(await request({ t:"stop", reason:"sim_console" })); await refreshStrategy(); }
  catch (error) { ui.result.textContent = String(error); }
});
const sceneSelect = document.getElementById("scene-select");
fetch("/api/scenes").then((r) => r.json()).then((scenes) => {
  const current = scenes.find((item) => item.name === "room") || scenes[0];
  sceneSelect.replaceChildren(...scenes.map((item) => {
    const option = document.createElement("option");
    option.value = item.name;
    option.textContent = item.name === "room" ? "room · 家具障碍场景" : "room_open · 空房场景";
    return option;
  }));
  const chosen = scenes.find((item) => item.source.endsWith("/room.world")) || scenes[0];
  sceneSelect.value = chosen.name;
  renderScene(chosen);
}).catch((error) => { document.getElementById("limits").textContent = `场景加载失败：${error}`; });
sceneSelect.addEventListener("change", async () => {
  sceneSelect.disabled = true;
  try {
    const response = await fetch("/api/scene", { method:"POST", headers:{"content-type":"application/json"}, body:JSON.stringify({name:`${sceneSelect.value}.world`}) });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || response.statusText);
    renderScene(result.scene);
  } catch (error) { ui.result.textContent = `场景切换失败：${error}`; }
  finally { applyGoalButtons(); }
});
fetch("/api/state").then((r) => r.json()).then((data) => {
  if (data.physical === false) paint(data.state);
  if (data.scene) {
    const sceneName = data.scene.name;
    if ([...sceneSelect.options].some((option) => option.value === sceneName)) sceneSelect.value = sceneName;
  }
}).catch(() => {});
loadModel().catch((error) => { ui.mesh.textContent = String(error); });

// docs/SIM-CONTRACT.md §3/§4: the workbench must show what the backend can actually
// supply. A missing sensor is listed as unavailable with its reason — never filled in
// from a mesh or from a video stream.
function fmtAge(value) {
  return (value === null || value === undefined) ? "—" : `${Number(value).toFixed(2)} s`;
}
function fmtOrDash(value, format) {
  return (value === null || value === undefined) ? "—" : format(value);
}
// mount is either a real URDF mount ({parent, xyz_m}) or a mesh-derived note
// ({parent, derived}); showing it keeps "外观不等于挂载" visible on screen.
function fmtMount(mount) {
  if (!mount || typeof mount !== "object") return "无挂载信息";
  const parent = mount.parent || "?";
  if (Array.isArray(mount.xyz_m)) {
    const xyz = mount.xyz_m.map((value) => Number(Number(value).toFixed(5))).join(", ");
    return `${parent} @ [${xyz}] m`;
  }
  if (mount.derived) return `${parent} · ${mount.derived}`;
  return parent;
}
function detailLine(text) {
  const span = document.createElement("span");
  span.className = "detail";
  span.textContent = text;
  return span;
}
function renderSensors(payload) {
  const host = document.getElementById("sensors");
  const sensors = payload.sensors || [];
  host.replaceChildren(...sensors.map((sensor) => {
    const li = document.createElement("li");
    li.dataset.state = sensor.availability;
    li.dataset.sensor = sensor.id;
    const head = document.createElement("strong");
    head.textContent = sensor.label || sensor.id;
    li.append(head, ` · ${sensor.availability}`);
    const frame = sensor.frame ? `${sensor.frame}` : "无 frame";
    const topic = sensor.topic ? `${sensor.topic}` : "无 topic";
    const substitute = sensor.substitute ? `（${sensor.substitute}）` : "";
    li.append(
      document.createElement("br"),
      detailLine(`${frame} · ${topic} · 来源 ${sensor.source}${substitute} · 龄 ${fmtAge(sensor.age_s)}`),
      document.createElement("br"),
      detailLine(`${sensor.msg_type || "无 msg_type"} · ${fmtMount(sensor.mount)}${sensor.controls === false ? " · 不可用于控制" : ""}`),
    );
    if (sensor.reason) {
      li.append(document.createElement("br"), sensor.reason);
    }
    return li;
  }));
  if (!sensors.length) {
    const li = document.createElement("li");
    li.dataset.state = "missing";
    li.textContent = "后端未报告任何传感器";
    host.replaceChildren(li);
  }
}
function renderMaps(payload) {
  const host = document.getElementById("maps");
  const labels = { truth: "真值图（SDF 几何投影）", slam: "建图结果（/scan + TF）", vehicle: "真车 /map" };
  const maps = payload.maps || {};
  const items = Object.keys(labels).map((key) => {
    const map = maps[key] || { availability: "missing" };
    const li = document.createElement("li");
    li.dataset.state = map.availability;
    li.dataset.mapKind = key;
    const head = document.createElement("strong");
    head.textContent = labels[key];
    li.append(head, ` · ${map.availability}`);
    // §4: every map reading names which map it is, and unavailable ones cannot
    // inherit a resolution or version from truth.
    const detail = map.availability === "ok"
      ? `${map.provenance || "?"} · ${map.scene || "?"} · 分辨率 ${fmtOrDash(map.resolution_m, (v) => `${v} m`)} · 版本 ${fmtOrDash(map.map_version, String)} · 已知格 ${fmtOrDash(map.known_cells, String)} · world ${map.world_sha256 ? String(map.world_sha256).slice(0, 12) : "未提供"}`
      : `provenance ${map.provenance || "?"} · 版本 ${fmtOrDash(map.map_version, String)} · 已知格 ${fmtOrDash(map.known_cells, String)}`;
    li.append(document.createElement("br"), detailLine(detail));
    if (map.availability !== "ok" && map.reason) {
      li.append(document.createElement("br"), map.reason);
    }
    if (map.note) {
      li.append(document.createElement("br"), detailLine(map.note));
    }
    return li;
  });
  host.replaceChildren(...items);
}
function stampContract(payload) {
  const stamp = document.getElementById("contract-stamp");
  if (!stamp) return;
  const generated = Number(payload?.generated_at);
  const age = Number.isFinite(generated) ? Math.max(0, Date.now() / 1000 - generated) : null;
  stamp.dataset.stale = age !== null && age > 5 ? "true" : "false";
  stamp.textContent = `契约快照 ${age === null ? "—" : `${age.toFixed(2)} s 前`} · physical=${String(payload?.physical)} · 来源 /api/sensors + /api/maps`;
}
async function refreshContract() {
  try {
    const [sensors, maps] = await Promise.all([
      fetch("/api/sensors").then((r) => r.json()),
      fetch("/api/maps").then((r) => r.json()),
    ]);
    renderSensors(sensors);
    renderMaps(maps);
    stampContract(maps);
  } catch (error) {
    const host = document.getElementById("sensors");
    const li = document.createElement("li");
    li.dataset.state = "missing";
    li.textContent = `契约数据读取失败：${error}`;
    host.replaceChildren(li);
  }
}
refreshContract();
setInterval(refreshContract, 2000);

// docs/SIM-CONTRACT.md §5: a workbench screenshot must carry the pose and the
// view. Exposing the live orbit camera here (instead of guessing in the capture
// script) lets the metadata describe the frame that was actually rendered.
window.__workbenchView = () => ({
  position_m: camera.position.toArray().map((value) => Number(value.toFixed(3))),
  target_m: controls.target.toArray().map((value) => Number(value.toFixed(3))),
  fov_deg: camera.fov,
  aspect: Number(camera.aspect.toFixed(4)),
  canvas_px: [canvas.width, canvas.height],
  pose_m_xy_yaw: lastPose ? lastPose.map((value) => Number(Number(value).toFixed(4))) : null,
  state_ts: lastStateTs,
  state_seq: lastStateSeq,
  view_mode: viewMode,
  state_age_s: lastStateTs === null ? null : Number((Date.now() / 1000 - lastStateTs).toFixed(3)),
  backend: "three.js-workbench",
  physical: false,
  workbench_not_camera_evidence: true,
});

connect();
// HTTP catches up when a browser resumes a suspended WebSocket/tab. Both read
// the very same daemon; do not render an older HTTP response over a newer frame.
setInterval(async () => {
  const age = lastStateTs === null ? Infinity : Date.now()/1000 - lastStateTs;
  if (age > 2) {
    ui.health.textContent = '画面状态陈旧，正在重连';
    ui.health.dataset.online = 'false';
    try { paint((await (await fetch('/api/state')).json()).state); } catch {}
  } else {
    ui.health.textContent = `实时 · ${age.toFixed(1)} s`;
    ui.health.dataset.online = 'true';
  }
}, 500);

let publishing = false;
setInterval(async () => {
  if (publishing || !lastStateTs || Date.now()/1000 - lastStateTs > 2) return;
  publishing = true;
  try {
    renderer.render(scene, camera);
    const png = canvas.toDataURL('image/png').split(',')[1];
    await fetch('/api/workbench/frame', {method:'POST', headers:{'content-type':'application/json'},
      body:JSON.stringify({png, view:window.__workbenchView()})});
  } catch {} finally { publishing = false; }
}, 1000);
function animate() { requestAnimationFrame(animate); controls.update(); renderer.render(scene, camera); }
animate();
