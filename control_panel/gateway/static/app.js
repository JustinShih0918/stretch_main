// Stretch control panel: one page, one WebSocket to the gateway (gateway/__main__.py).
"use strict";

const $ = (s) => document.querySelector(s);
const $$ = (s) => Array.from(document.querySelectorAll(s));
let S = null;            // the last `state` from the gateway
let lastStateAt = 0;

// ------------------------------------------------------------- connection
let ws = null, nextId = 1;
const pending = new Map();

function connect() {
  ws = new WebSocket((location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/ws");
  ws.onmessage = (ev) => {
    const m = JSON.parse(ev.data);
    if (m.type === "state") {
      if (m.scan === null && S) m.scan = S.scan;   // the scan rides on every other state
      S = m; lastStateAt = Date.now(); render();
    }
    else if (m.type === "osg_check") { renderChecks(m.checks); }
    else if (m.type === "reply") {
      const p = pending.get(m.id);
      if (p) { pending.delete(m.id); p(m); }
    }
  };
  ws.onclose = () => { ws = null; setTimeout(connect, 1000); };
  ws.onerror = () => { try { ws.close(); } catch (e) {} };
}
function send(obj) { if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj)); }
function call(op, params = {}, quiet = false) {
  return new Promise((resolve) => {
    if (!ws || ws.readyState !== 1) { toast("沒有連線到面板伺服器", true); resolve({ ok: false }); return; }
    const id = nextId++;
    pending.set(id, (m) => { if (!quiet || !m.ok) toast(m.message, !m.ok); resolve(m); });
    send(Object.assign({ type: "call", id, op }, params));
  });
}
async function busy(btn, fn) {
  btn.classList.add("busy");
  try { return await fn(); } finally { btn.classList.remove("busy"); }
}
let toastTimer = null;
function toast(text, bad = false) {
  if (!text) return;
  const t = $("#toast"); t.textContent = text; t.className = "show" + (bad ? " bad" : "");
  clearTimeout(toastTimer); toastTimer = setTimeout(() => (t.className = ""), bad ? 5000 : 2500);
}

// ------------------------------------------------------------------- tabs
let activeTab = "system";
$$("#tabs button").forEach((b) => b.addEventListener("click", () => {
  activeTab = b.dataset.tab;
  $$("#tabs button").forEach((x) => x.classList.toggle("on", x === b));
  $$(".tab").forEach((t) => t.classList.toggle("active", t.id === "tab-" + activeTab));
  if (activeTab === "map") { resizeMap(); mapDirty = true; }
  if (activeTab !== "drive") stopDrive();
}));

// ---------------------------------------------------------------- top bar
$("#estop").addEventListener("click", () => { stopDrive(); call("estop"); });
$("#runstop-off").addEventListener("click", (e) => busy(e.target, () => call("runstop", { on: false })));
$("#free-robot").addEventListener("click", (e) => {
  if (confirm("關閉占用機器人的其他程式（含底盤驅動）？之後需重新啟動底盤驅動。")) busy(e.target, () => call("free_robot"));
});
$$("[data-driver]").forEach((b) => b.addEventListener("click", () => {
  const a = b.dataset.driver;
  if (a === "home" && !confirm("歸零會移動手臂、升降與頭部，確定周圍淨空？")) return;
  if (a === "stow" && !confirm("收納會移動手臂與升降，確定？")) return;
  busy(b, () => call("driver", { action: a }));
}));
$$("#mode-seg button").forEach((b) => b.addEventListener("click",
  () => busy(b, () => call("driver", { action: "mode_" + b.dataset.mode }))));

function setBadge(el, text, cls) { el.textContent = text; el.className = "badge" + (cls ? " " + cls : ""); }

// ------------------------------------------------------------------ units
const unitRows = new Map();
function unitRow(name, u) {
  const row = document.createElement("div"); row.className = "unit";
  row.innerHTML = `<div><span class="name"></span><span class="pill"></span></div>
    <div class="ctl"></div><div class="help"></div>`;
  row.querySelector(".name").textContent = u.label || name;
  row.querySelector(".help").textContent = u.help || "";
  const ctl = row.querySelector(".ctl");
  if ((u.needs || []).includes("map")) {
    const sel = document.createElement("select"); sel.className = "map-select"; ctl.appendChild(sel);
  }
  const start = document.createElement("button"); start.className = "btn"; start.textContent = "啟動";
  const stop = document.createElement("button"); stop.className = "btn danger"; stop.textContent = "關閉";
  start.onclick = () => busy(start, () => {
    const args = {}; const sel = ctl.querySelector("select");
    if (sel) { if (!sel.value) { toast("先選一張地圖", true); return; } args.map = sel.value; }
    return call("unit", { action: "start", unit: name, args });
  });
  stop.onclick = () => busy(stop, () => call("unit", { action: "stop", unit: name }));
  ctl.append(start, stop);
  const log = document.createElement("div"); log.className = "log"; log.hidden = true; row.appendChild(log);
  return { row, start, stop, log, pill: row.querySelector(".pill") };
}
const DETECT_KEY = { mapping: "slam", localization: "slam" };
const STATE_TXT = { running: "執行中", starting: "啟動中", stopping: "關閉中", stopped: "未啟動", failed: "失敗" };

function renderUnits() {
  const box = $("#units");
  const sup = S.supervisor;
  const supAge = S.age ? S.age.supervisor : undefined;
  if (!sup || supAge === undefined || supAge > 5) {
    box.innerHTML = `<p class="hint">機器人上的 supervisor 沒有回應${supAge ? `（${Math.round(supAge)} 秒前）` : ""}。
      在 Stretch 上啟動 <code>control_panel/supervisor</code>（見 README）。下面是從 ROS 偵測到的狀態：</p>` +
      Object.entries(S.detected || {}).map(([k, v]) => `<div>${k}: ${v ? "✅ 有在跑" : "—"}</div>`).join("");
    unitRows.clear(); $("#sup-host").textContent = ""; return;
  }
  $("#sup-host").textContent = "@ " + sup.host;
  if (box.querySelector("p")) box.innerHTML = "";
  for (const [name, u] of Object.entries(sup.units)) {
    let r = unitRows.get(name);
    if (!r) { r = unitRow(name, u); unitRows.set(name, r); box.appendChild(r.row); }
    const slamUp = ["mapping", "localization", "rtabmap"].some((n) => ["running", "starting"].includes((sup.units[n] || {}).state));
    const ext = u.state === "stopped" && S.detected && S.detected[DETECT_KEY[name] || name] &&
                !(DETECT_KEY[name] === "slam" && slamUp);
    r.pill.className = "pill " + (ext ? "external" : u.state);
    r.pill.textContent = ext ? "外部已在跑" : (STATE_TXT[u.state] || u.state) +
      (u.args && u.args.map ? ` · ${u.args.map}` : "");
    const up = u.state === "running" || u.state === "starting";
    // "外部已在跑": the node is on the ROS graph but not from this supervisor
    // (someone's terminal, docker/deploy, a supervisor that was killed). The
    // supervisor refuses such a start anyway (its unit `check`), so do not
    // offer the button -- a second driver or a second Nav2 is what it would be.
    r.start.disabled = up || ext || u.state === "stopping";
    r.start.title = ext ? "已經有別人啟動的在跑，面板不會再開一個" : "";
    r.stop.disabled = !up;
    r.log.hidden = !(u.state === "failed" && u.log && u.log.length);
    if (!r.log.hidden) r.log.textContent = u.log.join("\n");
  }
  fillMapSelects(sup.maps || []);
}
function fillMapSelects(maps) {
  const key = maps.map((m) => m.name).join(",");
  $$("select.map-select, #flow-map-select").forEach((sel) => {
    if (sel.dataset.key === key) return;
    const keep = sel.value;
    sel.innerHTML = maps.length ? maps.map((m) => `<option value="${m.name}">${m.name}</option>`).join("")
                                : `<option value="">（沒有地圖）</option>`;
    if (maps.some((m) => m.name === keep)) sel.value = keep;
    sel.dataset.key = key;
  });
}

// ------------------------------------------------------------------ flows
function unitState(name) { return S && S.supervisor && S.supervisor.units[name] ? S.supervisor.units[name].state : null; }
function waitUnit(name, timeoutMs = 20000) {
  return new Promise((resolve) => {
    const t0 = Date.now();
    const iv = setInterval(() => {
      const st = unitState(name);
      if (st === "running" || st === "failed" || Date.now() - t0 > timeoutMs) { clearInterval(iv); resolve(st); }
    }, 250);
  });
}
async function ensureUnit(name, args = {}) {
  const st = unitState(name);
  if (st === "running") return true;
  $("#flow-status").textContent = `啟動 ${name}…`;
  const r = await call("unit", { action: "start", unit: name, args }, true);
  if (!r.ok) { $("#flow-status").textContent = r.message; return false; }
  const end = await waitUnit(name);
  if (end !== "running") { $("#flow-status").textContent = `${name} 沒有起來（${end}），看模組的紀錄`; return false; }
  return true;
}
async function runFlow(btn, steps) {
  await busy(btn, async () => {
    for (const [name, args] of steps) if (!(await ensureUnit(name, args))) return;
    $("#flow-status").textContent = "完成 ✔";
  });
}
// slam_toolbox, RTAB-Map and localization all own map -> odom: one at a time.
function slamBusy(except) {
  const NAMES = { mapping: "建圖 slam_toolbox", rtabmap: "建圖 RTAB-Map", localization: "定位" };
  const on = Object.keys(NAMES).find((n) => n !== except && ["running", "starting"].includes(unitState(n)));
  if (on) toast(`先關閉「${NAMES[on]}」`, true);
  return !!on;
}
$("#flow-map").addEventListener("click", (e) => {
  if (slamBusy("mapping")) return;
  runFlow(e.target, [["driver"], ["lidar"], ["mapping"], ["navigation"]]);
});
$("#flow-map-rtab").addEventListener("click", (e) => {
  if (slamBusy("rtabmap")) return;
  runFlow(e.target, [["driver"], ["lidar"], ["camera"], ["rtabmap"], ["navigation"]]);
});
$("#flow-nav").addEventListener("click", (e) => {
  const map = $("#flow-map-select").value;
  if (!map) { toast("先選一張地圖", true); return; }
  if (slamBusy("localization")) return;
  runFlow(e.target, [["driver"], ["lidar"], ["localization", { map }], ["navigation"]]);
});
$("#stop-all").addEventListener("click", (e) => {
  if (confirm("關閉機器人上所有模組（含底盤驅動）？")) busy(e.target, () => call("stop_all"));
});

// -------------------------------------------------------------------- map
const canvas = $("#map"), ctx = canvas.getContext("2d");
const mapImg = new Image(); let mapVersion = -1, mapInfo = null, mapDirty = true;
const view = { s: 4, x: 0, y: 0, fitted: false };  // canvas px per map px, and map-px offset
let tool = "pan", drag = null;
mapImg.onload = () => { mapDirty = true; if (!view.fitted) fitMap(); };

function resizeMap() {
  const r = canvas.getBoundingClientRect(), d = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, r.width * d); canvas.height = Math.max(1, r.height * d);
  if (!view.fitted) fitMap();
  mapDirty = true;
}
window.addEventListener("resize", resizeMap);
function fitMap() {
  if (!mapInfo || !mapImg.naturalWidth || canvas.width < 2 || activeTab !== "map") return;
  view.s = Math.min(canvas.width / mapInfo.width, canvas.height / mapInfo.height) * 0.95;
  view.x = (canvas.width / view.s - mapInfo.width) / 2; view.y = (canvas.height / view.s - mapInfo.height) / 2;
  view.fitted = true; mapDirty = true;
}
// world (m) <-> map image px <-> canvas px
function w2p(x, y) { const i = mapInfo; return [(x - i.origin[0]) / i.resolution, i.height - (y - i.origin[1]) / i.resolution]; }
function p2w(px, py) { const i = mapInfo; return [px * i.resolution + i.origin[0], (i.height - py) * i.resolution + i.origin[1]]; }
function p2c(px, py) { return [(px + view.x) * view.s, (py + view.y) * view.s]; }
function c2p(cx, cy) { return [cx / view.s - view.x, cy / view.s - view.y]; }
function evC(e) { const r = canvas.getBoundingClientRect(), d = window.devicePixelRatio || 1; return [(e.clientX - r.left) * d, (e.clientY - r.top) * d]; }

function drawArrow(cx, cy, yaw, len, color, fill = true) {
  ctx.save(); ctx.translate(cx, cy); ctx.rotate(-yaw);
  ctx.beginPath(); ctx.moveTo(len, 0); ctx.lineTo(-len * 0.6, len * 0.55); ctx.lineTo(-len * 0.3, 0);
  ctx.lineTo(-len * 0.6, -len * 0.55); ctx.closePath();
  if (fill) { ctx.fillStyle = color; ctx.fill(); } ctx.strokeStyle = "#000a"; ctx.lineWidth = 2; ctx.stroke();
  ctx.restore();
}
function drawMap() {
  if (activeTab !== "map") return;
  mapDirty = false;
  ctx.setTransform(1, 0, 0, 1, 0, 0); ctx.clearRect(0, 0, canvas.width, canvas.height);
  $("#map-empty").hidden = !!(mapInfo && mapImg.complete && mapImg.naturalWidth);
  if (!mapInfo || !mapImg.naturalWidth) return;
  const d = window.devicePixelRatio || 1;
  ctx.imageSmoothingEnabled = false;
  const [ox, oy] = p2c(0, 0);
  ctx.drawImage(mapImg, ox, oy, mapInfo.width * view.s, mapInfo.height * view.s);
  if (S && S.plan && S.plan.length > 1) {
    ctx.beginPath(); S.plan.forEach(([x, y], k) => { const [cx, cy] = p2c(...w2p(x, y)); k ? ctx.lineTo(cx, cy) : ctx.moveTo(cx, cy); });
    ctx.strokeStyle = "#3d8bfd"; ctx.lineWidth = 3 * d; ctx.stroke();
  }
  if (S && S.scan) {
    ctx.fillStyle = "#ff4d5a";
    for (const [x, y] of S.scan) { const [cx, cy] = p2c(...w2p(x, y)); ctx.fillRect(cx - 1.5 * d, cy - 1.5 * d, 3 * d, 3 * d); }
  }
  if (S && S.nav && S.nav.goal && (S.nav.active || S.nav.status === "arrived")) {
    const [gx, gy, gyaw] = S.nav.goal; const [cx, cy] = p2c(...w2p(gx, gy));
    drawArrow(cx, cy, gyaw, 12 * d, "#2fbf71");
  }
  const g = S && S.osg && S.osg.graph;
  if (g && g.objects) {
    ctx.font = `${11 * d}px sans-serif`; ctx.textBaseline = "bottom";
    for (const o of g.objects) {
      const [cx, cy] = p2c(...w2p(o.xyz[0], o.xyz[1]));
      ctx.beginPath(); ctx.arc(cx, cy, (o.target ? 6 : 4) * d, 0, 2 * Math.PI);
      ctx.fillStyle = o.target ? "#ff9b2f" : "#b58cff"; ctx.fill();
      if (o.target) { ctx.lineWidth = 2 * d; ctx.strokeStyle = "#fff"; ctx.stroke(); }
      const text = o.name || o.label;
      ctx.lineWidth = 3 * d; ctx.strokeStyle = "#000c"; ctx.strokeText(text, cx + 6 * d, cy - 3 * d);
      ctx.fillStyle = o.target ? "#ffd9a8" : "#e6dcff"; ctx.fillText(text, cx + 6 * d, cy - 3 * d);
    }
  }
  if (S && S.pose && S.pose.frame === (mapInfo.frame || "map")) {
    const [cx, cy] = p2c(...w2p(S.pose.x, S.pose.y));
    const rpx = Math.max(10 * d, (0.17 / mapInfo.resolution) * view.s);
    ctx.beginPath(); ctx.arc(cx, cy, rpx, 0, 2 * Math.PI); ctx.fillStyle = "#3d8bfd44"; ctx.fill();
    drawArrow(cx, cy, S.pose.yaw, Math.max(12 * d, rpx), "#3d8bfd");
  }
  if (drag && drag.kind !== "pan") {
    const [cx, cy] = p2c(...drag.p0);
    const yaw = drag.yaw || 0;
    ctx.beginPath(); ctx.arc(cx, cy, 6 * d, 0, 2 * Math.PI); ctx.fillStyle = drag.kind === "goal" ? "#2fbf71" : "#f0a92e"; ctx.fill();
    drawArrow(cx, cy, yaw, 22 * d, drag.kind === "goal" ? "#2fbf71" : "#f0a92e");
  }
}
function loop() { if (mapDirty) drawMap(); requestAnimationFrame(loop); }
requestAnimationFrame(loop);

// pan / zoom / pick
const pointers = new Map(); let pinch = null;
canvas.addEventListener("pointerdown", (e) => {
  canvas.setPointerCapture(e.pointerId); pointers.set(e.pointerId, evC(e));
  if (pointers.size === 2) {
    const [a, b] = [...pointers.values()];
    pinch = { d: Math.hypot(a[0] - b[0], a[1] - b[1]), s: view.s, mid: c2p((a[0] + b[0]) / 2, (a[1] + b[1]) / 2) };
    drag = null; return;
  }
  if (!mapInfo) return;
  const c = evC(e);
  drag = tool === "pan" ? { kind: "pan", last: c } : { kind: tool, p0: c2p(...c), yaw: 0 };
  mapDirty = true;
});
canvas.addEventListener("pointermove", (e) => {
  if (!pointers.has(e.pointerId)) return;
  pointers.set(e.pointerId, evC(e));
  if (pinch && pointers.size === 2) {
    const [a, b] = [...pointers.values()];
    const dist = Math.hypot(a[0] - b[0], a[1] - b[1]);
    const mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
    view.s = Math.min(60, Math.max(0.2, pinch.s * dist / pinch.d));
    view.x = mid[0] / view.s - pinch.mid[0]; view.y = mid[1] / view.s - pinch.mid[1];
    mapDirty = true; return;
  }
  if (!drag) return;
  const c = evC(e);
  if (drag.kind === "pan") { view.x += (c[0] - drag.last[0]) / view.s; view.y += (c[1] - drag.last[1]) / view.s; drag.last = c; }
  else { const p = c2p(...c); const dx = p[0] - drag.p0[0], dy = p[1] - drag.p0[1];
         if (Math.hypot(dx, dy) * view.s > 8) drag.yaw = Math.atan2(-dy, dx); }
  mapDirty = true;
});
function endPointer(e) {
  pointers.delete(e.pointerId);
  if (pointers.size < 2) pinch = null;
  if (!drag) return;
  const d = drag; drag = null; mapDirty = true;
  if (d.kind === "pan" || e.type === "pointercancel") return;
  const [x, y] = p2w(...d.p0);
  const deg = Math.round(d.yaw * 180 / Math.PI);
  if (d.kind === "goal") call("nav_goal", { x, y, yaw: d.yaw });
  else if (confirm(`把機器人位置設為 (${x.toFixed(2)}, ${y.toFixed(2)})，朝向 ${deg}°？`)) call("set_pose", { x, y, yaw: d.yaw });
}
canvas.addEventListener("pointerup", endPointer);
canvas.addEventListener("pointercancel", endPointer);
canvas.addEventListener("wheel", (e) => {
  e.preventDefault(); const c = evC(e); const p = c2p(...c);
  view.s = Math.min(60, Math.max(0.2, view.s * (e.deltaY < 0 ? 1.15 : 1 / 1.15)));
  view.x = c[0] / view.s - p[0]; view.y = c[1] / view.s - p[1]; mapDirty = true;
}, { passive: false });
$$("#tool-seg button").forEach((b) => b.addEventListener("click", () => {
  tool = b.dataset.tool; $$("#tool-seg button").forEach((x) => x.classList.toggle("on", x === b));
}));
$("#center").addEventListener("click", () => {
  if (!mapInfo) return;
  if (S && S.pose && S.pose.frame === (mapInfo.frame || "map")) {
    const [px, py] = w2p(S.pose.x, S.pose.y);
    view.x = canvas.width / view.s / 2 - px; view.y = canvas.height / view.s / 2 - py; mapDirty = true;
  } else fitMap();
});
$("#nav-cancel").addEventListener("click", () => call("nav_cancel"));
$("#save-map").addEventListener("click", (e) => {
  const name = $("#map-name").value.trim();
  if (!name) { toast("先輸入地圖名稱", true); return; }
  busy(e.target, () => call("save_map", { name }));
});

function renderMap() {
  if (S.map && S.map.version !== mapVersion) {
    mapVersion = S.map.version; mapInfo = S.map;
    mapImg.src = "/api/map.png?v=" + mapVersion + "&t=" + Date.now();
  }
  if (!S.map) { mapInfo = null; }
  const p = S.pose, n = S.nav || {};
  const NAV = { driving: "導航中", arrived: "已到達", failed: "失敗", canceled: "已取消", canceling: "取消中", rejected: "被拒絕", sent: "送出中", accepted: "已接受" };
  let info = p ? `位置 (${p.x.toFixed(2)}, ${p.y.toFixed(2)}) ${Math.round(p.yaw * 180 / Math.PI)}° [${p.frame}]` : "沒有 TF：機器人位置未知";
  if (n.status) info += ` · 導航：${NAV[n.status] || n.status}` + (n.active && n.distance != null ? ` 剩 ${n.distance} m` : "");
  $("#map-info").textContent = info;
  $("#nav-cancel").disabled = !n.active;
  $("#save-map").disabled = !["mapping", "rtabmap"].some((n) => unitState(n) === "running");
  mapDirty = true;
}

// ------------------------------------------------------------------ drive
const pad = $("#pad"), knob = $("#knob");
let stick = null, driveTimer = null;
function speedFrac() { return $("#speed").value / 100; }
$("#speed").addEventListener("input", () => ($("#speed-val").textContent = $("#speed").value + "%"));
function sendStick() {
  if (!stick || !S) return;
  const t = S.teleop || { max_linear: 0.3, max_angular: 1.0 };
  send({ type: "teleop", vx: -stick[1] * t.max_linear * speedFrac(), wz: -stick[0] * t.max_angular * speedFrac() });
}
function startDrive() { if (!driveTimer) driveTimer = setInterval(sendStick, 100); pad.classList.add("live"); }
function stopDrive() {
  stick = null; clearInterval(driveTimer); driveTimer = null; pad.classList.remove("live");
  knob.style.transform = ""; send({ type: "teleop_stop" });
}
function padMove(e) {
  const r = pad.getBoundingClientRect(), R = r.width / 2;
  let dx = (e.clientX - r.left - R) / R, dy = (e.clientY - r.top - R) / R;
  const m = Math.hypot(dx, dy); if (m > 1) { dx /= m; dy /= m; }
  const dead = (v) => (Math.abs(v) < 0.08 ? 0 : v);
  stick = [dead(dx), dead(dy)];
  knob.style.transform = `translate(${dx * R * 0.66}px, ${dy * R * 0.66}px)`;
}
pad.addEventListener("pointerdown", (e) => { pad.setPointerCapture(e.pointerId); padMove(e); sendStick(); startDrive(); });
pad.addEventListener("pointermove", (e) => { if (driveTimer) padMove(e); });
pad.addEventListener("pointerup", stopDrive);
pad.addEventListener("pointercancel", stopDrive);
document.addEventListener("visibilitychange", () => { if (document.hidden) stopDrive(); });
window.addEventListener("blur", stopDrive);
const keys = new Set();
function keyStick() {
  const x = (keys.has("d") ? 1 : 0) - (keys.has("a") ? 1 : 0), y = (keys.has("s") ? 1 : 0) - (keys.has("w") ? 1 : 0);
  if (!x && !y) { stopDrive(); return; } stick = [x, y]; startDrive(); sendStick();
}
document.addEventListener("keydown", (e) => {
  if (activeTab !== "drive" || !"wasd".includes(e.key) || e.repeat || e.target.tagName === "INPUT") return;
  keys.add(e.key); keyStick();
});
document.addEventListener("keyup", (e) => { if (keys.delete(e.key)) keyStick(); });

// The D435i is mounted sideways in the Stretch's head: one clockwise quarter
// turn makes it upright (OSG's ros2.rotate_deg -90 is the same turn, measured
// on the robot). The button cycles 90 / -90 / 0 in case a head differs.
let camRot = 90;
// Head (RealSense) pan / tilt, about 11° a tap; ⊙ straight ahead, 地 the floor.
const HEAD_STEP = 0.2;
const HEAD_MOVES = { up: { tilt: HEAD_STEP }, down: { tilt: -HEAD_STEP }, left: { pan: HEAD_STEP },
                     right: { pan: -HEAD_STEP }, forward: { preset: "forward" }, floor: { preset: "floor" } };
$$("[data-head]").forEach((b) => b.addEventListener("click", () =>
  busy(b, () => call("head", HEAD_MOVES[b.dataset.head], true))));
$("#cam-rot").addEventListener("click", () => { camRot = camRot === 90 ? -90 : camRot === -90 ? 0 : 90; drawCam(); });
window.addEventListener("resize", () => { if (activeTab === "drive") drawCam(); });
const camCanvas = $("#cam"), camCtx = camCanvas.getContext("2d");
let camFrame = null, camBusy = false;
function drawCam() {
  const r = camCanvas.getBoundingClientRect(), d = window.devicePixelRatio || 1;
  camCanvas.width = Math.max(1, r.width * d); camCanvas.height = Math.max(1, r.height * d);
  camCtx.clearRect(0, 0, camCanvas.width, camCanvas.height);
  if (!camFrame) return;
  const side = camRot !== 0;
  const w = side ? camFrame.height : camFrame.width, h = side ? camFrame.width : camFrame.height;
  const s = Math.min(camCanvas.width / w, camCanvas.height / h);
  camCtx.save();
  camCtx.translate(camCanvas.width / 2, camCanvas.height / 2);
  camCtx.rotate(camRot * Math.PI / 180);
  camCtx.drawImage(camFrame, -camFrame.width * s / 2, -camFrame.height * s / 2, camFrame.width * s, camFrame.height * s);
  camCtx.restore();
}
setInterval(() => {
  if (activeTab !== "drive" || camBusy) return;
  camBusy = true;
  const img = new Image();
  img.onload = () => { camFrame = img; drawCam(); $("#cam-empty").hidden = true; camBusy = false; };
  img.onerror = () => { $("#cam-empty").hidden = false; camBusy = false; };
  img.src = "/api/camera.jpg?t=" + Date.now();
}, 250);

function renderDrive() {
  const mode = S.robot && S.robot.mode;
  const warn = $("#drive-warn");
  let text = "";
  if (!S.detected || !S.detected.driver) text = "底盤驅動沒有在跑：到「系統」啟動 Driver";
  else if (S.robot.runstopped) text = "機器人在急停狀態：按「解除急停」";
  else if (mode && mode !== "navigation") text = `驅動模式是「${mode}」：面板駕駛需要「導航」模式`;
  warn.textContent = text; warn.classList.toggle("show", !!text);
}

// ----------------------------------------------------------------- render
function render() {
  const r = S.robot || {}, age = S.age || {};
  const fresh = (k, s = 3) => age[k] !== undefined && age[k] < s;
  setBadge($("#b-mode"), r.mode && fresh("mode") ? r.mode : "mode —", r.mode === "navigation" ? "ok" : (r.mode ? "warn" : ""));
  setBadge($("#b-batt"), r.battery_v && fresh("battery_v", 10) ? r.battery_v.toFixed(1) + " V" : "— V",
           r.battery_v ? (r.battery_v < 10.5 ? "bad" : r.battery_v < 11.2 ? "warn" : "ok") : "");
  setBadge($("#b-homed"), r.homed ? "已歸零" : "未歸零", r.homed ? "ok" : (r.homed === false ? "warn" : ""));
  setBadge($("#b-runstop"), r.runstopped ? "急停中" : (r.runstopped === false ? "可移動" : "—"),
           r.runstopped ? "bad" : (r.runstopped === false ? "ok" : ""));
  $$("#mode-seg button").forEach((b) => b.classList.toggle("on", b.dataset.mode === r.mode));
  renderUnits(); renderMap(); renderDrive(); renderOsg();
}
setInterval(() => {
  const c = $("#conn");
  c.className = "dot" + (!ws || ws.readyState !== 1 ? "" : Date.now() - lastStateAt < 2000 ? " ok" : " stale");
}, 500);


// -------------------------------------------------------------------- OSG
let P = { presets: [], targets: [], defaults: {}, demo: null };
let osgPreset = "stretch3_map";
const RUNS = ["map_run", "search_run"];
fetch("/api/osg_presets.json").then((r) => r.json()).then((p) => { P = Object.assign(P, p); setupOsg(); }).catch(() => {});

function osgUnits() { return (S && S.osg && S.osg.runner && S.osg.runner.state && S.osg.runner.state.units) || {}; }
function osgState(name) { return (osgUnits()[name] || {}).state || null; }
function osgArgs() {
  return { preset: osgPreset, map_tag: $("#osg-tag").value.trim(), target: $("#osg-target").value.trim(),
           max_steps: $("#osg-steps").value || String(P.defaults.max_steps || 400) };
}
function setPreset(id) {
  osgPreset = id;
  $$("#osg-preset button").forEach((b) => b.classList.toggle("on", b.dataset.id === id));
}
function setupOsg() {
  const seg = $("#osg-preset"); seg.innerHTML = "";
  for (const pr of P.presets || []) {
    const b = document.createElement("button"); b.dataset.id = pr.id; b.textContent = pr.label; b.title = pr.help || "";
    b.onclick = () => setPreset(pr.id); seg.appendChild(b);
  }
  setPreset((P.presets && P.presets[0] && P.presets[0].id) || "stretch3_map");
  const box = $("#osg-targets"); box.innerHTML = "";
  for (const t of P.targets || []) {
    const c = document.createElement("button"); c.className = "chip"; c.textContent = t.label;
    c.onclick = () => {
      $("#osg-target").value = t.target; if (t.preset) setPreset(t.preset);
      $$("#osg-targets .chip").forEach((x) => x.classList.toggle("on", x === c));
    };
    box.appendChild(c);
  }
  $("#osg-tag").value = (P.defaults && P.defaults.map_tag) || "lab";
  $("#osg-steps").value = (P.defaults && P.defaults.max_steps) || 400;
  renderDemo();
}
$("#osg-tags").addEventListener("change", (e) => { if (e.target.value) $("#osg-tag").value = e.target.value; });

async function osgRun(unit, args) {
  if (!args.target) { toast("先選或輸入目標", true); return { ok: false }; }
  if (!args.map_tag) { toast("先輸入地圖名稱", true); return { ok: false }; }
  return call("osg", { action: "start", unit, args });
}
$("#osg-map").addEventListener("click", (e) => busy(e.target, () => osgRun("map_run", osgArgs())));
$("#osg-search").addEventListener("click", (e) => busy(e.target, () => osgRun("search_run", osgArgs())));
$("#osg-stop").addEventListener("click", (e) => busy(e.target, async () => {
  for (const u of RUNS) if (["running", "starting"].includes(osgState(u))) await call("osg", { action: "stop", unit: u });
}));
$("#osg-check").addEventListener("click", (e) => busy(e.target, () => call("osg_check", { preset: osgPreset }, true)));

function renderChecks(checks) {
  const bad = checks.filter((c) => c.ok === false && c.hard).length;
  $("#osg-check-sum").textContent = bad ? `${bad} 項未就緒` : "全部就緒 ✔";
  $("#osg-checks").innerHTML = checks.map((c) => {
    const icon = c.ok === true ? "✅" : c.ok === null ? "➖" : c.hard ? "❌" : "⚠️";
    return `<div class="check"><span>${icon}</span><span>${esc(c.label)}</span>${c.detail ? `<span class="detail">${esc(c.detail)}</span>` : ""}</div>`;
  }).join("");
  lastCheck = { ok: !bad, at: Date.now() };
}
let lastCheck = null;
function esc(t) { return String(t).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c])); }

const STAGES = [["searching", "搜尋中"], ["checking", "確認中"], ["inspecting", "檢查中"], ["verdict", "判定"]];
let osgViewVersion = 0;
function renderOsg() {
  const o = S.osg || {}, r = o.runner || {};
  $("#osg-runner").textContent = r.alive ? "@ OSG 容器" : `OSG 容器沒有回應${r.error ? "：" + r.error : ""}`;
  // perception unit
  const pu = osgUnits().perception, pr = $("#osg-perception");
  if (!pr.dataset.built) {
    pr.innerHTML = `<div><span class="name">感知模型 Perception</span><span class="pill"></span></div>
      <div class="ctl"><button class="btn">啟動</button><button class="btn danger">關閉</button></div>
      <div class="help">BLIP-2 :13182、RAM++ :13185 等模型伺服器</div>`;
    const [b1, b2] = pr.querySelectorAll("button");
    b1.onclick = () => busy(b1, () => call("osg", { action: "start", unit: "perception" }));
    b2.onclick = () => busy(b2, () => call("osg", { action: "stop", unit: "perception" }));
    pr.dataset.built = "1";
  }
  const st = pu ? pu.state : null, pill = pr.querySelector(".pill");
  pill.className = "pill " + (st || ""); pill.textContent = st ? (STATE_TXT[st] || st) : "—";
  const [b1, b2] = pr.querySelectorAll("button");
  b1.disabled = !r.alive || ["running", "starting"].includes(st); b2.disabled = !r.alive || !["running", "starting"].includes(st);
  // saved OSG maps
  const maps = (r.state && r.state.maps) || [], key = maps.map((m) => m.name).join(",");
  const sel = $("#osg-tags");
  if (sel.dataset.key !== key) {
    sel.innerHTML = `<option value="">已存的地圖…</option>` + maps.map((m) => `<option>${esc(m.name)}</option>`).join("");
    sel.dataset.key = key;
  }
  // which run, and its state
  const run = RUNS.find((u) => ["running", "starting", "stopping"].includes(osgState(u))) ||
              RUNS.slice().sort((a, b) => ((osgUnits()[b] || {}).since || 0) - ((osgUnits()[a] || {}).since || 0))[0];
  const ru = osgUnits()[run] || {};
  const rp = $("#osg-run-state");
  rp.className = "pill " + (ru.state || "");
  rp.textContent = ru.state ? `${run === "map_run" ? "建圖趟" : "搜尋趟"} · ${STATE_TXT[ru.state] || ru.state}` +
                   (ru.state === "stopped" && ru.exit_code === 0 ? " ✔" : "") : "—";
  const busyRun = RUNS.some((u) => ["running", "starting", "stopping"].includes(osgState(u)));
  $("#osg-map").disabled = $("#osg-search").disabled = busyRun || !r.alive;
  $("#osg-stop").disabled = !busyRun;
  $("#osg-log").textContent = (ru.log || []).join("\n");
  // mission stage and verdict
  const m = o.mission;
  const idx = m ? STAGES.findIndex(([k]) => k === m.stage) : -1;
  $("#osg-stage").innerHTML = m && m.stage === "not_found"
    ? `<span class="now" style="background:var(--bad)">沒找到 ${esc(m.target || "")}</span>`
    : STAGES.map(([k, t], i) => `<span class="${i < idx ? "done" : i === idx ? "now" : ""}">${t}</span>`).join("");
  const v = $("#osg-verdict");
  if (m && m.stage === "verdict") {
    const cond = (m.condition || "unknown").toLowerCase();
    const label = { bad: "異常 BAD", good: "正常 GOOD", unknown: "無法判定" }[cond] || cond;
    if (o.view !== osgViewVersion) { osgViewVersion = o.view; }
    v.innerHTML = `<div class="verdict ${cond}">
      ${o.view ? `<img src="/api/osg_view.png?v=${o.view}">` : "<div></div>"}
      <div><div class="big">${label}${m.confidence != null ? ` · ${Math.round(m.confidence * 100)}%` : ""}</div>
      <div>${esc(m.object || m.target || "")}${m.where ? ` @ (${m.where.map((x) => (+x).toFixed(1)).join(", ")})` : ""}</div>
      ${(m.issues || []).map((i) => `<div>• ${esc(i)}</div>`).join("")}
      ${m.summary ? `<div class="muted">${esc(m.summary)}</div>` : ""}
      ${m.error ? `<div style="color:var(--bad)">${esc(m.error)}</div>` : ""}</div></div>`;
  } else v.innerHTML = "";
  const EV = { searching: "開始搜尋", checking: "停下確認", resumed: "不是目標，繼續找", inspecting: "開始檢查", verdict: "判定", not_found: "沒找到" };
  $("#osg-events").innerHTML = (o.events || []).slice().reverse().map((e) =>
    `<div>${new Date(e.t * 1000).toLocaleTimeString()} · ${EV[e.stage] || e.stage} ${esc(e.target || "")}` +
    `${e.where ? ` @ (${e.where.map((x) => (+x).toFixed(1)).join(", ")})` : ""}${e.reason ? ` — ${esc(e.reason)}` : ""}</div>`).join("");
  // scene graph
  const g = o.graph;
  $("#osg-graph-meta").textContent = g ? `step ${g.step} · ${g.state || ""} · ${g.objects.length} 物體` : "";
  $("#osg-objects").innerHTML = g ? g.objects.map((x) => `<span class="${x.target ? "target" : ""}">${esc(x.name || x.label)}</span>`).join("")
                                  : `<span class="muted">OSG 還沒有發佈場景圖（跑一趟就會出現）</span>`;
  renderDemo();
}

// ---------------------------------------------------------------- demo flow
let demoStep = 0, demoBusy = false;
try { demoStep = +localStorage.getItem("demoStep") || 0; } catch (e) {}
function setDemo(i) { demoStep = i; try { localStorage.setItem("demoStep", String(i)); } catch (e) {} renderDemo(); }
$("#demo-reset").addEventListener("click", () => setDemo(0));
function waitFor(pred, timeoutMs) {
  return new Promise((resolve) => {
    const t0 = Date.now();
    const iv = setInterval(() => { const v = pred(); if (v || Date.now() - t0 > timeoutMs) { clearInterval(iv); resolve(v); } }, 500);
  });
}
async function runDemoStep(i) {
  const d = P.demo, st = d.steps[i];
  const args = { preset: d.preset, map_tag: d.map_tag, target: st.target || "", max_steps: $("#osg-steps").value || "400" };
  demoBusy = true; renderDemo();
  try {
    if (st.kind === "check") {
      const r = await call("osg_check", { preset: d.preset }, true);
      if (r.ok) setDemo(i + 1); else toast("預檢沒過，看 ② 的清單", true);
    } else if (st.kind === "pause") {
      setDemo(i + 1);
    } else if (st.kind === "map" || st.kind === "search") {
      const unit = st.kind === "map" ? "map_run" : "search_run";
      const r = await osgRun(unit, args); if (!r.ok) return;
      await waitFor(() => ["running", "starting"].includes(osgState(unit)), 15000);
      const end = await waitFor(() => !["running", "starting", "stopping"].includes(osgState(unit)) && osgState(unit), 3600e3);
      const u = osgUnits()[unit] || {};
      if (end === "stopped" && u.exit_code === 0) setDemo(i + 1);
      else toast(`${st.label} 沒有正常結束（${end}），看「進度」的輸出`, true);
    }
  } finally { demoBusy = false; renderDemo(); }
}
let demoKey = "";
function renderDemo() {
  const d = P.demo; if (!d) return;
  const key = `${demoStep}|${demoBusy}`;
  if (key === demoKey) return;          // called on every state: rebuild only on a change
  demoKey = key;
  $("#demo-title").textContent = d.title || "";
  const box = $("#demo-steps");
  box.innerHTML = d.steps.map((st, i) => {
    const cls = i < demoStep ? "done" : i === demoStep ? "now" : "";
    const btn = i === demoStep ? `<button class="btn small primary" data-demo="${i}" ${demoBusy ? "disabled" : ""}>${
      st.kind === "pause" ? "繼續" : demoBusy ? "進行中…" : "執行"}</button>` : "";
    return `<div class="demo-step ${cls}"><span class="n">${i < demoStep ? "✓" : i + 1}</span>
      <div><div>${esc(st.label)}</div><div class="help">${esc(st.help || "")}</div></div>${btn}</div>`;
  }).join("") + (demoStep >= d.steps.length ? `<p class="hint">Demo 完成，結果在「進度」。</p>` : "");
  box.querySelectorAll("[data-demo]").forEach((b) => (b.onclick = () => runDemoStep(+b.dataset.demo)));
}

resizeMap();
connect();
