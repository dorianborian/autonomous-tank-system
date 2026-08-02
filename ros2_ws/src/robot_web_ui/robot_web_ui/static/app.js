// Phase 8 web UI frontend. Vanilla JS, no build step (see web_ui_node.py
// module docstring for why: no Node/npm on this Jetson, and a single-page
// UI like this doesn't need one).

const els = {
  healthDot: document.getElementById('health-dot'),
  healthText: document.getElementById('health-text'),
  connDot: document.getElementById('conn-dot'),
  connText: document.getElementById('conn-text'),
  driveSourceRow: document.getElementById('drive-source-row'),
  trackModeRow: document.getElementById('track-mode-row'),
  doorAngle: document.getElementById('door-angle'),
  doorStatus: document.getElementById('door-status'),
  doorSwing: document.getElementById('door-swing'),
  nodeGrid: document.getElementById('node-grid'),
  logLines: document.getElementById('log-lines'),
  overlayFront: document.getElementById('overlay-front'),
  overlayRear: document.getElementById('overlay-rear'),
  camFront: document.getElementById('cam-front'),
  camRear: document.getElementById('cam-rear'),
  stickDot: document.getElementById('stick-dot'),
  stickX: document.getElementById('stick-x'),
  stickY: document.getElementById('stick-y'),
  estopBtn: document.getElementById('estop-btn'),
  deckSvg: document.getElementById('deck-svg'),
  cloudCanvas: document.getElementById('cloud-canvas'),
  speedSlider: document.getElementById('speed-slider'),
  speedValue: document.getElementById('speed-value'),
  settingsBtn: document.getElementById('settings-btn'),
  settingsOverlay: document.getElementById('settings-overlay'),
  settingsClose: document.getElementById('settings-close'),
  resetCostmapBtn: document.getElementById('reset-costmap-btn'),
  minSpeedSliders: document.querySelectorAll('.min-speed-slider'),
  autoEnableToggle: document.getElementById('auto-enable-toggle'),
  frontInvertToggle: document.getElementById('front-invert-toggle'),
  rearInvertToggle: document.getElementById('rear-invert-toggle'),
};

let speedSliderDragging = false; // suppress telemetry echo while the user is actively moving it
const minSpeedDragging = {}; // per-param dragging suppression, same reason as speedSliderDragging

let bearingDeadband = 0.08; // updated from slow telemetry; see ros_bridge.py
let latestSlow = null;
const clientLog = []; // client-side-only lines (gamepad connect, ws status), merged into log display

function pushClientLog(text) {
  clientLog.push({ t: Date.now() / 1000, text: '[client] ' + text });
  if (clientLog.length > 20) clientLog.shift();
  renderLog();
}

function renderLog() {
  const serverLines = (latestSlow && latestSlow.event_log) || [];
  const merged = serverLines.concat(clientLog).sort((a, b) => a.t - b.t).slice(-16);
  els.logLines.innerHTML = merged.map(l => {
    const d = new Date(l.t * 1000);
    const ts = d.toTimeString().slice(0, 8);
    return `<div class="log-line"><span class="t">${ts}</span>${escapeHtml(l.text)}</div>`;
  }).join('');
  els.logLines.scrollTop = els.logLines.scrollHeight;
}

function escapeHtml(s) {
  return s.replace(/[&<>]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]));
}

// ---------------- WebSocket ----------------
let ws = null;
function connectWs() {
  ws = new WebSocket(`ws://${location.host}/ws/telemetry`);
  ws.onopen = () => {
    els.connDot.className = 'dot ok';
    els.connText.textContent = 'connected';
    pushClientLog('WebSocket connected');
  };
  ws.onclose = () => {
    els.connDot.className = 'dot down';
    els.connText.textContent = 'disconnected — retrying…';
    setTimeout(connectWs, 1000);
  };
  ws.onerror = () => ws.close();
  ws.onmessage = (evt) => {
    const msg = JSON.parse(evt.data);
    if (msg.type === 'fast') handleFast(msg);
    else if (msg.type === 'slow') handleSlow(msg);
  };
}
connectWs();

function sendWs(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
}

// ---------------- Fast telemetry: per-camera bbox + deadzone overlay ----------------
function handleFast(msg) {
  drawOverlay(els.overlayFront, els.camFront, msg.front);
  drawOverlay(els.overlayRear, els.camRear, msg.rear);
  drawPointcloud(els.cloudCanvas, msg.pointcloud || []);
}

// Robot-relative top-down scatter, +x forward +y left (REP-103, matches
// ros_bridge.py's _cloud_cb). Robot drawn fixed at canvas center facing up;
// points scaled to a fixed +/-POINTCLOUD_RANGE_M window so the view doesn't
// jump-zoom as point density/extent changes frame to frame.
const POINTCLOUD_RANGE_M = 3.0;

function drawPointcloud(canvas, points) {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (w === 0 || h === 0) return;
  if (canvas.width !== w) canvas.width = w;
  if (canvas.height !== h) canvas.height = h;
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, w, h);

  const cx = w / 2, cy = h / 2;
  const scale = Math.min(w, h) / (2 * POINTCLOUD_RANGE_M);

  ctx.fillStyle = '#5a5a5a';
  for (const [x, y] of points) {
    const px = cx - y * scale;
    const py = cy - x * scale;
    if (px < 0 || px > w || py < 0 || py > h) continue;
    ctx.fillRect(px - 1, py - 1, 2, 2);
  }

  // Robot marker: small green triangle pointing "up" (forward).
  ctx.fillStyle = '#2e9e4f';
  ctx.beginPath();
  ctx.moveTo(cx, cy - 6);
  ctx.lineTo(cx - 4, cy + 4);
  ctx.lineTo(cx + 4, cy + 4);
  ctx.closePath();
  ctx.fill();
}

function drawOverlay(overlayEl, panelEl, cam) {
  overlayEl.innerHTML = '';

  // Deadzone band: static, centered, +/-bearing_deadband*100% of frame width
  // (per spec: 0.08 -> +/-8% of frame width from center). This is a fixed
  // visual convention tied directly to the live bearing_deadband value from
  // target_pid_node (see ros_bridge.py _refresh_bearing_deadband) -- NOT a
  // second independent re-derivation of the tracker's own deadzone_ratio
  // parameter, which is a different concept (see odin_yolo_tracker_node.py).
  const band = document.createElement('div');
  band.className = 'deadzone-band';
  const halfPct = bearingDeadband * 100;
  band.style.left = (50 - halfPct) + '%';
  band.style.width = (2 * halfPct) + '%';
  overlayEl.appendChild(band);

  if (cam && cam.detected && Array.isArray(cam.bbox) && cam.bbox.length === 4) {
    const [x1, y1, x2, y2] = cam.bbox;
    const rect = document.createElement('div');
    rect.className = 'bbox-rect';
    rect.style.left = (x1 * 100) + '%';
    rect.style.top = (y1 * 100) + '%';
    rect.style.width = ((x2 - x1) * 100) + '%';
    rect.style.height = ((y2 - y1) * 100) + '%';
    const label = document.createElement('div');
    label.className = 'bbox-label';
    label.textContent = `d=${cam.distance.toFixed(2)}m b=${cam.bearing.toFixed(2)}`;
    rect.appendChild(label);
    overlayEl.appendChild(rect);
  }
}

// ---------------- Slow telemetry: modes, door, nodes, health, log ----------------
function handleSlow(msg) {
  latestSlow = msg;
  bearingDeadband = msg.bearing_deadband;

  setActiveButton(els.driveSourceRow, msg.drive_source);
  setActiveButton(els.trackModeRow, msg.track_mode);

  if (!speedSliderDragging) {
    const pct = Math.round(msg.speed_scale * 100);
    els.speedSlider.value = pct;
    els.speedValue.textContent = `${pct}%`;
  }

  els.minSpeedSliders.forEach(slider => {
    const param = slider.dataset.param;
    if (minSpeedDragging[param]) return;
    const value = msg.motor_min_speeds[param];
    if (value === undefined) return;
    slider.value = value;
    document.querySelector(`[data-readout="${param}"]`).textContent = value.toFixed(0);
  });

  els.autoEnableToggle.checked = !!msg.auto_enable_override;
  els.frontInvertToggle.checked = msg.front_bearing_sign < 0;
  els.rearInvertToggle.checked = msg.rear_bearing_sign < 0;

  els.doorAngle.textContent = `${msg.door_angle_deg.toFixed(0)} deg (est.)`;
  els.doorStatus.textContent = msg.door_state;
  els.doorSwing.style.transform = `rotate(${-msg.door_angle_deg}deg)`;

  const ok = msg.node_health.filter(n => n.level === 'ok').length;
  const total = msg.node_health.length;
  els.healthText.textContent = `${ok}/${total} nodes ok`;
  els.healthDot.className = 'dot ' + (ok === total ? 'ok' : (ok >= total * 0.7 ? 'warn' : 'down'));

  els.nodeGrid.innerHTML = msg.node_health.map(n =>
    `<div class="node-row"><span class="dot ${n.level}"></span>${escapeHtml(n.name)}</div>`
  ).join('');

  renderLog();
}

function setActiveButton(row, value) {
  row.querySelectorAll('button').forEach(b => {
    b.classList.toggle('active', b.dataset.value === value);
  });
}

// ---------------- Button wiring ----------------
els.driveSourceRow.addEventListener('click', (e) => {
  const btn = e.target.closest('button');
  if (btn) sendWs({ type: 'command', action: 'drive_source', value: btn.dataset.value });
});
els.trackModeRow.addEventListener('click', (e) => {
  const btn = e.target.closest('button');
  if (btn) sendWs({ type: 'command', action: 'track_mode', value: btn.dataset.value });
});
document.querySelectorAll('[data-door]').forEach(btn => {
  btn.addEventListener('click', () => sendWs({ type: 'command', action: 'door', value: btn.dataset.door }));
});
els.estopBtn.addEventListener('click', () => sendWs({ type: 'command', action: 'estop' }));

// Live-update the % readout while dragging, but only publish the actual
// parameter change on release ('change') -- this hits serial_driver's
// SetParameters service, not a lightweight topic, so firing it on every
// 'input' tick would spam the parameter service for no benefit.
els.speedSlider.addEventListener('input', () => {
  speedSliderDragging = true;
  els.speedValue.textContent = `${els.speedSlider.value}%`;
});
els.speedSlider.addEventListener('change', () => {
  sendWs({ type: 'command', action: 'max_speed', value: Number(els.speedSlider.value) / 100 });
  speedSliderDragging = false;
});

// Per-wheel min-speed sliders (settings menu) -- same drag/release pattern.
els.minSpeedSliders.forEach(slider => {
  const param = slider.dataset.param;
  slider.addEventListener('input', () => {
    minSpeedDragging[param] = true;
    document.querySelector(`[data-readout="${param}"]`).textContent = slider.value;
  });
  slider.addEventListener('change', () => {
    sendWs({ type: 'command', action: 'motor_min_speed', name: param, value: Number(slider.value) });
    minSpeedDragging[param] = false;
  });
});

// Settings modal open/close.
els.settingsBtn.addEventListener('click', () => els.settingsOverlay.classList.add('open'));
els.settingsClose.addEventListener('click', () => els.settingsOverlay.classList.remove('open'));
els.resetCostmapBtn.addEventListener('click', () => sendWs({ type: 'command', action: 'reset_costmap' }));
els.settingsOverlay.addEventListener('click', (e) => {
  if (e.target === els.settingsOverlay) els.settingsOverlay.classList.remove('open');
});

// Auto-drive (no trigger held) toggle -- a discrete click, not a drag, so no
// suppression needed: the backend's optimistic local update means the next
// telemetry echo just confirms what the checkbox already shows.
els.autoEnableToggle.addEventListener('change', () => {
  sendWs({ type: 'command', action: 'auto_enable_override', value: els.autoEnableToggle.checked });
});

// Per-camera turn-direction invert (settings menu, debug) -- same discrete-
// click pattern as auto-enable above.
els.frontInvertToggle.addEventListener('change', () => {
  sendWs({ type: 'command', action: 'bearing_sign', camera: 'front', value: els.frontInvertToggle.checked });
});
els.rearInvertToggle.addEventListener('change', () => {
  sendWs({ type: 'command', action: 'bearing_sign', camera: 'rear', value: els.rearInvertToggle.checked });
});

// ---------------- Gamepad API ----------------
// Test step 1 (per spec): confirm real input BEFORE wiring anything to it.
// This logs full raw gamepad state to the console on connect, and again for
// a few seconds after, so the actual axis/button layout the Deck's browser
// reports can be visually confirmed against the WEB_AXIS_*/WEB_BUTTON_*
// assumptions in web_ui_node.py. Recommend Desktop Mode for this first pass
// specifically because devtools/console are readily available there;
// Gaming Mode's embedded browser does not offer that easily, so the
// on-screen event log also gets a compact echo of the same data as a
// fallback (see logGamepadDebug below).
window.addEventListener('gamepadconnected', (e) => {
  const gp = e.gamepad;
  pushClientLog(`Gamepad connected: "${gp.id}" mapping=${gp.mapping} axes=${gp.axes.length} buttons=${gp.buttons.length}`);
  console.log('[gamepad] connected', gp);
  debugUntil = performance.now() + 8000; // verbose log burst for 8s after connect
});
window.addEventListener('gamepaddisconnected', (e) => {
  pushClientLog(`Gamepad disconnected: "${e.gamepad.id}"`);
});

let debugUntil = 0;
let lastGamepadSendAt = 0;
const GAMEPAD_SEND_PERIOD_MS = 50; // ~20Hz, matches ESP32 watchdog cadence

// Semantic slot -> deck schematic role, using the SAME WEB_AXIS_*/WEB_BUTTON_*
// assumptions as web_ui_node.py's translate_gamepad_to_joy_arrays (W3C
// "standard" gamepad mapping) -- kept in one place here for the visual
// schematic; if live testing shows a different real mapping, both this table
// and the matching constants in web_ui_node.py need the same correction.
const BUTTON_ROLE = {
  0: 'btnA', 1: 'btnB', 2: 'btnX', 3: 'btnY',
  4: 'l1', 5: 'r1', 6: 'l2', 7: 'r2',
  8: 'select', 9: 'start', 10: 'stickL', 11: 'stickR',
  12: 'dpadUp', 13: 'dpadDown', 14: 'dpadLeft', 15: 'dpadRight',
  16: 'home',
};

function pollGamepad() {
  requestAnimationFrame(pollGamepad);
  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  let gp = null;
  for (const p of pads) { if (p) { gp = p; break; } }
  if (!gp) return;

  if (performance.now() < debugUntil) {
    console.log('[gamepad] axes=', gp.axes.map(a => a.toFixed(2)), 'buttons=',
      gp.buttons.map(b => b.pressed ? 1 : 0));
  }

  // Deck schematic highlight (buttons + analog trigger intensity via opacity).
  document.querySelectorAll('.deck-part').forEach(el => el.classList.remove('active'));
  gp.buttons.forEach((b, i) => {
    const role = BUTTON_ROLE[i];
    if (!role) return;
    if (b.pressed || b.value > 0.15) {
      const el = document.getElementById('part-' + role);
      if (el) el.classList.add('active');
    }
  });
  // Sticks: highlight when deflected past a small deadzone, same convention
  // as ps_controller_bridge's own STICK_DEADZONE.
  const lx = gp.axes[0] || 0, ly = gp.axes[1] || 0;
  const rx = gp.axes[2] || 0, ry = gp.axes[3] || 0;
  if (Math.hypot(lx, ly) > 0.15) document.getElementById('part-stickL').classList.add('active');
  if (Math.hypot(rx, ry) > 0.15) document.getElementById('part-stickR').classList.add('active');

  // Left-stick position readout widget.
  els.stickX.textContent = lx.toFixed(2);
  els.stickY.textContent = ly.toFixed(2);
  els.stickDot.style.left = `${50 + lx * 40}%`;
  els.stickDot.style.top = `${50 + ly * 40}%`;

  // Relay to backend at ~20Hz (not every animation frame).
  const now = performance.now();
  if (now - lastGamepadSendAt >= GAMEPAD_SEND_PERIOD_MS) {
    lastGamepadSendAt = now;
    sendWs({ type: 'gamepad', axes: Array.from(gp.axes), buttons: gp.buttons.map(b => b.pressed ? 1 : 0) });
  }
}

buildDeckSchematic(els.deckSvg);
requestAnimationFrame(pollGamepad);
