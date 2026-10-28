// Open play: the phone page. Speaks the phone side of openplay/PROTOCOL.md over /ws.
// Plain ES module, no dependencies. Control ideas come from dashboard/static/tilt.html and
// joystick.html: iOS motion permission asked inside a tap, tilt -> velocity with a deadzone,
// hold-to-drive deadman, fixed send rate, everything released on lift/hide/disconnect.

import { openChess, chessMessage, chessInvolves } from './chess.js';

const RATE_MS = 66;                 // ~15 Hz while a control is held
const TILT_DEAD = 3, TILT_FULL = 25; // degrees (same as tilt.html)
const STICK_DEAD = 0.08;
const JOINTS = ['Base', 'Shoulder', 'Elbow', 'Wrist bend', 'Wrist turn', 'Wrist roll'];
const CHIPS = ['wave hello', 'do a happy dance', 'bow to the audience', 'look around curiously', 'nod yes three times'];
const END_REASONS = {
  time: 'Your time is up. Hope you had fun!',
  left: 'You ended your turn.',
  idle: 'Your turn ended because nothing happened for a minute.',
  skipped: 'The staff moved on to the next person.',
  operator: 'The staff ended your turn.',
};

const $ = (id) => document.getElementById(id);
const el = (tag, cls, text) => { const e = document.createElement(tag); if (cls) e.className = cls; if (text != null) e.textContent = text; return e; };
const store = {
  get(k, d) { try { const v = localStorage.getItem(k); return v === null ? d : v; } catch (_) { return d; } },
  set(k, v) { try { localStorage.setItem(k, v); } catch (_) { /* private mode */ } },
};
const clamp = (v, a = -1, b = 1) => Math.max(a, Math.min(b, v));
const r3 = (v) => Math.round(v * 1000) / 1000;

// ------------------------------------------------------------------ identity
function newPid() {
  if (window.crypto && crypto.randomUUID) return crypto.randomUUID();
  return 'p-' + Math.random().toString(36).slice(2) + Date.now().toString(36);
}
let pid = store.get('openplay-pid', '');
if (!pid) { pid = newPid(); store.set('openplay-pid', pid); }
let myName = store.get('openplay-name', '');

// ------------------------------------------------------------------ state
const S = {
  welcome: null, lobby: null, arm: null, poses: [],
  screen: null, turnRemaining: 0, turnAt: 0, lastTurnMsg: 0, warning: false,
  tab: sessionStorage.getItem('openplay-tab') || 'drive',
  mode: sessionStorage.getItem('openplay-mode') || 'tilt',
  speed: parseFloat(store.get('openplay-speed', '1')) || 1,
  wantJoin: false,
};
const features = () => Object.assign({ say: true, grab: true, tilt: true, joystick: true, poses: true }, (S.welcome && S.welcome.features) || {});

// ------------------------------------------------------------------ connection
let ws = null, backoff = 500, lastRx = 0, reconnectTimer = null, helloName = null;
function wsUrl() {
  const k = new URLSearchParams(location.search).get('k');
  return `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws` + (k ? `?k=${encodeURIComponent(k)}` : '');
}
function send(m) { if (ws && ws.readyState === 1) { ws.send(JSON.stringify(m)); return true; } return false; }
function hello() { helloName = myName; send({ op: 'hello', pid, name: myName }); }
function connect() {
  clearTimeout(reconnectTimer);
  setConn('Connecting…');
  try { ws = new WebSocket(wsUrl()); } catch (e) { scheduleReconnect(); return; }
  const me = ws;
  me.onopen = () => {
    if (me !== ws) return;
    backoff = 500; lastRx = Date.now(); setConn(null); hello();
    if (S.wantJoin) { send({ op: 'join' }); S.wantJoin = false; }
  };
  me.onmessage = (ev) => {
    if (me !== ws) return;
    lastRx = Date.now();
    let m; try { m = JSON.parse(ev.data); } catch (_) { return; }
    handle(m);
  };
  me.onclose = () => {
    if (me !== ws) return;
    releaseAll(); ws = null; S.arm = null; renderArm(); scheduleReconnect();
  };
  me.onerror = () => { try { me.close(); } catch (_) {} };
}
function scheduleReconnect() {
  const wait = Math.min(backoff, 8000) * (0.8 + Math.random() * 0.4);
  backoff = Math.min(backoff * 2, 8000);
  setConn(`Lost the connection. Retrying in ${Math.max(1, Math.round(wait / 1000))} s…`);
  clearTimeout(reconnectTimer);
  reconnectTimer = setTimeout(connect, wait);
}
function setConn(text) { const c = $('conn'); c.hidden = !text; if (text) c.textContent = text; }
// liveness: the server sends arm state at >= 2 Hz; ping so a silent socket is noticed
setInterval(() => {
  if (!ws || ws.readyState !== 1) return;
  send({ op: 'ping' });
  if (Date.now() - lastRx > 12000) { try { ws.close(); } catch (_) {} }
}, 5000);
document.addEventListener('visibilitychange', () => {
  if (document.hidden) releaseAll();
  else if (!ws) { backoff = 500; connect(); }
});
window.addEventListener('pagehide', releaseAll);

// ------------------------------------------------------------------ messages
function handle(m) {
  switch (m.op) {
    case 'welcome':
      S.welcome = m;
      if (m.name && !myName) { myName = m.name; $('name').value = m.name; }
      renderWelcome(); renderLobby(); break;
    case 'lobby':
      S.lobby = m; renderLobby();
      if (m.you_active && S.screen !== 'turn' && m.active) startTurnScreen(m.active.remaining_s);
      if (S.screen === 'turn' && !m.you_active && Date.now() - S.lastTurnMsg > 3000) show('lobby');
      break;
    case 'turn':
      S.turnRemaining = m.remaining_s; S.turnAt = performance.now(); S.lastTurnMsg = Date.now(); S.warning = !!m.warning;
      if (S.screen !== 'turn') startTurnScreen(m.remaining_s);
      renderClock(); break;
    case 'turn_end':
      endTurnScreen(m.reason); break;
    case 'arm':
      S.arm = m; renderArm(); break;
    case 'status':
      // Say and Poses show their own progress card, so a "busy" echo there would just cover it.
      if (m.kind === 'busy' && S.screen === 'turn' && (S.tab === 'say' || S.tab === 'poses')) break;
      toast(m.text, m.kind || 'info'); break;
    case 'refused': {
      const r = m.reason || 'Not now';
      toast(/^busy/i.test(r) && S.screen === 'turn' ? `${r}. Tap STOP first.` : r, 'error', '✋'); break;
    }
    case 'say':
      renderSay(m); break;
    case 'poses':
      S.poses = m.list || []; renderPoses(); break;
    case 'seq':
      renderSeq(m); break;
    case 'chess':
      chessMessage(m); $('chessLobbyCard').hidden = !chessInvolves(); break;
    case 'pong': break;
  }
}

// ------------------------------------------------------------------ screens
function show(name) {
  if (S.screen === name) return;
  S.screen = name;
  for (const id of ['lobby', 'turn', 'thanks']) $(id).hidden = id !== name;
  document.body.classList.toggle('in-turn', name === 'turn');
  window.scrollTo(0, 0); clearToast();
  if (name !== 'turn') releaseAll();
  wakeLock(name === 'turn');
}
function startTurnScreen(remaining) {
  if (remaining != null) { S.turnRemaining = remaining; S.turnAt = performance.now(); }
  S.lastTurnMsg = Date.now();
  $('tbName').textContent = myName ? `, ${myName}` : '';
  show('turn'); selectTab(S.tab); renderClock(); renderArm();
  try { navigator.vibrate && navigator.vibrate([120, 60, 120]); } catch (_) {}
}
function endTurnScreen(reason) {
  releaseAll();
  $('thanksTitle').textContent = myName ? `Thanks for playing, ${myName}!` : 'Thanks for playing!';
  $('thanksWhy').textContent = END_REASONS[reason] || 'Your turn is over.';
  show('thanks');
}

// ------------------------------------------------------------------ lobby
function fmtDur(s) {
  s = Math.max(0, Math.round(s));
  return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`;
}
function fmtEta(s) {
  if (s == null) return '';
  if (s < 60) return 'Less than a minute to go';
  return `About ${Math.round(s / 60)} min to go`;
}
let lastQueueKey = '';
function renderWelcome() {
  const w = S.welcome; if (!w) return;
  if (w.turn_s) $('turnLen').textContent = w.turn_s % 60 ? `${fmtDur(w.turn_s)} min` : `${w.turn_s / 60} min`;
  renderMoves(); renderSpots(); applyFeatures();
}
function renderLobby() {
  const L = S.lobby; if (!L) return;
  const a = L.active, turnS = (S.welcome && S.welcome.turn_s) || 600;
  $('nowName').textContent = a ? (L.you_active ? `${a.name} (you!)` : a.name) : 'Nobody yet: it could be you!';
  $('nowName').classList.toggle('none', !a);
  $('nowTime').textContent = a ? `${fmtDur(a.remaining_s)} left` : '';
  $('nowMeterWrap').hidden = !a;
  if (a) $('nowMeter').style.width = `${clamp(a.remaining_s / turnS, 0, 1) * 100}%`;

  const queued = L.position != null;
  $('joinCard').hidden = queued || L.you_active;
  $('queuedCard').hidden = !queued;
  if (queued) {
    $('pos').textContent = L.position;
    const next = L.position === 1;
    $('queuedCard').classList.toggle('next', next);
    $('posTitle').textContent = next ? (a ? "You're next!" : 'Starting your turn…') : "You're in line";
    $('eta').textContent = fmtEta(L.eta_s);
  }
  const q = L.queue || [];
  const key = JSON.stringify(q);
  $('queueCard').hidden = !q.length;
  if (key !== lastQueueKey) {
    lastQueueKey = key;
    const ol = $('queue'); ol.replaceChildren();
    q.slice(0, 6).forEach((p) => ol.append(el('li', p.you ? 'you' : '', p.name || 'Guest')));
    if (q.length > 6) ol.append(el('li', 'more', `and ${q.length - 6} more`));
  }
}
$('name').value = myName;
function doJoin() {
  const n = $('name').value.trim().slice(0, 24);
  if (!n) { $('name').focus(); $('name').animate([{ transform: 'translateX(-6px)' }, { transform: 'translateX(6px)' }, { transform: 'none' }], { duration: 240 }); toast('Type your name first', 'info', '👋'); return; }
  myName = n; store.set('openplay-name', n);
  if (!ws || ws.readyState !== 1) { S.wantJoin = true; toast('Joining as soon as we reconnect…', 'busy'); return; }
  if (helloName !== myName) hello();
  send({ op: 'join' });
}
$('joinBtn').addEventListener('click', doJoin);
$('name').addEventListener('keydown', (e) => { if (e.key === 'Enter') { e.preventDefault(); $('name').blur(); doJoin(); } });
$('leaveBtn').addEventListener('click', () => send({ op: 'leave' }));
$('againBtn').addEventListener('click', () => { show('lobby'); doJoin(); });
$('lobbyBtn').addEventListener('click', () => show('lobby'));

// ------------------------------------------------------------------ turn: clock + arm readout
const RING_LEN = 2 * Math.PI * 21;
function remainingNow() { return Math.max(0, S.turnRemaining - (performance.now() - S.turnAt) / 1000); }
function renderClock() {
  if (S.screen !== 'turn') return;
  const rem = remainingNow(), total = Math.max((S.welcome && S.welcome.turn_s) || 600, S.turnRemaining);
  $('ringTxt').textContent = fmtDur(Math.ceil(rem));
  $('ringFg').style.strokeDashoffset = String(RING_LEN * (1 - clamp(rem / total, 0, 1)));
  const warn = S.warning || rem <= 30;
  $('ring').classList.toggle('warning', warn);
  $('ring').classList.toggle('final', rem <= 10);
  $('tbTitle').firstChild.textContent = warn ? (rem < 1 ? 'Time!' : 'Last 30 seconds!') : 'Your turn';
  $('tbName').hidden = warn;
  document.querySelector('.topbar').classList.toggle('warning', warn);
}
setInterval(renderClock, 250);
function renderArm() {
  const a = S.arm, pill = $('armPill'), txt = pill.querySelector('span');
  let cls = '', t = ws && ws.readyState === 1 ? 'waiting for the arm' : 'offline: reconnecting';
  if (a) {
    if (a.busy) { cls = 'busy'; t = `busy: ${a.busy}`; }
    else if (driving()) { cls = 'moving'; t = "you're driving"; }
    else if (a.moving) { cls = 'moving'; t = 'moving'; }
    else { cls = 'still'; t = 'still, ready for you'; }
  }
  pill.className = 'armpill ' + cls; txt.textContent = t;
  const busy = a && a.busy;
  $('busyBar').hidden = !busy;
  if (busy) $('busyTxt').textContent = `The arm is busy: ${busy}`;
  for (const c of document.querySelectorAll('.move')) c.classList.toggle('playing', !!busy && (c.dataset.name === busy || c.dataset.title === busy));
  $('moves').classList.toggle('dim', !!busy);
  for (const j of jogEls) j.q.textContent = a && a.q ? `${a.q[j.i].toFixed(2)} rad` : '';
  if (a && typeof a.grip === 'number' && Date.now() - gripTouched > 1500) $('gripSlider').value = a.grip;
}
$('endBtn').addEventListener('click', () => { if (confirm('End your turn now? The next person will get the arm.')) send({ op: 'leave' }); });

// ------------------------------------------------------------------ tabs + features
function applyFeatures() {
  const f = features(), w = S.welcome || {};
  const has = {
    drive: f.tilt || f.joystick,
    moves: (w.moves || []).length > 0,
    say: f.say, poses: f.poses,
    grab: f.grab && (w.spots || []).length > 1,
    chess: !!f.chess,
  };
  for (const b of $('tabs').children) b.hidden = !has[b.dataset.tab];
  const modes = { tilt: f.tilt, sticks: f.joystick, joints: f.joystick };
  for (const b of $('driveModes').children) b.hidden = !modes[b.dataset.mode];
  if (!modes[S.mode]) S.mode = Object.keys(modes).find((k) => modes[k]) || 'tilt';
  if (!has[S.tab]) S.tab = Object.keys(has).find((k) => has[k]) || 'drive';
  selectTab(S.tab); selectMode(S.mode);
}
function selectTab(name) {
  S.tab = name; sessionStorage.setItem('openplay-tab', name);
  for (const t of document.querySelectorAll('.tab')) t.hidden = t.dataset.tab !== name;
  for (const b of $('tabs').children) b.setAttribute('aria-selected', String(b.dataset.tab === name));
  if (name !== 'drive') releaseAll();
  window.scrollTo(0, 0);
}
function selectMode(name) {
  S.mode = name; sessionStorage.setItem('openplay-mode', name);
  releaseAll();
  for (const m of ['tilt', 'sticks', 'joints']) $('mode-' + m).hidden = m !== name;
  for (const b of $('driveModes').children) b.setAttribute('aria-selected', String(b.dataset.mode === name));
}
for (const b of $('tabs').children) b.addEventListener('click', () => selectTab(b.dataset.tab));
for (const b of $('driveModes').children) b.addEventListener('click', () => selectMode(b.dataset.mode));
function selectSpeed(v) {
  S.speed = v; store.set('openplay-speed', String(v));
  for (const b of $('speedSeg').children) b.setAttribute('aria-selected', String(parseFloat(b.dataset.speed) === v));
}
for (const b of $('speedSeg').children) b.addEventListener('click', () => selectSpeed(parseFloat(b.dataset.speed)));
selectSpeed(S.speed);

// ------------------------------------------------------------------ live control (stream)
const ctl = {
  tilt: false, z: 0,
  sticks: { L: { x: 0, y: 0, on: false }, R: { x: 0, y: 0, on: false } },
  jog: [0, 0, 0, 0, 0, 0], jogOn: new Set(),
};
let streamTimer = null, ori = { beta: null, gamma: null }, neutral = null, sensorsOn = false;
const driving = () => ctl.tilt || ctl.z !== 0 || ctl.sticks.L.on || ctl.sticks.R.on || ctl.jogOn.size > 0;
// Refuse to start a live control while the arm is running an action or it's not our turn.
function canDrive() {
  if (S.screen !== 'turn') return false;
  if (!ws || ws.readyState !== 1) { toast('Not connected right now. Reconnecting…', 'error', '📡'); return false; }
  if (S.arm && S.arm.busy) { toast(`The arm is busy: ${S.arm.busy}. Tap STOP first.`, 'busy', '⏳'); return false; }
  return true;
}
function axis(d) { const a = Math.abs(d); return a < TILT_DEAD ? 0 : Math.sign(d) * Math.min(1, (a - TILT_DEAD) / (TILT_FULL - TILT_DEAD)); }
const dz = (v) => (Math.abs(v) < STICK_DEAD ? 0 : (v - Math.sign(v) * STICK_DEAD) / (1 - STICK_DEAD));
function tiltCmd() {
  if (!ctl.tilt || !neutral || ori.beta === null) return { x: 0, y: 0 };
  // same mapping as tilt.html: tilt the top away = +x (away), tilt left = +y
  return { x: axis(-(ori.beta - neutral.beta)), y: axis(-(ori.gamma - neutral.gamma)) };
}
function tick() {
  if (!driving()) { stopStream(); return; }
  const k = S.speed;
  if (S.mode === 'joints' && ctl.jogOn.size) {
    send({ op: 'stream', qd: ctl.jog.map((v) => r3(clamp(v) * k)) });
  } else {
    let v = { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 };
    if (S.mode === 'tilt') { const c = tiltCmd(); v.x = c.x; v.y = c.y; v.z = ctl.z; }
    else if (S.mode === 'sticks') {
      const L = ctl.sticks.L, R = ctl.sticks.R; // same mapping as joystick.html
      v.x = -dz(L.y); v.y = -dz(L.x); v.z = -dz(R.y); v.yaw = -dz(R.x);
    }
    for (const key in v) v[key] = r3(clamp(v[key]) * k);
    send({ op: 'stream', v });
  }
  drawDot();
}
function startStream() { if (!streamTimer) { tick(); streamTimer = setInterval(tick, RATE_MS); renderArm(); } }
function stopStream() {
  if (streamTimer) {
    clearInterval(streamTimer); streamTimer = null;
    // one explicit zero so the arm stops now rather than at the server's 300 ms deadman
    if (S.mode === 'joints') send({ op: 'stream', qd: [0, 0, 0, 0, 0, 0] });
    else send({ op: 'stream', v: { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 } });
    renderArm();
  }
}
function maybeStop() { if (!driving()) stopStream(); }
function releaseAll() {
  ctl.tilt = false; ctl.z = 0; ctl.jogOn.clear(); ctl.jog.fill(0);
  for (const s of Object.values(ctl.sticks)) { s.on = false; s.x = s.y = 0; s.reset && s.reset(); }
  for (const j of jogEls) j.reset();
  for (const id of ['tiltPad', 'zUp', 'zDown']) $(id).classList.remove('on');
  stopStream(); drawDot();
}
function holdable(node, down, up) {
  let pidHeld = null;
  node.addEventListener('pointerdown', (e) => {
    if (pidHeld !== null || !canDrive()) return;
    e.preventDefault(); pidHeld = e.pointerId;
    try { node.setPointerCapture(e.pointerId); } catch (_) {}
    node.classList.add('on'); down(e);
  });
  node.addEventListener('pointermove', (e) => { if (e.pointerId === pidHeld && node._move) node._move(e); });
  const end = (e) => { if (e.pointerId !== pidHeld) return; pidHeld = null; node.classList.remove('on'); up(e); };
  for (const t of ['pointerup', 'pointercancel', 'lostpointercapture']) node.addEventListener(t, end);
  node.addEventListener('contextmenu', (e) => e.preventDefault());
  return () => { pidHeld = null; node.classList.remove('on'); };
}

// tilt ------------------------------------------------------------
async function enableSensors() {
  const msg = $('sensorMsg');
  try {
    const E = window.DeviceOrientationEvent;
    if (!E) throw new Error('This browser has no motion sensors. Try Sticks instead.');
    if (typeof E.requestPermission === 'function') { // iOS 13+: only inside a tap
      const r = await E.requestPermission();
      if (r !== 'granted') throw new Error('Motion access was not allowed. Reload the page to ask again, or use Sticks.');
    }
    window.addEventListener('deviceorientation', onOrientation);
    msg.textContent = 'Waiting for sensor data…';
    setTimeout(() => { if (!sensorsOn) msg.textContent = 'No tilt data yet. This device may not have motion sensors: try Sticks.'; }, 2500);
  } catch (e) { msg.textContent = e.message; }
}
function onOrientation(e) {
  if (e.beta === null || e.beta === undefined) return;
  ori = { beta: e.beta, gamma: e.gamma };
  if (!sensorsOn) {
    sensorsOn = true;
    $('sensorGate').hidden = true; $('tiltPad').hidden = false; $('zRow').hidden = false; $('tiltHint').hidden = false;
  }
  if (ctl.tilt) drawDot();
}
function drawDot() {
  const c = tiltCmd();
  const pad = $('tiltPad'), r = pad.getBoundingClientRect();
  if (r.width) $('tiltDot').style.transform = `translate(${-c.y * (r.width / 2 - 30)}px, ${-c.x * (r.height / 2 - 30)}px)`;
}
$('sensorsBtn').addEventListener('click', enableSensors);
holdable($('tiltPad'), () => { neutral = { ...ori }; ctl.tilt = true; startStream(); }, () => { ctl.tilt = false; drawDot(); maybeStop(); });
holdable($('zUp'), () => { ctl.z = 1; startStream(); }, () => { ctl.z = 0; maybeStop(); });
holdable($('zDown'), () => { ctl.z = -1; startStream(); }, () => { ctl.z = 0; maybeStop(); });

// sticks ------------------------------------------------------------
function setupStick(id, s) {
  const node = $(id), knob = node.querySelector('.knob');
  const update = (e) => {
    const r = node.getBoundingClientRect(), rad = r.width / 2;
    let dx = (e.clientX - (r.left + rad)) / (rad * 0.7), dy = (e.clientY - (r.top + rad)) / (rad * 0.7);
    const n = Math.hypot(dx, dy); if (n > 1) { dx /= n; dy /= n; }
    s.x = dx; s.y = dy;
    knob.style.transform = `translate(${dx * rad * 0.7}px, ${dy * rad * 0.7}px)`;
  };
  node._move = update;
  const reset = holdable(node, (e) => { s.on = true; update(e); startStream(); },
    () => { s.on = false; s.x = s.y = 0; knob.style.transform = ''; maybeStop(); });
  s.reset = () => { reset(); knob.style.transform = ''; };
}
setupStick('stickL', ctl.sticks.L); setupStick('stickR', ctl.sticks.R);

// joint jog bars (spring back to zero) -------------------------------
const jogEls = [];
JOINTS.forEach((label, i) => {
  const node = el('div', 'jog');
  node.append(el('div', 'mid'));
  const fill = el('div', 'fill'); node.append(fill);
  node.append(el('span', 'jl', `${i + 1} · ${label}`));
  const q = el('span', 'jq', ''); node.append(q);
  $('jogs').append(node);
  const set = (v) => {
    ctl.jog[i] = v;
    fill.style.left = v >= 0 ? '50%' : `${50 + v * 50}%`;
    fill.style.width = `${Math.abs(v) * 50}%`;
  };
  node._move = (e) => {
    const r = node.getBoundingClientRect();
    const v = clamp(((e.clientX - (r.left + r.width / 2)) / (r.width / 2)) * 1.15);
    set(Math.abs(v) < 0.1 ? 0 : v);
  };
  const reset = holdable(node, (e) => { ctl.jogOn.add(i); node._move(e); startStream(); },
    () => { ctl.jogOn.delete(i); set(0); maybeStop(); });
  jogEls.push({ i, q, reset: () => { reset(); set(0); } });
});

// gripper + ready ---------------------------------------------------
let gripTouched = 0, gripSentAt = 0, gripTimer = null;
function sendGrip(v, now) {
  gripTouched = Date.now();
  clearTimeout(gripTimer);
  const go = () => { gripSentAt = Date.now(); send({ op: 'grip', value: r3(clamp(v, 0, 1)) }); };
  if (now || Date.now() - gripSentAt > 200) go(); else gripTimer = setTimeout(go, 200);
}
$('gripSlider').addEventListener('input', () => sendGrip(parseFloat($('gripSlider').value)));
$('gripSlider').addEventListener('change', () => sendGrip(parseFloat($('gripSlider').value), true));
$('gripOpen').addEventListener('click', () => { $('gripSlider').value = 1; sendGrip(1, true); });
$('gripClose').addEventListener('click', () => { $('gripSlider').value = 0; sendGrip(0, true); });
$('readyBtn').addEventListener('click', () => { releaseAll(); send({ op: 'ready' }); });

// STOP ------------------------------------------------------------
$('stopBtn').addEventListener('click', () => {
  releaseAll(); send({ op: 'stop' });
  try { navigator.vibrate && navigator.vibrate(40); } catch (_) {}
});

// ------------------------------------------------------------------ moves
function renderMoves() {
  const box = $('moves'); box.replaceChildren();
  for (const m of (S.welcome && S.welcome.moves) || []) {
    const b = el('button', 'move'); b.dataset.name = m.name; b.dataset.title = m.title || m.name;
    b.append(el('span', 'em', m.emoji || '🤖'), el('span', 'tt', m.title || m.name));
    if (m.blurb) b.append(el('span', 'bl', m.blurb));
    if (m.duration_s) b.append(el('span', 'du', `${Math.round(m.duration_s)} s`));
    b.addEventListener('click', () => { releaseAll(); send({ op: 'move', name: m.name }); });
    box.append(b);
  }
  renderArm();
}

// ------------------------------------------------------------------ say
const sayText = $('sayText');
function updateCount() { $('sayCount').textContent = `${sayText.value.length}/200`; $('sayBtn').disabled = !sayText.value.trim(); }
sayText.addEventListener('input', updateCount);
sayText.addEventListener('keydown', (e) => { if (e.key === 'Enter' && !e.shiftKey) { e.preventDefault(); doSay(); } });
function doSay(text) {
  const t = (text || sayText.value).trim().slice(0, 200);
  if (!t) return;
  releaseAll(); sayText.blur();
  if (send({ op: 'say', text: t })) renderSay({ stage: 'sending', text: t });
}
$('sayBtn').addEventListener('click', () => doSay());
for (const c of CHIPS) {
  const b = el('button', 'chip', c);
  b.addEventListener('click', () => { sayText.value = c; updateCount(); doSay(c); });
  $('sayChips').append(b);
}
updateCount();
function renderSay(m) {
  const box = $('sayProg'); box.hidden = false;
  box.className = 'card sayprog ' + m.stage;
  const stage = $('spStage'); stage.replaceChildren();
  const icon = { sending: '📨', thinking: '🤔', playing: '🦾', done: '✅', error: '😕' }[m.stage] || '🤖';
  $('spIcon').textContent = icon;
  if (m.stage === 'thinking' || m.stage === 'sending') { stage.append('Thinking up a move', el('span', 'dots')); }
  else if (m.stage === 'playing') stage.textContent = `Playing: ${m.title || 'your move'}`;
  else if (m.stage === 'done') stage.textContent = m.title ? `Done: ${m.title}` : 'Done!';
  else stage.textContent = "Couldn't do that one";
  if (m.text) $('spText').textContent = `“${m.text}”`;
  if (m.stage === 'sending' || m.stage === 'thinking') box.scrollIntoView({ block: 'nearest', behavior: 'smooth' });
  const note = $('spNote'); note.hidden = !m.note; note.textContent = m.note || '';
  const wrap = $('spMeterWrap'), fill = $('spMeter');
  if (m.stage === 'playing' && m.duration_s) {
    wrap.hidden = false; fill.style.transition = 'none'; fill.style.width = '0';
    void fill.offsetWidth; fill.style.transition = `width ${m.duration_s}s linear`; fill.style.width = '100%';
  } else if (m.stage === 'done') { fill.style.transition = 'none'; fill.style.width = '100%'; }
  else wrap.hidden = true;
}

// ------------------------------------------------------------------ poses + sequence
$('poseSave').addEventListener('click', () => { releaseAll(); send({ op: 'pose.save' }); });
$('poseClear').addEventListener('click', () => { if (confirm('Delete all your saved poses?')) send({ op: 'pose.clear' }); });
$('seqGentle').addEventListener('click', () => { releaseAll(); send({ op: 'seq.play', style: 'gentle' }); });
$('seqLively').addEventListener('click', () => { releaseAll(); send({ op: 'seq.play', style: 'lively' }); });
function renderPoses() {
  const ol = $('poses'); ol.replaceChildren();
  S.poses.forEach((p, n) => {
    const li = el('li', 'pose');
    li.append(el('span', 'pn', String(n + 1)));
    const bars = el('div', 'bars'); bars.title = 'joint angles';
    for (const q of p.q || []) {
      const i = el('i', q < 0 ? 'neg' : '');
      i.style.setProperty('--h', `${Math.min(50, (Math.abs(q) / Math.PI) * 50)}%`);
      bars.append(i);
    }
    li.append(bars);
    if (typeof p.grip === 'number') li.append(el('span', 'gp', p.grip > 0.5 ? '🖐️ open' : '✊ closed'));
    const go = el('button', 'btn', 'Go');
    go.addEventListener('click', () => { releaseAll(); send({ op: 'pose.goto', i: p.i }); });
    const del = el('button', 'btn del', '🗑️'); del.setAttribute('aria-label', `delete pose ${n + 1}`);
    del.addEventListener('click', () => send({ op: 'pose.delete', i: p.i }));
    li.append(go, del); ol.append(li);
  });
  const n = S.poses.length;
  $('posesEmpty').hidden = n > 0;
  $('seqGentle').disabled = $('seqLively').disabled = n < 1;
  $('poseClear').hidden = n === 0;
}
renderPoses();
function renderSeq(m) {
  const st = $('seqStat'); st.hidden = false; st.className = 'seqstat ' + m.stage;
  const d = m.duration_s ? ` (${Math.round(m.duration_s * 10) / 10} s)` : '';
  st.textContent = m.stage === 'playing' ? `▶ Playing your sequence${d}` : m.stage === 'done' ? '✅ Sequence done' : '😕 Could not play it';
  const ul = $('seqNotes'); ul.replaceChildren();
  for (const n of m.notes || []) ul.append(el('li', '', n));
}

// ------------------------------------------------------------------ grab
const grab = { from: null, to: null };
function renderSpots() {
  const spots = (S.welcome && S.welcome.spots) || [];
  for (const [boxId, key] of [['spotsFrom', 'from'], ['spotsTo', 'to']]) {
    const box = $(boxId); box.replaceChildren();
    for (const s of spots) {
      const b = el('button', 'spot', s.title || s.name); b.dataset.name = s.name;
      b.addEventListener('click', () => { grab[key] = s.name; if (grab.from === grab.to) grab[key === 'from' ? 'to' : 'from'] = null; renderGrab(); });
      box.append(b);
    }
  }
  renderGrab();
}
function renderGrab() {
  const spots = (S.welcome && S.welcome.spots) || [];
  const title = (n) => (spots.find((s) => s.name === n) || {}).title || n;
  for (const [boxId, key] of [['spotsFrom', 'from'], ['spotsTo', 'to']])
    for (const b of $(boxId).children) b.setAttribute('aria-pressed', String(grab[key] === b.dataset.name));
  const sum = $('grabSum'); sum.replaceChildren();
  if (grab.from && grab.to) sum.append('Pick up at ', el('b', '', title(grab.from)), ', put down at ', el('b', '', title(grab.to)));
  else sum.textContent = grab.from ? 'Now choose where it goes.' : 'Choose a "from" and a "to" spot.';
  $('grabBtn').disabled = !(grab.from && grab.to);
  if (grab.from && grab.to) $('grabBtn').scrollIntoView({ block: 'nearest', behavior: 'smooth' });
}
$('grabBtn').addEventListener('click', () => { releaseAll(); send({ op: 'grab', from: grab.from, to: grab.to }); });

// ------------------------------------------------------------------ toasts
// One toast at a time: the newest replaces the last, so messages never pile up over the controls.
let toastTimer = null, toastText = '';
function toast(text, kind = 'info', icon) {
  if (!text) return;
  const box = $('toasts');
  let t = box.firstChild;
  if (!t || text !== toastText) {
    box.replaceChildren();
    t = el('div', 'toast ' + kind);
    t.append(el('b', '', icon || { done: '✅', busy: '⏳', error: '⚠️', info: '💡' }[kind] || '💡'), el('span', '', text));
    t.addEventListener('click', clearToast);
    box.append(t); toastText = text;
  }
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { t.classList.add('out'); setTimeout(() => { if (t.parentNode) t.remove(); if (!box.firstChild) toastText = ''; }, 260); },
    kind === 'error' ? 4000 : 2400);
}
function clearToast() { clearTimeout(toastTimer); $('toasts').replaceChildren(); toastText = ''; }

// ------------------------------------------------------------------ wake lock
let wake = null;
async function wakeLock(on) {
  try {
    if (on && !wake && navigator.wakeLock && !document.hidden) { wake = await navigator.wakeLock.request('screen'); wake.onrelease = () => { wake = null; }; }
    if (!on && wake) { await wake.release(); wake = null; }
  } catch (_) { /* not supported or not allowed */ }
}
document.addEventListener('visibilitychange', () => { if (!document.hidden && S.screen === 'turn') wakeLock(true); });

// ------------------------------------------------------------------ start
show('lobby');
for (const id of ['chessTurn', 'chessLobby']) openChess($(id), { send, toast });
applyFeatures();
connect();
window.openplay = { S, send, ctl, ws: () => ws }; // debugging / tests
