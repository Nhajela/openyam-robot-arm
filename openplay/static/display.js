// Open play laptop display: the show (arm in 3D, QR, now playing, captions, Claude "say" cards with
// a ghost preview) plus the operator rail (E-STOP, approvals, turn controls, queue, joints).
// Talks to openplay.server over ws://<host>/display/ws (see openplay/PROTOCOL.md, display side).
// The arm view is three.js + the URDF when the server has it (hello.urdf) and the CDN loads;
// otherwise a self-contained canvas rendering from kin.js (works offline).
import * as K from './kin.js';

const $ = (id) => document.getElementById(id);
const esc = (t) => String(t ?? '').replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const clamp = (v, a = 0, b = 1) => Math.min(b, Math.max(a, v));
const fmtClock = (s) => { s = Math.max(0, Math.ceil(s)); return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`; };
const agoText = (ms) => { const s = Math.round(ms / 1000); return s < 60 ? `${s}s` : s < 3600 ? `${Math.round(s / 60)}m` : `${Math.round(s / 3600)}h`; };
const READY = K.POSES.ready;
const JOINTS = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'];

// ------------------------------------------------------------------ state
const S = {
  ws: null, wsOpen: false, wsEverOpen: false, retry: 0,
  hello: null,
  cfg: { auto_play: null, paused: false, turn_s: 600 },
  arm: null, armAt: 0, armCount: 0, armHz: 0,
  lobby: { queue: [], active: null, players: 0 }, lobbyAt: 0, turnTotal: { pid: null, s: 600 },
  say: null,        // {stage, text, title, duration_s, note, who, kfs, dur, at}
  pending: null,    // {id, who, title, duration_s, kfs, dur, at}
  problems: [],     // [{at, text, cls}]
};

// ------------------------------------------------------------------ websocket
function wsUrl() { return (location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/display/ws'; }
function connect() {
  let ws;
  try { ws = new WebSocket(wsUrl()); } catch (e) { scheduleReconnect(); return; }
  S.ws = ws;
  ws.onopen = () => { S.wsOpen = true; S.wsEverOpen = true; S.retry = 0; updateBanner(); updateHealth(); };
  ws.onmessage = (ev) => { let m; try { m = JSON.parse(ev.data); } catch (e) { return; } handle(m); };
  ws.onclose = () => { S.wsOpen = false; S.ws = null; updateBanner(); updateHealth(); scheduleReconnect(); };
  ws.onerror = () => { /* onclose follows */ };
}
function scheduleReconnect() { S.retry++; setTimeout(connect, Math.min(5000, 400 * S.retry)); }
function send(op, fields = {}, label) {
  if (!S.ws || !S.wsOpen) { toast('Not connected to the open-play server'); return false; }
  S.ws.send(JSON.stringify({ op, ...fields }));
  if (label) toast(label);
  return true;
}

function handle(m) {
  switch (m.op) {
    case 'hello': onHello(m); break;
    case 'arm': onArm(m); break;
    case 'lobby': onLobby(m); break;
    case 'event': onEvent(m); break;
    case 'say': onSay(m); break;
    case 'pending': onPending(Object.prototype.hasOwnProperty.call(m, 'pending') ? m.pending : (m.id != null ? m : null)); break;
    case 'config': onConfig(m); break;
    case 'refused': case 'error':
      addProblem(m.reason || m.text || m.op, 'warn'); toast('Server: ' + (m.reason || m.text || m.op)); break;
    default: break;
  }
}

// ------------------------------------------------------------------ message handlers
function onHello(m) {
  S.hello = m;
  onConfig(m);
  const real = m.backend && m.backend !== 'mock';
  const pill = $('backendPill');
  pill.textContent = m.backend ? (real ? `real arm · ${m.backend}` : 'mock arm') : 'backend ?';
  pill.className = 'pill ' + (real ? 'real' : 'mock');
  const rb = $('railBackend');
  rb.textContent = m.backend || '?';
  rb.className = 'rail-backend ' + (real ? 'real' : 'mock');
  $('qrImg').src = '/qr.png?v=' + Date.now();
  $('joinUrl').textContent = m.join_url || '';
  buildJoints();
  if (m.urdf) tryThree();
  renderLobby();
}
function onConfig(m) {
  for (const k of ['auto_play', 'paused', 'turn_s']) if (m[k] !== undefined && m[k] !== null) S.cfg[k] = m[k];
  $('autoPlay').checked = !!S.cfg.auto_play;
  $('pauseTurns').checked = !!S.cfg.paused;
  $('autoPlayHelp').textContent = S.cfg.auto_play
    ? "on: players' moves play without your approval"
    : 'off: you approve each move here (Enter / Esc)';
  document.querySelectorAll('#turnLen button').forEach((b) => b.classList.toggle('on', +b.dataset.s === +S.cfg.turn_s));
  $('inviteMinutes').textContent = Math.round((S.cfg.turn_s || 600) / 60);
  renderLobby();
}
function onArm(m) {
  const wasEstop = !!(S.arm && S.arm.estop);
  S.arm = m; S.armAt = performance.now(); S.armCount++;
  if (Array.isArray(m.q) && m.q.length >= 6) target.q = m.q.slice(0, 6).map(Number);
  if (Number.isFinite(+m.grip)) target.grip = +m.grip;
  if (!!m.estop !== wasEstop) { updateBanner(); updateHealth(); $('resetBtn').disabled = !m.estop; }
}
function onLobby(m) {
  S.lobby = { queue: Array.isArray(m.queue) ? m.queue : [], active: m.active || null, players: m.players | 0 };
  S.lobbyAt = performance.now();
  const a = S.lobby.active;
  if (a) {
    if (S.turnTotal.pid !== a.pid) S.turnTotal = { pid: a.pid, s: Math.max(S.cfg.turn_s || 0, a.remaining_s || 0) };
    else S.turnTotal.s = Math.max(S.turnTotal.s, a.remaining_s || 0);
  } else S.turnTotal = { pid: null, s: S.cfg.turn_s };
  renderLobby();
}
function onEvent(m) {
  addCaption(m);
  if (m.kind === 'refused') addProblem((m.who && !String(m.text || '').includes(m.who) ? m.who + ': ' : '') + (m.text || 'refused'), 'warn');
  if (m.kind === 'estop') addProblem(m.text || 'E-STOP', 'bad');
}
function onSay(m) {
  const now = performance.now();
  if (m.stage === 'thinking' || !S.say || (m.text && S.say.text !== m.text && m.stage !== 'done')) {
    S.say = { at: now };
  }
  Object.assign(S.say, m);
  if (m.stage === 'playing') {
    S.say.kfs = K.normaliseKeyframes(m.keyframes);
    S.say.dur = Math.max(K.keyframesDuration(S.say.kfs), +m.duration_s || 0);
    S.say.playAt = now;
  }
  if (m.stage === 'error') addProblem(`${m.who || 'someone'} asked "${m.text || ''}": ${m.note || 'failed'}`, 'warn');
  S.say.stageAt = now;
  renderHero();
}
function onPending(p) {
  if (!p) { S.pending = null; }
  else {
    const kfs = K.normaliseKeyframes(p.keyframes);
    S.pending = { ...p, kfs, dur: Math.max(K.keyframesDuration(kfs), +p.duration_s || 0), at: performance.now() };
  }
  renderPending();
}

// ------------------------------------------------------------------ show: lobby, now playing
function renderLobby() {
  const L = S.lobby, a = L.active;
  const queue = L.queue.filter((p) => !a || p.pid !== a.pid);
  document.body.classList.toggle('attract', !a);
  $('nowName').textContent = a ? a.name : '–';
  const list = $('nextList');
  const shown = queue.slice(0, 5);
  const nextKey = JSON.stringify(shown.map((p) => [p.pid, p.name]));
  if (list.dataset.key !== nextKey) { // lobby arrives every second: only touch the DOM on a change
    list.dataset.key = nextKey;
    list.innerHTML = shown.length ? shown.map((p) => `<li>${esc(p.name)}</li>`).join('') : '<li class="empty">Nobody yet. Scan to be next!</li>';
  }
  $('nextMore').textContent = queue.length > 5 ? `+ ${queue.length - 5} more` : '';
  $('nextCard').classList.toggle('empty-q', !queue.length);
  const n = queue.length;
  $('queueCount').textContent = n === 0 ? (a ? 'Nobody waiting: you could be next' : 'No queue: play right away')
    : n === 1 ? '1 person in the queue' : `${n} people in the queue`;
  // attract headline
  let kicker = 'Open play', title = 'Move a real robot arm<br>with your phone';
  if (!a && S.cfg.paused) { kicker = 'Short break'; title = 'Back in a moment.<br>Join the queue now'; }
  else if (!a && n) { kicker = 'Next turn starting'; title = `Get ready,<br>${esc(queue[0].name)}!`; }
  $('inviteKicker').textContent = kicker;
  $('inviteTitle').innerHTML = title;
  renderRailQueue(queue);
}
function remainingNow() {
  const a = S.lobby.active;
  if (!a || !Number.isFinite(+a.remaining_s)) return null;
  return Math.max(0, a.remaining_s - (performance.now() - S.lobbyAt) / 1000);
}
let lastRingText = '';
function tickNow() {
  const rem = remainingNow();
  const fg = $('ringFg');
  if (rem === null) { fg.style.strokeDashoffset = 326.73; if (lastRingText !== '–') $('ringTime').textContent = lastRingText = '–'; return; }
  const frac = clamp(rem / Math.max(1, S.turnTotal.s || S.cfg.turn_s || 600));
  fg.style.strokeDashoffset = (326.73 * (1 - frac)).toFixed(1);
  fg.classList.toggle('warn', rem <= 30);
  const t = fmtClock(rem);
  if (t !== lastRingText) { $('ringTime').textContent = lastRingText = t; const rr = $('railRem'); if (rr) rr.textContent = t; }
  const arm = S.arm || {};
  const doing = arm.busy ? `Playing ${arm.busy}` : arm.moving ? 'is moving the arm' : 'has the controls';
  const el = $('nowDoing');
  if (el.textContent !== doing) { el.textContent = doing; el.classList.toggle('busy', !!arm.busy); }
}

// ------------------------------------------------------------------ captions
const capEls = [];
function addCaption(m) {
  const text = String(m.text || '');
  if (!text) return;
  const el = document.createElement('div');
  el.className = 'cap';
  let html = esc(text);
  if (m.who) {
    const w = esc(m.who);
    html = html.includes(w) ? html.replace(w, `<b>${w}</b>`) : `<b>${w}</b> · ${html}`;
  }
  el.innerHTML = `<span class="k k-${esc(m.kind || 'info')}"></span><span>${html}</span>`;
  el.dataset.at = performance.now();
  $('captions').appendChild(el);
  capEls.push(el);
  while (capEls.length > 5) capEls.shift().remove();
  restyleCaptions();
}
function restyleCaptions() {
  const now = performance.now();
  capEls.forEach((el, i) => {
    const age = capEls.length - 1 - i;
    el.classList.toggle('newest', age === 0);
    for (let k = 1; k <= 4; k++) el.classList.toggle('age' + k, age === k);
    const life = age === 0 ? 90000 : 30000;
    el.classList.toggle('stale', now - +el.dataset.at > life);
  });
}

// ------------------------------------------------------------------ say hero card
let heroHideTimer = null;
function renderHero() {
  const s = S.say, hero = $('hero');
  clearTimeout(heroHideTimer);
  if (!s) { hero.hidden = true; return; }
  hero.hidden = false; hero.classList.remove('out');
  $('heroWho').textContent = s.who || 'Someone';
  $('heroText').textContent = s.text || '';
  $('heroNote').textContent = s.note || '';
  const st = $('heroStage');
  hero.classList.toggle('playing', s.stage === 'playing');
  st.className = 'hero-stage ' + (s.stage || '');
  if (s.stage === 'thinking') st.innerHTML = '<span class="dots"><i></i><i></i><i></i></span><span class="shimmer">Claude is choreographing…</span>';
  else if (s.stage === 'playing') st.innerHTML = `<i class="ghost-dot"></i><span id="heroPlayText"></span>`;
  else if (s.stage === 'done') st.innerHTML = `<span>✓ Done${s.title ? ': ' + esc(s.title) : ''}</span>`;
  else if (s.stage === 'error') st.innerHTML = `<span>Couldn't do that one${s.note ? '' : ', try asking another way'}</span>`;
  if (s.stage === 'done' || s.stage === 'error') {
    heroHideTimer = setTimeout(() => {
      hero.classList.add('out');
      heroHideTimer = setTimeout(() => { if (S.say && (S.say.stage === 'done' || S.say.stage === 'error')) { S.say = null; hero.hidden = true; } }, 650);
    }, s.stage === 'done' ? 4000 : 8000);
  }
}
let lastPlayText = '';
function tickHero() {
  const s = S.say;
  if (!s || s.stage !== 'playing') return;
  const el = $('heroPlayText');
  const dur = s.dur || +s.duration_s || 0;
  const t = (performance.now() - s.playAt) / 1000;
  const left = Math.max(0, Math.ceil(dur - t));
  const txt = `Playing${s.title ? ' ' + s.title : ''} (${dur ? (left > 0 ? left + ' s' : 'finishing') : '…'})`;
  if (el && txt !== lastPlayText) el.textContent = lastPlayText = txt;
  $('heroBar').style.width = (dur ? clamp(t / dur) * 100 : 0).toFixed(1) + '%';
}

// ------------------------------------------------------------------ pending approval
function renderPending() {
  const p = S.pending;
  $('pendingCard').hidden = !p;
  $('waitCard').hidden = !p;
  if (!p) return;
  const dur = p.duration_s ? ` (${Math.round(p.duration_s)} s)` : '';
  $('pendingTitle').textContent = (p.title || 'a move') + dur;
  $('pendingWho').textContent = p.who ? `from ${p.who}` : '';
  $('waitTitle').textContent = `${p.who ? p.who + ': ' : ''}${p.title || 'a move'}${dur}`;
}
function approve() { if (S.pending) { send('approve', { id: S.pending.id }, `Approved: ${S.pending.title || S.pending.id}`); } }
function deny() { if (S.pending) { send('deny', { id: S.pending.id }, `Denied: ${S.pending.title || S.pending.id}`); } }

// ------------------------------------------------------------------ banners and health
function armStaleMs() { return S.armAt ? performance.now() - S.armAt : Infinity; }
function armDisconnected() {
  const a = S.arm;
  if (!a) return true;
  if (!Array.isArray(a.q)) return true;
  return ['disconnected', 'offline', 'no_arm', 'error'].includes(String(a.mode || '').toLowerCase());
}
let bannerKey = '';
function updateBanner() {
  let key = '', title = '', sub = '', bad = false;
  if (S.arm && S.arm.estop) { key = 'estop'; bad = true; title = 'E-STOP: the arm is stopped'; sub = 'Paused for safety. Operator: check the area, then press Reset E-STOP.'; }
  else if (!S.wsOpen) { key = 'ws'; title = S.wsEverOpen ? 'Lost the open-play server' : 'Connecting to the open-play server…'; sub = 'Reconnecting automatically.'; }
  else if (armStaleMs() > 1500 || armDisconnected()) { key = 'arm'; title = 'Arm disconnected'; sub = S.arm ? `No arm data for ${Math.round(armStaleMs() / 1000)} s (mode: ${S.arm.mode || '?'}). Check the armctl daemon and cables.` : 'Waiting for the first arm reading from the armctl daemon.'; }
  document.body.classList.toggle('estop', key === 'estop');
  document.body.classList.toggle('has-banner', !!key);
  document.body.classList.toggle('arm-off', key === 'arm' || key === 'ws');
  const b = $('banner');
  if (!key) { b.hidden = true; bannerKey = ''; return; }
  b.hidden = false; b.classList.toggle('bad', bad);
  if (key + title + sub !== bannerKey) { $('bannerTitle').textContent = title; $('bannerSub').textContent = sub; bannerKey = key + title + sub; }
}
function setHealth(id, cls, label) { const el = $(id); el.className = 'health ' + cls; el.querySelector('span').textContent = label; }
function updateHealth() {
  setHealth('hServer', S.wsOpen ? 'ok' : 'bad', S.wsOpen ? 'server' : 'server down');
  const stale = armStaleMs();
  const a = S.arm;
  if (!S.wsOpen || !a) setHealth('hArm', 'bad', 'arm: no data');
  else if (a.estop) setHealth('hArm', 'bad', 'E-STOP');
  else if (stale > 1500 || armDisconnected()) setHealth('hArm', 'bad', `arm: stale ${Math.round(stale / 1000)}s`);
  else if (S.armHz && S.armHz < 10) setHealth('hArm', 'warn', `arm ${S.armHz} Hz`);
  else setHealth('hArm', 'ok', `arm ${S.armHz || '…'} Hz`);
  const bits = [];
  if (a) {
    bits.push(`mode ${a.mode || '?'}`);
    if (a.busy) bits.push(`busy: ${a.busy}`);
    else if (a.moving) bits.push('moving');
    const lease = a.lease || (S.hello && (S.hello.lease || S.hello.daemon));
    if (lease) bits.push('lease ' + (typeof lease === 'string' ? lease : (lease.holder || lease.program || JSON.stringify(lease))));
  }
  if (S.hello && S.hello.join_url) bits.push(S.hello.join_url);
  $('railStatus').textContent = bits.join(' · ') || '–';
  $('railStatus').title = bits.join(' · ');
  $('resetBtn').disabled = !(a && a.estop);
  $('skipBtn').disabled = $('extendBtn').disabled = !S.lobby.active;
}

// ------------------------------------------------------------------ rail: queue, problems, joints
let railQueueKey = '';
function renderRailQueue(queue) {
  const a = S.lobby.active;
  $('railPlayers').textContent = `${S.lobby.players || 0} phone${S.lobby.players === 1 ? '' : 's'} connected`;
  updateHealth();
  // rebuild only when who is in the list changes (lobby arrives every second; an armed Kick must survive)
  const key = JSON.stringify([a && [a.pid, a.name], queue.map((p) => [p.pid, p.name])]);
  if (key === railQueueKey) return;
  railQueueKey = key;
  const rows = [];
  if (a) rows.push(`<li class="active"><span class="n">▶</span><span class="nm">${esc(a.name)}</span><span class="t" id="railRem"></span><button class="btn danger" data-kick="${esc(a.pid)}" type="button">Kick</button></li>`);
  queue.forEach((p, i) => rows.push(`<li><span class="n">${i + 1}</span><span class="nm">${esc(p.name)}</span><button class="btn danger" data-kick="${esc(p.pid)}" type="button">Kick</button></li>`));
  $('railQueue').innerHTML = rows.join('') || '<li class="muted">nobody playing or waiting</li>';
}
function addProblem(text, cls = 'warn') {
  S.problems.unshift({ at: Date.now(), text: String(text), cls });
  S.problems.length = Math.min(S.problems.length, 12);
  renderProblems();
}
function renderProblems() {
  const now = Date.now();
  $('railProblems').innerHTML = S.problems.length
    ? S.problems.slice(0, 4).map((p) => `<li class="${p.cls}" title="${esc(p.text)}"><span class="ago">${agoText(now - p.at)}</span><span>${esc(p.text)}</span></li>`).join('')
    : '<li class="muted">none yet</li>';
}
let jointEls = [];
function buildJoints() {
  const env = (S.hello && S.hello.envelope) || {};
  const ul = env.urdf_limits || {}, sl = env.soft_limits || {};
  const host = $('joints');
  host.innerHTML = '';
  jointEls = JOINTS.map((j, i) => {
    const lim = ul[j] || K.URDF_LIMITS[i];
    const soft = sl[j] || [lim[0] + 0.1, lim[1] - 0.1];
    const pct = (v) => ((v - lim[0]) / (lim[1] - lim[0])) * 100;
    host.insertAdjacentHTML('beforeend', `<span class="jn">${j}</span><span class="jv">–</span>
      <span class="jbar"><i class="soft" style="left:${pct(soft[0])}%;width:${pct(soft[1]) - pct(soft[0])}%"></i>${lim[0] < 0 && lim[1] > 0 ? `<i class="zero" style="left:${pct(0)}%"></i>` : ''}<i class="mk"></i></span>`);
    const kids = host.children;
    return { v: kids[kids.length - 2], mk: kids[kids.length - 1].querySelector('.mk'), lim, soft, pct };
  });
  host.insertAdjacentHTML('beforeend', `<span class="jn">grip</span><span class="jv">–</span><span class="jbar grip"><i class="mk"></i></span>`);
  const kids = host.children;
  jointEls.grip = { v: kids[kids.length - 2], mk: kids[kids.length - 1].querySelector('.mk') };
}
function tickJoints() {
  const a = S.arm;
  if (!jointEls.length) buildJoints();
  if (!a || !Array.isArray(a.q)) return;
  jointEls.forEach((e, i) => {
    const v = +a.q[i];
    e.v.textContent = (v >= 0 ? '+' : '') + v.toFixed(2);
    e.mk.style.left = clamp(e.pct(v), 0, 100).toFixed(1) + '%';
    e.mk.classList.toggle('out', v < e.soft[0] - 1e-3 || v > e.soft[1] + 1e-3);
  });
  const g = clamp(+a.grip || 0);
  jointEls.grip.v.textContent = g.toFixed(2);
  jointEls.grip.mk.style.left = (g * 100).toFixed(1) + '%';
  $('jointMode').textContent = armStaleMs() > 1500 ? 'last known' : a.busy ? `▶ ${a.busy}` : a.moving ? 'moving' : 'still';
}

// ------------------------------------------------------------------ operator controls
let toastTimer = null;
function toast(text) {
  const t = $('toast');
  t.textContent = text; t.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { t.hidden = true; }, 2200);
}
function estop() { send('estop', {}, 'E-STOP sent'); }
$('estopBtn').addEventListener('click', estop);
$('miniEstop').addEventListener('click', estop);
$('resetBtn').addEventListener('click', () => send('estop_reset', {}, 'E-STOP reset sent'));
$('approveBtn').addEventListener('click', approve);
$('denyBtn').addEventListener('click', deny);
$('skipBtn').addEventListener('click', () => send('skip', {}, 'Skipping this turn'));
$('extendBtn').addEventListener('click', () => send('extend', { s: 60 }, '+1 minute for this turn'));
$('autoPlay').addEventListener('change', (e) => { S.cfg.auto_play = e.target.checked; send('auto_play', { on: e.target.checked }, `Auto-play ${e.target.checked ? 'on' : 'off'}`); onConfig({}); });
$('pauseTurns').addEventListener('change', (e) => { S.cfg.paused = e.target.checked; send('pause', { on: e.target.checked }, e.target.checked ? 'New turns paused' : 'New turns resumed'); onConfig({}); });
$('turnLen').addEventListener('click', (e) => {
  const b = e.target.closest('button[data-s]');
  if (!b) return;
  send('turn_s', { s: +b.dataset.s }, `Turn length ${Math.round(+b.dataset.s / 60)} min`);
});
// Kick needs two clicks within 3 s (the first arms the button).
$('railQueue').addEventListener('click', (e) => {
  const b = e.target.closest('button[data-kick]');
  if (!b) return;
  if (!b.classList.contains('armed')) {
    b.classList.add('armed'); b.textContent = 'Sure?';
    setTimeout(() => { if (b.isConnected) { b.classList.remove('armed'); b.textContent = 'Kick'; } }, 3000);
    return;
  }
  const name = b.closest('li').querySelector('.nm').textContent;
  send('kick', { pid: b.dataset.kick }, `Kicked ${name}`);
});
// Buttons lose focus after a click so Enter/Space stay keyboard shortcuts.
document.addEventListener('click', (e) => { const b = e.target.closest('button, label'); if (b && b.blur) setTimeout(() => b.blur(), 0); });
function toggleRail() { document.body.classList.toggle('rail-off'); layoutDirty = true; }
window.addEventListener('keydown', (e) => {
  if (e.ctrlKey || e.metaKey || e.altKey) return;
  if (e.key === ' ' || e.code === 'Space') { e.preventDefault(); if (!e.repeat) estop(); return; }
  if (e.key === 'Enter') { e.preventDefault(); if (!e.repeat) approve(); return; }
  if (e.key === 'Escape') { e.preventDefault(); if (!e.repeat) deny(); return; }
  if (e.key === 'h' || e.key === 'H' || e.key === 'o' || e.key === 'O') { toggleRail(); }
}, true);

// ------------------------------------------------------------------ arm views
const target = { q: READY.slice(), grip: 0 };
const live = { q: READY.slice(), grip: 0 };
const cam = { az: 1.15, el: 0.36, dist: 1.5, target: [0.03, 0, 0.24], cx: 400, cy: 400, span: 0.8 };
let view = null; // the three.js view once ready; the canvas view otherwise

// ---- canvas fallback ----
const BG = [9, 12, 17];
const ARM = { base: [255, 138, 61], dark: [120, 52, 14], hi: [255, 214, 180] };
const GRAPHITE = { base: [62, 70, 82], dark: [22, 26, 32], hi: [150, 160, 175] };
const sub3 = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot3 = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross3 = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm3 = (a) => { const n = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / n, a[1] / n, a[2] / n]; };
const mix = (c, d, t) => c.map((v, i) => Math.round(v + (d[i] - v) * t));
const rgba = (c, a = 1) => `rgba(${c[0]},${c[1]},${c[2]},${a})`;

function makeProjector(c, W, H) {
  const ce = Math.cos(c.el), se = Math.sin(c.el);
  const C = [c.target[0] + c.dist * ce * Math.cos(c.az), c.target[1] + c.dist * ce * Math.sin(c.az), c.target[2] + c.dist * se];
  const f = norm3(sub3(c.target, C));
  const r = norm3(cross3(f, [0, 0, 1]));
  const u = cross3(r, f);
  const F = (H * c.dist) / c.span;
  return (P) => { const d = sub3(P, C); const z = Math.max(0.05, dot3(d, f)); return { x: c.cx + (F * dot3(d, r)) / z, y: c.cy - (F * dot3(d, u)) / z, z, s: F / z }; };
}

function armPrims(q, grip) {
  const f = K.frames(q);
  const P = (n) => f[n].p, g = f.gripper;
  const A = (v) => K.apply(g, v);
  const yo = 0.007 + 0.024 * clamp(grip);
  const segs = [
    { a: [0, 0, 0.0], b: [0, 0, 0.068], r: 0.05, mat: GRAPHITE },
    { a: P('link1'), b: P('link2'), r: 0.036, mat: GRAPHITE },
    { a: P('link2'), b: P('link3'), r: 0.031, mat: ARM },
    { a: P('link3'), b: P('link4'), r: 0.027, mat: ARM },
    { a: P('link4'), b: P('link5'), r: 0.024, mat: ARM },
    { a: P('link5'), b: P('gripper'), r: 0.023, mat: GRAPHITE },
    { a: P('gripper'), b: A([0, 0, -0.046]), r: 0.026, mat: GRAPHITE },
    { a: A([0, -yo - 0.006, -0.05]), b: A([0, yo + 0.006, -0.05]), r: 0.01, mat: GRAPHITE },
    { a: A([0, yo, -0.05]), b: A([0, yo, -0.108]), r: 0.008, mat: ARM, finger: true },
    { a: A([0, -yo, -0.05]), b: A([0, -yo, -0.108]), r: 0.008, mat: ARM, finger: true },
  ];
  const hubs = [
    { p: P('link2'), r: 0.04 }, { p: P('link3'), r: 0.036 }, { p: P('link4'), r: 0.032 }, { p: P('link5'), r: 0.028 },
  ];
  return { segs, hubs, tip: P('gripper_tip') };
}

function createCanvasView(canvas) {
  const ctx = canvas.getContext('2d');
  const trail = [];
  let W = 0, H = 0, dpr = 1;
  function size() {
    dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (w !== W || h !== H || canvas.width !== Math.round(w * dpr)) {
      W = w; H = h; canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
    }
  }
  function tube(pa, pb, r, mat, fog, alpha) {
    const w = Math.max(1.5, r * 2 * (pa.s + pb.s) / 2);
    const dx = pb.x - pa.x, dy = pb.y - pa.y, L = Math.hypot(dx, dy) || 1;
    let nx = -dy / L, ny = dx / L;
    if (ny > 0) { nx = -nx; ny = -ny; } // highlight on the upper side
    const base = mix(mat.base, BG, fog), dark = mix(mat.dark, BG, fog), hi = mix(mat.hi, BG, fog);
    ctx.lineCap = 'round';
    ctx.globalAlpha = alpha;
    ctx.strokeStyle = rgba(dark); ctx.lineWidth = w;
    ctx.beginPath(); ctx.moveTo(pa.x, pa.y); ctx.lineTo(pb.x, pb.y); ctx.stroke();
    ctx.strokeStyle = rgba(base); ctx.lineWidth = w * 0.74;
    ctx.beginPath(); ctx.moveTo(pa.x + nx * w * 0.08, pa.y + ny * w * 0.08); ctx.lineTo(pb.x + nx * w * 0.08, pb.y + ny * w * 0.08); ctx.stroke();
    ctx.strokeStyle = rgba(hi, 0.85); ctx.lineWidth = w * 0.2;
    ctx.beginPath(); ctx.moveTo(pa.x + nx * w * 0.22, pa.y + ny * w * 0.22); ctx.lineTo(pb.x + nx * w * 0.22, pb.y + ny * w * 0.22); ctx.stroke();
    ctx.globalAlpha = 1;
  }
  function hub(p, r, fog, alpha, ring) {
    const R = Math.max(2, r * p.s);
    const g = ctx.createRadialGradient(p.x - R * 0.35, p.y - R * 0.4, R * 0.1, p.x, p.y, R);
    g.addColorStop(0, rgba(mix([120, 130, 145], BG, fog)));
    g.addColorStop(1, rgba(mix([20, 24, 30], BG, fog)));
    ctx.globalAlpha = alpha;
    ctx.fillStyle = g;
    ctx.beginPath(); ctx.arc(p.x, p.y, R, 0, Math.PI * 2); ctx.fill();
    ctx.strokeStyle = rgba(mix(ring, BG, fog), 0.9); ctx.lineWidth = Math.max(1, R * 0.16);
    ctx.beginPath(); ctx.arc(p.x, p.y, R * 0.62, 0, Math.PI * 2); ctx.stroke();
    ctx.globalAlpha = 1;
  }
  function floor(proj) {
    // soft spotlight pool under the arm
    const o = proj([0, 0, 0]);
    const ring = [];
    for (let i = 0; i < 48; i++) { const a = (i / 48) * Math.PI * 2; ring.push(proj([0.55 * Math.cos(a), 0.55 * Math.sin(a), 0])); }
    const rx = Math.max(...ring.map((p) => Math.abs(p.x - o.x)));
    const ry = Math.max(...ring.map((p) => Math.abs(p.y - o.y)));
    ctx.save();
    ctx.translate(o.x, o.y); ctx.scale(1, ry / rx);
    const g = ctx.createRadialGradient(0, 0, 0, 0, 0, rx);
    g.addColorStop(0, 'rgba(92,200,255,0.10)'); g.addColorStop(0.55, 'rgba(92,200,255,0.035)'); g.addColorStop(1, 'rgba(92,200,255,0)');
    ctx.fillStyle = g; ctx.beginPath(); ctx.arc(0, 0, rx, 0, Math.PI * 2); ctx.fill();
    ctx.restore();
    // grid, fading with distance from the base
    ctx.lineWidth = 1;
    const R = 0.6, step = 0.1, segN = 24;
    for (let k = -6; k <= 6; k++) {
      for (const axis of [0, 1]) {
        let prev = null;
        for (let i = 0; i <= segN; i++) {
          const s = -R + (2 * R * i) / segN;
          const P = axis ? [k * step, s, 0] : [s, k * step, 0];
          const p = proj(P);
          if (prev) {
            const rr = Math.hypot((P[0] + prev.P[0]) / 2, (P[1] + prev.P[1]) / 2);
            const a = 0.32 * Math.pow(clamp(1 - rr / R), 1.6) * (k === 0 ? 1.5 : 1);
            if (a > 0.004) { ctx.strokeStyle = `rgba(120,160,200,${a.toFixed(3)})`; ctx.beginPath(); ctx.moveTo(prev.p.x, prev.p.y); ctx.lineTo(p.x, p.y); ctx.stroke(); }
          }
          prev = { p, P };
        }
      }
    }
    // +x (forward) tick
    const a = proj([0.12, 0, 0]), b = proj([0.3, 0, 0]);
    ctx.strokeStyle = 'rgba(255,138,61,0.35)'; ctx.lineWidth = 2; ctx.setLineDash([6, 6]);
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke(); ctx.setLineDash([]);
  }
  function shadow(proj, prims, alpha) {
    ctx.save();
    ctx.lineCap = 'round';
    ctx.filter = 'blur(6px)';
    ctx.strokeStyle = `rgba(0,0,0,${0.5 * alpha})`;
    for (const s of prims.segs) {
      const pa = proj([s.a[0], s.a[1], 0]), pb = proj([s.b[0], s.b[1], 0]);
      ctx.lineWidth = Math.max(2, s.r * 2.2 * pa.s);
      ctx.beginPath(); ctx.moveTo(pa.x, pa.y); ctx.lineTo(pb.x, pb.y); ctx.stroke();
    }
    ctx.restore();
  }
  function drawArm(proj, prims, alpha) {
    const items = [];
    for (const s of prims.segs) { const pa = proj(s.a), pb = proj(s.b); items.push({ z: (pa.z + pb.z) / 2 + (s.finger ? -0.01 : 0), draw: (fog) => tube(pa, pb, s.r, s.mat, fog, alpha) }); }
    for (const h of prims.hubs) { const p = proj(h.p); items.push({ z: p.z - 0.03, draw: (fog) => hub(p, h.r, fog, alpha, ARM.base) }); }
    const zs = items.map((i) => i.z), z0 = Math.min(...zs), z1 = Math.max(...zs);
    items.sort((a, b) => b.z - a.z);
    for (const it of items) it.draw(0.35 * ((it.z - z0) / Math.max(1e-6, z1 - z0)));
  }
  function drawGhost(proj, prims, alpha) {
    ctx.save();
    ctx.globalCompositeOperation = 'lighter';
    ctx.lineCap = 'round';
    for (const s of prims.segs) {
      const pa = proj(s.a), pb = proj(s.b);
      const w = Math.max(1.5, s.r * 2 * (pa.s + pb.s) / 2);
      ctx.strokeStyle = `rgba(92,200,255,${(0.16 * alpha).toFixed(3)})`; ctx.lineWidth = w * 1.25;
      ctx.beginPath(); ctx.moveTo(pa.x, pa.y); ctx.lineTo(pb.x, pb.y); ctx.stroke();
      ctx.strokeStyle = `rgba(150,225,255,${(0.32 * alpha).toFixed(3)})`; ctx.lineWidth = w * 0.55;
      ctx.beginPath(); ctx.moveTo(pa.x, pa.y); ctx.lineTo(pb.x, pb.y); ctx.stroke();
    }
    for (const h of prims.hubs) {
      const p = proj(h.p), R = h.r * p.s;
      ctx.strokeStyle = `rgba(150,225,255,${(0.5 * alpha).toFixed(3)})`; ctx.lineWidth = Math.max(1, R * 0.18);
      ctx.beginPath(); ctx.arc(p.x, p.y, R * 0.7, 0, Math.PI * 2); ctx.stroke();
    }
    const t = proj(prims.tip);
    const g = ctx.createRadialGradient(t.x, t.y, 0, t.x, t.y, 0.05 * t.s);
    g.addColorStop(0, `rgba(160,230,255,${(0.6 * alpha).toFixed(3)})`); g.addColorStop(1, 'rgba(92,200,255,0)');
    ctx.fillStyle = g; ctx.beginPath(); ctx.arc(t.x, t.y, 0.05 * t.s, 0, Math.PI * 2); ctx.fill();
    ctx.restore();
  }
  function drawTrail(proj) {
    if (trail.length < 2) return;
    ctx.save();
    ctx.globalCompositeOperation = 'lighter';
    ctx.lineCap = 'round'; ctx.lineJoin = 'round';
    for (let i = 1; i < trail.length; i++) {
      const a = proj(trail[i - 1]), b = proj(trail[i]);
      const k = i / trail.length;
      ctx.strokeStyle = `rgba(255,150,80,${(0.4 * k * k).toFixed(3)})`;
      ctx.lineWidth = 1 + 4 * k;
      ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
    }
    ctx.restore();
  }
  return {
    kind: 'canvas',
    render(c, q, grip, ghost) {
      size();
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, W, H);
      const proj = makeProjector(c, W, H);
      const prims = armPrims(q, grip);
      // tip trail (last ~1.5 s); skip still points so it fades away when the arm rests
      const tp = prims.tip, lastT = trail[trail.length - 1];
      if (!lastT || Math.hypot(tp[0] - lastT[0], tp[1] - lastT[1], tp[2] - lastT[2]) > 0.002) trail.push(tp);
      else if (trail.length) trail.shift();
      while (trail.length > 70) trail.shift();
      floor(proj);
      shadow(proj, prims, 1);
      let gp = null;
      if (ghost && ghost.alpha > 0.01) { gp = armPrims(ghost.q, ghost.grip); shadow(proj, gp, 0.25 * ghost.alpha); }
      // the ghost goes behind the live arm: it glows where the coming move differs from now
      if (gp) drawGhost(proj, gp, ghost.alpha);
      drawTrail(proj);
      drawArm(proj, prims, 1);
    },
  };
}

// ---- three.js + URDF (only when hello.urdf; falls back to the canvas on any failure) ----
let threeTried = false;
async function tryThree() {
  if (threeTried) return;
  threeTried = true;
  try {
    const timeout = (ms) => new Promise((_, rej) => setTimeout(() => rej(new Error('timeout')), ms));
    const [THREE, { default: URDFLoader }] = await Promise.race([Promise.all([import('three'), import('urdf-loader')]), timeout(10000)]);
    const v = await Promise.race([createThreeView(THREE, URDFLoader, $('threeHost')), timeout(20000)]);
    view = v;
    $('armCanvas').style.display = 'none';
  } catch (e) {
    console.warn('URDF 3D view unavailable, using the built-in view:', e && e.message);
    $('threeHost').innerHTML = '';
  }
}
async function createThreeView(THREE, URDFLoader, host) {
  const FINGER_MIN = -0.04695;
  const load = (matFn) => new Promise((resolve, reject) => {
    const loader = new URDFLoader();
    loader.packages = { yam_description: '/urdf' };
    const base = loader.defaultMeshLoader.bind(loader);
    loader.loadMeshCb = (path, manager, done) => base(path, manager, (obj, err) => { done(obj, err); if (obj) obj.traverse((o) => { if (o.isMesh) matFn(o); }); });
    // meshes get the material as they load; primitive <geometry> (box/cylinder/sphere) only here
    loader.load('/urdf/i2rt/yam.urdf', (r) => { r.traverse((o) => { if (o.isMesh) matFn(o); }); resolve(r); }, undefined, reject);
  });
  const solid = (m) => {
    if (m.userData.styled) return;
    m.userData.styled = true;
    const c = m.material && m.material.color ? m.material.color.clone() : new THREE.Color(0xaaaaaa);
    m.material = new THREE.MeshStandardMaterial({ color: c.lerp(new THREE.Color(0xff8a3d), 0.55), metalness: 0.2, roughness: 0.5 });
  };
  const ghostMat = new THREE.MeshBasicMaterial({ color: 0x5cc8ff, transparent: true, opacity: 0.2, depthWrite: false });
  const ghostFn = (m) => { m.material = ghostMat; m.renderOrder = 2; };
  const [robot, ghost] = await Promise.all([load(solid), load(ghostFn)]);
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.setClearColor(0x000000, 0);
  host.appendChild(renderer.domElement);
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(30, 1, 0.01, 20);
  scene.add(new THREE.HemisphereLight(0xdde8ff, 0x202530, 1.5));
  const sun = new THREE.DirectionalLight(0xffffff, 1.8); sun.position.set(1, 2.2, 1.2); scene.add(sun);
  const rim = new THREE.DirectionalLight(0x5cc8ff, 0.8); rim.position.set(-1.5, 1, -1); scene.add(rim);
  const grid = new THREE.GridHelper(1.2, 12, 0x34506a, 0x1c2836);
  grid.material.transparent = true; grid.material.opacity = 0.5;
  scene.add(grid);
  const world = new THREE.Group(); world.rotation.x = -Math.PI / 2; scene.add(world);
  world.add(robot); world.add(ghost);
  const setPose = (r, q, grip) => {
    q.forEach((v, i) => { const j = r.joints['joint' + (i + 1)]; if (j) { j.ignoreLimits = true; j.setJointValue(v); } });
    const g = FINGER_MIN * clamp(grip);
    if (r.joints.joint7) r.setJointValue('joint7', g);
    if (r.joints.joint8) r.setJointValue('joint8', g);
  };
  return {
    kind: 'three',
    render(c, q, grip, gh) {
      const W = host.clientWidth, H = host.clientHeight;
      const sz = renderer.getSize(new THREE.Vector2());
      if (sz.x !== W || sz.y !== H) renderer.setSize(W, H, false);
      setPose(robot, q, grip);
      ghost.visible = !!(gh && gh.alpha > 0.01);
      if (ghost.visible) { setPose(ghost, gh.q, gh.grip); ghostMat.opacity = 0.22 * gh.alpha; }
      const ce = Math.cos(c.el), se = Math.sin(c.el);
      const p = [c.target[0] + c.dist * ce * Math.cos(c.az), c.target[1] + c.dist * ce * Math.sin(c.az), c.target[2] + c.dist * se];
      camera.position.set(p[0], p[2], -p[1]); // URDF z-up -> three y-up
      camera.up.set(0, 1, 0);
      camera.lookAt(c.target[0], c.target[2], -c.target[1]);
      camera.fov = (2 * Math.atan(c.span / 2 / c.dist) * 180) / Math.PI;
      camera.aspect = W / Math.max(1, H);
      camera.setViewOffset(W, H, W / 2 - c.cx, H / 2 - c.cy, W, H);
      camera.updateProjectionMatrix();
      renderer.render(scene, camera);
    },
  };
}

// ------------------------------------------------------------------ ghost source
function ghostPose(now) {
  const s = S.say;
  if (s && s.stage === 'playing' && s.kfs && s.kfs.length) {
    const t = (now - s.playAt) / 1000 + 0.5; // a little ahead of the arm: where it is going
    if (t <= s.dur + 1.0) { const p = K.sampleKeyframes(s.kfs, t); return { ...p, want: 1 }; }
  }
  const p = S.pending;
  if (p && p.kfs && p.kfs.length) {
    const loop = p.dur + 1.5;
    const t = ((now - p.at) / 1000) % loop;
    return { ...K.sampleKeyframes(p.kfs, Math.min(t, p.dur)), want: 1 };
  }
  return null;
}

// ------------------------------------------------------------------ layout -> camera focus
let layoutDirty = true, focus = { cx: 400, cy: 400, span: 0.8 };
function measure() {
  const show = $('show').getBoundingClientRect();
  const side = $('side').getBoundingClientRect();
  const attract = document.body.classList.contains('attract');
  const right = Math.max(200, side.left - show.left - 16);
  const H = show.height;
  if (attract) {
    // the invitation sits top-left; put the arm lower and to the right of the free area
    focus = { cx: right * 0.64, cy: H * 0.42, span: Math.max(1.35, 1.0 * H / right) };
  } else {
    focus = { cx: right * 0.52, cy: H * 0.6, span: Math.max(1.0, 0.85 * H / right) };
  }
  layoutDirty = false;
}
new ResizeObserver(() => { layoutDirty = true; }).observe($('show'));
new MutationObserver(() => { layoutDirty = true; }).observe(document.body, { attributes: true, attributeFilter: ['class'] });

// ------------------------------------------------------------------ main loop
const canvasView = createCanvasView($('armCanvas'));
let lastFrame = performance.now(), ghostA = 0, lastGhost = null, slowAt = 0, t0 = performance.now();
let measureAt = 0;
const wrapPi = (a) => Math.atan2(Math.sin(a), Math.cos(a));
function frame(now) {
  requestAnimationFrame(frame);
  const dt = Math.min(0.1, (now - lastFrame) / 1000); lastFrame = now;
  if (layoutDirty || now - measureAt > 1000) { measure(); measureAt = now; }
  // live arm: ease toward the latest 20 Hz sample
  const k = 1 - Math.exp(-dt / 0.06);
  for (let i = 0; i < 6; i++) live.q[i] += (target.q[i] - live.q[i]) * k;
  live.grip += (target.grip - live.grip) * k;
  // camera: slow full orbit in attract mode, a gentle sway around a 3/4 view while someone plays
  const attract = document.body.classList.contains('attract');
  const T = (now - t0) / 1000;
  if (attract) cam.az += dt * (Math.PI * 2 / 70);
  else cam.az += wrapPi(1.15 + 0.38 * Math.sin(T * Math.PI * 2 / 36) - cam.az) * (1 - Math.exp(-dt / 1.8));
  const kc = 1 - Math.exp(-dt / 0.7);
  cam.cx += (focus.cx - cam.cx) * kc; cam.cy += (focus.cy - cam.cy) * kc; cam.span += (focus.span - cam.span) * kc;
  // ghost fades in/out
  const g = ghostPose(now);
  if (g) lastGhost = g;
  ghostA += ((g ? 1 : 0) - ghostA) * (1 - Math.exp(-dt / 0.35));
  const ghost = lastGhost && ghostA > 0.01 ? { q: lastGhost.q, grip: lastGhost.grip, alpha: ghostA } : null;
  (view || canvasView).render(cam, live.q, live.grip, ghost);
  tickNow();
  tickHero();
  if (now - slowAt > 250) {
    slowAt = now;
    tickJoints(); updateBanner(); updateHealth();
  }
}
setInterval(() => { S.armHz = S.armCount; S.armCount = 0; restyleCaptions(); renderProblems(); }, 1000);
requestAnimationFrame(frame);
cam.cx = focus.cx; cam.cy = focus.cy;
buildJoints();
connect();
