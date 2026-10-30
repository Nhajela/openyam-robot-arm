// Open play, arcade phone page: the arm as a claw machine. Speaks the same phone side of
// openplay/PROTOCOL.md as play.js (hello/join/stream/grip/ready/move/say/stop), so the server,
// the daemon's envelope, the 300 ms deadman and E-STOP are unchanged. Only the presentation
// differs: instead of levers (speed, sliders, two sticks) the player gets one verb per thumb.
//
//   steer  drag anywhere on the map (a top-down view of the table, the claw drawn where forward
//          kinematics puts the fingertip). The drag is a floating stick: velocity in the base
//          frame's x/y, sent at 15 Hz while the finger is down, an explicit zero on lift.
//   drop   hold DROP: the claw lowers (v.z < 0) and stops at DROP_FLOOR_Z by itself. Let go: the
//          claw shuts (`grip` 0) and comes back up to the ready pose (`ready`), like a real claw
//          machine. Lifting the finger is still what stops the descent.
//   emote  tap a library move (`move`). ✨ Ask it is `say`.
//
// Two camera games when the server relays the wrist camera (welcome.features.cam, GET /cam.jpg):
//   👀 Robot Eyes  the phone shows what the robot sees; drag to look around (joint velocities:
//                  the base turns, the wrist tilts), pick a filter, SNAP a framed photo to keep.
//   🤳 Selfie      the arm takes your photo: 🎯 Find me turns the camera to where people stand
//                  (selfie.aim: the server aims from armctl/audience.json, through the daemon and
//                  the operator's approval), then 5-4-3-2-1 and selfie.snap: the server saves the
//                  camera's original on the laptop, the phone frames it, offers "Save to my phone"
//                  and uploads the framed copy to the laptop too.
//   🎥 Claw Cam    the claw machine from the claw's own eye: the same steer and DROP, the camera
//                  picture instead of the map.
//
// The game layer is phone-only and never moves anything: stars to fly the claw over (scored from
// the arm's reported joints), a combo for quick chains, a score and a best score on this phone.
// The walls (R_MIN/R_MAX) and the drop floor are phone-side conveniences computed from the arm's
// reported q, not safety limits: the daemon's envelope and the operator's E-STOP are the safety.

import { tip } from './kin.js';

const RATE_MS = 66;                   // ~15 Hz while steering or dropping (as play.js)
const THUMB_R = 70;                   // px: a drag this far is full speed
const THUMB_DEAD = 0.12;
const DROP_FLOOR_Z = 0.03;            // m, base frame: DROP stops here (6 cm above the table)
const R_MIN = 0.14, R_MAX = 0.42;     // m: soft walls on the map (tip distance from the base axis)
const J1_EDGE = 0.12;                 // rad: side walls this far inside the daemon's j1 soft limits (state lags ~0.1 s)
const J1_DEFAULT = [-1.0, 1.0];       // rad: if the server sends no limits (older server)
const MAP_R = 0.47;                   // m: the floor shows this far from the base
const TABLE_Z = -0.031;               // m, base frame: the table (29 Oct calibration)
const STAR_Z = 0.43;                  // m: stars float at the ready pose's height
const KD = 0.85;                      // floor foreshortening of the angled view
const STAR_R = 0.045;                 // m: fly the tip this close (x/y) to collect a star
const DIVE_Z = 0.17;                  // m: gold stars count only below this height
const COMBO_S = 5;                    // s: collect again within this for the next multiplier
const GRAB_MIN_DROP = 0.04;           // m: a DROP that went at least this far down scores a grab
const LOOK_K = 0.6;                   // Robot Eyes: full drag = 0.6 x the server's joint speed (0.3 rad/s)
const FILTERS = [['none', '🙂 Normal'], ['robot', '🤖 Robot'], ['thermal', '🔥 Thermal'], ['pixel', '👾 8-bit'], ['comic', '💥 Comic']];
const CHIPS = ['wave hello', 'do a happy dance', 'bow to the audience', 'look around curiously', 'pretend to be a sleepy cat'];
const END_REASONS = {
  time: "Time's up!", left: 'You ended your turn.', idle: 'Nothing happened for a minute.',
  skipped: 'The staff moved on to the next player.', operator: 'The staff ended your turn.',
};

const $ = (id) => document.getElementById(id);
const el = (tag, cls, text) => { const e = document.createElement(tag); if (cls) e.className = cls; if (text != null) e.textContent = text; return e; };
const store = {
  get(k, d) { try { const v = localStorage.getItem(k); return v === null ? d : v; } catch (_) { return d; } },
  set(k, v) { try { localStorage.setItem(k, v); } catch (_) { /* private mode */ } },
};
const clamp = (v, a = -1, b = 1) => Math.max(a, Math.min(b, v));
const r3 = (v) => Math.round(v * 1000) / 1000;
const buzz = (p) => { try { navigator.vibrate && navigator.vibrate(p); } catch (_) {} };
const fmtDur = (s) => { s = Math.max(0, Math.round(s)); return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`; };

// ------------------------------------------------------------------ identity (shared with play.js)
let pid = store.get('openplay-pid', '');
if (!pid) {
  pid = (window.crypto && crypto.randomUUID) ? crypto.randomUUID() : 'p-' + Math.random().toString(36).slice(2) + Date.now().toString(36);
  store.set('openplay-pid', pid);
}
let myName = store.get('openplay-name', '');

// ------------------------------------------------------------------ state
const S = {
  welcome: null, lobby: null, arm: null, screen: null,
  turnRemaining: 0, turnAt: 0, lastTurnMsg: 0, warning: false,
  flip: store.get('arcade-flip', '0') === '1', // the player stands across the table from the base
  wantJoin: false, started: false,
  game: 'claw',                 // claw | selfie | eyes (Robot Eyes) | cam (Claw Cam)
  filterBy: { eyes: 'robot', selfie: 'none' },
};
const G = { score: 0, combo: 0, lastAt: 0, stars: [], pops: [], stats: null, emotesDone: new Set(), trail: [] };
const newStats = () => ({ stars: 0, dives: 0, grabs: 0, emotes: 0, bestCombo: 0, photos: 0 });
G.stats = newStats();
const curFilter = () => S.filterBy[S.game] || 'none';
const looking = () => S.game === 'eyes' || S.game === 'selfie'; // drag = look around (joint velocities)
const features = () => Object.assign({ say: true, joystick: true, cam: false }, (S.welcome && S.welcome.features) || {});
// the base's allowed turn (the tip's angle on the map equals j1), from the daemon's soft limits
function j1Range() {
  const l = S.welcome && S.welcome.limits && S.welcome.limits.j1;
  const [lo, hi] = Array.isArray(l) && l.length === 2 ? l : J1_DEFAULT;
  return [Math.max(lo, -1.4) + J1_EDGE, Math.min(hi, 1.4) - J1_EDGE];
}

// ------------------------------------------------------------------ connection (as play.js)
let ws = null, backoff = 500, lastRx = 0, reconnectTimer = null, helloName = null, txSeq = 0;
function wsUrl() {
  const k = new URLSearchParams(location.search).get('k');
  return `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws` + (k ? `?k=${encodeURIComponent(k)}` : '');
}
function send(m) {
  m.seq = ++txSeq; m.t = Date.now();
  if (ws && ws.readyState === 1) {
    if (m.op === 'stream' && ws.bufferedAmount) m.buf = ws.bufferedAmount;
    ws.send(JSON.stringify(m)); return true;
  }
  return false;
}
function hello() {
  helloName = myName;
  send({ op: 'hello', pid, name: myName, rate_ms: RATE_MS, ui: 'arcade', ua: (navigator.userAgent || '').slice(0, 120) });
}
let syncRtt = null, syncOffset = null;
function onPong(m) {
  if (m.t0 == null || m.t1 == null || m.t2 == null) return;
  const t3 = Date.now(), rtt = (t3 - m.t0) - (m.t2 - m.t1);
  if (syncRtt === null || rtt <= syncRtt) { syncRtt = rtt; syncOffset = ((m.t0 - m.t1) + (t3 - m.t2)) / 2; }
  send({ op: 'sync', s: [m.t0, m.t1, m.t2, t3], offset_ms: syncOffset });
}
const diagBuf = [];
function diag(ev, extra) {
  diagBuf.push(Object.assign({ t: Date.now(), ev, hidden: document.hidden }, extra || {}));
  if (diagBuf.length > 60) diagBuf.shift();
}
function flushDiag() {
  if (!diagBuf.length || !ws || ws.readyState !== 1) return;
  if (send({ op: 'diag', events: diagBuf.slice() })) diagBuf.length = 0;
}
function connect() {
  clearTimeout(reconnectTimer);
  setConn('Connecting…');
  try { ws = new WebSocket(wsUrl()); } catch (e) { diag('ws_error', { where: 'new' }); scheduleReconnect(); return; }
  const me = ws, opened = Date.now();
  me.onopen = () => {
    if (me !== ws) return;
    backoff = 500; lastRx = Date.now(); setConn(null); hello();
    diag('ws_open', { connect_ms: Date.now() - opened }); flushDiag();
    syncRtt = null;
    for (let i = 0; i < 5; i++) setTimeout(() => { if (me === ws) send({ op: 'ping' }); }, 150 * i);
    if (S.wantJoin) { send({ op: 'join' }); S.wantJoin = false; }
  };
  me.onmessage = (ev) => {
    if (me !== ws) return;
    lastRx = Date.now();
    let m; try { m = JSON.parse(ev.data); } catch (_) { return; }
    handle(m);
  };
  me.onclose = (ev) => {
    if (me !== ws) return;
    diag('ws_close', { code: ev.code, reason: (ev.reason || '').slice(0, 80), clean: ev.wasClean,
      open_s: Math.round((Date.now() - opened) / 100) / 10, streaming: !!streamTimer });
    releaseAll('close'); ws = null; S.arm = null; renderArm(); scheduleReconnect();
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
setInterval(() => {
  if (!ws || ws.readyState !== 1) return;
  send({ op: 'ping' }); flushDiag();
  if (Date.now() - lastRx > 12000) { diag('rx_silent', { silent_ms: Date.now() - lastRx }); try { ws.close(); } catch (_) {} }
}, 5000);
document.addEventListener('visibilitychange', () => {
  diag(document.hidden ? 'hidden' : 'visible', { streaming: !!streamTimer });
  if (document.hidden) releaseAll('hidden');
  else if (!ws) { backoff = 500; connect(); }
  else { flushDiag(); if (S.screen === 'turn') wakeLock(true); }
});
window.addEventListener('pagehide', () => { diag('pagehide', { streaming: !!streamTimer }); releaseAll('pagehide'); });

// ------------------------------------------------------------------ messages
function handle(m) {
  switch (m.op) {
    case 'welcome':
      S.welcome = m;
      if (m.name && !myName) { myName = m.name; $('name').value = m.name; }
      if (m.turn_s) $('turnLen').textContent = m.turn_s % 60 ? `${fmtDur(m.turn_s)} min` : `${m.turn_s / 60} min`;
      renderEmotes(); $('askBtn').disabled = !features().say; renderGames(); renderLobby(); break;
    case 'lobby':
      S.lobby = m; renderLobby();
      if (m.you_active && S.screen !== 'turn' && m.active) startTurn(m.active.remaining_s);
      if (S.screen === 'turn' && !m.you_active && Date.now() - S.lastTurnMsg > 3000) show('lobby');
      break;
    case 'turn':
      S.turnRemaining = m.remaining_s; S.turnAt = performance.now(); S.lastTurnMsg = Date.now(); S.warning = !!m.warning;
      if (S.screen !== 'turn') startTurn(m.remaining_s);
      renderClock(); break;
    case 'turn_end': gameOver(m.reason); break;
    case 'arm': S.arm = m; onArm(); break;
    case 'status':
      if (m.kind === 'error') toast(m.text, 'error');
      else if (/^The arm can't go further/.test(m.text || '')) { ticker(looking() ? "🧱 Can't turn further that way" : '🧱 Wall! Steer the other way'); buzz(30); }
      else ticker(m.text);
      break;
    case 'refused': {
      const r = m.reason || 'Not now';
      if (/needs the arm unfolded/.test(r)) {
        if ($('lowBar').hidden) { releaseAll('refused'); buzz([30, 40, 30]); }
        $('lowBar').hidden = false; break;
      }
      toast(/^busy/i.test(r) && S.screen === 'turn' ? `${r}. Tap STOP first.` : r, 'error'); break;
    }
    case 'say': onSay(m); break;
    case 'selfie': onSelfie(m); break;
    case 'pong': onPong(m); break;
  }
}

// ------------------------------------------------------------------ screens
function show(name) {
  if (S.screen === name) return;
  S.screen = name;
  for (const id of ['lobby', 'turn', 'over']) $(id).hidden = id !== name;
  window.scrollTo(0, 0); clearToast();
  if (name !== 'turn') { releaseAll(); camStop(); for (const id of ['intro', 'pause', 'askSheet', 'photo']) $(id).hidden = true; }
  wakeLock(name === 'turn');
  if (name === 'turn') requestAnimationFrame(resizeMap);
}
function startTurn(remaining) {
  if (remaining != null) { S.turnRemaining = remaining; S.turnAt = performance.now(); }
  S.lastTurnMsg = Date.now();
  G.score = 0; G.combo = 0; G.lastAt = 0; G.stats = newStats(); G.emotesDone.clear(); G.trail = []; G.pops = [];
  G.stars = []; S.started = false;
  Object.assign(selfie, { aimed: false, aiming: false, after: false, countdown: false });
  renderScore(); renderEmotes(); setGame('claw', true);
  $('introMore').hidden = !features().cam;
  $('introName').textContent = myName || 'Player one';
  show('turn');
  $('intro').hidden = false; $('countdown').hidden = true;
  renderClock(); renderArm();
  buzz([120, 60, 120]);
}
function gameOver(reason) {
  releaseAll();
  const best = parseInt(store.get('arcade-best', '0'), 10) || 0;
  const isBest = G.score > best;
  if (isBest) store.set('arcade-best', String(G.score));
  $('overWhy').textContent = END_REASONS[reason] || 'Your turn is over.';
  $('overScore').textContent = String(G.score);
  const ob = $('overBest');
  ob.textContent = isBest && G.score > 0 ? '🏆 NEW HIGH SCORE!' : best ? `Best on this phone: ${best}` : '';
  ob.classList.toggle('new', isBest && G.score > 0);
  const st = $('overStats'); st.replaceChildren();
  for (const [n, label] of [[G.stats.stars, 'stars'], [G.stats.dives, 'gold dives'], [G.stats.grabs, 'claw grabs'], [`x${Math.max(1, G.stats.bestCombo)}`, 'best combo']].concat(G.stats.photos ? [[G.stats.photos, 'robot photos']] : [])) {
    const li = el('li'); li.append(el('b', '', String(n)), label); st.append(li);
  }
  show('over');
  renderBest();
}

// ------------------------------------------------------------------ lobby
function renderBest() {
  const best = parseInt(store.get('arcade-best', '0'), 10) || 0;
  $('bestLine').textContent = best ? `your best: ★ ${best}` : 'no high score yet';
}
function renderLobby() {
  const L = S.lobby; if (!L) return;
  const a = L.active;
  $('nowName').textContent = a ? (L.you_active ? `${a.name} (you!)` : a.name) : 'Nobody yet. It could be you!';
  $('nowTime').textContent = a ? `${fmtDur(a.remaining_s)} left` : '';
  const queued = L.position != null;
  $('joinCard').hidden = queued || L.you_active;
  $('queuedCard').hidden = !queued;
  if (queued) {
    $('pos').textContent = L.position;
    const next = L.position === 1;
    $('queuedCard').classList.toggle('next', next);
    $('posTitle').textContent = next ? (a ? "You're next! Get ready" : 'Starting your game…') : "You're in line";
    $('eta').textContent = L.eta_s == null ? '' : L.eta_s < 60 ? 'Less than a minute' : `About ${Math.round(L.eta_s / 60)} min`;
  }
}
function doJoin() {
  const n = $('name').value.trim().slice(0, 24);
  if (!n) { $('name').focus(); toast('Type your player name first', 'info'); return; }
  myName = n; store.set('openplay-name', n);
  if (!ws || ws.readyState !== 1) { S.wantJoin = true; toast('Joining as soon as we reconnect…'); return; }
  if (helloName !== myName) hello();
  send({ op: 'join' });
}
$('name').value = myName;
$('joinBtn').addEventListener('click', doJoin);
$('name').addEventListener('keydown', (e) => { if (e.key === 'Enter') { e.preventDefault(); $('name').blur(); doJoin(); } });
$('leaveBtn').addEventListener('click', () => send({ op: 'leave' }));
$('againBtn').addEventListener('click', () => { show('lobby'); doJoin(); });
$('lobbyBtn').addEventListener('click', () => show('lobby'));
// keep the room key on the classic links (the cookie usually covers it; the key is belt and braces)
for (const id of ['classicLink', 'classicTurnLink']) $(id).href = '/classic' + location.search;

// ------------------------------------------------------------------ HUD
function remainingNow() { return Math.max(0, S.turnRemaining - (performance.now() - S.turnAt) / 1000); }
function renderClock() {
  if (S.screen !== 'turn') return;
  const rem = remainingNow();
  $('clock').textContent = fmtDur(Math.ceil(rem));
  $('hudTime').classList.toggle('warning', S.warning || rem <= 30);
  $('hudTime').classList.toggle('final', rem <= 10);
}
setInterval(renderClock, 250);
function renderScore(bump) {
  const s = $('score'); s.textContent = String(G.score);
  if (bump) { s.classList.remove('bump'); void s.offsetWidth; s.classList.add('bump'); }
  const c = $('combo'); c.hidden = G.combo < 2;
  if (G.combo >= 2) { c.textContent = `x${G.combo}`; c.style.animation = 'none'; void c.offsetWidth; c.style.animation = ''; }
}
let tickerTimer = null;
function ticker(text) {
  if (!text) return;
  $('ticker').textContent = text;
  clearTimeout(tickerTimer);
  tickerTimer = setTimeout(() => { $('ticker').textContent = idleTicker(); }, 4000);
}
function idleTicker() {
  if (!S.started) return 'Press start!';
  if (S.game === 'eyes') return 'Drag to look around · SNAP a photo';
  if (S.game === 'selfie') return selfie.aimed ? 'Drag to frame yourself · TAKE MY SELFIE' : 'Stand where people watch, then 🎯 Find me';
  if (S.game === 'cam') return "Claw's-eye view: steer, hold DROP, let go to grab";
  const gold = G.stars.some((s) => s.gold);
  return gold ? 'Gold ⬇ star: steer over it and hold DROP' : 'Fly the claw over the stars';
}
$('endBtn').addEventListener('click', () => { if (confirm('End your game now? The next player gets the claw.')) send({ op: 'leave' }); });

// ------------------------------------------------------------------ intro + pause
$('startBtn').addEventListener('click', () => {
  buzz(25);
  if (!(S.arm && S.arm.busy)) send({ op: 'ready' }); // the claw goes to its start position during the countdown
  const cd = $('countdown'); cd.hidden = false;
  const steps = ['3', '2', '1', 'GO!'];
  let i = 0;
  const step = () => {
    if (S.screen !== 'turn') return;
    if (i >= steps.length) { $('intro').hidden = true; cd.hidden = true; S.started = true; spawnStars(); ticker(idleTicker()); return; }
    cd.classList.toggle('go', i === steps.length - 1);
    cd.replaceChildren(el('span', '', steps[i]));
    buzz(i === steps.length - 1 ? [60, 40, 120] : 20);
    i++; setTimeout(step, i === steps.length ? 600 : 700);
  };
  step();
});
$('pauseBtn').addEventListener('click', () => { releaseAll(); $('pause').hidden = false; });
$('resumeBtn').addEventListener('click', () => { $('pause').hidden = true; });
$('pause').addEventListener('click', (e) => { if (e.target === $('pause')) $('pause').hidden = true; });
$('howAgainBtn').addEventListener('click', () => { $('pause').hidden = true; $('intro').hidden = false; $('countdown').hidden = true; });
$('flipBtn').addEventListener('click', () => {
  S.flip = !S.flip; store.set('arcade-flip', S.flip ? '1' : '0');
  $('pause').hidden = true; toast(S.flip ? 'Map flipped: the arm is at the top' : 'Map flipped: the arm is at the bottom');
});
$('homeBtn').addEventListener('click', () => { $('pause').hidden = true; releaseAll(); send({ op: 'home' }); });
$('amBtn').addEventListener('click', () => { releaseAll(); send({ op: 'ready' }); });
$('lowReady').addEventListener('click', () => { releaseAll(); if (send({ op: 'ready' })) $('lowBar').hidden = true; });

// ------------------------------------------------------------------ the arm on the map
const armFolded = () => !!(S.arm && S.arm.q && !S.arm.busy && S.arm.q.every((x) => Math.abs(x) < 0.08));
let tipNow = null; // [x, y, z] from the last arm state
function onArm() {
  const a = S.arm;
  tipNow = a && Array.isArray(a.q) && a.q.length >= 6 ? tip(a.q) : null;
  if (tipNow) {
    G.trail.push({ x: tipNow[0], y: tipNow[1], t: performance.now() });
    if (G.trail.length > 40) G.trail.shift();
    collectStars();
  }
  renderFault(); renderArm();
}
function renderFault() {
  const f = S.arm && S.arm.fault;
  if (!f) { $('faultBar').hidden = true; return; }
  if ($('faultBar').hidden) { releaseAll(); buzz([200, 80, 200]); }
  $('faultBar').hidden = false; $('faultWhy').textContent = f.reason || '';
}
function renderArm() {
  const a = S.arm, busy = a && a.busy;
  let icon = '', text = '', btn = false;
  if (!ws || ws.readyState !== 1) { icon = '📡'; text = 'Reconnecting…'; }
  else if (!a) { icon = '🤖'; text = 'Waiting for the claw…'; }
  else if (busy === 'ready') { icon = grab.state ? '🎁' : '🦾'; text = grab.state ? 'Bringing it back up…' : 'Claw to start position…'; }
  else if (busy === 'home') { icon = '🏠'; text = 'Parking the claw…'; }
  else if (busy) { icon = '💃'; text = `Playing: ${busy}`; }
  else if (armFolded()) { icon = '😴'; text = 'The claw is parked'; btn = true; }
  const msg = $('arenaMsg');
  msg.hidden = !text || !$('lowBar').hidden;
  if (text) { $('amIcon').textContent = icon; $('amText').textContent = text; $('amBtn').hidden = !btn; }
  // a grab is over once the return trip (`ready`) has started and finished
  if (grab.state === 'returning' && busy === 'ready') grab.seen = true;
  if (grab.state === 'returning' && grab.seen && !busy) grab.state = null;
  for (const b of document.querySelectorAll('.emote')) b.classList.toggle('playing', !!busy && (b.dataset.name === busy || b.dataset.title === busy));
  $('emotes').classList.toggle('dim', !!busy);
  const shut = !!(a && typeof a.grip === 'number' && a.grip < 0.5);
  $('clawBtn').classList.toggle('shut', shut);
  $('clawIcon').textContent = shut ? '✊' : '🖐️';
  $('clawLbl').textContent = shut ? 'Claw shut' : 'Claw open';
  $('dragHint').hidden = driving() || !S.started;
  $('dragHint').textContent = S.game === 'selfie' ? 'drag to frame yourself' : S.game === 'eyes' ? 'drag to look around' : 'drag to steer';
}

// ------------------------------------------------------------------ stars (the game layer)
function spawnStar(gold) {
  for (let n = 0; n < 40; n++) {
    const [lo, hi] = j1Range(), a = Math.max(lo + 0.08, -0.95), b = Math.min(hi - 0.08, 0.95);
    const r = 0.17 + Math.random() * 0.19, th = a + Math.random() * Math.max(0, b - a);
    const s = { x: r * Math.cos(th), y: r * Math.sin(th), gold: !!gold, born: performance.now() };
    const far = (o) => Math.hypot(o.x - s.x, o.y - s.y) > 0.09;
    if (G.stars.every(far) && (!tipNow || Math.hypot(tipNow[0] - s.x, tipNow[1] - s.y) > 0.1)) return s;
  }
  return null;
}
function spawnStars() {
  while (G.stars.filter((s) => !s.gold).length < 3) { const s = spawnStar(false); if (!s) break; G.stars.push(s); }
  if (!G.stars.some((s) => s.gold) && G.stats.stars >= 2) { const s = spawnStar(true); if (s) G.stars.push(s); }
}
function collectStars() {
  if (!S.started || S.screen !== 'turn' || !tipNow || S.game !== 'claw') return;
  const busy = S.arm && S.arm.busy;
  if (busy && busy !== 'ready') return; // emotes and Ask don't collect stars: you have to fly there
  const [x, y, z] = tipNow;
  const now = performance.now();
  for (const s of G.stars.slice()) {
    if (Math.hypot(s.x - x, s.y - y) > STAR_R || (s.gold && z > DIVE_Z)) continue;
    G.stars.splice(G.stars.indexOf(s), 1);
    G.combo = now - G.lastAt < COMBO_S * 1000 ? Math.min(5, G.combo + 1) : 1;
    G.lastAt = now;
    const pts = (s.gold ? 250 : 100) * G.combo;
    G.score += pts;
    if (s.gold) G.stats.dives++; else G.stats.stars++;
    G.stats.bestCombo = Math.max(G.stats.bestCombo, G.combo);
    G.pops.push({ x: s.x, y: s.y, z: s.gold ? DIVE_Z : STAR_Z, text: `+${pts}`, t: now, gold: s.gold });
    buzz(s.gold ? [30, 30, 30, 30, 80] : [25, 30, 40]);
    ticker(G.combo >= 2 ? `COMBO x${G.combo}!` : s.gold ? 'GOLD DIVE!' : 'Nice!');
    renderScore(true);
  }
  spawnStars();
}
function award(pts, text) {
  G.score += pts; renderScore(true);
  if (tipNow) G.pops.push({ x: tipNow[0], y: tipNow[1], z: tipNow[2], text, t: performance.now(), gold: false });
}

// The map: an angled view into a claw machine. The table is the floor (foreshortened by KD), the
// arm's base sits at its near edge (far edge when flipped: the player stands across the table),
// +x runs up the screen, +y to the left. The claw hangs on a cable at its real height above its
// shadow, so the one picture shows where it is over the table and how high.
const canvas = $('map'), cx2d = canvas.getContext('2d');
const M = { w: 0, h: 0, dpr: 1, s: 1, zs: 1, near: 0, far: 0 };
function resizeMap() {
  const r = canvas.getBoundingClientRect(); if (!r.width) return;
  M.dpr = Math.min(3, window.devicePixelRatio || 1);
  M.w = r.width; M.h = r.height;
  canvas.width = Math.round(r.width * M.dpr); canvas.height = Math.round(r.height * M.dpr);
  M.s = (M.w - 44) / (2 * 0.45);                         // px per metre across the table
  M.near = M.h - 22; M.far = M.near - MAP_R * M.s * KD;   // the floor's near and far edges
  M.zs = Math.max(60, (M.far - 34) / (0.6 - TABLE_Z));    // px per metre of height
}
window.addEventListener('resize', resizeMap);
const F = () => (S.flip && S.game === 'claw' ? -1 : 1); // the camera games steer in the camera's frame
// base frame (x, y, z) -> screen px
function toScreen(x, y, z = TABLE_Z) {
  const fy = S.flip ? M.far + x * M.s * KD : M.near - x * M.s * KD;
  return [M.w / 2 - F() * y * M.s, fy - (z - TABLE_Z) * M.zs];
}
function floorPath(c, pts) { c.beginPath(); pts.forEach(([x, y], i) => { const [px, py] = toScreen(x, y); i ? c.lineTo(px, py) : c.moveTo(px, py); }); }
function arcPts(r, a, b, n = 40) { const o = []; for (let i = 0; i <= n; i++) { const t = a + (b - a) * i / n; o.push([r * Math.cos(t), r * Math.sin(t)]); } return o; }

// ------------------------------------------------------------------ drawing
function draw() {
  requestAnimationFrame(draw);
  if (S.screen !== 'turn' || !M.w || S.game !== 'claw') return;
  const c = cx2d, now = performance.now();
  c.setTransform(M.dpr, 0, 0, M.dpr, 0, 0);
  c.clearRect(0, 0, M.w, M.h);
  c.textAlign = 'center';
  // the floor: rings every 10 cm
  c.lineWidth = 1; c.strokeStyle = 'rgba(169,159,214,.16)';
  for (let r = 0.1; r <= MAP_R + 1e-6; r += 0.1) { floorPath(c, arcPts(r, -Math.PI / 2, Math.PI / 2)); c.stroke(); }
  // the play zone: between the soft walls, inside the base's allowed turn
  const [lo, hi] = j1Range();
  floorPath(c, [...arcPts(R_MAX, lo, hi), ...arcPts(R_MIN, hi, lo, 12)]); c.closePath();
  c.fillStyle = 'rgba(62,230,255,.07)'; c.fill();
  c.strokeStyle = 'rgba(62,230,255,.4)'; c.lineWidth = 2; c.setLineDash([6, 6]); c.stroke(); c.setLineDash([]);
  // the arm's base
  const [bx, by] = toScreen(0, 0);
  c.fillStyle = '#3a2b7a'; c.beginPath(); c.ellipse(bx, by, 22, 22 * KD, 0, 0, 2 * Math.PI); c.fill();
  c.fillStyle = '#a99fd6'; c.font = '800 10px system-ui'; c.fillText('ARM', bx, by + 4);
  // the claw's shadow and trail on the floor
  if (tipNow) {
    c.lineCap = 'round';
    for (let i = 1; i < G.trail.length; i++) {
      const a = G.trail[i - 1], b = G.trail[i], k = i / G.trail.length;
      const [ax, ay] = toScreen(a.x, a.y), [b2x, b2y] = toScreen(b.x, b.y);
      c.strokeStyle = `rgba(62,230,255,${0.35 * k})`; c.lineWidth = 5 * k;
      c.beginPath(); c.moveTo(ax, ay); c.lineTo(b2x, b2y); c.stroke();
    }
    const [sx, sy] = toScreen(tipNow[0], tipNow[1]);
    c.fillStyle = 'rgba(0,0,0,.5)'; c.beginPath(); c.ellipse(sx, sy, 16, 16 * KD, 0, 0, 2 * Math.PI); c.fill();
  }
  // stars: a ring on the floor where to steer, the star at the height that collects it
  const stars = G.stars.slice().sort((a, b) => F() * (b.x - a.x)); // far ones first
  for (const s of stars) {
    const z = s.gold ? DIVE_Z - 0.06 : STAR_Z, col = s.gold ? '#ffb020' : '#ffd23f';
    const [fx, fy] = toScreen(s.x, s.y), [px, py] = toScreen(s.x, s.y, z);
    const pulse = 1 + 0.12 * Math.sin(now / 220 + s.x * 40), grow = Math.min(1, (now - s.born) / 330);
    c.strokeStyle = s.gold ? 'rgba(255,176,32,.55)' : 'rgba(255,210,63,.35)'; c.lineWidth = 2;
    c.beginPath(); c.ellipse(fx, fy, STAR_R * M.s, STAR_R * M.s * KD, 0, 0, 2 * Math.PI); c.stroke();
    const g = c.createLinearGradient(0, py, 0, fy);
    g.addColorStop(0, s.gold ? 'rgba(255,176,32,.35)' : 'rgba(255,210,63,.22)'); g.addColorStop(1, 'rgba(255,210,63,0)');
    c.fillStyle = g; c.fillRect(px - 3, py, 6, fy - py);
    star(c, px, py, (s.gold ? 17 : 14) * pulse * grow, col);
    if (s.gold) { c.fillStyle = '#2a1600'; c.font = '900 12px system-ui'; c.fillText('⬇', px, py + 4); }
  }
  // the claw on its cable, at its real height
  if (tipNow) {
    const [x, y, z] = tipNow, [tx, ty] = toScreen(x, y, z);
    c.strokeStyle = 'rgba(169,159,214,.55)'; c.lineWidth = 2;
    c.beginPath(); c.moveTo(tx, 0); c.lineTo(tx, ty - 22); c.stroke();
    claw(c, tx, ty - 14, 1.15, S.arm && typeof S.arm.grip === 'number' ? S.arm.grip : 1, ctl.drop);
    heightGauge(c, z);
  }
  // score pops
  G.pops = G.pops.filter((p) => now - p.t < 1100);
  for (const p of G.pops) {
    const k = (now - p.t) / 1100, [px, py] = toScreen(p.x, p.y, p.z);
    c.globalAlpha = 1 - k; c.fillStyle = p.gold ? '#ffb020' : '#4dffa6'; c.font = `900 ${22 + 8 * (1 - k)}px system-ui`;
    c.fillText(p.text, px, py - 24 - 40 * k); c.globalAlpha = 1;
  }
}
function star(c, x, y, r, color) {
  c.save(); c.translate(x, y); c.beginPath();
  for (let i = 0; i < 10; i++) {
    const a = -Math.PI / 2 + i * Math.PI / 5, rr = i % 2 ? r * 0.45 : r;
    c.lineTo(Math.cos(a) * rr, Math.sin(a) * rr);
  }
  c.closePath(); c.shadowColor = color; c.shadowBlur = 14; c.fillStyle = color; c.fill(); c.restore();
}
function claw(c, x, y, s, grip, dropping) {
  c.save(); c.translate(x, y); c.scale(s, s);
  c.strokeStyle = dropping ? '#ffd23f' : '#3ee6ff'; c.lineWidth = 4; c.lineCap = 'round'; c.lineJoin = 'round';
  c.shadowColor = c.strokeStyle; c.shadowBlur = 12;
  c.fillStyle = '#231757'; c.beginPath(); c.arc(0, -8, 8, 0, 2 * Math.PI); c.fill(); c.stroke();
  const o = 4 + 8 * clamp(grip, 0, 1); // fingers spread with the gripper opening
  c.beginPath(); c.moveTo(-5, -2); c.lineTo(-o - 4, 8); c.lineTo(-o + 1, 16); c.stroke();
  c.beginPath(); c.moveTo(5, -2); c.lineTo(o + 4, 8); c.lineTo(o - 1, 16); c.stroke();
  c.restore();
}
function heightGauge(c, z) {
  const x = M.w - 14, top = 18, bot = M.h - 18, h = bot - top;
  const zMax = 0.6, k = clamp((z - DROP_FLOOR_Z) / (zMax - DROP_FLOOR_Z), 0, 1);
  c.fillStyle = 'rgba(18,11,46,.8)'; c.fillRect(x - 4, top, 8, h);
  const dive = clamp((DIVE_Z - DROP_FLOOR_Z) / (zMax - DROP_FLOOR_Z), 0, 1);
  c.fillStyle = 'rgba(255,176,32,.35)'; c.fillRect(x - 4, bot - dive * h, 8, dive * h);
  c.fillStyle = ctl.drop ? '#ffd23f' : '#3ee6ff'; c.beginPath(); c.arc(x, bot - k * h, 6, 0, 2 * Math.PI); c.fill();
}
requestAnimationFrame(draw);

// ------------------------------------------------------------------ live control (stream)
const ctl = { steer: false, sx: 0, sy: 0, drop: false, dropFromZ: null, floorHit: false };
let streamTimer = null, holdEp = 0, lastTick = 0, streamKind = 'v';
const grab = { state: null, seen: false }; // null | 'closing' (claw shutting) | 'returning' (ready sent)
const driving = () => ctl.steer || ctl.drop;
function canDrive() {
  if (S.screen !== 'turn' || !S.started) return false;
  if (!$('pause').hidden || !$('askSheet').hidden || !$('intro').hidden || !$('photo').hidden) return false;
  if (!ws || ws.readyState !== 1) { toast('Not connected right now. Reconnecting…', 'error'); return false; }
  if (S.arm && S.arm.busy) { buzz(15); ticker(`Wait for it… (${S.arm.busy})`); return false; }
  if (armFolded()) { toast('The claw is parked. Tap "Unfold the claw".'); return false; }
  return true;
}
const dz = (v) => (Math.abs(v) < THUMB_DEAD ? 0 : (v - Math.sign(v) * THUMB_DEAD) / (1 - THUMB_DEAD));
function steerVel() {
  // thumb offset -> base-frame velocity (as play.js Sticks: up = +x away from the base, right = -y),
  // with a softer curve near the centre for fine aiming; flipped when the player stands across
  let ux = dz(ctl.sx), uy = dz(ctl.sy);
  const n = Math.hypot(ux, uy);
  if (n > 0) { const k = Math.pow(Math.min(1, n), 1.5) / n; ux *= k; uy *= k; }
  let vx = F() * -uy, vy = F() * -ux;
  // soft walls: drop the component that pushes further out of the play zone
  if (tipNow) {
    const r = Math.hypot(tipNow[0], tipNow[1]);
    if (r > 1e-3) {
      const ex = tipNow[0] / r, ey = tipNow[1] / r, out = vx * ex + vy * ey;
      if ((r >= R_MAX && out > 0) || (r <= R_MIN && out < 0)) { vx -= out * ex; vy -= out * ey; }
      // side walls: the base's turn (the tip's angle) stays inside the j1 soft limits
      const [lo, hi] = j1Range(), ang = Math.atan2(tipNow[1], tipNow[0]), tx = -ey, ty = ex, turn = vx * tx + vy * ty;
      if ((ang >= hi && turn > 0) || (ang <= lo && turn < 0)) { vx -= turn * tx; vy -= turn * ty; }
    }
  }
  return { vx, vy };
}
function tick() {
  const now = Date.now();
  if (lastTick && now - lastTick > Math.max(250, 3 * RATE_MS)) diag('late_tick', { dt_ms: now - lastTick, ep: holdEp });
  lastTick = now;
  if (!driving()) { stopStream('up'); return; }
  if (looking()) {
    // look around: drag right = turn right (j1 -), drag up = tilt the camera up (j4 +; FK: at ready
    // the camera looks down the table and +j4 raises it)
    const ux = dz(ctl.sx), uy = dz(ctl.sy);
    streamKind = 'qd';
    send({ op: 'stream', qd: [r3(clamp(-ux * LOOK_K)), 0, 0, r3(clamp(-uy * LOOK_K)), 0, 0], ep: holdEp });
    return;
  }
  streamKind = 'v';
  const v = { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 };
  if (ctl.steer) { const s = steerVel(); v.x = s.vx; v.y = s.vy; }
  if (ctl.drop) {
    const atFloor = tipNow && tipNow[2] <= DROP_FLOOR_Z;
    if (atFloor && !ctl.floorHit) { ctl.floorHit = true; buzz([40, 30, 40]); ticker('Floor! Let go to grab'); }
    v.z = atFloor ? 0 : -1;
  }
  for (const k in v) v[k] = r3(clamp(v[k]));
  send({ op: 'stream', v, ep: holdEp });
}
function startStream() {
  if (!streamTimer) { holdEp++; lastTick = 0; tick(); streamTimer = setInterval(tick, RATE_MS); renderArm(); }
}
function stopStream(why) {
  if (!streamTimer) return;
  clearInterval(streamTimer); streamTimer = null; lastTick = 0;
  // one explicit zero of the same kind, so the arm stops now rather than at the 300 ms deadman
  if (streamKind === 'qd') send({ op: 'stream', qd: [0, 0, 0, 0, 0, 0], ep: holdEp, why: why || 'ui' });
  else send({ op: 'stream', v: { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 }, ep: holdEp, why: why || 'ui' });
  renderArm();
}
let endWhy = null;
function maybeStop() { if (!driving()) stopStream(endWhy || 'up'); }
function releaseAll(why) {
  why = typeof why === 'string' ? why : 'ui';
  ctl.steer = false; ctl.sx = ctl.sy = 0; ctl.drop = false; ctl.dropFromZ = null; ctl.floorHit = false;
  for (const r of resets) r();
  $('thumb').hidden = true;
  $('arena').classList.remove('steering', 'dropping');
  stopStream(why);
}
const resets = [];
function holdable(node, down, up) {
  let held = null;
  node.addEventListener('pointerdown', (e) => {
    if (held !== null || !canDrive()) return;
    e.preventDefault(); held = e.pointerId;
    try { node.setPointerCapture(e.pointerId); } catch (_) {}
    node.classList.add('on'); down(e);
  });
  node.addEventListener('pointermove', (e) => { if (e.pointerId === held && node._move) node._move(e); });
  const end = (e) => {
    if (e.pointerId !== held) return;
    held = null; node.classList.remove('on');
    endWhy = e.type === 'pointerup' ? 'up' : e.type === 'pointercancel' ? 'cancel' : 'lostcapture';
    if (endWhy !== 'up') diag(e.type, { id: node.id, streaming: !!streamTimer });
    try { up(e); } finally { endWhy = null; }
  };
  for (const t of ['pointerup', 'pointercancel', 'lostpointercapture']) node.addEventListener(t, end);
  node.addEventListener('contextmenu', (e) => e.preventDefault());
  resets.push(() => { held = null; node.classList.remove('on'); });
}

// steer: a floating stick wherever the thumb lands on the map
const arena = $('arena'), thumb = $('thumb'), knob = thumb.querySelector('i');
let origin = null;
arena._move = (e) => {
  if (!origin) return;
  let dx = (e.clientX - origin.x) / THUMB_R, dy = (e.clientY - origin.y) / THUMB_R;
  const n = Math.hypot(dx, dy); if (n > 1) { dx /= n; dy /= n; }
  ctl.sx = dx; ctl.sy = dy;
  knob.style.transform = `translate(${dx * THUMB_R * 0.6}px, ${dy * THUMB_R * 0.6}px)`;
};
holdable(arena, (e) => {
  const r = arena.getBoundingClientRect();
  origin = { x: e.clientX, y: e.clientY };
  thumb.style.left = `${e.clientX - r.left}px`; thumb.style.top = `${e.clientY - r.top}px`;
  thumb.hidden = false; knob.style.transform = '';
  ctl.steer = true; ctl.sx = ctl.sy = 0;
  arena.classList.add('steering'); buzz(10);
  startStream();
}, () => {
  origin = null; thumb.hidden = true; ctl.steer = false; ctl.sx = ctl.sy = 0;
  arena.classList.remove('steering'); maybeStop();
});

// drop: hold to lower; let go = shut the claw, then come back up to the ready pose
holdable($('dropBtn'), () => {
  ctl.drop = true; ctl.floorHit = false; ctl.dropFromZ = tipNow ? tipNow[2] : null;
  arena.classList.add('dropping'); buzz(20); ticker('Dropping… let go to grab');
  startStream();
}, () => {
  const fell = ctl.dropFromZ != null && tipNow ? ctl.dropFromZ - tipNow[2] : 0;
  ctl.drop = false; ctl.dropFromZ = null; arena.classList.remove('dropping');
  // the grab ends the steering too: one clean stop, then the claw shuts and returns
  ctl.steer = false; origin = null; thumb.hidden = true; arena.classList.remove('steering');
  stopStream(endWhy || 'up');
  if (endWhy && endWhy !== 'up') return; // the browser ended the hold (not a deliberate let-go): just stop
  grab.state = 'closing'; grab.seen = false;
  send({ op: 'grip', value: 0 });
  buzz([30, 60, 30]); ticker('GRAB!');
  if (fell >= GRAB_MIN_DROP) { G.stats.grabs++; award(50, '+50 GRAB'); }
  setTimeout(() => {
    if (grab.state !== 'closing') return; // STOP or another action came first
    if (S.screen === 'turn' && !driving() && !(S.arm && S.arm.busy) && send({ op: 'ready' })) {
      grab.state = 'returning';
      setTimeout(() => { if (grab.state === 'returning' && !grab.seen) grab.state = null; }, 3000); // refused
    } else grab.state = null;
  }, 800);
});

// claw: tap to open / shut
$('clawBtn').addEventListener('click', () => {
  if (S.screen !== 'turn') return;
  const shut = S.arm && typeof S.arm.grip === 'number' && S.arm.grip < 0.5;
  send({ op: 'grip', value: shut ? 1 : 0 }); buzz(15);
});

// STOP
$('stopBtn').addEventListener('click', () => {
  selfie.after = false; releaseAll(); grab.state = null; send({ op: 'stop' }); buzz(40); });

// ------------------------------------------------------------------ emotes (library moves)
function renderEmotes() {
  const box = $('emotes'); box.replaceChildren();
  const moves = (S.welcome && S.welcome.moves) || [];
  box.hidden = !moves.length || looking();
  for (const m of moves) {
    const b = el('button', 'emote' + (G.emotesDone.has(m.name) ? ' done' : ''), m.emoji || '🤖');
    b.dataset.name = m.name; b.dataset.title = m.title || m.name;
    b.setAttribute('aria-label', m.title || m.name);
    b.append(el('small', '', m.title || m.name));
    b.addEventListener('click', () => {
      if (S.screen !== 'turn' || !S.started) return;
      releaseAll(); buzz(15);
      if (send({ op: 'move', name: m.name }) && !G.emotesDone.has(m.name)) {
        G.emotesDone.add(m.name); G.stats.emotes++; b.classList.add('done'); award(25, '+25');
      }
    });
    box.append(b);
  }
}

// ------------------------------------------------------------------ ask it (say)
const askText = $('askText');
const askOk = () => { $('askGo').disabled = !askText.value.trim(); };
askText.addEventListener('input', askOk);
for (const c of CHIPS) {
  const b = el('button', 'chip', c);
  b.addEventListener('click', () => { askText.value = c; askOk(); });
  $('askChips').append(b);
}
$('askBtn').addEventListener('click', () => { if (S.screen === 'turn' && S.started) { releaseAll(); $('askSheet').hidden = false; askOk(); } });
$('askCancel').addEventListener('click', () => { $('askSheet').hidden = true; askText.blur(); });
function doAsk() {
  const t = askText.value.trim().slice(0, 200);
  if (!t) return;
  askText.blur(); $('askSheet').hidden = true;
  if (send({ op: 'say', text: t })) ticker(`✨ "${t}": thinking…`);
}
$('askGo').addEventListener('click', doAsk);
askText.addEventListener('keydown', (e) => { if (e.key === 'Enter' && !e.shiftKey) { e.preventDefault(); doAsk(); } });
function onSay(m) {
  if (m.stage === 'thinking') ticker('✨ Thinking up a move…');
  else if (m.stage === 'playing') ticker(`✨ ${m.title || 'Your move'}${m.note ? ` (${m.note})` : ''}`);
  else if (m.stage === 'done') { ticker('✨ Done!'); award(25, '+25'); }
  else if (m.stage === 'error') toast(m.note || "Couldn't do that one", 'error');
}

// ------------------------------------------------------------------ camera games
function renderGames() {
  const cam = features().cam;
  $('games').hidden = !cam;
  if (!cam && S.game !== 'claw') { setGame('claw'); toast('The camera went away: back to the claw game'); }
}
function setGame(g, quiet) {
  if (g !== 'claw' && !features().cam) g = 'claw';
  if (S.game !== g) releaseAll();
  if (g !== 'selfie') selfie.aimed = false;   // the arm will move: aim again before the next selfie
  S.game = g;
  for (const b of $('games').children) b.setAttribute('aria-pressed', String(b.dataset.game === g));
  const eyes = g === 'eyes', sel = g === 'selfie', look = eyes || sel, claw = g === 'claw';
  $('map').hidden = !claw; $('camView').hidden = claw;
  $('emotes').hidden = look || !((S.welcome && S.welcome.moves) || []).length;
  $('filters').hidden = !look;
  for (const id of ['clawBtn', 'dropBtn', 'askBtn']) $(id).hidden = look;
  $('filterBtn').hidden = !look;
  $('snapBtn').hidden = !eyes; $('waveBtn').hidden = !eyes || !waveMove();
  $('findBtn').hidden = !sel; $('selfieBtn').hidden = !sel;
  renderFilters();
  if (!claw) camStart(); else camStop();
  if (claw) requestAnimationFrame(resizeMap); else requestAnimationFrame(resizeCam);
  renderArm();
  if (!quiet) { buzz(15); ticker(idleTicker()); }
}
for (const b of $('games').children) b.addEventListener('click', () => setGame(b.dataset.game));

// frames: GET /cam.jpg?pid=&after=<seq> (the server waits up to 1 s for a newer frame), one at a time
const camCanvas = $('camView'), camCtx = camCanvas.getContext('2d');
const C = { on: false, seq: 0, img: null, w: 0, h: 0, dpr: 1, err: null, gen: 0, fails: 0 };
function resizeCam() {
  const r = camCanvas.getBoundingClientRect(); if (!r.width) return;
  C.dpr = Math.min(2, window.devicePixelRatio || 1);
  C.w = r.width; C.h = r.height;
  camCanvas.width = Math.round(r.width * C.dpr); camCanvas.height = Math.round(r.height * C.dpr);
}
window.addEventListener('resize', resizeCam);
function camUrl() {
  const k = new URLSearchParams(location.search).get('k');
  return `/cam.jpg?pid=${encodeURIComponent(pid)}&after=${C.seq}` + (k ? `&k=${encodeURIComponent(k)}` : '');
}
function camStart() { if (C.on) return; C.on = true; C.gen++; C.fails = 0; camLoop(C.gen); }
function camStop() { C.on = false; C.gen++; }
async function camLoop(gen) {
  while (C.on && gen === C.gen && S.screen === 'turn') {
    try {
      const r = await fetch(camUrl(), { cache: 'no-store', credentials: 'same-origin' });
      if (!r.ok) throw new Error(r.status === 403 ? 'not your turn' : (await r.text()).slice(0, 120) || `HTTP ${r.status}`);
      C.seq = parseInt(r.headers.get('X-Seq') || '0', 10) || C.seq;
      const blob = await r.blob();
      const img = await createImageBitmap(blob);
      if (gen !== C.gen) break;
      if (C.img && C.img.close) C.img.close();
      C.img = img; C.err = null; C.fails = 0;
    } catch (e) {
      C.err = String(e.message || e); C.fails++;
      await new Promise((ok) => setTimeout(ok, Math.min(3000, 500 * C.fails)));
    }
  }
}
// filters on a small copy (fast on any phone), drawn back scaled to the arena
const fx = document.createElement('canvas'), fxc = fx.getContext('2d', { willReadFrequently: true });
const THERMAL = [[0, 0, 40], [60, 0, 120], [180, 0, 120], [255, 60, 0], [255, 180, 0], [255, 255, 160]];
function thermal(l) {
  const x = l / 255 * (THERMAL.length - 1), i = Math.min(THERMAL.length - 2, Math.floor(x)), f = x - i, a = THERMAL[i], b = THERMAL[i + 1];
  return [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f];
}
function filtered(img, filter) {
  // the frame at its own aspect on a small canvas (fast on any phone), with the filter applied
  const sw = filter === 'pixel' ? 64 : Math.min(img.width, 400), sh = Math.round(sw * img.height / img.width);
  fx.width = sw; fx.height = sh;
  fxc.imageSmoothingEnabled = true;
  fxc.drawImage(img, 0, 0, sw, sh);
  if (filter === 'thermal' || filter === 'robot' || filter === 'comic') {
    const d = fxc.getImageData(0, 0, sw, sh), p = d.data;
    for (let i = 0; i < p.length; i += 4) {
      const l = 0.3 * p[i] + 0.59 * p[i + 1] + 0.11 * p[i + 2];
      if (filter === 'thermal') { const c = thermal(l); p[i] = c[0]; p[i + 1] = c[1]; p[i + 2] = c[2]; }
      else if (filter === 'robot') { const y = (i / 4 / sw) | 0, scan = y % 3 === 0 ? 0.55 : 1; p[i] = l * 0.25 * scan; p[i + 1] = Math.min(255, l * 1.15) * scan; p[i + 2] = l * 0.45 * scan; }
      else { const q = l > 170 ? 255 : l > 95 ? 150 : l > 45 ? 70 : 10; p[i] = Math.min(255, q * 1.1); p[i + 1] = q * 0.85; p[i + 2] = q * 0.55 + 20; }
    }
    fxc.putImageData(d, 0, 0);
  }
  return fx;
}
// draw a (filtered) frame into the box x, y, w, h: "contain" shows all of it, "cover" fills the box
function fitDraw(c, src, x, y, w, h, mode) {
  const k = (mode === 'cover' ? Math.max : Math.min)(w / src.width, h / src.height);
  const dw = src.width * k, dh = src.height * k;
  c.save(); c.beginPath(); c.rect(x, y, w, h); c.clip();
  c.drawImage(src, x + (w - dw) / 2, y + (h - dh) / 2, dw, dh);
  c.restore();
  return [x + Math.max(0, (w - dw) / 2), y + Math.max(0, (h - dh) / 2), Math.min(w, dw), Math.min(h, dh)];
}
function drawCam() {
  requestAnimationFrame(drawCam);
  if (S.screen !== 'turn' || S.game === 'claw' || !C.w) return;
  const c = camCtx, w = C.w, h = C.h, now = performance.now();
  c.setTransform(C.dpr, 0, 0, C.dpr, 0, 0);
  c.fillStyle = '#000'; c.fillRect(0, 0, w, h);
  let box = [0, 0, w, h];
  if (C.img) {
    const f = S.game === 'eyes' ? curFilter() : 'none';
    c.imageSmoothingEnabled = f !== 'pixel';
    box = fitDraw(c, filtered(C.img, f), 0, 0, w, h, 'contain'); // the whole field of view
  } else {
    c.fillStyle = '#a99fd6'; c.font = '700 16px system-ui'; c.textAlign = 'center';
    c.fillText(C.err ? `📷 ${C.err}` : '📷 Connecting to the camera…', w / 2, h / 2);
  }
  c.textAlign = 'left'; c.font = '800 13px system-ui'; c.lineWidth = 3; c.lineCap = 'round';
  if (S.game === 'eyes') {
    // a robot's HUD: corner brackets, a blinking REC, a reticle
    c.strokeStyle = curFilter() === 'robot' ? '#4dffa6' : 'rgba(255,255,255,.85)';
    const [bx, by, bw, bh] = box, m = 10, L = 26;
    for (const [x, y, sx, sy] of [[bx + m, by + m, 1, 1], [bx + bw - m, by + m, -1, 1], [bx + m, by + bh - m, 1, -1], [bx + bw - m, by + bh - m, -1, -1]]) {
      c.beginPath(); c.moveTo(x, y + sy * L); c.lineTo(x, y); c.lineTo(x + sx * L, y); c.stroke();
    }
    if (Math.floor(now / 600) % 2) { c.fillStyle = '#ff3b5c'; c.beginPath(); c.arc(bx + m + 18, by + m + 22, 6, 0, 2 * Math.PI); c.fill(); }
    c.fillStyle = '#fff'; c.fillText('ROBOT EYES', bx + m + 30, by + m + 27);
    c.beginPath(); c.arc(w / 2, h / 2, 22, 0, 2 * Math.PI); c.stroke();
    c.beginPath(); c.moveTo(w / 2 - 34, h / 2); c.lineTo(w / 2 - 12, h / 2); c.moveTo(w / 2 + 12, h / 2); c.lineTo(w / 2 + 34, h / 2); c.stroke();
  } else if (S.game === 'selfie') {
    // a face guide and the state of the aim
    c.strokeStyle = selfie.aimed ? 'rgba(77,255,166,.85)' : 'rgba(255,255,255,.6)'; c.setLineDash([8, 8]);
    c.beginPath(); c.ellipse(w / 2, h / 2, Math.min(w, h) * 0.2, Math.min(w, h) * 0.26, 0, 0, 2 * Math.PI); c.stroke(); c.setLineDash([]);
    c.fillStyle = 'rgba(18,11,46,.7)'; c.fillRect(10, h - 38, 190, 26);
    c.fillStyle = '#fff';
    c.fillText(selfie.aimed ? '🤳 LOOKING AT YOU' : selfie.aiming ? '🤳 TURNING TO YOU…' : '🤳 TAP 🎯 FIND ME', 18, h - 20);
  } else {
    // claw cam: crosshair and the claw's height over the table
    c.strokeStyle = ctl.drop ? '#ffd23f' : '#3ee6ff';
    c.beginPath(); c.moveTo(w / 2 - 20, h / 2); c.lineTo(w / 2 + 20, h / 2); c.moveTo(w / 2, h / 2 - 20); c.lineTo(w / 2, h / 2 + 20); c.stroke();
    c.fillStyle = 'rgba(18,11,46,.7)'; c.fillRect(10, h - 38, 170, 26);
    c.fillStyle = '#fff';
    c.fillText(tipNow ? `CLAW CAM · ${Math.max(0, Math.round((tipNow[2] - TABLE_Z) * 100))} cm up` : 'CLAW CAM', 18, h - 20);
  }
}
requestAnimationFrame(drawCam);

// filters
function renderFilters() {
  const box = $('filters'); box.replaceChildren();
  for (const [id, label] of FILTERS) {
    const b = el('button', 'filter', label); b.setAttribute('aria-pressed', String(curFilter() === id));
    b.addEventListener('click', () => setFilter(id)); box.append(b);
  }
  $('filterLbl').textContent = (FILTERS.find((f) => f[0] === curFilter()) || FILTERS[0])[1];
}
function setFilter(id) { S.filterBy[S.game] = id; buzz(10); renderFilters(); }
$('filterBtn').addEventListener('click', () => {
  const i = FILTERS.findIndex((f) => f[0] === curFilter());
  setFilter(FILTERS[(i + 1) % FILTERS.length][0]);
});
renderFilters();

// wave at the person holding the phone (a library move named like "wave", if there is one)
function waveMove() { return ((S.welcome && S.welcome.moves) || []).find((m) => /wave/i.test(m.name + ' ' + (m.title || ''))); }
$('waveBtn').addEventListener('click', () => {
  const m = waveMove(); if (!m || S.screen !== 'turn' || !S.started) return;
  releaseAll(); buzz(15); send({ op: 'move', name: m.name });
});

// Photos (Robot Eyes SNAP and 🤳 Selfie): count down, ask the server to take it (it saves the
// camera's original on the laptop), frame the original here, offer it to save, and upload the
// framed copy so the laptop has it too.
const selfie = { aimed: false, aiming: false, after: false, kind: 'eyes', countdown: false };
const photo = { id: null, url: null, blob: null, kind: 'eyes' };
const sleep = (ms) => new Promise((ok) => setTimeout(ok, ms));
async function countdownAndSnap(kind) {
  if (selfie.countdown || S.screen !== 'turn') return;
  if (!C.img) { toast('No picture from the camera yet'); return; }
  selfie.countdown = true; selfie.kind = kind; releaseAll();
  const cnt = $('camCount'), steps = kind === 'selfie' ? ['5', '4', '3', '2', '1', '😁'] : ['3', '2', '1'];
  for (const n of steps) {
    if (S.screen !== 'turn' || !looking()) { cnt.hidden = true; selfie.countdown = false; return; }
    cnt.hidden = false; cnt.textContent = n; buzz(20); await sleep(n === '😁' ? 350 : 700);
  }
  cnt.hidden = true;
  const fl = $('camFlash'); fl.hidden = false; fl.style.animation = 'none'; void fl.offsetWidth; fl.style.animation = '';
  setTimeout(() => { fl.hidden = true; }, 520);
  buzz([40, 30, 80]);
  if (!send({ op: 'selfie.snap' })) toast('Not connected right now', 'error');
  ticker('📸 Got it! Framing your photo…');
  selfie.countdown = false;
}
function onSelfie(m) {
  if (m.stage === 'aiming') { selfie.aiming = true; ticker(m.text || 'Turning to look at you…'); }
  else if (m.stage === 'aimed') {
    selfie.aiming = false; selfie.aimed = true; buzz([20, 40, 20]);
    ticker(m.text || 'Looking at you: strike a pose!');
    if (selfie.after && S.game === 'selfie') { selfie.after = false; countdownAndSnap('selfie'); }
  } else if (m.stage === 'taken') showPhoto(m.id, selfie.kind);
  else if (m.stage === 'error') { selfie.aiming = false; selfie.after = false; toast(m.text || "Couldn't take the photo", 'error'); }
}
// the server refuses the aim while the arm is busy; the busy/refused paths already toast
$('findBtn').addEventListener('click', () => {
  if (S.screen !== 'turn' || !S.started) return;
  releaseAll(); buzz(15); selfie.after = false;
  if (send({ op: 'selfie.aim' })) { selfie.aiming = true; ticker('Turning to look at you…'); }
});
$('selfieBtn').addEventListener('click', () => {
  if (S.screen !== 'turn' || !S.started || selfie.countdown) return;
  buzz(15);
  if (selfie.aimed) return countdownAndSnap('selfie');
  releaseAll(); selfie.after = true;               // aim first, the countdown starts when it's there
  if (send({ op: 'selfie.aim' })) { selfie.aiming = true; ticker('Turning to look at you… then 5-4-3-2-1!'); }
});
$('snapBtn').addEventListener('click', () => countdownAndSnap('eyes'));

function qs() {
  const k = new URLSearchParams(location.search).get('k');
  return `pid=${encodeURIComponent(pid)}` + (k ? `&k=${encodeURIComponent(k)}` : '');
}
async function showPhoto(id, kind) {
  try {
    const r = await fetch(`/selfie/${encodeURIComponent(id)}.jpg?${qs()}`, { cache: 'no-store', credentials: 'same-origin' });
    if (!r.ok) throw new Error(`HTTP ${r.status}`);
    const img = await createImageBitmap(await r.blob());
    const f = kind === 'selfie' ? (S.filterBy.selfie || 'none') : (S.filterBy.eyes || 'none');
    // the whole picture (a wide camera: someone a little off-centre is still in it), then the caption
    const W = 1080, pad = 48, ph = Math.round((W - 2 * pad) * img.height / img.width), H = ph + 2 * pad + 190;
    const out = document.createElement('canvas'); out.width = W; out.height = H;
    const c = out.getContext('2d');
    const g = c.createLinearGradient(0, 0, 0, H); g.addColorStop(0, '#3b1f8f'); g.addColorStop(1, '#120b2e');
    c.fillStyle = g; c.fillRect(0, 0, W, H);
    c.imageSmoothingEnabled = f !== 'pixel';
    fitDraw(c, f === 'none' ? img : filtered(img, f), pad, pad, W - 2 * pad, ph, 'cover');
    c.strokeStyle = '#ffd23f'; c.lineWidth = 8; c.strokeRect(pad, pad, W - 2 * pad, ph);
    c.textAlign = 'center'; c.fillStyle = '#ffd23f'; c.font = '900 62px system-ui';
    c.fillText(kind === 'selfie' ? 'SELFIE WITH THE ROBOT 🤖' : 'SEEN BY THE ROBOT 🤖', W / 2, pad + ph + 100);
    c.fillStyle = '#f4f1ff'; c.font = '700 40px system-ui';
    const when = new Date().toLocaleDateString(undefined, { day: 'numeric', month: 'short', year: 'numeric' });
    c.fillText(`${myName || 'A friend'} · OpenYAM arm · ${when}`, W / 2, pad + ph + 170);
    const blob = await new Promise((ok) => out.toBlob(ok, 'image/jpeg', 0.9));
    if (photo.url) URL.revokeObjectURL(photo.url);
    Object.assign(photo, { id, blob, kind, url: URL.createObjectURL(blob) });
    $('photoImg').src = photo.url;
    $('photoRetake').hidden = kind !== 'selfie';
    const lap = $('photoLaptop'); lap.className = 'photo-saved'; lap.textContent = "⏳ Saving a copy on the robot's laptop…";
    $('photo').hidden = false;
    G.stats.photos++;
    if (G.stats.photos <= 3) award(50, '+50 PHOTO');
    ticker(kind === 'selfie' ? '🤳 Your selfie! Save it to your phone' : '📸 Your photo! Save it to your phone');
    // the laptop keeps the framed copy too (it already has the camera's original)
    const up = await fetch(`/selfie/${encodeURIComponent(id)}/framed?${qs()}`, { method: 'POST', body: blob, credentials: 'same-origin',
      headers: { 'Content-Type': 'image/jpeg' } });
    if (!up.ok) throw Object.assign(new Error(`HTTP ${up.status}`), { upload: true });
    lap.className = 'photo-saved ok'; lap.textContent = "✅ Also saved on the robot's laptop";
  } catch (e) {
    if (e.upload) { const lap = $('photoLaptop'); lap.className = 'photo-saved bad'; lap.textContent = `⚠️ Only the original is on the laptop (${e.message})`; }
    else toast(`Couldn't get your photo (${e.message || e})`, 'error');
  }
}
// Browsers can't write to the photo library on their own: one tap. iPhone: the share sheet's
// "Save Image" puts it in Photos; elsewhere a download (Android: Downloads / Gallery).
const iOS = /iPhone|iPad|iPod/.test(navigator.userAgent || '') || (navigator.platform === 'MacIntel' && navigator.maxTouchPoints > 1);
$('photoHint').textContent = iOS ? 'Tip: in the share sheet, tap “Save Image” to put it in Photos. Or press and hold the photo.'
  : 'Or press and hold the photo, then Download image.';
$('photoSave').addEventListener('click', async () => {
  if (!photo.blob) return;
  const name = `robot-${photo.kind === 'selfie' ? 'selfie' : 'photo'}-${(myName || 'me').replace(/[^A-Za-z0-9]+/g, '-').toLowerCase()}.jpg`;
  const file = new File([photo.blob], name, { type: 'image/jpeg' });
  if (iOS && navigator.canShare && navigator.canShare({ files: [file] })) {
    try { await navigator.share({ files: [file] }); return; } catch (e) { if (e && e.name === 'AbortError') return; }
  }
  const a = document.createElement('a'); a.href = photo.url; a.download = name;
  document.body.append(a); a.click(); a.remove();
  toast('Saved to your phone 📲');
});
$('photoRetake').addEventListener('click', () => { $('photo').hidden = true; countdownAndSnap('selfie'); });
$('photoClose').addEventListener('click', () => { $('photo').hidden = true; });

// ------------------------------------------------------------------ toasts + wake lock
let toastTimer = null;
function toast(text, kind = 'info') {
  if (!text) return;
  const box = $('toasts'); box.replaceChildren();
  const t = el('div', 'toast ' + kind);
  t.append(el('b', '', kind === 'error' ? '⚠️' : '💡'), el('span', '', text));
  t.addEventListener('click', clearToast); box.append(t);
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { t.classList.add('out'); setTimeout(() => t.remove(), 260); }, kind === 'error' ? 4000 : 2400);
}
function clearToast() { clearTimeout(toastTimer); $('toasts').replaceChildren(); }
let wake = null;
async function wakeLock(on) {
  try {
    if (on && !wake && navigator.wakeLock && !document.hidden) { wake = await navigator.wakeLock.request('screen'); wake.onrelease = () => { wake = null; }; }
    if (!on && wake) { await wake.release(); wake = null; }
  } catch (_) { /* not supported or not allowed */ }
}

// ------------------------------------------------------------------ start
renderBest();
show('lobby');
connect();
// debugging / tests / screenshots
window.openplay = { S, G, ctl, send, handle, ws: () => ws, startTurn, spawnStars, toScreen, tip: () => tipNow, setGame, selfie, photo };
