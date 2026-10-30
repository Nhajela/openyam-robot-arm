// Open play camera modes (the wrist camera on the phone), ported from the arcade page:
//   Robot Eyes  drag to look around (base turn j1, wrist tilt j4), filters, 3-2-1 SNAP, wave
//   Claw Cam    the claw's own view: drag to steer, hold DROP to lower, let go to grab
//   Grab        a view-only live feed above the spot picker
// Self-contained: play.js calls
//   openCam({eyes, clawcam, grab}, api)   once; api = {send, toast, buzz, canDrive, welcome, pid, name}
//   camMode(id)                           whenever the play view shows a mode (null: none)
//   camArm(state)                         for every `arm` message
//   camRelease(why)                       from play.js releaseAll (STOP, view changes, faults)
// Frames come from GET /cam.jpg?pid=&after=<seq> (the server waits up to 1 s for a newer frame and
// serves only the player whose turn it is).
import { tip } from './kin.js';

const RATE_MS = 66;                   // ~15 Hz while dragging or dropping (as play.js)
const THUMB_R = 70;                   // px: a drag this far is full speed
const THUMB_DEAD = 0.12;
const LOOK_K = 0.6;                   // Robot Eyes: full drag = 0.6 x the server's joint speed
const DROP_FLOOR_Z = 0.03;            // m, base frame: DROP stops here (6 cm above the table)
const TABLE_Z = -0.031;               // m, base frame: the table (29 Oct calibration)
const R_MIN = 0.14, R_MAX = 0.42;     // m: soft walls (tip distance from the base axis)
const J1_EDGE = 0.12, J1_DEFAULT = [-1.0, 1.0];
const FILTERS = [['none', '🙂 Normal'], ['robot', '🤖 Robot'], ['thermal', '🔥 Thermal'], ['pixel', '👾 8-bit'], ['comic', '💥 Comic']];

let api = { send: () => false, toast: () => {}, buzz: () => {}, canDrive: () => false, welcome: () => null, pid: () => '', name: () => '' };
let hosts = {};
let mode = null;                      // 'eyes' | 'clawcam' | 'grab' | null
let arm = null, tipNow = null, filter = 'none';
const v = {};                         // elements

const clamp = (x, a = -1, b = 1) => Math.max(a, Math.min(b, x));
const r3 = (x) => Math.round(x * 1000) / 1000;
const dz = (x) => (Math.abs(x) < THUMB_DEAD ? 0 : (x - Math.sign(x) * THUMB_DEAD) / (1 - THUMB_DEAD));
function mk(tag, cls, text) { const e = document.createElement(tag); if (cls) e.className = cls; if (text != null) e.textContent = text; return e; }

export function openCam(h, opts) {
  api = { ...api, ...opts };
  hosts = h;
  // the camera box (moved into whichever mode is showing)
  v.box = mk('div', 'cam-box');
  v.canvas = mk('canvas', 'cam-canvas');
  v.count = mk('div', 'cam-count'); v.count.hidden = true;
  v.flash = mk('div', 'cam-flash'); v.flash.hidden = true;
  v.hint = mk('div', 'cam-hint', 'drag to look around');
  v.box.append(v.canvas, v.hint, v.count, v.flash);
  v.ctx = v.canvas.getContext('2d');

  // Robot Eyes
  v.filters = mk('div', 'cam-filters');
  v.eyesRow = mk('div', 'cam-row');
  v.snap = mk('button', 'btn primary big', '📸 SNAP');
  v.wave = mk('button', 'btn', '👋 Wave');
  v.eyesRow.append(v.snap, v.wave);
  v.snap.addEventListener('click', snap);
  v.wave.addEventListener('click', () => { const m = waveMove(); if (m && api.canDrive()) { camRelease(); api.buzz(15); api.send({ op: 'move', name: m.name }); } });
  hosts.eyes.classList.add('cam-mode');
  hosts.eyes.append(v.filters, v.eyesRow);
  renderFilters();

  // Claw Cam
  v.clawRow = mk('div', 'cam-row');
  v.claw = mk('button', 'btn', '🖐️ Claw open');
  v.drop = mk('div', 'btn hold cam-drop', '⬇ HOLD TO DROP'); v.drop.setAttribute('role', 'button');
  v.clawRow.append(v.claw, v.drop);
  v.claw.addEventListener('click', () => {
    const shut = arm && typeof arm.grip === 'number' && arm.grip < 0.5;
    api.send({ op: 'grip', value: shut ? 1 : 0 }); api.buzz(15);
  });
  hosts.clawcam.classList.add('cam-mode');
  hosts.clawcam.append(v.clawRow);

  // the photo sheet
  v.photo = mk('div', 'cam-photo'); v.photo.hidden = true;
  v.photoImg = mk('img'); v.photoImg.alt = 'your photo, seen by the robot';
  const row = mk('div', 'cam-row');
  v.save = mk('a', 'btn primary', '💾 Save'); v.save.download = 'robot-arm-photo.jpg';
  v.share = mk('button', 'btn', '📤 Share');
  v.close = mk('button', 'btn ghost', 'Close');
  row.append(v.save, v.share, v.close);
  v.photo.append(v.photoImg, row);
  document.body.append(v.photo);
  v.close.addEventListener('click', () => { v.photo.hidden = true; });
  v.share.addEventListener('click', share);

  setupDrag();
  window.addEventListener('resize', resize);
  requestAnimationFrame(draw);
}

// ------------------------------------------------------------------ mode switching
export function camMode(id) {
  const next = id === 'eyes' || id === 'clawcam' || id === 'grab' ? id : null;
  if (next !== mode) camRelease();
  mode = next;
  if (!mode) { camStop(); v.box.remove(); return; }
  const host = hosts[mode];
  host.prepend(v.box);
  v.box.classList.toggle('view-only', mode === 'grab');
  v.hint.hidden = mode === 'grab';
  v.hint.textContent = mode === 'eyes' ? 'drag to look around' : 'drag to steer';
  v.wave.hidden = !waveMove();
  renderClaw();
  camStart();
  requestAnimationFrame(resize);
}
export function camArm(state) {
  arm = state;
  tipNow = arm && Array.isArray(arm.q) && arm.q.length >= 6 ? tip(arm.q) : null;
  renderClaw();
}
function renderClaw() {
  const shut = !!(arm && typeof arm.grip === 'number' && arm.grip < 0.5);
  v.claw.textContent = shut ? '✊ Claw shut' : '🖐️ Claw open';
}

// ------------------------------------------------------------------ frames
const C = { on: false, seq: 0, img: null, w: 0, h: 0, dpr: 1, err: null, gen: 0, fails: 0 };
function resize() {
  const r = v.canvas.getBoundingClientRect(); if (!r.width) return;
  C.dpr = Math.min(2, window.devicePixelRatio || 1);
  C.w = r.width; C.h = r.height;
  v.canvas.width = Math.round(r.width * C.dpr); v.canvas.height = Math.round(r.height * C.dpr);
}
function camUrl() {
  const k = new URLSearchParams(location.search).get('k');
  return `/cam.jpg?pid=${encodeURIComponent(api.pid())}&after=${C.seq}` + (k ? `&k=${encodeURIComponent(k)}` : '');
}
function camStart() { if (C.on) return; C.on = true; C.gen++; C.fails = 0; camLoop(C.gen); }
function camStop() { C.on = false; C.gen++; }
async function camLoop(gen) {
  while (C.on && gen === C.gen) {
    try {
      const r = await fetch(camUrl(), { cache: 'no-store', credentials: 'same-origin' });
      if (!r.ok) throw new Error(r.status === 403 ? 'not your turn' : (await r.text()).slice(0, 120) || `HTTP ${r.status}`);
      C.seq = parseInt(r.headers.get('X-Seq') || '0', 10) || C.seq;
      const img = await createImageBitmap(await r.blob());
      if (gen !== C.gen) break;
      if (C.img && C.img.close) C.img.close();
      C.img = img; C.err = null; C.fails = 0;
    } catch (e) {
      C.err = String(e.message || e); C.fails++;
      await new Promise((ok) => setTimeout(ok, Math.min(3000, 500 * C.fails)));
    }
  }
}

// filters on a small copy (fast on any phone), drawn back scaled
const fx = document.createElement('canvas'), fxc = fx.getContext('2d', { willReadFrequently: true });
const THERMAL = [[0, 0, 40], [60, 0, 120], [180, 0, 120], [255, 60, 0], [255, 180, 0], [255, 255, 160]];
function thermal(l) {
  const x = l / 255 * (THERMAL.length - 1), i = Math.min(THERMAL.length - 2, Math.floor(x)), f = x - i, a = THERMAL[i], b = THERMAL[i + 1];
  return [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f];
}
function filtered(img, f) {
  const sw = f === 'pixel' ? 64 : Math.min(img.width, 400), sh = Math.round(sw * img.height / img.width);
  fx.width = sw; fx.height = sh;
  fxc.imageSmoothingEnabled = true;
  fxc.drawImage(img, 0, 0, sw, sh);
  if (f === 'thermal' || f === 'robot' || f === 'comic') {
    const d = fxc.getImageData(0, 0, sw, sh), p = d.data;
    for (let i = 0; i < p.length; i += 4) {
      const l = 0.3 * p[i] + 0.59 * p[i + 1] + 0.11 * p[i + 2];
      if (f === 'thermal') { const c = thermal(l); p[i] = c[0]; p[i + 1] = c[1]; p[i + 2] = c[2]; }
      else if (f === 'robot') { const y = (i / 4 / sw) | 0, scan = y % 3 === 0 ? 0.55 : 1; p[i] = l * 0.25 * scan; p[i + 1] = Math.min(255, l * 1.15) * scan; p[i + 2] = l * 0.45 * scan; }
      else { const q = l > 170 ? 255 : l > 95 ? 150 : l > 45 ? 70 : 10; p[i] = Math.min(255, q * 1.1); p[i + 1] = q * 0.85; p[i + 2] = q * 0.55 + 20; }
    }
    fxc.putImageData(d, 0, 0);
  }
  return fx;
}
function fitDraw(c, src, x, y, w, h, how) {
  const k = (how === 'cover' ? Math.max : Math.min)(w / src.width, h / src.height);
  const dw = src.width * k, dh = src.height * k;
  c.save(); c.beginPath(); c.rect(x, y, w, h); c.clip();
  c.drawImage(src, x + (w - dw) / 2, y + (h - dh) / 2, dw, dh);
  c.restore();
  return [x + Math.max(0, (w - dw) / 2), y + Math.max(0, (h - dh) / 2), Math.min(w, dw), Math.min(h, dh)];
}
function draw() {
  requestAnimationFrame(draw);
  if (!mode || !C.w) return;
  const c = v.ctx, w = C.w, h = C.h, now = performance.now();
  c.setTransform(C.dpr, 0, 0, C.dpr, 0, 0);
  c.fillStyle = '#000'; c.fillRect(0, 0, w, h);
  let box = [0, 0, w, h];
  if (C.img) {
    const f = mode === 'eyes' ? filter : 'none';
    c.imageSmoothingEnabled = f !== 'pixel';
    box = fitDraw(c, filtered(C.img, f), 0, 0, w, h, 'contain');
  } else {
    c.fillStyle = '#a99fd6'; c.font = '700 15px system-ui'; c.textAlign = 'center';
    c.fillText(C.err ? `📷 ${C.err}` : '📷 Connecting to the camera…', w / 2, h / 2);
  }
  c.textAlign = 'left'; c.font = '800 13px system-ui'; c.lineWidth = 3; c.lineCap = 'round';
  if (mode === 'eyes') {
    c.strokeStyle = filter === 'robot' ? '#4dffa6' : 'rgba(255,255,255,.85)';
    const [bx, by, bw, bh] = box, m = 10, L = 26;
    for (const [x, y, sx, sy] of [[bx + m, by + m, 1, 1], [bx + bw - m, by + m, -1, 1], [bx + m, by + bh - m, 1, -1], [bx + bw - m, by + bh - m, -1, -1]]) {
      c.beginPath(); c.moveTo(x, y + sy * L); c.lineTo(x, y); c.lineTo(x + sx * L, y); c.stroke();
    }
    if (Math.floor(now / 600) % 2) { c.fillStyle = '#ff3b5c'; c.beginPath(); c.arc(bx + m + 18, by + m + 22, 6, 0, 2 * Math.PI); c.fill(); }
    c.fillStyle = '#fff'; c.fillText('ROBOT EYES', bx + m + 30, by + m + 27);
    c.beginPath(); c.arc(w / 2, h / 2, 22, 0, 2 * Math.PI); c.stroke();
  } else if (mode === 'clawcam') {
    c.strokeStyle = ctl.drop ? '#ffd23f' : '#3ee6ff';
    c.beginPath(); c.moveTo(w / 2 - 20, h / 2); c.lineTo(w / 2 + 20, h / 2); c.moveTo(w / 2, h / 2 - 20); c.lineTo(w / 2, h / 2 + 20); c.stroke();
    c.fillStyle = 'rgba(18,11,46,.7)'; c.fillRect(10, h - 38, 170, 26);
    c.fillStyle = '#fff';
    c.fillText(tipNow ? `CLAW CAM · ${Math.max(0, Math.round((tipNow[2] - TABLE_Z) * 100))} cm up` : 'CLAW CAM', 18, h - 20);
  } else {
    c.fillStyle = 'rgba(18,11,46,.7)'; c.fillRect(10, 10, 104, 24);
    c.fillStyle = '#fff'; c.fillText('📷 LIVE', 18, 27);
  }
  if (drag.origin) { // the floating stick under the thumb
    const { x, y } = drag.origin;
    c.strokeStyle = 'rgba(255,255,255,.7)'; c.lineWidth = 2;
    c.beginPath(); c.arc(x, y, THUMB_R * 0.6, 0, 2 * Math.PI); c.stroke();
    c.fillStyle = 'rgba(255,255,255,.85)';
    c.beginPath(); c.arc(x + ctl.sx * THUMB_R * 0.6, y + ctl.sy * THUMB_R * 0.6, 16, 0, 2 * Math.PI); c.fill();
  }
}

// ------------------------------------------------------------------ live control
const ctl = { steer: false, sx: 0, sy: 0, drop: false, floorHit: false };
const drag = { id: null, origin: null };
let timer = null, ep = 0, kind = 'v', endWhy = null;
const grab = { state: null, seen: false };
const driving = () => ctl.steer || ctl.drop;
function j1Range() {
  const w = api.welcome(), l = w && w.limits && w.limits.j1;
  const [lo, hi] = Array.isArray(l) && l.length === 2 ? l : J1_DEFAULT;
  return [Math.max(lo, -1.4) + J1_EDGE, Math.min(hi, 1.4) - J1_EDGE];
}
function steerVel() {
  // thumb offset -> base-frame velocity (as Sticks: up = +x away from the base, right = -y),
  // a softer curve near the centre, soft walls on reach and on the base's turn
  let ux = dz(ctl.sx), uy = dz(ctl.sy);
  const n = Math.hypot(ux, uy);
  if (n > 0) { const k = Math.pow(Math.min(1, n), 1.5) / n; ux *= k; uy *= k; }
  let vx = -uy, vy = -ux;
  if (tipNow) {
    const r = Math.hypot(tipNow[0], tipNow[1]);
    if (r > 1e-3) {
      const ex = tipNow[0] / r, ey = tipNow[1] / r, out = vx * ex + vy * ey;
      if ((r >= R_MAX && out > 0) || (r <= R_MIN && out < 0)) { vx -= out * ex; vy -= out * ey; }
      const [lo, hi] = j1Range(), ang = Math.atan2(tipNow[1], tipNow[0]), tx = -ey, ty = ex, turn = vx * tx + vy * ty;
      if ((ang >= hi && turn > 0) || (ang <= lo && turn < 0)) { vx -= turn * tx; vy -= turn * ty; }
    }
  }
  return { vx, vy };
}
function tick() {
  if (!driving()) { stopStream('up'); return; }
  if (mode === 'eyes') {
    // drag right = turn right (j1 -), drag up = tilt the camera up (j4 +)
    kind = 'qd';
    api.send({ op: 'stream', qd: [r3(clamp(-dz(ctl.sx) * LOOK_K)), 0, 0, r3(clamp(-dz(ctl.sy) * LOOK_K)), 0, 0], ep });
    return;
  }
  kind = 'v';
  const vel = { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 };
  if (ctl.steer) { const s = steerVel(); vel.x = s.vx; vel.y = s.vy; }
  if (ctl.drop) {
    const atFloor = tipNow && tipNow[2] <= DROP_FLOOR_Z;
    if (atFloor && !ctl.floorHit) { ctl.floorHit = true; api.buzz([40, 30, 40]); api.toast('Floor! Let go to grab', 'info', '⬇️'); }
    vel.z = atFloor ? 0 : -1;
  }
  for (const k in vel) vel[k] = r3(clamp(vel[k]));
  api.send({ op: 'stream', v: vel, ep });
}
function startStream() { if (!timer) { ep++; tick(); timer = setInterval(tick, RATE_MS); } }
function stopStream(why) {
  if (!timer) return;
  clearInterval(timer); timer = null;
  // one explicit zero of the same kind, so the arm stops now rather than at the 300 ms deadman
  if (kind === 'qd') api.send({ op: 'stream', qd: [0, 0, 0, 0, 0, 0], ep, why: why || 'ui' });
  else api.send({ op: 'stream', v: { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 }, ep, why: why || 'ui' });
}
export function camRelease(why) {
  ctl.steer = false; ctl.sx = ctl.sy = 0; ctl.drop = false; ctl.floorHit = false;
  drag.id = null; drag.origin = null; grab.state = null;
  v.drop && v.drop.classList.remove('on');
  stopStream(typeof why === 'string' ? why : 'ui');
}
function setupDrag() {
  const c = v.canvas;
  c.addEventListener('pointerdown', (e) => {
    if (mode !== 'eyes' && mode !== 'clawcam') return;
    if (drag.id !== null || !api.canDrive()) return;
    e.preventDefault(); drag.id = e.pointerId;
    try { c.setPointerCapture(e.pointerId); } catch (_) {}
    const r = c.getBoundingClientRect();
    drag.origin = { x: e.clientX - r.left, y: e.clientY - r.top, cx: e.clientX, cy: e.clientY };
    ctl.steer = true; ctl.sx = ctl.sy = 0; api.buzz(10);
    startStream();
  });
  c.addEventListener('pointermove', (e) => {
    if (e.pointerId !== drag.id || !drag.origin) return;
    let dx = (e.clientX - drag.origin.cx) / THUMB_R, dy = (e.clientY - drag.origin.cy) / THUMB_R;
    const n = Math.hypot(dx, dy); if (n > 1) { dx /= n; dy /= n; }
    ctl.sx = dx; ctl.sy = dy;
  });
  const end = (e) => {
    if (e.pointerId !== drag.id) return;
    drag.id = null; drag.origin = null; ctl.steer = false; ctl.sx = ctl.sy = 0;
    if (!driving()) stopStream(e.type === 'pointerup' ? 'up' : 'cancel');
  };
  for (const t of ['pointerup', 'pointercancel', 'lostpointercapture']) c.addEventListener(t, end);
  c.addEventListener('contextmenu', (e) => e.preventDefault());

  // DROP: hold to lower; let go = shut the claw, then back up to the ready pose
  let held = null;
  v.drop.addEventListener('pointerdown', (e) => {
    if (held !== null || mode !== 'clawcam' || !api.canDrive()) return;
    e.preventDefault(); held = e.pointerId;
    try { v.drop.setPointerCapture(e.pointerId); } catch (_) {}
    v.drop.classList.add('on');
    ctl.drop = true; ctl.floorHit = false; api.buzz(20);
    startStream();
  });
  const up = (e) => {
    if (e.pointerId !== held) return;
    held = null; v.drop.classList.remove('on');
    endWhy = e.type === 'pointerup' ? 'up' : 'cancel';
    ctl.drop = false; ctl.steer = false; drag.id = null; drag.origin = null;
    stopStream(endWhy);
    if (endWhy !== 'up') return; // the browser ended the hold: just stop
    grab.state = 'closing';
    api.send({ op: 'grip', value: 0 }); api.buzz([30, 60, 30]);
    setTimeout(() => {
      if (grab.state !== 'closing') return; // STOP or another action came first
      if (mode === 'clawcam' && !driving() && !(arm && arm.busy)) api.send({ op: 'ready' });
      grab.state = null;
    }, 800);
  };
  for (const t of ['pointerup', 'pointercancel', 'lostpointercapture']) v.drop.addEventListener(t, up);
  v.drop.addEventListener('contextmenu', (e) => e.preventDefault());
}

// ------------------------------------------------------------------ filters, wave, snap
function renderFilters() {
  v.filters.replaceChildren();
  for (const [id, label] of FILTERS) {
    const b = mk('button', 'chip', label); b.setAttribute('aria-pressed', String(filter === id));
    b.addEventListener('click', () => { filter = id; api.buzz(10); renderFilters(); });
    v.filters.append(b);
  }
}
function waveMove() {
  const w = api.welcome();
  return ((w && w.moves) || []).find((m) => /wave/i.test(m.name + ' ' + (m.title || '')));
}
let snapping = false;
async function snap() {
  if (snapping) return;
  if (!C.img) { api.toast('No picture from the camera yet', 'info', '📷'); return; }
  snapping = true; camRelease();
  for (const n of ['3', '2', '1']) { v.count.hidden = false; v.count.textContent = n; api.buzz(20); await new Promise((ok) => setTimeout(ok, 650)); }
  v.count.hidden = true;
  v.flash.hidden = false; v.flash.style.animation = 'none'; void v.flash.offsetWidth; v.flash.style.animation = '';
  setTimeout(() => { v.flash.hidden = true; }, 520);
  api.buzz([40, 30, 80]);
  const W = 1080, H = 1350, pad = 48, ph = 1000;
  const out = document.createElement('canvas'); out.width = W; out.height = H;
  const c = out.getContext('2d');
  const g = c.createLinearGradient(0, 0, 0, H); g.addColorStop(0, '#3b1f8f'); g.addColorStop(1, '#120b2e');
  c.fillStyle = g; c.fillRect(0, 0, W, H);
  c.imageSmoothingEnabled = filter !== 'pixel';
  fitDraw(c, filtered(C.img, filter), pad, pad, W - 2 * pad, ph, 'cover');
  c.strokeStyle = '#ffd23f'; c.lineWidth = 8; c.strokeRect(pad, pad, W - 2 * pad, ph);
  c.textAlign = 'center'; c.fillStyle = '#ffd23f'; c.font = '900 64px system-ui';
  c.fillText('SEEN BY THE ROBOT 🤖', W / 2, pad + ph + 100);
  c.fillStyle = '#f4f1ff'; c.font = '700 40px system-ui';
  const when = new Date().toLocaleDateString(undefined, { day: 'numeric', month: 'short', year: 'numeric' });
  c.fillText(`${api.name() || 'A friend'} · OpenYAM arm · ${when}`, W / 2, pad + ph + 170);
  const url = out.toDataURL('image/jpeg', 0.9);
  v.photoImg.src = url; v.save.href = url;
  v.share.hidden = !(navigator.canShare && navigator.share);
  v.photo.hidden = false;
  snapping = false;
}
async function share() {
  try {
    const blob = await (await fetch(v.photoImg.src)).blob();
    const file = new File([blob], 'robot-arm-photo.jpg', { type: 'image/jpeg' });
    if (navigator.canShare && navigator.canShare({ files: [file] })) await navigator.share({ files: [file], title: 'Seen by the robot' });
    else api.toast('Sharing is not available here: use Save', 'info', '📤');
  } catch (_) { /* cancelled */ }
}
