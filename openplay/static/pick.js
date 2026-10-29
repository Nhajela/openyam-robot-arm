// Open play "Pick it up": the phone side (PROTOCOL.md "Pick it up"). The arm looks down at the
// table with its wrist camera; you tap the thing you want (or type / say "the cup on the table"),
// the phone shows what was found and asks "Pick this?", then the arm homes in and picks it up.
// Self-contained: play.js calls
//   openPick(host, {send, toast})   once (the panel's container)
//   pickMessage(m)                  for every `pick` and `pick.frame` message
// Coordinates sent to the server are 0..1 across the still picture.

let api = { send: () => false, toast: () => {} };
let v = null;                 // the view's elements
let last = null;              // latest `pick` message
let still = null;             // {id, jpeg, w, h}
let tapAt = null;             // where the player last tapped (0..1), shown until the answer

function mk(tag, cls, text) { const e = document.createElement(tag); if (cls) e.className = cls; if (text != null) e.textContent = text; return e; }
function buzz(ms = 30) { try { if (navigator.vibrate) navigator.vibrate(ms); } catch (_) { /* no haptics */ } }
const SVG = 'http://www.w3.org/2000/svg';
function svg(tag, attrs) { const e = document.createElementNS(SVG, tag); for (const k in attrs) e.setAttribute(k, attrs[k]); return e; }

export function openPick(host, opts) {
  api = { ...api, ...opts };
  host.classList.add('pick');
  host.replaceChildren();
  v = {};
  v.status = mk('div', 'pk-status', 'Point the camera at the table first.');
  v.refusal = mk('div', 'pk-refusal'); v.refusal.hidden = true;
  v.look = mk('button', 'btn primary big pk-look', '📷 Look at the table');
  v.look.addEventListener('click', () => { buzz(); tapAt = null; api.send({ op: 'pick.look' }); });
  v.wrap = mk('div', 'pk-wrap'); v.wrap.hidden = true;
  v.img = mk('img', 'pk-img'); v.img.alt = 'the wrist camera looking down at the table';
  v.svg = svg('svg', { class: 'pk-svg', viewBox: '0 0 1 1', preserveAspectRatio: 'none' });
  v.wrap.append(v.img, v.svg);
  v.wrap.addEventListener('click', onTap);
  v.ask = mk('form', 'pk-ask');
  v.text = mk('input', 'pk-text'); v.text.type = 'text'; v.text.maxLength = 200;
  v.text.placeholder = 'or say it: "the cup on the table"'; v.text.enterKeyHint = 'go';
  v.mic = mk('button', 'btn pk-mic', '🎤'); v.mic.type = 'button';
  v.askBtn = mk('button', 'btn pk-askbtn', 'Find it'); v.askBtn.type = 'submit';
  v.ask.append(v.text, v.mic, v.askBtn);
  v.ask.addEventListener('submit', (e) => { e.preventDefault(); ask(v.text.value); });
  setupMic();
  v.confirm = mk('div', 'pk-confirm'); v.confirm.hidden = true;
  v.yes = mk('button', 'btn primary big', '✅ Pick this');
  v.no = mk('button', 'btn ghost', '✖ Not that');
  v.yes.addEventListener('click', () => { buzz(60); if (last && last.target) api.send({ op: 'pick.confirm', target: last.target.id }); });
  v.no.addEventListener('click', () => { buzz(); tapAt = null; render(); api.send({ op: 'pick.cancel' }); });
  v.confirm.append(v.yes, v.no);
  v.progress = mk('ol', 'pk-progress');
  v.again = mk('button', 'btn ghost pk-again', '📷 Look again');
  v.again.addEventListener('click', () => { buzz(); tapAt = null; api.send({ op: 'pick.look' }); });
  host.append(v.refusal, v.status, v.look, v.wrap, v.ask, v.confirm, v.progress, v.again);
  render();
}

function ask(text) {
  text = String(text || '').trim();
  if (!text || !still) return;
  buzz(); tapAt = null;
  api.send({ op: 'pick.ask', frame: still.id, text });
}

function setupMic() {
  const SR = window.SpeechRecognition || window.webkitSpeechRecognition;
  if (!SR) { v.mic.hidden = true; return; }
  let rec = null;
  v.mic.addEventListener('click', () => {
    if (rec) { rec.stop(); return; }
    rec = new SR(); rec.lang = navigator.language || 'en-US'; rec.interimResults = true; rec.maxAlternatives = 1;
    v.mic.classList.add('on');
    rec.onresult = (e) => {
      const r = e.results[e.results.length - 1];
      v.text.value = r[0].transcript;
      if (r.isFinal) ask(v.text.value);
    };
    rec.onend = () => { v.mic.classList.remove('on'); rec = null; };
    rec.onerror = () => { v.mic.classList.remove('on'); rec = null; api.toast('The microphone didn\'t catch that: type it instead', 'error'); };
    try { rec.start(); } catch (_) { rec = null; v.mic.classList.remove('on'); }
  });
}

function onTap(e) {
  if (!still || !last || !['frame', 'target'].includes(last.stage)) return;
  const r = v.img.getBoundingClientRect();
  const x = (e.clientX - r.left) / r.width, y = (e.clientY - r.top) / r.height;
  if (x < 0 || x > 1 || y < 0 || y > 1) return;
  buzz(25);
  tapAt = [x, y];
  api.send({ op: 'pick.tap', frame: still.id, x: +x.toFixed(4), y: +y.toFixed(4) });
  render();
}

export function pickMessage(m) {
  if (m.op === 'pick.frame') {
    if (m.kind === 'still') { still = { id: m.id, jpeg: m.jpeg, w: m.w, h: m.h }; tapAt = null; }
  } else if (m.op === 'pick') {
    last = m;
    if (!m.frame) still = null;
    if (m.stage !== 'finding') tapAt = null;
  }
  if (v) render();
}

function render() {
  const m = last || { stage: 'idle' };
  const st = m.stage;
  v.refusal.hidden = !m.refusal;
  v.refusal.textContent = m.refusal ? `📷 ${m.refusal[0].toUpperCase()}${m.refusal.slice(1)}. Ask the staff.` : '';
  const busy = ['moving', 'finding', 'picking'].includes(st);
  v.look.hidden = !!still || busy;
  v.look.disabled = !!m.refusal;
  v.wrap.hidden = !still;
  if (still && v.img.dataset.id !== still.id) { v.img.src = still.jpeg; v.img.dataset.id = still.id; }
  v.ask.hidden = !still || !['frame', 'target'].includes(st);
  v.confirm.hidden = !(st === 'target' && m.target);
  v.yes.disabled = !!(m.target && m.target.refusal);
  v.again.hidden = !still || busy;
  v.status.className = 'pk-status' + (m.error ? ' err' : '') + (st === 'done' ? ' ok' : '');
  v.status.textContent = m.error ? `✋ ${m.error}` : (m.text || {
    idle: 'Tap Look: the arm points its camera at the table and shows you what it sees.',
    moving: 'Moving the camera over the table…', frame: 'Tap the thing to pick up, or say what it is.',
    finding: 'Looking…', picking: 'Picking it up…' }[st] || '');
  // overlay: the tap, the found outline, Claude's candidates
  v.svg.replaceChildren();
  if (still) {
    const t = m.target;
    if (t && t.outline && t.outline.length > 2 && ['target', 'picking', 'done', 'failed'].includes(st)) {
      v.svg.append(svg('polygon', { points: t.outline.map((p) => p.join(',')).join(' '),
        class: 'pk-outline' + (t.refusal ? ' bad' : '') }));
      if (t.point) v.svg.append(svg('circle', { cx: t.point[0], cy: t.point[1], r: 0.012, class: 'pk-pt' }));
    }
    (m.candidates || []).forEach((c, i) => {
      v.svg.append(svg('circle', { cx: c.x, cy: c.y, r: 0.03, class: 'pk-cand' }));
      const tx = svg('text', { x: c.x, y: c.y + 0.012, class: 'pk-cand-n' }); tx.textContent = String(i + 1);
      v.svg.append(tx);
    });
    if (tapAt) v.svg.append(svg('circle', { cx: tapAt[0], cy: tapAt[1], r: 0.02, class: 'pk-tap' }));
  }
  const prog = st === 'picking' || st === 'done' || st === 'failed' ? (m.progress || []) : [];
  v.progress.replaceChildren(...prog.map((p) => mk('li', '', p.text)));
  v.progress.hidden = !prog.length;
}
