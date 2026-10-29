// Open play "Pick it up": the laptop display's panel (PROTOCOL.md "Pick it up"). display.js calls
// pickDisplay(m, send) for every `pick` and `pick.frame` message. Shows the wrist camera (the
// still the player is choosing on, with the found outline; then the engine's live looks while it
// picks), who is picking, the engine's state and its progress. The operator approves the look and
// the pick in the normal pending card; "Cancel pick" here aborts it (the arm stops).

const $ = (id) => document.getElementById(id);
const esc = (t) => String(t ?? '').replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
let last = null, still = null, live = null, sendFn = null;

function panel() {
  let p = $('pickPanel');
  if (!p) {
    p = document.createElement('section');
    p.id = 'pickPanel'; p.hidden = true;
    (document.querySelector('main') || document.body).append(p);
    if (!document.querySelector('link[href="/static/pick.css"]')) {
      const l = document.createElement('link'); l.rel = 'stylesheet'; l.href = '/static/pick.css'; document.head.append(l);
    }
  }
  return p;
}

export function pickDisplay(m, send) {
  sendFn = send;
  if (m.op === 'pick.frame') {
    if (m.kind === 'still') { still = m; live = null; } else live = m;
  } else if (m.op === 'pick') {
    last = m;
    if (!m.frame) { still = null; live = null; }
    if (m.stage === 'frame' || m.stage === 'target') live = null;
  }
  render();
}

function render() {
  const p = panel();
  const m = last;
  if (!m || m.stage === 'idle') { p.hidden = true; p.innerHTML = ''; return; }
  p.hidden = false;
  const img = live || still;
  const t = m.target;
  let overlay = '';
  if (img === still && t && t.outline && t.outline.length > 2) {
    overlay = `<svg viewBox="0 0 1 1" preserveAspectRatio="none" class="pk-svg">
      <polygon class="pk-outline${t.refusal ? ' bad' : ''}" points="${t.outline.map((q) => q.join(',')).join(' ')}"/>
      ${t.point ? `<circle class="pk-pt" cx="${t.point[0]}" cy="${t.point[1]}" r="0.01"/>` : ''}</svg>`;
  }
  const cands = (m.candidates || []).map((c, i) => `${i + 1}. ${esc(c.label || '?')}`).join(' · ');
  const tinfo = t ? `${esc(t.label)} · ${t.width_mm} mm across · ${t.height_mm} mm tall${t.height_known ? '' : ' (height guessed)'}`
    + ` · table (${t.xy_table[0].toFixed(3)}, ${t.xy_table[1].toFixed(3)}) m` : '';
  const prog = (m.progress || []).map((x) => `<li>${esc(x.text)}</li>`).join('');
  const running = ['moving', 'finding', 'picking'].includes(m.stage);
  p.innerHTML = `<div class="dp-k">Pick it up · wrist camera${img === live ? ' (live)' : ''}</div>
    <div class="dp-who">${esc(m.who || '')}</div>
    ${img ? `<div class="pk-wrap"><img class="pk-img" src="${img.jpeg}" alt="wrist camera">${overlay}</div>` : ''}
    <div class="dp-state${m.error ? ' err' : ''}">${esc(m.error ? '✋ ' + m.error : m.text || m.stage)}</div>
    ${tinfo ? `<div class="dp-row"><span>${tinfo}</span></div>` : ''}
    ${t && t.refusal ? `<div class="dp-row"><span>⚠️ ${esc(t.refusal)}</span></div>` : ''}
    ${cands ? `<div class="dp-row"><span>Claude's candidates: ${cands}</span></div>` : ''}
    ${m.refusal ? `<div class="dp-row"><span>⚠️ ${esc(m.refusal)}</span></div>` : ''}
    ${prog ? `<ol>${prog}</ol>` : ''}
    <div class="dp-row"><span>engine: ${esc(m.engine_state || '–')}</span>
      <button class="btn" data-op="pick.cancel" ${running ? '' : 'disabled'}>Cancel pick</button></div>`;
  const b = p.querySelector('[data-op="pick.cancel"]');
  if (b) b.addEventListener('click', () => sendFn && sendFn('pick.cancel', {}, 'Cancelling the pick'));
}
