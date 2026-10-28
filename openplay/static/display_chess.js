// Open play chess: the laptop display's board panel (#modePanel) and the operator's chess
// controls in the rail (PROTOCOL.md "Chess"). display.js calls chessDisplay(m, send) for every
// `chess` message. The panel mirrors the real board: position, last move, both clocks, an eval
// bar (material, or the engine's when there is one), the board cameras' latest frames.
// Keyboard: "c" presses the chess clock for the player to move (a USB/BT button can send "c").
// Space stays E-STOP.

const GLYPH = { K: '♔', Q: '♕', R: '♖', B: '♗', N: '♘', P: '♙', k: '♚', q: '♛', r: '♜', b: '♝', n: '♞', p: '♟' };
const FILES = 'abcdefgh';
const $ = (id) => document.getElementById(id);
const esc = (t) => String(t ?? '').replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const fmt = (s) => { s = Math.max(0, Math.ceil(s || 0)); return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`; };
let last = null, lastAt = 0, sendFn = null;

function parseFen(fen) {
  const out = {};
  String(fen || '').split(' ')[0].split('/').forEach((row, i) => {
    let f = 0;
    for (const ch of row) { if (/\d/.test(ch)) f += +ch; else { out[FILES[f] + (8 - i)] = ch; f++; } }
  });
  return out;
}

function boardHtml(g) {
  const pos = parseFen(g.fen);
  const lastSq = g.last ? [g.last.from, g.last.to] : [];
  let h = '';
  for (let r = 8; r >= 1; r--) {
    for (let f = 0; f < 8; f++) {
      const sq = FILES[f] + r, p = pos[sq];
      const cls = ['dc-sq', (f + r) % 2 ? 'light' : 'dark', lastSq.includes(sq) ? 'last' : ''].join(' ');
      h += `<div class="${cls}">${p ? `<span class="pc ${p === p.toUpperCase() ? 'w' : 'b'}">${GLYPH[p]}</span>` : ''}`
        + (f === 0 ? `<i class="rk">${r}</i>` : '') + (r === 1 ? `<i class="fl">${FILES[f]}</i>` : '') + '</div>';
    }
  }
  return h;
}

function clockLeft(g, c) {
  const k = g.clock || {};
  return k.running === c ? (+k[c] || 0) - (performance.now() - lastAt) / 1000 : +k[c] || 0;
}

function railSection() {
  let sec = $('railChess');
  if (sec) return sec;
  sec = document.createElement('div');
  sec.className = 'rail-sec'; sec.id = 'railChess';
  sec.innerHTML = '<div class="sec-h">Chess</div><div class="rc-state"></div>'
    + '<div class="btn-row"><button class="btn" type="button" data-op="chess.clock">Clock <kbd>c</kbd></button>'
    + '<button class="btn ok" type="button" data-op="chess.fixed">Board fixed</button>'
    + '<button class="btn danger" type="button" data-op="chess.reset">End game</button></div>';
  sec.addEventListener('click', (e) => {
    const b = e.target.closest('button[data-op]');
    if (!b || !sendFn) return;
    if (b.dataset.op === 'chess.reset' && !confirm('End the chess game?')) return;
    sendFn(b.dataset.op, {}, { 'chess.clock': 'Clock pressed', 'chess.fixed': 'Board fixed: play on', 'chess.reset': 'Chess game ended' }[b.dataset.op]);
  });
  const queue = $('railQueue');
  const anchor = queue ? queue.closest('.rail-sec') : null;
  if (anchor) anchor.before(sec); else $('rail').append(sec);
  return sec;
}

window.addEventListener('keydown', (e) => {
  if (e.key !== 'c' || e.repeat || e.ctrlKey || e.metaKey || e.altKey) return;
  if (/^(INPUT|TEXTAREA|SELECT)$/.test((e.target && e.target.tagName) || '')) return;
  const g = last && last.game;
  if (g && g.status === 'playing' && sendFn) sendFn('chess.clock', {}, 'Clock pressed');
});

setInterval(() => {   // tick the running clock between updates
  const g = last && last.game;
  if (!g || !(g.clock || {}).running) return;
  for (const c of ['white', 'black']) { const el = $('dcClock-' + c); if (el) el.textContent = fmt(clockLeft(g, c)); }
}, 250);

export function chessDisplay(m, send) {
  last = m; lastAt = performance.now(); sendFn = send;
  const panel = $('modePanel');
  const g = m.game;
  document.body.classList.toggle('mode-panel', !!g);
  panel.hidden = !g;
  const sec = railSection();
  sec.hidden = !g;
  if (!g) { panel.innerHTML = ''; return; }
  const turnName = g[g.turn] && g[g.turn].name;
  const state = g.status === 'over' ? `Game over: ${esc(g.result || '')} · ${esc(g.reason || '')}`
    : g.status === 'waiting' ? 'Waiting for an opponent to accept on their phone'
    : g.looking ? '👀 Looking at the board…'
    : g.moving ? `🦾 The arm is moving <b>${esc(g.moving)}</b>`
    : g.thinking ? '🤔 The arm is thinking…'
    : (g.candidates || []).length ? `Which move? ${g.candidates.map((c) => esc(c.san)).join(' or ')} (the player picks on their phone)`
    : g.unseen ? `🙈 Couldn't see ${esc(turnName)}'s move: hit the clock again or tap it on the phone`
    : `<b>${esc(turnName)}</b> to move: move a piece, then hit the clock${g.check ? ' · <b class="chk">check!</b>' : ''}`;
  const warns = [];
  if (!m.calibrated) warns.push('Board not calibrated: square positions are placeholders (workcell.json calibrated: false).'
    + (m.arm_ok ? ' OK for the mock/sim only.' : ' Chess is OFF on the real arm.'));
  if (m.refusal && m.calibrated) warns.push(esc(m.refusal));
  if (g.attention) warns.push(`Needs a hand: ${esc(g.attention)}. Then press “Board fixed”.`);
  const sans = g.san || [];
  const moves = sans.map((s, i) => (i % 2 ? '' : `${i / 2 + 1}. `) + esc(s)).join(' ');
  const running = (g.clock || {}).running;
  const face = (c) => `<div class="dc-face ${running === c ? 'run' : ''}"><div class="dc-fn">${c === 'white' ? '♔' : '♚'} ${esc(g[c].name || 'waiting…')}${g[c].engine ? ' 🤖' : ''}</div>`
    + `<div class="dc-ft" id="dcClock-${c}">${fmt(clockLeft(g, c))}</div></div>`;
  const pawns = (g.eval && +g.eval.pawns) || 0;
  const whiteShare = Math.max(0.05, Math.min(0.95, 0.5 + pawns / 20));
  const frames = (m.frames || []).map((n) => `<figure><img src="/chess/frame/${encodeURIComponent(n)}?t=${esc(m.frames_t)}" alt="${esc(n)}"><figcaption>${esc(n)}</figcaption></figure>`).join('');
  const seen = g.last ? { camera: 'seen by the cameras', picked: 'picked on the phone', tap: 'tapped on the phone', arm: 'played by the arm' }[g.last.how] || '' : '';
  panel.innerHTML = `<div class="dc">
    <div class="dc-k">Chess · real board</div>
    <div class="dc-clocks">${face('white')}${face('black')}</div>
    <div class="dc-main">
      <div class="dc-eval" title="material: ${pawns > 0 ? '+' : ''}${pawns}"><i style="height:${(whiteShare * 100).toFixed(1)}%"></i></div>
      <div class="dc-board">${boardHtml(g)}</div>
    </div>
    <div class="dc-state">${state}</div>
    ${g.last ? `<div class="dc-last">${esc(g.last.by)} played <b>${esc(g.last.san)}</b> (${esc(g.last.from)}→${esc(g.last.to)}, ${esc(seen)})</div>` : ''}
    <div class="dc-moves">${moves}</div>
    ${frames ? `<div class="dc-cams">${frames}</div>` : ''}
    ${warns.map((w) => `<div class="dc-warn">⚠️ ${w}</div>`).join('')}
  </div>`;
  sec.querySelector('.rc-state').innerHTML = esc(`${g.white.name || '?'} vs ${g.black.name || '?'} · ${g.status} · sensor ${m.sensor || '?'}`)
    + (g.attention ? `<br><b>${esc(g.attention)}</b>` : '');
  sec.querySelector('[data-op="chess.clock"]').disabled = !(g.status === 'playing' && !g[g.turn].engine);
  sec.querySelector('[data-op="chess.fixed"]').disabled = !g.attention;
  sec.querySelector('[data-op="chess.reset"]').disabled = g.status === 'over';
}
