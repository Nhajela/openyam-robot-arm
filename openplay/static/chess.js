// Open play chess: the phone side (PROTOCOL.md "Chess"). A real board on the table: you move a
// piece by hand, then hit the big chess-clock button ("I've moved"); the cameras work out which
// move it was. If they're not sure you pick from 2-4 buttons; if they saw nothing, the tap board
// below is the fallback. Self-contained: play.js (or the mode menu) calls
//   openChess(host, {send, toast})   once per container (data-role="turn" | "lobby")
//   chessMessage(m)                  for every `chess` message
// The server checks every move with python-chess.

const GLYPH = { K: '♔', Q: '♕', R: '♖', B: '♗', N: '♘', P: '♙', k: '♚', q: '♛', r: '♜', b: '♝', n: '♞', p: '♟' };
const PROMO = { q: 'Queen', r: 'Rook', b: 'Bishop', n: 'Knight' };
const FILES = 'abcdefgh';
const views = [];
let last = null, lastAt = 0;   // the latest `chess` message and when it arrived
let api = { send: () => false, toast: () => {} };

function mk(tag, cls, text) { const e = document.createElement(tag); if (cls) e.className = cls; if (text != null) e.textContent = text; return e; }
function fmt(s) { s = Math.max(0, Math.ceil(s || 0)); return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`; }
function buzz(ms = 40) { try { if (navigator.vibrate) navigator.vibrate(ms); } catch (_) { /* no haptics */ } }

function parseFen(fen) {
  const out = {};
  String(fen || '').split(' ')[0].split('/').forEach((row, i) => {
    let f = 0;
    for (const ch of row) { if (/\d/.test(ch)) f += +ch; else { out[FILES[f] + (8 - i)] = ch; f++; } }
  });
  return out;
}

// ------------------------------------------------------------------ one view
function makeView(host) {
  const v = { host, sel: null, promo: null, showBoard: false };
  host.classList.add('chess');
  host.replaceChildren();
  v.head = mk('div', 'ch-head');
  v.clocks = mk('div', 'ch-clocks');
  v.faces = { me: mk('div', 'ch-face'), them: mk('div', 'ch-face') };
  v.clocks.append(v.faces.them, v.faces.me);
  v.press = mk('button', 'ch-press', "I've moved");
  v.press.addEventListener('click', () => { buzz(60); api.send({ op: 'chess.clock' }); });
  v.say = mk('div', 'ch-say');
  v.cands = mk('div', 'ch-cands');
  v.warn = mk('div', 'ch-warn'); v.warn.hidden = true;
  v.toggle = mk('button', 'btn tiny ghost ch-toggle', 'Tap your move instead');
  v.toggle.addEventListener('click', () => { v.showBoard = !v.showBoard; render(v); });
  v.boardWrap = mk('div', 'ch-boardwrap');
  v.board = mk('div', 'ch-board');
  v.promoBox = mk('div', 'ch-promo'); v.promoBox.hidden = true;
  v.boardWrap.append(v.board, v.promoBox);
  v.moves = mk('div', 'ch-moves small muted');
  v.actions = mk('div', 'ch-actions');
  host.append(v.head, v.warn, v.clocks, v.press, v.say, v.cands, v.toggle, v.boardWrap, v.moves, v.actions);
  v.board.addEventListener('click', (e) => { const sq = e.target.closest('[data-sq]'); if (sq) tap(v, sq.dataset.sq); });
  return v;
}

function legalFrom(g, from) { return (g.legal || []).filter((u) => u.slice(0, 2) === from); }

function tap(v, sq) {
  const g = last && last.game;
  if (!g || !g.your_move || !(g.legal || []).length) return;
  if (v.sel) {
    const opts = legalFrom(g, v.sel).filter((u) => u.slice(2, 4) === sq);
    if (opts.length === 1) { sendMove('chess.move', opts[0]); return; }
    if (opts.length > 1) { v.promo = opts; return render(v); }   // promotion: pick a piece
  }
  v.sel = legalFrom(g, sq).length ? (v.sel === sq ? null : sq) : null;
  v.promo = null;
  render(v);
}

function sendMove(op, uci) {
  buzz();
  for (const v of views) { v.sel = null; v.promo = null; v.showBoard = false; }
  api.send({ op, uci });
  renderAll();
}

function clockLeft(g, colour) {
  const c = g.clock || {};
  const base = +c[colour] || 0;
  return c.running === colour ? base - (performance.now() - lastAt) / 1000 : base;
}

function render(v) {
  const m = last || {}, g = m.game;
  const turnView = v.host.dataset.role === 'turn';
  const blocked = m.refusal;
  v.warn.hidden = !(blocked || (g && g.attention));
  v.warn.textContent = blocked ? `⚠️ ${blocked}` : g && g.attention ? `⏳ Waiting for the staff: ${g.attention}` : '';
  v.actions.replaceChildren();
  v.cands.replaceChildren();
  const playing = g && g.status === 'playing';
  const mine = g && g.you;
  v.clocks.hidden = !(g && g.status !== 'waiting' && (mine || turnView));
  v.press.hidden = !(playing && mine);
  v.toggle.hidden = !(playing && mine && g.your_move);
  v.say.textContent = '';

  if (!g || g.status === 'over') {
    v.head.textContent = g ? `Game over: ${g.result || ''} (${g.reason || ''})` : 'Play chess with the arm on the real board.';
    if (turnView) {
      v.actions.append(mk('div', 'label', 'New game against'));
      for (const [opp, color, label] of [['engine', 'white', '🤖 The arm (you play white)'], ['engine', 'black', '🤖 The arm (you play black)'], ['human', 'white', '🧑 Another player in the queue']]) {
        const b = mk('button', 'btn', label);
        b.addEventListener('click', () => api.send({ op: 'chess.new', opponent: opp, color }));
        v.actions.append(b);
      }
    }
    v.boardWrap.hidden = !g;
    if (g) drawBoard(v, g);
    v.moves.textContent = g ? (g.san || []).join(' ') : '';
    return;
  }
  const me = g.you || 'white', them = me === 'white' ? 'black' : 'white';
  const who = (c) => (g[c].name || 'waiting…') + (g[c].engine ? ' 🤖' : '');
  for (const [k, c] of [['me', me], ['them', them]]) {
    const f = v.faces[k];
    f.className = 'ch-face' + ((g.clock || {}).running === c ? ' run' : '');
    f.replaceChildren(mk('div', 'ch-fname', (g.you === c ? 'You · ' : '') + who(c) + (c === 'white' ? ' ♔' : ' ♚')), mk('div', 'ch-ftime', fmt(clockLeft(g, c))));
  }
  if (g.status === 'waiting') {
    v.head.textContent = `${who('white')} vs ${who('black')}: waiting for someone to accept`;
    if (!g.you) {
      const b = mk('button', 'btn primary', '♟️ Accept and play');
      b.addEventListener('click', () => api.send({ op: 'chess.accept' }));
      v.actions.append(b);
    }
  } else {
    v.head.textContent = `${who('white')} vs ${who('black')}${g.check ? ' · check!' : ''}`;
  }
  // what's happening, in one line
  let say = '';
  if (g.looking) say = '👀 Looking at the board…';
  else if (g.moving) say = `🦾 The arm is moving: ${g.moving}`;
  else if (g.thinking) say = '🤔 The arm is thinking…';
  else if ((g.candidates || []).length && g.your_move) say = 'Which move did you make?';
  else if (g.unseen && g.your_move) say = `🙈 ${g.unseen}`;
  else if (g.your_move) say = 'Your move: move a piece on the board, then hit the clock.';
  else if (playing) say = `${g[g.turn].name}'s move`;
  if (g.last && !g.looking && !g.moving) say = `Last: ${g.last.by} played ${g.last.san}. ` + say;
  v.say.textContent = say;
  v.press.disabled = !g.your_move;
  if (g.your_move) {
    for (const c of g.candidates || []) {
      const label = c.uci.length === 5 ? `${c.san} (${PROMO[c.uci[4]]})` : c.san;
      const b = mk('button', 'btn primary', label);
      b.addEventListener('click', () => sendMove('chess.pick', c.uci));
      v.cands.append(b);
    }
  }
  const showBoard = (g.your_move && (v.showBoard || !!g.unseen)) || !turnView || !mine;
  v.boardWrap.hidden = !showBoard;
  v.toggle.textContent = v.showBoard ? 'Hide the board' : 'Tap your move instead';
  drawBoard(v, g);
  const sans = g.san || [];
  v.moves.textContent = sans.map((s, i) => (i % 2 ? '' : `${i / 2 + 1}. `) + s).join(' ');
  v.promoBox.hidden = !v.promo;
  v.promoBox.replaceChildren();
  if (v.promo) {
    v.promoBox.append(mk('span', 'small', 'Promote to: '));
    for (const u of v.promo) {
      const b = mk('button', 'btn', PROMO[u[4]] || u);
      b.addEventListener('click', () => sendMove('chess.move', u));
      v.promoBox.append(b);
    }
  }
  if (g.you) {
    const b = mk('button', 'btn tiny ghost', g.status === 'waiting' ? 'Cancel game' : 'Resign');
    b.addEventListener('click', () => { if (confirm('End this game?')) api.send({ op: 'chess.resign' }); });
    v.actions.append(b);
  }
}

function drawBoard(v, g) {
  const pos = parseFen(g.fen);
  const flip = g.you === 'black';
  const targets = new Set(v.sel ? legalFrom(g, v.sel).map((u) => u.slice(2, 4)) : []);
  const movable = new Set((g.legal || []).map((u) => u.slice(0, 2)));
  const lastSq = g.last ? [g.last.from, g.last.to] : [];
  v.board.replaceChildren();
  for (let r = 0; r < 8; r++) {
    for (let f = 0; f < 8; f++) {
      const file = flip ? 7 - f : f, rank = flip ? r + 1 : 8 - r;
      const sq = FILES[file] + rank;
      const d = mk('div', 'ch-sq ' + ((file + rank) % 2 ? 'light' : 'dark'));
      d.dataset.sq = sq;
      if (lastSq.includes(sq)) d.classList.add('last');
      if (v.sel === sq) d.classList.add('sel');
      if (targets.has(sq)) d.classList.add(pos[sq] ? 'hit' : 'dot');
      if (!v.sel && movable.has(sq)) d.classList.add('can');
      const p = pos[sq];
      if (p) d.append(mk('span', 'pc ' + (p === p.toUpperCase() ? 'w' : 'b'), GLYPH[p]));
      if (f === 0) d.append(mk('i', 'rk', String(rank)));
      if (r === 7) d.append(mk('i', 'fl', FILES[file]));
      v.board.append(d);
    }
  }
}

function renderAll() { for (const v of views) render(v); }
setInterval(() => {   // tick the running clock between server updates
  const g = last && last.game;
  if (!g || !(g.clock || {}).running) return;
  for (const v of views) {
    for (const [k, face] of Object.entries(v.faces)) {
      const c = k === 'me' ? (g.you || 'white') : (g.you === 'black' ? 'white' : 'black');
      const t = face.querySelector('.ch-ftime');
      if (t) t.textContent = fmt(clockLeft(g, c));
    }
  }
}, 250);

// ------------------------------------------------------------------ entry points
export function openChess(host, opts = {}) {
  if (opts.send) api.send = opts.send;
  if (opts.toast) api.toast = opts.toast;
  if (host.__chess) return host.__chess;
  const v = makeView(host);
  host.__chess = v;
  views.push(v);
  render(v);
  return v;
}

export function chessMessage(m) {
  const prev = last && last.game;
  last = m; lastAt = performance.now();
  const g = m.game;
  if (!g || !prev || prev.fen !== g.fen || !g.your_move) for (const v of views) { v.sel = null; v.promo = null; }
  if (g && prev && g.last && (!prev.last || prev.last.san !== g.last.san || prev.san.length !== g.san.length) && g.your_move) buzz(80);
  renderAll();
}

// Does this phone have a chess game to show outside its turn (the lobby card)?
export function chessInvolves() {
  const g = last && last.game;
  return !!(g && g.status !== 'over' && (g.you || g.status === 'waiting'));
}
