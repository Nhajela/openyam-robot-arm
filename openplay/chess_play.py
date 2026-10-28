"""openplay.chess_play: chess in open play (the rules side; the server runs the arm).

One game at a time, because there is one real board on the table. A game is created by the
player whose turn it is (``chess.new``), against the arm's engine or against another player in
the room who accepts it (``chess.accept``). Humans move their own pieces by hand and press the
chess clock; the server compares board snapshots (``interfaces.chess_vision.sensor``) to find
which legal move it was (a tap-to-move board is the fallback). The arm plays the engine's
moves: ``plan_move`` turns a move into pick-and-place playables with
``interfaces.grasp.chess_arm.plan`` (captures go to the graveyard row beside the board) and
the server plays them through the same validate -> operator approval -> Play path as every
other playable. The logical board changes only once a move is on the real board.

The table frame and the board placement in ``interfaces/grasp/workcell.json`` are placeholders
until someone calibrates them (HANDOVER.md §5 step 8). ``calibration_refusal`` keeps chess off
the real arm until ``calibrated: true``; the mock and the DimOS simulator may play anyway, and
the display always shows the warning.

    from openplay.chess_play import Game, plan_move, engine_move
    g = Game.new("pid-asha", "Asha", opponent="engine", color="white")
    mv = g.check_move("pid-asha", "e2e4")        # ChessError(reason) if not allowed
    steps = plan_move(g.board, mv, g.captured)    # [{"what", "playable", "duration_s", "caps"}]
    g.apply(mv, how="arm")                        # once it is on the real board
"""
from __future__ import annotations

import time
import uuid
from pathlib import Path
from typing import Any

import chess

from interfaces.grasp import chess_arm
from interfaces.grasp.grasp import WORKCELL, Unreachable, Workcell

ENGINE_NAME = "the arm"
CLOCK_S = 600.0          # each side's clock (shown; nobody loses on time)
PIECE_VALUE = {chess.PAWN: 1, chess.KNIGHT: 3, chess.BISHOP: 3, chess.ROOK: 5, chess.QUEEN: 9}
SIM_ADAPTERS = ("mock", "sim")
COLORS = {"white": chess.WHITE, "black": chess.BLACK}


class ChessError(ValueError):
    """A chess request is refused; ``str(e)`` is shown to the player."""


def cname(color: bool) -> str:
    return "white" if color == chess.WHITE else "black"


# ------------------------------------------------------------------ workcell + calibration
def load_workcell(path: Path | None = None) -> Workcell:
    return Workcell.load(Path(path) if path else WORKCELL)


def calibration_refusal(adapter: str | None, wc: Workcell) -> str | None:
    """Chess on the real arm needs a calibrated table frame. ``adapter`` is the daemon's
    (``mock`` | ``sim`` | ``real`` | None); anything that is not mock or sim counts as real."""
    if adapter in SIM_ADAPTERS or wc.calibrated:
        return None
    return ("chess is off on the real arm until the board is calibrated "
            "(interfaces/grasp/workcell.json has calibrated: false; see HANDOVER.md §5 step 8)")


# ------------------------------------------------------------------ move -> playables
def plan_move(board: chess.Board, move: chess.Move, captured: int = 0, wc: Workcell | None = None,
              caps_order: tuple[str, ...] = ("dance", "default")) -> list[dict]:
    """The pick-and-place playables for a legal ``move`` (before it is pushed).

    -> [{"what", "playable" (None for a by-hand step such as a promotion swap), "duration_s",
    "caps"}]. Each pick and place uses "dance" caps when that is valid and within the
    playable time limit, else "default" (as ``interfaces.grasp.spots`` does). Raises
    ChessError if the move is illegal or a square is out of reach."""
    from armctl import playable as pl

    from interfaces.timeline import coach

    if move not in board.legal_moves:
        raise ChessError(f"{move.uci()} is not a legal move here")
    wc = wc or load_workcell()
    geom = chess_arm.Board.from_workcell(wc)
    by_caps: dict[str, list] = {}

    def planned(caps: str) -> list:
        if caps not in by_caps:
            try:
                by_caps[caps] = chess_arm.plan(wc, geom, board, move, captured, caps)
            except Unreachable as e:
                raise ChessError(f"the arm can't reach that: {e}") from None
        return by_caps[caps]

    steps = []
    for k, (op, _p, _info) in enumerate(planned(caps_order[0])):
        if _p is None:
            steps.append({"what": op["what"], "playable": None, "duration_s": 0.0, "caps": None})
            continue
        chosen = None
        for caps in caps_order:
            p = planned(caps)[k][1]
            r = coach.explain(p)
            if r["ok"] and (r["duration"] or 0) <= pl.MAX_DURATION_S:
                chosen = (p, caps, float(r["duration"]))
                break
        if chosen is None:
            raise ChessError(f"can't plan a safe move for {op['what']}")
        p, caps, dur = chosen
        p["notes"] = f"chess {board.san(move)}: {op['what']}. " + p.get("notes", "")
        steps.append({"what": op["what"], "playable": p, "duration_s": round(dur, 1), "caps": caps})
    return steps


def engine_move(board: chess.Board, think: float = 0.3) -> chess.Move:
    """Stockfish if it's installed, else a one-ply "take the most valuable piece" picker."""
    if board.is_game_over():
        raise ChessError("the game is over")
    return chess_arm.choose(board, think)


# ------------------------------------------------------------------ the game
class Game:
    def __init__(self, owner: str, owner_name: str, opponent: str = "engine", color: str = "white",
                 clock_s: float = CLOCK_S, clock=time.monotonic):
        if opponent not in ("engine", "human"):
            raise ChessError("opponent is 'engine' or 'human'")
        if color not in COLORS:
            raise ChessError("color is 'white' or 'black'")
        self.id = uuid.uuid4().hex[:8]
        self.opponent = opponent
        self.owner = owner
        self.board = chess.Board()
        self.created = time.time()
        mine, theirs = COLORS[color], not COLORS[color]
        self.players: dict[bool, dict[str, Any]] = {
            mine: {"pid": owner, "name": owner_name, "engine": False},
            theirs: ({"pid": None, "name": ENGINE_NAME, "engine": True} if opponent == "engine"
                     else {"pid": None, "name": None, "engine": False}),
        }
        self.status = "playing" if opponent == "engine" else "waiting"
        self.result: str | None = None
        self.reason: str | None = None
        self.captured = 0
        self.last: dict | None = None
        self.sans: list[str] = []
        self.attention: str | None = None   # something on the physical board needs a human
        self.moving: str | None = None      # the arm is playing this move ("Nf3")
        self.thinking = False               # the engine is choosing
        self.looking = False                # reading the board after a clock press
        self.candidates: list[dict] = []    # [{uci, san, score}] when the camera wasn't sure
        self.unseen: str | None = None      # "I couldn't see your move" (tap-to-move fallback)
        self._clock = clock
        self.clock_s = float(clock_s)
        self.used = {chess.WHITE: 0.0, chess.BLACK: 0.0}
        self.running: bool | None = None    # whose clock runs
        self._since = 0.0

    @classmethod
    def new(cls, owner: str, owner_name: str, opponent: str = "engine", color: str = "white", **kw) -> "Game":
        return cls(owner, owner_name, opponent, color, **kw)

    # ---- the chess clock
    def clock_start(self, color: bool) -> None:
        self.clock_stop()
        if not self.over:
            self.running, self._since = color, self._clock()

    def clock_stop(self) -> None:
        if self.running is not None:
            self.used[self.running] += self._clock() - self._since
            self.running = None

    def remaining(self, color: bool) -> float:
        used = self.used[color] + (self._clock() - self._since if self.running == color else 0.0)
        return max(0.0, self.clock_s - used)

    def material(self) -> int:
        """White minus black material in pawns (the eval bar when there's no engine eval)."""
        return sum(v * (len(self.board.pieces(t, chess.WHITE)) - len(self.board.pieces(t, chess.BLACK)))
                   for t, v in PIECE_VALUE.items())

    # ---- who's who
    def pids(self) -> set[str]:
        return {p["pid"] for p in self.players.values() if p["pid"]}

    def color_of(self, pid: str | None) -> bool | None:
        for c, p in self.players.items():
            if pid and p["pid"] == pid:
                return c
        return None

    @property
    def to_move(self) -> dict:
        return self.players[self.board.turn]

    @property
    def engine_to_move(self) -> bool:
        return self.status == "playing" and bool(self.to_move["engine"])

    @property
    def over(self) -> bool:
        return self.status == "over"

    # ---- changes
    def accept(self, pid: str, name: str) -> None:
        if self.status != "waiting":
            raise ChessError("this game is not waiting for an opponent")
        if pid == self.owner:
            raise ChessError("you can't play yourself: ask someone else to accept")
        free = next(c for c, p in self.players.items() if p["pid"] is None)
        self.players[free] = {"pid": pid, "name": name, "engine": False}
        self.status = "playing"

    def check_move(self, pid: str, uci: Any) -> chess.Move:
        """The move ``pid`` asked for, if it may be played now. Raises ChessError(reason)."""
        if self.status == "waiting":
            raise ChessError("waiting for an opponent to accept the game")
        if self.over:
            raise ChessError("the game is over")
        color = self.color_of(pid)
        if color is None:
            raise ChessError("you're not playing in this game")
        if color != self.board.turn:
            raise ChessError(f"it's {cname(self.board.turn)}'s move")
        if self.attention:
            raise ChessError(f"waiting for the board to be fixed: {self.attention}")
        if self.moving:
            raise ChessError(f"the arm is still playing {self.moving}")
        try:
            mv = chess.Move.from_uci(str(uci or "").strip().lower())
        except (ValueError, TypeError):
            raise ChessError(f"{uci!r} is not a move (use from-square + to-square, e.g. e2e4)") from None
        if mv not in self.board.legal_moves:
            piece = self.board.piece_at(mv.from_square)
            if piece is None or piece.color != color:
                raise ChessError(f"illegal move {mv.uci()}: no piece of yours on {chess.square_name(mv.from_square)}")
            options = sorted({chess.square_name(m.to_square) for m in self.board.legal_moves
                              if m.from_square == mv.from_square})
            raise ChessError(f"illegal move {mv.uci()}" + (f" (that piece can go to {', '.join(options)})"
                                                           if options else " (that piece can't move)"))
        return mv

    def apply(self, move: chess.Move, by: str | None = None, how: str = "arm") -> str:
        """Push a legal move that has been made on the real board. ``how``: "camera" (seen),
        "picked" (the player chose among candidates), "tap" (entered by hand), "arm". -> SAN."""
        san = self.board.san(move)
        self.captured += 1 if self.board.is_capture(move) else 0
        self.last = {"uci": move.uci(), "san": san, "from": chess.square_name(move.from_square),
                     "to": chess.square_name(move.to_square), "by": by or self.to_move["name"], "how": how}
        self.candidates, self.unseen = [], None
        self.board.push(move)
        self.sans.append(san)
        outcome = self.board.outcome(claim_draw=True)
        if outcome is not None:
            self.status = "over"
            self.result = outcome.result()
            self.reason = outcome.termination.name.lower().replace("_", " ")
            self.clock_stop()
        return san

    def resign(self, pid: str) -> None:
        color = self.color_of(pid)
        if color is None:
            raise ChessError("you're not playing in this game")
        if self.over:
            raise ChessError("the game is over")
        self.status, self.reason = "over", ("resigned" if self.sans else "cancelled")
        self.result = "0-1" if color == chess.WHITE else "1-0"
        self.clock_stop()

    def end(self, reason: str) -> None:
        if not self.over:
            self.status, self.reason, self.result = "over", reason, "*"
            self.clock_stop()

    # ---- views (PROTOCOL.md "chess")
    def legal(self) -> list[str]:
        return [m.uci() for m in self.board.legal_moves]

    def view(self, for_pid: str | None = None, display: bool = False, can_move: bool = True) -> dict:
        def who(c: bool) -> dict:
            p = self.players[c]
            d = {"name": p["name"], "engine": p["engine"]}
            if display:
                d["pid"] = p["pid"]
            return d

        you = self.color_of(for_pid)
        your_move = (you is not None and self.status == "playing" and you == self.board.turn
                     and not self.attention and not self.moving and not self.looking)
        out = {"id": self.id, "opponent": self.opponent, "status": self.status, "fen": self.board.fen(),
               "turn": cname(self.board.turn), "check": self.board.is_check(),
               "white": who(chess.WHITE), "black": who(chess.BLACK),
               "last": self.last, "san": list(self.sans), "captured": self.captured,
               "moving": self.moving, "thinking": self.thinking, "result": self.result, "reason": self.reason,
               "attention": self.attention, "looking": self.looking, "candidates": list(self.candidates),
               "unseen": self.unseen,
               "clock": {"white": round(self.remaining(chess.WHITE), 1), "black": round(self.remaining(chess.BLACK), 1),
                         "running": cname(self.running) if self.running is not None else None,
                         "clock_s": self.clock_s},
               "eval": {"pawns": self.material(), "source": "material"}}
        if not display:
            out["you"] = cname(you) if you is not None else None
            out["your_move"] = your_move
            out["owner"] = for_pid == self.owner
            out["legal"] = self.legal() if your_move and can_move else []
        return out

    def describe(self) -> str:
        w, b = self.players[chess.WHITE]["name"] or "?", self.players[chess.BLACK]["name"] or "?"
        return f"{w} (white) vs {b} (black)"
