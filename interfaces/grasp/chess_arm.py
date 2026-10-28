"""interfaces.grasp.chess_arm: play chess with the arm (moves pieces on a real board).

    # where can the arm reach? prints an 8x8 map for the board placement in workcell.json
    .venv/bin/python -m interfaces.grasp.chess_arm check
    # plan one move (from the start position or --fen) and print the playables' timing
    .venv/bin/python -m interfaces.grasp.chess_arm move e2e4
    # play it on the armctl daemon (mock: --auto-confirm presses Play for you)
    .venv/bin/python -m interfaces.grasp.chess_arm move e2e4 --play --auto-confirm
    # a game: you move a piece by hand and type it, the arm answers
    .venv/bin/python -m interfaces.grasp.chess_arm game --play

Board geometry is in the table frame (tag 0, see grasp.py): ``center`` of the board, ``square``
size, and ``rank_dir``, the direction from rank 1 to rank 8. The default (``rank_dir`` = pi)
puts white's rank 1 on the far side, so the arm plays black from the near side. Captured pieces go to a row of slots
beside the board. Moves are checked with python-chess; the reply comes from Stockfish if it's
installed (``apt install stockfish``), else from a one-ply "take the most valuable piece" picker.

Special moves: castling moves the king, then the rook; en passant removes the passed pawn;
promotion asks the human to swap the piece (the arm moves the pawn to the last rank).
"""
from __future__ import annotations

import argparse
import asyncio
import math
import shutil
import sys
from dataclasses import dataclass, field

import chess
import numpy as np

from ..timeline import coach
from .grasp import Unreachable, Workcell, pick_place

VALUES = {chess.PAWN: 1, chess.KNIGHT: 3, chess.BISHOP: 3, chess.ROOK: 5, chess.QUEEN: 9, chess.KING: 0}


@dataclass
class Board:
    center: tuple = (0.0, 0.0)          # table frame, m
    square: float = 0.035               # m
    rank_dir: float = math.pi           # rad: direction rank 1 -> rank 8 in the table frame
                                        # (pi: white's rank 1 is the far side, the arm plays black)
    heights: dict = field(default_factory=lambda: {"p": 0.030, "n": 0.040, "b": 0.042, "r": 0.032, "q": 0.050, "k": 0.058})
    widths: dict = field(default_factory=lambda: {"p": 0.018, "n": 0.020, "b": 0.020, "r": 0.020, "q": 0.022, "k": 0.022})
    graveyard_offset: float = 0.06      # m beyond the board's h-file edge

    @classmethod
    def from_workcell(cls, wc: Workcell) -> "Board":
        extra = getattr(wc, "chess", None) or {}
        return cls(**extra)

    def _axes(self):
        r = np.array([math.cos(self.rank_dir), math.sin(self.rank_dir)])  # rank 1 -> 8
        f = np.array([r[1], -r[0]])  # file a -> h: white's right hand, facing along r
        return r, f

    def xy(self, sq: int) -> np.ndarray:
        r, f = self._axes()
        return (np.array(self.center) + (chess.square_rank(sq) - 3.5) * self.square * r
                + (chess.square_file(sq) - 3.5) * self.square * f)

    def grave_xy(self, k: int) -> np.ndarray:
        r, f = self._axes()
        return (np.array(self.center) + (4 * self.square + self.graveyard_offset) * f
                + (3.5 - (k % 8)) * self.square * r + (k // 8) * 0.04 * f)


def piece_dims(board: Board, piece: chess.Piece) -> tuple[float, float]:
    s = piece.symbol().lower()
    return board.heights[s], board.widths[s]


def operations(b: chess.Board, move: chess.Move, captured_so_far: int = 0) -> list[dict]:
    """Physical pick/place steps for a legal move (before it is pushed)."""
    ops = []
    piece = b.piece_at(move.from_square)
    if b.is_en_passant(move):
        victim = chess.square(chess.square_file(move.to_square), chess.square_rank(move.from_square))
        ops.append({"what": f"take en-passant pawn {chess.square_name(victim)}", "from": victim, "to": "grave",
                    "piece": b.piece_at(victim)})
    elif b.is_capture(move):
        ops.append({"what": f"take {b.piece_at(move.to_square).symbol()} on {chess.square_name(move.to_square)}",
                    "from": move.to_square, "to": "grave", "piece": b.piece_at(move.to_square)})
    ops.append({"what": f"{piece.symbol()} {chess.square_name(move.from_square)}-{chess.square_name(move.to_square)}",
                "from": move.from_square, "to": move.to_square, "piece": piece})
    if b.is_castling(move):
        kingside = chess.square_file(move.to_square) > chess.square_file(move.from_square)
        rank = chess.square_rank(move.from_square)
        rf, rt = (chess.square(7, rank), chess.square(5, rank)) if kingside else (chess.square(0, rank), chess.square(3, rank))
        ops.append({"what": f"rook {chess.square_name(rf)}-{chess.square_name(rt)}", "from": rf, "to": rt,
                    "piece": b.piece_at(rf)})
    for k, op in enumerate(ops):
        if op["to"] == "grave":
            op["grave_slot"] = captured_so_far
    return ops


def plan(wc: Workcell, board: Board, b: chess.Board, move: chess.Move, captured_so_far: int = 0, caps: str = "default"):
    """-> list of (op, playable, info). Raises Unreachable with the square that's out of reach."""
    out = []
    for i, op in enumerate(operations(b, move, captured_so_far)):
        h, w = piece_dims(board, op["piece"])
        src = board.xy(op["from"])
        dst = board.grave_xy(op["grave_slot"]) if op["to"] == "grave" else board.xy(op["to"])
        name = f"chess-{move.uci()}-{i + 1}"
        try:
            p, info = pick_place(wc, (*src, 0.0), (*dst, 0.0), height=h, width=w, yaw=board.rank_dir,
                                 name=name, caps=caps)
        except Unreachable as e:
            where = chess.square_name(op["from"]) if op["to"] == "grave" else \
                f"{chess.square_name(op['from'])}->{chess.square_name(op['to'])}"
            raise Unreachable(f"{op['what']} ({where}): {e}") from e
        out.append((op, p, info))
    if move.promotion:
        out.append(({"what": f"PROMOTION: swap the pawn on {chess.square_name(move.to_square)} for a "
                             f"{chess.piece_name(move.promotion)} by hand"}, None, None))
    return out


def check_board(wc: Workcell, board: Board) -> list[str]:
    """8x8 reach map: '.' reachable for the tallest piece, 'X' not, with reasons."""
    from armctl import kin

    tall = max(board.heights.values())
    rows, notes = [], []
    for rank in range(7, -1, -1):
        row = f"{rank + 1} "
        for file in range(8):
            sq = chess.square(file, rank)
            xy = wc.to_base((*board.xy(sq), 0.0))
            ok = True
            for z in (max(wc.travel_z, tall + wc.approach_z), max(wc.min_tip_z, tall * 0.6)):
                r = kin.ik((xy[0], xy[1], z), down=True, check_table=False)
                if r.err_pos > 0.006 or r.err_rot > 0.1:
                    ok = False
                    notes.append(f"{chess.square_name(sq)} at z={z:.2f}: {r.reason}")
                    break
            row += " ." if ok else " X"
        rows.append(row)
    rows.append("   " + " ".join("abcdefgh"))
    near = min((wc.to_base((*board.xy(s), 0.0))[0], chess.square_name(s)) for s in chess.SQUARES)
    far = max((wc.to_base((*board.xy(s), 0.0))[0], chess.square_name(s)) for s in chess.SQUARES)
    rows.append(f"nearest square {near[1]} at x={near[0]:.3f} m, furthest {far[1]} at x={far[0]:.3f} m from the base")
    return rows + notes[:10]


def choose(b: chess.Board, think: float = 0.5) -> chess.Move:
    sf = shutil.which("stockfish")
    if sf:
        import chess.engine
        with chess.engine.SimpleEngine.popen_uci(sf) as eng:
            return eng.play(b, chess.engine.Limit(time=think)).move
    best, score = None, -1e9
    for m in b.legal_moves:
        s = VALUES.get(b.piece_type_at(m.to_square), 0) if b.is_capture(m) else 0
        s += 0.5 if b.gives_check(m) else 0
        s += 0.01 * (hash(m.uci()) % 7)  # vary quiet moves a little
        if s > score:
            best, score = m, s
    return best


async def play_all(steps, url: str, auto_confirm: bool) -> None:
    from ..client import ArmClient

    async with ArmClient(url, program="chess", name="chess arm") as arm:
        print(f"connected (backend {arm.backend}); approve 'chess' in the console", flush=True)
        await arm.acquire()
        for op, p, _ in steps:
            if p is None:
                print(op["what"])
                continue
            print(f"playing: {op['what']} ({p['keyframes'][-1]['t']:.1f} s)", flush=True)
            await arm.play(p, auto_confirm=auto_confirm, timeout=120)


def _print_plan(steps) -> bool:
    ok = True
    for op, p, info in steps:
        if p is None:
            print("  " + op["what"])
            continue
        r = coach.explain(p)
        ok &= r["ok"]
        print(f"  {op['what']}: {r['summary'][0]}")
        for line in r["summary"][1:]:
            print("   " + line)
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check")
    m = sub.add_parser("move")
    m.add_argument("uci", nargs="+", help="moves in order, e.g. e2e4 e7e5; the last one is planned")
    g = sub.add_parser("game")
    g.add_argument("--arm-plays", choices=("black", "white"), default="black")
    for s in (m, g):
        s.add_argument("--fen", default=chess.STARTING_FEN)
        s.add_argument("--play", action="store_true")
        s.add_argument("--auto-confirm", action="store_true", help="press Play for you (mock backend only)")
        s.add_argument("--url", default="ws://127.0.0.1:8091/armctl")
        s.add_argument("--caps", default="default")
    a = ap.parse_args()
    wc = Workcell.load()
    board = Board.from_workcell(wc)
    if not wc.calibrated:
        print("note: workcell.json is not calibrated; positions are placeholders\n")
    if a.cmd == "check":
        print("\n".join(check_board(wc, board)))
        return 0
    b = chess.Board(a.fen)
    captured = 0
    if a.cmd == "move":
        *before, last = a.uci
        for u in before:
            mv = chess.Move.from_uci(u)
            captured += b.is_capture(mv)
            b.push(mv)
        mv = chess.Move.from_uci(last)
        if mv not in b.legal_moves:
            print(f"{last} is not legal here ({b.fen()})")
            return 2
        steps = plan(wc, board, b, mv, captured, a.caps)
        print(f"{b.san(mv)}:")
        ok = _print_plan(steps)
        if a.play and ok:
            asyncio.run(play_all(steps, a.url, a.auto_confirm))
        return 0 if ok else 1
    # game
    arm_color = chess.BLACK if a.arm_plays == "black" else chess.WHITE
    while not b.is_game_over():
        print(b, "\n")
        if b.turn != arm_color:
            u = input("your move (uci, e.g. e2e4; q to quit): ").strip()
            if u == "q":
                return 0
            try:
                mv = chess.Move.from_uci(u)
                assert mv in b.legal_moves
            except Exception:
                print("not a legal move")
                continue
            captured += b.is_capture(mv)
            b.push(mv)
            continue
        mv = choose(b)
        print(f"arm plays {b.san(mv)}")
        steps = plan(wc, board, b, mv, captured, a.caps)
        _print_plan(steps)
        if a.play:
            asyncio.run(play_all(steps, a.url, a.auto_confirm))
        captured += b.is_capture(mv)
        b.push(mv)
    print(b.result())
    return 0


if __name__ == "__main__":
    sys.exit(main())
