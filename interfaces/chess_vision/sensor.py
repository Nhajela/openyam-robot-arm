"""interfaces.chess_vision.sensor: what a camera says about the real chess board, and which
legal move explains a change (CHESS.md §1 layer 2: "I moved" button + before/after compare).

The game never depends on recognising pieces. We know the position (python-chess), so each
human move is "which legal move turns the before snapshot into the after snapshot?". A sensor
reports, per square, how likely it is occupied and how likely the piece there is white.

    sensor = FakeBoardSensor(chess.Board())          # or interfaces.chess_vision.camera.CameraBoardSensor()
    before = sensor.snapshot()
    sensor.play("e2e4")                              # (the fake only: a human moving a piece)
    after = sensor.snapshot()
    rank_moves(board, before, after)[0]              # (Move.from_uci('e2e4'), 0.97)

Shared contract: the camera implementation (``camera.py``, written separately) implements
``BoardSensor``. ``openplay.server`` decides with ``rank_moves``: one confident move is
accepted, a few close ones are offered as buttons, nothing good means "I couldn't see it".
"""
from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable

import chess

SQUARES = [chess.square_name(s) for s in chess.SQUARES]


@dataclass
class Snapshot:
    t: float
    occupancy: dict[str, float]   # square name 'a1'..'h8' -> P(occupied) 0..1
    colour: dict[str, float]      # square -> P(white piece | occupied) 0..1 (may be 0.5 if unknown)
    frames: dict[str, bytes] = field(default_factory=dict)  # camera name -> JPEG, for the display strip


@runtime_checkable
class BoardSensor(Protocol):
    def snapshot(self) -> Snapshot: ...


# ------------------------------------------------------------------ ranking
def _expected(board: chess.Board) -> dict[str, tuple[float, float]]:
    """square -> (occupied 0/1, white 0/1) for a known position."""
    out = {}
    for s in chess.SQUARES:
        p = board.piece_at(s)
        out[chess.square_name(s)] = (1.0, 1.0 if p.color == chess.WHITE else 0.0) if p else (0.0, 0.5)
    return out


def _get(snap: Snapshot, sq: str) -> tuple[float, float]:
    occ = float(snap.occupancy.get(sq, 0.0))
    col = float(snap.colour.get(sq, 0.5))
    return min(max(occ, 0.0), 1.0), min(max(col, 0.0), 1.0)


def _agree(obs: tuple[float, float], exp: tuple[float, float]) -> float:
    """How well an observed square matches an expected one, 0..1."""
    occ, col = obs
    e_occ, e_col = exp
    a = 1.0 - abs(occ - e_occ)
    if e_occ > 0.5:
        # the colour only counts as far as the sensor is sure of it (0.5 = no idea)
        sure = abs(col - 0.5) * 2.0
        a *= 1.0 - sure * abs(col - e_col)
    return a


def change_evidence(before: Snapshot, after: Snapshot, sq: str) -> float:
    """0..1: how much square ``sq`` looks different between the two snapshots."""
    (o0, c0), (o1, c1) = _get(before, sq), _get(after, sq)
    d_col = abs(c1 - c0) * min(o0, o1)
    return min(1.0, max(abs(o1 - o0), d_col))


def rank_moves(board: chess.Board, before: Snapshot, after: Snapshot) -> list[tuple[chess.Move, float]]:
    """Legal moves ranked by how well they explain before->after (score 0..1, best first).

    For a move m, the squares whose contents change (from, to, the captured pawn's square for en
    passant, the rook's squares for castling) must match the position after m in ``after``, and
    the other squares should not have changed between ``before`` and ``after``. Promotions to
    different pieces look the same to occupancy/colour, so they tie (the caller asks)."""
    exp0 = _expected(board)
    change = {sq: change_evidence(before, after, sq) for sq in SQUARES}
    out = []
    for mv in board.legal_moves:
        b = board.copy(stack=False)
        b.push(mv)
        exp1 = _expected(b)
        touched = [sq for sq in SQUARES if exp1[sq] != exp0[sq]]
        if not touched:
            continue
        fit = sum(_agree(_get(after, sq), exp1[sq]) for sq in touched) / len(touched)
        # unexplained changes elsewhere (a knocked piece, a hand in the picture) count against it
        noise = sum(max(0.0, change[sq] - 0.2) for sq in SQUARES if sq not in touched) / 0.8
        score = fit / (1.0 + 0.5 * noise)
        out.append((mv, round(score, 4)))
    out.sort(key=lambda x: (-x[1], x[0].uci()))
    return out


def decide(ranked: list[tuple[chess.Move, float]], sure: float = 0.75, margin: float = 0.15,
           floor: float = 0.45, top: int = 4) -> tuple[str, list[tuple[chess.Move, float]]]:
    """-> ("seen", [best]) | ("ambiguous", candidates) | ("unseen", []).

    Seen: the best move scores >= ``sure`` and beats the runner-up by ``margin``. Ambiguous:
    the moves within ``margin`` of the best that score >= ``floor`` (up to ``top``; promotions
    to different pieces always land here). Unseen: nothing scores >= ``floor``."""
    if not ranked or ranked[0][1] < floor:
        return "unseen", []
    best = ranked[0][1]
    close = [(m, s) for m, s in ranked if s >= floor and best - s < margin]
    if best >= sure and len(close) == 1:
        return "seen", close
    return "ambiguous", close[:top] if len(close) > 1 else ranked[:1]


# ------------------------------------------------------------------ a sensor for tests and the mock/sim
class FakeBoardSensor:
    """Reads a python-chess board standing in for the real one (the "world"). Tests (and a
    rehearsal) move pieces with ``play``; ``sync`` puts the world in a known position, as when
    the arm has made its move. ``noise`` adds uniform noise to every reading; ``colour_known``
    False reports colour 0.5 everywhere (an occupancy-only camera)."""

    def __init__(self, world: chess.Board | None = None, noise: float = 0.0, colour_known: bool = True,
                 seed: int | None = None):
        self.world = world.copy() if world is not None else chess.Board()
        self.noise, self.colour_known = noise, colour_known
        self.rng = random.Random(seed)
        self.t = 0.0

    def play(self, uci: str) -> None:
        self.world.push(chess.Move.from_uci(uci))

    def sync(self, board: chess.Board) -> None:
        self.world = board.copy()

    def _n(self, x: float) -> float:
        return min(1.0, max(0.0, x + (self.rng.uniform(-self.noise, self.noise) if self.noise else 0.0)))

    def snapshot(self) -> Snapshot:
        self.t += 1.0
        occ, col = {}, {}
        for sq, (o, c) in _expected(self.world).items():
            occ[sq] = self._n(o)
            col[sq] = self._n(c) if self.colour_known and o else 0.5
        return Snapshot(t=self.t, occupancy=occ, colour=col)
