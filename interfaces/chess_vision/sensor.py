from __future__ import annotations
from dataclasses import dataclass, field
from typing import Protocol
import chess
@dataclass
class Snapshot:
    t: float
    occupancy: dict[str, float]   # square name 'a1'..'h8' -> P(occupied) 0..1
    colour: dict[str, float]      # square -> P(white piece | occupied) 0..1 (0.5 if unknown)
    frames: dict[str, bytes] = field(default_factory=dict)  # camera name -> JPEG, for the display strip
class BoardSensor(Protocol):
    def snapshot(self) -> Snapshot: ...
def rank_moves(board: chess.Board, before: Snapshot, after: Snapshot) -> list[tuple[chess.Move, float]]:
    """Legal moves ranked by how well they explain before->after (score 0..1, best first)."""
    raise NotImplementedError  # implemented by the game agent; leave this stub as-is
