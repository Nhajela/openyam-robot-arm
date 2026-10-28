"""Board sensor contract + legal-move matching (interfaces.chess_vision.sensor). No camera.

    .venv/bin/python -m pytest -q interfaces/tests/test_chess_vision.py
"""
import chess
import pytest

from interfaces.chess_vision.sensor import BoardSensor, FakeBoardSensor, Snapshot, decide, rank_moves


def look(fen, uci, noise=0.0, colour_known=True, seed=7):
    b = chess.Board(fen)
    s = FakeBoardSensor(b, noise=noise, colour_known=colour_known, seed=seed)
    before = s.snapshot()
    if uci:
        s.play(uci)
    return b, rank_moves(b, before, s.snapshot())


CASES = [
    ("quiet pawn", chess.STARTING_FEN, "e2e4"),
    ("knight", chess.STARTING_FEN, "g1f3"),
    ("capture", "rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2", "e4d5"),
    ("castle short", "r3k2r/8/8/8/8/8/8/R3K2R w KQkq - 0 1", "e1g1"),
    ("castle long, black", "r3k2r/8/8/8/8/8/8/R3K2R b KQkq - 0 1", "e8c8"),
    ("en passant", "4k3/8/8/3pP3/8/8/8/4K3 w - d6 0 2", "e5d6"),
]


@pytest.mark.parametrize("what,fen,uci", CASES, ids=[c[0] for c in CASES])
@pytest.mark.parametrize("noise", [0.0, 0.15])
def test_the_move_made_is_seen(what, fen, uci, noise):
    b, ranked = look(fen, uci, noise)
    verdict, cands = decide(ranked)
    assert verdict == "seen" and cands[0][0].uci() == uci, (what, ranked[:3])
    assert all(m in b.legal_moves for m, _ in ranked) and 0 <= ranked[-1][1] <= ranked[0][1] <= 1


def test_capture_without_colour_still_found():
    _, ranked = look(CASES[2][1], "e4d5", colour_known=False)
    assert decide(ranked) == ("seen", [ranked[0]]) and ranked[0][0].uci() == "e4d5"


def test_nothing_moved_is_unseen():
    _, ranked = look(chess.STARTING_FEN, None, noise=0.1)
    assert decide(ranked) == ("unseen", [])


def test_illegal_change_is_unseen():
    b = chess.Board()
    s = FakeBoardSensor(b)
    before = s.snapshot()
    s.world.remove_piece_at(chess.E2)          # a pawn taken off the board, put nowhere
    s.world.set_piece_at(chess.E5, chess.Piece(chess.PAWN, chess.WHITE))  # e2-e5 is not legal
    verdict, _ = decide(rank_moves(b, before, s.snapshot()))
    assert verdict != "seen"


def test_promotion_is_ambiguous_with_all_four_pieces():
    _, ranked = look("8/4P3/8/8/8/8/k7/4K3 w - - 0 1", "e7e8q")
    verdict, cands = decide(ranked)
    assert verdict == "ambiguous" and sorted(m.uci() for m, _ in cands) == ["e7e8b", "e7e8n", "e7e8q", "e7e8r"]


def test_unexplained_changes_lower_the_score():
    b = chess.Board()
    s = FakeBoardSensor(b)
    before = s.snapshot()
    s.play("e2e4")
    after = s.snapshot()
    clean = rank_moves(b, before, after)[0]
    after.occupancy["a7"] = 0.0  # someone knocked a pawn off as well
    after.occupancy["h7"] = 0.0
    messy = rank_moves(b, before, after)[0]
    assert messy[0] == clean[0] and messy[1] < clean[1]


def test_contract_shapes():
    s = FakeBoardSensor()
    assert isinstance(s, BoardSensor)
    snap = s.snapshot()
    assert isinstance(snap, Snapshot) and len(snap.occupancy) == 64 and snap.frames == {}
    assert snap.colour["a1"] == 1.0 and snap.colour["a8"] == 0.0 and snap.colour["e4"] == 0.5
