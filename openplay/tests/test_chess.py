"""Chess in open play: rules (openplay.chess_play), move -> pick-and-place plan, the engine,
the calibration refusal, and the server's chess protocol in-process against a fake arm (no
daemon, no ports, no hardware).

    .venv/bin/python -m pytest -q openplay/tests/test_chess.py
"""
from __future__ import annotations

import asyncio
import json

import chess
import pytest

from armctl import playable as pl
from interfaces.grasp import chess_arm
from interfaces.grasp.grasp import WORKCELL, Workcell
from openplay import chess_play as CP
from openplay.server import OpenPlay


# ---------------------------------------------------------------- rules
def game_vs_engine(color="white"):
    return CP.Game.new("pa", "Asha", opponent="engine", color=color)


def test_legal_move_and_apply():
    g = game_vs_engine()
    mv = g.check_move("pa", "e2e4")
    assert mv == chess.Move.from_uci("e2e4")
    assert g.apply(mv, how="tap") == "e4"
    assert g.last == {"uci": "e2e4", "san": "e4", "from": "e2", "to": "e4", "by": "Asha", "how": "tap"}
    assert g.board.turn == chess.BLACK and g.engine_to_move
    v = g.view("pa")
    assert v["you"] == "white" and v["your_move"] is False and v["legal"] == []
    assert v["black"] == {"name": "the arm", "engine": True}


@pytest.mark.parametrize("pid,uci,why", [
    ("pa", "e2e5", "illegal move e2e5 (that piece can go to e3, e4)"),
    ("pa", "e7e5", "no piece of yours on e7"),
    ("pa", "a1a3", "that piece can't move"),
    ("pa", "zz", "is not a move"),
    ("pb", "e2e4", "not playing in this game"),
])
def test_illegal_moves_refused(pid, uci, why):
    g = game_vs_engine()
    with pytest.raises(CP.ChessError, match=why.replace("(", r"\(").replace(")", r"\)")):
        g.check_move(pid, uci)
    assert g.board.fen() == chess.STARTING_FEN


def test_wrong_turn_attention_and_over():
    g = game_vs_engine("black")
    with pytest.raises(CP.ChessError, match="white's move"):
        g.check_move("pa", "e7e5")
    g.apply(chess.Move.from_uci("e2e4"))
    g.attention = "set the pieces up"
    with pytest.raises(CP.ChessError, match="board to be fixed"):
        g.check_move("pa", "e7e5")
    g.attention = None
    g.resign("pa")
    assert g.over and g.result == "1-0"
    with pytest.raises(CP.ChessError, match="over"):
        g.check_move("pa", "e7e5")


def test_checkmate_ends_the_game():
    g = CP.Game.new("pa", "Asha", opponent="human")
    g.accept("pb", "Ben")
    for pid, u in [("pa", "f2f3"), ("pb", "e7e5"), ("pa", "g2g4"), ("pb", "d8h4")]:
        g.apply(g.check_move(pid, u))
    assert g.over and g.result == "0-1" and g.reason == "checkmate" and g.sans[-1] == "Qh4#"


def test_human_game_needs_an_opponent():
    g = CP.Game.new("pa", "Asha", opponent="human", color="black")
    assert g.status == "waiting"
    with pytest.raises(CP.ChessError, match="waiting for an opponent"):
        g.check_move("pa", "e7e5")
    with pytest.raises(CP.ChessError, match="play yourself"):
        g.accept("pa", "Asha")
    g.accept("pb", "Ben")
    assert g.players[chess.WHITE]["pid"] == "pb" and g.status == "playing"
    assert g.view("pb")["legal"] and g.view("pa")["legal"] == []


def test_engine_reply_is_legal_and_greedy(monkeypatch):
    monkeypatch.setattr(chess_arm.shutil, "which", lambda name: None)  # no Stockfish: the 1-ply picker
    b = chess.Board("4k3/8/8/3q4/4P3/8/8/4K3 w - - 0 1")  # e4 takes the queen on d5
    assert CP.engine_move(b) == chess.Move.from_uci("e4d5")
    b = chess.Board()
    for _ in range(6):
        mv = CP.engine_move(b)
        assert mv in b.legal_moves
        b.push(mv)


# ---------------------------------------------------------------- move -> plan
def _valid(p):
    r = pl.validate(p)
    assert r["ok"], r.get("violations")
    return r


def test_plan_quiet_move():
    b = chess.Board()
    steps = CP.plan_move(b, chess.Move.from_uci("e2e4"))
    assert [s["what"] for s in steps] == ["P e2-e4"]
    assert steps[0]["caps"] == "dance" and 0 < steps[0]["duration_s"] <= pl.MAX_DURATION_S
    _valid(steps[0]["playable"])
    assert b.fen() == chess.STARTING_FEN  # planning never changes the board


def test_plan_capture_goes_to_the_graveyard():
    b = chess.Board()
    for u in ("e2e4", "d7d5"):
        b.push_uci(u)
    steps = CP.plan_move(b, chess.Move.from_uci("e4d5"), captured=3)
    assert [s["what"] for s in steps] == ["take p on d5", "P e4-d5"]
    for s in steps:
        _valid(s["playable"])
    wc = Workcell.load()
    geom = chess_arm.Board.from_workcell(wc)
    # the first pick-and-place drops the black pawn in graveyard slot 3 (base-frame place xy)
    ops = chess_arm.operations(b, chess.Move.from_uci("e4d5"), 3)
    assert ops[0]["to"] == "grave" and ops[0]["grave_slot"] == 3
    assert geom.grave_xy(3) is not None


def test_plan_castling_and_promotion():
    b = chess.Board("r3k2r/8/8/8/8/8/8/R3K2R w KQkq - 0 1")
    steps = CP.plan_move(b, chess.Move.from_uci("e1g1"))
    assert [s["what"] for s in steps] == ["K e1-g1", "rook h1-f1"]
    b = chess.Board("8/4P3/8/8/8/8/k7/4K3 w - - 0 1")
    steps = CP.plan_move(b, chess.Move.from_uci("e7e8q"))
    assert steps[-1]["playable"] is None and "PROMOTION" in steps[-1]["what"]


def test_plan_refuses_illegal_and_unreachable(tmp_path):
    with pytest.raises(CP.ChessError, match="not a legal move"):
        CP.plan_move(chess.Board(), chess.Move.from_uci("e2e5"))
    raw = json.loads(WORKCELL.read_text())
    raw["chess"] = {"center": [0.5, 0.0], "square": 0.05}  # far out of reach
    f = tmp_path / "far.json"
    f.write_text(json.dumps(raw))
    with pytest.raises(CP.ChessError, match="can't reach"):
        CP.plan_move(chess.Board(), chess.Move.from_uci("a2a4"), wc=Workcell.load(f))


# ---------------------------------------------------------------- calibration
def test_calibration_refusal():
    wc = Workcell(calibrated=False)
    assert CP.calibration_refusal("mock", wc) is None
    assert CP.calibration_refusal("sim", wc) is None
    for adapter in ("real", None, "something-new"):
        assert "calibrated" in CP.calibration_refusal(adapter, wc)
    assert CP.calibration_refusal("real", Workcell(calibrated=True)) is None


# ---------------------------------------------------------------- server protocol (in-process)
class FakeArm:
    """Stands in for openplay.arm.ArmBridge: validates with armctl.playable, 'plays' instantly."""

    def __init__(self, adapter="mock"):
        self._adapter = adapter
        self.state = {"q": [0.0] * 6, "grip": 0.0, "moving": False}
        self.hello = {"poses": {"ready": [0.0] * 6}}
        self.played: list[dict] = []
        self.connected = self.ready = self.has_lease = True
        self.estop = False
        self.fail_after: int | None = None

    backend = property(lambda self: "mock" if self._adapter == "mock" else "dimos")
    adapter = property(lambda self: self._adapter)

    def unavailable_reason(self):
        return None

    def speeds(self):
        return {"lin_mps": 0.08, "ang_rps": 0.6, "joint_rps": 0.5}

    async def validate(self, p):
        return pl.validate(p)

    async def play(self, p, confirm, timeout=90.0):
        from openplay.arm import Refused

        assert confirm is True
        if self.fail_after is not None and len(self.played) >= self.fail_after:
            raise Refused("interrupted by stop")
        self.played.append(p)
        await asyncio.sleep(0)
        return {"op": "done"}

    async def stop(self):
        pass

    async def do(self, cmd, **kw):
        return {"op": "done"}


class Conn:
    def __init__(self):
        self.msgs: list[dict] = []
        self.pid = None
        self.alive = True
        self.last_stream_refusal = 0.0

    def send(self, m):
        self.msgs.append(m)

    def last(self, op):
        return next((m for m in reversed(self.msgs) if m.get("op") == op), None)

    def refusals(self):
        return [m["reason"] for m in self.msgs if m.get("op") == "refused"]


async def station(adapter="mock", auto_play="on", workcell=None, players=("Asha", "Ben"), chess_sensor="fake"):
    op = OpenPlay(events_log=None, auto_play=auto_play, workcell=workcell, chess_sensor=chess_sensor)
    op.arm = FakeArm(adapter)
    disp = Conn()
    op.display_joined(disp)
    phones = []
    for name in players:
        c = Conn()
        await op.phone_msg(c, {"op": "hello", "pid": "p-" + name.lower(), "name": name})
        await op.phone_msg(c, {"op": "join"})
        phones.append(c)
    op._lobby_events(op.lobby.tick(can_start=True))
    assert op.lobby.active.pid == "p-" + players[0].lower()
    return op, disp, phones


async def settle(op, timeout=20.0):
    for _ in range(int(timeout / 0.02)):
        if not op.busy:
            return
        await asyncio.sleep(0.02)
    raise AssertionError(f"still busy: {op.busy}")




async def until(pred, timeout=20.0):
    for _ in range(int(timeout / 0.02)):
        if pred():
            return
        await asyncio.sleep(0.02)
    raise AssertionError("condition not met")


@pytest.fixture(autouse=True)
def greedy_engine(monkeypatch):
    monkeypatch.setattr(chess_arm.shutil, "which", lambda name: None)  # deterministic-ish: no Stockfish


def test_server_clock_press_sees_the_move_and_the_arm_replies():
    async def main():
        op, disp, (a, b) = await station()
        assert a.last("welcome")["features"]["chess"] is True
        await op.phone_msg(a, {"op": "chess.new", "opponent": "engine", "color": "white"})
        g = a.last("chess")["game"]
        assert g["you"] == "white" and g["your_move"] and "e2e4" in g["legal"]
        assert g["clock"]["running"] == "white" and op.chess_ref is not None
        d = disp.last("chess")
        assert d["calibrated"] is False and d["arm_ok"] is True and d["sensor"] == "fake"
        # not in the game
        await op.phone_msg(b, {"op": "chess.clock"})
        assert b.refusals()[-1] == "you're not playing in this game"
        # Asha moves a piece on the real board, then presses the clock
        op.chess_sensor.play("g1f3")
        await op.phone_msg(a, {"op": "chess.clock"})
        assert op.chess.sans[0] == "Nf3" and op.chess.last["how"] == "camera"
        assert any(m.get("op") == "status" and m["text"] == "You played Nf3" for m in a.msgs)
        assert any(m.get("op") == "event" and m["text"] == "Asha played Nf3" for m in disp.msgs)
        assert op.busy and op.busy["kind"] == "chess"          # the arm's reply
        await op.phone_msg(a, {"op": "chess.clock"})
        assert a.refusals()[-1] == "it's the arm's move"
        await settle(op)
        game = op.chess
        assert len(game.sans) == 2 and game.last["by"] == "the arm" and game.last["how"] == "arm"
        assert len(op.arm.played) == 1 and op.arm.played[0]["name"].startswith("chess-")
        assert game.running == chess.WHITE and game.remaining(chess.BLACK) < game.clock_s  # the arm's clock ran
        v = a.last("chess")["game"]
        assert v["your_move"] and v["legal"] and v["clock"]["running"] == "white"
        dv = disp.last("chess")["game"]
        assert dv["last"]["by"] == "the arm" and dv["white"]["pid"] == "p-asha" and "pawns" in dv["eval"]
        # the fake world now has the arm's move, so the next clock press sees the next move
        op.chess_sensor.play("e2e4" if game.board.piece_at(chess.E2) else "d2d4")
        await op.phone_msg(a, {"op": "chess.clock"})
        assert len(op.chess.sans) == 3
        await settle(op)
    asyncio.run(main())


def test_server_unseen_then_tap_fallback():
    async def main():
        op, disp, (a, _b) = await station()
        await op.phone_msg(a, {"op": "chess.new", "opponent": "engine"})
        await op.phone_msg(a, {"op": "chess.clock"})  # nothing moved on the board
        g = op.chess
        assert g.unseen and "couldn't see" in g.unseen and g.sans == [] and g.running == chess.WHITE
        assert a.last("chess")["game"]["unseen"] and not op.arm.played
        await op.phone_msg(a, {"op": "chess.move", "uci": "e2e5"})   # illegal: refused, nothing changes
        assert "illegal move e2e5" in a.refusals()[-1] and g.sans == []
        await op.phone_msg(a, {"op": "chess.move", "uci": "e2e4"})   # the fallback: tap the move
        assert g.sans[0] == "e4" and g.last["how"] == "tap" and g.unseen is None
        await settle(op)
        assert len(g.sans) == 2 and len(op.arm.played) >= 1  # only the arm's own move uses the arm
    asyncio.run(main())


def test_server_ambiguous_promotion_asks():
    async def main():
        op, disp, (a, b) = await station()
        op.chess = CP.Game.new("p-asha", "Asha", opponent="human")
        op.chess.accept("p-ben", "Ben")
        op.chess.board = chess.Board("8/4P3/8/8/8/8/k7/4K3 w - - 0 1")
        op._chess_sync()
        op.chess_ref = op.chess_sensor.snapshot()
        op.chess_sensor.play("e7e8q")
        await op.phone_msg(a, {"op": "chess.clock"})
        cands = a.last("chess")["game"]["candidates"]
        assert sorted(c["uci"] for c in cands) == ["e7e8b", "e7e8n", "e7e8q", "e7e8r"] and op.chess.sans == []
        await op.phone_msg(a, {"op": "chess.pick", "uci": "e2e4"})
        assert a.refusals()[-1] == "pick one of the moves shown"
        await op.phone_msg(a, {"op": "chess.pick", "uci": "e7e8q"})
        assert op.chess.sans == ["e8=Q"] and op.chess.last["how"] == "picked" and not op.arm.played
        assert op.chess.running == chess.BLACK  # Ben's clock (a human game: no arm)
    asyncio.run(main())


def test_server_engine_moves_first_when_player_is_black():
    async def main():
        op, disp, (a, _b) = await station()
        await op.phone_msg(a, {"op": "chess.new", "opponent": "engine", "color": "black"})
        await settle(op)
        assert len(op.chess.sans) == 1 and op.chess.board.turn == chess.BLACK
        assert a.last("chess")["game"]["your_move"] and op.chess.running == chess.BLACK
    asyncio.run(main())


CAPTURE_FEN = "rnbqkbnr/pppp1ppp/8/4p3/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 2"  # after d4, the arm takes: exd4


def test_server_arm_capture_one_approval():
    async def main():
        op, disp, (a, _b) = await station(auto_play="off")
        await op.phone_msg(a, {"op": "chess.new", "opponent": "engine"})
        op.chess.board = chess.Board(CAPTURE_FEN)
        op._chess_sync()
        op.chess_ref = op.chess_sensor.snapshot()
        op.chess_sensor.play("d2d4")
        await op.phone_msg(a, {"op": "chess.clock"})
        await until(lambda: op.pending)
        pend = disp.last("pending")["pending"]
        assert pend["title"] == "Chess: the arm plays exd4" and pend["duration_s"] > 0 and pend["keyframes"]
        assert not op.arm.played and op.chess.board.piece_at(chess.D4).symbol() == "P"
        await op.display_msg(disp, {"op": "approve", "id": pend["id"]})
        await settle(op)
        assert len(op.arm.played) == 2  # take the pawn to the graveyard, then move: one approval
        assert len([m for m in disp.msgs if m.get("op") == "pending" and m["pending"]]) == 1
        assert op.chess.sans[-1] == "exd4" and op.chess.captured == 1 and op.chess.running == chess.WHITE
    asyncio.run(main())


def test_server_denied_or_stopped_arm_move_needs_the_operator():
    async def main():
        op, disp, (a, _b) = await station(auto_play="off")
        await op.phone_msg(a, {"op": "chess.new", "opponent": "engine"})
        op.chess.board = chess.Board(CAPTURE_FEN)
        op._chess_sync()
        op.chess_ref = op.chess_sensor.snapshot()
        await op.phone_msg(a, {"op": "chess.move", "uci": "d2d4"})
        await until(lambda: op.pending)
        await op.display_msg(disp, {"op": "deny", "id": op.pending["id"]})
        await settle(op)
        g = op.chess
        assert g.sans[-1] == "d4" and "didn't start" in g.attention and not op.arm.played
        # retry after "Board fixed", auto-play on, and stop it half-way
        op.auto_play_mode = "on"
        op.arm.fail_after = 1
        await op.display_msg(disp, {"op": "chess.fixed"})
        await settle(op)
        assert g.sans[-1] == "exd4" and "finish e5d4 by hand" in g.attention
        assert disp.last("chess")["game"]["attention"]
        await op.phone_msg(a, {"op": "chess.clock"})
        assert "board to be fixed" in a.refusals()[-1]
        await op.display_msg(disp, {"op": "chess.fixed"})
        assert g.attention is None and g.running == chess.WHITE and op.chess_ref is not None
        # the operator can press the clock for the player (laptop key / USB button)
        op.chess_sensor.play("d1d4")
        await op.display_msg(disp, {"op": "chess.clock"})
        assert g.sans[-1] == "Qxd4"
        await settle(op)
    asyncio.run(main())


def test_server_human_game_and_turn_rules():
    async def main():
        op, disp, (a, b) = await station()
        await op.phone_msg(b, {"op": "chess.new", "opponent": "human"})
        assert b.refusals()[-1] == "not your turn"
        await op.phone_msg(a, {"op": "chess.new", "opponent": "human", "color": "black"})
        assert b.last("chess")["game"]["status"] == "waiting" and b.last("chess")["game"]["you"] is None
        await op.phone_msg(b, {"op": "chess.accept"})
        assert op.chess.players[chess.WHITE]["name"] == "Ben" and op.chess.running == chess.WHITE
        await op.phone_msg(a, {"op": "chess.clock"})
        assert a.refusals()[-1] == "it's Ben's move"
        op.chess_sensor.play("e2e4")
        await op.phone_msg(b, {"op": "chess.clock"})  # in the game, not the active player
        assert op.chess.sans == ["e4"] and not op.arm.played and op.chess.running == chess.BLACK
        await op.phone_msg(a, {"op": "chess.new", "opponent": "engine"})
        assert "finish or resign" in a.refusals()[-1]
        await op.phone_msg(a, {"op": "chess.resign"})
        assert op.chess.over and op.chess.result == "1-0" and op.chess.running is None
        await op.phone_msg(a, {"op": "chess.new", "opponent": "engine"})
        assert op.chess.attention == "set the pieces up in the start position"
        await op.phone_msg(a, {"op": "chess.clock"})
        assert "board to be fixed" in a.refusals()[-1]
    asyncio.run(main())


def test_server_camera_sensor_missing_refuses_clearly():
    async def main():
        op, disp, (a, _b) = await station(chess_sensor="camera")
        await op.phone_msg(a, {"op": "chess.new", "opponent": "engine"})
        return op, a

    op, a = asyncio.run(main())
    if op.chess_sensor is None:  # interfaces/chess_vision/camera.py not there (or it failed to start)
        assert op.chess is None and "chess camera mode is unavailable" in a.refusals()[-1]
        assert a.last("chess")["arm_ok"] is False and a.last("chess")["sensor"] == "camera"


def test_server_refuses_chess_on_an_uncalibrated_real_arm(tmp_path):
    async def main(workcell, adapter):
        op, disp, (a, _b) = await station(adapter=adapter, auto_play="off", workcell=workcell)
        await op.phone_msg(a, {"op": "chess.new", "opponent": "engine", "color": "white"})
        return op, disp, a

    for adapter in ("real", None):  # an unknown adapter on a non-mock backend counts as real
        op, disp, a = asyncio.run(main(None, adapter))
        assert op.chess is None and "calibrated" in a.refusals()[-1]
        assert a.last("chess")["arm_ok"] is False and "calibrated" in a.last("chess")["refusal"]

    async def engine_on_real():  # a game that exists anyway: the arm still won't move
        op, disp, (a, _b) = await station(adapter="real", auto_play="on")
        op.chess = CP.Game.new("p-asha", "Asha", opponent="engine")
        op._chess_sync()
        await op.phone_msg(a, {"op": "chess.move", "uci": "e2e4"})
        await settle(op)
        assert not op.arm.played and op.chess.sans == ["e4"] and "calibrated" in op.chess.attention
    asyncio.run(engine_on_real())

    # the sim may play on the placeholder board; a calibrated workcell lets the real arm play
    op, _d, a = asyncio.run(main(None, "sim"))
    assert op.chess is not None and not a.refusals()
    raw = json.loads(WORKCELL.read_text())
    raw["calibrated"] = True
    f = tmp_path / "workcell.json"
    f.write_text(json.dumps(raw))
    op, _d, a = asyncio.run(main(f, "real"))
    assert op.chess is not None and not a.refusals() and a.last("chess")["calibrated"] is True
