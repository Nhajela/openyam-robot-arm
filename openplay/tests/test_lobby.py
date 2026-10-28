"""Unit tests for openplay.lobby (pure queue/turn rules, fake clock)."""
from __future__ import annotations

import pytest

from openplay.lobby import Lobby


class Clock:
    def __init__(self) -> None:
        self.t = 1000.0

    def __call__(self) -> float:
        return self.t

    def advance(self, s: float) -> None:
        self.t += s


@pytest.fixture
def clk() -> Clock:
    return Clock()


def mk(clk: Clock, **kw) -> Lobby:
    kw.setdefault("turn_s", 100.0)
    return Lobby(clock=clk, **kw)


def kinds(evs):
    return [(e["kind"], e["pid"], e.get("reason")) for e in evs]


def players(lb: Lobby, *pids: str) -> None:
    for p in pids:
        lb.hello(p, p.upper())
        assert lb.join(p) is None


def test_first_join_starts_turn_and_order(clk):
    lb = mk(clk)
    players(lb, "a", "b", "c")
    assert kinds(lb.tick()) == [("turn_start", "a", None)]
    assert lb.is_active("a") and lb.queue == ["b", "c"]
    assert lb.position("b") == 1 and lb.position("c") == 2 and lb.position("a") is None
    assert lb.tick() == []


def test_join_requires_hello_and_is_idempotent(clk):
    lb = mk(clk)
    assert lb.join("x") == "say hello first"
    players(lb, "a")
    assert lb.join("a") is None and lb.queue == ["a"]
    lb.tick()
    assert lb.join("a") is None and lb.queue == []  # active players don't re-queue


def test_time_out_hands_over_next_tick(clk):
    lb = mk(clk)
    players(lb, "a", "b")
    lb.tick()
    clk.advance(99.0)
    lb.touch("a")
    assert lb.tick() == [] or all(e["kind"] == "warning" for e in lb.tick())
    clk.advance(1.0)
    evs = lb.tick()
    assert kinds(evs) == [("turn_end", "a", "time")]  # never in the same tick
    assert lb.active is None
    assert kinds(lb.tick()) == [("turn_start", "b", None)]
    assert lb.queue == []  # the finished player is not re-queued


def test_can_start_false_holds_next(clk):
    lb = mk(clk)
    players(lb, "a", "b")
    lb.tick()
    lb.skip()
    assert lb.tick(can_start=False) == []
    assert kinds(lb.tick()) == [("turn_start", "b", None)]


def test_warning_once_at_30s(clk):
    lb = mk(clk)
    players(lb, "a")
    lb.tick()
    clk.advance(65.0)
    lb.touch("a")
    assert lb.tick() == []
    clk.advance(5.5)
    lb.touch("a")
    evs = lb.tick()
    assert [e["kind"] for e in evs] == ["warning"] and evs[0]["remaining_s"] <= 30
    clk.advance(1.0)
    assert lb.tick() == []


def test_extend_resets_warning_and_adds_time(clk):
    lb = mk(clk)
    players(lb, "a")
    lb.tick()
    clk.advance(80.0)
    lb.touch("a")
    assert [e["kind"] for e in lb.tick()] == ["warning"]
    lb.extend(60)
    assert lb.remaining() == pytest.approx(80.0)
    assert lb.active.warned is False
    lb.extend(-1000)  # shortening never goes below 1 s
    assert lb.remaining() == pytest.approx(1.0)


def test_idle_ends_turn(clk):
    lb = mk(clk, idle_s=60)
    players(lb, "a", "b")
    lb.tick()
    clk.advance(59.0)
    assert lb.tick() == []
    clk.advance(1.0)
    assert kinds(lb.tick()) == [("turn_end", "a", "idle")]


def test_touch_only_counts_for_active(clk):
    lb = mk(clk, idle_s=60)
    players(lb, "a", "b")
    lb.tick()
    clk.advance(50)
    lb.touch("b")  # not their turn: does nothing
    clk.advance(10)
    assert kinds(lb.tick()) == [("turn_end", "a", "idle")]


def test_disconnect_grace_and_reconnect_keeps_turn(clk):
    lb = mk(clk, disconnect_s=20)
    players(lb, "a", "b")
    lb.tick()
    lb.disconnect("a")
    clk.advance(15)
    lb.touch("a")
    assert lb.tick() == []
    lb.hello("a", "A")  # back with the same pid
    clk.advance(10)
    assert lb.tick() == [] and lb.is_active("a")
    lb.disconnect("a")
    clk.advance(20)
    assert kinds(lb.tick()) == [("turn_end", "a", "left")]


def test_reconnect_keeps_queue_place(clk):
    lb = mk(clk)
    players(lb, "a", "b", "c")
    lb.tick()
    lb.disconnect("b")
    clk.advance(30)
    lb.hello("b", "Bee")
    assert lb.queue == ["b", "c"] and lb.name("b") == "Bee"


def test_disconnected_queued_player_is_skipped_then_dropped(clk):
    lb = mk(clk, drop_s=120)
    players(lb, "a", "b", "c")
    lb.tick()
    lb.disconnect("b")
    lb.skip()
    assert kinds(lb.tick()) == [("turn_start", "c", None)]  # b is away: c goes first
    assert lb.queue == ["b"]
    clk.advance(120)
    lb.touch("c")
    assert ("dropped", "b", None) in kinds(lb.tick())
    assert lb.queue == []


def test_leave_during_turn_and_while_queued(clk):
    lb = mk(clk)
    players(lb, "a", "b", "c")
    lb.tick()
    assert lb.leave("b") == [] and lb.queue == ["c"]
    assert kinds(lb.leave("a")) == [("turn_end", "a", "left")]
    assert kinds(lb.tick()) == [("turn_start", "c", None)]


def test_skip_and_operator_end(clk):
    lb = mk(clk)
    players(lb, "a", "b")
    lb.tick()
    assert kinds(lb.skip()) == [("turn_end", "a", "skipped")]
    assert lb.skip() == []
    lb.tick()
    assert kinds(lb.end_operator()) == [("turn_end", "b", "operator")]


def test_kick_active_and_queued_with_cooldown(clk):
    lb = mk(clk)
    players(lb, "a", "b", "c")
    lb.tick()
    assert lb.kick("c") == [] and lb.queue == ["b"]
    assert kinds(lb.kick("a")) == [("turn_end", "a", "operator")]
    assert "removed" in lb.join("a")
    clk.advance(301)
    assert lb.join("a") is None


def test_pause_blocks_new_turns_but_current_finishes(clk):
    lb = mk(clk)
    players(lb, "a", "b")
    lb.set_paused(True)
    assert lb.tick() == []  # nothing starts
    lb.set_paused(False)
    lb.tick()
    lb.set_paused(True)
    clk.advance(100)
    assert kinds(lb.tick()) == [("turn_end", "a", "time")]
    assert lb.tick() == [] and lb.active is None
    assert lb.eta("b") is None  # unknown while paused
    lb.set_paused(False)
    assert kinds(lb.tick()) == [("turn_start", "b", None)]


def test_turn_s_applies_to_next_turn_and_is_clamped(clk):
    lb = mk(clk)
    players(lb, "a", "b")
    lb.tick()
    lb.set_turn_s(30)
    assert lb.remaining() == pytest.approx(100)
    lb.skip()
    lb.tick()
    assert lb.remaining() == pytest.approx(30)
    lb.set_turn_s(1)
    assert lb.turn_s == 5.0
    lb.set_turn_s(1e9)
    assert lb.turn_s == 3600.0


def test_eta(clk):
    lb = mk(clk)
    players(lb, "a", "b", "c", "d")
    lb.tick()
    clk.advance(40)
    lb.touch("a")
    assert lb.eta("b") == pytest.approx(60)
    assert lb.eta("c") == pytest.approx(160)
    lb.disconnect("c")  # away players don't count against people behind them
    assert lb.eta("d") == pytest.approx(160)
    assert lb.eta("a") is None
    # history: a turn of 40 s makes the estimate shorter than turn_s
    lb.skip()
    lb.tick()
    assert lb.avg_turn_s() == pytest.approx(40)
    assert lb.eta("d") == pytest.approx(100)  # b has 100 s left; c is away


def test_eta_zero_when_free(clk):
    lb = mk(clk)
    lb.hello("a", "A")
    lb.set_paused(False)
    lb.join("a")
    assert lb.eta("a") == 0


def test_views(clk):
    lb = mk(clk)
    players(lb, "a", "b", "c")
    lb.tick()
    clk.advance(10)
    lb.touch("a")
    v = lb.view_for("c")
    assert v["queue"] == [{"name": "B", "you": False}, {"name": "C", "you": True}]
    assert v["position"] == 2 and v["you_active"] is False and v["active"] == {"name": "A", "remaining_s": 90}
    assert v["eta_s"] == pytest.approx(190)
    va = lb.view_for("a")
    assert va["you_active"] is True and va["position"] is None and va["eta_s"] is None
    d = lb.view_display()
    assert d["queue"] == [{"pid": "b", "name": "B"}, {"pid": "c", "name": "C"}]
    assert d["active"]["pid"] == "a" and d["players"] == 3
    lb.disconnect("c")
    assert lb.view_display()["players"] == 2
    assert lb.view_for(None)["position"] is None


def test_forget_only_idle_players(clk):
    lb = mk(clk)
    players(lb, "a", "b")
    lb.tick()
    lb.forget("a")
    lb.forget("b")
    assert "a" in lb.players and "b" in lb.players
    lb.leave("b")
    lb.forget("b")
    assert "b" not in lb.players


def test_forgotten_active_player_ends_turn(clk):
    lb = mk(clk)
    players(lb, "a")
    lb.tick()
    del lb.players["a"]
    assert kinds(lb.tick()) == [("turn_end", "a", "left")]
