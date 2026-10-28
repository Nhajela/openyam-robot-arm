"""Unit tests for openplay.say (fallbacks; Claude is never called)."""
from __future__ import annotations

import asyncio
import sys
import time
import types

import pytest

from openplay import say

PLAYABLE = {"name": "wave", "space": "joint", "start": "ready",
            "keyframes": [{"t": 0, "q": [0, 0.5, 0.8, 0, 0, 0], "grip": 0.0}]}


@pytest.fixture
def fake_moves(monkeypatch):
    m = types.ModuleType("openplay.moves")
    m.catalogue = lambda: [{"name": "wave", "title": "Wave", "emoji": "👋", "duration_s": 9, "blurb": ""},
                           {"name": "nod", "title": "Nod yes", "emoji": "🙂", "duration_s": 9, "blurb": ""}]
    m.match = lambda text: "wave" if "wave" in text else None
    m.load = lambda name: {**PLAYABLE, "name": name}
    monkeypatch.setitem(sys.modules, "openplay.moves", m)
    import openplay
    monkeypatch.setattr(openplay, "moves", m, raising=False)
    return m


def run(coro):
    return asyncio.run(coro)


def test_title():
    assert say.make_title("  wave at the kids!! ") == "Wave at the kids"
    t = say.make_title("do a really long and complicated dance for everyone in the room please")
    assert len(t) <= 33 and t.endswith("…") and t.startswith("Do a really")
    assert say.make_title("") == "Something new"


def test_no_key_falls_back_to_library(monkeypatch, fake_moves):
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    monkeypatch.setattr(say, "_claude", lambda text: pytest.fail("Claude must not be called without a key"))
    r = run(say.author("wave at the kids"))
    assert r["source"] == "library" and r["playable"]["name"] == "wave"
    assert r["title"] == "Wave" and "no Claude key" in r["note"]


def test_no_match_plays_a_default(monkeypatch, fake_moves):
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    r = run(say.author("juggle three oranges"))
    assert r["source"] == "library" and r["playable"]["name"] == "wave"
    assert "nothing matched" in r["note"]


def test_claude_success(monkeypatch, fake_moves):
    monkeypatch.setenv("ANTHROPIC_API_KEY", "test-not-a-real-key")
    pl = {**PLAYABLE, "name": "kids-wave"}
    monkeypatch.setattr(say, "_claude", lambda text: {"ok": True, "playable": pl, "retimed": False})
    r = run(say.author("wave at the kids"))
    assert r == {"playable": pl, "title": "Wave at the kids", "source": "claude", "note": ""}


def test_claude_invalid_falls_back(monkeypatch, fake_moves):
    monkeypatch.setenv("ANTHROPIC_API_KEY", "test-not-a-real-key")
    monkeypatch.setattr(say, "_claude", lambda text: {"ok": False, "playable": PLAYABLE})
    r = run(say.author("wave please"))
    assert r["source"] == "library" and "safety check" in r["note"]


def test_claude_error_falls_back_without_leaking_key(monkeypatch, fake_moves):
    monkeypatch.setenv("ANTHROPIC_API_KEY", "test-not-a-real-key")

    def boom(text):
        raise RuntimeError("401 bad key test-not-a-real-key")

    monkeypatch.setattr(say, "_claude", boom)
    r = run(say.author("wave"))
    assert r["source"] == "library" and "RuntimeError" in r["note"]
    assert "test-not-a-real-key" not in r["note"]


def test_claude_timeout_falls_back(monkeypatch, fake_moves):
    monkeypatch.setenv("ANTHROPIC_API_KEY", "test-not-a-real-key")
    monkeypatch.setattr(say, "_claude", lambda text: time.sleep(1.0) or {"ok": True, "playable": PLAYABLE})
    t0 = time.monotonic()
    r = run(say.author("wave", timeout_s=0.2))
    assert r["source"] == "library" and "longer than" in r["note"]
    assert time.monotonic() - t0 < 2.0


def test_empty_text_and_missing_library(monkeypatch):
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    with pytest.raises(say.SayError):
        run(say.author("   "))
    monkeypatch.setitem(sys.modules, "openplay.moves", None)  # import fails
    import openplay
    monkeypatch.delattr(openplay, "moves", raising=False)
    with pytest.raises(say.SayError, match="unavailable"):
        run(say.author("wave"))


def test_real_library_match_if_present(monkeypatch):
    """Against the real openplay.moves when it exists (owned by another module)."""
    moves = pytest.importorskip("openplay.moves")
    if not moves.catalogue():
        pytest.skip("empty catalogue")
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    r = run(say.author("say hi to everyone"))
    assert r["source"] == "library" and r["playable"].get("keyframes")
