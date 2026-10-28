"""Tests for openplay.moves and the moves library it catalogues.

Run from the repo root: .venv/bin/python -m pytest openplay/tests/test_moves.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from armctl import kin  # noqa: E402
from armctl import playable as P  # noqa: E402
from interfaces.timeline import coach  # noqa: E402
from openplay import moves  # noqa: E402

READY = [0.0, 0.5, 0.8, 0.0, 0.0, 0.0]
ENTRIES = json.loads((REPO / "openplay" / "moves.json").read_text())
NAMES = [e["name"] for e in ENTRIES]
NEW = ["bow", "shake-no", "shrug", "celebrate", "robot-dance", "sleepy-stretch", "thinking",
       "peekaboo", "conductor", "disco-point", "high-five-offer", "curious-sniff", "gripper-clap"]


def test_moves_json_shape():
    assert len(NAMES) == len(set(NAMES)), "duplicate names in moves.json"
    for e in ENTRIES:
        assert set(e) >= {"name", "title", "emoji", "blurb", "tags"}, e
        assert P.NAME_RE.match(e["name"])
        assert len(e["blurb"]) <= 60, (e["name"], len(e["blurb"]))
        assert e["tags"] and all(isinstance(t, str) and t for t in e["tags"])
    for n in ("rom-arm", "rom-wrist"):
        assert n not in NAMES


def test_catalogue_keeps_every_entry():
    cat = moves.catalogue()
    assert [m["name"] for m in cat] == NAMES, "a catalogued move was dropped (see the log)"
    for m in cat:
        assert set(m) == set(moves.FIELDS)
        assert 0 < m["duration_s"] <= 30


@pytest.mark.parametrize("name", NAMES)
def test_move_is_playable(name):
    p = P.load(name)
    r = P.validate(p)
    assert r["ok"], P.report(r)
    assert r["transition"] is None, "first keyframe should be ready (no transition)"
    assert r["duration"] <= 30.0
    kf = p["keyframes"]
    assert np.allclose(kf[0]["q"], READY, atol=1e-6) and kf[0]["t"] == 0
    assert np.allclose(kf[-1]["q"], READY, atol=1e-6), "must end at ready"
    assert p.get("start", "ready") == "ready"
    assert p.get("loop", 1) == 1
    e = coach.explain(p)
    assert e["ok"], e["summary"]
    assert e["clearance"]["ok"], e["clearance"]["msg"]


@pytest.mark.parametrize("name", NEW)
def test_new_move_house_rules(name):
    p = P.load(name)
    assert p["format"] == "openyam-playable" and p["version"] == "1.0"
    assert p["author"] == "human:openplay library (claude agent)"
    assert p.get("notes")
    assert all(k.get("ease", "cubic") in ("cubic", "hold") for k in p["keyframes"])
    r = P.validate(p)
    assert 4.0 <= r["duration"] <= 20.0
    s = P.sample(p, rate=50.0)
    Q = np.asarray(s["q"], float)
    assert Q[:, 2].max() <= 2.8 + 1e-6, "j3 too close to full stretch"
    assert Q[:, 1].min() >= 0.3 - 1e-6 and Q[:, 1].max() <= 3.0 + 1e-6
    assert min(kin.tip(q)[2] for q in Q) >= 0.10, "tip within 10 cm of the table"


def test_bow_lowers_the_tip():
    # hardware finding: a bow must LOWER the tip, not raise it
    p = P.load("bow")
    Q = np.asarray(P.sample(p, rate=20.0)["q"], float)
    assert min(kin.tip(q)[2] for q in Q) < kin.tip(READY)[2] - 0.2


def test_load():
    p = moves.load("wave-hello")
    assert p["name"] == "wave-hello"
    with pytest.raises(KeyError):
        moves.load("rom-arm")  # exists as a playable, but isn't a phone move
    with pytest.raises(KeyError):
        moves.load("../armctl/envelope")


@pytest.mark.parametrize("text,want", [
    ("say hi to everyone", "wave-hello"),
    ("wave at the kids", "wave-hello"),
    ("Hello!", "wave-hello"),
    ("do a little dance", "happy-dance"),
    ("dance like a robot", "robot-dance"),
    ("take a bow", "bow"),
    ("say thank you", "bow"),
    ("nodding yes", "nod"),
    ("no no no", "shake-no"),
    ("I don't know", "shrug"),
    ("we won!!", "celebrate"),
    ("high five", "high-five-offer"),
    ("clapping", "gripper-clap"),
    ("conduct the orchestra", "conductor"),
    ("point to the sky", "disco-point"),
    ("peekabo", "peekaboo"),            # typo
    ("celebrat", "celebrate"),          # typo
    ("you look sleepy", "sleepy-stretch"),
    ("come closer and sniff", "curious-sniff"),
    ("hmm let me think", "thinking"),
    ("look around the room", "look-around"),
    ("can the robot shrug", "shrug"),
])
def test_match(text, want):
    assert moves.match(text) == want


@pytest.mark.parametrize("text", ["", "   ", "what's the weather in Goa", "rainbow", "robot", "please"])
def test_match_none(text):
    assert moves.match(text) is None
