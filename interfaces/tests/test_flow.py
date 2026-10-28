"""Flowing playback: the ``flow`` ease (armctl.playable) and interfaces.timeline.flow.

    .venv/bin/python -m pytest -q interfaces/tests/test_flow.py
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from armctl import envelope as envm
from armctl import kin
from armctl import playable as pl
from interfaces.timeline import coach, record
from interfaces.timeline.flow import make_flow, table_failures
from interfaces.timeline.poses import sequence_playable

READY = list(envm.POSES["ready"])
# every joint that moves keeps moving the same way from pose to pose: a monotone sequence
MONO = [{"q": [0.25 * i, 0.5 + 0.12 * i, 0.8 + 0.1 * i, 0.05 * i, 0.0, -0.1 * i]} for i in range(1, 5)]
POSES = [{"q": [0.4, 0.9, 1.0, 0.0, 0.0, 0.0], "grip": 1.0},
         {"q": [-0.8, 0.7, 2.1, 0.3, 0.0, 0.5], "grip": 0.0},
         {"q": [0.0, 1.2, 1.5, -0.5, 0.3, -0.4]}]


def _sampled(p):
    s = pl.sample(p)
    t, Q = np.array(s["t"]), np.array(s["q"])
    return t, Q, np.gradient(Q, t, axis=0)


def _at(t, k_t):
    return int(np.argmin(np.abs(t - k_t)))


def _kf(t, q, ease="cubic", **kw):
    return {"t": t, "q": list(q), "ease": ease, **kw}


def _playable(kfs, caps="dance"):
    kfs[0].pop("ease", None)
    return {"format": pl.FORMAT, "version": "1.1", "name": "flow-test", "space": "joint",
            "start": "ready", "caps": caps, "keyframes": kfs}


# ---------------------------------------------------------------- the ease itself
def test_lone_flow_segment_is_exactly_cubic():
    a, b = np.array(READY), np.array(READY) + [0.3, 0.1, 0.2, 0, 0, 0]
    pc = _playable([_kf(0, a), _kf(2.0, b, "cubic")])
    pf = _playable([_kf(0, a), _kf(2.0, b, "flow")])
    assert np.allclose(pl.sample(pc)["q"], pl.sample(pf)["q"])


def test_flow_tangents_monotone_and_zero_at_reversal():
    ts = [0, 1, 2, 3]
    X = np.array([[0.0, 0.0], [1.0, 1.0], [2.0, 0.5], [2.5, 0.6]])
    M = pl.flow_tangents(ts, X, ["", "flow", "flow", "flow"])
    assert np.all(M[0] == 0) and np.all(M[-1] == 0)          # rests at both ends
    assert M[1, 0] > 0 and M[2, 0] > 0                        # channel 0 keeps going
    assert M[1, 1] == 0                                        # channel 1 turns round at key 1
    for i in (1, 2):                                           # never more than 3x a secant
        for c in range(2):
            for d in ((X[i, c] - X[i - 1, c]), (X[i + 1, c] - X[i, c])):
                assert abs(M[i, c]) <= 3 * abs(d) + 1e-12
    stopped = pl.flow_tangents(ts, X, ["", "flow", "flow", "flow"], [False, False, True, False])
    assert np.all(stopped[2] == 0) and np.allclose(stopped[1], M[1])
    mixed = pl.flow_tangents(ts, X, ["", "flow", "cubic", "flow"])
    assert np.all(mixed[1] == 0) and np.all(mixed[2] == 0)    # next to a cubic: rest


# ---------------------------------------------------------------- pose sequences
@pytest.mark.parametrize("caps", ["dance", "default"])
def test_flow_sequence_passes_through_keyframes_without_stopping(caps):
    p, notes = sequence_playable(MONO, style="flow", caps=caps)
    r = pl.validate(p)
    assert r["ok"], pl.report(r)
    assert coach.explain(p)["ok"] and not table_failures(p)
    kfs = p["keyframes"]
    assert [k.get("ease") for k in kfs[1:]] == ["flow"] * (len(kfs) - 1)
    assert not any(k.get("ease") == "hold" for k in kfs)
    t, Q, V = _sampled(p)
    # passes through every saved pose, at its keyframe time
    for pose in MONO:
        k = next(k for k in kfs if np.allclose(k["q"], pose["q"], atol=1e-4))
        assert Q[_at(t, k["t"])] == pytest.approx(pose["q"], abs=1e-4)
    # rests at the start and the end ...
    assert np.abs(V[0]).max() < 0.02 and np.abs(V[-1]).max() < 0.02
    # ... but not at the poses in between (pose 4 turns round to go back to ready)
    for k in kfs[1:4]:
        assert np.abs(V[_at(t, k["t"])]).max() > 0.05, k
    # and it never slows to a stop anywhere between the first pose and pose 3
    i0, i1 = _at(t, kfs[1]["t"]), _at(t, kfs[3]["t"])
    assert np.linalg.norm(V[i0:i1 + 1], axis=1).min() > 0.05
    assert "flow" in notes[0] and "flows through" in p["notes"]


def test_default_sequence_still_stops_and_holds():
    p, _ = sequence_playable(MONO)
    kfs = p["keyframes"]
    assert all(k.get("ease") in ("cubic", "hold") for k in kfs[1:])
    t, _, V = _sampled(p)
    for k in kfs[1:]:
        assert np.abs(V[_at(t, k["t"])]).max() < 0.02


@pytest.mark.parametrize("style", ["lively", "gentle"])
def test_flow_flag_with_any_style_is_valid_and_quicker(style):
    p, _ = sequence_playable(POSES, style=style, flow=True)
    q, _ = sequence_playable(POSES, style=style)
    assert pl.validate(p)["ok"] and coach.explain(p)["ok"]
    assert p["keyframes"][-1]["t"] < q["keyframes"][-1]["t"]   # no holds, no stops


def test_flow_style_refusals_unchanged():
    with pytest.raises(ValueError, match="pose 2 .*j4 .*limit 1\\.47"):
        sequence_playable([POSES[0], {"q": [0.0, 0.5, 0.8, 1.55, 0.0, 0.0]}], style="flow")
    with pytest.raises(ValueError, match="style"):
        sequence_playable(POSES, style="wild")


# ---------------------------------------------------------------- caps, limits, table
def test_too_tight_timing_is_stretched_until_it_passes_never_clipped():
    # monotone j1 sweep, timed far too fast for the caps
    kfs = [_kf(0, READY)] + [_kf(0.3 * i, np.array(READY) + [0.3 * i, 0, 0, 0, 0, 0]) for i in range(1, 6)]
    p = _playable(kfs, caps="dance")
    assert not pl.validate(p)["ok"]
    f, notes = make_flow(p)
    r = pl.validate(f)
    assert r["ok"], pl.report(r)
    assert max(r["max_vel"]) <= 0.6 + 1e-6 and max(r["max_acc"]) <= 2.0 * 1.01
    assert any("slowed from" in n for n in notes)
    # same keyframe positions: the path was slowed, not shrunk
    assert [k["q"] for k in f["keyframes"]] == [k["q"] for k in p["keyframes"]]
    assert f["keyframes"][-1]["t"] > p["keyframes"][-1]["t"]
    _, _, V = _sampled(f)
    assert np.abs(V[_at(_sampled(f)[0], f["keyframes"][3]["t"]), 0]) > 0.1   # still flowing


def test_no_overshoot_past_soft_limit_near_the_edge():
    """Keyframes right at j1's soft upper limit, with the curve wanting to swing past it:
    the monotone tangents keep every sample inside the box."""
    hi = pl.envelope().soft[0][1]
    base = np.array(READY)
    qs = [base, base + [hi - 0.6, 0, 0, 0, 0, 0], base + [hi - 0.02, 0, 0, 0, 0, 0],
          base + [hi, 0, 0, 0, 0, 0], base + [hi - 0.5, 0, 0, 0, 0, 0]]
    p = _playable([_kf(0, qs[0])] + [_kf(1.5 * i, q) for i, q in enumerate(qs[1:], 1)])
    f, _ = make_flow(p)
    assert pl.validate(f)["ok"]
    _, Q, _ = _sampled(f)
    assert Q[:, 0].max() <= hi + 1e-9
    # a plain Catmull-Rom tangent would have overshot here; ours doesn't
    lo = pl.envelope().soft[0][0]
    assert Q[:, 0].min() >= lo - 1e-9


def test_overshooting_input_is_refused_not_clipped():
    hi = pl.envelope().soft[0][1]
    p = _playable([_kf(0, READY), _kf(3.0, np.array(READY) + [hi + 0.1, 0, 0, 0, 0, 0])])
    with pytest.raises(ValueError, match="soft limit"):
        make_flow(p)


def test_dip_toward_table_falls_back_to_a_straight_move(monkeypatch):
    """If the curve between two keyframes would come too close to the table, that segment goes
    back to the straight (stopping) move and the notes say so."""
    base = np.array(READY)
    qs = [base, base + [0.3, 0.3, 0.3, 0, 0, 0], base + [0.6, 0.45, 0.35, 0, 0, 0],
          base + [0.9, 0.5, 0.4, 0, 0, 0]]
    p = _playable([_kf(0, qs[0])] + [_kf(2.0 * i, q) for i, q in enumerate(qs[1:], 1)])
    real = kin.clearance
    straight = []
    for a, b in zip(qs, qs[1:]):
        straight += [a + s * (b - a) for s in np.linspace(0, 1, 201)]
    straight = np.array(straight)

    def fake(q):  # "the table" is anywhere off the straight polyline in j2 by > 1 mrad
        tz, lz, ln = real(q)
        off = np.min(np.abs(straight - np.asarray(q)).max(axis=1))
        return (0.0 if off > 1e-3 else tz), lz, ln

    monkeypatch.setattr(kin, "clearance", fake)
    f, notes = make_flow(p)
    assert pl.validate(f)["ok"] and not table_failures(f)
    assert any("straight move" in n for n in notes)
    assert any(k.get("ease") == "cubic" for k in f["keyframes"][1:])


# ---------------------------------------------------------------- recordings
def _recording():
    t = np.arange(0, 6, 0.05)
    base = np.array(envm.POSES["ready"], float)
    Q = np.array([base + np.array([0.4 * math.sin(x), 0.2 * (1 - math.cos(x)), 0.2 * math.sin(2 * x), 0, 0, 0])
                  for x in t])
    return {"format": "openyam-recording", "rate_hz": 20,
            "samples": [{"t": float(a), "q": q.tolist(), "grip": 0.0} for a, q in zip(t, Q)]}


def test_recording_flows():
    rec = _recording()
    p, info = record.to_playable(rec, "test", tol=0.04, caps="dance", flow=True)
    c, cinfo = record.to_playable(rec, "test", tol=0.04, caps="dance")
    assert pl.validate(p)["ok"] and coach.explain(p)["ok"]
    assert info["max_dev_rad"] <= 0.06
    assert any(k.get("ease") == "flow" for k in p["keyframes"])
    assert info["keyframes"] <= cinfo["keyframes"]
    assert "flow_notes" in info
    # the flowing version doesn't stop at its interior keyframes (the cubic one does)
    t, _, V = _sampled(p)
    moving = [k for k in p["keyframes"][1:-1] if not k.get("stop")]
    assert moving and all(np.linalg.norm(V[_at(t, k["t"])]) > 0.02 for k in moving)
