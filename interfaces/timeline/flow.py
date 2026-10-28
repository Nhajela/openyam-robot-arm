"""interfaces.timeline.flow: make a keyframe timeline flow instead of stopping at each keyframe.

A ``cubic`` playable comes to rest at every keyframe. ``make_flow`` turns its cubic segments
into ``flow`` segments (``armctl.playable``: a monotone cubic Hermite, PCHIP tangents, zero
velocity only at the ends, at ``"stop": true`` keyframes and next to non-flow segments), then
makes sure the result is still playable:

* soft limits: monotone tangents never overshoot (each joint stays between its two keyframe
  values), and the validator checks every 100 Hz sample anyway;
* speed caps: the flowing path can be faster than the stop-start one in places, so it is
  time-stretched (``armctl.playable.retime`` on the offending segments, else a uniform
  stretch) until ``armctl.playable.validate`` passes. Never clipped;
* table: the curve is not the straight joint-space line, so every 100 Hz sample is checked with
  forward kinematics (``coach.clearance``, the playable's ``min_tip_z``); a segment that dips
  goes back to a straight ``cubic`` move, and that is reported in the notes.

Anything else the validator refuses raises ValueError with its reason.

    from interfaces.timeline.flow import make_flow
    flowing, notes = make_flow(playable)
"""
from __future__ import annotations

import copy
import math
from typing import Any, Iterable

from armctl import kin
from armctl import playable as pl

from . import coach

SPEED = pl.SPEED_KINDS
TABLE_RATE_HZ = 100.0  # same grid as the validator; contains coach.explain's 20 Hz grid


def fit_caps(p: dict) -> dict:
    """Stretch time until the validator passes; ValueError for anything stretching can't fix."""
    r = pl.validate(p)
    if r["ok"]:
        return p
    other = [v for v in r["violations"] if v["kind"] not in SPEED]
    if other:
        raise ValueError("not playable: " + "; ".join(v["msg"] for v in other[:3]))
    try:
        return pl.retime(p)
    except ValueError:
        pass
    q = copy.deepcopy(p)
    for _ in range(25):  # uniform stretch: velocity scales 1/f, acceleration 1/f^2, exactly
        r = pl.validate(q)
        if r["ok"]:
            return q
        other = [v for v in r["violations"] if v["kind"] not in SPEED]
        if other:
            raise ValueError("not playable: " + "; ".join(v["msg"] for v in other[:3]))
        f = 1.0
        for v in r["violations"]:
            ratio = abs(v["value"]) / v["limit"]
            f = max(f, ratio if v["kind"] != "acceleration" else math.sqrt(ratio))
        f = min(f * 1.03, 3.0)
        for k in q["keyframes"]:
            k["t"] = round(k["t"] * f, 3)
    raise ValueError("could not slow the flowing timeline down enough to meet the speed caps: "
                     + "; ".join(v["msg"] for v in pl.validate(q)["violations"][:3]))


def table_failures(p: dict) -> list[float]:
    """Playback times (s) where the sampled path is too close to the table."""
    s = pl.sample(p, rate=TABLE_RATE_HZ)
    floor = float(p.get("min_tip_z", kin.MIN_TIP_Z))
    out = []
    for t, q in zip(s["t"], s["q"]):
        tz, lz, _ = kin.clearance(q)
        if tz < floor or lz < kin.MIN_LINK_Z:
            out.append(float(t))
    return out


def make_flow(p: dict, stops: Iterable[int] = (), keep: Iterable[int] = ()) -> tuple[dict, list[str]]:
    """Return (flowing copy of ``p``, notes). ``stops``: keyframe indices where the arm should
    still come to rest; ``keep``: keyframe indices whose incoming segment stays as authored.
    Only ``cubic`` segments (the default ease) are turned into ``flow``; ``linear`` and ``hold``
    are left alone (the arm rests next to a hold)."""
    q = copy.deepcopy(p)
    kfs = q["keyframes"]
    keep = set(keep)
    for k in stops:
        kfs[k]["stop"] = True
    for i in range(1, len(kfs)):
        if i not in keep and (kfs[i].get("ease") or "cubic") in ("cubic", "flow"):
            kfs[i]["ease"] = "flow"

    q["format"], q["version"] = pl.FORMAT, "1.1"  # "flow" is a 1.1 ease
    before = pl.validate(q).get("duration") or kfs[-1]["t"]

    straight: list[int] = []
    for _ in range(len(kfs) + 2):
        q = fit_caps(q)
        bad = table_failures(q)
        if not bad:
            break
        segs, _, _ = pl.timeline(q)
        hit = sorted({sg.seg_index for sg in segs for t in bad
                      if sg.t0 - 1e-9 <= t <= sg.t1 + 1e-9 and sg.seg_index is not None
                      and q["keyframes"][sg.seg_index].get("ease") == "flow"})
        if not hit:
            raise ValueError(coach.clearance(*_samples(q), float(q.get("min_tip_z", kin.MIN_TIP_Z)))["msg"]
                             or "the path is too close to the table")
        for k in hit:
            q["keyframes"][k]["ease"] = "cubic"
            straight.append(k)
    else:
        raise ValueError("could not keep the flowing path clear of the table")
    after = pl.validate(q)["duration"]

    flowing = sum(1 for k in q["keyframes"][1:] if k.get("ease") == "flow")
    notes = [f"flows through {flowing} of {len(kfs) - 1} moves without stopping at the keyframes"]
    if after > before + 0.05:
        notes.append(f"slowed from {before:.1f} s to {after:.1f} s to stay under the speed caps")
    for k in sorted(set(straight)):
        notes.append(f"keyframe {k - 1} → {k} stays a straight move (stopping) to keep clear of the table")
    return q, notes


def _samples(p: dict) -> tuple[list, list]:
    s = pl.sample(p, rate=TABLE_RATE_HZ)
    return s["t"], s["q"]
