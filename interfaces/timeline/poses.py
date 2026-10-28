"""interfaces.timeline.poses: saved poses -> one smooth, feasible playable.

People drive the arm somewhere with their phone and press "save pose", several times. This
turns that list into a single timeline the arm can play:

    ready -> pose 1 (hold) -> pose 2 (hold) -> ... -> pose N (hold) -> ready

Every segment is a cubic ease timed from the speed caps (``coach.need_time``, the same motion
model the validator uses) times a style factor: ``lively`` 1.25x the minimum, ``gentle`` 1.9x,
never under 0.5 s; gripper changes respect the grip speed cap too. At each pose the arm holds
for a moment (0.3 s lively, 0.6 s gentle) via a ``hold`` keyframe repeating q.

Refused, with a plain-English reason naming the pose: a pose outside the soft box, a pose too
close to the table (``kin.clear_of_table``), a move between two poses that would dip too close
to the table (unless going via 'ready' avoids it), and a sequence over the playable 60 s limit.

    from interfaces.timeline.poses import sequence_playable
    p, notes = sequence_playable([{"q": [0.4, 0.9, 1.0, 0, 0, 0], "grip": 1.0},
                                  {"q": [-0.4, 0.7, 0.9, 0.3, 0, 0.5], "grip": 0.0}])
"""
from __future__ import annotations

import math
from typing import Any

import numpy as np

from armctl import kin
from armctl import playable as pl

from . import coach

STYLES = {"lively": {"factor": 1.25, "hold": 0.3}, "gentle": {"factor": 1.9, "hold": 0.6}}
MIN_SEG_S = 0.5
PATH_CHECKS = 41  # points checked along each move for table clearance


def _ceil(x: float, step: float = 0.01) -> float:
    return math.ceil(x / step - 1e-9) * step


def _parse_poses(poses: Any, lo: np.ndarray, hi: np.ndarray) -> list[tuple[np.ndarray, float]]:
    if not isinstance(poses, (list, tuple)) or not poses:
        raise ValueError("no poses saved yet: drive the arm somewhere and press 'save pose' first")
    out: list[tuple[np.ndarray, float]] = []
    grip = 0.0
    for i, p in enumerate(poses, 1):
        q = p.get("q") if isinstance(p, dict) else None
        try:
            q = np.asarray(q, float).reshape(6)
        except (TypeError, ValueError):
            raise ValueError(f"pose {i} is not a list of 6 joint angles")
        if not np.all(np.isfinite(q)):
            raise ValueError(f"pose {i} has a missing or invalid joint angle")
        g = p.get("grip")
        if g is not None:
            try:
                g = float(g)
            except (TypeError, ValueError):
                raise ValueError(f"pose {i}: grip must be a number 0 (closed) .. 1 (open)")
            if not math.isfinite(g) or not 0.0 <= g <= 1.0:
                raise ValueError(f"pose {i}: grip must be 0 (closed) .. 1 (open), got {g}")
            grip = g
        for j in range(6):
            if q[j] < lo[j] - 1e-9 or q[j] > hi[j] + 1e-9:
                side, lim = ("below", lo[j]) if q[j] < lo[j] else ("above", hi[j])
                raise ValueError(f"pose {i} is outside the arm's safe range: {pl.JOINT_NAMES[j]} is "
                                 f"{q[j]:.2f} rad, {side} its limit {lim:.2f} rad. Save it again a little "
                                 f"further from the end of travel.")
        if not kin.clear_of_table(q):
            tz, lz, ln = kin.clearance(q)
            def height(z: float) -> str:
                return f"{z * 100:.0f} cm above" if z >= 0 else f"{-z * 100:.0f} cm below"
            what = (f"the fingertips would be {height(tz)} the table top (need {kin.MIN_TIP_Z * 100:.0f} cm "
                    f"above)" if tz < kin.MIN_TIP_Z else
                    f"{ln} would be {height(lz)} the table top (need {kin.MIN_LINK_Z * 100:.0f} cm above)")
            raise ValueError(f"pose {i} is too close to the table: {what}. Save it again higher up.")
        out.append((q, grip))
    return out


def _path_clear(a: np.ndarray, b: np.ndarray) -> bool:
    """Every playback ease moves along the straight joint-space line a -> b."""
    return all(kin.clear_of_table(a + s * (b - a)) for s in np.linspace(0.0, 1.0, PATH_CHECKS))


def _seg(a: np.ndarray, b: np.ndarray, ga: float, gb: float, vcap, acap, gcap, factor: float):
    """(time, need, why) for a cubic move a -> b. ``why`` names what limits it."""
    nt = coach.need_time(b - a, "cubic", vcap, acap)
    j = int(np.argmax(nt))
    need = float(nt[j])
    d = abs(float(b[j] - a[j]))
    if 1.5 * d / vcap[j] >= math.sqrt(6.0 * d / acap[j]):
        why = f"{pl.JOINT_NAMES[j]} moves {d:.1f} rad (limit {vcap[j]:.1f} rad/s)"
    else:
        why = f"{pl.JOINT_NAMES[j]} moves {d:.1f} rad (acceleration limit {acap[j]:.1f} rad/s²)"
    gneed = abs(gb - ga) * 1.5 / gcap
    if gneed > need:
        need, why = gneed, f"the gripper changes {abs(gb - ga) * 100:.0f}% (limit {gcap * 100:.0f}%/s)"
    T = max(MIN_SEG_S, _ceil(need * factor))
    return T, need, why


def sequence_playable(poses: list[dict], name: str = "my-sequence", style: str = "lively",
                      caps: str = "dance", start_from=None) -> tuple[dict, list[str]]:
    """Saved poses ``[{"q": [6 rad], "grip": 0..1}, ...]`` -> (playable, notes).

    ``style``: "lively" (1.25x the minimum feasible time per move, 0.3 s hold at each pose)
    or "gentle" (1.9x, 0.6 s hold). ``caps``: the playable's cap preset ("dance" or "default").
    ``start_from``: named pose the sequence starts and ends at (default "ready"; it must be
    inside the soft box, so not "home"). ``grip`` missing on a pose keeps the previous one.

    The playable starts at the start pose, visits each pose, returns, and passes
    ``armctl.playable.validate`` and ``coach.explain``. ``notes`` are short lines for a phone.
    Raises ValueError(reason) naming the pose number when it can't be played.
    """
    if style not in STYLES:
        raise ValueError(f"style must be 'lively' or 'gentle', got {style!r}")
    if caps not in pl.CAP_PRESETS:
        raise ValueError(f"caps must be one of {', '.join(sorted(pl.CAP_PRESETS))}, got {caps!r}")
    start = "ready" if start_from is None else start_from
    env = pl.envelope()
    if not isinstance(start, str) or start not in env.poses:
        raise ValueError(f"start_from must be a named pose ({', '.join(env.poses)}), got {start_from!r}")
    lo = np.array([a for a, _ in env.soft])
    hi = np.array([b for _, b in env.soft])
    q_start = np.array(env.poses[start], float)
    if np.any(q_start < lo - 1e-9) or np.any(q_start > hi + 1e-9):
        raise ValueError(f"start_from {start!r} is outside the soft box (sequences start and end there); "
                         f"use 'ready'")
    parsed = _parse_poses(poses, lo, hi)
    cfg = STYLES[style]

    p: dict[str, Any] = {"format": pl.FORMAT, "version": "1.0", "name": pl.slugify(name),
                         "author": "poses:interfaces.timeline", "created": pl.now_iso(), "space": "joint",
                         "start": start, "caps": caps, "keyframes": []}
    vcap, acap, gcap, _ = coach.caps_for(p)
    g_first = parsed[0][1]
    # (q, grip, label) waypoints; a "via" waypoint is inserted when a direct move would dip
    pts: list[tuple[np.ndarray, float, str]] = [(q_start, g_first, start)]
    via_notes: list[str] = []
    for i, (q, g) in enumerate(parsed, 1):
        pts.append((q, g, f"pose {i}"))
    pts.append((q_start, parsed[-1][1], start))
    route: list[tuple[np.ndarray, float, str]] = [pts[0]]
    for k in range(1, len(pts)):
        (qa, ga, la), (qb, gb, lb) = route[-1], pts[k]
        if not _path_clear(qa, qb):
            if _path_clear(qa, q_start) and _path_clear(q_start, qb) and la != start and lb != start:
                route.append((q_start, ga, f"{start} (via)"))
                via_notes.append(f"{la} → {lb} goes via {start} to stay clear of the table")
            else:
                raise ValueError(f"the move from {la} to {lb} would pass too close to the table. "
                                 f"Add a pose in between, higher up, or save {lb} higher.")
        route.append(pts[k])

    # keyframes: cubic move into each waypoint, a hold after each saved pose
    kfs = [{"t": 0.0, "q": [round(float(x), 4) for x in q_start], "grip": round(g_first, 3)}]
    segs = []  # (label, T, need, why)
    t = 0.0
    for k in range(1, len(route)):
        (qa, ga, la), (qb, gb, lb) = route[k - 1], route[k]
        T, need, why = _seg(qa, qb, ga, gb, vcap, acap, gcap, cfg["factor"])
        t = round(t + T, 2)
        kfs.append({"t": t, "q": [round(float(x), 4) for x in qb], "grip": round(gb, 3), "ease": "cubic"})
        both = la.startswith("pose ") and lb.startswith("pose ")
        segs.append((f"{la} → {lb[5:]}" if both else f"{la} → {lb}", T, need, why))
        if lb.startswith("pose "):
            t = round(t + cfg["hold"], 2)
            kfs.append({"t": t, "q": kfs[-1]["q"], "grip": round(gb, 3), "ease": "hold"})
    p["keyframes"] = kfs
    total = kfs[-1]["t"]
    p["notes"] = f"{len(parsed)} saved poses, {style}, caps '{caps}'"

    if total > pl.MAX_DURATION_S:
        hint = ("Delete some poses." if style == "lively" else
                "Delete some poses or play it 'lively'.")
        raise ValueError(f"this sequence would take {total:.0f} s; the arm plays at most "
                         f"{pl.MAX_DURATION_S:.0f} s at a time. {hint}")

    v = pl.validate(p)
    if not v["ok"]:  # should not happen: timed from the caps
        raise ValueError("the sequence doesn't pass the safety check: "
                         + "; ".join(x["msg"] for x in v["violations"][:3]))
    r = coach.explain(p)
    if not r["ok"]:
        raise ValueError("the sequence doesn't pass the safety check: " + "; ".join(r["summary"][1:4]))

    notes = [f"{len(parsed)} pose{'s' if len(parsed) != 1 else ''}, {total:.1f} s in total ({style})"]
    notes += via_notes
    slow = sorted((s for s in segs if s[2] * cfg["factor"] >= MIN_SEG_S), key=lambda s: -s[2])[:3]
    for label, _T, need, why in sorted(slow, key=lambda s: segs.index(s)):
        notes.append(f"{label} takes at least {need:.1f} s: {why}")
    return p, notes
