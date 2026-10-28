"""interfaces.timeline.coach: explain, in words, what a timeline asks of the arm and what it can't do.

``armctl.playable.validate`` says whether a playable is legal. The coach says *why not* in
terms a person editing a timeline can act on, per segment:

    "0.80 -> 1.30 s: j3 moves 0.85 rad in 0.50 s. At the caps (0.60 rad/s, 2.0 rad/s^2) that
     needs at least 1.30 s, or a move of at most 0.17 rad in 0.50 s."

and gives, for every keyframe, the band each joint can reach from the previous keyframe in
the time allowed (``bounds``), so an editor can draw "you can't go further than this, this
fast". It also checks table clearance with forward kinematics (armctl.kin) and offers two
fixes: ``fit(p, "stretch")`` slows the offending segments down, ``fit(p, "shrink")`` keeps
the timing and scales the offending moves down.

Motion model (same as armctl.playable): ``cubic`` ease peaks at 1.5 d/T velocity and
6 d/T^2 acceleration; ``linear`` at d/T (and needs matching neighbours); ``hold`` doesn't move.
``flow`` depends on its neighbours, so its segments are judged (and stretched) by the
validator itself rather than by this per-segment model.
"""
from __future__ import annotations

import copy
from typing import Any

import numpy as np

from armctl import kin
from armctl import playable as pl

JOINTS = pl.JOINT_NAMES


def caps_for(playable: dict) -> tuple[np.ndarray, np.ndarray, float, str]:
    """(vel caps, acc caps, grip vel cap, preset name) the validator will use for this playable."""
    preset = playable.get("caps", "default")
    c = pl.CAP_PRESETS.get(preset)
    env = pl.envelope()
    if c is None:
        return np.array(env.vel_cap), np.array(env.acc_cap), env.grip_vel_cap, preset
    return np.full(6, c[0]), np.full(6, c[1]), env.grip_vel_cap, preset


def need_time(d: np.ndarray | float, ease: str, vcap, acap) -> np.ndarray:
    """Shortest segment time for a move of ``d`` rad (per joint) with this ease."""
    d = np.abs(np.asarray(d, float))
    if ease == "hold":
        return np.zeros_like(d)
    if ease == "linear":
        return d / vcap
    return np.maximum(1.5 * d / vcap, np.sqrt(6.0 * d / acap))


def max_move(T: float, ease: str, vcap, acap) -> np.ndarray:
    """Largest move (rad, per joint) that fits in T seconds with this ease."""
    vcap, acap = np.asarray(vcap, float), np.asarray(acap, float)
    if ease == "hold":
        return np.zeros_like(vcap)
    if ease == "linear":
        return vcap * T
    return np.minimum(vcap * T / 1.5, acap * T * T / 6.0)


def _keys(p: dict) -> list[dict]:
    kfs = p.get("keyframes") or []
    out, grip = [], 0.0
    for k in kfs:
        grip = float(k.get("grip", grip))
        out.append({"t": float(k["t"]), "q": np.array(k["q"], float), "grip": grip,
                    "ease": k.get("ease", "cubic")})
    return out


def explain(p: dict, *, table: bool = True) -> dict[str, Any]:
    """Full report for an editor: segments, per-keyframe bounds, clearance, validator output."""
    vcap, acap, gcap, preset = caps_for(p)
    speed = float(p.get("speed", 1.0) or 1.0)
    env = pl.envelope()
    lo = np.array([a for a, _ in env.soft])
    hi = np.array([b for _, b in env.soft])
    keys = _keys(p)
    segs, bounds = [], []
    if keys:
        bounds.append({"k": 0, "t": keys[0]["t"], "lo": lo.round(3).tolist(), "hi": hi.round(3).tolist()})
    for i in range(1, len(keys)):
        a, b = keys[i - 1], keys[i]
        T = (b["t"] - a["t"]) / speed          # playback seconds
        d = b["q"] - a["q"]
        need = need_time(d, b["ease"], vcap, acap) * speed   # back in authored seconds
        dmax = max_move(T, b["ease"], vcap, acap)
        reach_lo = np.maximum(lo, a["q"] - dmax)
        reach_hi = np.minimum(hi, a["q"] + dmax)
        gneed = abs(b["grip"] - a["grip"]) * (1.5 if b["ease"] == "cubic" else 1.0) / gcap * speed
        problems = []
        for j in range(6):
            # a "flow" segment's speed depends on its neighbours: the validator judges it
            if b["ease"] != "flow" and need[j] > (b["t"] - a["t"]) * 1.001 + 1e-9:
                problems.append({
                    "joint": JOINTS[j], "kind": "too_fast", "need_s": round(float(need[j]), 3),
                    "max_move": round(float(dmax[j]), 3),
                    "msg": (f"{a['t']:g} -> {b['t']:g} s: {JOINTS[j]} moves {abs(d[j]):.2f} rad in "
                            f"{b['t'] - a['t']:.2f} s. At the caps ({vcap[j]:.2f} rad/s, {acap[j]:.1f} rad/s^2) "
                            f"that needs at least {need[j]:.2f} s, or a move of at most {dmax[j]:.2f} rad in this time."
                            )})
        if gneed > (b["t"] - a["t"]) * 1.001 + 1e-9:
            problems.append({"joint": "grip", "kind": "too_fast", "need_s": round(gneed, 3),
                             "msg": f"{a['t']:g} -> {b['t']:g} s: gripper changes {abs(b['grip'] - a['grip']):.2f} "
                                    f"in {b['t'] - a['t']:.2f} s; needs at least {gneed:.2f} s"})
        for j in range(6):
            if b["q"][j] < lo[j] - 1e-9 or b["q"][j] > hi[j] + 1e-9:
                problems.append({"joint": JOINTS[j], "kind": "out_of_range",
                                 "msg": f"keyframe {i} (t={b['t']:g}): {JOINTS[j]} {b['q'][j]:.2f} is outside "
                                        f"its soft range [{lo[j]:.2f}, {hi[j]:.2f}]"})
        segs.append({"i": i, "t0": a["t"], "t1": b["t"], "ease": b["ease"],
                     "need_s": round(float(max(need.max(), gneed)), 3),
                     "ok": not problems, "problems": problems})
        bounds.append({"k": i, "t": b["t"], "lo": reach_lo.round(3).tolist(), "hi": reach_hi.round(3).tolist(),
                       "from": a["q"].round(3).tolist(), "max_move": dmax.round(3).tolist()})

    v = pl.validate(p)
    out: dict[str, Any] = {"ok": bool(v["ok"]), "caps": {"preset": preset, "vel": vcap.round(3).tolist(),
                                                         "acc": acap.round(3).tolist(), "grip_vel": gcap},
                           "duration": v.get("duration"), "segments": segs, "bounds": bounds,
                           "violations": [x["msg"] for x in v.get("violations", [])],
                           "transition": v.get("transition")}
    # sampled path for plotting and the table check
    try:
        s = pl.sample(p, rate=20.0)
        ts = np.asarray(s.get("t", []), float)
        Q = np.asarray(s.get("q", []), float)
        G = np.asarray(s.get("grip", np.zeros(len(ts))), float)
        out["samples"] = {"t": ts.round(3).tolist(), "q": Q.round(4).tolist(), "grip": G.round(3).tolist()}
    except Exception as e:  # invalid playables may not sample
        out["samples"] = None
        out["sample_error"] = str(e)
    if table and out.get("samples"):
        out["clearance"] = clearance(out["samples"]["t"], out["samples"]["q"], float(p.get("min_tip_z", kin.MIN_TIP_Z)))
        if not out["clearance"]["ok"]:
            out["ok"] = False
    out["summary"] = summary(out)
    return out


def clearance(ts, Q, min_tip_z: float = kin.MIN_TIP_Z) -> dict[str, Any]:
    """Table check on sampled joints. ``min_tip_z`` below the default is for playables that
    touch the table on purpose (grasps); the links keep their own floor regardless."""
    worst = (9.0, None, "")
    fails = []
    for t, q in zip(ts, Q):
        tz, lz, ln = kin.clearance(q)
        z = min(tz - min_tip_z, lz - kin.MIN_LINK_Z)
        if z < worst[0]:
            worst = (z, t, ln if lz - kin.MIN_LINK_Z < tz - min_tip_z else "gripper_tip")
        if z < 0:
            fails.append(t)
    msg = ""
    if fails:
        msg = (f"too close to the table from playback {fails[0]:.2f} s to {fails[-1]:.2f} s "
               f"({worst[2]} {abs(worst[0]) * 100:.1f} cm under the safe height at {worst[1]:.2f} s)")
    return {"ok": not fails, "min_margin_m": round(worst[0], 4), "t_worst": worst[1], "msg": msg}


def summary(r: dict) -> list[str]:
    """One headline, then one line per problem segment (all its joints together)."""
    lines = []
    bad = [s for s in r["segments"] if not s["ok"]]
    lines.append(("OK" if r["ok"] else "NOT PLAYABLE") + f": {len(r['segments'])} segments, "
                 f"{r.get('duration')} s, caps '{r['caps']['preset']}' "
                 f"({r['caps']['vel'][0]} rad/s, {r['caps']['acc'][0]} rad/s^2)")
    for s in bad:
        fast = [p for p in s["problems"] if p["kind"] == "too_fast"]
        if fast:
            worst = max(fast, key=lambda p: p["need_s"])
            moves = ", ".join(p["joint"] for p in fast)
            hint = f"; in {s['t1'] - s['t0']:.2f} s {worst['joint']} can move at most {worst['max_move']:.2f} rad" \
                if "max_move" in worst else ""
            lines.append(f"  - {s['t0']:g} -> {s['t1']:g} s is too fast for {moves}: needs at least "
                         f"{s['need_s']:.2f} s (has {s['t1'] - s['t0']:.2f}){hint}")
        for p in s["problems"]:
            if p["kind"] != "too_fast":
                lines.append("  - " + p["msg"])
    if r.get("clearance") and not r["clearance"]["ok"]:
        lines.append("  - " + r["clearance"]["msg"])
    if not bad:
        lines += ["  - validator: " + v for v in r["violations"][:8]]
    return lines


def fit(p: dict, how: str = "stretch") -> dict:
    """Return a copy that fits the caps. ``stretch``: lengthen the too-fast segments (and shift
    everything after). ``shrink``: keep the timing, pull each too-fast keyframe back toward the
    previous one until the move fits (changes the shape, keeps the rhythm, e.g. for music)."""
    q = copy.deepcopy(p)
    vcap, acap, gcap, _ = caps_for(q)
    speed = float(q.get("speed", 1.0) or 1.0)
    kfs = q["keyframes"]
    if how == "stretch":
        prev_t_old = kfs[0]["t"]
        for i in range(1, len(kfs)):
            a, b = kfs[i - 1], kfs[i]
            dt_old = b["t"] - prev_t_old
            prev_t_old = b["t"]
            d = np.array(b["q"]) - np.array(a["q"])
            ease = b.get("ease", "cubic")
            ga, gb = a.get("grip"), b.get("grip")
            gneed = 0.0 if ga is None or gb is None else abs(gb - ga) * 1.5 / gcap
            need = max(float(need_time(d, ease, vcap, acap).max()), gneed) * speed * 1.02
            if ease == "flow":  # timed by the validator below
                need = 0.0
            new_dt = max(dt_old, need)
            b["t"] = round(a["t"] + new_dt, 3)
        if any(k.get("ease") == "flow" for k in kfs):
            from .flow import fit_caps
            try:
                q = fit_caps(q)
            except ValueError:  # not a speed problem: leave it for the report
                pass
        return q
    if how == "shrink":
        for i in range(1, len(kfs)):
            a, b = kfs[i - 1], kfs[i]
            T = (b["t"] - a["t"]) / speed
            ease = b.get("ease", "cubic")
            dmax = max_move(T, ease, vcap, acap) * 0.98
            qa, qb = np.array(a["q"], float), np.array(b["q"], float)
            d = qb - qa
            scale = 1.0
            for j in range(6):
                if abs(d[j]) > dmax[j] > 0:
                    scale = min(scale, dmax[j] / abs(d[j]))
            if scale < 1.0:  # scale the whole move so the pose keeps its shape
                b["q"] = [round(float(x), 4) for x in qa + d * scale]
        return q
    raise ValueError("how must be 'stretch' or 'shrink'")


def report_text(p: dict) -> str:
    return "\n".join(explain(p)["summary"])


if __name__ == "__main__":  # .venv/bin/python -m interfaces.timeline.coach wave beat-dance
    import sys

    for n in sys.argv[1:] or pl.list_playables():
        print(f"{n}: " + report_text(pl.load(n)))
