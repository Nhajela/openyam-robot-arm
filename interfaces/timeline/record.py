"""interfaces.timeline.record: record what the arm does, turn it into a playable.

A recording is the arm's measured joints over time, from any interface (pose mirroring,
phone joystick, keyboard, or the arm moved by hand once torque-off teach mode exists):

    {"format": "openyam-recording", "version": "1.0", "rate_hz": 20,
     "samples": [{"t": 0.0, "q": [6 rad], "grip": 0.0}, ...]}

``to_playable`` keeps the moments where the arm (nearly) pauses, adds keyframes until the
cubic-eased playback stays within ``tol`` rad of the recording, then runs the coach's
``fit`` so it obeys the speed caps. What you get back is a normal playable: validate,
preview, edit in the timeline page, play.

``flow=True`` (``--flow``) plays it back as one flowing curve instead: keyframes get the
``flow`` ease (``interfaces.timeline.flow``), so the arm keeps moving through them and only
rests where the recording paused (``"stop": true``). The keys are refined against that curve,
so usually fewer are needed; then it is time-stretched to the caps and table-checked.

    # record from a running daemon for 20 s (observer only: needs no lease, moves nothing)
    .venv/bin/python -m interfaces.timeline.record --live --seconds 20 --out /tmp/rec.json
    # recording -> playable (prints the coach report), save to playables/
    .venv/bin/python -m interfaces.timeline.record /tmp/rec.json --name my-move --save
"""
from __future__ import annotations

import argparse
import asyncio
import json
import sys
import time
from pathlib import Path

import numpy as np

from armctl import envelope as envm
from armctl import playable as pl

from . import coach

STILL_SPEED = 0.03   # rad/s: slower than this counts as paused


def load_recording(path: str | Path) -> dict:
    r = json.loads(Path(path).read_text())
    if r.get("format") != "openyam-recording":
        raise ValueError(f"{path}: not an openyam-recording")
    return r


def _arrays(rec: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    S = rec["samples"]
    t = np.array([s["t"] for s in S], float)
    Q = np.array([s["q"] for s in S], float)
    G = np.array([s.get("grip", 0.0) or 0.0 for s in S], float)
    t = t - t[0]
    return t, Q, G


def trim(t, Q, G, speed=STILL_SPEED):
    """Drop the still lead-in and tail (keep one still sample at each end)."""
    if len(t) < 3:
        return t, Q, G
    v = np.linalg.norm(np.gradient(Q, t, axis=0), axis=1)
    moving = np.where(v > speed)[0]
    if len(moving) == 0:
        return t[:1], Q[:1], G[:1]
    a, b = max(0, moving[0] - 1), min(len(t) - 1, moving[-1] + 1)
    return t[a:b + 1] - t[a], Q[a:b + 1], G[a:b + 1]


def _cubic_eval(t, kt, KQ):
    """Evaluate cubic-ease keyframes (zero velocity at each key) at times t."""
    out = np.empty((len(t), KQ.shape[1]))
    idx = np.clip(np.searchsorted(kt, t, side="right") - 1, 0, len(kt) - 2)
    s = (t - kt[idx]) / (kt[idx + 1] - kt[idx])
    s = np.clip(s, 0, 1)
    w = 3 * s**2 - 2 * s**3
    return KQ[idx] + w[:, None] * (KQ[idx + 1] - KQ[idx])


def _flow_eval(t, kt, KQ, stops):
    """Evaluate ``flow`` keyframes (armctl.playable's Hermite, PCHIP tangents) at times t."""
    n = len(kt)
    M = pl.flow_tangents(kt, KQ, ["flow"] * n, stops)
    idx = np.clip(np.searchsorted(kt, t, side="right") - 1, 0, n - 2)
    h = kt[idx + 1] - kt[idx]
    s = np.clip((t - kt[idx]) / h, 0, 1)[:, None]
    h = h[:, None]
    return ((2 * s**3 - 3 * s**2 + 1) * KQ[idx] + (s**3 - 2 * s**2 + s) * h * M[idx]
            + (-2 * s**3 + 3 * s**2) * KQ[idx + 1] + (s**3 - s**2) * h * M[idx + 1])


def _speed(t, Q):
    return np.linalg.norm(np.gradient(Q, t, axis=0), axis=1) if len(t) > 1 else np.zeros(len(t))


def _stops(idx, v) -> np.ndarray:
    """Keyframes where the recording (nearly) paused: the arm rests there in flow playback."""
    st = np.array([v[i] < STILL_SPEED * 3 for i in idx])
    st[0] = st[-1] = True
    return st


def _playback(t, kt, KQ, idx, v, flow: bool):
    return _flow_eval(t, kt, KQ, _stops(idx, v)) if flow else _cubic_eval(t, kt, KQ)


def keyframes(t, Q, G, tol: float = 0.04, min_gap: float = 0.25, max_keys: int = 80,
              flow: bool = False) -> list[int]:
    """Indices to keep: ends, pauses (local speed minima), then greedy refinement where the
    playback (cubic, or ``flow``) is furthest from the recording, until within ``tol`` rad."""
    n = len(t)
    if n <= 2:
        return list(range(n))
    v = _speed(t, Q)
    keep = {0, n - 1}
    for i in range(1, n - 1):
        pause = v[i] < STILL_SPEED * 3
        if v[i] <= v[i - 1] and v[i] <= v[i + 1] and (pause or (not flow and v[i] < 0.35 * v.max())):
            keep.add(i)
    # grip changes are worth a key at both ends
    dg = np.abs(np.diff(G))
    for i in np.where(dg > 0.05)[0]:
        keep.update((int(i), int(i) + 1))
    keep = _space(sorted(keep), t, min_gap)
    while len(keep) < max_keys:
        kt, KQ = t[keep], Q[keep]
        err = np.abs(_playback(t, kt, KQ, keep, v, flow) - Q).max(axis=1)
        i = int(err.argmax())
        if err[i] <= tol:
            break
        cand = sorted(set(keep) | {i})
        spaced = _space(cand, t, min_gap * 0.5)
        if len(spaced) == len(keep):
            break
        keep = spaced
    return keep


def _space(idx: list[int], t, gap: float) -> list[int]:
    out = [idx[0]]
    for i in idx[1:-1]:
        if t[i] - t[out[-1]] >= gap:
            out.append(i)
    if t[idx[-1]] - t[out[-1]] < gap and len(out) > 1:
        out.pop()
    out.append(idx[-1])
    return out


def to_playable(rec: dict, name: str, tol: float = 0.04, caps: str = "default",
                fit_how: str = "stretch", flow: bool = False) -> tuple[dict, dict]:
    """-> (playable, info). The playable starts at 'ready' (a transition is added on play).
    ``flow``: flowing playback (see the module docstring); ``info["flow_notes"]`` says what
    ``interfaces.timeline.flow.make_flow`` had to do."""
    t, Q, G = trim(*_arrays(rec))
    env = envm.DEFAULT
    Q = np.clip(Q, env.soft_lower, env.soft_upper)  # a recording touching home's stops
    idx = keyframes(t, Q, G, tol, flow=flow)
    v = _speed(t, Q)
    kfs = []
    for k, i in enumerate(idx):
        kf = {"t": round(float(t[i] - t[idx[0]]), 3), "q": [round(float(x), 4) for x in Q[i]],
              "grip": round(float(G[i]), 3)}
        if k:
            kf["ease"] = "cubic"
        kfs.append(kf)
    p = {"format": pl.FORMAT, "version": "1.0", "name": pl.slugify(name),
         "author": f"recording:{rec.get('source', '?')}", "created": pl.now_iso(),
         "notes": f"recorded {len(rec['samples'])} samples at {rec.get('rate_hz', '?')} Hz, "
                  f"{len(kfs)} keyframes (tolerance {tol} rad)",
         "space": "joint", "start": "ready", "caps": caps, "keyframes": kfs}
    flow_notes: list[str] = []
    if flow:
        from .flow import make_flow
        if fit_how == "shrink":
            p = coach.fit(p, "shrink")
        st = _stops(idx, v)
        fitted, flow_notes = make_flow(p, stops=[k for k in range(1, len(kfs) - 1) if st[k]])
    else:
        fitted = coach.fit(p, fit_how)
    played = _playback(t, t[idx], Q[idx], idx, v, flow)
    info = {"samples": len(t), "keyframes": len(kfs), "recorded_s": round(float(t[-1]), 2),
            "playable_s": fitted["keyframes"][-1]["t"],
            "max_dev_rad": round(float(np.abs(played - Q).max()), 3)}
    if flow:
        info["flow_notes"] = flow_notes
    return fitted, info


async def record_live(url: str, seconds: float, rate: float = 20.0) -> dict:
    """Observe a running daemon (hello only, never acquires the lease) and sample its state."""
    from ..client import ArmClient

    samples = []
    async with ArmClient(url, program="recorder", name="timeline recorder") as arm:
        await arm.wait_state()
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            samples.append({"t": round(time.monotonic() - t0, 3), "q": [round(float(x), 4) for x in arm.q],
                            "grip": arm.state.get("grip", 0.0)})
            await asyncio.sleep(1.0 / rate)
    return {"format": "openyam-recording", "version": "1.0", "source": "live", "rate_hz": rate,
            "created": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "samples": samples}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording", nargs="?")
    ap.add_argument("--live", action="store_true", help="record from the daemon instead of a file")
    ap.add_argument("--url", default="ws://127.0.0.1:8091/armctl")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--out", help="with --live: where to save the recording")
    ap.add_argument("--name", default="recorded-move")
    ap.add_argument("--tol", type=float, default=0.04, help="max deviation from the recording, rad")
    ap.add_argument("--caps", choices=sorted(pl.CAP_PRESETS), default="default")
    ap.add_argument("--fit", choices=("stretch", "shrink"), default="stretch",
                    help="how to make it obey the caps: slow down, or keep the timing and shrink moves")
    ap.add_argument("--flow", action="store_true",
                    help="flow through the keyframes instead of stopping at each (rests only at pauses)")
    ap.add_argument("--save", action="store_true", help="save to playables/<name>.json")
    a = ap.parse_args()
    if a.live:
        rec = asyncio.run(record_live(a.url, a.seconds))
        if a.out:
            Path(a.out).write_text(json.dumps(rec))
            print(f"recording -> {a.out}")
    elif a.recording:
        rec = load_recording(a.recording)
    else:
        ap.error("give a recording file or --live")
    p, info = to_playable(rec, a.name, a.tol, a.caps, a.fit, flow=a.flow)
    print(json.dumps(info))
    print(coach.report_text(p))
    if a.save:
        print(f"saved -> {pl.save(p)}")
    else:
        print(json.dumps(p)[:400] + " ...")
    return 0


if __name__ == "__main__":
    sys.exit(main())
