"""Quantify the existing playables library: amplitude, tempo variety, holds, repetition.

    /home/naman/openyam-robot-arm/.venv/bin/python experiments/2026-10-29-visual-choreo/library_stats.py
"""
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
from armctl import playable as P  # noqa: E402

rows = []
for f in sorted((REPO / "playables").glob("*.json")):
    p = json.loads(f.read_text())
    k = p.get("keyframes") or []
    if len(k) < 2:
        continue
    t = np.array([x["t"] for x in k], float)
    Q = np.array([x["q"] for x in k], float)
    dt = np.diff(t)
    rng = Q.max(0) - Q.min(0)
    eases = sorted({x.get("ease", "cubic") for x in k})
    holds = sum(x.get("ease") == "hold" for x in k)
    uniq = len({tuple(np.round(x, 2)) for x in Q})
    grips = len({round(float(x.get("grip", 0)), 2) for x in k})
    ok = P.validate(p)["ok"]
    rows.append((f.stem, len(k), t[-1], dt.std() / dt.mean() if dt.mean() else 0, rng, eases, holds, uniq, grips, ok))
print(f"{'name':24s} {'keys':>4s} {'dur':>5s} {'tempoCV':>7s}  range j1..j6 (rad)              "
      f"eases            holds uniq grips ok")
for n, nk, d, cv, rng, e, h, u, g, ok in rows:
    print(f"{n:24s} {nk:4d} {d:5.1f} {cv:7.2f}  {' '.join(f'{x:4.2f}' for x in rng)}  "
          f"{','.join(e):16s} {h:5d} {u:4d} {g:5d} {ok}")
a = np.array([r[4] for r in rows])
print("median range per joint:", np.round(np.median(a, 0), 2), " median tempo CV:",
      round(float(np.median([r[3] for r in rows])), 2))
