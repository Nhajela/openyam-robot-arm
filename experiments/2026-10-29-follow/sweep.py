"""Sweep the follow simulation over paths, speeds, latency (true vs assumed) and lead.

    PYTHONPATH=. .venv/bin/python experiments/2026-10-29-follow/sweep.py [--quick] > experiments/2026-10-29-follow/logs/sweep.jsonl
"""
import argparse
import json
import sys
import time

from interfaces.follow.control import FollowConfig
from interfaces.follow.sim import SimConfig, simulate
from interfaces.follow.track import TrackConfig

ap = argparse.ArgumentParser()
ap.add_argument("--quick", action="store_true")
ap.add_argument("--only", default="")
a = ap.parse_args()

CASES = [
    ("static", dict(path="static")),
    ("line 4cm/s", dict(path="line", speed=0.04)),
    ("circle 4cm/s", dict(path="circle", speed=0.04)),
    ("circle 8cm/s", dict(path="circle", speed=0.08)),
    ("stopgo 5cm/s", dict(path="stopgo", speed=0.05)),
    ("jump 4.5cm", dict(path="jump")),
    ("vanish", dict(path="vanish")),
    ("circle 4cm/s lead 0", dict(path="circle", speed=0.04, lead=0.0)),
    ("circle 4cm/s lead 0.05", dict(path="circle", speed=0.04, lead=0.05)),
    ("circle 4cm/s no feedforward", dict(path="circle", speed=0.04), dict(ff_gain=0.0)),
    ("circle 4cm/s latency 150 ms (known)", dict(path="circle", speed=0.04, latency=0.15)),
    ("circle 4cm/s latency 150 ms, assumed 80", dict(path="circle", speed=0.04, latency=0.15, assumed_latency=0.08)),
    ("circle 4cm/s 8 fps", dict(path="circle", speed=0.04, fps=8.0)),
    ("circle 4cm/s v_max 0.03 (real-arm start)", dict(path="circle", speed=0.04), dict(v_max=0.03)),
    ("circle 3cm/s + identical cup 8 cm away", dict(path="circle", speed=0.03, distractor=(-0.02, -0.25))),
    ("stopgo 5cm/s acc_sigma 0.3", dict(path="stopgo", speed=0.05), {}, dict(acc_sigma=0.3)),
    ("circle 4cm/s acc_sigma 0.3", dict(path="circle", speed=0.04), {}, dict(acc_sigma=0.3)),
]
if a.quick:
    CASES = CASES[:7]
for case in CASES:
    name, sk = case[0], case[1]
    fk = case[2] if len(case) > 2 else {}
    tk = case[3] if len(case) > 3 else None
    if a.only and a.only not in name:
        continue
    t = time.time()
    sc = SimConfig(duration=10.0, **sk)
    r = simulate(sc, FollowConfig(**fk), TrackConfig(height=0.08, size=0.06, **tk) if tk else None)
    r["case"], r["wall_s"] = name, round(time.time() - t, 1)
    print(json.dumps(r, default=float), flush=True)
    print(f"{name:45s} centring med {r['centring_median_mm']:6.1f} p95 {r['centring_p95_mm']:6.1f} mm | est med "
          f"{r['est_err_median_mm']} p95 {r['est_err_p95_mm']} | tracked {r['tracked_frac']:.2f} | lost {r['track']['lost']}"
          f" resets {r['track']['resets']} | {r.get('moved_after_vanish_rad', '')}", file=sys.stderr, flush=True)
