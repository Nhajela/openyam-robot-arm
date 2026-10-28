#!/usr/bin/env python3
"""Wrist-camera look-around: visit look poses (fingers down) and record frames + joint state.

Every frame is saved with the arm's joint angles at the moment it was fetched, and the
wrist-camera pose follows from FK (gripper_tip; the camera sits between the fingers), so the
frames can be mapped back to where the camera was. All motion goes through the armctl daemon
(move_joints, soft limits, speed capped by --speed). Ctrl-C sends stop (support the arm).

    .venv/bin/python experiments/2026-10-28-laptop-systems-check/scan_record.py --speed 0.3

Writes <out>/frames/NNNN.jpg, <out>/frames.csv (t, pose label, q1..q6, tip xyz, frame file),
<out>/poses.json. Frames may show people: the default out folder is git-ignored (-local).
"""
from __future__ import annotations

import argparse
import asyncio
import csv
import hashlib
import json
import sys
import time
import urllib.request
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))
from armctl import envelope as env  # noqa: E402
from armctl import kin  # noqa: E402
from interfaces.client import ArmClient, Refused  # noqa: E402

# tip positions (base frame: x forward, y = arm's left, z up), fingers straight down.
# Arm's right = -y. Top-down is reachable only up to z ~0.20 m (j4 soft limit).
LOOKS = [("right-mid", (0.28, -0.12, 0.20)), ("right-near", (0.22, -0.20, 0.20)),
         ("right-far", (0.34, -0.20, 0.18)), ("centre-far", (0.34, -0.06, 0.18)),
         ("centre-near", (0.22, -0.04, 0.20)), ("left-mid", (0.28, 0.04, 0.20)),
         ("right-mid-low", (0.28, -0.12, 0.14))]


def panorama(j1_max: float = 1.4, step: float = 0.35) -> list[tuple[str, list[float]]]:
    """Joint-space look-around: tip ~0.30 m out and 0.29-0.36 m up (above the platform), the
    wrist camera aimed outward-down at two tilts, base swept across the front (serpentine).
    The wrist camera looks ~55 deg 'back' from the fingers (scan1), so the fingers point
    outward, slightly up, for an outward-down view."""
    j1s = np.round(np.arange(-j1_max, j1_max + 1e-9, step), 3)
    out = []
    for k, j4 in enumerate((-0.3, 0.2)):
        for j1 in (j1s if k % 2 == 0 else j1s[::-1]):
            out.append((f"pan j4={j4:+.1f} j1={j1:+.2f}", [float(j1), 0.9, 1.0, j4, 0.0, 0.0]))
    return out


def plan() -> list[tuple[str, list[float]]]:
    out, prev = [], np.array(env.POSES["ready"], float)
    for name, xyz in LOOKS:
        r = kin.ik(xyz, prev, down=True)
        if not r.ok:
            raise SystemExit(f"look pose {name} {xyz} not reachable (err {r.err_pos:.4f})")
        out.append((name, [round(float(v), 4) for v in r.q]))
        prev = r.q
    return out


class Recorder:
    def __init__(self, arm: ArmClient, url: str, out: Path):
        self.arm, self.url, self.out = arm, url, out
        self.label, self.rows, self.last_hash, self.n = "start", [], None, 0
        (out / "frames").mkdir(parents=True, exist_ok=True)

    def grab(self) -> None:
        try:
            jpg = urllib.request.urlopen(self.url, timeout=1.0).read()
        except Exception:
            return
        h = hashlib.md5(jpg).hexdigest()
        if h == self.last_hash:
            return
        self.last_hash = h
        q = np.array(self.arm.q, float)
        tip = kin.fk(q)[:3, 3]
        name = f"{self.n:04d}.jpg"
        (self.out / "frames" / name).write_bytes(jpg)
        self.rows.append({"t": round(time.time(), 3), "pose": self.label, "file": name,
                          **{f"q{i+1}": round(float(q[i]), 4) for i in range(6)},
                          "tip_x": round(float(tip[0]), 4), "tip_y": round(float(tip[1]), 4),
                          "tip_z": round(float(tip[2]), 4)})
        self.n += 1

    async def run(self, stop: asyncio.Event, hz: float) -> None:
        while not stop.is_set():
            await asyncio.to_thread(self.grab)
            await asyncio.sleep(1 / hz)

    def save(self) -> None:
        if not self.rows:
            return
        with (self.out / "frames.csv").open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(self.rows[0]))
            w.writeheader()
            w.writerows(self.rows)


async def main(a) -> None:
    if a.looks:  # FILE:KEY -> [[name, q], ...] planned elsewhere (checked below)
        f, key = a.looks.rsplit(":", 1)
        looks = [(n, list(q)) for n, q in json.loads(Path(f).read_text())[key]]
    else:
        looks = panorama() if a.panorama else plan()
    E = env.Envelope()
    for name, q in looks:  # every stop inside the soft box and well clear of the table
        qa = np.array(q)
        if not ((qa >= E.soft_lower) & (qa <= E.soft_upper)).all() or kin.clearance(qa)[0] < 0.10:
            raise SystemExit(f"refusing look {name}: outside the soft box or < 10 cm from the table")
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "poses.json").write_text(json.dumps({"looks": LOOKS, "q": looks, "speed": a.speed}, indent=1))
    async with ArmClient(a.url, program="script", name="wristcam-scan") as arm:
        await arm.wait_state()
        print(f"backend={arm.backend} start q={np.round(arm.q, 3).tolist()}", flush=True)
        await arm.acquire(timeout=10)
        rec, stop = Recorder(arm, a.camera, out), asyncio.Event()
        task = asyncio.create_task(rec.run(stop, a.hz))
        try:
            rec.label = "to-ready"
            await arm.do("ready", timeout=90)
            for name, q in looks:
                rec.label = f"to-{name}"
                print(f"-> {name} {q}", flush=True)
                try:
                    await arm.do("move_joints", q=q, speed=a.speed, timeout=90)
                except Refused as e:
                    print(f"   (verify refused, holding where it got to: {e})", flush=True)
                rec.label = name
                await asyncio.sleep(a.dwell)
            rec.label = "to-ready-end"
            await arm.do("ready", timeout=90)
        except (asyncio.CancelledError, KeyboardInterrupt):
            await arm.stop()
            raise
        finally:
            stop.set()
            await task
            rec.save()
            print(f"wrote {rec.n} frames to {out}", flush=True)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default="ws://127.0.0.1:8091/armctl")
    ap.add_argument("--camera", default="http://127.0.0.1:8090/camera.jpg")
    ap.add_argument("--speed", type=float, default=0.3, help="rad/s for the look moves")
    ap.add_argument("--dwell", type=float, default=2.0, help="seconds still at each look pose")
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--looks", default=None, help="FILE.json:KEY with [[name, q6], ...] look poses")
    ap.add_argument("--panorama", action="store_true", help="joint-space look-around sweep (pass 2)")
    ap.add_argument("--out", default=str(HERE / f"scan-{time.strftime('%H%M%S')}-local"))
    try:
        asyncio.run(main(ap.parse_args()))
    except KeyboardInterrupt:
        print("interrupted (stop sent)")
