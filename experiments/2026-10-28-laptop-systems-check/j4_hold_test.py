#!/usr/bin/env python3
"""j4 holding-error test: hold a few poses and log how far j4 sits from its target.

Question: is j4's holding error (kp 10 on the wrist) explained by the gravity torque the
DimOS model gets wrong (error tracks the model torque), or is it a constant offset
(encoder zero, friction)? See LOG.md 12:55.

For each pose (ready with j4 at -0.3, 0, +0.3, +0.7), this moves there with move_joints
through the armctl daemon (soft limits, speed caps), holds for --hold seconds, and logs at 10 Hz:
  - the daemon's joint state (what DimOS measures)
  - optionally, from the read-only lab console (--console), j4's decoded torque and temperatures
It ends with home. Ctrl-C sends stop (support the arm).

    # sim (validates the script; the mock arm doesn't sag)
    ZENOH_SCOUT_ADDR=224.0.0.124:7450 .venv/bin/python experiments/2026-10-28-laptop-systems-check/j4_hold_test.py \
        --url ws://127.0.0.1:8791/armctl --hold 10
    # real arm (only with the person at the arm agreeing): daemon on 8091, console on 8090
    .venv/bin/python experiments/2026-10-28-laptop-systems-check/j4_hold_test.py --console http://127.0.0.1:8090 --hold 120
"""
from __future__ import annotations

import argparse
import asyncio
import csv
import json
import sys
import time
import urllib.request
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))
from interfaces.client import ArmClient, Refused  # noqa: E402

GRAVITY_URDF = Path.home() / ".local/share/dimos/repo/data/yam_description/urdf/yam_gripper_gravity.urdf"
READY = [0.0, 0.5, 0.8, 0.0, 0.0, 0.0]
J4_OFFSETS = (0.0, -0.3, 0.3, 0.7)
KP_J4 = 10.0  # DimOS default (dimos/robot/manipulators/openyam/config.py); --kp4 overrides


def model_tau_j4(q6) -> float | None:
    """j4 gravity torque that DimOS feeds forward (pinocchio on the same URDF)."""
    try:
        import pinocchio as pin
    except ImportError:
        return None
    m = pin.buildModelFromUrdf(str(GRAVITY_URDF))
    q = np.zeros(m.nq)
    q[:6] = q6
    return float(pin.computeGeneralizedGravity(m, m.createData(), q)[3])


def console_j4(base: str | None) -> dict:
    if not base:
        return {}
    try:
        with urllib.request.urlopen(f"{base}/api/state", timeout=0.3) as r:
            s = json.load(r)
        m = s["motors"][3]
        return {"c_pos": m.get("pos"), "c_tau": m.get("tau"), "t_mos": m.get("t_mos"),
                "t_rotor": m.get("t_rotor"), "ext_ctrl": s["bus"].get("external_controller")}
    except Exception:
        return {}


async def hold(arm: ArmClient, name: str, target, seconds: float, console: str | None, rows: list) -> None:
    t0 = time.monotonic()
    while (t := time.monotonic() - t0) < seconds:
        q = arm.q
        row = {"pose": name, "t": round(t, 2), **{f"q{i+1}": round(float(q[i]), 4) for i in range(6)},
               "j4_err": round(float(q[3] - target[3]), 4), **console_j4(console)}
        rows.append(row)
        if len(rows) % 50 == 1:
            print(f"  {name} t={t:5.1f}s j4={q[3]:+.3f} err={row['j4_err']:+.3f} "
                  f"tau={row.get('c_tau')} rotor={row.get('t_rotor')}C", flush=True)
        await asyncio.sleep(0.1)


def summarize(rows: list, targets: dict) -> list[str]:
    out = [f"{'pose':12} {'model tau4':>10} {'err first5s':>11} {'err last5s':>10} {'kp*err':>7} "
           f"{'meas tau4':>9} {'rotor C':>9}"]
    for name, target in targets.items():
        r = [x for x in rows if x["pose"] == name]
        if not r:
            continue
        tend = r[-1]["t"]
        first = np.mean([x["j4_err"] for x in r if x["t"] <= 5])
        last = np.mean([x["j4_err"] for x in r if x["t"] >= tend - 5])
        taus = [x["c_tau"] for x in r if x.get("c_tau") is not None and x["t"] >= tend - 5]
        temps = [x["t_rotor"] for x in r if x.get("t_rotor") is not None]
        mt = model_tau_j4(target)
        out.append(f"{name:12} {mt if mt is None else round(mt, 3):>10} {first:+11.4f} {last:+10.4f} "
                   f"{KP_J4 * last:+7.3f} {np.mean(taus) if taus else float('nan'):9.3f} "
                   f"{(str(temps[0]) + '->' + str(temps[-1])) if temps else '-':>9}")
    return out


async def main(a) -> None:
    targets = {}
    for off in J4_OFFSETS:
        q = list(READY)
        q[3] = off
        targets[f"j4={off:+.1f}"] = q
    rows: list = []
    stamp = time.strftime("%Y%m%d-%H%M%S")
    async with ArmClient(a.url, program="script", name="j4-hold-test") as arm:
        await arm.wait_state()
        print(f"backend={arm.backend} start q={np.round(arm.q, 3).tolist()}", flush=True)
        await arm.acquire(timeout=10)
        try:
            for name, target in targets.items():
                print(f"-> {name} {target}", flush=True)
                try:
                    await arm.do("move_joints", q=target, timeout=90)
                except Refused as e:  # arrival check: the arm still holds where it got to
                    print(f"   (verify refused, still holding: {e})", flush=True)
                await hold(arm, name, target, a.hold, a.console, rows)
            print("-> home", flush=True)
            await arm.do("home", timeout=90)
        except (asyncio.CancelledError, KeyboardInterrupt):
            await arm.stop()
            raise
        finally:
            out = HERE / "logs" / f"j4-hold-{stamp}.csv"
            out.parent.mkdir(exist_ok=True)
            keys = sorted({k for r in rows for k in r}, key=lambda k: (k not in ("pose", "t"), k))
            with out.open("w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=keys)
                w.writeheader()
                w.writerows(rows)
            print(f"wrote {out} ({len(rows)} rows)")
            print("\n".join(summarize(rows, targets)))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default="ws://127.0.0.1:8091/armctl")
    ap.add_argument("--hold", type=float, default=120.0, help="seconds to hold each pose")
    ap.add_argument("--kp4", type=float, default=None, help="j4 kp the coordinator runs (for the kp*err column; default 10)")
    ap.add_argument("--console", default=None, help="lab console base URL for j4 torque/temperature (read-only)")
    args = ap.parse_args()
    if args.kp4:
        KP_J4 = args.kp4
    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        print("interrupted (stop sent)")
