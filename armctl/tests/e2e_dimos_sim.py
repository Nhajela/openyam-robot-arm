#!/usr/bin/env python3
"""End-to-end check of DimosBackend against DimOS ``--simulation`` (a script; not collected by pytest).

SIMULATION ONLY. Run from the repo root, all on the SIM bus (CLAUDE.md), DimOS venv:

    ZENOH_SCOUT_ADDR=224.0.0.124:7450 ../dimos-venv/bin/python -m armctl.dimos_coordinator --simulation
    ZENOH_SCOUT_ADDR=224.0.0.124:7450 ../dimos-venv/bin/python -m armctl.daemon --backend dimos \\
        --auto-approve --port 8791 --log ''
    ZENOH_SCOUT_ADDR=224.0.0.124:7450 ../dimos-venv/bin/python armctl/tests/e2e_dimos_sim.py [--url ws://127.0.0.1:8791/armctl]

It refuses to run unless the daemon reports backend "dimos" with adapter "sim", because it
presses operator buttons itself (Play, E-STOP) and moves the arm through every path:
ready, move_joints, a playable (validate -> play -> confirm), home, refusals, Cartesian
streaming from ready (+x moves the tip +x by armctl.kin FK), the deadman, joint-velocity
streaming, gripper, stop mid-trajectory, E-STOP and reset.

With ``--probe`` (default when dimos is importable) it also subscribes READ-ONLY to the
coordinator's ``/coordinator_joint_state`` (100 Hz) to time latencies and stops precisely;
it never publishes on the DimOS bus.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys
import threading
import time
import urllib.request
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from armctl import kin  # noqa: E402
from armctl.backend import plan_move  # noqa: E402
from interfaces.client import ArmClient, Refused  # noqa: E402

SIM_BUS = "224.0.0.124:7450"
RESULTS: list[tuple[bool, str]] = []
NUMBERS: dict[str, float] = {}
READY = np.array([0.0, 0.5, 0.8, 0.0, 0.0, 0.0])


def check(ok: bool, what: str) -> None:
    RESULTS.append((bool(ok), what))
    print(("PASS " if ok else "FAIL ") + what, flush=True)


def num(key: str, value: float, unit: str) -> None:
    NUMBERS[key] = float(value)
    print(f"  {key} = {value:.4g} {unit}", flush=True)


class Probe:
    """Read-only 100 Hz joint-state recorder straight from DimOS (no publishing)."""

    def __init__(self) -> None:
        if os.environ.get("ZENOH_SCOUT_ADDR", "").strip() != SIM_BUS:
            raise SystemExit(f"--probe needs ZENOH_SCOUT_ADDR={SIM_BUS} (the sim bus)")
        os.environ.setdefault("DIMOS_TRANSPORT", "zenoh")
        from dimos.core.transport_factory import make_transport
        from dimos.msgs.sensor_msgs.JointState import JointState

        self.lock = threading.Lock()
        self.buf: list[tuple[float, np.ndarray]] = []
        self.tr = make_transport("/coordinator_joint_state", JointState)
        self.tr.subscribe(self._on)

    def _on(self, m) -> None:  # noqa: ANN001
        d = dict(zip(m.name, m.position))
        if all(f"yam_joint{i}" in d for i in range(1, 7)):
            q = np.array([d[f"yam_joint{i}"] for i in range(1, 7)], float)
            with self.lock:
                self.buf.append((time.monotonic(), q))
                if len(self.buf) > 20000:
                    del self.buf[:10000]

    def latest(self) -> np.ndarray:
        with self.lock:
            return self.buf[-1][1].copy()

    def since(self, t0: float) -> list[tuple[float, np.ndarray]]:
        with self.lock:
            return [x for x in self.buf if x[0] >= t0]

    def first_motion(self, t0: float, q0: np.ndarray, eps: float = 1e-4) -> float | None:
        for t, q in self.since(t0):
            if np.max(np.abs(q - q0)) > eps:
                return t
        return None

    def still_after(self, t0: float, speed: float = 0.01, window: float = 0.1
                    ) -> tuple[float | None, np.ndarray | None]:
        """First time after t0 from which every joint moves slower than ``speed`` rad/s (the
        daemon's "moving" threshold) over ``window`` s, and stays so for the rest of the record."""
        xs = self.since(t0)
        still_from = None
        for i in range(len(xs)):
            j = i
            while j < len(xs) and xs[j][0] - xs[i][0] < window:
                j += 1
            if j >= len(xs):
                break
            v = np.max(np.abs(xs[j][1] - xs[i][1])) / (xs[j][0] - xs[i][0])
            if v < speed:
                if still_from is None:
                    still_from = i
            else:
                still_from = None
        if still_from is None:
            return None, None
        return xs[still_from][0], xs[still_from][1]


def operator(base: str, path: str, body: dict | None = None) -> dict:
    req = urllib.request.Request(base + path, data=json.dumps(body or {}).encode(),
                                 headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read() or b"{}")


async def wait_still(arm: ArmClient, timeout: float = 5.0) -> dict:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        st = await arm.wait_state()
        if not st.get("moving") and st.get("mode") == "hold":
            return st
    return arm.state


async def stream_for(arm: ArmClient, seconds: float, *, v: dict | None = None, qd=None,
                     hz: float = 25.0, ttl_ms: int = 200) -> tuple[float, float]:
    """Send setpoints for ``seconds``; returns (monotonic first send, monotonic last send)."""
    t_first = time.monotonic()
    t_last = t_first
    end = t_first + seconds
    while time.monotonic() < end:
        msg = {"op": "stream", "ttl_ms": ttl_ms}
        if v is not None:
            msg["v"] = v
        else:
            msg["qd"] = list(map(float, qd))
        await arm.send(msg)
        t_last = time.monotonic()
        await asyncio.sleep(1.0 / hz)
    return t_first, t_last


async def timed_move(arm: ArmClient, probe: Probe | None, label: str, cmd: str, **args) -> np.ndarray:
    q0 = arm.q.copy()
    t0 = time.monotonic()
    done = await arm.do(cmd, timeout=90.0, **args)
    dt = time.monotonic() - t0
    st = await arm.wait_state()
    q = np.array(st["q"])
    check(done.get("op") == "done", f"{label}: done in {dt:.2f} s")
    return q


async def run(url: str, use_probe: bool) -> int:
    base = url.replace("ws://", "http://").rsplit("/armctl", 1)[0]
    probe = Probe() if use_probe else None
    async with ArmClient(url, program="e2e-dimos", name="e2e dimos sim") as arm:
        st = await arm.wait_state()
        if arm.backend != "dimos" or st.get("adapter") != "sim":
            print(f"refusing: daemon backend={arm.backend!r} adapter={st.get('adapter')!r}; this script "
                  "is for DimosBackend against DimOS --simulation only", file=sys.stderr)
            return 2
        check(True, f"daemon backend dimos, adapter {st['adapter']} ({st.get('adapter_type')}), faults {st.get('faults')}")
        await arm.acquire(timeout=10)
        check(arm.has_lease, "lease granted (auto-approve)")

        # start from home, whatever the sim was doing
        if not np.allclose(arm.q, 0, atol=0.02):
            await arm.do("home", timeout=90)

        # ---- refusals that must not move anything
        try:
            await arm.send({"op": "stream", "v": {"x": 0.02}, "ttl_ms": 200})
            await asyncio.sleep(0.3)
            r = [m for m in arm.refusals if "unfolded" in m.get("reason", "")]
            check(bool(r), "twist from folded home refused: " + (r[-1]["reason"][:90] if r else "NOT refused"))
        except Refused as e:
            check("unfolded" in str(e), f"twist from home refused: {e}")
        try:
            await arm.do("move_joints", q=[0, 0.5, 0.8, 1.6, 0, 0])
            check(False, "j4=1.6 refused (soft limit)")
        except Refused as e:
            check("j4" in str(e), f"j4=1.6 refused: {str(e)[:80]}")
        try:
            await arm.do("relax", wait=False)
            check(False, "relax refused")
        except Refused as e:
            check("disable_all" in str(e), f"relax refused: {str(e)[:100]}")

        # ---- discrete moves
        q = await timed_move(arm, probe, "ready", "ready")
        num("ready.final_error_rad", float(np.max(np.abs(q - READY))), "rad")
        tgt = np.array([0.3, 0.7, 1.0, 0.2, -0.2, 0.3])
        q0 = arm.q.copy()
        plan = plan_move(q0, tgt)
        t_start = time.monotonic()
        q = await timed_move(arm, probe, "move_joints", "move_joints", q=tgt.tolist())
        num("move_joints.final_error_rad", float(np.max(np.abs(q - tgt))), "rad")
        if probe is not None:  # tracking: measured vs our 100 Hz plan, at the best-fitting start lag
            xs = probe.since(t_start)
            ts = np.array([t for t, _ in xs])
            qs = np.array([qq for _, qq in xs])
            tp = np.arange(len(plan.q)) * plan.dt
            best = None
            for lag in np.arange(0.0, 0.3, 0.002):
                m = (ts - t_start - lag >= 0) & (ts - t_start - lag <= tp[-1])
                if m.sum() < 10:
                    continue
                ref = np.stack([np.interp(ts[m] - t_start - lag, tp, plan.q[:, i]) for i in range(6)], 1)
                e = float(np.max(np.abs(qs[m] - ref)))
                if best is None or e < best[1]:
                    best = (lag, e)
            if best:
                num("move_joints.start_lag_s", best[0], "s (ack-free: from sending do to the fitted plan start)")
                num("move_joints.max_tracking_error_rad", best[1], "rad (vs our 100 Hz plan at that lag)")

        # ---- playable: validate -> play -> operator confirm (sim + auto-approve)
        r = await arm.request({"op": "playable.validate", "name": "wave"}, until="playable.result")
        check(bool(r.get("ok")), f"playable wave validates ({len(r.get('violations', []))} violations)")
        pj = json.loads((REPO / "playables" / "wave.json").read_text())
        mid = 9001
        fut_t = time.monotonic()
        await arm.send({"op": "playable.play", "name": r["name"], "id": mid})
        await asyncio.sleep(0.3)
        res = operator(base, "/playable/confirm", {"name": r["name"]})
        check(bool(res.get("ok")), f"operator confirm: {res}")
        done = await arm._wait_op("done", mid, 120.0)
        st = await arm.wait_state()
        last = pj.get("keyframes", [{}])[-1]
        print(f"  wave: done {done.get('cmd')} after {time.monotonic() - fut_t:.1f} s; q {np.round(st['q'], 3).tolist()}")
        check(done.get("op") == "done", "playable wave played to done")
        if isinstance(last, dict) and "q" in last:
            num("wave.final_error_rad", float(np.max(np.abs(np.array(st["q"]) - np.array(last["q"])))), "rad")

        # ---- Cartesian streaming from ready
        await arm.do("ready", timeout=90)
        await asyncio.sleep(0.3)
        q0 = probe.latest() if probe is not None else arm.q.copy()
        p0 = kin.tip(q0)
        t_first, t_last = await stream_for(arm, 2.0, v={"x": 0.02})
        st = await wait_still(arm, 3.0)
        await asyncio.sleep(0.5)
        st = await arm.wait_state()
        q1 = np.array(st["q"])
        d = kin.tip(q1) - p0
        print(f"  +x 0.02 m/s for 2 s: tip moved {np.round(d * 1000, 1).tolist()} mm")
        check(d[0] > 0.02 and abs(d[1]) < 0.005 and abs(d[2]) < 0.008, "+x twist moves the tip +x (FK)")
        num("twist_x.distance_m", float(d[0]), "m (commanded 0.040 m)")
        R0, R1 = kin.fk(q0)[:3, :3], kin.fk(q1)[:3, :3]
        num("twist_x.orientation_change_rad", float(np.linalg.norm(kin.rot_log(R1 @ R0.T))), "rad")
        if probe is not None:
            tm = probe.first_motion(t_first, q0)
            if tm is not None:
                num("twist.latency_first_motion_s", tm - t_first, "s")
            ts, _ = probe.still_after(t_last)
            if ts is not None:
                num("twist.deadman_stop_s", ts - t_last, "s after the last setpoint (ttl 200 ms)")
        await asyncio.sleep(0.6)
        q0 = arm.q.copy()
        p0 = kin.tip(q0)
        await stream_for(arm, 1.5, v={"z": 0.02})
        st = await wait_still(arm, 3.0)
        d = kin.tip(np.array(st["q"])) - p0
        print(f"  +z 0.02 m/s for 1.5 s: tip moved {np.round(d * 1000, 1).tolist()} mm")
        check(d[2] > 0.015 and abs(d[0]) < 0.006 and abs(d[1]) < 0.005, "+z twist moves the tip +z (FK)")

        # explicit stop during a twist stream
        q0 = arm.q.copy()
        task = asyncio.create_task(stream_for(arm, 3.0, v={"x": -0.02}))
        await asyncio.sleep(1.0)
        task.cancel()
        t_stop = time.monotonic()
        await arm.stop()
        st = await wait_still(arm, 3.0)
        await asyncio.sleep(0.5)
        if probe is not None:
            ts, _ = probe.still_after(t_stop)
            if ts is not None:
                num("twist.stop_to_still_s", ts - t_stop, "s")
        check(st.get("mode") == "hold", f"twist stopped by do stop (mode {st.get('mode')})")

        # ---- joint-velocity streaming (rolling trajectories)
        await arm.do("ready", timeout=90)
        await asyncio.sleep(0.3)
        q0 = probe.latest() if probe is not None else arm.q.copy()
        t_first, t_last = await stream_for(arm, 1.5, qd=[0.2, 0, 0, 0, 0, 0])
        st = await wait_still(arm, 3.0)
        await asyncio.sleep(0.5)
        dq = np.array(st["q"]) - q0
        print(f"  qd j1 0.2 rad/s for 1.5 s: dq {np.round(dq, 3).tolist()}")
        check(0.2 < dq[0] < 0.36 and np.max(np.abs(dq[1:])) < 1e-3, "joint stream moves j1 only")
        if probe is not None:
            tm = probe.first_motion(t_first, q0)
            if tm is not None:
                num("jstream.latency_first_motion_s", tm - t_first, "s")
            xs = [x for x in probe.since(t_first + 0.4) if x[0] <= t_last - 0.1]  # cruise phase
            if len(xs) > 20:
                ts = np.array([t for t, _ in xs])
                j1 = np.array([qq[0] for _, qq in xs])
                a, b = np.polyfit(ts - ts[0], j1, 1)
                num("jstream.cruise_speed_rad_s", float(a), "rad/s (cmd 0.2; regression over the cruise)")
                num("jstream.ripple_rad", float(np.max(np.abs(j1 - (a * (ts - ts[0]) + b)))), "rad (max deviation from that line)")
                d = np.diff(j1)
                num("jstream.zero_step_fraction", float(np.mean(np.abs(d) < 1e-6)), "(fraction of 100 Hz samples without motion)")
            ts, _ = probe.still_after(t_last)
            if ts is not None:
                num("jstream.deadman_stop_s", ts - t_last, "s after the last setpoint (ttl 200 ms)")
        # guard: j5 towards its soft limit (1.471) from 0: it must stop at the box
        await arm.do("move_joints", q=[0.2, 0.5, 0.8, 0.0, 1.3, 0.0], timeout=90)
        n_ref = len(arm.refusals)
        await stream_for(arm, 1.5, qd=[0, 0, 0, 0, 0.3, 0])
        st = await wait_still(arm, 3.0)
        g = [m for m in arm.refusals[n_ref:] if m.get("stream")]
        print(f"  j5 streamed to {st['q'][4]:.3f} (soft limit 1.471); guard refusals {len(g)}")
        check(st["q"][4] <= 1.4708 + 1e-3 and bool(g), "joint stream guard stops j5 at the soft limit and says so")

        # ---- gripper
        for w in (1.0, 0.3, 0.0):
            await arm.do("gripper", width=w, wait=False)
            end = time.monotonic() + 3
            while time.monotonic() < end and abs(arm.state.get("grip", -1) - w) > 0.02:
                await arm.wait_state()
            check(abs(arm.state.get("grip", -1) - w) <= 0.02, f"gripper width {w}: grip {arm.state.get('grip')}")

        # ---- stop mid-trajectory
        await arm.do("ready", timeout=90)
        tgt = np.array([1.0, 0.5, 0.8, 0.0, 0.0, 0.0])
        await arm.do("move_joints", q=tgt.tolist(), wait=False)
        await asyncio.sleep(1.5)
        qa = arm.q.copy()
        t_stop = time.monotonic()
        await arm.stop()
        st = await wait_still(arm, 4.0)
        await asyncio.sleep(0.5)
        q1 = np.array(st["q"])
        print(f"  stopped at j1 {q1[0]:.3f} (was {qa[0]:.3f} when stop was sent; target 1.0)")
        check(q1[0] < 0.95 and st.get("mode") == "hold", "stop mid-trajectory halts before the target")
        intr = [m for m in arm.refusals if "interrupted" in m.get("reason", "")]
        check(bool(intr), "the interrupted move is reported: " + (intr[-1]["reason"] if intr else "none"))
        if probe is not None:
            ts, qs = probe.still_after(t_stop)
            if ts is not None:
                num("traj.stop_to_still_s", ts - t_stop, "s (braking along the path at the accel cap)")
                num("traj.stop_overrun_rad", float(qs[0] - qa[0]), "rad on j1 after stop was sent")

        # ---- E-STOP while moving, then reset and move again
        await arm.do("move_joints", q=[-0.8, 0.5, 0.8, 0.0, 0.0, 0.0], wait=False)
        await asyncio.sleep(1.5)
        check(bool(arm.state.get("moving")), "moving when E-STOP is pressed")
        t_e = time.monotonic()
        operator(base, "/estop")
        await asyncio.sleep(0.6)
        st = await arm.wait_state()
        qe = np.array(st["q"])
        await asyncio.sleep(0.5)
        st2 = await arm.wait_state()
        check(np.max(np.abs(np.array(st2["q"]) - qe)) < 1e-3 and st2.get("estop"), "E-STOP halts and latches")
        if probe is not None:
            ts, _ = probe.still_after(t_e)
            if ts is not None:
                num("estop.to_still_s", ts - t_e, "s")
        operator(base, "/estop/reset")
        await arm.acquire(timeout=10)
        q = await timed_move(arm, probe, "ready_after_estop", "ready")
        check(np.max(np.abs(q - READY)) < 0.02, "motion works again after E-STOP reset")
        await arm.do("gripper", width=0.5, wait=False)
        await asyncio.sleep(0.5)
        check(abs(arm.state.get("grip", -1) - 0.5) <= 0.02, f"gripper after E-STOP reset: {arm.state.get('grip')}")

        # ---- home
        q = await timed_move(arm, probe, "home", "home")
        num("home.final_error_rad", float(np.max(np.abs(q))), "rad")
        st = await arm.wait_state()
        check(not st.get("faults"), f"no backend faults at the end ({st.get('faults')})")

    bad = [w for ok, w in RESULTS if not ok]
    print(f"\n{len(RESULTS) - len(bad)}/{len(RESULTS)} passed")
    print("numbers:", json.dumps({k: round(v, 5) for k, v in NUMBERS.items()}))
    return 1 if bad else 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default="ws://127.0.0.1:8791/armctl")
    ap.add_argument("--no-probe", action="store_true", help="don't subscribe to DimOS joint state for timings")
    a = ap.parse_args()
    use_probe = not a.no_probe
    if use_probe:
        try:
            import dimos  # noqa: F401
        except ImportError:
            use_probe = False
    return asyncio.run(run(a.url, use_probe))


if __name__ == "__main__":
    sys.exit(main())
