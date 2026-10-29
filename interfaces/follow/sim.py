"""interfaces.follow.sim: the follow loop on a virtual clock (no daemon, no camera, no arm).

* ``VelocityArm``: a fake arm that takes streamed joint velocities the way armctl's mock backend
  does: per-joint speed and acceleration caps, a deadman (a setpoint expires after its ttl and
  the arm decelerates to rest), ``envelope.clamp_step`` per tick, and refusal of anything
  outside the soft box.
* The ray-cast wrist camera from ``interfaces.wristcam.synth`` renders at ``fps``; each frame
  reaches the tracker ``latency`` s after it was captured, paired with q at capture time.
* The slow anchor (a ``CupFinder`` by default) takes ``anchor_delay`` s of virtual time.
* The cup moves along a path: ``static``, ``line`` (back and forth), ``circle``, ``stopgo``,
  ``jump`` (teleports once: the track must be lost and found again) or ``vanish`` (removed:
  the arm must stop and stay still).

``simulate(...)`` returns metrics: the centring error (where the optical axis hits the cup's
top plane vs the cup's true centre), the estimate error, losses, the lowest tip and table
margin, joint speeds, and whether anything left the soft box.

    .venv/bin/python -m interfaces.follow.sim --path circle --duration 12 --out /tmp/follow-sim
"""
from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import numpy as np

from armctl import envelope as envm
from armctl import kin

from interfaces.servo import table_margin
from interfaces.wristcam import synth
from interfaces.wristcam.locate import CupFinder, Hint, View

from .control import FollowConfig, FollowController
from .look import look_pose
from .track import AnchorWorker, QHistory, TrackConfig, Tracker


class VelocityArm:
    """Streamed-velocity fake arm (the mock backend's rules, deterministic)."""

    def __init__(self, q0, env: envm.Envelope = envm.DEFAULT):
        self.q = np.asarray(q0, float).copy()
        self.qd = np.zeros(6)
        self.env = env
        self.cmd = np.zeros(6)
        self.expires = -1.0
        self.violations: list[str] = []
        self.max_qd = np.zeros(6)

    def stream(self, qd, ttl: float, t: float) -> None:
        self.cmd = np.asarray(qd, float).copy()
        self.expires = t + min(ttl, 0.5)

    def step(self, dt: float, t: float) -> None:
        want = self.cmd if t < self.expires else np.zeros(6)
        want = np.clip(want, -self.env.vel_caps, self.env.vel_caps)
        dv = np.clip(want - self.qd, -self.env.acc_caps * dt, self.env.acc_caps * dt)
        self.qd = self.qd + dv
        self.qd, _ = self.env.clamp_step(self.q, self.qd, self.env.lookahead())
        self.q = self.q + self.qd * dt
        self.max_qd = np.maximum(self.max_qd, np.abs(self.qd))
        viol = self.env.check_point(self.q)
        if viol:
            self.violations.append(f"t={t:.2f}: " + "; ".join(v.message for v in viol))


# --------------------------------------------------------------------------- cup paths
def cup_path(kind: str, start, speed: float = 0.04, radius: float = 0.05, t_move: float = 1.5):
    """xy(t) in the table frame. Motion starts at ``t_move`` so the tracker locks on first."""
    x0, y0 = map(float, start)

    def f(t):
        s = max(0.0, t - t_move)
        if kind == "static":
            return x0, y0
        if kind == "line":           # back and forth along table -x (toward the base), 2 * radius long
            L = 2 * radius
            d = (speed * s) % (2 * L)
            d = d if d <= L else 2 * L - d
            return x0 - d, y0
        if kind == "circle":         # centre (x0 - radius, y0): starts at (x0, y0)
            w = speed / radius
            return x0 - radius + radius * math.cos(w * s), y0 + radius * math.sin(w * s)
        if kind == "stopgo":         # 1.5 s moving along y, 1.5 s still, ...
            n, r = divmod(s, 3.0)
            d = n * speed * 1.5 + speed * min(r, 1.5)
            return x0, y0 + d
        if kind == "jump":           # teleports 4 cm at t_move + 2 s
            return (x0 + 0.04, y0 + 0.02) if s > 2.0 else (x0, y0)
        if kind == "vanish":
            return x0, y0
        raise ValueError(f"unknown path {kind!r}")
    return f


@dataclass
class SimConfig:
    path: str = "circle"
    start: tuple = (0.0, -0.12)      # cup centre, table frame
    speed: float = 0.04              # m/s
    radius: float = 0.05
    duration: float = 10.0
    fps: float = 15.0
    latency: float = 0.08            # s, capture -> tracker
    assumed_latency: float | None = None   # what the tracker believes (default: the truth)
    lead: float = 0.05               # s ahead the controller asks the filter for (run.py's default)
    control_hz: float = 30.0
    ttl: float = 0.2
    anchor_delay: float = 0.4        # s of virtual time for one slow look
    size: tuple = (640, 360)
    joint_noise: float = 0.0005
    seed: int = 0
    vanish_at: float = 4.0           # path "vanish": the cup is removed at this time
    distractor: tuple | None = None  # a second, identical cup that stays put (table xy)
    settle: float = 3.0              # metrics ignore the first seconds (lock-on, catching up)


def simulate(sc: SimConfig | None = None, fc: FollowConfig | None = None, tc: TrackConfig | None = None,
             out: str | Path | None = None, frames_every: int = 0) -> dict:
    sc = sc or SimConfig()
    fc = fc or FollowConfig()
    rng = np.random.default_rng(sc.seed)
    cup = synth.Cup(tuple(sc.start))
    world = synth.World(cups=[cup] + ([synth.Cup(tuple(sc.distractor))] if sc.distractor else []), board=False)
    cam = synth.TRUE_CAMERA.scaled(sc.size)
    T_tc = synth.TRUE_T_TIP_CAM
    T_bt = world.T_base_table
    tc = tc or TrackConfig(height=cup.height, size=2 * cup.radius)
    path = cup_path(sc.path, sc.start, sc.speed, sc.radius)
    q0 = look_pose(sc.start, cup.height, fc.follow_h, T_tc, T_bt)
    if q0 is None:
        raise RuntimeError("no camera-down look pose over the start")
    arm = VelocityArm(q0)
    ctrl = FollowController(T_tc, T_bt, envm.DEFAULT.vel_caps, envm.DEFAULT.acc_caps, fc)
    trk = Tracker(tc)
    anchor = AnchorWorker(CupFinder(diameter=2 * cup.radius, height=cup.height), threaded=False)
    qh = QHistory(keep_s=3.0)
    assumed = sc.latency if sc.assumed_latency is None else sc.assumed_latency
    outp = Path(out) if out else None
    if outp:
        (outp / "frames").mkdir(parents=True, exist_ok=True)
    log = []
    frames_q: list = []          # (deliver_at, t_capture, frame)
    anchors_q: list = []         # (deliver_at, (t, det, frame, view))
    dt = 0.01
    next_frame = next_ctrl = 0.0
    n_frames = 0
    t = 0.0
    moved_after_vanish = 0.0
    q_at_vanish = None
    while t < sc.duration:
        # the world
        if sc.path == "vanish" and t >= sc.vanish_at:
            world.cups = [c for c in world.cups if c is not cup]
            if q_at_vanish is None:
                q_at_vanish = arm.q.copy()
        else:
            cup.xy = path(t)
        q_meas = arm.q + rng.normal(0, sc.joint_noise, 6) if sc.joint_noise else arm.q.copy()
        qh.add(t, q_meas)
        # the camera
        if t >= next_frame - 1e-9:
            next_frame += 1.0 / sc.fps
            n_frames += 1
            img = world.render(cam, kin.fk_vision(arm.q) @ T_tc, ss=1, seed=n_frames, noise=2.0)
            frames_q.append((t + sc.latency, t, img))
        # deliver frames to the tracker
        while frames_q and frames_q[0][0] <= t + 1e-9:
            _, t_cap_true, img = frames_q.pop(0)
            t_cap = t - assumed          # what the program knows: arrival time - assumed latency
            qv = qh.at(t_cap)
            view = View.from_q(qv, cam, T_tc, T_bt)
            trk.update(img, view, t_cap)
            if trk.wants_anchor(t_cap) and not anchors_q:
                hint = trk.hint(t_cap) if trk.kf is not None else Hint(tuple(sc.start), 0.05)   # the seed: near --expect
                anchor.submit(img, view, t_cap, hint)
                anchors_q.append((t + sc.anchor_delay, anchor.poll()))
            if outp and frames_every and n_frames % frames_every == 0:
                cv2.imwrite(str(outp / "frames" / f"f{n_frames:04d}.jpg"), _annotate(img, view, trk, t_cap, tc.height))
        while anchors_q and anchors_q[0][0] <= t + 1e-9:
            _, r = anchors_q.pop(0)
            if r is not None:
                ta, det, fa, va = r
                trk.anchor(det, fa, va, ta)
        # control
        if t >= next_ctrl - 1e-9:
            next_ctrl += 1.0 / sc.control_hz
            est = trk.estimate(t + sc.lead)
            row = {"t": round(t, 3), "state": trk.state}
            if est is not None:
                qd, info = ctrl.step(q_meas, [*est.xy, tc.height], 1.0 / sc.control_hz, v_table=est.v)
                arm.stream(qd, sc.ttl, t)
                row.update(e_pos=round(info["e_pos"], 4), e_ang=round(info["e_ang_deg"], 2), guard=info["guard"],
                           est=[round(float(v), 4) for v in est.xy])
            else:
                ctrl.reset()     # send nothing: the deadman stops the arm
            if cup in world.cups:
                truth = [*cup.xy, cup.height]
                row["true"] = [round(float(v), 4) for v in cup.xy]
                row["centring"] = round(ctrl.centring_error(arm.q, truth), 4)
                ne = trk.estimate(t)
                row["est_err"] = None if ne is None else round(float(np.hypot(*(ne.xy - np.asarray(cup.xy)))), 4)
            row["tip_z"] = round(float(kin.tip(arm.q)[2]), 4)
            row["margin"] = round(float(table_margin(arm.q)), 4)
            log.append(row)
        arm.step(dt, t)
        if q_at_vanish is not None:
            moved_after_vanish = max(moved_after_vanish, float(np.abs(arm.q - q_at_vanish).max()))
        t += dt
    res = _metrics(log, sc, trk, arm)
    if sc.path == "vanish":
        res["moved_after_vanish_rad"] = round(moved_after_vanish, 4)
        res["final_qd"] = float(np.abs(arm.qd).max())
    if outp:
        (outp / "log.jsonl").write_text("\n".join(json.dumps(r) for r in log) + "\n")
        (outp / "result.json").write_text(json.dumps({"sim": asdict(sc), "follow": asdict(fc), "track": asdict(tc),
                                                      **res}, indent=1, default=float) + "\n")
        _plot(log, outp / "plot.png", sc)
    return res


def _metrics(log, sc: SimConfig, trk: Tracker, arm: VelocityArm) -> dict:
    after = [r for r in log if r["t"] >= sc.settle and "centring" in r]
    cen = np.array([r["centring"] for r in after]) if after else np.array([np.nan])
    est = np.array([r["est_err"] for r in after if r.get("est_err") is not None]) if after else np.array([])
    tracked = np.mean([r["state"] in ("tracking", "coasting") for r in after]) if after else 0.0
    return {
        "path": sc.path,
        "centring_median_mm": round(float(np.median(cen)) * 1000, 2),
        "centring_p95_mm": round(float(np.percentile(cen, 95)) * 1000, 2),
        "centring_max_mm": round(float(np.max(cen)) * 1000, 2),
        "est_err_median_mm": round(float(np.median(est)) * 1000, 2) if len(est) else None,
        "est_err_p95_mm": round(float(np.percentile(est, 95)) * 1000, 2) if len(est) else None,
        "tracked_frac": round(float(tracked), 3),
        "min_tip_z": round(min(r["tip_z"] for r in log), 4),
        "min_margin": round(min(r["margin"] for r in log), 4),
        "max_qd": [round(float(v), 3) for v in arm.max_qd],
        "box_violations": arm.violations[:3],
        "track": trk.summary(),
    }


def _annotate(img, view: View, trk: Tracker, t: float, z: float) -> np.ndarray:
    out = img.copy()
    est = trk.estimate(t)
    if est is not None:
        uv = view.project_table([[est.xy[0], est.xy[1], z]])[0]
        if np.isfinite(uv).all():
            cv2.drawMarker(out, (int(uv[0]), int(uv[1])), (0, 255, 0), cv2.MARKER_CROSS, 18, 2)
    c = view.cam.K[:2, 2]
    cv2.circle(out, (int(c[0]), int(c[1])), 4, (255, 255, 255), 1)
    cv2.putText(out, f"t={t:.2f} {trk.state}", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return out


def _plot(log, path: Path, sc: SimConfig) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    t = [r["t"] for r in log if "centring" in r]
    c = [r["centring"] * 1000 for r in log if "centring" in r]
    e = [(r.get("est_err") or np.nan) * 1000 for r in log if "centring" in r]
    fig, ax = plt.subplots(figsize=(8, 3.2))
    ax.plot(t, c, label="centring error (optical axis vs cup centre)")
    ax.plot(t, e, label="estimate error", alpha=0.7)
    ax.axvline(sc.settle, color="grey", lw=0.8, ls=":")
    ax.set_xlabel("s")
    ax.set_ylabel("mm")
    ax.set_title(f"follow sim: {sc.path}, {sc.speed * 100:.0f} cm/s, {sc.fps:.0f} fps, latency {sc.latency * 1000:.0f} ms")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--path", default="circle", choices=["static", "line", "circle", "stopgo", "jump", "vanish"])
    ap.add_argument("--speed", type=float, default=0.04, help="cup speed, m/s")
    ap.add_argument("--radius", type=float, default=0.05)
    ap.add_argument("--duration", type=float, default=10.0)
    ap.add_argument("--fps", type=float, default=15.0)
    ap.add_argument("--latency", type=float, default=0.08)
    ap.add_argument("--assumed-latency", type=float, default=None)
    ap.add_argument("--lead", type=float, default=0.05)
    ap.add_argument("--follow-h", type=float, default=0.15)
    ap.add_argument("--v-max", type=float, default=0.06)
    ap.add_argument("--no-ff", action="store_true", help="no velocity feedforward")
    ap.add_argument("--frames-every", type=int, default=0)
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    sc = SimConfig(path=a.path, speed=a.speed, radius=a.radius, duration=a.duration, fps=a.fps, latency=a.latency,
                   assumed_latency=a.assumed_latency, lead=a.lead)
    fc = FollowConfig(follow_h=a.follow_h, v_max=a.v_max, ff_gain=0.0 if a.no_ff else 1.0)
    res = simulate(sc, fc, out=a.out, frames_every=a.frames_every)
    print(json.dumps(res, indent=1, default=float))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
