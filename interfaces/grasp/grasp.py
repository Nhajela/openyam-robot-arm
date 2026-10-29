"""interfaces.grasp.grasp: plan a top-down pick-and-place as a playable.

    from interfaces.grasp.grasp import Workcell, pick_place
    wc = Workcell.load()                                   # interfaces/grasp/workcell.json
    p, info = pick_place(wc, pick=(0.05, 0.10, 0), place=(0.10, -0.05, 0), height=0.04, width=0.03)

Positions are in the **table frame** (tag 0's frame from vision/detect_tags: origin at the tag
centre, z up, metres). ``Workcell.T_base_table`` maps them to the arm's base frame. Until a
real calibration exists it's a placeholder (tag 0 at 0.30 m in front of the base, axes aligned);
``fit_table_frame`` computes it from three or more points touched with the gripper tip.

The plan, as keyframes (each pose solved with armctl.kin.ik, fingers pointing down):

    ready -> above pick (open) -> down to grasp -> close -> lift -> above place
          -> down -> open -> lift -> ready

Segment times come from the same speed caps the validator uses, so the playable is feasible
by construction; ``interfaces.timeline.coach`` checks it anyway. The playable carries
``"min_tip_z"``: it touches the table on purpose, only at the grasp and release poses.

What we don't know yet (needs the real arm, see LOG.md):
* where exactly ``gripper_tip`` sits relative to the finger pads (``tip_to_pad``)
* the mapping from playable grip 0..1 to finger opening (assumed linear, 0 .. ``max_width``)
* grasp detection: the gripper motor stalls on an object before reaching its target, so a
  measured grip well above the commanded one after closing means "holding something"
  (``holding``). The mock backend can't show this.
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

from armctl import envelope as envm
from armctl import kin
from armctl import playable as pl

from ..timeline import coach

HERE = Path(__file__).resolve().parent
WORKCELL = HERE / "workcell.json"


@dataclass
class Workcell:
    T_base_table: list = field(default_factory=lambda: [[1, 0, 0, 0.30], [0, 1, 0, 0.0], [0, 0, 1, 0.0], [0, 0, 0, 1]])
    calibrated: bool = False
    max_width: float = 0.094      # m, finger opening at grip 1 (2 x 0.04695 URDF stroke)
    tip_to_pad: float = 0.0       # m, how far the pads' centre sits above gripper_tip
    travel_z: float = 0.14        # m above the table for moves between pick and place
    approach_z: float = 0.08      # m above the object top before descending
    min_tip_z: float = 0.01       # m: lowest the tip may go (only at grasp/release)
    open_margin: float = 0.02     # m wider than the object when approaching
    squeeze: float = 0.01         # m narrower than the object when closing
    notes: str = "placeholder: T_base_table not calibrated"
    chess: dict = field(default_factory=dict)   # interfaces.grasp.chess_arm.Board overrides
    spots: dict = field(default_factory=dict)   # named table spots, see spots.py
    objects: dict = field(default_factory=dict) # known objects (height, width), see spots.py

    @classmethod
    def load(cls, path: Path = WORKCELL) -> "Workcell":
        if path.exists():
            return cls(**json.loads(path.read_text()))
        return cls()

    def save(self, path: Path = WORKCELL) -> None:
        path.write_text(json.dumps(asdict(self), indent=2) + "\n")

    def to_base(self, p_table) -> np.ndarray:
        T = np.array(self.T_base_table, float)
        return (T @ np.r_[np.asarray(p_table, float), 1.0])[:3]

    def yaw_to_base(self, yaw_table: float) -> float:
        T = np.array(self.T_base_table, float)
        return yaw_table + math.atan2(T[1, 0], T[0, 0])

    def grip_for(self, width: float) -> float:
        return float(np.clip(width / self.max_width, 0.0, 1.0))


def fit_table_frame(points_table, points_base) -> np.ndarray:
    """Rigid transform base<-table (Kabsch) from >= 3 matched points, e.g. the tip touched to
    tag 0's centre and to marks 10 cm along its x and y axes (FK of the measured joints)."""
    A, B = np.asarray(points_table, float), np.asarray(points_base, float)
    ca, cb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, cb - R @ ca
    return T


def holding(grip_measured: float, grip_target: float, width_tol: float = 0.05) -> bool:
    """After a close: the fingers stopped well short of the target -> something is in them."""
    return grip_measured - grip_target > width_tol


class Unreachable(ValueError):
    pass


def _solve(xyz, seed, yaw: float | None, floor: float, what: str) -> np.ndarray:
    r = kin.ik(xyz, seed, down=True, check_table=False, corr=kin.vision_correction())
    tz, lz, ln = kin.clearance(r.q)
    if r.err_pos > 0.006 or r.err_rot > 0.1:
        raise Unreachable(f"{what}: can't reach ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}) m with the fingers "
                          f"down ({r.reason or 'not converged'})")
    if tz < floor - 1e-3 or lz < kin.MIN_LINK_Z:
        raise Unreachable(f"{what}: too low (tip {tz:.3f} m, {ln} {lz:.3f} m)")
    q = r.q.copy()
    if yaw is not None:
        q = _set_finger_yaw(q, yaw)
    return q


def _finger_dir(q) -> np.ndarray:
    """Direction the fingers close along, projected on the table (base frame)."""
    R = kin.fk_vision(q)[:3, :3]
    d = R[:, 1].copy()
    d[2] = 0
    return d / max(np.linalg.norm(d), 1e-9)


def _set_finger_yaw(q, yaw: float) -> np.ndarray:
    """Turn j6 so the fingers close along ``yaw`` (rad, base frame). A parallel gripper is
    symmetric under 180 deg; search j6 numerically and prefer the angle nearest 0."""
    env = envm.DEFAULT
    best, best_cost = q, None
    for j6 in np.linspace(env.soft_lower[5], env.soft_upper[5], 361):
        cand = q.copy()
        cand[5] = j6
        d = _finger_dir(cand)
        err = abs((math.atan2(d[1], d[0]) - yaw + math.pi / 2) % math.pi - math.pi / 2)
        cost = err + 0.01 * abs(j6)
        if best_cost is None or cost < best_cost:
            best, best_cost = cand, cost
    return best


def _seg_time(qa, qb, ga, gb, vcap, acap, gcap, slow: float = 1.0) -> float:
    need = float(coach.need_time(np.asarray(qb) - np.asarray(qa), "cubic", vcap, acap).max())
    gneed = abs(gb - ga) * 1.5 / gcap
    return round(max(0.6, max(need, gneed) * 1.25 * slow), 2)


def pick_place(wc: Workcell, pick, place, height: float, width: float, yaw: float = 0.0,
               place_yaw: float | None = None, name: str = "pick-place", caps: str = "default",
               grasp_frac: float = 0.6) -> tuple[dict, dict]:
    """Playable that picks an object (top-centre in table frame, ``height`` tall, ``width``
    across the fingers, long axis at ``yaw``) and puts it down at ``place`` (table frame,
    base of the object). Grasps at ``grasp_frac`` of the height from the bottom."""
    pick, place = np.asarray(pick, float), np.asarray(place, float)
    b_pick, b_place = wc.to_base(pick), wc.to_base(place)
    yaw_b = wc.yaw_to_base(yaw) + math.pi / 2          # close across the object's long axis
    place_yaw_b = wc.yaw_to_base(place_yaw if place_yaw is not None else yaw) + math.pi / 2
    z_grasp = max(wc.min_tip_z, pick[2] + height * grasp_frac - wc.tip_to_pad)
    z_release = max(wc.min_tip_z, place[2] + height * grasp_frac - wc.tip_to_pad + 0.003)
    top = max(pick[2], place[2]) + height
    z_above = max(wc.travel_z, top + wc.approach_z)
    g_open = wc.grip_for(width + wc.open_margin)
    g_closed = wc.grip_for(max(0.0, width - wc.squeeze))

    ready = np.array(envm.POSES["ready"], float)
    steps = []  # (label, q, grip, slow)
    q_above_pick = _solve(b_pick[:2].tolist() + [wc.to_base([0, 0, z_above])[2]], ready, yaw_b, wc.min_tip_z, "above pick")
    q_grasp = _solve(b_pick[:2].tolist() + [wc.to_base([0, 0, z_grasp])[2]], q_above_pick, yaw_b, wc.min_tip_z, "grasp")
    q_grasp[5] = q_above_pick[5]
    q_above_place = _solve(b_place[:2].tolist() + [wc.to_base([0, 0, z_above])[2]], q_above_pick, place_yaw_b, wc.min_tip_z, "above place")
    q_release = _solve(b_place[:2].tolist() + [wc.to_base([0, 0, z_release])[2]], q_above_place, place_yaw_b, wc.min_tip_z, "release")
    q_release[5] = q_above_place[5]
    steps += [("start", ready, 0.0, 1.0),
              ("above pick, open", q_above_pick, g_open, 1.0),
              ("down to grasp", q_grasp, g_open, 2.0),
              ("close", q_grasp, g_closed, 1.0),
              ("lift", q_above_pick, g_closed, 2.0),
              ("above place", q_above_place, g_closed, 1.0),
              ("down to release", q_release, g_closed, 2.0),
              ("open", q_release, g_open, 1.0),
              ("lift", q_above_place, g_open, 1.5),
              ("ready", ready, 0.0, 1.0)]

    p = {"format": pl.FORMAT, "version": "1.0", "name": pl.slugify(name), "author": "planner:interfaces.grasp",
         "created": pl.now_iso(), "space": "joint", "start": "ready", "caps": caps,
         "min_tip_z": wc.min_tip_z, "keyframes": []}
    vcap, acap, gcap, _ = coach.caps_for(p)
    t = 0.0
    labels = []
    for k, (label, q, g, slow) in enumerate(steps):
        if k:
            _, qa, ga, _ = steps[k - 1]
            t += _seg_time(qa, q, ga, g, vcap, acap, gcap, slow)
        kf = {"t": round(t, 2), "q": [round(float(x), 4) for x in q], "grip": round(g, 3)}
        if k:
            kf["ease"] = "cubic"
        p["keyframes"].append(kf)
        labels.append(f"{t:5.2f}s {label}")
    p["notes"] = (f"pick table({pick[0]:.3f},{pick[1]:.3f},{pick[2]:.3f}) -> place table({place[0]:.3f},"
                  f"{place[1]:.3f},{place[2]:.3f}); object {height * 100:.1f} cm tall, {width * 100:.1f} cm wide. "
                  + ("" if wc.calibrated else "UNCALIBRATED workcell (placeholder table frame)."))
    info = {"steps": labels, "grip_open": g_open, "grip_closed": g_closed, "z_grasp": round(z_grasp, 3),
            "z_above": round(z_above, 3), "base_pick": b_pick.round(3).tolist(), "base_place": b_place.round(3).tolist()}
    return p, info
