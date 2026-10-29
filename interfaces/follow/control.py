"""interfaces.follow.control: keep the wrist camera over a tracked object (joint-velocity servo).

Position-based visual servoing on the tracker's estimate:

* **Where the camera should be:** ``follow_h`` above the object's top, straight over it. The
  object is clamped to a reach annulus around the base (``reach``). Outside it the camera stops
  at the edge and only turns to look.
* **Where it should look:** at the object (look-at), tilted at most ``max_tilt`` from straight
  down. The wrist centres the object fast while the position catches up, and once the position
  has caught up, look-at *is* straight down.
* **Task:** 5-D, the camera point's velocity (3) and the optical axis's turn rate (2, the parts
  perpendicular to the axis). Roll about the optical axis is free; a null-space term holds j6
  where it started (the camera stays in the arm's vertical plane, like ``engine.camera_over``).
  Damped least squares on the geometric Jacobian (``armctl.kin.jacobian_geometric``, tip point
  moved to the camera centre).
* **Feedforward:** the object's estimated velocity, so a moving object is followed without a
  steady lag.
* **Limits, in order:** Cartesian speed and turn-rate caps; the joint caps (scaled down by
  ``speed_scale``; translation first, then as much of the axis turn as still fits); the acceleration caps per tick; ``envelope.clamp_step`` with
  the braking lookahead (never leave the soft box); ``interfaces.servo.table_guard`` (never dip
  toward the table). The daemon checks every streamed setpoint again.

``FollowController.step(q, target, dt)`` returns (qd, info). It never sends anything: the caller
streams qd through the daemon (deadman ttl), or into the simulated arm.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from armctl import envelope as envm
from armctl import kin

from interfaces.servo import table_guard


@dataclass
class FollowConfig:
    follow_h: float = 0.15            # m, camera above the object's top
    max_tilt_deg: float = 25.0        # look-at tilt from straight down
    kp_pos: float = 1.8               # 1/s, position error -> camera speed
    kp_rot: float = 2.5               # 1/s, axis error (rad) -> turn rate
    v_max: float = 0.06               # m/s, camera speed cap (real arm: start at 0.03)
    w_max: float = 0.6                # rad/s, turn-rate cap
    ff_gain: float = 1.0              # feedforward of the object's velocity (0 = pure feedback)
    deadband_pos: float = 0.002       # m: closer than this counts as there
    deadband_rot_deg: float = 0.8
    reach: tuple = (0.16, 0.44)       # m, camera xy distance from the base axis
    damping: float = 0.04             # DLS damping
    null_gain: float = 1.0            # 1/s, j6 back to where it started
    speed_scale: float = 0.6          # fraction of the envelope's joint caps
    min_margin: float = 0.02          # m above the table rule before table_guard kicks in early


def _skew(r) -> np.ndarray:
    return np.array([[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]], float)


def _perp_basis(a) -> np.ndarray:
    """2x3: two unit vectors perpendicular to a (and to each other)."""
    a = a / np.linalg.norm(a)
    h = np.array([1.0, 0, 0]) if abs(a[0]) < 0.9 else np.array([0, 1.0, 0])
    e1 = np.cross(a, h)
    e1 /= np.linalg.norm(e1)
    return np.vstack([e1, np.cross(a, e1)])


def _cap(v, vmax: float) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v * (vmax / n) if n > vmax > 0 else v


class FollowController:
    def __init__(self, T_tip_cam, T_base_table, vel_caps, acc_caps, cfg: FollowConfig | None = None,
                 envelope: envm.Envelope = envm.DEFAULT):
        self.T_tip_cam = np.asarray(T_tip_cam, float)
        self.T_base_table = np.asarray(T_base_table, float)
        self.cfg = cfg or FollowConfig()
        self.vel_caps = np.asarray(vel_caps, float) * self.cfg.speed_scale
        self.acc_caps = np.asarray(acc_caps, float)
        self.env = envelope
        self.qd = np.zeros(6)
        self.j6_home: float | None = None

    def reset(self) -> None:
        """Forget the commanded velocity (the arm is stopping: the deadman or a hold)."""
        self.qd = np.zeros(6)

    # ------------------------------------------------------------------ geometry
    def table_to_base(self, xyz_table) -> np.ndarray:
        return (self.T_base_table @ np.r_[np.asarray(xyz_table, float), 1.0])[:3]

    def camera(self, q) -> np.ndarray:
        """T_base_cam (vision-corrected FK, as the tracker sees the world)."""
        return kin.fk_vision(q) @ self.T_tip_cam

    def desired(self, target_base, q) -> tuple[np.ndarray, np.ndarray, bool]:
        """(camera position, unit optical axis, clamped?) for an object top at ``target_base``."""
        c = self.cfg
        T = self.camera(q)
        p_c = T[:3, 3]
        xy = np.asarray(target_base[:2], float)
        r = float(np.linalg.norm(xy))
        clamped = False
        if r < c.reach[0] or r > c.reach[1]:
            xy = xy * (np.clip(r, *c.reach) / max(r, 1e-6))
            clamped = True
        z_up = float(self.T_base_table[2, 2]) if abs(self.T_base_table[2, 2]) > 0.5 else 1.0
        p_des = np.r_[xy, float(target_base[2]) + c.follow_h * z_up]
        look = np.asarray(target_base, float) - p_c
        look /= np.linalg.norm(look) or 1.0
        down = np.array([0.0, 0.0, -1.0])
        ang = float(np.arccos(np.clip(look @ down, -1, 1)))
        lim = np.radians(c.max_tilt_deg)
        if ang > lim:
            # rotate from straight down toward the look-at direction by the limit
            k = np.cross(down, look)
            k /= np.linalg.norm(k) or 1.0
            look = down * np.cos(lim) + np.cross(k, down) * np.sin(lim) + k * (k @ down) * (1 - np.cos(lim))
        return p_des, look, clamped

    # ------------------------------------------------------------------ the step
    def step(self, q, target_table, dt: float, v_table=None) -> tuple[np.ndarray, dict]:
        """One control tick. ``target_table``: the object's top centre (table frame, x y z),
        ``v_table``: its velocity (table frame, m/s) or None. Returns (qd, info)."""
        c = self.cfg
        q = np.asarray(q, float)
        if self.j6_home is None:
            self.j6_home = float(q[5])
        target_b = self.table_to_base(target_table)
        p_des, a_des, clamped = self.desired(target_b, q)
        T = self.camera(q)
        p_c, a = T[:3, 3], T[:3, 2]
        e_p = p_des - p_c
        cosang = float(np.clip(a @ a_des, -1, 1))
        ang = float(np.arccos(cosang))
        # camera twist wanted
        v_c = c.kp_pos * e_p
        if v_table is not None and c.ff_gain and not clamped:
            v_ff = self.T_base_table[:3, :3] @ np.r_[np.asarray(v_table, float)[:2], 0.0]
            v_c = v_c + c.ff_gain * v_ff
        if np.linalg.norm(e_p) < c.deadband_pos and (v_table is None or np.linalg.norm(v_table) < 0.005):
            v_c = np.zeros(3)
        v_c = _cap(v_c, c.v_max)
        axis = np.cross(a, a_des)
        s = float(np.linalg.norm(axis))
        w = (axis / s) * ang * c.kp_rot if s > 1e-9 and ang > np.radians(c.deadband_rot_deg) else np.zeros(3)
        w = _cap(w, c.w_max)
        # 5-D task Jacobian at the camera centre
        Tt = kin.fk(q)
        r = p_c - Tt[:3, 3]
        J = kin.jacobian_geometric(q)
        Jv, Jw = J[:3], J[3:]
        Jc = Jv - _skew(r) @ Jw              # camera point: v_c = v_tip + w x r
        E = _perp_basis(a)
        A = np.vstack([Jc, E @ Jw])
        lam2 = c.damping**2
        Ainv = A.T @ np.linalg.inv(A @ A.T + lam2 * np.eye(5))
        # Two parts: translate with the optical axis held (qd_p), and turn the axis (qd_r). The
        # translation gets the joint-speed budget first. Scaling the sum uniformly starved it:
        # a lagging camera asks for more look-at tilt, j4 saturates, the shared scale shrinks the
        # translation, so it lags more (the 'line' sim drifted 8 cm with j4 pinned at its cap).
        qd_p = Ainv @ np.r_[v_c, 0.0, 0.0]
        qd_r = Ainv @ np.r_[np.zeros(3), E @ w]
        null = np.zeros(6)
        null[5] = c.null_gain * (self.j6_home - q[5])
        qd_p = qd_p + (np.eye(6) - Ainv @ A) @ null
        caps = np.maximum(self.vel_caps, 1e-9)
        over = np.max(np.abs(qd_p) / caps)
        if over > 1:
            qd_p, k = qd_p / over, 0.0
        else:
            k = 1.0     # the largest share of qd_r that still fits under the caps
            for j in range(6):
                if abs(qd_r[j]) > 1e-12:
                    room = (np.sign(qd_r[j]) * caps[j] - qd_p[j]) / qd_r[j]
                    k = min(k, max(0.0, room))
        qd = qd_p + k * qd_r
        # acceleration caps
        dv = np.clip(qd - self.qd, -self.acc_caps * dt, self.acc_caps * dt)
        qd = self.qd + dv
        # the soft box, with the braking lookahead
        # (+ 0.1 s: the q we have is up to a state period old, and the daemon predicts from its own)
        qd, viol = self.env.clamp_step(q, qd, self.env.lookahead() + 0.1)
        # the table
        qd, guarded = table_guard(q, qd)
        self.qd = np.asarray(qd, float)
        info = {"e_pos": float(np.linalg.norm(e_p)), "e_ang_deg": float(np.degrees(ang)), "clamped": clamped,
                "box": [v.message for v in viol][:2], "guard": bool(guarded),
                "v_cam": float(np.linalg.norm(v_c)), "qd_max": float(np.abs(self.qd).max()), "turn_share": round(k, 3)}
        return self.qd.copy(), info

    # ------------------------------------------------------------------ measures
    def centring_error(self, q, target_table) -> float:
        """Horizontal distance (m) on the object's top plane between where the optical axis hits
        it and the object's centre: 0 = the object is in the image centre."""
        T = self.camera(q)
        p_c, a = T[:3, 3], T[:3, 2]
        tb = self.table_to_base(target_table)
        if a[2] >= -1e-6:
            return float("inf")
        s = (tb[2] - p_c[2]) / a[2]
        hit = p_c + s * a
        return float(np.linalg.norm(hit[:2] - tb[:2]))
