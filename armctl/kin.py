"""armctl.kin: forward and inverse kinematics of the YAM arm in plain numpy.

No pinocchio and no URDF file needed: the joint origins and axes are copied from
yam_description/i2rt/yam.urdf (same table as openyam-playground src/lib/arm/kinematics.ts,
which is checked against pinocchio). Units: metres, radians. Base frame: z up, x forward
(where the arm reaches at j1 = 0). ``gripper_tip`` is DimOS's IK frame, 10 cm past the
gripper mount toward the fingertips.

Known values (pinocchio): home tip (0.211, 0, 0.174), ready tip (0.205, 0, 0.432).

Nothing here moves anything. IK results are joint targets that still go through the
armctl envelope gate like everything else.

Public API
----------
fk(q) -> 4x4 tip pose           frames(q) -> {name: 4x4} for every link + tip
tip(q) -> xyz                    clearance(q) -> (tip z, lowest link z, link name)
ik(xyz, q0, down=False, yaw=None) -> IKResult(q, ok, err_pos, err_rot, iters, reason)
"""
from __future__ import annotations

from dataclasses import dataclass, field
from functools import lru_cache

import numpy as np

from . import envelope as env

# (name, child, xyz, rpy, axis): joint1..joint6, copied from yam.urdf.
CHAIN = [
    ("joint1", "link1", (-2.49777947129965e-7, -9.17402363407408e-13, 0.0679999999995413),
     (0, 1.5708, 3.1415963267949), (-1, 0, 0)),
    ("joint2", "link2", (-0.0455, -0.0339, -0.02), (-1.7953e-16, 5.55112e-17, -3.14159),
     (3.58252e-15, 1, -3.2007e-16)),
    ("joint3", "link3", (4.08431e-7, -0.0688, 0.264), (-2.99502e-14, -1.6015e-14, -3.14159),
     (4.54273e-15, 1, 1.04275e-14)),
    ("joint4", "link4", (-0.0600003, -0.0688, -0.244999), (2.95536e-14, 8.52374e-14, -2.10585e-14), (0, 1, 0)),
    ("joint5", "link5", (-0.0405003, 0.0338995, -0.0739989), (2.73264e-14, -4.32956e-16, -8.32667e-17), (1, 0, 0)),
    ("joint6", "gripper", (0.0404996, 2.39858e-7, -0.0356), (8.83309e-16, -5.88588e-16, 7.69861e-14),
     (1.46014e-15, 2.40808e-14, -1)),
]
TIP_OFFSET = np.array([0.0, 0.0, -0.1])  # gripper -> gripper_tip
# The tip frame's -z points from the wrist toward the fingertips (the approach direction).
APPROACH_AXIS = np.array([0.0, 0.0, -1.0])

MIN_TIP_Z = 0.08   # m, same rule as armctl.fk
MIN_LINK_Z = 0.03  # m


def _origin(xyz, rpy) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    T = np.eye(4)
    T[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    T[:3, 3] = xyz
    return T


def _axis_angle(axis, a) -> np.ndarray:
    x, y, z = np.asarray(axis, float) / np.linalg.norm(axis)
    c, s = np.cos(a), np.sin(a)
    C = 1 - c
    R = np.eye(4)
    R[:3, :3] = [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                 [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                 [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]
    return R


_ORIGINS = [_origin(xyz, rpy) for _, _, xyz, rpy, _ in CHAIN]
_AXES = [np.asarray(ax, float) / np.linalg.norm(ax) for *_, ax in CHAIN]


# --- kinematic corrections (added 2026-10-29, experiments/2026-10-29-kin-calib) -------------
# A ``KinCorrection`` is a small, measured deviation of the real arm from yam.urdf, fitted from
# wrist-camera views of a fixed board (``experiments/2026-10-29-kin-calib/kincal.py``). Joint
# angles are ALWAYS DimOS's (encoder) joint space: the correction lives inside the FK function,
#     fk(q, corr) = FK_urdf(q + joint_offsets + compliance * tau_grav(q); origins + origin_deltas),
# so an IK solved against ``fk(., corr)`` returns DimOS joint targets directly. By default every
# function here uses NO correction (corr=None): commanded motion, the envelope, table clearance
# and choreography are unchanged. Vision / grasp planning opt in with ``corr=vision_correction()``
# (or ``fk_vision``), which reads ``armctl/kin_corrections.json`` if it exists and is enabled.
CORRECTIONS_PATH = __import__("pathlib").Path(__file__).with_name("kin_corrections.json")


@dataclass(frozen=True)
class KinCorrection:
    joint_offsets: tuple = (0.0,) * 6                 # rad, added to the measured q
    origin_deltas: tuple = ((0.0, 0.0, 0.0),) * 6     # m, added to joint1..6 origin xyz (parent frame)
    tip_delta: tuple = (0.0, 0.0, 0.0)                # m, added to TIP_OFFSET (gripper frame)
    compliance: tuple = (0.0,) * 6                    # rad per N*m of model gravity torque (joint sag
    #                                                   not seen by the encoder); 0 = rigid
    source: str = ""

    @property
    def is_zero(self) -> bool:
        return not (np.any(self.joint_offsets) or np.any(self.origin_deltas) or np.any(self.tip_delta)
                    or np.any(self.compliance))

    def to_dict(self) -> dict:
        return {"joint_offsets": [float(x) for x in self.joint_offsets],
                "origin_deltas": [[float(v) for v in r] for r in self.origin_deltas],
                "tip_delta": [float(x) for x in self.tip_delta], "compliance": [float(x) for x in self.compliance]}

    @classmethod
    def from_dict(cls, d: dict) -> "KinCorrection":
        return cls(tuple(float(x) for x in d.get("joint_offsets", (0.0,) * 6)),
                   tuple(tuple(float(v) for v in r) for r in d.get("origin_deltas", ((0.0,) * 3,) * 6)),
                   tuple(float(x) for x in d.get("tip_delta", (0.0,) * 3)),
                   tuple(float(x) for x in d.get("compliance", (0.0,) * 6)), str(d.get("source", "")))


def load_correction(path=None) -> KinCorrection | None:
    """The correction in ``path`` (default ``armctl/kin_corrections.json``), or None if the file is
    missing or has ``"enabled": false``."""
    import json
    import os
    if path is None and os.environ.get("ARMCTL_KIN_CORRECTIONS"):   # "off" / "none" disables
        path = os.environ["ARMCTL_KIN_CORRECTIONS"]
        if path.lower() in ("off", "none", "0"):
            return None
    p = __import__("pathlib").Path(path) if path else CORRECTIONS_PATH
    if not p.exists():
        return None
    d = json.loads(p.read_text())
    if not d.get("enabled", True):
        return None
    c = KinCorrection.from_dict(d.get("correction", d))
    return None if c.is_zero else c


_VISION_CORR: list = []


def vision_correction() -> KinCorrection | None:
    """The correction for vision / grasp planning (cached; ``reload_vision_correction`` re-reads)."""
    if not _VISION_CORR:
        _VISION_CORR.append(load_correction())
    return _VISION_CORR[0]


def reload_vision_correction() -> KinCorrection | None:
    _VISION_CORR.clear()
    return vision_correction()


# Point masses (kg, position in the child link frame) for the gravity-torque proxy used by the
# compliance term: forearm, wrist, gripper + camera. Only the SHAPE of tau(q) matters (the fitted
# compliance absorbs the scale); masses roughly from i2rt yam.urdf inertials.
_SAG_MASSES = (("link3", 0.55, (-0.03, -0.0688, -0.12)), ("link5", 0.45, (0.0, 0.0, -0.03)),
               ("gripper", 0.75, (0.0, 0.0, -0.05)))


def gravity_torque(q) -> np.ndarray:
    """Approximate static gravity torque (N*m) about each joint axis for the point-mass model."""
    q = env.as_vector(q)
    f = frames(q)
    child_idx = {c: i for i, (_, c, *_r) in enumerate(CHAIN)}
    pts = [(child_idx[lk], m, (f[lk] @ np.r_[p, 1.0])[:3]) for lk, m, p in _SAG_MASSES]
    g = np.array([0.0, 0.0, -9.81])
    tau = np.zeros(6)
    T = np.eye(4)
    for i in range(6):
        T = T @ _ORIGINS[i]
        o, ax = T[:3, 3], T[:3, :3] @ _AXES[i]
        T = T @ _axis_angle(_AXES[i], q[i])
        tau[i] = sum(m * ax @ np.cross(p - o, g) for k, m, p in pts if k >= i)   # masses distal to joint i
    return tau


def frames(q, corr: KinCorrection | None = None) -> dict[str, np.ndarray]:
    """4x4 pose of every link (and ``gripper_tip``) in the base frame. ``corr``: optional
    measured correction (see ``KinCorrection``); None = the nominal yam.urdf model."""
    q = env.as_vector(q)
    tip_off = TIP_OFFSET
    origins = _ORIGINS
    if corr is not None and not corr.is_zero:
        q = q + np.asarray(corr.joint_offsets, float)
        if np.any(corr.compliance):
            q = q + np.asarray(corr.compliance, float) * gravity_torque(q)
        origins = [o.copy() for o in _ORIGINS]
        for i, d in enumerate(corr.origin_deltas):
            origins[i][:3, 3] += np.asarray(d, float)
        tip_off = TIP_OFFSET + np.asarray(corr.tip_delta, float)
    T = np.eye(4)
    out = {"base": T.copy()}
    for i, (_, child, *_r) in enumerate(CHAIN):
        T = T @ origins[i] @ _axis_angle(_AXES[i], q[i])
        out[child] = T.copy()
    tip = np.eye(4)
    tip[:3, 3] = tip_off
    out["gripper_tip"] = T @ tip
    return out


def fk(q, corr: KinCorrection | None = None) -> np.ndarray:
    """4x4 pose of ``gripper_tip`` (nominal model unless ``corr`` is given)."""
    return frames(q, corr)["gripper_tip"]


def fk_vision(q) -> np.ndarray:
    """FK for vision / grasp planning: nominal model + ``armctl/kin_corrections.json`` if present."""
    return fk(q, vision_correction())


def tip(q, corr: KinCorrection | None = None) -> np.ndarray:
    return fk(q, corr)[:3, 3].copy()


def approach(q, corr: KinCorrection | None = None) -> np.ndarray:
    """Unit vector the fingers point along (base frame)."""
    return fk(q, corr)[:3, :3] @ APPROACH_AXIS


def clearance(q, corr: KinCorrection | None = None) -> tuple[float, float, str]:
    """(tip z, lowest link-origin z excluding base/link1, that link's name)."""
    f = frames(q, corr)
    links = [(float(T[2, 3]), n) for n, T in f.items() if n not in ("base", "link1", "gripper_tip")]
    lz, ln = min(links)
    return float(f["gripper_tip"][2, 3]), lz, ln


def clear_of_table(q, min_tip_z: float = MIN_TIP_Z, min_link_z: float = MIN_LINK_Z) -> bool:
    tz, lz, _ = clearance(q)
    return tz >= min_tip_z and lz >= min_link_z


def jacobian(q, eps: float = 1e-6, corr: KinCorrection | None = None) -> np.ndarray:
    """6x6 numeric Jacobian: rows 0-2 tip position, rows 3-5 approach direction (unit vector)."""
    q = env.as_vector(q)
    J = np.zeros((6, 6))
    p0, a0 = tip(q, corr), approach(q, corr)
    for i in range(6):
        dq = q.copy()
        dq[i] += eps
        J[:3, i] = (tip(dq, corr) - p0) / eps
        J[3:, i] = (approach(dq, corr) - a0) / eps
    return J


@dataclass
class IKResult:
    q: np.ndarray
    ok: bool
    err_pos: float  # m
    err_rot: float  # rad between wanted and achieved approach direction (0 if not asked)
    iters: int
    reason: str = ""
    tried: list = field(default_factory=list)

    def to_dict(self) -> dict:
        return {"q": [round(float(x), 4) for x in self.q], "ok": self.ok, "err_pos": round(self.err_pos, 4),
                "err_rot": round(self.err_rot, 4), "iters": self.iters, "reason": self.reason}


def ik(xyz, q0=None, *, down: bool = False, approach_dir=None, envelope: env.Envelope = env.DEFAULT,
       pos_tol: float = 0.003, rot_tol: float = 0.05, max_iters: int = 300, restarts: int = 6,
       check_table: bool = True, corr: KinCorrection | None = None) -> IKResult:
    """Joint target that puts the tip at ``xyz`` (base frame).

    ``down=True`` also points the fingers straight down (top-down grasp); or give any unit
    ``approach_dir``. Damped least squares, clamped to the soft box every step; restarts from
    a few seeds if the first one gets stuck. ``ok`` means within tolerance AND inside the
    soft box AND clear of the table (unless ``check_table=False``). It never returns a
    target outside the soft box.

    ``corr``: solve against the corrected model (vision / grasp planning, ``vision_correction()``);
    the result is still a DimOS joint target. The table check then uses the LOWER of the nominal
    and corrected clearances.
    """
    target = np.asarray(xyz, float)
    want = np.array([0.0, 0.0, -1.0]) if down else (None if approach_dir is None else
                                                    np.asarray(approach_dir, float) / np.linalg.norm(approach_dir))
    lo, hi = envelope.soft_lower, envelope.soft_upper
    seeds = [env.as_vector(q0) if q0 is not None else np.array(env.POSES["ready"])]
    yaw = float(np.arctan2(target[1], target[0]))
    rng = np.random.default_rng(0)
    seeds.append(np.array([yaw, 1.0, 1.2, 0.0, 0.0, 0.0]))
    seeds.append(np.array([yaw, 1.6, 1.6, 0.6, 0.0, 0.0]))
    for _ in range(max(0, restarts - 3)):
        seeds.append(np.clip(rng.uniform(lo, hi), lo, hi))
    best: IKResult | None = None
    tried = []
    for seed in seeds:
        q = np.clip(seed.astype(float), lo, hi)
        lam = 0.05
        it = 0
        for it in range(1, max_iters + 1):
            e = np.zeros(6)
            e[:3] = target - tip(q, corr)
            w = np.ones(6)
            if want is not None:
                e[3:] = (want - approach(q, corr)) * 0.3  # 1 rad of approach error ~ 0.3 m
                w[3:] = 0.3
            else:
                w[3:] = 0.0
            if np.linalg.norm(e[:3]) < pos_tol and (want is None or np.linalg.norm(e[3:]) / 0.3 < rot_tol):
                break
            J = jacobian(q, corr=corr) * w[:, None]
            dq = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(6), e)
            step = np.max(np.abs(dq))
            if step > 0.2:
                dq *= 0.2 / step
            q = np.clip(q + dq, lo, hi)
        ep = float(np.linalg.norm(target - tip(q, corr)))
        er = float(np.arccos(np.clip(np.dot(want, approach(q, corr)), -1, 1))) if want is not None else 0.0
        tz, lz, ln = clearance(q)
        if corr is not None:
            tz2, lz2, ln2 = clearance(q, corr)
            tz, lz, ln = min(tz, tz2), min(lz, lz2), (ln if lz <= lz2 else ln2)
        table_ok = (not check_table) or (tz >= MIN_TIP_Z and lz >= MIN_LINK_Z)
        ok = ep < pos_tol * 2 and er < rot_tol * 2 and table_ok
        reason = "" if ok else (
            f"tip {ep * 1000:.0f} mm from target" if ep >= pos_tol * 2 else
            f"approach {np.degrees(er):.0f} deg off" if er >= rot_tol * 2 else
            f"too close to the table (tip z {tz:.3f} m, {ln} z {lz:.3f} m)")
        res = IKResult(q, ok, ep, er, it, reason)
        tried.append(res.to_dict())
        if ok:
            res.tried = tried
            return res
        if best is None or ep + 0.3 * er < best.err_pos + 0.3 * best.err_rot:
            best = res
    assert best is not None
    best.tried = tried
    if not best.reason:
        best.reason = "no seed converged"
    return best


@lru_cache(maxsize=1)
def reach_summary() -> dict:
    """Rough numbers for docs and prompts: tip at home/ready, max horizontal reach."""
    pts = [tip([0, j2, j3, 0, 0, 0]) for j2 in np.linspace(0.2, 3.5, 25) for j3 in np.linspace(0.2, 3.0, 25)]
    pts = [p for p in pts if p[2] > MIN_TIP_Z]
    return {"home_tip": [round(float(x), 3) for x in tip(env.POSES["home"])],
            "ready_tip": [round(float(x), 3) for x in tip(env.POSES["ready"])],
            "max_reach_xy": round(max(float(np.hypot(p[0], p[1])) for p in pts), 3)}


# --- velocity kinematics (added 2026-10-28: Cartesian twists in the mock backend) ----------

def rot_log(R) -> np.ndarray:
    """Rotation vector (axis * angle, rad) of a 3x3 rotation matrix: the inverse of Rodrigues."""
    R = np.asarray(R, float)
    c = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    a = float(np.arccos(c))
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if a < 1e-9:
        return 0.5 * w
    if np.pi - a < 1e-6:  # near 180 deg: axis from the diagonal
        ax = np.sqrt(np.maximum((np.diag(R) + 1.0) / 2.0, 0.0))
        i = int(np.argmax(ax))
        col = (R[:, i] + np.eye(3)[:, i]) / 2.0
        ax = col / max(np.sqrt(col[i]), 1e-12)
        return ax / np.linalg.norm(ax) * a
    return a / (2.0 * np.sin(a)) * w


def jacobian_geometric(q, eps: float = 1e-6) -> np.ndarray:
    """6x6 geometric Jacobian of ``gripper_tip`` in the base frame.

    Rows 0-2: linear velocity of the tip point (m/s per rad/s); rows 3-5: angular velocity of
    the tip frame about the base axes (rad/s per rad/s). Columns j1..j6. So for joint
    velocities ``qd``, ``J @ qd = [vx, vy, vz, wx, wy, wz]``, the same convention as a twist
    {x, y, z, roll, pitch, yaw} in the base frame. Numeric: forward differences of the tip
    position and ``rot_log(R(q + dq) R(q)^T) / eps``.
    """
    q = env.as_vector(q)
    T0 = fk(q)
    p0, R0 = T0[:3, 3], T0[:3, :3]
    J = np.zeros((6, 6))
    for i in range(6):
        dq = q.copy()
        dq[i] += eps
        T = fk(dq)
        J[:3, i] = (T[:3, 3] - p0) / eps
        J[3:, i] = rot_log(T[:3, :3] @ R0.T) / eps
    return J


def dls_velocity(q, twist, damping: float = 0.03, J=None) -> np.ndarray:
    """Joint velocity for a base-frame twist [vx, vy, vz, wx, wy, wz] by damped least squares:
    ``qd = J^T (J J^T + damping^2 I)^-1 twist``. Damping keeps qd bounded near singularities
    (at the cost of tracking error there). Columns of ``J`` that are zero (joints the caller
    has blocked) get zero velocity."""
    J = jacobian_geometric(q) if J is None else np.asarray(J, float)
    v = np.asarray(twist, float).reshape(6)
    return J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(6), v)
