"""interfaces.wristcam.handeye: where the wrist camera sits on the gripper, and where the table is.

The calibration sheet (``board.py``) is taped flat on the table; the arm visits K poses
(``plan_views``) and at each one we record the measured joint angles q and a frame. Per frame the
board pose in the camera (``T_cam_board``, PnP with the calibrated lens) and per q the tip pose
(``armctl.kin.fk``) give the classic AX = XB problem:

    T_base_tip(q_i) @ T_tip_cam @ T_cam_board_i = T_base_board        (same for every i)

``cv2.calibrateHandEye`` is run with TSAI, PARK, HORAUD, ANDREFF and DANIILIDIS, and
``cv2.calibrateRobotWorldHandEye`` with SHAH and LI; among the candidates that satisfy the bracket
prior (optical axis within 25 deg of the nominal mounting, camera within 15 cm of the tip) the one whose
``T_base_board`` estimates agree best (the board does not move) seeds a joint refinement of
camera translation and ``T_base_board`` over all board corners (reprojection error in pixels, scipy
least squares; the rotation stays the closed-form one, which is more robust to FK noise), kept only
if it makes the board poses agree better. By-product: ``T_base_table = T_base_board @ T_board_table`` (table origin at the
board centre), written into ``interfaces/grasp/workcell.json`` with ``calibrated: true`` (the old
file is backed up, other keys kept) ONLY if every ``sanity_gates`` check passes; otherwise the
result goes to ``calib/handeye.rejected-<time>.json`` and the workcell is left alone.

    # 1. plan + check the views (nothing moves): board centre in the base frame, roughly
    .venv/bin/python -m interfaces.wristcam.handeye plan --board-center 0.30,0,0 --n 15
    # 2. visit them and record frames + q (moves the arm: each motion is approved at the terminal)
    .venv/bin/python -m interfaces.wristcam.handeye collect --board-center 0.30,0,0 --n 15 \\
        --out interfaces/wristcam/data/he1 --execute
    # 3. solve (uses calib/wristcam.json for the lens), write calib/handeye.json and the workcell
    .venv/bin/python -m interfaces.wristcam.handeye solve --dir interfaces/wristcam/data/he1 --write-workcell

The same collected folder also calibrates the lens: ``intrinsics solve --dir <folder>`` (use
``plan --purpose intrinsics`` for a view set with more tilt and the board off-centre).
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import shutil
import time
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

from armctl import envelope as envm
from armctl import kin

from .board import BoardDetector, BoardSpec, board_pose
from .camera import (finger_dir_for_cam_axis, DEFAULT_HANDEYE, DEFAULT_INTRINSICS, JAW_POLYGONS, T_TIP_CAM_NOMINAL, CameraModel, inv_T,
                     make_T, mean_pose, nominal_camera, rot_err_deg)

REPO = Path(__file__).resolve().parents[2]
WORKCELL = REPO / "interfaces" / "grasp" / "workcell.json"
METHODS = {"TSAI": cv2.CALIB_HAND_EYE_TSAI, "PARK": cv2.CALIB_HAND_EYE_PARK, "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
           "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF, "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS}
# AX = ZB form (camera on the tip AND the board in the base frame, solved together)
RWHE_METHODS = {"RWHE-SHAH": cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH, "RWHE-LI": cv2.CALIB_ROBOT_WORLD_HAND_EYE_LI}

# Bracket prior: the camera is on an angled bracket ~60 deg off the fingers (camera.T_TIP_CAM_NOMINAL)
# and within a hand's width of the tip. A solution outside this is a wrong solution, not a new mount.
PRIOR_AXIS_DEG = 25.0         # optical axis vs the nominal optical axis
PRIOR_MAX_DIST_M = 0.15       # camera centre vs gripper_tip
# Sanity gates for writing workcell.json calibrated:true (see sanity_gates)
GATES = {"rms_px": 3.0, "spread_mm": 6.0, "spread_deg": 1.0, "table_z_m": 0.02,
         "axis_deg": PRIOR_AXIS_DEG, "cam_dist_m": PRIOR_MAX_DIST_M, "board_scale_tol": 0.05}


# --------------------------------------------------------------------------- view planning
def _in_jaws(uv, size) -> np.ndarray:
    w, h = size
    out = np.zeros(len(uv), bool)
    for poly in JAW_POLYGONS:
        p = np.array([[x * w, y * h] for x, y in poly], np.float32)
        out |= np.array([cv2.pointPolygonTest(p, (float(u), float(v)), False) >= 0 for u, v in uv], bool)
    return out


def visible_fraction(q, pts_base, cam: CameraModel, T_tip_cam=T_TIP_CAM_NOMINAL, margin: float = 0.03) -> float:
    """Fraction of ``pts_base`` that land inside the image (with a margin) and outside the jaws."""
    T_cb = inv_T(kin.fk(q) @ T_tip_cam)
    P = np.asarray(pts_base, float) @ T_cb[:3, :3].T + T_cb[:3, 3]
    uv = cam.project(P)
    w, h = cam.size
    ok = np.isfinite(uv).all(1)
    ok &= (uv[:, 0] > margin * w) & (uv[:, 0] < (1 - margin) * w) & (uv[:, 1] > margin * h) & (uv[:, 1] < (1 - margin) * h)
    ok[ok] &= ~_in_jaws(uv[ok], cam.size)
    return float(ok.mean())


def board_points_base(center_xyz, spec: BoardSpec, yaw: float = 0.0, outer_only: bool = False) -> np.ndarray:
    """Board corners (inner chessboard corners, or just the 4 outer corners) in the base frame,
    assuming the sheet is taped with table axes = base axes rotated by ``yaw``."""
    w, h = spec.size_m
    if outer_only:
        P = np.array([[-w / 2, -h / 2, 0], [w / 2, -h / 2, 0], [w / 2, h / 2, 0], [-w / 2, h / 2, 0]])
    else:
        sq = spec.square_mm / 1000
        xs = (np.arange(1, spec.squares[0]) * sq) - w / 2
        ys = (np.arange(1, spec.squares[1]) * sq) - h / 2
        P = np.array([[x, y, 0.0] for y in ys for x in xs])
    c, s = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return P @ Rz.T + np.asarray(center_xyz, float)


def check_pose(q, min_tip_z: float = kin.MIN_TIP_Z) -> str:
    """'' if q is inside the soft box and clear of the table, else the reason."""
    viol = envm.DEFAULT.check_point(q)
    if viol:
        return "; ".join(v.message for v in viol)
    tz, lz, ln = kin.clearance(q)
    if tz < min_tip_z - 1e-4 or lz < kin.MIN_LINK_Z:
        return f"too low (tip z {tz:.3f} m, {ln} z {lz:.3f} m)"
    return ""


def rotation_diversity(qs, T_tip_cam=None) -> dict:
    """How well a view set constrains hand-eye rotation: pairwise camera rotation angles and how
    the relative-rotation axes spread over 3D (singular values of the stacked unit axes, normalised
    so that 1.0 = evenly spread over all three axes, 0 = all motions about one axis: degenerate)."""
    T_tc = T_TIP_CAM_NOMINAL if T_tip_cam is None else np.asarray(T_tip_cam, float)
    Rs = [(kin.fk(np.asarray(q, float)) @ T_tc)[:3, :3] for q in qs]
    ang, axes = [], []
    for i in range(len(Rs)):
        for j in range(i + 1, len(Rs)):
            rv = cv2.Rodrigues(Rs[i].T @ Rs[j])[0].ravel()
            th = float(np.linalg.norm(rv))
            ang.append(np.degrees(th))
            if th > np.radians(3):
                axes.append(rv / th)
    if len(axes) < 3:
        return {"n": len(Rs), "median_deg": float(np.median(ang)) if ang else 0.0, "max_deg": max(ang, default=0.0),
                "axis_spread": 0.0}
    sv = np.linalg.svd(np.array(axes), compute_uv=False)
    return {"n": len(Rs), "median_deg": round(float(np.median(ang)), 1), "max_deg": round(float(max(ang)), 1),
            "axis_spread": round(float(sv[2] / sv[0]), 3), "axis_sv": np.round(sv / sv[0], 3).tolist()}


def plan_views(center_xyz, n: int = 15, *, purpose: str = "handeye", cam: CameraModel | None = None,
               T_tip_cam=None, spec: BoardSpec | None = None, heights=(0.17, 0.28), tilt_max: float | None = None,
               tilt_min: float | None = None, roll_max: float = 0.6, roll_min: float | None = None,
               aim_jitter: float | None = None, board_yaw: float = 0.0, seed: int = 0,
               max_tries: int | None = None) -> list[dict]:
    """Feasible, rotation-diverse camera poses that look at the board centred at ``center_xyz`` (base frame).

    Camera heights (above the board) in ``heights``. Hand-eye needs rotations about at least two
    axes (prior art: Tsai/Park degrade to nonsense when every motion turns about one axis), so for
    ``purpose='handeye'`` each view gets a target: the camera axis tilted ``tilt_min``..``tilt_max``
    (default 10..35 deg) toward a heading that walks round the board (golden-angle steps), and a j6
    roll of alternating sign, ``roll_min``..``roll_max`` rad (default 0.15..0.6). Intrinsics views
    (default tilt 0..32 deg, aimed off-centre so the board reaches the image edges) keep random
    headings. The camera is aimed with ``finger_dir_for_cam_axis`` (the bracket camera is ~60 deg off
    the fingers). Each pose: IK ok, inside the soft box, tip >= armctl.kin.MIN_TIP_Z and links clear
    of the table, and the board in view through the nominal camera (handeye: all 4 outer corners;
    intrinsics: >= 45% of the corners). Exactly-down tip poses only reach tip z ~0.21 m (j4 soft limit),
    so the higher views are found tilted. Returned ordered as a short joint-space tour from ``ready``.
    ``rotation_diversity`` scores the result.
    """
    cam = cam or nominal_camera()
    T_tip_cam = T_TIP_CAM_NOMINAL if T_tip_cam is None else np.asarray(T_tip_cam, float)
    spec = spec or BoardSpec.load()
    intr = purpose == "intrinsics"
    tilt_max = tilt_max if tilt_max is not None else (32.0 if intr else 35.0)
    tilt_min = tilt_min if tilt_min is not None else (0.0 if intr else 10.0)
    roll_min = roll_min if roll_min is not None else (0.0 if intr else 0.15)
    aim_jitter = aim_jitter if aim_jitter is not None else (0.14 if intr else 0.02)
    max_tries = max_tries or max(600, 60 * n)
    rng = np.random.default_rng(seed)
    c = np.asarray(center_xyz, float)
    outer = board_points_base(c, spec, board_yaw, outer_only=True)
    inner = board_points_base(c, spec, board_yaw)
    lo, hi = envm.DEFAULT.soft_lower, envm.DEFAULT.soft_upper
    views: list[dict] = []
    seed_q = np.array(envm.POSES["ready"], float)
    az0 = rng.uniform(0, 2 * np.pi)
    slot_tries = 0
    stats = {"tries": 0, "ik": 0, "box": 0, "view": 0, "tilt": 0, "dup": 0}
    for _ in range(max_tries):
        if len(views) >= n:
            break
        h = rng.uniform(*heights)
        k = len(views)
        stats["tries"] += 1
        if intr:
            tilt = np.radians(rng.uniform(tilt_min, tilt_max)) if rng.uniform() > 0.15 else 0.0
            az = rng.uniform(0, 2 * np.pi)
            roll = rng.uniform(-roll_max, roll_max)
        else:
            # stratified target for slot k; after many failures the heading is freed (the arm can't
            # reach every heading round the board) but tilt and roll stay in range
            slot_tries += 1
            free = slot_tries > 40
            tilt = np.radians(rng.uniform(tilt_min, tilt_max))
            az = rng.uniform(0, 2 * np.pi) if free else az0 + k * 2.39996 + rng.uniform(-0.35, 0.35)
            roll = (1 if k % 2 == 0 else -1) * rng.uniform(roll_min, roll_max)
        aim = c + np.r_[rng.uniform(-aim_jitter, aim_jitter, 2), 0.0]
        a = np.array([np.sin(tilt) * np.cos(az), np.sin(tilt) * np.sin(az), -np.cos(tilt)])
        cam_pos = aim - (h / np.cos(tilt)) * a
        tip_pos = cam_pos.copy()
        q = None
        n_it = 1 if np.allclose(T_tip_cam[:3, 3], 0) else 2
        s_q = seed_q
        for _it in range(n_it):   # put the CAMERA (not the tip) at cam_pos
            # aim the CAMERA along a: the fingers point ~60 deg away from it (measured mounting)
            r = kin.ik(tip_pos, s_q, approach_dir=finger_dir_for_cam_axis(a, tip_pos, T_tip_cam), restarts=2, max_iters=60)
            if not r.ok:
                break
            T = kin.fk(r.q) @ T_tip_cam
            tip_pos = tip_pos + (cam_pos - T[:3, 3])
            q = s_q = r.q
        if q is None or not r.ok:
            stats["ik"] += 1
            continue
        q = r.q.copy()
        q[5] = float(np.clip(q[5] + roll, lo[5], hi[5]))
        if check_pose(q):
            stats["box"] += 1
            continue
        frac_outer = visible_fraction(q, outer, cam, T_tip_cam)
        frac_inner = visible_fraction(q, inner, cam, T_tip_cam)
        if (frac_inner < 0.45) if intr else (frac_outer < 1.0):
            stats["view"] += 1
            continue
        T = kin.fk(q) @ T_tip_cam
        axis = T[:3, 2]
        tilt_now = float(np.degrees(np.arccos(np.clip(-axis[2], -1, 1))))
        if not intr and not (tilt_min <= tilt_now <= tilt_max):
            stats["tilt"] += 1  # the j6 roll moved the (off-axis) camera out of the tilt band
            continue
        if any(np.degrees(np.arccos(np.clip(axis @ v["_axis"], -1, 1))) < 6 and np.linalg.norm(T[:3, 3] - v["_pos"]) < 0.03
               and abs(q[5] - v["q"][5]) < 0.2 for v in views):
            stats["dup"] += 1
            continue
        views.append({"q": [round(float(x), 4) for x in q], "tip": kin.tip(q).round(4).tolist(),
                      "cam_height": round(float(T[2, 3] - c[2]), 3),
                      "tilt_deg": round(tilt_now, 1), "roll_rad": round(float(roll), 3),
                      "visible": round(frac_inner, 2), "_axis": axis, "_pos": T[:3, 3]})
        slot_tries = 0
    # greedy tour from ready
    order, cur, left = [], np.array(envm.POSES["ready"]), list(range(len(views)))
    while left:
        k = min(left, key=lambda i: np.max(np.abs(np.array(views[i]["q"]) - cur)))
        order.append(views[k])
        cur = np.array(views[k]["q"])
        left.remove(k)
    for v in order:
        v.pop("_axis")
        v.pop("_pos")
    plan_views.last_stats = stats
    return order


# --------------------------------------------------------------------------- solving
@dataclass
class Sample:
    q: np.ndarray
    obj: np.ndarray          # board corners, board frame (N,3)
    img: np.ndarray          # pixels (N,2)
    T_cam_board: np.ndarray
    pnp_rms: float
    name: str = ""


@dataclass
class HandEyeResult:
    T_tip_cam: np.ndarray
    T_base_board: np.ndarray
    T_base_table: np.ndarray
    method: str
    methods: dict
    rms_px: float
    per_view_px: list
    n_views: int
    spread_mm: float
    spread_deg: float
    extra: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {"T_tip_cam": self.T_tip_cam.round(6).tolist(), "T_base_board": self.T_base_board.round(6).tolist(),
                "T_base_table": self.T_base_table.round(6).tolist(), "method": self.method, "methods": self.methods,
                "rms_px": round(self.rms_px, 3), "per_view_px": [round(x, 3) for x in self.per_view_px],
                "n_views": self.n_views, "spread_mm": round(self.spread_mm, 2), "spread_deg": round(self.spread_deg, 3),
                "solved_at": dt.datetime.now().isoformat(timespec="seconds"), **self.extra}


def make_samples(qs, frames, cam: CameraModel, spec: BoardSpec | None = None, min_corners: int = 10,
                 names=None) -> list[Sample]:
    det = BoardDetector(spec)
    mask = cam.jaw_mask()
    out = []
    for i, (q, f) in enumerate(zip(qs, frames)):
        if f is None:
            continue
        c = cam.for_image(f)
        d = det.detect(f, mask if c is cam else None)
        if d is None or len(d[0]) < min_corners:
            continue
        T, rms = board_pose(c, d[0], d[1])
        if T is None or rms > 3.0:
            continue
        out.append(Sample(np.asarray(q, float), d[0], d[1], T, rms, (names or [""] * len(qs))[i]))
    return out


def _consistency(samples, T_tip_cam):
    Tb = [kin.fk(s.q) @ T_tip_cam @ s.T_cam_board for s in samples]
    M = mean_pose(Tb)
    dt_ = np.array([np.linalg.norm(T[:3, 3] - M[:3, 3]) for T in Tb])
    dr = np.array([rot_err_deg(T, M) for T in Tb])
    return M, float(np.sqrt(np.mean(dt_ ** 2))) * 1000, float(np.sqrt(np.mean(dr ** 2)))


def _pack(T1, T2):
    return np.r_[cv2.Rodrigues(T1[:3, :3])[0].ravel(), T1[:3, 3], cv2.Rodrigues(T2[:3, :3])[0].ravel(), T2[:3, 3]]


def _unpack(x):
    return (make_T(cv2.Rodrigues(x[0:3])[0], x[3:6]), make_T(cv2.Rodrigues(x[6:9])[0], x[9:12]))


def _residuals(x, samples, cam, fks):
    T_tc, T_bb = _unpack(x)
    res = []
    for s, F in zip(samples, fks):
        T_cb = inv_T(F @ T_tc) @ T_bb
        uv = cam.project(s.obj @ T_cb[:3, :3].T + T_cb[:3, 3])
        r = (uv - s.img).ravel()
        res.append(np.where(np.isfinite(r), r, 50.0))
    return np.concatenate(res)


def axis_offset_deg(T_tip_cam, nominal=None) -> float:
    """Angle between the solved optical axis and the nominal one (both in the tip frame). Roll about
    the axis is not part of the bracket prior: it is what hand-eye is for."""
    N = T_TIP_CAM_NOMINAL if nominal is None else np.asarray(nominal, float)
    return float(np.degrees(np.arccos(np.clip(np.asarray(T_tip_cam)[:3, 2] @ N[:3, 2], -1, 1))))


def prior_violations(T_tip_cam, nominal=None, axis_deg: float = PRIOR_AXIS_DEG,
                     max_dist_m: float = PRIOR_MAX_DIST_M) -> list[str]:
    """Bracket prior: [] if the camera pose is physically plausible for the angled wrist bracket."""
    out = []
    a = axis_offset_deg(T_tip_cam, nominal)
    if a > axis_deg:
        out.append(f"optical axis {a:.1f} deg from the nominal mounting (limit {axis_deg:g})")
    d = float(np.linalg.norm(np.asarray(T_tip_cam)[:3, 3]))
    if d > max_dist_m:
        out.append(f"camera {d * 100:.1f} cm from the tip (limit {max_dist_m * 100:g})")
    return out


def robot_world_hand_eye(samples: list[Sample], method: str = "RWHE-SHAH") -> tuple[np.ndarray, np.ndarray]:
    """cv2.calibrateRobotWorldHandEye (AX = ZB): solves the camera on the tip and the board in the
    base frame at once. Returns (T_tip_cam, T_base_board)."""
    Rw, tw, Rb, tb = [], [], [], []
    for s in samples:
        Rw.append(s.T_cam_board[:3, :3])           # world (board) -> camera
        tw.append(s.T_cam_board[:3, 3].reshape(3, 1))
        Ti = inv_T(kin.fk(s.q))                     # base -> gripper
        Rb.append(Ti[:3, :3])
        tb.append(Ti[:3, 3].reshape(3, 1))
    R_bw, t_bw, R_gc, t_gc = cv2.calibrateRobotWorldHandEye(Rw, tw, Rb, tb, method=RWHE_METHODS[method])
    return inv_T(make_T(R_gc, t_gc.ravel())), inv_T(make_T(R_bw, t_bw.ravel()))


def _candidates(samples, fks):
    """Closed-form hand-eye from every OpenCV solver: {name: T_tip_cam or error string}."""
    Rg = [F[:3, :3] for F in fks]
    tg = [F[:3, 3] for F in fks]
    Rt = [s.T_cam_board[:3, :3] for s in samples]
    tt = [s.T_cam_board[:3, 3] for s in samples]
    out = {}
    for name, m in METHODS.items():
        try:
            R, t = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=m)
            out[name] = make_T(R, t.ravel())
        except cv2.error as e:
            out[name] = str(e).strip().splitlines()[-1][:120]
    for name in RWHE_METHODS:
        try:
            out[name] = robot_world_hand_eye(samples, name)[0]
        except cv2.error as e:
            out[name] = str(e).strip().splitlines()[-1][:120]
    return out


def solve_handeye(samples: list[Sample], cam: CameraModel, spec: BoardSpec | None = None,
                  refine: bool = True, nominal=None, use_prior: bool = True) -> HandEyeResult:
    """Closed-form candidates (5 calibrateHandEye methods + 2 calibrateRobotWorldHandEye methods),
    the best by board-pose agreement among those that satisfy the bracket prior (``prior_violations``;
    if none does, the best overall, flagged in ``extra['prior']``), then the translation refinement."""
    if len(samples) < 4:
        raise ValueError(f"need >= 4 usable views for hand-eye, have {len(samples)}")
    spec = spec or BoardSpec.load()
    fks = [kin.fk(s.q) for s in samples]
    methods, best, best_any = {}, None, None
    for name, T in _candidates(samples, fks).items():
        if isinstance(T, str) or not np.all(np.isfinite(T)):
            methods[name] = {"error": T if isinstance(T, str) else "not finite"}
            continue
        _, smm, sdeg = _consistency(samples, T)
        score = smm + 2.0 * sdeg
        viol = prior_violations(T, nominal) if use_prior else []
        methods[name] = {"spread_mm": round(smm, 2), "spread_deg": round(sdeg, 3),
                         "t_mm": (T[:3, 3] * 1000).round(1).tolist(),
                         "axis_vs_nominal_deg": round(axis_offset_deg(T, nominal), 1), "prior": viol or "ok"}
        if best_any is None or score < best_any[0]:
            best_any = (score, name, T)
        if not viol and (best is None or score < best[0]):
            best = (score, name, T)
    if best_any is None:
        raise RuntimeError(f"every hand-eye method failed: {methods}")
    prior_note = "ok" if best is not None else "no closed-form solution satisfies the bracket prior"
    _, name, T_tc = best or best_any
    T_bb, smm, sdeg = _consistency(samples, T_tc)
    if refine:
        # Refine the camera TRANSLATION and the board pose on all corners, keeping the closed-form
        # rotation: with FK noise a full 12-parameter refinement trades camera rotation against
        # translation (small view rotations make them nearly collinear) and got worse in tests.
        from scipy.optimize import least_squares
        R_tc = T_tc[:3, :3].copy()

        def res9(x):
            return _residuals(np.r_[cv2.Rodrigues(R_tc)[0].ravel(), x], samples, cam, fks)

        x0 = _pack(T_tc, T_bb)[3:]
        r = least_squares(res9, x0, loss="soft_l1", f_scale=1.0, x_scale="jac")
        T_tc2, T_bb2 = _unpack(np.r_[cv2.Rodrigues(R_tc)[0].ravel(), r.x])
        _, smm2, sdeg2 = _consistency(samples, T_tc2)
        if smm2 + 2.0 * sdeg2 <= smm + 2.0 * sdeg + 1e-9:
            T_tc, T_bb, smm, sdeg = T_tc2, T_bb2, smm2, sdeg2
        else:
            refine = False
    res = _residuals(_pack(T_tc, T_bb), samples, cam, fks).reshape(-1, 2)
    per_view, k = [], 0
    for s in samples:
        n = len(s.obj)
        per_view.append(float(np.sqrt(np.mean(np.sum(res[k:k + n] ** 2, axis=1)))))
        k += n
    rms = float(np.sqrt(np.mean(np.sum(res ** 2, axis=1))))
    T_bt = T_bb @ spec.T_board_table
    return HandEyeResult(T_tc, T_bb, T_bt, name + ("+refined" if refine else ""), methods, rms, per_view,
                         len(samples), smm, sdeg, extra={"prior": prior_note})


def estimate_board_scale(samples: list[Sample], cam: CameraModel, res: HandEyeResult) -> tuple[float, float]:
    """Diagnostic: the board scale factor (true square / assumed square) that best explains the
    corners given the arm's kinematics, from a joint refinement of camera pose, board pose and scale.
    The arm's FK carries metric scale, so a wrong ``square_mm`` shows up as a factor != 1.
    Returns (factor, its 1-sigma). Measure the printed square with a ruler before trusting it."""
    from scipy.optimize import least_squares
    fks = [kin.fk(s.q) for s in samples]

    def f(x):
        out = []
        for s, F in zip(samples, fks):
            T_tc, T_bb = _unpack(x[:12])
            T_cb = inv_T(F @ T_tc) @ T_bb
            P = s.obj * x[12]
            r = (cam.project(P @ T_cb[:3, :3].T + T_cb[:3, 3]) - s.img).ravel()
            out.append(np.where(np.isfinite(r), r, 50.0))
        return np.concatenate(out)
    best = None
    for s0 in (0.6, 0.8, 1.0, 1.25):
        T0 = res.T_tip_cam.copy()
        T0[:3, 3] *= s0
        B0 = mean_pose([F @ T0 @ make_T(s.T_cam_board[:3, :3], s.T_cam_board[:3, 3] * s0) for s, F in zip(samples, fks)])
        r = least_squares(f, np.r_[_pack(T0, B0), s0], loss="soft_l1", f_scale=2.0, x_scale="jac")
        if best is None or r.cost < best.cost:
            best = r
    e = best.fun
    try:
        cov = np.linalg.inv(best.jac.T @ best.jac) * (e @ e / max(1, e.size - best.x.size))
        sd = float(np.sqrt(cov[12, 12]))
    except np.linalg.LinAlgError:
        sd = float("nan")
    return float(best.x[12]), sd


def sanity_gates(res: HandEyeResult, nominal=None, table_z: float = 0.0, limits: dict | None = None,
                 board_scale: float | None = None) -> list[dict]:
    """Physical sanity checks a hand-eye result must pass before the table frame is trusted
    (28 Oct: a 13 px / 16.5 mm solution with the table 12 cm under the base was written as calibrated).
    ``table_z``: expected height of the board surface in the base frame (0 = base on the table top).
    ``board_scale``: the factor from ``estimate_board_scale`` (PnP distance vs FK geometry); when
    given, it must be within ``board_scale_tol`` of 1 (a wrong ``square_mm`` fails here).
    Returns [{name, ok, value, limit, text}]."""
    L = {**GATES, **(limits or {})}
    a = axis_offset_deg(res.T_tip_cam, nominal)
    d = float(np.linalg.norm(res.T_tip_cam[:3, 3]))
    z = float(res.T_base_table[2, 3]) - table_z
    rows = [("reprojection", res.rms_px, L["rms_px"], f"reprojection {res.rms_px:.2f} px (< {L['rms_px']:g})"),
            ("spread_mm", res.spread_mm, L["spread_mm"], f"board-pose spread {res.spread_mm:.2f} mm (< {L['spread_mm']:g})"),
            ("spread_deg", res.spread_deg, L["spread_deg"], f"board-pose spread {res.spread_deg:.2f} deg (< {L['spread_deg']:g})"),
            ("table_z", abs(z), L["table_z_m"], f"table origin z {res.T_base_table[2, 3]:+.3f} m vs expected {table_z:+.3f} "
                                                f"(|diff| < {L['table_z_m']:g})"),
            ("optical_axis", a, L["axis_deg"], f"optical axis {a:.1f} deg from the nominal 60 deg mounting (< {L['axis_deg']:g})"),
            ("camera_distance", d, L["cam_dist_m"], f"camera {d * 100:.1f} cm from the tip (< {L['cam_dist_m'] * 100:g})")]
    if board_scale is not None:
        rows.append(("board_scale", abs(board_scale - 1.0), L["board_scale_tol"],
                     f"board scale vs arm kinematics x{board_scale:.3f} (|x-1| < {L['board_scale_tol']:g}; "
                     f"PnP distances disagree with FK if not)"))
    return [{"name": n, "ok": bool(np.isfinite(v) and v < lim), "value": round(float(v), 4), "limit": lim, "text": t}
            for n, v, lim, t in rows]


def write_handeye(res: HandEyeResult, path=DEFAULT_HANDEYE) -> Path:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(res.to_dict(), indent=1) + "\n")
    return p


def write_workcell(T_base_table, path: Path = WORKCELL, note: str = "") -> tuple[Path, Path | None]:
    """Set T_base_table + calibrated:true in workcell.json, keeping every other key. The old file
    goes to workcell.json.bak-<timestamp>. Returns (path, backup)."""
    path = Path(path)
    raw = json.loads(path.read_text()) if path.exists() else {}
    backup = None
    if path.exists():
        backup = path.with_name(path.name + f".bak-{dt.datetime.now():%Y%m%d-%H%M%S}")
        shutil.copy2(path, backup)
    raw["T_base_table"] = [[round(float(v), 6) for v in row] for row in np.asarray(T_base_table, float)]
    raw["calibrated"] = True
    raw["notes"] = note or (f"T_base_table from wristcam hand-eye (board centre = table origin), "
                            f"{dt.datetime.now():%Y-%m-%d %H:%M}")
    path.write_text(json.dumps(raw, indent=2) + "\n")
    return path, backup


def load_samples_dir(folder, cam: CameraModel, spec: BoardSpec | None = None) -> list[Sample]:
    folder = Path(folder)
    meta = json.loads((folder / "samples.json").read_text())
    qs, frames, names = [], [], []
    for s in meta["samples"]:
        qs.append(s["q_measured"])
        frames.append(cv2.imread(str(folder / s["frame"]), cv2.IMREAD_COLOR))
        names.append(s["frame"])
    return make_samples(qs, frames, cam, spec, names=names)


# --------------------------------------------------------------------------- CLI
def _xyz(s):
    v = [float(x) for x in s.split(",")]
    if len(v) != 3:
        raise argparse.ArgumentTypeError("x,y,z in metres")
    return v


def cmd_plan(a):
    views = plan_views(a.board_center, a.n, purpose=a.purpose, seed=a.seed)
    print(f"{len(views)} views ({a.purpose}) around board centre {a.board_center} (base frame):")
    for i, v in enumerate(views):
        print(f"  {i:2d} q={v['q']}  tip={v['tip']}  cam h {v['cam_height']:.3f} m  tilt {v['tilt_deg']:4.1f} deg  "
              f"j6 roll {v['roll_rad']:+.2f}  board corners in view {v['visible']:.0%}")
    print("rotation diversity:", rotation_diversity([v["q"] for v in views]))
    print("planner tries / rejections:", getattr(plan_views, "last_stats", {}))
    if len(views) < a.n:
        print(f"WARNING: only {len(views)} of {a.n} views are reachable with the board in view: move the sheet "
              "where the arm reaches it from several sides (~0.25-0.35 m in front of the base).")
    if a.out:
        Path(a.out).write_text(json.dumps(views, indent=1) + "\n")
        print("wrote", a.out)
    return views


def cmd_collect(a):
    views = plan_views(a.board_center, a.n, purpose=a.purpose, seed=a.seed)
    out = Path(a.out)
    print(f"{len(views)} views planned; frames + joint angles go to {out}")
    if not a.execute:
        for i, v in enumerate(views):
            print(f"  {i:2d} move_joints q={v['q']} (tilt {v['tilt_deg']} deg, camera {v['cam_height']} m above the board)")
        print("dry run: nothing moved. Add --execute to drive the arm (each motion asks for approval).")
        return
    from .engine import DaemonArm
    out.mkdir(parents=True, exist_ok=True)
    samples = []
    with DaemonArm(source=a.source, program="wristcam-handeye", speed=a.speed, confirm=not a.yes) as arm:
        for i, v in enumerate(views):
            arm.move_joints(v["q"], label=f"hand-eye view {i + 1}/{len(views)}")
            time.sleep(a.settle)
            f = arm.frame()
            name = f"view_{i:02d}.png"
            cv2.imwrite(str(out / name), f)
            samples.append({"frame": name, "q_planned": v["q"], "q_measured": [round(float(x), 5) for x in arm.q()]})
            (out / "samples.json").write_text(json.dumps({"board_center": a.board_center, "samples": samples}, indent=1))
            print(f"  view {i + 1}: saved {name}")
        arm.ready()
    print(f"done: {len(samples)} views in {out}. Next: intrinsics solve --dir {out}; handeye solve --dir {out}")


def cmd_solve(a):
    cam = CameraModel.load(a.calib)
    spec = BoardSpec.load().with_measured(a.square_mm)
    samples = load_samples_dir(a.dir, cam, spec)
    print(f"{len(samples)} usable views")
    res = solve_handeye(samples, cam, spec)
    t = res.T_tip_cam[:3, 3] * 1000
    print(f"method {res.method}; reprojection {res.rms_px:.2f} px; board-pose spread {res.spread_mm:.2f} mm / "
          f"{res.spread_deg:.2f} deg over {res.n_views} views")
    for m, s in res.methods.items():
        print(f"   {m:10s} {s}")
    fingers = np.degrees(np.arccos(np.clip(-res.T_tip_cam[2, 2], -1, 1)))
    print(f"camera in tip frame: t = ({t[0]:.1f}, {t[1]:.1f}, {t[2]:.1f}) mm; optical axis {fingers:.1f} deg from the "
          f"fingers, {axis_offset_deg(res.T_tip_cam):.1f} deg from the nominal axis (full rotation vs nominal "
          f"{rot_err_deg(res.T_tip_cam, T_TIP_CAM_NOMINAL):.1f} deg); bracket prior: {res.extra.get('prior')}")
    o = res.T_base_table[:3, 3]
    print(f"table origin (board centre) in base: ({o[0]:.4f}, {o[1]:.4f}, {o[2]:.4f}) m, yaw "
          f"{np.degrees(np.arctan2(res.T_base_table[1, 0], res.T_base_table[0, 0])):.2f} deg, tilt "
          f"{np.degrees(np.arccos(np.clip(res.T_base_table[2, 2], -1, 1))):.2f} deg")
    # PnP distances vs the arm's geometry: a wrong printed square size scales every PnP distance
    k, sd = estimate_board_scale(samples, cam, res)
    res.extra["board_scale_fit"] = {"factor": round(k, 4), "sigma": round(sd, 4), "square_mm": round(spec.square_mm * k, 2)}
    print(f"board scale from the arm's kinematics: x{k:.3f} ± {sd:.3f} -> square ≈ {spec.square_mm * k:.2f} ± "
          f"{spec.square_mm * sd:.2f} mm (assumed {spec.square_mm:.2f}). If far from x1: measure a printed square with a "
          f"ruler and re-run with --square-mm (a fitted scale is never written to the workcell).")
    gates = sanity_gates(res, table_z=a.table_z, board_scale=k)
    res.extra["gates"] = gates
    failed = [g for g in gates if not g["ok"]]
    print("sanity gates:")
    for g in gates:
        print(f"   {'PASS' if g['ok'] else 'FAIL'}  {g['text']}")
    if failed:
        out = Path(a.out).with_name(Path(a.out).stem + f".rejected-{dt.datetime.now():%H%M%S}.json")
        print("wrote", write_handeye(res, out), "(REJECTED: not used by the grasp code)")
    else:
        print("wrote", write_handeye(res, a.out))
    if a.write_workcell:
        if failed:
            print(f"REFUSED to write {a.workcell}: {len(failed)} sanity gate(s) failed: "
                  + "; ".join(g["text"] for g in failed) + ". The workcell is unchanged.")
            return 2
        p, b = write_workcell(res.T_base_table, Path(a.workcell))
        print(f"wrote {p} (calibrated: true); backup {b}")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("plan", "collect"):
        s = sub.add_parser(name)
        s.add_argument("--board-center", type=_xyz, default=[0.30, 0.0, 0.0], help="x,y,z of the board centre, base frame")
        s.add_argument("--n", type=int, default=15)
        s.add_argument("--purpose", choices=("handeye", "intrinsics"), default="handeye")
        s.add_argument("--seed", type=int, default=0)
    sub.choices["plan"].add_argument("--out", default=None, help="write the views as JSON")
    c = sub.choices["collect"]
    c.add_argument("--out", required=True)
    c.add_argument("--source", default="dashboard")
    c.add_argument("--execute", action="store_true", help="really move the arm (default: print the plan)")
    c.add_argument("--yes", action="store_true", help="don't ask before each motion (NOT for the real arm)")
    c.add_argument("--speed", type=float, default=0.25, help="rad/s for move_joints (daemon caps at 0.3)")
    c.add_argument("--settle", type=float, default=1.5, help="seconds to wait before grabbing a frame")
    s = sub.add_parser("solve")
    s.add_argument("--dir", required=True)
    s.add_argument("--calib", default=str(DEFAULT_INTRINSICS))
    s.add_argument("--square-mm", type=float, default=None)
    s.add_argument("--out", default=str(DEFAULT_HANDEYE))
    s.add_argument("--write-workcell", action="store_true",
                   help="set T_base_table + calibrated:true, ONLY if every sanity gate passes")
    s.add_argument("--workcell", default=str(WORKCELL))
    s.add_argument("--table-z", type=float, default=0.0,
                   help="expected board-surface height in the base frame, m (e.g. -0.03 if the base sits on a 3 cm plate)")
    a = ap.parse_args(argv)
    r = {"plan": cmd_plan, "collect": cmd_collect, "solve": cmd_solve}[a.cmd](a)
    return r if isinstance(r, int) else 0


if __name__ == "__main__":
    raise SystemExit(main())
