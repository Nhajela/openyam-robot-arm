"""interfaces.follow.look: a camera-down look pose over a table point (the start of a follow).

The same construction as ``interfaces.wristcam.engine.GraspEngine.camera_over``, without an
arm: IK for "camera straight down" with the tip placed so that the optical axis passes through
(xy, z_top) at ``h`` above it, iterated because the camera sits off the tip. Lowers h in 1 cm
steps when the pose can't be reached. Checked against the soft box and the table rule.
"""
from __future__ import annotations

import numpy as np

from armctl import envelope as envm
from armctl import kin

from interfaces.wristcam.camera import finger_dir_for_cam_axis


def _ik_cam_down(xyz_base, seed, T_tip_cam):
    f = finger_dir_for_cam_axis([0.0, 0.0, -1.0], xyz_base, T_tip_cam)
    r = kin.ik(xyz_base, seed, approach_dir=f, check_table=False, restarts=3, max_iters=200,
               corr=kin.vision_correction())
    if r.err_pos > 0.004 or r.err_rot > 0.08:
        return None
    q = r.q.copy()
    q[5] = 0.0     # keep the camera in the arm's vertical plane
    return q


def look_pose(xy_table, z_top: float, h: float, T_tip_cam, T_base_table, seed=None,
              h_min: float = 0.06) -> np.ndarray | None:
    """q with the camera looking straight down at (xy, z_top) from h above it (or lower), or None."""
    T_bt = np.asarray(T_base_table, float)
    T_tc = np.asarray(T_tip_cam, float)
    target = np.asarray(xy_table, float)[:2]
    seed = np.array(envm.POSES["ready"], float) if seed is None else np.asarray(seed, float)
    while h >= h_min - 1e-9:
        tip_xy = target.copy()
        q, s = None, seed
        for _ in range(5):
            Tv = kin.fk_vision(s)
            cam_above_tip = float((Tv[:3, :3] @ T_tc[:3, 3])[2])
            p = (T_bt @ np.r_[tip_xy, z_top + h - cam_above_tip, 1.0])[:3]
            q = _ik_cam_down(p, s, T_tc)
            if q is None:
                break
            Tc = np.linalg.inv(T_bt) @ kin.fk_vision(q) @ T_tc
            C, a = Tc[:3, 3], Tc[:3, 2]
            if a[2] >= -1e-6:
                q = None
                break
            hit = (C + (z_top - C[2]) / a[2] * a)[:2]
            err = target - hit
            tip_xy, s = tip_xy + err, q
            if np.linalg.norm(err) < 3e-4:
                break
        if q is not None and not envm.DEFAULT.check_point(q) and kin.clear_of_table(q):
            return q
        h -= 0.01
    return None
