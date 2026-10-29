"""Joint kinematic calibration from wrist-camera views of a fixed board (offline, no arm).

Jointly fits joint zero offsets (+ optional link-length deltas and joint compliance), the camera
on the tip (T_tip_cam) and one board pose per dataset by minimising the reprojection of every
detected corner with the calibrated fisheye lens (soft-L1 loss). Model comparison, identifiability
(normalised Jacobian SVD, sigmas, correlations), leave-one-view-out and cross-dataset prediction,
and the hand-eye sanity gates with the best model.

    .venv/bin/python -I experiments/2026-10-29-kin-calib/kincal.py \
        --data /home/naman/openyam-robot-arm/interfaces/wristcam/data  [--loo] [--write]

Reads only: <data>/he1, <data>/he2 (frames + samples.json q_measured), interfaces/wristcam/calib/
wristcam.json, interfaces/wristcam/print/printed_{tags_25mm,vision_board}.json. Writes (with
--write) only into this experiment folder and armctl/kin_corrections.json of THIS checkout.
Nothing here talks to the arm, DimOS, zenoh or a camera.

Gauge: a j1 offset is a rotation of everything about the base z axis -> absorbed by the free board
poses; a j6 offset is a rotation about the last axis -> absorbed by T_tip_cam. Both are fixed at 0.
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO))

from armctl import kin  # noqa: E402
from interfaces.wristcam import handeye as he  # noqa: E402
from interfaces.wristcam.board import BoardSpec  # noqa: E402
from interfaces.wristcam.camera import CameraModel, inv_T, make_T, mean_pose, rot_err_deg  # noqa: E402

PRINT = REPO / "interfaces" / "wristcam" / "print"
LENS = REPO / "interfaces" / "wristcam" / "calib" / "wristcam.json"
HE1_SQUARE_MM = 22.9      # he1 ChArUco: printed square unknown; its scale is a free parameter anyway

# kinematic parameter name -> (kind, index[, axis])
KIN_PARAMS = {
    "d2": ("off", 1), "d3": ("off", 2), "d4": ("off", 3), "d5": ("off", 4),
    # Link LENGTHS in the plane of the parallel j2/j3/j4 axes (their y axes). A change of a link's
    # DIRECTION in that plane is exactly a combination of the j2..j4 offsets, so only lengths are
    # identifiable (an xyz delta of joint4's origin made the problem singular: cond 4e8).
    "Lu": ("len", 2),       # upper arm j2->j3 (joint3 origin, xz part 0.264 m)
    "Lf": ("len", 3),       # forearm j3->j4 (joint4 origin, xz part 0.252 m)
    "c2": ("cmp", 1), "c3": ("cmp", 2), "c4": ("cmp", 3),
}
MODELS = {
    "M0 hand-eye only": [],
    "M1 + offsets j2-j5": ["d2", "d3", "d4", "d5"],
    # j2's offset is a rotation of everything distal about the j2 axis: with a free board pose it is
    # only observable through the j1 spread of the views (corr with board z -0.95), so try without
    "M1a offsets j4 j5": ["d4", "d5"],
    "M1b offsets j3 j4 j5": ["d3", "d4", "d5"],
    "M2 + offsets + links": ["d2", "d3", "d4", "d5", "Lu", "Lf"],
    "M3 + offsets + compliance": ["d2", "d3", "d4", "d5", "c2", "c3", "c4"],
    "M4 offsets + links + compliance": ["d2", "d3", "d4", "d5", "Lu", "Lf", "c2", "c3", "c4"],
    # offsets fitted separately per dataset (power-cycle / re-zero between 28 and 29 Oct?)
    "M5 offsets per dataset": ["d2@0", "d3@0", "d4@0", "d5@0", "d2@1", "d3@1", "d4@1", "d5@1"],
}
UNITS = {"off": ("mrad", 1e3), "org": ("mm", 1e3), "len": ("mm", 1e3), "cmp": ("mrad/(N*m)", 1e3)}


def _kp(name):
    """'d4@1' -> (KIN_PARAMS['d4'], 1); 'd4' -> (..., None)."""
    base, _, ds = name.partition("@")
    return KIN_PARAMS[base], (int(ds) if ds else None)


def _len_dir(i):
    xyz = np.array(kin.CHAIN[i][2], float)
    d = np.array([xyz[0], 0.0, xyz[2]])
    return d / np.linalg.norm(d)


@dataclass
class View:
    ds: int
    name: str
    q: np.ndarray
    obj: np.ndarray
    img: np.ndarray
    T_cam_board: np.ndarray
    cam: CameraModel


def correction(names, vals, ds=None) -> kin.KinCorrection:
    """KinCorrection from fitted parameters; per-dataset parameters ('d4@1') only for ``ds``."""
    off, cmp_ = np.zeros(6), np.zeros(6)
    org = np.zeros((6, 3))
    for n, v in zip(names, vals):
        k, only = _kp(n)
        if only is not None and only != ds:
            continue
        if k[0] == "len":
            org[k[1]] += v * _len_dir(k[1])
        elif k[0] == "off":
            off[k[1]] = v
        elif k[0] == "cmp":
            cmp_[k[1]] = v
        else:
            org[k[1], k[2]] = v
    return kin.KinCorrection(tuple(off), tuple(map(tuple, org)), (0.0, 0.0, 0.0), tuple(cmp_))


def load_views(data: Path, lens: CameraModel) -> tuple[list[View], list[BoardSpec], list[str]]:
    specs = [BoardSpec.load(PRINT / "printed_vision_board.json").with_measured(HE1_SQUARE_MM),
             BoardSpec.load(PRINT / "printed_tags_25mm.json")]
    names = ["he1", "he2"]
    views = []
    for k, (nm, sp) in enumerate(zip(names, specs)):
        for s in he.load_samples_dir(data / nm, lens, sp):
            views.append(View(k, f"{nm}/{s.name}", s.q, s.obj, s.img, s.T_cam_board, lens))
    return views, specs, names


class Problem:
    """x = [kin params | T_tip_cam (rvec, t) | per dataset: board (rvec, t) | free scales]."""

    def __init__(self, views, kin_names, free_scale=(True, False), n_ds=2):
        self.views, self.kn, self.n_ds = views, list(kin_names), n_ds
        self.free_scale = list(free_scale)
        self.nk = len(self.kn)
        self.ns = sum(self.free_scale)

    def split(self, x):
        k = x[:self.nk]
        T_tc = make_T(cv2.Rodrigues(x[self.nk:self.nk + 3])[0], x[self.nk + 3:self.nk + 6])
        o = self.nk + 6
        boards = [make_T(cv2.Rodrigues(x[o + 6 * d:o + 6 * d + 3])[0], x[o + 6 * d + 3:o + 6 * d + 6])
                  for d in range(self.n_ds)]
        o += 6 * self.n_ds
        scales, j = [], 0
        for d in range(self.n_ds):
            if self.free_scale[d]:
                scales.append(x[o + j]); j += 1
            else:
                scales.append(1.0)
        return k, T_tc, boards, scales

    def pack(self, k, T_tc, boards, scales):
        x = list(k) + list(cv2.Rodrigues(T_tc[:3, :3])[0].ravel()) + list(T_tc[:3, 3])
        for B in boards:
            x += list(cv2.Rodrigues(B[:3, :3])[0].ravel()) + list(B[:3, 3])
        x += [s for s, f in zip(scales, self.free_scale) if f]
        return np.array(x, float)

    def view_res(self, v: View, corr, T_tc, B, s):
        F = kin.fk(v.q, corr)
        T_cb = inv_T(F @ T_tc) @ B
        uv = v.cam.project((v.obj * s) @ T_cb[:3, :3].T + T_cb[:3, 3])
        r = (uv - v.img).ravel()
        return np.where(np.isfinite(r), r, 50.0)

    def residuals(self, x, views=None):
        k, T_tc, boards, scales = self.split(x)
        corrs = [correction(self.kn, k, d) for d in range(self.n_ds)]
        return np.concatenate([self.view_res(v, corrs[v.ds], T_tc, boards[v.ds], scales[v.ds])
                               for v in (self.views if views is None else views)])

    def solve(self, x0, views=None, f_scale=2.0):
        return least_squares(lambda x: self.residuals(x, views), x0, loss="soft_l1", f_scale=f_scale,
                             x_scale="jac", max_nfev=400)


def init_x(prob: Problem, T_tc0, prev=None):
    """Board poses from the per-view estimates with the nominal model."""
    if prev is not None:
        return prev
    boards = []
    for d in range(prob.n_ds):
        Ts = [kin.fk(v.q) @ T_tc0 @ v.T_cam_board for v in prob.views if v.ds == d]
        boards.append(mean_pose(Ts))
    return prob.pack(np.zeros(prob.nk), T_tc0, boards, [1.0] * prob.n_ds)


def stats(prob: Problem, x, views=None):
    views = prob.views if views is None else views
    k, T_tc, boards, scales = prob.split(x)
    corrs = [correction(prob.kn, k, d) for d in range(prob.n_ds)]
    per_view, per_ds = [], {d: [] for d in range(prob.n_ds)}
    spread = {d: ([], []) for d in range(prob.n_ds)}
    for v in views:
        corr = corrs[v.ds]
        r = prob.view_res(v, corr, T_tc, boards[v.ds], scales[v.ds]).reshape(-1, 2)
        e = np.sum(r ** 2, axis=1)
        per_view.append(float(np.sqrt(e.mean())))
        per_ds[v.ds].append(e)
        Tcb = v.T_cam_board.copy()
        Tcb[:3, 3] *= scales[v.ds]           # PnP distances scale with the board
        Tb = kin.fk(v.q, corr) @ T_tc @ Tcb
        spread[v.ds][0].append(np.linalg.norm(Tb[:3, 3] - boards[v.ds][:3, 3]))
        spread[v.ds][1].append(rot_err_deg(Tb, boards[v.ds]))
    out = {"per_view_px": per_view}
    for d in range(prob.n_ds):
        if per_ds[d]:
            out[f"rms_px_{d}"] = float(np.sqrt(np.concatenate(per_ds[d]).mean()))
            out[f"spread_mm_{d}"] = float(np.sqrt(np.mean(np.square(spread[d][0])))) * 1000
            out[f"spread_deg_{d}"] = float(np.sqrt(np.mean(np.square(spread[d][1]))))
    out["rms_px"] = float(np.sqrt(np.concatenate([e for d in per_ds.values() for e in d]).mean()))
    return out


def identifiability(prob: Problem, r):
    J = r.jac
    e = r.fun
    dof = max(1, e.size - J.shape[1])
    s2 = float(e @ e / dof)
    cn = np.linalg.norm(J, axis=0)
    Jn = J / np.where(cn > 0, cn, 1)
    sv = np.linalg.svd(Jn, compute_uv=False)
    cov = np.linalg.pinv(J.T @ J) * s2
    sd = np.sqrt(np.maximum(np.diag(cov), 0))
    with np.errstate(divide="ignore", invalid="ignore"):
        corr = cov / np.outer(sd, sd)
    return {"sigma": sd, "corr": corr, "sv": sv / sv[0], "cond": float(sv[0] / sv[-1]), "s_px": float(np.sqrt(s2))}


def param_labels(prob: Problem):
    lab = list(prob.kn) + ["tc_rx", "tc_ry", "tc_rz", "tc_x", "tc_y", "tc_z"]
    for d in range(prob.n_ds):
        lab += [f"b{d}_{c}" for c in ("rx", "ry", "rz", "x", "y", "z")]
    lab += [f"scale{d}" for d in range(prob.n_ds) if prob.free_scale[d]]
    return lab


def predict_view(prob: Problem, x, v: View):
    """Prediction error of a held-out view: reprojection (px) and board-origin error (mm)."""
    k, T_tc, boards, scales = prob.split(x)
    corr = correction(prob.kn, k, v.ds)
    r = prob.view_res(v, corr, T_tc, boards[v.ds], scales[v.ds]).reshape(-1, 2)
    Tcb = v.T_cam_board.copy()
    Tcb[:3, 3] *= scales[v.ds]
    Tb = kin.fk(v.q, corr) @ T_tc @ Tcb
    return float(np.sqrt(np.mean(np.sum(r ** 2, 1)))), float(np.linalg.norm(Tb[:3, 3] - boards[v.ds][:3, 3]) * 1000)


def loo(prob: Problem, x_full):
    px, mm = [], []
    for i, v in enumerate(prob.views):
        rest = [w for j, w in enumerate(prob.views) if j != i]
        r = prob.solve(x_full, rest)
        a, b = predict_view(prob, r.x, v)
        px.append(a); mm.append(b)
    return np.array(px), np.array(mm)


def cross_dataset(views, kn, x_full, T_tc0):
    """Fit kinematics + hand-eye on one dataset, then only the other dataset's board pose (and
    scale for he1): how well do the corrections transfer?"""
    out = {}
    for src in (0, 1):
        dst = 1 - src
        sv = [v for v in views if v.ds == src]
        p1 = Problem(views, kn)
        x0 = x_full.copy()
        r1 = p1.solve(x0, sv)
        k, T_tc, boards, scales = p1.split(r1.x)
        # refit only dst board (+ scale if free)
        dv = [v for v in views if v.ds == dst]
        o = p1.nk + 6 + 6 * dst
        idx = list(range(o, o + 6))
        if p1.free_scale[dst]:
            idx.append(p1.nk + 6 + 6 * p1.n_ds)

        def f(z, base=r1.x.copy()):
            xx = base.copy(); xx[idx] = z
            return p1.residuals(xx, dv)
        # re-seed the dst board from its own PnP poses through the src-fitted model
        corr = correction(p1.kn, k, dst)
        B0 = mean_pose([kin.fk(v.q, corr) @ T_tc @ v.T_cam_board for v in dv])
        z0 = r1.x[idx].copy()
        z0[:3] = cv2.Rodrigues(B0[:3, :3])[0].ravel()
        z0[3:6] = B0[:3, 3]
        if p1.free_scale[dst]:
            z0[6] = 1.0
        rz = least_squares(f, z0, loss="soft_l1", f_scale=2.0, x_scale="jac")
        xx = r1.x.copy(); xx[idx] = rz.x
        st = stats(p1, xx, dv)
        out[f"he{src + 1}->he{dst + 1}"] = {"rms_px": round(st[f"rms_px_{dst}"], 2),
                                            "spread_mm": round(st[f"spread_mm_{dst}"], 2),
                                            "spread_deg": round(st[f"spread_deg_{dst}"], 3)}
    return out


def fmt_kin(prob, x, ident):
    rows = []
    for i, n in enumerate(prob.kn):
        u, f = UNITS[_kp(n)[0][0]]
        rows.append(f"{n}={x[i] * f:+.2f}±{ident['sigma'][i] * f:.2f} {u}")
    return ", ".join(rows)


def run(args):
    lens = CameraModel.load(LENS)
    views, specs, names = load_views(Path(args.data), lens)
    print(f"views: he1 {sum(v.ds == 0 for v in views)}, he2 {sum(v.ds == 1 for v in views)}; "
          f"corners {sum(len(v.obj) for v in views)}; PnP per view median "
          f"{np.median([he.board_pose(v.cam, v.obj, v.img)[1] for v in views]):.2f} px")
    # hand-eye seed with the nominal model on he2 (the pipeline's own solver)
    s2 = [he.Sample(v.q, v.obj, v.img, v.T_cam_board, 0.0, v.name) for v in views if v.ds == 1]
    seed = he.solve_handeye(s2, lens, specs[1])
    print(f"pipeline he2 (nominal FK): {seed.method} {seed.rms_px:.2f} px, spread {seed.spread_mm:.2f} mm / "
          f"{seed.spread_deg:.2f} deg, t_tc {np.round(seed.T_tip_cam[:3, 3] * 1000, 1)} mm")
    T_tc0 = seed.T_tip_cam
    results = {}
    prev_x = None
    for mname, kn in MODELS.items():
        if args.models and not any(mname.startswith(m + " ") for m in args.models.split(",")):
            continue
        prob = Problem(views, kn)
        x0 = init_x(prob, T_tc0)
        if prev_x is not None:   # warm start the shared part
            pk, pT, pB, pS = prev_x
            x0 = prob.pack(np.zeros(prob.nk), pT, pB, pS)
        r = prob.solve(x0)
        # second start from the nominal seed to avoid a bad basin
        r2 = prob.solve(init_x(prob, T_tc0))
        if r2.cost < r.cost:
            r = r2
        st = stats(prob, r.x)
        idn = identifiability(prob, r)
        k, T_tc, boards, scales = prob.split(r.x)
        prev_x = (k, T_tc, boards, scales)
        res = {"x": r.x.tolist(), "kin": dict(zip(kn, r.x[:prob.nk].tolist())),
               "kin_sigma": dict(zip(kn, idn["sigma"][:prob.nk].tolist())), "stats": st,
               "cond": idn["cond"], "sv": idn["sv"].tolist(), "s_px": idn["s_px"], "T_tip_cam": T_tc.tolist(),
               "boards": [B.tolist() for B in boards], "scales": scales, "n_params": len(r.x), "cost": float(r.cost)}
        print(f"\n== {mname}: {len(r.x)} params, rms {st['rms_px']:.2f} px (he1 {st['rms_px_0']:.2f}, he2 "
              f"{st['rms_px_1']:.2f}); spread he1 {st['spread_mm_0']:.1f} mm/{st['spread_deg_0']:.2f} deg, he2 "
              f"{st['spread_mm_1']:.1f} mm/{st['spread_deg_1']:.2f} deg; he1 scale x{scales[0]:.4f} "
              f"(square {HE1_SQUARE_MM * scales[0]:.2f} mm); cond {idn['cond']:.0f}")
        print(f"   t_tc {np.round(T_tc[:3, 3] * 1000, 1)} mm, axis-vs-fingers "
              f"{np.degrees(np.arccos(np.clip(-T_tc[2, 2], -1, 1))):.1f} deg")
        if kn:
            print("   " + fmt_kin(prob, r.x, idn))
            lab = param_labels(prob)
            C = idn["corr"]
            big = [(lab[i], lab[j], C[i, j]) for i in range(prob.nk) for j in range(len(lab))
                   if j != i and (j >= prob.nk or j > i) and abs(C[i, j]) > 0.8]
            if big:
                print("   |corr|>0.8: " + ", ".join(f"{a}~{b} {c:+.2f}" for a, b, c in big))
            res["high_corr"] = [(a, b, round(float(c), 3)) for a, b, c in big]
            res["corr_kin"] = np.round(C[:prob.nk, :prob.nk], 3).tolist()
        print(f"   per-view px: {np.round(st['per_view_px'], 1).tolist()}")
        if args.loo:
            px, mm = loo(prob, r.x)
            res["loo_px"], res["loo_mm"] = px.tolist(), mm.tolist()
            print(f"   leave-one-out: reprojection median {np.median(px):.2f} / rms {np.sqrt(np.mean(px ** 2)):.2f} px; "
                  f"board origin err median {np.median(mm):.1f} / rms {np.sqrt(np.mean(mm ** 2)):.1f} mm "
                  f"(he1 rms {np.sqrt(np.mean(mm[[v.ds == 0 for v in views]] ** 2)):.1f}, he2 rms "
                  f"{np.sqrt(np.mean(mm[[v.ds == 1 for v in views]] ** 2)):.1f})")
            cd = cross_dataset(views, kn, r.x, T_tc0)
            res["cross"] = cd
            print(f"   cross-dataset: {cd}")
        results[mname] = res
    out = HERE / "logs" / f"kincal_results{args.tag}.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(results, indent=1, default=float) + "\n")
    print("wrote", out)
    return views, specs, results


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(REPO / "interfaces" / "wristcam" / "data"))
    ap.add_argument("--loo", action="store_true")
    ap.add_argument("--models", default="", help="comma list of model ids to run, e.g. M0,M1a")
    ap.add_argument("--tag", default="", help="suffix for logs/kincal_results<tag>.json")
    run(ap.parse_args())
