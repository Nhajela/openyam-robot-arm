"""Kinematic corrections (armctl.kin KinCorrection) and the joint calibration that fits them
(experiments/2026-10-29-kin-calib/kincal.py), on synthetic data: no arm, no camera, no DimOS.

    .venv/bin/python -m pytest -q armctl/tests/test_kin_corrections.py
"""
from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from armctl import envelope as env
from armctl import kin

REPO = Path(__file__).resolve().parents[2]
KINCAL = REPO / "experiments" / "2026-10-29-kin-calib" / "kincal.py"


def _kincal():
    spec = importlib.util.spec_from_file_location("kincal", KINCAL)
    m = importlib.util.module_from_spec(spec)
    import sys
    sys.modules["kincal"] = m   # dataclasses need the module registered
    spec.loader.exec_module(m)
    return m


def test_no_correction_is_the_nominal_model():
    rng = np.random.default_rng(0)
    for _ in range(20):
        q = rng.uniform(env.DEFAULT.soft_lower, env.DEFAULT.soft_upper)
        assert np.allclose(kin.fk(q), kin.fk(q, kin.KinCorrection()))
        assert np.allclose(kin.fk(q), kin.fk(q, None))
    # pinocchio reference values in the module docstring
    assert np.allclose(kin.tip(env.POSES["home"]), [0.211, 0, 0.174], atol=1e-3)


def test_joint_offset_equals_shifted_q():
    q = np.array([0.1, 1.2, 0.9, -0.3, 0.2, 0.4])
    off = np.array([0.0, 0.03, -0.01, -0.04, -0.024, 0.0])
    c = kin.KinCorrection(joint_offsets=tuple(off))
    assert np.allclose(kin.fk(q, c), kin.fk(q + off))


def test_json_roundtrip_and_disabled(tmp_path):
    c = kin.KinCorrection((0, 0.01, 0, -0.04, -0.02, 0), ((0, 0, 0),) * 2 + ((0, 0, 0.001),) + ((0, 0, 0),) * 3)
    p = tmp_path / "kc.json"
    p.write_text(json.dumps({"enabled": True, "correction": c.to_dict()}))
    c2 = kin.load_correction(p)
    q = [0.0, 1.0, 1.0, 0.0, 0.0, 0.0]
    assert c2 is not None and np.allclose(kin.fk(q, c), kin.fk(q, c2))
    p.write_text(json.dumps({"enabled": False, "correction": c.to_dict()}))
    assert kin.load_correction(p) is None
    assert kin.load_correction(tmp_path / "missing.json") is None


def test_ik_with_correction_returns_dimos_joint_targets():
    c = kin.KinCorrection(joint_offsets=(0, 0.04, 0, -0.04, -0.024, 0))
    target = np.array([0.32, 0.05, 0.18])
    r = kin.ik(target, down=True, corr=c)
    assert r.ok
    # the corrected model puts the tip on target with the returned (DimOS) q; nominal FK does not
    assert np.linalg.norm(kin.tip(r.q, c) - target) < 0.006
    assert np.linalg.norm(kin.tip(r.q) - target) > 0.005
    assert env.DEFAULT.check_point(r.q) == [] or not env.DEFAULT.check_point(r.q)


# --------------------------------------------------------------------------- calibration recovery
QS = [[0.30, 0.75, 0.17, 0.32, 0.34, 0.35], [0.11, 1.33, 0.40, 0.43, 0.18, 0.43],
      [-0.31, 1.63, 0.79, 0.04, -0.45, 0.27], [-0.11, 1.71, 1.02, -0.23, -0.21, 0.43],
      [-0.22, 1.66, 0.97, -0.24, -0.39, 0.32], [-0.07, 1.93, 1.43, -0.72, -0.10, 0.51],
      [0.20, 1.10, 0.55, 0.10, 0.30, -0.40], [0.05, 1.45, 0.70, -0.10, 0.00, -0.30],
      [-0.15, 1.20, 0.35, 0.45, -0.25, -0.50], [0.25, 1.80, 1.20, -0.50, 0.35, 0.20],
      [0.00, 1.00, 0.30, 0.60, 0.10, 0.00], [-0.25, 1.50, 1.10, -0.40, 0.20, -0.20],
      [0.15, 1.70, 1.30, -0.60, -0.30, 0.60], [-0.05, 0.90, 0.25, 0.50, 0.40, -0.60]]


def _synthetic(m, truth: dict, noise_px=0.4, seed=0):
    from interfaces.wristcam.camera import CameraModel, inv_T, make_T, rot_axis
    cam = CameraModel("fisheye", [[537.0, 0, 640], [0, 533.0, 360], [0, 0, 1]], [0.05, -0.02, 0.01, 0.0], (1280, 720))
    T_tc = make_T(rot_axis([0, 1, 0], -45) @ np.diag([1.0, -1.0, -1.0]) @ rot_axis([0, 0, 1], 90), [-0.08, 0.003, 0.04])
    corr = m.correction(list(truth), list(truth.values()))
    rng = np.random.default_rng(seed)
    gx, gy = np.meshgrid(np.arange(8) * 0.022, np.arange(5) * 0.022)
    obj = np.c_[gx.ravel(), gy.ravel(), np.zeros(gx.size)]
    views = []
    # board under the camera: aim at where the nominal arm looks at view 0
    for d in range(2):
        views_d = []
        qs = QS if d == 1 else [list(np.array(q) + [0.15, 0, 0, 0, 0, 0]) for q in QS[:10]]
        F0 = kin.fk(qs[0], corr) @ T_tc
        # board origin where the camera axis meets z = -0.02
        o, a = F0[:3, 3], F0[:3, 2]
        t = (-0.02 - o[2]) / a[2]
        B = make_T(rot_axis([0, 0, 1], 10 + 30 * d) @ np.diag([1.0, -1.0, -1.0]), o + t * a - [0.07, 0.05, 0])
        for i, q in enumerate(qs):
            T_cb = inv_T(kin.fk(q, corr) @ T_tc) @ B
            uv = cam.project((obj * (1.0 if d == 1 else 1.1)) @ T_cb[:3, :3].T + T_cb[:3, 3])
            ok = np.isfinite(uv).all(1)
            if ok.sum() < 12:
                continue
            uv = uv[ok] + rng.normal(0, noise_px, (ok.sum(), 2))
            # the board in this dataset is printed at x1.1 of its spec (free scale for ds 0)
            o_ = obj[ok]
            from interfaces.wristcam.board import board_pose
            Tp, _ = board_pose(cam, o_, uv)
            views_d.append(m.View(d, f"s{d}/{i}", np.asarray(q, float), o_, uv, Tp, cam))
        views += views_d
    return views, T_tc


def test_calibration_recovers_injected_offsets():
    m = _kincal()
    truth = {"d2": 0.035, "d3": -0.010, "d4": -0.040, "d5": -0.024}
    views, T_tc = _synthetic(m, truth)
    assert sum(v.ds == 1 for v in views) >= 10 and sum(v.ds == 0 for v in views) >= 6
    T0 = T_tc.copy()
    T0[:3, 3] += [0.008, -0.005, 0.006]
    from interfaces.wristcam.camera import rot_axis
    T0[:3, :3] = rot_axis([1, 1, 0], 2.0) @ T0[:3, :3]
    p0 = m.Problem(views, [])
    r0 = p0.solve(m.init_x(p0, T0))
    p = m.Problem(views, list(truth))
    r = p.solve(m.init_x(p, T0))
    st0, st = m.stats(p0, r0.x), m.stats(p, r.x)
    assert st0["rms_px"] > 2.0              # hand-eye alone cannot explain the offsets
    assert st["rms_px"] < 0.8               # with them it fits to the noise
    idn = m.identifiability(p, r)
    for i, (n, v) in enumerate(truth.items()):
        assert abs(r.x[i] - v) < max(4 * idn["sigma"][i], 0.004), (n, r.x[i], v, idn["sigma"][i])
    _, T_fit, _, scales = p.split(r.x)
    assert np.linalg.norm(T_fit[:3, 3] - T_tc[:3, 3]) < 0.004
    assert abs(scales[0] - 1.1) < 0.01


def test_calibration_without_offsets_finds_none():
    m = _kincal()
    views, T_tc = _synthetic(m, {"d2": 0.0}, seed=3)
    p = m.Problem(views, ["d2", "d3", "d4", "d5"])
    r = p.solve(m.init_x(p, T_tc))
    assert np.all(np.abs(r.x[:4]) < 0.005), r.x[:4]
