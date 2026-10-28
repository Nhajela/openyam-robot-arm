"""Wrist-camera calibration (interfaces/wristcam) on synthetic renders: no camera, no arm.

    .venv/bin/python -m pytest -q interfaces/tests/test_wristcam_calib.py -s     (-s prints the numbers)

The renderer ray-casts every pixel through the TRUE lens (a ~100 deg fisheye) mounted at a known
offset from gripper_tip, at FK(q) for the planned views, onto a table with the taped calibration
sheet; supersampled, blurred, noisy, JPEG, jaws in the lower corners. Reported joint angles carry
encoder noise.
"""
from __future__ import annotations

import json
import shutil

import cv2
import numpy as np
import pytest

from armctl import kin
from interfaces.wristcam import board as boardm
from interfaces.wristcam import handeye as he
from interfaces.wristcam import intrinsics as intr
from interfaces.wristcam import synth
from interfaces.wristcam.camera import CameraModel, rot_err_deg

CENTER = (0.30, 0.0, 0.0)   # where the person tapes the sheet (roughly): the truth is 5 mm / 4 deg off


def _ray_err_deg(a: CameraModel, b: CameraModel, margin=0.08):
    w, h = a.size
    uv = np.stack(np.meshgrid(np.linspace(margin * w, (1 - margin) * w, 25),
                              np.linspace(margin * h, (1 - margin) * h, 15)), -1).reshape(-1, 2)
    r1, r2 = a.rays(uv), b.rays(uv)
    return np.degrees(np.arccos(np.clip((r1 * r2).sum(1), -1, 1)))


@pytest.fixture(scope="module")
def data():
    world = synth.World()
    rng = np.random.default_rng(5)
    vi = he.plan_views(CENTER, 20, purpose="intrinsics", seed=1)
    vh = he.plan_views(CENTER, 15, purpose="handeye", seed=0)
    fi = [world.render(synth.TRUE_CAMERA, kin.fk(v["q"]) @ synth.TRUE_T_TIP_CAM, seed=i) for i, v in enumerate(vi)]
    fh = [world.render(synth.TRUE_CAMERA, kin.fk(v["q"]) @ synth.TRUE_T_TIP_CAM, seed=100 + i) for i, v in enumerate(vh)]
    views, size, _ = intr.board_views(fi + fh, mask=synth.TRUE_CAMERA.jaw_mask())
    cam, info = intr.solve(views, size)
    q_meas = [np.asarray(v["q"]) + rng.normal(0, 0.001, 6) for v in vh]   # encoder noise
    return {"world": world, "vi": vi, "vh": vh, "fh": fh, "cam": cam, "info": info, "q_meas": q_meas}


def test_print_sheet_is_a_detectable_board():
    page = cv2.imread(str(boardm.PRINT_DIR / "wristcam_calib_A4.png"), cv2.IMREAD_GRAYSCALE)
    assert page is not None and (boardm.PRINT_DIR / "wristcam_calib_A4.pdf").exists()
    assert (boardm.PRINT_DIR / "wristcam_tags_A4.pdf").exists()
    small = cv2.resize(page, None, fx=0.4, fy=0.4, interpolation=cv2.INTER_AREA)
    d = boardm.BoardDetector().detect(small)
    spec = boardm.BoardSpec.load()
    assert d is not None and len(d[0]) == (spec.squares[0] - 1) * (spec.squares[1] - 1)
    # 50 mm scale bar: the board square measures 28 mm at 300 dpi
    assert spec.square_mm == 28.0 and spec.squares == (9, 6)


def test_table_frame_convention():
    spec = boardm.BoardSpec()
    T = spec.T_board_table
    w, h = spec.size_m
    # board centre is the table origin; board top-left corner (0,0) is table (-w/2, +h/2)
    assert np.allclose(boardm.table_points(spec, [[w / 2, h / 2, 0]]), 0)
    assert np.allclose(boardm.table_points(spec, [[0, 0, 0]]), [[-w / 2, h / 2, 0]])
    assert np.isclose(np.linalg.det(T[:3, :3]), 1.0)


def test_plan_views_feasible(data):
    for purpose, views, n in (("handeye", data["vh"], 15), ("intrinsics", data["vi"], 20)):
        assert len(views) == n, purpose
        for v in views:
            q = np.asarray(v["q"])
            assert he.check_pose(q) == "", (purpose, v)
            assert kin.clearance(q)[0] >= kin.MIN_TIP_Z
            if purpose == "handeye":
                assert 10.0 <= v["tilt_deg"] <= 35.0 and 0.14 <= v["cam_height"] <= 0.32, v
                assert 0.15 <= abs(v["roll_rad"]) <= 0.6
                outer = he.board_points_base(CENTER, boardm.BoardSpec(), outer_only=True)
                assert he.visible_fraction(q, outer, synth.TRUE_CAMERA, synth.TRUE_T_TIP_CAM, margin=0.0) >= 0.75
        tilts = [v["tilt_deg"] for v in views]
        assert max(tilts) - min(tilts) > 8          # varied
        rolls = [v["q"][5] for v in views]
        # the real camera sits ~60 deg off the fingers: big j6 rolls swing the board out of view,
        # so the surviving views vary roll less (heading and tilt still vary)
        assert max(rolls) - min(rolls) > 0.3
    # hand-eye views turn about more than one axis (j6 roll both ways + tilts toward many headings)
    rolls = [v["roll_rad"] for v in data["vh"]]
    assert min(rolls) < -0.15 and max(rolls) > 0.15
    div = he.rotation_diversity([v["q"] for v in data["vh"]])
    print("\nhand-eye view rotation diversity:", div)
    assert div["median_deg"] > 15 and div["axis_spread"] > 0.2


def test_intrinsics_recovery(data):
    cam, info = data["cam"], data["info"]
    print("\nintrinsics:", intr.report(cam, info))
    assert cam.model == "fisheye"
    assert cam.rms < 0.3
    assert abs(cam.K[0, 0] / synth.TRUE_CAMERA.K[0, 0] - 1) < 0.005
    assert np.allclose(cam.K[:2, 2], synth.TRUE_CAMERA.K[:2, 2], atol=2.0)
    err = _ray_err_deg(synth.TRUE_CAMERA, cam)
    print(f"ray direction error: median {np.median(err):.3f} deg, max {err.max():.3f} deg (inner 84%)")
    assert np.median(err) < 0.1 and err.max() < 0.5


def test_intrinsics_standard_lens_and_capture_io(tmp_path):
    """A narrower lens with ordinary radial distortion: whichever model wins (ties go to the one with
    fewer parameters) must reproduce the true rays, and the CLI solve from a folder writes a loadable model."""
    truth = CameraModel("pinhole", [[900.0, 0, 640.0], [0, 900.0, 362.0], [0, 0, 1]], [-0.11, 0.08, 0.0, 0.0, -0.02], (1280, 720))
    world = synth.World()
    views = he.plan_views(CENTER, 14, purpose="intrinsics", seed=3, cam=truth)
    for i, v in enumerate(views):
        cv2.imwrite(str(tmp_path / f"f{i:02d}.png"), world.render(truth, kin.fk(v["q"]) @ synth.TRUE_T_TIP_CAM, seed=i, jaws=False))
    out = tmp_path / "cam.json"
    intr.main(["solve", "--dir", str(tmp_path), "--out", str(out)])
    cam = CameraModel.load(out)
    err = _ray_err_deg(truth, cam, margin=0.15)
    print(f"\nstandard lens: chose {cam.model} rms {cam.rms:.3f}, candidates {cam.meta['candidates']}, "
          f"ray err median {np.median(err):.3f} max {err.max():.3f} deg")
    assert cam.model in ("pinhole", "rational", "fisheye")
    assert np.median(err) < 0.1 and err.max() < 0.6


def test_handeye_and_table_frame_recovery(data, tmp_path):
    cam = data["cam"]   # the RECOVERED lens, not the truth
    samples = he.make_samples(data["q_meas"], data["fh"], cam)
    assert len(samples) >= 10
    res = he.solve_handeye(samples, cam)
    rot = rot_err_deg(res.T_tip_cam, synth.TRUE_T_TIP_CAM)
    tr = np.linalg.norm(res.T_tip_cam[:3, 3] - synth.TRUE_T_TIP_CAM[:3, 3]) * 1000
    t_rot = rot_err_deg(res.T_base_table, synth.T_BASE_TABLE_TRUE)
    t_tr = np.linalg.norm(res.T_base_table[:3, 3] - synth.T_BASE_TABLE_TRUE[:3, 3]) * 1000
    print(f"\nhand-eye ({res.method}, {res.n_views} views, reproj {res.rms_px:.2f} px): rotation error {rot:.3f} deg, "
          f"translation error {tr:.2f} mm; table frame: {t_tr:.2f} mm, {t_rot:.3f} deg; methods {res.methods}")
    assert rot < 1.0 and tr < 3.0
    assert t_tr < 3.0 and t_rot < 0.5
    p = he.write_handeye(res, tmp_path / "he.json")
    assert np.allclose(json.loads(p.read_text())["T_tip_cam"], res.T_tip_cam, atol=1e-5)


def test_write_workcell_keeps_keys_and_backs_up(tmp_path):
    wc = tmp_path / "workcell.json"
    shutil.copy(he.WORKCELL, wc)
    before = json.loads(wc.read_text())
    T = synth.T_BASE_TABLE_TRUE
    p, backup = he.write_workcell(T, wc)
    after = json.loads(p.read_text())
    assert backup is not None and json.loads(backup.read_text()) == before
    assert after["calibrated"] is True and np.allclose(after["T_base_table"], T, atol=1e-6)
    for k in before:
        if k not in ("T_base_table", "calibrated", "notes"):
            assert after[k] == before[k]
    from interfaces.grasp.spots import _workcell
    assert _workcell(after).calibrated


# --------------------------------------------------------------------------- solver + sanity gates
def _exact_samples(views, T_tc, T_bb, spec=None):
    spec = spec or boardm.BoardSpec()
    obj = spec.board().getChessboardCorners().astype(float)
    out = []
    for v in views:
        q = np.asarray(v["q"], float)
        T_cb = he.inv_T(kin.fk(q) @ T_tc) @ T_bb
        out.append(he.Sample(q, obj, np.zeros((len(obj), 2)), T_cb, 0.0))
    return out


def test_robot_world_hand_eye_solver_exact(data):
    """calibrateRobotWorldHandEye recovers camera-on-tip AND board-in-base from noise-free poses."""
    T_bb = synth.T_BASE_TABLE_TRUE @ he.inv_T(boardm.BoardSpec().T_board_table)
    samples = _exact_samples(data["vh"], synth.TRUE_T_TIP_CAM, T_bb)
    for m in he.RWHE_METHODS:
        T_tc, T_b = he.robot_world_hand_eye(samples, m)
        assert rot_err_deg(T_tc, synth.TRUE_T_TIP_CAM) < 0.05, m
        assert np.linalg.norm(T_tc[:3, 3] - synth.TRUE_T_TIP_CAM[:3, 3]) < 1e-3, m
        assert np.linalg.norm(T_b[:3, 3] - T_bb[:3, 3]) < 1e-3 and rot_err_deg(T_b, T_bb) < 0.05, m


def test_bracket_prior():
    N = he.T_TIP_CAM_NOMINAL
    assert he.prior_violations(N) == []
    rolled = N @ he.make_T(np.diag([-1.0, -1.0, 1.0]))          # roll about the axis: allowed
    assert he.prior_violations(rolled) == [] and he.axis_offset_deg(rolled) < 1e-6
    flipped = N @ he.make_T(np.diag([1.0, -1.0, -1.0]))         # looking backwards (28 Oct)
    assert any("optical axis" in v for v in he.prior_violations(flipped))
    far = N.copy()
    far[:3, 3] = [0.0, 0.0, 0.2]
    assert any("from the tip" in v for v in he.prior_violations(far))


def _result(**kw):
    T_bt = he.make_T(t=[0.3, 0.0, kw.pop("table_z", 0.0)])
    base = dict(T_tip_cam=he.T_TIP_CAM_NOMINAL.copy(), T_base_board=T_bt, T_base_table=T_bt, method="x", methods={},
                rms_px=0.8, per_view_px=[0.8], n_views=15, spread_mm=1.5, spread_deg=0.2)
    base.update(kw)
    return he.HandEyeResult(**base)


def test_sanity_gates_pass_and_fail():
    assert all(g["ok"] for g in he.sanity_gates(_result()))
    # the rejected 28 Oct solution: 13 px, 16.5 mm, table 12.5 cm under the base, camera looking backwards
    bad = _result(rms_px=13.4, spread_mm=16.5, spread_deg=1.19, table_z=-0.125,
                  T_tip_cam=he.make_T(he.T_TIP_CAM_NOMINAL[:3, :3] @ np.diag([1.0, -1.0, -1.0]), [-0.095, 0.03, 0.048]))
    failed = {g["name"] for g in he.sanity_gates(bad) if not g["ok"]}
    assert failed == {"reprojection", "spread_mm", "spread_deg", "table_z", "optical_axis"}
    far = _result(T_tip_cam=he.make_T(he.T_TIP_CAM_NOMINAL[:3, :3], [0.0, 0.12, 0.12]))
    assert [g["name"] for g in he.sanity_gates(far) if not g["ok"]] == ["camera_distance"]
    # a base raised on a plate: the expected table height is a parameter, not a loophole
    # PnP distances disagreeing with the arm's geometry (wrong square size) fail the scale gate
    assert all(g["ok"] for g in he.sanity_gates(_result(), board_scale=1.02))
    assert [g["name"] for g in he.sanity_gates(_result(), board_scale=0.75) if not g["ok"]] == ["board_scale"]
    raised = _result(table_z=-0.045)
    assert not all(g["ok"] for g in he.sanity_gates(raised))
    assert all(g["ok"] for g in he.sanity_gates(raised, table_z=-0.04))


def _write_handeye_folder(data, folder):
    folder.mkdir()
    rows = []
    for i, (q, f) in enumerate(zip(data["q_meas"], data["fh"])):
        cv2.imwrite(str(folder / f"view_{i:02d}.png"), f)
        rows.append({"frame": f"view_{i:02d}.png", "q_measured": [float(x) for x in q]})
    (folder / "samples.json").write_text(json.dumps({"samples": rows}))
    return data["cam"].save(folder / "cam.json")


def test_solve_cli_writes_workcell_only_when_gates_pass(data, tmp_path, capsys):
    folder = tmp_path / "he"
    calib = _write_handeye_folder(data, folder)
    wc = tmp_path / "workcell.json"
    shutil.copy(he.WORKCELL, wc)
    before = wc.read_text()
    args = ["solve", "--dir", str(folder), "--calib", str(calib), "--workcell", str(wc), "--write-workcell"]
    # wrong printed square size (the 28 Oct failure mode): scale error -> gates fail -> refused
    rc = he.main(args + ["--square-mm", "20", "--out", str(tmp_path / "bad.json")])
    out = capsys.readouterr().out
    assert rc == 2 and "REFUSED" in out and "FAIL" in out
    assert wc.read_text() == before and not (tmp_path / "bad.json").exists()
    assert list(tmp_path.glob("bad.rejected-*.json"))
    # right square size -> every gate passes -> calibrated
    rc = he.main(args + ["--out", str(tmp_path / "good.json")])
    out = capsys.readouterr().out
    assert rc == 0 and "REFUSED" not in out
    after = json.loads(wc.read_text())
    assert after["calibrated"] is True and (tmp_path / "good.json").exists()


def test_board_scale_estimate_finds_a_wrong_square(data):
    """The arm's kinematics carry metric scale: a board declared 25% too large is detected."""
    cam = data["cam"]
    spec = boardm.BoardSpec().with_measured(35.0)          # truth is 28 mm
    samples = he.make_samples(data["q_meas"], data["fh"], cam, spec)
    res = he.solve_handeye(samples, cam, spec)
    k, sd = he.estimate_board_scale(samples, cam, res)
    print(f"\nscale factor {k:.4f} ± {sd:.4f} (truth {28 / 35:.4f})")
    assert abs(k - 28 / 35) < 0.01
