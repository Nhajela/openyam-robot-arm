"""Object localisation from one wrist frame (interfaces/wristcam/locate.py), synthetic renders.

    .venv/bin/python -m pytest -q interfaces/tests/test_wristcam_locate.py -s
"""
from __future__ import annotations

import numpy as np
import pytest

from armctl import kin
from interfaces.wristcam import locate, synth

CUP_XY = (-0.01, -0.16)
BOX_XY = (0.03, 0.17)


@pytest.fixture(scope="module")
def world():
    return synth.World(cups=[synth.Cup(CUP_XY)], boxes=[synth.Box(BOX_XY, size=(0.05, 0.05, 0.04), tag_id=2, yaw=0.4)])


def _look_at(world, target_table, dist, tilt_deg=18.0, az=np.pi):
    """q whose camera is ``dist`` from the target, looking at it with the given tilt."""
    T = world.T_base_table
    tgt = (T @ np.r_[target_table, 1.0])[:3]
    t = np.radians(tilt_deg)
    a = np.array([np.sin(t) * np.cos(az), np.sin(t) * np.sin(az), -np.cos(t)])
    cam_pos = tgt - dist * a
    tip = cam_pos.copy()
    for _ in range(3):
        r = kin.ik(tip, approach_dir=a, check_table=False, restarts=4)
        assert r.ok, r.reason
        tip = tip + cam_pos - (kin.fk(r.q) @ synth.TRUE_T_TIP_CAM)[:3, 3]
    return r.q


def _view(world, q):
    return locate.View.from_q(q, synth.TRUE_CAMERA, synth.TRUE_T_TIP_CAM, world.T_base_table)


# The camera 25 cm from the rim needs a view tilted down-and-forward (exactly-down views only reach
# tip z ~0.21 m, i.e. the camera ~0.18 m above the rim); plus a near-vertical view 25 cm from the
# table under the cup.
@pytest.mark.parametrize("target_z,dist,tilt,az", [(0.08, 0.25, 30.0, 0.0), (0.08, 0.24, 20.0, 0.0),
                                                   (0.0, 0.25, 10.0, -np.pi / 2)])
def test_cup_at_25cm(world, target_z, dist, tilt, az):
    q = _look_at(world, [CUP_XY[0], CUP_XY[1], target_z], dist, tilt, az)
    img = world.render(synth.TRUE_CAMERA, kin.fk(q) @ synth.TRUE_T_TIP_CAM, seed=11)
    d = locate.CupFinder().find(img, _view(world, q))
    assert d is not None
    err = np.hypot(d.xyz_table[0] - CUP_XY[0], d.xyz_table[1] - CUP_XY[1]) * 1000
    print(f"\ncup, camera {dist} m from table z={target_z}, tilt {tilt}: error {err:.2f} mm, "
          f"diameter {d.diameter * 1000:.1f} mm, sigma {d.sigma_xy * 1000:.1f} mm")
    assert err < 3.0
    assert abs(d.diameter - 0.06) < 0.003
    assert d.sigma_xy < 0.006


def test_cup_partly_under_the_jaws(world, monkeypatch):
    """Bigger jaws than the default guess (the real ones are not measured yet), and a straight-down look
    where >= 10% of the rim is behind a jaw and at most 40% is lost in total (jaw or off-image)."""
    from interfaces.wristcam import camera as camm
    big = [[(0.0, 0.45), (0.38, 1.0), (0.0, 1.0)], [(1.0, 0.45), (0.62, 1.0), (1.0, 1.0)]]
    monkeypatch.setattr(camm, "JAW_POLYGONS", big)
    monkeypatch.setattr(synth, "JAW_POLYGONS", big)
    cam, mask = synth.TRUE_CAMERA, synth.TRUE_CAMERA.jaw_mask()
    a = np.linspace(0, 2 * np.pi, 60, endpoint=False)
    rim = np.c_[CUP_XY[0] + 0.03 * np.cos(a), CUP_XY[1] + 0.03 * np.sin(a), np.full(60, 0.08)]
    chosen = None
    for dx, dy in ((0.075, 0.05), (0.075, 0.075), (0.1, 0.075), (0.05, 0.075), (0.1, 0.1)):
        tip = (world.T_base_table @ np.r_[CUP_XY[0] + dx, CUP_XY[1] + dy, 0.20, 1.0])[:3]
        r = kin.ik(tip, down=True, check_table=False, restarts=3, max_iters=150)
        if not r.ok:
            continue
        for j6 in np.linspace(-1.9, 1.9, 39):     # roll the camera so the cup swings into a lower corner
            q = r.q.copy()
            q[5] = j6
            uv = _view(world, q).project_table(rim)
            w, h = cam.size
            inside = np.isfinite(uv).all(1) & (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
            hidden = np.zeros(len(uv), bool)
            hidden[inside] = mask[uv[inside, 1].astype(int), uv[inside, 0].astype(int)] == 0
            lost = hidden | ~inside
            if hidden.mean() >= 0.1 and lost.mean() <= 0.4:
                chosen = (q, hidden.mean())
                break
        if chosen:
            break
    assert chosen is not None
    q, frac = chosen
    img = world.render(cam, kin.fk(q) @ synth.TRUE_T_TIP_CAM, seed=12)
    d = locate.CupFinder().find(img, _view(world, q))
    assert d is not None
    err = np.hypot(d.xyz_table[0] - CUP_XY[0], d.xyz_table[1] - CUP_XY[1]) * 1000
    print(f"\ncup with {frac:.0%} of the rim behind a jaw: error {err:.2f} mm, coverage {d.extra['coverage']}")
    assert err < 3.0


def test_no_cup_no_false_positive_on_the_board():
    world = synth.World()   # just the calibration sheet: lots of edges, no circle
    q = kin.ik((0.30, 0.0, 0.17), down=True, check_table=False).q
    img = world.render(synth.TRUE_CAMERA, kin.fk(q) @ synth.TRUE_T_TIP_CAM, seed=13)
    assert locate.CupFinder().find(img, _view(world, q)) is None


def test_tagged_box_at_25cm(world):
    q = _look_at(world, [BOX_XY[0], BOX_XY[1], 0.04], 0.25, 30.0, 0.0)   # camera 25 cm from the tag
    img = world.render(synth.TRUE_CAMERA, kin.fk(q) @ synth.TRUE_T_TIP_CAM, seed=14)
    d = locate.TagFinder().find(img, _view(world, q))
    assert d is not None and d.id == 2
    err = np.hypot(d.xyz_table[0] - BOX_XY[0], d.xyz_table[1] - BOX_XY[1]) * 1000
    yaw_err = np.degrees(abs((d.yaw_table - 0.4 + np.pi / 2) % np.pi - np.pi / 2))
    print(f"\ntag at 25 cm: error {err:.2f} mm, top height {d.height * 1000:.1f} mm, yaw error {yaw_err:.2f} deg")
    assert err < 3.0 and abs(d.height - 0.04) < 0.004 and yaw_err < 2.0


def test_colour_blob_finder():
    world = synth.World(boxes=[synth.Box((0.0, -0.10), size=(0.05, 0.05, 0.04), yaw=0.3)])   # plain green box
    tip = (world.T_base_table @ np.r_[0.0, -0.10, 0.17, 1.0])[:3]
    q = kin.ik(tip, down=True, check_table=False).q
    img = world.render(synth.TRUE_CAMERA, kin.fk(q) @ synth.TRUE_T_TIP_CAM, seed=15)
    d = locate.ColorBlobFinder(plane_height=0.04).find(img, _view(world, q))
    assert d is not None
    err = np.hypot(d.xyz_table[0], d.xyz_table[1] + 0.10) * 1000
    print(f"\ngreen box top by colour: error {err:.2f} mm, size {d.extra['size_m']}")
    assert err < 3.0


def test_claude_seam(world):
    q = _look_at(world, [CUP_XY[0], CUP_XY[1], 0.0], 0.22, 0.0)
    img = world.render(synth.TRUE_CAMERA, kin.fk(q) @ synth.TRUE_T_TIP_CAM, seed=16)
    view = _view(world, q)
    assert isinstance(locate.CupFinder(), locate.ObjectFinder)
    with pytest.raises(NotImplementedError):
        locate.ClaudeVisionFinder().find(img, view)
    uv = view.project_table([[CUP_XY[0], CUP_XY[1], 0.08]])[0]
    f = locate.ClaudeVisionFinder(height=0.08, ask=lambda jpg, prompt: {"u": uv[0] + 2, "v": uv[1] - 1, "sigma_px": 6})
    d = f.find(img, view)
    assert d is not None and np.hypot(d.xyz_table[0] - CUP_XY[0], d.xyz_table[1] - CUP_XY[1]) < 0.003
