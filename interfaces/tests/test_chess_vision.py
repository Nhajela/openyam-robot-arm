"""Board vision v1 (interfaces/chess_vision) against synthetic renders: no camera, no arm.

    .venv/bin/python -m pytest -q interfaces/tests/test_chess_vision.py      (-s prints the accuracy table)

The renders (interfaces/chess_vision/synth.py) have an overhead camera and two oblique side
cameras, shaded cylinder pieces that lean into their neighbours, markers placed a few mm off and
rotated, lighting changes (gain 0.65-1.25, gradients, tint), blur, noise and JPEG.
"""
from __future__ import annotations

import random

import chess
import numpy as np
import pytest

from interfaces.chess_vision import calibrate as cal_cli
from interfaces.chess_vision import watch as watch_cli
from interfaces.chess_vision.board_geom import BoardLocator, BoardSpec, Canon, SQUARES, apply_h
from interfaces.chess_vision.camera import CameraBoardSensor, PosedFrame, calibrate_camera, load_calibration, save_calibration
from interfaces.chess_vision.sensor import BoardSensor, Snapshot
from interfaces.chess_vision.synth import Cam, FrameBox, Lighting, Scene, render, standard_cameras

SCENE = Scene(marker_offset_mm={"h1": (3.0, -2.0), "a8": (-2.0, 3.0)}, marker_rot={"h8": 1, "a1": 2, "a8": 3})
SPEC = SCENE.spec
CAMS = standard_cameras(SPEC)


@pytest.fixture(scope="module")
def calib(tmp_path_factory):
    d = tmp_path_factory.mktemp("chess_calib")
    canon = Canon(SPEC)
    cams = {}
    for name, cam in CAMS.items():
        c = calibrate_camera(name, [render(SCENE, cam, None, seed=1)], [render(SCENE, cam, chess.Board(), seed=2)],
                             SPEC, canon)
        cams[name] = c
    save_calibration(d, SPEC, canon, cams)
    return d, cams


def make_sensor(calib_dir, names):
    boxes = {n: FrameBox(n) for n in names}
    return CameraBoardSensor([(n, boxes[n]) for n in names], calib_dir), boxes


def show(sensor, boxes, board, lighting=None, seed=0, **kw) -> Snapshot:
    for n, box in boxes.items():
        box.frame = render(SCENE, CAMS[n], board, lighting=lighting, seed=seed, **kw)
    return sensor.snapshot()


def arrays(snap):
    return np.array([snap.occupancy[s] for s in SQUARES]), np.array([snap.colour[s] for s in SQUARES])


def truth(board):
    occ = np.array([board.piece_at(i) is not None for i in range(64)])
    white = np.array([board.piece_at(i) is not None and board.piece_at(i).color == chess.WHITE for i in range(64)])
    return occ, white


def random_board(seed) -> chess.Board:
    rng = random.Random(seed)
    b = chess.Board()
    for _ in range(rng.randint(6, 70)):
        ms = list(b.legal_moves)
        if not ms:
            break
        caps = [m for m in ms if b.is_capture(m)]
        b.push(rng.choice(caps) if caps and rng.random() < 0.6 else rng.choice(ms))
    return b


# ---------------------------------------------------------------------- geometry
def test_marker_ids_do_not_clash_with_vision_tags():
    import json
    from pathlib import Path
    tags = json.loads((Path(__file__).resolve().parents[2] / "vision/targets/tags.json").read_text())
    assert tags["dict"] == SPEC.dict_name
    assert not set(SPEC.ids.values()) & {int(k) for k in tags["roles"]}
    assert SPEC.ids == {"a1": 10, "h1": 11, "h8": 12, "a8": 13}


def test_localisation_error_mm(calib):
    """Grid points through the estimated homography vs the render's true geometry."""
    _, cams = calib
    grid = SPEC.grid_points()
    for name, cam in CAMS.items():
        img = render(SCENE, cam, None, seed=7, lighting=Lighting(0.8, (0.2, -0.2)))
        img_pts = apply_h(cam.ground_H(), grid)
        nominal = BoardLocator(SPEC).locate(img, 0.0)
        refined = BoardLocator(SPEC, fix=cams[name]["fix"]).locate(img, 0.0)
        assert refined.ok and refined.n_markers == 4
        e_nom = np.linalg.norm(apply_h(nominal.H, img_pts) - grid, axis=1)
        e_ref = np.linalg.norm(apply_h(refined.H, img_pts) - grid, axis=1)
        print(f"\n{name}: localisation error nominal markers mean {e_nom.mean():.2f} max {e_nom.max():.2f} mm; "
              f"after grid refinement mean {e_ref.mean():.2f} max {e_ref.max():.2f} mm")
        assert e_ref.mean() < 0.6 and e_ref.max() < 1.5            # square = 35 mm
        assert e_ref.mean() < e_nom.mean()


def test_orientation_from_ids_camera_behind_black():
    """A camera on black's side sees the board upside down; a1 must still map to a1."""
    c = np.array([SPEC.size_mm / 2, SPEC.size_mm / 2, 0])
    cam = Cam("black", c + [-30, 470, 520], c, f=1000)
    img = render(SCENE, cam, chess.Board(), seed=3)
    view = BoardLocator(SPEC).locate(img, 0.0)
    assert view.ok
    for sq in ("a1", "h1", "e4", "h8"):
        p_img = apply_h(cam.ground_H(), [SPEC.square_center(sq)])
        assert np.linalg.norm(apply_h(view.H, p_img)[0] - SPEC.square_center(sq)) < 3.0
    assert view.cam_pos[1] > SPEC.size_mm   # it knows it is on black's side


def test_hand_over_marker_reuses_homography_with_staleness():
    cam = CAMS["side"]
    loc = BoardLocator(SPEC, staleness_s=30.0)
    v0 = loc.locate(render(SCENE, cam, chess.Board(), seed=4), t=0.0)
    hand = render(SCENE, cam, chess.Board(), seed=5, hand_over="h1")
    v1 = loc.locate(hand, t=5.0)
    assert v1.ok and v1.n_markers == 3 and v1.age_s == pytest.approx(5.0) and np.allclose(v1.H, v0.H)
    assert v1.quality < v0.quality
    v2 = loc.locate(hand, t=100.0)   # stale: falls back to a fresh 3-marker homography
    assert v2.ok and v2.age_s == 0.0 and "fresh" in v2.reason
    two = render(SCENE, cam, chess.Board(), seed=6, hand_over=["h1", "a1"])
    loc2 = BoardLocator(SPEC, staleness_s=30.0)
    loc2.locate(render(SCENE, cam, chess.Board(), seed=4), t=0.0)
    assert loc2.locate(two, t=10.0).ok                 # 2 markers, recent, consistent: reused
    assert not loc2.locate(two, t=31.0).ok             # too old
    # the camera was bumped: the visible markers disagree with the old homography
    bumped = Cam("side", cam.C + [60, 0, 0], cam.target + [40, 0, 0], cam.f)
    v3 = loc2.locate(render(SCENE, bumped, chess.Board(), seed=8, hand_over=["h1", "a1"]), t=11.0)
    assert not v3.ok and "moved" in v3.reason


# ---------------------------------------------------------------------- occupancy, colour, fusion
def accuracy(sensor, boxes, n_pos=10, seed0=100, **kw):
    rng = np.random.default_rng(seed0)
    occ_ok = col_ok = n_sq = n_occ = 0
    for k in range(n_pos):
        b = random_board(seed0 + k)
        snap = show(sensor, boxes, b, lighting=Lighting.random(rng), seed=seed0 + k, **kw)
        o, c = arrays(snap)
        to, tw = truth(b)
        occ_ok += ((o > 0.5) == to).sum()
        col_ok += ((c > 0.5) == tw)[to].sum()
        n_sq += 64
        n_occ += to.sum()
    return occ_ok / n_sq, col_ok / n_occ


@pytest.fixture(scope="module")
def results(calib):
    d, _ = calib
    out = {}
    for key, names in {"fused top+side+left": ["top", "side", "left"], "top": ["top"], "side": ["side"],
                       "left": ["left"], "fused side+left": ["side", "left"]}.items():
        s, boxes = make_sensor(d, names)
        out[key] = accuracy(s, boxes)
    print("\nsynthetic accuracy over 10 random positions with lighting changes (occupancy, colour):")
    for k, (o, c) in out.items():
        print(f"  {k:22s} occupancy {o:.2%}  colour {c:.2%}")
    return out


def test_occupancy_and_colour_accuracy(results):
    o, c = results["fused top+side+left"]
    assert o > 0.99 and c > 0.99
    o, c = results["top"]
    assert o > 0.99 and c > 0.98
    for side in ("side", "left"):          # oblique alone: tall pieces hide what is behind them
        assert results[side][0] > 0.8
    assert results["fused side+left"][0] >= max(results["side"][0], results["left"][0]) - 0.01


def test_snapshot_shape_and_frames(calib):
    d, _ = calib
    s, boxes = make_sensor(d, ["top", "side"])
    sensor_api: BoardSensor = s          # structural: the game only needs snapshot()
    assert callable(sensor_api.snapshot)
    snap = show(s, boxes, chess.Board(), seed=11)
    assert set(snap.occupancy) == set(SQUARES) == set(snap.colour)
    assert all(0.0 <= v <= 1.0 for v in snap.occupancy.values())
    assert set(snap.frames) == {"top", "side"}
    assert all(f[:2] == b"\xff\xd8" for f in snap.frames.values())


def test_camera_without_markers_is_ignored(calib):
    d, _ = calib
    s, boxes = make_sensor(d, ["top", "side"])
    boxes["top"].frame = render(SCENE, CAMS["top"], chess.Board(), seed=12)
    boxes["side"].frame = np.full((720, 1280, 3), 90, np.uint8)       # covered lens
    o, c = arrays(s.snapshot())
    to, tw = truth(chess.Board())
    assert ((o > 0.5) == to).all() and ((c > 0.5) == tw)[to].all()
    assert not s.last["cameras"]["side"]["view"].ok


def test_occluded_marker_during_snapshot(calib):
    d, _ = calib
    s, boxes = make_sensor(d, ["top"])
    show(s, boxes, chess.Board(), seed=13)
    b = chess.Board()
    b.push_san("e4")
    snap = show(s, boxes, b, seed=14, hand_over="h8")
    assert "reused" in s.last["cameras"]["top"]["view"].reason
    o, _ = arrays(snap)
    assert ((o > 0.5) == truth(b)[0]).all()


def test_posed_frame_without_markers_uses_pose(calib):
    """Arm-scan seam: a frame with a known pose (here the render's true geometry) still works when
    every corner marker is hidden."""
    d, _ = calib
    s, boxes = make_sensor(d, ["top"])
    cam = CAMS["top"]
    b = chess.Board()
    b.push_san("Nf3")
    img = render(SCENE, cam, b, seed=31, hand_over=["a1", "h1", "h8", "a8"])
    boxes["top"].frame = PosedFrame(img, cam_pos_mm=cam.C, H_img2board=cam.true_img2board())
    o, c = arrays(s.snapshot())
    to, tw = truth(b)
    assert s.last["cameras"]["top"]["view"].reason == "homography from the camera pose"
    assert ((o > 0.5) == to).all() and ((c > 0.5) == tw)[to].all()


# ---------------------------------------------------------------------- end to end: which squares changed
def changed(before: Snapshot, after: Snapshot):
    ob, cb = arrays(before)
    oa, ca = arrays(after)
    occ_change = {SQUARES[i] for i in range(64) if (ob[i] > 0.5) != (oa[i] > 0.5)}
    colour_flip = {SQUARES[i] for i in range(64) if ob[i] > 0.5 and oa[i] > 0.5 and (cb[i] > 0.5) != (ca[i] > 0.5)}
    return occ_change, colour_flip


@pytest.mark.parametrize("names", [["top", "side", "left"], ["top"]])
def test_e2e_moves_change_the_right_squares(calib, names):
    d, _ = calib
    s, boxes = make_sensor(d, names)
    rng = np.random.default_rng(5)
    cases = [
        ([], "e2e4", {"e2", "e4"}, set()),
        (["e4", "d5"], "exd5", {"e4"}, {"d5"}),                                     # capture: d5 changes colour
        (["e4", "e5", "Nf3", "Nc6", "Bc4", "Bc5"], "O-O", {"e1", "f1", "g1", "h1"}, set()),
        (["e4", "d5", "e5", "f5"], "exf6", {"e5", "f5", "f6"}, set()),               # en passant
    ]
    for k, (pre, mv, want_occ, want_flip) in enumerate(cases):
        b = chess.Board()
        for m in pre:
            b.push_san(m)
        before = show(s, boxes, b, lighting=Lighting.random(rng), seed=200 + k)
        b.push_san(mv)
        after = show(s, boxes, b, lighting=Lighting.random(rng), seed=300 + k)
        occ_change, flip = changed(before, after)
        assert occ_change == want_occ, (mv, occ_change)
        assert flip == want_flip, (mv, flip)


# ---------------------------------------------------------------------- CLIs and file round trip
def test_calibrate_and_watch_cli_on_image_folders(tmp_path):
    import cv2
    srcs, watch_srcs = [], []
    for name in ("top", "side"):
        f = tmp_path / "frames" / name
        f.mkdir(parents=True)
        cv2.imwrite(str(f / "0_empty.png"), render(SCENE, CAMS[name], None, seed=21))
        cv2.imwrite(str(f / "1_start.png"), render(SCENE, CAMS[name], chess.Board(), seed=22))
        b = chess.Board()
        b.push_san("d4")
        live = tmp_path / "live" / name
        live.mkdir(parents=True)
        cv2.imwrite(str(live / "d4.png"), render(SCENE, CAMS[name], b, seed=23))
        srcs += ["--source", f"{name}={f}"]
        watch_srcs += ["--source", f"{name}={live}"]
    out = tmp_path / "calib"
    jp = cal_cli.main([*srcs, "--out", str(out), "--yes"])
    assert jp.exists() and (out / "top_empty.png").exists() and (out / "side_check.jpg").exists()
    spec, canon, cams = load_calibration(out)
    assert spec == SPEC and set(cams) == {"top", "side"}
    w = tmp_path / "watch"
    snap = watch_cli.main([*watch_srcs, "--calib", str(out), "--out", str(w), "--count", "1"])
    assert snap.occupancy["d4"] > 0.5 and snap.occupancy["d2"] < 0.5
    assert (w / "latest_board.png").exists() and (w / "latest_top_warp.jpg").exists()
