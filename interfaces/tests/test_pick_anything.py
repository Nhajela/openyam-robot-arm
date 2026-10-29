"""Pick up anything you name (interfaces/wristcam/detect.py + target.resolve_prompt + DetectorFinder):
a red cup, a green box and a blue box on the synthetic table; a ground-truth detector stands in for
YOLO-E (which runs on the lab laptop's GPU). Name one, the same geometry as a tap turns its mask into
a footprint, height and grasp, and the closed loop re-detects it by name at every look.
No daemon, no camera, no arm, no model weights.

    .venv/bin/python -m pytest -q interfaces/tests/test_pick_anything.py -s
"""
from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from interfaces.grasp.grasp import Workcell
from interfaces.wristcam import detect as D
from interfaces.wristcam import synth
from interfaces.wristcam import target as T

CUP_XY = (-0.02, -0.20)
GREEN_XY, GREEN_SIZE, GREEN_YAW = (0.06, -0.08), (0.05, 0.035, 0.04), 0.5
BLUE_XY, BLUE_SIZE = (-0.05, -0.09), (0.04, 0.03, 0.05)
LOOK_XY = (0.01, -0.13)
CAM, T_TC = synth.TRUE_CAMERA, synth.TRUE_T_TIP_CAM


def make_world():
    cup = synth.Cup(CUP_XY)
    green = synth.Box(GREEN_XY, size=GREEN_SIZE, yaw=GREEN_YAW)
    blue = synth.Box(BLUE_XY, size=BLUE_SIZE, yaw=-0.3, color=(190, 90, 40))
    cup.label, green.label, blue.label = "red cup", "green box", "blue box"
    return synth.World(board=False, cups=[cup], boxes=[green, blue])


@pytest.fixture(scope="module")
def look():
    world = make_world()
    arm = synth.SimArm(world, ss=1)
    wc = Workcell(T_base_table=world.T_base_table.tolist(), calibrated=True)
    q, _ = T.look_pose(wc, CAM, T_TC, arm.q(), LOOK_XY, 0.25)
    arm.move_joints(q)
    lf = T.LookFrame(arm.frame(), [float(x) for x in q])
    return world, wc, lf, lf.view(CAM, T_TC, world.T_base_table)


def _mm(xy, true):
    return float(np.hypot(xy[0] - true[0], xy[1] - true[1]) * 1000)


def test_name_it_and_get_its_footprint(look):
    world, wc, lf, view = look
    det = D.SynthDetector(world)
    t, cands = T.resolve_prompt(lf.image, view, "green box", det, wc)
    print("\ngreen box:", t.label, np.round(t.xy_table, 4), round(t.height, 4), np.round(t.size, 4), cands)
    assert t.label == "green box" and t.refusal is None and len(cands) == 1
    assert _mm(t.xy_table, GREEN_XY) < 3.0
    assert abs(t.height - GREEN_SIZE[2]) < 0.012
    assert abs(t.width - GREEN_SIZE[1]) < 0.008          # closes across the narrow side
    cup, _ = T.resolve_prompt(lf.image, view, "cup", det, wc)
    assert cup.label == "red cup" and cup.kind == "cup" and _mm(cup.xy_table, CUP_XY) < 4.0


def test_everything_on_the_table_and_not_found(look):
    world, wc, lf, view = look
    det = D.SynthDetector(world)
    best, cands = T.resolve_prompt(lf.image, view, None, det, wc)
    labels = sorted(c["label"] for c in cands)
    print("\neverything:", [(c["label"], c.get("xy"), c.get("refusal") or c.get("error")) for c in cands])
    assert labels == ["blue box", "green box", "red cup"] and all("xy" in c for c in cands)
    with pytest.raises(T.TargetRefused) as e:
        T.resolve_prompt(lf.image, view, "banana", det, wc)
    assert e.value.code == "not_found" and "banana" in str(e.value)


def test_too_wide_named_object_says_why():
    big = synth.Box((0.02, -0.12), size=(0.13, 0.12, 0.05), yaw=0.1)
    big.label = "shoebox"
    world = synth.World(board=False, boxes=[big])
    arm = synth.SimArm(world, ss=1)
    wc = Workcell(T_base_table=world.T_base_table.tolist(), calibrated=True)
    q, _ = T.look_pose(wc, CAM, T_TC, arm.q(), (0.02, -0.12), 0.25)
    arm.move_joints(q)
    lf = T.LookFrame(arm.frame(), [float(x) for x in q])
    t, _ = T.resolve_prompt(lf.image, lf.view(CAM, T_TC, world.T_base_table), "shoebox", D.SynthDetector(world), wc)
    assert t.refusal and "too wide" in t.refusal
    assert T.run_pick(None, t, CAM, T_TC, wc)["ok"] is False


def test_closed_loop_picks_the_named_blue_box(look, tmp_path):
    world0, wc, lf, view = look
    t, _ = T.resolve_prompt(lf.image, view, "blue box", D.SynthDetector(world0), wc)
    world = make_world()
    cup, green, blue = world.cups[0], world.boxes[0], world.boxes[1]
    arm = synth.SimArm(world, ss=1, joint_noise=0.0005)
    arm.move_joints(lf.q)
    finder = T.DetectorFinder(t, D.SynthDetector(world), "blue box")
    lifted = []

    def on_event(state, event, e):
        if e.get("decision") == "verified":
            lifted.append(blue.z0)
    res = T.run_pick(arm, t, CAM, T_TC, wc, finder=finder, on_event=on_event, log_path=tmp_path / "run.jsonl")
    srcs = [h.get("result") for h in finder.history]
    print("\npick blue:", {k: res[k] for k in ("ok", "reason", "trace", "estimate_table", "observations")}, srcs)
    assert res["ok"], res["reason"]
    assert lifted and lifted[0] > 0.05
    assert srcs.count("detector") >= 3                  # it re-detected by name, not by template
    assert _mm(blue.xy, BLUE_XY) < 6 and blue.z0 == 0.0  # put back where it was
    assert tuple(cup.xy) == CUP_XY and _mm(green.xy, GREEN_XY) < 1e-6
    assert not world.collisions


def test_detector_misses_fall_back_to_the_template(look):
    """A detector that sees nothing at all (a bad look): the template keeps the target."""
    world, wc, lf, view = look
    t, _ = T.resolve_prompt(lf.image, view, "green box", D.SynthDetector(world), wc)
    blind = D.SynthDetector(world, miss={"green box"})
    f = T.DetectorFinder(t, blind, "green box")
    det = f.find(lf.image, view)
    assert det is not None and det.extra["source"].startswith("template")
    assert _mm(det.xyz_table, GREEN_XY) < 4.0


def test_ultralytics_results_become_full_frame_masks():
    """The YOLO-E adapter: polygons (already in frame px) -> masks, names, scores, track ids."""
    class Boxes:
        xyxy = np.array([[100, 100, 200, 180], [5, 5, 9, 9]], float)
        conf, cls, id = np.array([0.8, 0.4]), np.array([1, 0]), np.array([7, 8])

        def __len__(self):
            return 2
    poly = np.array([[100, 100], [200, 100], [200, 180], [100, 180]], float)
    res = SimpleNamespace(boxes=Boxes(), masks=SimpleNamespace(xy=[poly, np.array([[5, 5], [9, 5], [9, 9]], float)]),
                          names={0: "speck", 1: "marker"})
    dets = D.from_ultralytics(res, (360, 640, 3))
    assert len(dets) == 1                                   # the 3-px speck is dropped
    d = dets[0]
    assert d.label == "marker" and d.track_id == 7 and d.score == pytest.approx(0.8)
    assert d.mask.shape == (360, 640) and 7500 < d.area < 8500
    assert np.allclose(d.centre, [150, 140], atol=1.0)
