"""Closed-loop look-and-grasp (interfaces/wristcam/engine.py, look_and_grasp.py) against the synthetic
world and a fake arm whose gripper stalls on the cup. No daemon, no camera, no arm.

    .venv/bin/python -m pytest -q interfaces/tests/test_wristcam_engine.py -s
"""
from __future__ import annotations

import json

import cv2
import numpy as np

from interfaces.wristcam.camera import finger_dir_for_cam_axis
import pytest

from armctl import envelope as envm
from armctl import kin
from armctl import playable as pl
from interfaces.grasp.grasp import Workcell
from interfaces.wristcam import engine, locate, look_and_grasp as lg, synth
from interfaces.wristcam.camera import CameraModel


def test_closed_loop_success(tmp_path):
    res = lg.simulate(tmp_path)
    print("\nsimulate:", {k: res[k] for k in ("ok", "trace", "estimate_table", "observations", "cup_final_table")})
    assert res["ok"], res["reason"]
    assert res["plan_ok"] and not res["collisions"]
    assert np.hypot(res["estimate_table"][0] + 0.01, res["estimate_table"][1] + 0.16) < 0.002
    assert np.hypot(res["cup_final_table"][0] - 0.08, res["cup_final_table"][1] + 0.05) < 0.005
    events = [json.loads(l) for l in (tmp_path / "run.jsonl").read_text().splitlines()]
    decisions = [e["decision"] for e in events if e["event"] == "decision"]
    assert "converged" in decisions and "holding" in decisions and "verified" in decisions
    assert decisions.count("closer") >= 1      # it went closer before converging
    # every motion was a checked move_joints within the soft box
    for e in events:
        if e["event"] == "move":
            assert not envm.DEFAULT.check_point(e["q_to"]) and e["speed"] <= 0.3
    assert (tmp_path / "look.jpg").exists() and list((tmp_path / "frames").glob("*.jpg"))
    pfile = next(tmp_path.glob("*.playable.json"))
    assert pl.validate(json.loads(pfile.read_text()))["ok"]


def test_miss_then_retry(tmp_path):
    res = lg.simulate(tmp_path, miss_first=True, seed=1)
    assert res["ok"], res["reason"]
    t = res["trace"]
    assert "miss" in t and t.index("miss") < t.index("lift")
    assert t.count("close") == 2


def test_gives_up_when_nothing_there(tmp_path):
    world = synth.World(board=False)   # the real procedure removes the sheet after calibration
    arm = synth.SimArm(world)
    wc = Workcell(T_base_table=world.T_base_table.tolist(), calibrated=True)
    cfg = engine.GraspConfig(search_offsets=((0, 0), (0.05, 0)))
    res = engine.GraspEngine(arm, locate.CupFinder(), synth.TRUE_CAMERA, synth.TRUE_T_TIP_CAM, wc, cfg,
                             log_path=tmp_path / "run.jsonl").run()
    assert not res["ok"] and "not found" in res["reason"]
    assert res["trace"][-1] == "failed"


# Was a strict xfail until 29 Oct (a ~6 cm circle on the sheet read as a cup); search views now only
# look near their own spot (Hint), which also keeps the board's circle out.
def test_gives_up_with_only_the_board_there(tmp_path):
    world = synth.World()
    wc = Workcell(T_base_table=world.T_base_table.tolist(), calibrated=True)
    cfg = engine.GraspConfig(search_offsets=((0, 0), (0.05, 0)))
    res = engine.GraspEngine(synth.SimArm(world), locate.CupFinder(), synth.TRUE_CAMERA, synth.TRUE_T_TIP_CAM, wc, cfg,
                             log_path=tmp_path / "run.jsonl").run()
    assert not res["ok"] and "not found" in res["reason"]


def test_check_step_refuses_the_table():
    q_ready = np.array(envm.POSES["ready"])
    q_low = kin.ik((0.30, 0.0, 0.02), down=True, check_table=False).q
    assert engine.check_step(q_ready, q_low, kin.MIN_TIP_Z) != ""
    assert engine.check_step(q_ready, q_low, 0.01) == ""
    bad = q_ready.copy()
    bad[2] = 3.2
    assert "soft" in engine.check_step(q_ready, bad, kin.MIN_TIP_Z) or engine.check_step(q_ready, bad, kin.MIN_TIP_Z)


def test_dry_run_cli_end_to_end(tmp_path):
    """The real-arm dry run on a saved frame: detection -> validated pick_place playable + JSON + image."""
    wc = Workcell.load()
    world = synth.World(T_base_table=np.array(wc.T_base_table, float), cups=[synth.Cup((0.0, -0.15))])
    # camera-down look ~0.2 m above the cup (the real camera sits ~60 deg off the fingers)
    tip = (np.array(wc.T_base_table, float) @ np.r_[0.0, -0.15, 0.22, 1.0])[:3]
    q = kin.ik(tip, approach_dir=finger_dir_for_cam_axis([0, 0, -1], tip), check_table=False, restarts=3).q.copy()
    q[5] = 0.0
    img = world.render(synth.TRUE_CAMERA, kin.fk(q) @ synth.TRUE_T_TIP_CAM, seed=3)
    cv2.imwrite(str(tmp_path / "look.png"), img)
    synth.TRUE_CAMERA.save(tmp_path / "cam.json")
    (tmp_path / "he.json").write_text(json.dumps({"T_tip_cam": synth.TRUE_T_TIP_CAM.tolist()}))
    out = tmp_path / "out"
    rc = lg.main(["--image", str(tmp_path / "look.png"), "--q=" + ",".join(str(x) for x in q), "--calib",
                  str(tmp_path / "cam.json"), "--handeye", str(tmp_path / "he.json"), "--out", str(out)])
    assert rc == 0
    plan = json.loads((out / "plan.json").read_text())
    d = plan["detection"]
    assert d and np.hypot(d["xyz_table"][0], d["xyz_table"][1] + 0.15) < 0.003
    assert plan["validation"]["ok"] and plan["validation"]["coach_ok"]
    assert all(lp["q"] is not None and lp["check"] == "ok" for lp in plan["look_poses"])
    p = json.loads(next(out.glob("*.playable.json")).read_text())
    assert pl.validate(p)["ok"] and p["min_tip_z"] == round(wc.floor_base(), 4)   # base frame: min_tip_z above the table
    assert (out / "look.jpg").exists()


def test_execute_refuses_without_calibration(tmp_path):
    with pytest.raises(SystemExit):
        lg.main(["--execute", "--calib", str(tmp_path / "none.json"), "--handeye", str(tmp_path / "none.json"),
                 "--out", str(tmp_path)])


def test_camera_model_roundtrip(tmp_path):
    p = synth.TRUE_CAMERA.save(tmp_path / "c.json")
    c = CameraModel.load(p)
    uv = np.array([[100.0, 80.0], [640, 360], [1200, 700]])
    assert np.allclose(c.project(c.rays(uv)), uv, atol=1e-3)
