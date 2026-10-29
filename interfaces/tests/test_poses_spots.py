"""Pose recorder -> playable (interfaces.timeline.poses) and grab spots (interfaces.grasp.spots).

    .venv/bin/python -m pytest -q interfaces/tests/test_poses_spots.py
"""
from __future__ import annotations

import itertools
import json

import numpy as np
import pytest

from armctl import envelope as envm
from armctl import kin
from armctl import playable as pl
from interfaces.grasp import spots
from interfaces.timeline import coach
from interfaces.timeline.poses import sequence_playable

READY = list(envm.POSES["ready"])
POSES = [{"q": [0.4, 0.9, 1.0, 0.0, 0.0, 0.0], "grip": 1.0},
         {"q": [-0.8, 0.7, 2.1, 0.3, 0.0, 0.5], "grip": 0.0},
         {"q": [0.0, 1.2, 1.5, -0.5, 0.3, -0.4]}]


# ---------------------------------------------------------------- poses
@pytest.mark.parametrize("style", ["lively", "gentle"])
@pytest.mark.parametrize("caps", ["dance", "default"])
def test_sequence_is_valid_and_visits_every_pose(style, caps):
    p, notes = sequence_playable(POSES, name="My Sequence!", style=style, caps=caps)
    assert pl.validate(p)["ok"], pl.report(pl.validate(p))
    assert coach.explain(p)["ok"]
    assert p["name"] == "my-sequence" and p["start"] == "ready" and p["caps"] == caps
    kfs = p["keyframes"]
    assert kfs[0]["q"] == pytest.approx(READY) and kfs[-1]["q"] == pytest.approx(READY)
    assert pl.validate(p)["transition"] is None          # starts exactly at ready
    visited = [k["q"] for k in kfs]
    for pose in POSES:
        assert any(np.allclose(q, pose["q"], atol=1e-4) for q in visited)
    holds = [k for k in kfs if k.get("ease") == "hold"]
    assert len(holds) == len(POSES)                       # one hold at each pose
    for a, b in zip(kfs, kfs[1:]):
        assert b["t"] - a["t"] >= 0.3 - 1e-9
    assert notes[0].startswith("3 poses") and f"{kfs[-1]['t']:.1f} s" in notes[0]
    assert any("pose 1 → 2 takes at least" in n and "rad/s" in n for n in notes)


def test_grip_carries_over_and_respects_grip_cap():
    p, _ = sequence_playable([{"q": [0.4, 0.9, 1.0, 0, 0, 0], "grip": 1.0},
                              {"q": [0.4, 0.9, 1.0, 0, 0, 0.01], "grip": 0.0},   # grip-limited move
                              {"q": [0.0, 0.9, 1.0, 0, 0, 0]}])
    r = pl.validate(p)
    assert r["ok"] and r["max_grip_vel"] <= pl.envelope().grip_vel_cap
    assert p["keyframes"][-1]["grip"] == 0.0               # pose 3 keeps pose 2's grip


def test_gentle_is_slower_than_lively():
    pl_, _ = sequence_playable(POSES, style="lively")
    pg, _ = sequence_playable(POSES, style="gentle")
    assert pg["keyframes"][-1]["t"] > pl_["keyframes"][-1]["t"] * 1.3


def test_refuses_pose_outside_soft_box_naming_it():
    bad = [POSES[0], {"q": [0.0, 0.5, 0.8, 1.55, 0.0, 0.0]}]
    with pytest.raises(ValueError, match=r"pose 2 .*j4 .*limit 1\.47"):
        sequence_playable(bad)


def test_refuses_pose_near_table_naming_it():
    low = [0.0, 2.6, 0.3, 0.0, 0.0, 0.0]
    assert not kin.clear_of_table(low)
    with pytest.raises(ValueError, match=r"pose 3 is too close to the table"):
        sequence_playable([POSES[0], POSES[1], {"q": low}])


def test_refuses_bad_input():
    with pytest.raises(ValueError, match="no poses"):
        sequence_playable([])
    with pytest.raises(ValueError, match="pose 1 is not a list of 6"):
        sequence_playable([{"q": [0, 1]}])
    with pytest.raises(ValueError, match="style"):
        sequence_playable(POSES, style="wild")
    with pytest.raises(ValueError, match="start_from"):
        sequence_playable(POSES, start_from="home")


def test_60_s_limit():
    many = POSES * 6
    with pytest.raises(ValueError, match=r"at most 60 s.*Delete some poses or play it 'lively'"):
        sequence_playable(many, style="gentle")
    with pytest.raises(ValueError, match=r"at most 60 s at a time\. Delete some poses\.$"):
        sequence_playable(POSES * 12, style="lively")
    p, _ = sequence_playable(POSES * 3, style="lively")
    assert pl.validate(p)["ok"] and p["keyframes"][-1]["t"] <= 60


def test_single_pose_and_duplicates():
    p, notes = sequence_playable([{"q": READY}, {"q": READY}])
    assert pl.validate(p)["ok"] and notes[0].startswith("2 poses")
    for a, b in zip(p["keyframes"], p["keyframes"][1:]):
        assert b["t"] - a["t"] >= 0.3 - 1e-9


# ---------------------------------------------------------------- spots
def test_list_spots():
    s = spots.list_spots()
    assert [x["name"] for x in s] == ["A", "B", "C"]
    for x in s:
        assert x["title"] and len(x["xyz_table"]) == 3 and x["xyz_table"][2] == 0.0
    # wherever the spots are on today's table, every pair must plan (the file holds real positions now)
    from interfaces.grasp import place_spots
    assert set(place_spots.check()["pairs"].values()) == {"ok"}
    objs = {o["name"]: o for o in spots.list_objects()}
    assert objs["block"]["height"] == 0.04 and objs["block"]["width"] == 0.04
    for o in objs.values():
        assert o["width"] + 0.02 <= 0.094


@pytest.mark.parametrize("obj", ["block", "cup"])
def test_every_pair_plans_and_validates(obj):
    for a, b in itertools.permutations([x["name"] for x in spots.list_spots()], 2):
        p, info = spots.grab_playable(a, b, obj=obj)
        assert pl.validate(p)["ok"], (a, b, pl.report(pl.validate(p)))
        assert coach.explain(p)["ok"]
        assert info["caps"] in ("dance", "default") and info["duration_s"] <= 60
        assert info["src"] == a and info["dst"] == b and p["caps"] == info["caps"]


def test_grab_refusals():
    with pytest.raises(ValueError, match="unknown spot 'Z'"):
        spots.grab_playable("A", "Z")
    with pytest.raises(ValueError, match="two different spots"):
        spots.grab_playable("A", "A")
    with pytest.raises(ValueError, match="unknown object 'piano'"):
        spots.grab_playable("A", "B", obj="piano")


def test_spots_from_another_workcell_file(tmp_path):
    raw = json.loads(spots.WORKCELL.read_text())
    raw["spots"] = {"far": {"title": "too far", "xyz": [0.5, 0.0, 0.0]}, "A": raw["spots"]["A"]}
    raw["objects"]["wide"] = {"height": 0.04, "width": float(raw.get("max_width", 0.094))}   # no room to spare
    f = tmp_path / "wc.json"
    f.write_text(json.dumps(raw))
    with pytest.raises(spots.Unreachable):
        spots.grab_playable("A", "far", path=f)
    with pytest.raises(ValueError, match="too wide"):
        spots.grab_playable("A", "far", obj="wide", path=f)


def test_refuses_a_move_that_dips_between_two_clear_poses():
    a = [1.02, 1.03, 0.22, -1.54, 0.92, 1.65]          # clear of the table itself...
    assert kin.clear_of_table(a)
    with pytest.raises(ValueError, match=r"move from ready to pose 1 would pass too close to the table"):
        sequence_playable([{"q": a}])                   # ...but the straight path from ready dips
