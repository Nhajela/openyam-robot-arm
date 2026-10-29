"""interfaces.grasp.place_spots + Workcell.floor_base: spots and the chess board taught from the
fingertip, and the grasp floor measured from the calibrated table (which may sit below the base).
No daemon: the joint reading is faked.

    .venv/bin/python -m pytest -q interfaces/tests/test_place_spots.py
"""
from __future__ import annotations

import json
import math

import numpy as np
import pytest

from armctl import kin
from interfaces.grasp import place_spots as PS
from interfaces.grasp.grasp import Unreachable, Workcell, pick_place
from interfaces.wristcam.camera import make_T, rot_axis

# like the lab on 29 Oct: table origin 0.41 m out, 3.1 cm below the base, x axis turned ~97 deg
T_BT = make_T(rot_axis([0, 0, 1], -83.0), [0.411, -0.014, -0.031])


def _wc(tmp_path, **kw):
    p = tmp_path / "workcell.json"
    w = Workcell(T_base_table=T_BT.tolist(), calibrated=True, spots={"A": {"title": "A", "xyz": [0.0, 0.12, 0.0]}},
                 objects={"block": {"height": 0.03, "width": 0.03}}, **kw)
    w.save(p)
    return p


def _q_over(wc: Workcell, xy, z=0.06):
    b = wc.to_base([xy[0], xy[1], z])
    r = kin.ik(b, down=True, check_table=False, corr=kin.vision_correction())
    assert r.err_pos < 0.003
    return r.q


def test_floor_is_measured_from_the_calibrated_table(tmp_path, monkeypatch):
    wc = Workcell.load(_wc(tmp_path))
    assert wc.floor_base() == pytest.approx(-0.031 + wc.min_tip_z, abs=1e-6)
    assert Workcell(min_tip_z=0.01).floor_base() == 0.01          # uncalibrated: table assumed at the base
    p = np.linalg.inv(T_BT) @ np.r_[0.30, 0.10, -0.031, 1.0]
    args = (wc, [p[0], p[1], 0.0], [p[0], p[1] + 0.05, 0.0])
    pick_place(*args, height=0.03, width=0.03)                     # a 3 cm block: fine now
    # the old rule (min_tip_z read as a base-frame height) kept the tip 4 cm above this table
    monkeypatch.setattr(Workcell, "floor_base", lambda self: float(self.min_tip_z))
    with pytest.raises(Unreachable, match="too low"):
        pick_place(*args, height=0.03, width=0.03)


def test_teach_a_spot_from_the_fingertip(tmp_path, monkeypatch):
    path = _wc(tmp_path)
    wc = Workcell.load(path)
    b = np.array([0.30, 0.12])
    target = (np.linalg.inv(T_BT) @ np.r_[b, -0.031, 1.0])[:2]
    q = _q_over(wc, target)
    monkeypatch.setattr(PS, "_read_q", lambda url: q)
    res = PS.teach_spot("A", "Spot A (left)", path=path)
    raw = json.loads(path.read_text())
    assert np.allclose(raw["spots"]["A"]["xyz"][:2], target, atol=0.002) and raw["spots"]["A"]["xyz"][2] == 0.0
    assert raw["spots"]["A"]["title"] == "Spot A (left)" and raw["spots"]["A"]["taught"]["tip_above_table_m"] > 0.05
    assert (tmp_path / res["backup"].split("/")[-1]).exists()          # the old file was kept
    assert raw["objects"]["block"]["height"] == 0.03                   # other keys untouched


def test_teach_the_chess_board_from_a1_and_h8(tmp_path, monkeypatch):
    from interfaces.grasp.chess_arm import Board
    import chess

    path = _wc(tmp_path)
    wc = Workcell.load(path)
    monkeypatch.setattr(PS, "BOARD_TEACH", tmp_path / "board_teach.json")
    truth = Board(center=tuple((np.linalg.inv(T_BT) @ np.r_[0.30, 0.02, -0.031, 1.0])[:2]), square=0.03, rank_dir=1.1)
    for corner, sq in (("a1", chess.A1), ("h8", chess.H8)):
        q = _q_over(wc, truth.xy(sq))
        monkeypatch.setattr(PS, "_read_q", lambda url, q=q: q)
        res = PS.teach_board(corner, path=path)
    got = Board(**json.loads(path.read_text())["chess"])
    assert got.square == pytest.approx(0.03, abs=0.0008)
    assert math.cos(got.rank_dir - 1.1) > 0.9995
    for sq in (chess.A1, chess.H8, chess.E4, chess.B7):
        assert np.linalg.norm(got.xy(sq) - truth.xy(sq)) < 0.003
    assert "check the square size" in res["note"]
    out = PS.check(path=path)
    assert out["board"]["reachable_squares"] == 64, out["board"]["unreachable"]
