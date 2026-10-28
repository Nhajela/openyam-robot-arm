"""Tests for the interfaces (no camera, no daemon, no hardware).

    .venv/bin/python -m pytest -q interfaces/tests
"""
from __future__ import annotations

import math

import chess
import numpy as np
import pytest

from armctl import envelope as envm
from armctl import kin
from armctl import playable as pl
from interfaces.grasp import chess_arm
from interfaces.grasp.grasp import Unreachable, Workcell, _finger_dir, fit_table_frame, holding, pick_place
from interfaces.mirror.pose import SyntheticBody
from interfaces.mirror.retarget import Retargeter, arm_angles, clutch_engaged
from interfaces.servo import OneEuro, TargetServo, table_guard, table_margin
from interfaces.timeline import coach, record


# ---------------------------------------------------------------- kinematics
def test_fk_matches_pinocchio_reference():
    assert np.allclose(kin.tip(envm.POSES["home"]), [0.211, 0, 0.174], atol=1e-3)
    assert np.allclose(kin.tip(envm.POSES["ready"]), [0.205, 0, 0.432], atol=1e-3)


def test_joint_semantics():
    # +j1 swings the tip to +y; +j4 tilts the fingers up
    assert kin.tip([0.5, 0.5, 0.8, 0, 0, 0])[1] > 0.05
    assert kin.approach([0, 0.5, 0.8, 0.5, 0, 0])[2] > kin.approach(envm.POSES["ready"])[2]


@pytest.mark.parametrize("xyz", [(0.30, 0.0, 0.10), (0.25, -0.15, 0.15), (0.45, 0.10, 0.10)])
def test_ik_top_down_round_trip(xyz):
    r = kin.ik(xyz, down=True)
    assert r.ok, r.reason
    assert np.linalg.norm(kin.tip(r.q) - xyz) < 0.006
    assert kin.approach(r.q)[2] < -0.99
    assert not envm.DEFAULT.check_point(r.q)


def test_ik_unreachable_is_not_ok():
    r = kin.ik((0.9, 0, 0.1), down=True)
    assert not r.ok and r.reason


# ---------------------------------------------------------------- mirror
def test_synthetic_arm_angles():
    body = SyntheticBody()(0.0)            # right arm horizontal, out to the side
    a = arm_angles(body, "right")
    assert abs(a["elev"] - math.pi / 2) < 0.05
    assert abs(a["yaw"] + math.pi / 2) < 0.05   # person's right = image left = negative yaw
    assert abs(a["opening"] - (math.pi - math.radians(45))) < 0.05


def test_mirror_sign_and_safety():
    rt = Retargeter()
    q, info = rt(SyntheticBody()(0.0))
    assert q is not None
    assert q[0] > 0.5          # hand at image left -> arm swings to its left (+y): a mirror
    assert not envm.DEFAULT.check_point(q)
    assert kin.clear_of_table(q)


def test_every_synthetic_target_is_safe():
    rt, syn = Retargeter(), SyntheticBody()
    for t in np.linspace(0, 8, 81):
        q, _ = rt(syn(t))
        assert q is not None
        assert not envm.DEFAULT.check_point(q)
        assert kin.clear_of_table(q)


def test_hand_mode_tracks_reach():
    rt = Retargeter(mode="hand")
    q, info = rt(SyntheticBody()(0.0))
    assert q is not None
    assert kin.tip(q)[1] > 0.1   # hand out to the image left -> tip to +y


def test_no_person_no_target():
    q, info = Retargeter()(None)
    assert q is None and info["why"] == "no person"
    body = SyntheticBody()(0.0)
    body.visibility[14] = 0.1
    q, info = Retargeter()(body)
    assert q is None


def test_clutch():
    body = SyntheticBody()(0.0)
    assert not clutch_engaged(body, "right")       # left hand hangs
    body.world[15] = body.world[11] + np.array([0, -0.3, 0])
    assert clutch_engaged(body, "right")


# ---------------------------------------------------------------- servo
def test_servo_reaches_target_within_caps():
    v, a = np.full(6, 0.6), np.full(6, 2.0)
    s = TargetServo(v, a)
    q = np.array(envm.POSES["ready"], float)
    tgt = q + np.array([0.4, 0.3, -0.2, 0.1, 0, 0])
    dt = 1 / 30
    prev = np.zeros(6)
    for _ in range(300):
        qd = s.step(q, tgt, dt)
        assert np.all(np.abs(qd) <= v + 1e-9)
        assert np.all(np.abs(qd - prev) <= a * dt + 1e-9)
        prev = qd
        q = q + qd * dt
    assert np.abs(q - tgt).max() < 0.01


def test_one_euro_smooths():
    f = OneEuro(min_cutoff=1.0, beta=0.0)
    rng = np.random.default_rng(1)
    xs = [f(np.array([1.0]) + rng.normal(0, 0.05, 1), t / 30) for t in range(60)]
    assert np.std([x[0] for x in xs[30:]]) < 0.02


def test_table_guard_stops_a_dip():
    q = np.array([0.0, 2.0, 1.6, 0.0, 0.0, 0.0])
    qd = np.array([0, 0.6, -0.6, 0, 0, 0])
    assert table_margin(q) > 0 and table_margin(q + qd * 0.5) < 0   # would dip
    qd2, changed = table_guard(q, qd, horizon=0.5)
    assert changed and table_margin(q + qd2 * 0.5) >= 0


# ---------------------------------------------------------------- timeline
def _fast_wave():
    p = pl.load("wave")
    for k in p["keyframes"]:
        k["t"] = round(k["t"] * 0.4, 3)
    return p


def test_coach_explains_too_fast():
    r = coach.explain(_fast_wave())
    assert not r["ok"]
    bad = [s for s in r["segments"] if not s["ok"]]
    assert bad and "needs at least" in bad[0]["problems"][0]["msg"]
    assert "too fast" in r["summary"][1]


@pytest.mark.parametrize("how", ["stretch", "shrink"])
def test_fit_makes_it_playable(how):
    p = coach.fit(_fast_wave(), how)
    assert pl.validate(p)["ok"]
    if how == "shrink":
        assert p["keyframes"][-1]["t"] == _fast_wave()["keyframes"][-1]["t"]


def test_bounds_band_contains_feasible_targets():
    r = coach.explain(pl.load("wave"))
    for b in r["bounds"][1:]:
        assert all(lo <= hi + 1e-9 for lo, hi in zip(b["lo"], b["hi"]))


def test_need_time_formula():
    # cubic: max(1.5 d / v, sqrt(6 d / a))
    assert coach.need_time(0.3, "cubic", 0.3, 1.0) == pytest.approx(1.5)
    assert coach.max_move(1.5, "cubic", 0.3, 1.0) == pytest.approx(0.3)


def test_recording_to_playable():
    t = np.arange(0, 6, 0.05)
    base = np.array(envm.POSES["ready"], float)
    Q = np.array([base + np.array([0.4 * math.sin(x), 0.2 * (1 - math.cos(x)), 0.2 * math.sin(2 * x), 0, 0, 0])
                  for x in t])
    rec = {"format": "openyam-recording", "rate_hz": 20,
           "samples": [{"t": float(a), "q": q.tolist(), "grip": 0.0} for a, q in zip(t, Q)]}
    p, info = record.to_playable(rec, "test", tol=0.04, caps="dance")
    assert pl.validate(p)["ok"]
    assert info["max_dev_rad"] <= 0.06
    assert 3 <= info["keyframes"] <= 40


# ---------------------------------------------------------------- grasp + chess
def test_pick_place_is_valid_and_points_down():
    wc = Workcell()
    p, info = pick_place(wc, (0.0, 0.0, 0.0), (0.08, -0.12, 0.0), height=0.04, width=0.03, yaw=0.3)
    r = coach.explain(p)
    assert r["ok"], r["summary"]
    grasp_q = p["keyframes"][2]["q"]
    assert kin.approach(grasp_q)[2] < -0.99
    d = _finger_dir(grasp_q)
    want = wc.yaw_to_base(0.3) + math.pi / 2
    err = abs((math.atan2(d[1], d[0]) - want + math.pi / 2) % math.pi - math.pi / 2)
    assert err < 0.05
    grips = [k["grip"] for k in p["keyframes"]]
    assert grips[2] > grips[3]  # closes on the object


def test_unreachable_pick_says_why():
    with pytest.raises(Unreachable, match="can't reach"):
        pick_place(Workcell(), (0.4, 0.0, 0.0), (0.0, 0.0, 0.0), height=0.04, width=0.03)


def test_fit_table_frame_recovers_transform():
    a = 0.4
    T = np.array([[math.cos(a), -math.sin(a), 0, 0.3], [math.sin(a), math.cos(a), 0, 0.05], [0, 0, 1, 0.01], [0, 0, 0, 1]])
    pts = np.array([[0, 0, 0], [0.1, 0, 0], [0, 0.1, 0], [0.1, 0.1, 0]])
    base = (T @ np.c_[pts, np.ones(4)].T).T[:, :3]
    assert np.allclose(fit_table_frame(pts, base), T, atol=1e-9)


def test_holding():
    assert holding(0.30, 0.15) and not holding(0.16, 0.15)


def test_chess_operations():
    b = chess.Board()
    for u in ["e2e4", "e7e5", "g1f3", "b8c6"]:
        b.push_uci(u)
    ops = chess_arm.operations(b, chess.Move.from_uci("f3e5"))
    assert [o["to"] for o in ops][0] == "grave" and len(ops) == 2
    b = chess.Board("r3k2r/8/8/8/8/8/8/R3K2R w KQkq - 0 1")
    ops = chess_arm.operations(b, chess.Move.from_uci("e1g1"))
    assert [chess.square_name(o["from"]) for o in ops] == ["e1", "h1"]
    b = chess.Board("8/8/8/3pP3/8/8/8/4K2k w - d6 0 1")
    ops = chess_arm.operations(b, chess.Move.from_uci("e5d6"))
    assert chess.square_name(ops[0]["from"]) == "d5"


def test_board_geometry():
    bd = chess_arm.Board()
    # white far, arm plays black: rank 1 further from the base than rank 8, h-file on the arm's left (+y)
    assert bd.xy(chess.A1)[0] > bd.xy(chess.A8)[0]
    assert bd.xy(chess.H1)[1] > bd.xy(chess.A1)[1]
    assert np.linalg.norm(bd.xy(chess.A1) - bd.xy(chess.B1)) == pytest.approx(bd.square)
