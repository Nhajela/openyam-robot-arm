"""Envelope + planning tests. Run: .venv/bin/python -m pytest armctl/tests"""
import json

import numpy as np
import pytest

from armctl import envelope as env
from armctl.backend import plan_home, plan_move, twist_to_qd

HOME = env.POSES["home"]
READY = env.POSES["ready"]


def msgs(vs):
    return [v.message for v in vs]


def test_limits_and_soft_box():
    e = env.DEFAULT
    assert env.JOINTS == ("j1", "j2", "j3", "j4", "j5", "j6")
    assert e.soft_lower[1] == pytest.approx(0.1) and e.soft_upper[3] == pytest.approx(1.471)
    custom = env.Envelope(margin={"j1": 0.2, "j2": 0.1, "j3": 0.1, "j4": 0.05, "j5": 0.1, "j6": 0.1})
    assert custom.soft_upper[3] == pytest.approx(1.521)
    with pytest.raises(ValueError):
        env.Envelope(margin=2.0)  # empty box


def test_check_point_home_and_ready_allowed():
    assert env.check_point(HOME) == []
    assert env.check_point([0, -0.005, -0.003, 0, 0, 0]) == []  # measured home reading
    assert env.check_point(READY) == []
    assert "TO BE VERIFIED" in env.POSE_NOTES["ready"]


def test_check_point_reasons():
    v = env.check_point([0, 0.5, 0.8, 1.62, 0, 0])
    assert msgs(v) == ["j4 target 1.620 > soft limit 1.471 (URDF 1.571 - margin 0.10, past URDF limit 1.571)"]
    assert v[0].joint == "j4" and v[0].kind == "soft_upper" and v[0].limit == pytest.approx(1.471)
    v = env.check_point([0, 0.05, 0.8, 0, 0, 0])  # near home but not home
    assert msgs(v) == ["j2 target 0.050 < soft limit 0.100 (URDF 0.000 + margin 0.10)"]
    assert env.check_point([1, 2, 3])[0].kind == "shape"
    assert env.check_point([0, 0.5, np.nan, 0, 0, 0])[0].kind == "not_finite"
    assert env.check_point({"j1": 0, "j2": 0.5, "j3": 0.8, "yam_joint4": 0, "joint5": 0, "j6": 0}) == []


def test_check_path_inside_and_leaving():
    t = plan_move(READY, [0.5, 0.6, 1.0, 0.3, -0.2, 0.4])
    assert env.check_path(t.q, t.dt) == []
    bad = plan_move(READY, [0, 0.5, 0.8, 1.55, 0, 0])
    v = env.check_path(bad.q, bad.dt)
    assert len(v) == 1 and v[0].joint == "j4" and "soft limit 1.471" in v[0].message and "first leaves" in v[0].message


def test_check_path_velocity_and_accel_caps():
    q = np.array([READY, [0.01, 0.5, 0.8, 0, 0, 0]])  # 1 rad/s jump
    v = env.check_path(q, 0.01)
    assert any(x.kind == "velocity" and x.joint == "j1" and "cap 0.300" in x.message for x in v)
    q = np.array([READY, READY, [0.002, 0.5, 0.8, 0, 0, 0]])  # 0.2 rad/s in one tick: 20 rad/s^2
    assert any(x.kind == "acceleration" for x in env.check_path(q, 0.01))


def test_home_paths():
    out = plan_move(HOME, READY)  # leaving home: starts outside the box
    assert env.check_path(out.q, out.dt) != []
    assert env.check_path(out.q, out.dt, from_outside=True) == []
    back = plan_home(READY)
    assert back.allow_home
    assert env.check_path(back.q, back.dt, allow_home=True, from_outside=True) == []
    assert env.check_path(back.q, back.dt) != []  # without the home rule it leaves the box
    # final approach is slow: last 0.1 rad of j2 at <= 0.05 rad/s
    v = np.abs(np.diff(back.q[:, 1])) / back.dt
    tail = back.q[1:, 1] < 0.1
    assert v[tail].max() <= 0.05 * 1.001


def test_from_outside_must_move_back():
    # j4 past its URDF limit (as seen on the real arm, 1.638): recovering inward is fine...
    rec = plan_move([0, 0.5, 0.8, 1.638, 0, 0], READY)
    assert env.check_path(rec.q, rec.dt, from_outside=True) == []
    # ...going further out is not
    worse = plan_move([0, 0.5, 0.8, 1.5, 0, 0], [0, 0.5, 0.8, 1.6, 0, 0])
    assert env.check_path(worse.q, worse.dt, from_outside=True)[0].joint == "j4"


def test_clamp_step():
    qd, v = env.clamp_step([0, 0.5, 0.8, 1.45, 0, 0], [0.1, 0, 0, 0.3, 0, 0], env.DEFAULT.lookahead())
    assert qd.tolist() == [0.1, 0, 0, 0, 0, 0]
    assert v[0].joint == "j4" and "motion zeroed" in v[0].message
    # at home, moving j2/j3 up (into the box) is allowed, down is not
    qd, v = env.clamp_step(HOME, [0, 0.3, 0.3, 0, 0, 0], 0.16)
    assert qd.tolist() == [0, 0.3, 0.3, 0, 0, 0] and v == []
    qd, v = env.clamp_step(HOME, [0, -0.3, 0, 0, 0, 0], 0.16)
    assert qd[1] == 0 and v[0].joint == "j2"


def test_cap_velocity_and_describe():
    qd, s = env.DEFAULT.cap_velocity([0.6, 0.3, 0, 0, 0, 0])
    assert s == pytest.approx(0.5) and qd.tolist() == pytest.approx([0.3, 0.15, 0, 0, 0, 0])
    d = env.DEFAULT.describe()
    json.dumps(d)
    assert d["soft_limits"]["j4"] == [-1.5581, 1.471]  # DimOS adapter lower -1.65806 (2026-10-28)


def test_plan_move_respects_caps():
    t = plan_move(HOME, [1.0, 0.5, 0.8, 0, 0, 0])
    v = np.abs(np.diff(t.q, axis=0)) / t.dt
    assert v.max() <= env.VEL_CAP * 1.001
    assert t.q[0].tolist() == list(HOME) and t.q[-1].tolist() == pytest.approx([1.0, 0.5, 0.8, 0, 0, 0])


def test_joint_index_and_twist():
    assert env.joint_index("j4") == env.joint_index("yam_joint4") == env.joint_index(4) == 3
    with pytest.raises(ValueError):
        env.joint_index("j7")
    assert twist_to_qd({"x": 0.1})[1] > 0
    with pytest.raises(ValueError):
        twist_to_qd({"w": 1})
