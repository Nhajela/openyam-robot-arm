"""MockBackend Cartesian twists with real kinematics (armctl.kin.jacobian_geometric + DLS).

Drives the sim deterministically with ``step(now)`` (no background thread).
"""
from __future__ import annotations

import time

import numpy as np
import pytest

from armctl import envelope as env
from armctl import kin
from armctl.backend import MockBackend, twist_qd

READY = list(env.POSES["ready"])


def test_geometric_jacobian_matches_finite_motion():
    rng = np.random.default_rng(3)
    for _ in range(5):
        q = rng.uniform(env.DEFAULT.soft_lower, env.DEFAULT.soft_upper)
        qd = rng.normal(size=6) * 0.2
        h = 1e-5
        T0, T1 = kin.fk(q), kin.fk(q + qd * h)
        lin = (T1[:3, 3] - T0[:3, 3]) / h
        ang = kin.rot_log(T1[:3, :3] @ T0[:3, :3].T) / h
        np.testing.assert_allclose(kin.jacobian_geometric(q) @ qd, np.r_[lin, ang], atol=2e-4)


def test_rot_log_round_trip():
    for w in ([0.3, -0.2, 0.5], [0.0, 0.0, 1e-12], [0.0, 3.1, 0.0]):
        w = np.array(w)
        a = np.linalg.norm(w)
        K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]]) / max(a, 1e-30)
        R = np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K
        np.testing.assert_allclose(kin.rot_log(R), w, atol=1e-6)


def _run(b: MockBackend, twist: dict, seconds: float, ttl: float = 5.0, t0: float | None = None):
    t0 = time.monotonic() if t0 is None else t0
    b.set_twist(twist, ttl)
    n = int(round(seconds / b.dt))
    for k in range(1, n + 1):
        b.step(t0 + k * b.dt)
    return t0 + n * b.dt


def _mock(q0=READY):
    events: list[tuple[str, dict]] = []
    b = MockBackend(q0=q0, autostart=False, on_event=lambda k, d: events.append((k, d)))
    return b, events


@pytest.mark.parametrize("axis,sign", [(0, 1), (0, -1), (2, 1), (2, -1)])
def test_twist_moves_tip_along_the_axis(axis, sign):
    b, events = _mock()
    p0, R0 = kin.tip(b.q), kin.fk(b.q)[:3, :3]
    _run(b, {"xyz"[axis]: 0.05 * sign}, 0.5)
    d = kin.tip(b.q) - p0
    assert d[axis] * sign > 0.012                                     # moved >= 1.2 cm the right way
    off = np.delete(np.abs(d), axis)
    assert off.max() < 0.25 * abs(d[axis])                            # roughly not sideways
    assert np.linalg.norm(kin.rot_log(kin.fk(b.q)[:3, :3] @ R0.T)) < 0.03   # fingers keep orientation
    assert not [e for e in events if e[0] == "guard"]


def test_plus_y_and_yaw():
    b, _ = _mock()
    p0 = kin.tip(b.q)
    _run(b, {"y": 0.05}, 0.5)
    d = kin.tip(b.q) - p0
    assert d[1] > 0.004 and abs(d[0]) < 0.3 * d[1] and abs(d[2]) < 0.3 * d[1]
    b, _ = _mock()
    p0, R0 = kin.tip(b.q), kin.fk(b.q)[:3, :3]
    _run(b, {"yaw": 0.3}, 0.5)
    w = kin.rot_log(kin.fk(b.q)[:3, :3] @ R0.T)
    assert w[2] > 0.03 and np.linalg.norm(kin.tip(b.q) - p0) < 0.003


def test_twist_respects_velocity_and_accel_caps():
    b, _ = _mock()
    t = time.monotonic()
    b.set_twist({"x": 0.5, "y": 0.5, "z": 0.5}, 5.0)                  # far more than the caps allow
    prev = b.qd.copy()
    for k in range(1, 101):
        b.step(t + k * b.dt)
        assert np.all(np.abs(b.qd) <= b.envelope.vel_caps + 1e-9)
        assert np.all(np.abs(b.qd - prev) <= b.envelope.acc_caps * b.dt + 1e-9)
        prev = b.qd.copy()


def test_deadman_stops_a_twist():
    b, events = _mock()
    t_end = _run(b, {"x": 0.05}, 0.3, ttl=0.3)                         # stream for 0.3 s, then silence
    for k in range(1, 61):
        b.step(t_end + k * b.dt + 0.01)
    assert [e for e in events if e[0] == "deadman"]
    assert not np.any(b.qd) and b.mode == "hold"
    q = b.q.copy()
    for k in range(61, 90):
        b.step(t_end + k * b.dt)
    np.testing.assert_array_equal(b.q, q)


def test_twist_guarded_at_the_soft_limit():
    q0 = [0.0, 0.5, 0.8, 1.44, 0.0, 0.0]                               # j4 near its soft upper 1.471
    b, events = _mock(q0)
    p0 = kin.tip(b.q)
    j4max = 0.0
    t = time.monotonic()
    b.set_twist({"x": 0.05}, 5.0)                                     # +x wants j4 to go up
    for k in range(1, 101):
        b.step(t + k * b.dt)
        j4max = max(j4max, b.q[3])
    assert j4max <= b.envelope.soft_upper[3] + 1e-9
    guards = [d for k, d in events if k == "guard"]
    assert len(guards) == 1 and guards[0]["violations"][0]["joint"] == "j4"
    assert kin.tip(b.q)[0] - p0[0] > 0.01                             # the other joints carry on
    # a fresh command clears the block (re-checked by the guard), like a joint stream; still safe
    b.set_twist({"x": 0.05}, 5.0)
    assert not b.twist_blocked.any()
    for k in range(101, 201):
        b.step(t + k * b.dt)
        assert b.q[3] <= b.envelope.soft_upper[3] + 1e-9


def test_twist_refusals_unchanged():
    b, _ = _mock()
    with pytest.raises(ValueError):
        b.set_twist({"w": 1.0}, 0.3)
    with pytest.raises(ValueError):
        b.set_twist({"x": float("nan")}, 0.3)
    b.relax()
    with pytest.raises(RuntimeError, match="torque is off"):
        b.set_twist({"x": 0.01}, 0.3)


def test_joint_stream_after_twist_is_plain_joint_velocity():
    b, _ = _mock()
    _run(b, {"x": 0.05}, 0.1)
    b.set_joint_velocity([0.1, 0, 0, 0, 0, 0], 5.0)
    assert b.cmd_twist is None
    t = time.monotonic()
    for k in range(1, 51):
        b.step(t + k * b.dt)
    assert b.qd[0] == pytest.approx(0.1) and np.allclose(b.qd[1:], 0, atol=1e-9)


def test_twist_qd_helper():
    qd = twist_qd(READY, {"z": 0.05})
    J = kin.jacobian_geometric(READY)
    np.testing.assert_allclose(J @ qd, [0, 0, 0.05, 0, 0, 0], atol=0.004)
