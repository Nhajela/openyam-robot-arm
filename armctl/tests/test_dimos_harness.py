"""The DimOS adapter-limit harness (incident 2026-10-28: armctl let j1 reach +1.62, the Damiao
adapter latched "yam_joint1 reported 1.62 outside [-3.927, 1.5708]" and the arm went limp).

Layers: 1 envelope.URDF_LIMITS = i2rt ∩ DimOS adapter limits; 1b dimos_limits.check_pinned()
at connect; 2 DimosBackend's last-line gate on every position sent; 3 the edge-of-reach watchdog
for streams; 4 adapter faults -> daemon E-STOP + state["fault"]. Fakes only: no DimOS, no zenoh,
no CAN.

Run: .venv/bin/python -m pytest -q armctl/tests/test_dimos_harness.py
"""
import asyncio
import logging
import time

import numpy as np
import pytest

from armctl import backend as b
from armctl import daemon as d
from armctl import dimos_limits as dl
from armctl import envelope as env
from test_dimos_backend import READY, FakeLink, make, run_for, sim_info

FAULT_TEXT = ("Damiao feedback fault is latched: yam_joint1 reported 1.62 outside "
              "[-3.92699, 1.5708] by more than 0.05 rad")


# ---------------------------------------------------------------- layer 1: intersection
def test_urdf_limits_are_the_intersection():
    L = env.URDF_LIMITS
    assert L["j1"] == (-2.618, 1.5708)  # i2rt lower, DimOS upper (the incident joint)
    assert L["j6"] == (-2.094, 1.8326)
    assert L["j4"] == (-1.6581, 1.571)
    for j in env.JOINTS:
        for src in (env.I2RT_URDF_LIMITS, env.DIMOS_ADAPTER_LIMITS):
            assert src[j][0] - 1e-4 <= L[j][0] and L[j][1] <= src[j][1] + 1e-4
    E = env.DEFAULT
    assert E.soft_upper[0] == pytest.approx(1.4708)
    assert E.check_point([1.47, 0.5, 0.8, 0, 0, 0]) == []
    assert any(v.joint == "j1" for v in E.check_point([1.6, 0.5, 0.8, 0, 0, 0]))


# ---------------------------------------------------------------- layer 1b: installed model file
URDF_T = """<robot name="yam">{joints}<joint name="yam_hand_joint" type="fixed"/></robot>"""


def write_urdf(path, limits):
    js = "".join(f'<joint name="yam_joint{j[1]}" type="revolute"><limit effort="10" lower="{lo}" '
                 f'upper="{hi}" velocity="3"/></joint>' for j, (lo, hi) in limits.items())
    path.write_text(URDF_T.format(joints=js))
    return path


def test_check_pinned_with_temp_urdf(tmp_path):
    pinned = dict(env.DIMOS_ADAPTER_LIMITS)
    same = write_urdf(tmp_path / "same.urdf", pinned)
    assert dl.read_limits(same)["j1"] == (-3.92699, 1.5708)
    assert dl.check_pinned([same]) == []
    looser = write_urdf(tmp_path / "loose.urdf", {**pinned, "j1": (-4.0, 2.0)})
    assert dl.check_pinned([looser]) == []  # looser is fine: armctl stays inside the pinned box
    tighter = write_urdf(tmp_path / "tight.urdf", {**pinned, "j1": (-3.92699, 1.50), "j6": (-2.0, 1.8326)})
    probs = dl.check_pinned([tighter])
    assert len(probs) == 2 and "j1 upper limit 1.50000" in probs[0] and "j6 lower" in probs[1]
    assert "not found" in dl.check_pinned([tmp_path / "missing.urdf"])[0]
    broken = write_urdf(tmp_path / "broken.urdf", {k: v for k, v in pinned.items() if k != "j3"})
    assert "unreadable" in dl.check_pinned([broken])[0] and "j3" in dl.check_pinned([broken])[0]
    assert dl.candidate_paths({"ARMCTL_DIMOS_URDF": str(same)}) == [same]


def real_info():
    return sim_info(adapters={"openyam": "openyam_damiao"}, simulation=False, can_port="nonexistent_can9",
                    scout_addr=b.DIMOS_REAL_BUS)


def test_backend_refuses_real_adapter_on_limit_mismatch(monkeypatch):
    monkeypatch.setenv("ZENOH_SCOUT_ADDR", b.DIMOS_REAL_BUS)  # fake link: no zenoh session is opened
    calls = []
    monkeypatch.setattr(dl, "check_pinned", lambda: calls.append(1) or ["j1 upper limit 1.50000 is tighter"])
    link = FakeLink(lambda: 0.0, q0=READY, info=real_info())
    with pytest.raises(RuntimeError, match="do not match armctl's pinned DIMOS_ADAPTER_LIMITS.*j1 upper"):
        b.DimosBackend(env.DEFAULT, link=link, expect="real", autostart=False)
    assert link.closed and calls == [1]
    monkeypatch.setattr(dl, "check_pinned", lambda: [])
    be = b.DimosBackend(env.DEFAULT, link=FakeLink(lambda: 0.0, q0=READY, info=real_info()), expect="real",
                        autostart=False)
    assert be.adapter == "real"
    # sim adapters are not checked (the mock has no Damiao feedback limits)
    monkeypatch.setattr(dl, "check_pinned", lambda: pytest.fail("checked a sim adapter"))
    monkeypatch.setenv("ZENOH_SCOUT_ADDR", b.DIMOS_SIM_BUS)
    make(q0=READY)


# ---------------------------------------------------------------- layer 2: last-line gate
def test_gate_refuses_j1_and_j6_past_dimos_limits():
    t = np.linspace(0, 1, 50)[:, None]
    to_j1 = READY + t * (np.array([1.6, 0.5, 0.8, 0, 0, 0]) - READY)
    with pytest.raises(RuntimeError, match=r"j1 1\.600 > 1\.541 \(DimOS limit 1\.5708\)"):
        b.dimos_gate(to_j1)
    to_j6 = READY + t * (np.array([0, 0.5, 0.8, 0, 0, 1.85]) - READY)
    with pytest.raises(RuntimeError, match=r"j6 1\.850 > 1\.803"):
        b.dimos_gate(to_j6)
    b.dimos_gate(READY + t * (np.array([1.47, 0.5, 0.8, 0, 0, 1.73]) - READY))  # soft box is fine
    b.dimos_gate(b.plan_home(READY).q)  # home: j2/j3 on their stops at the limit 0
    b.dimos_gate([[0, -0.015, -0.01, 0, 0, 0], [0, 0.1, 0.1, 0, 0, 0]])  # resting a hair below 0
    with pytest.raises(RuntimeError, match="j2"):
        b.dimos_gate([[0, 0.1, 0.1, 0, 0, 0], [0, -0.04, 0.1, 0, 0, 0]])
    # starting outside the gate (overshoot) may move back in, never further out, never past the limit
    back = np.linspace([1.56, 0.5, 0.8, 0, 0, 0], [1.3, 0.5, 0.8, 0, 0, 0], 20)
    b.dimos_gate(back)
    with pytest.raises(RuntimeError, match="j1"):
        b.dimos_gate(np.linspace([1.56, 0.5, 0.8, 0, 0, 0], [1.565, 0.5, 0.8, 0, 0, 0], 20))


def test_backend_execute_gate_sends_nothing():
    be, link, clock, ev = make(q0=READY)
    for target in ([1.6, 0.5, 0.8, 0, 0, 0], [0, 0.5, 0.8, 0, 0, 1.85]):
        q = np.linspace(READY, target, 200)  # bypasses the daemon's envelope gate on purpose
        with pytest.raises(RuntimeError, match="DimOS limit gate refused"):
            be.execute(b.Trajectory(q, b.DT, label="bad"))
    assert link.executed == [] and be.state()["mode"] == "hold"
    be.execute(b.plan_move(READY, [1.4, 0.5, 0.8, 0, 0, 0], label="ok"))
    assert len(link.executed) == 1


def test_joint_stream_gate_refuses_step(monkeypatch):
    """A joint-stream step past the gate (here: a planner bug that ignores the envelope) is not sent."""
    q0 = READY.copy()
    q0[0] = 1.45
    be, link, clock, ev = make(q0=q0)
    monkeypatch.setattr(b, "velocity_plan", lambda q, qd, cmd, t, E: (np.linspace(q, q + [0.2, 0, 0, 0, 0, 0], 30), []))
    be.set_joint_velocity([0.3, 0, 0, 0, 0, 0], 0.5)
    assert any(k == "gate" for k, _ in ev)
    assert all(p[:, 0].max() <= 1.5408 + 1e-9 for _, p in link.executed)


# ---------------------------------------------------------------- layer 3: edge watchdog
class MovingLink(FakeLink):
    """FakeLink whose twists move the arm (armctl's own Jacobian, like Pink IK would)."""

    def __init__(self, clock, q0=None):
        super().__init__(clock, q0)
        self.t_last = clock()

    def joint_positions(self):
        now = self.clock()
        if self.run is None and self.twists and self.twists[-1].any():
            self.q = self.q + b.twist_qd(self.q, self.twists[-1]) * (now - self.t_last)
        self.t_last = now
        return super().joint_positions()


def make_moving(q0):
    be, link, clock, ev = make(q0=q0)
    ml = MovingLink(clock, q0)
    be.link = ml
    return be, ml, clock, ev


def steer(be, clock, v, seconds):
    raised = None
    for _ in range(int(round(seconds / 0.04))):
        try:
            be.set_twist(v, 0.2)
        except b.EdgeOfReach as e:
            raised = e
        run_for(be, clock, 0.04)
    return raised


def test_watchdog_stops_twist_near_j1_limit_and_allows_reverse():
    q0 = READY.copy()
    q0[0] = 1.25
    be, link, clock, ev = make_moving(q0)
    yaw = 0.3 if b.twist_qd(q0, {"yaw": 1.0})[0] > 0 else -0.3  # the yaw that turns j1 up
    lim = min(env.DEFAULT.soft_upper[0], env.DIMOS_ADAPTER_LIMITS["j1"][1])
    steer(be, clock, {"yaw": yaw}, 1.5)
    edges = [dd for k, dd in ev if k == "edge"]
    assert edges and edges[0]["joint"] == "j1" and edges[0]["limit"] == pytest.approx(lim, abs=1e-4)
    assert edges[0]["message"].startswith("edge of reach: j1 near its limit (") and "steer back" in edges[0]["message"]
    assert lim - b.EDGE_ZONE - 0.02 < edges[0]["q"] < lim
    assert not link.twists[-1].any()  # stopped with zero twists
    q_stop = be.state()["q"][0]
    assert q_stop < lim and be.state()["edge"] == ["j1+"]
    # pushing on is refused at once, and nothing moves
    with pytest.raises(b.EdgeOfReach, match=r"edge of reach: j1 near its limit \(1\.\d\d / 1\.47\): steer back"):
        be.set_twist({"yaw": yaw}, 0.2)
    run_for(be, clock, 0.3)
    assert be.state()["q"][0] == pytest.approx(q_stop, abs=1e-3)
    # steering back is allowed and clears the latch once clear of the zone
    assert steer(be, clock, {"yaw": -yaw}, 1.5) is None
    assert be.state()["q"][0] < lim - b.EDGE_ZONE - b.EDGE_RELEASE
    assert be.state()["edge"] == []
    assert sum(1 for k, _ in ev if k == "edge") == 1


def test_watchdog_ignores_motion_away_from_a_near_limit():
    q0 = READY.copy()
    q0[0] = env.DEFAULT.soft_upper[0] - 0.05  # already inside the zone
    be, link, clock, ev = make_moving(q0)
    yaw = 0.3 if b.twist_qd(q0, {"yaw": 1.0})[0] > 0 else -0.3
    assert steer(be, clock, {"yaw": -yaw}, 0.5) is None
    assert not any(k == "edge" for k, _ in ev) and be.state()["q"][0] < q0[0] - 0.05


# ---------------------------------------------------------------- layer 4: adapter faults
class FaultyLink(FakeLink):
    fail = False

    def joint_positions(self):
        if self.fail:
            raise RuntimeError(FAULT_TEXT)
        return super().joint_positions()

    def execute(self, *a):
        if self.fail:
            raise RuntimeError(FAULT_TEXT)
        return super().execute(*a)


def test_fault_latches_backend_and_refuses_motion(caplog):
    be, link, clock, ev = make(q0=READY)
    fl = FaultyLink(clock, READY)
    be.link = fl
    be.execute(b.plan_move(READY, READY + [0.2, 0, 0, 0, 0, 0], label="m"))
    run_for(be, clock, 0.2)
    fl.fail = True
    with caplog.at_level(logging.ERROR, logger="armctl"):
        run_for(be, clock, 0.1)
    assert any("DimOS ADAPTER FAULT" in r.getMessage() for r in caplog.records if r.levelno == logging.ERROR)
    st = be.state()
    f = st["fault"]
    assert f and f["joint"] == "j1" and "1.62 outside" in f["reason"] and "disable_all" in f["message"]
    assert st["mode"] == "estop" and st["torque"] is False
    assert ("trajectory_done", {"label": "m", "ok": False}) in ev
    assert [k for k, _ in ev].count("fault") == 1
    for call in (lambda: be.execute(b.plan_move(READY, READY, label="x")), lambda: be.set_twist({"x": 0.01}, 0.2),
                 lambda: be.set_joint_velocity([0.1, 0, 0, 0, 0, 0], 0.2), lambda: be.gripper(0.5), be.enable):
        with pytest.raises(b.ArmFaulted, match="ARM FAULTED"):
            call()
    be.stop()
    be.estop()  # still safe to call
    assert be.state()["fault"] == f  # no auto-recovery
    run_for(be, clock, 2.0)
    assert be.state()["fault"] == f


def test_fault_detected_from_execute_error():
    be, link, clock, ev = make(q0=READY)
    fl = FaultyLink(clock, READY)
    fl.fail = True
    be.link = fl
    with pytest.raises(b.ArmFaulted):
        be.execute(b.plan_move(READY, READY + [0.1, 0, 0, 0, 0, 0], label="m"))
    assert be.state()["fault"]["joint"] == "j1"


def test_daemon_fault_estop_state_flag_and_refusals(caplog):
    async def main():
        link = FaultyLink(time.monotonic, READY)
        be = b.DimosBackend(env.DEFAULT, link=link)
        daemon = d.Daemon(be, auto_approve=True)
        await daemon.start()
        inbox = []

        async def send(m):
            inbox.append(m)
        c = d.Client(send, "test")
        try:
            await daemon.connect(c)
            await daemon.handle(c, {"op": "hello", "program": "t"})
            await daemon.handle(c, {"op": "acquire"})
            assert daemon.holder is c
            link.fail = True
            t_end = time.monotonic() + 3
            while not any(m["op"] == "fault" for m in inbox) and time.monotonic() < t_end:
                await asyncio.sleep(0.02)
            assert daemon.fault and daemon.estopped and daemon.holder is None
            assert any(m["op"] == "fault" and m["joint"] == "j1" for m in inbox)
            assert any(m["op"] == "estop" and m["latched"] for m in inbox)
            sm = daemon.state_msg()
            assert sm["fault"]["joint"] == "j1" and sm["estop"] and sm["torque"] is False
            assert daemon.status()["fault"]["joint"] == "j1"
            await daemon.handle(c, {"op": "acquire", "id": 5})
            r = inbox[-1]
            assert r["op"] == "refused" and r["reason"].startswith("ARM FAULTED") and "disable_all" in r["reason"]
            with pytest.raises(d.Refused, match="cannot reset: ARM FAULTED"):
                await daemon.estop_reset()
            with pytest.raises(d.Refused, match="cannot enable"):
                await daemon.enable()
            assert daemon.estopped
            await daemon.handle(c, {"op": "hello", "program": "t"})
            assert inbox[-1]["fault"]["joint"] == "j1"
        finally:
            await daemon.shutdown()
            be.close()
    with caplog.at_level(logging.ERROR, logger="armctl"):
        asyncio.run(main())
    assert any("ARM FAULTED" in r.getMessage() for r in caplog.records if r.levelno == logging.ERROR)


def test_daemon_relays_edge_refusal_rate_limited():
    async def main():
        be, link, clock, ev = make(q0=READY)
        daemon = d.Daemon(be, auto_approve=True)
        inbox = []

        async def send(m):
            inbox.append(m)
        c = d.Client(send, "test")
        await daemon.connect(c)
        await daemon.handle(c, {"op": "hello", "program": "t"})
        daemon.holder = c
        msg = "edge of reach: j1 near its limit (1.40 / 1.47): steer back"
        for _ in range(3):
            await daemon._backend_event("edge", {"joint": "j1", "q": 1.4, "limit": 1.4708, "message": msg})
        refs = [m for m in inbox if m["op"] == "refused"]
        assert len(refs) == 1 and refs[0]["reason"] == msg and refs[0]["stream"] and refs[0]["edge"]
        assert refs[0]["joint"] == "j1"
    asyncio.run(main())
