"""DimosBackend unit tests. They need NO DimOS: a fake coordinator link stands in for
``DimosLink`` (perfect position tracking, DimOS-like trajectory/status/E-STOP semantics).
The one test that builds real DimOS messages skips when dimos is not importable.

Run: .venv/bin/python -m pytest armctl/tests/test_dimos_backend.py
End to end against DimOS --simulation: armctl/tests/e2e_dimos_sim.py (not collected).
"""
import asyncio
import copy
import logging

import numpy as np
import pytest

from armctl import backend as b
from armctl import daemon as d
from armctl import envelope as env

READY = np.array(env.POSES["ready"], float)
logging.getLogger("armctl").handlers.clear()


def sim_info(**over):
    arm = list(b.DIMOS_ARM_JOINTS)
    info = {
        "launcher": "armctl.dimos_coordinator/1", "adapters": {"openyam": "mock_whole_body"},
        "simulation": True, "can_port": None, "pid": 1, "scout_addr": b.DIMOS_SIM_BUS,
        "tasks": {
            "eef_twist_arm": {"type": "eef_twist", "priority": 10, "joints": arm, "params": {
                "target_frame": "gripper_tip", "command_limit_margin": 0.1, "max_joint_velocity_rad_s": 0.3,
                "timeout": 0.3}},
            "joint_trajectory": {"type": "trajectory", "priority": 20, "joints": arm, "params": {
                "start_position_tolerance": 0.05, "velocity_limits": {j: 0.5 for j in arm}}},
            "openyam_gripper": {"type": "gripper", "priority": 20, "joints": ["arm/gripper"], "params": {}},
        },
    }
    info.update(over)
    return info


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class FakeLink:
    """Stands in for DimosLink. Trajectories are followed exactly (linear interpolation, like
    DimOS's sampler); a new one continues from the commanded position (DimOS anchoring)."""

    def __init__(self, clock, q0=None, info=None):
        self.clock = clock
        self.q = np.array(q0 if q0 is not None else np.zeros(6), float)
        self.grip = 0.0
        self.info_d = info or sim_info()
        self.run = None  # (t0, times, positions)
        self.st = "IDLE"
        self.estopped = False
        self.executed = []  # (times, positions)
        self.cancels = 0
        self.twists = []
        self.estop_calls = []
        self.closed = False

    def _advance(self):
        if self.run is not None:
            t0, ts, ps = self.run
            e = self.clock() - t0
            if e >= ts[-1]:
                self.q = ps[-1].copy()
                self.run, self.st = None, "COMPLETED"
            else:
                self.q = np.array([np.interp(e, ts, ps[:, i]) for i in range(6)])

    def info(self):
        return copy.deepcopy(self.info_d)

    def joint_positions(self):
        self._advance()
        return {**{f"yam_joint{i + 1}": float(self.q[i]) for i in range(6)}, "arm/gripper": self.grip}

    def execute(self, times, positions, velocities):
        self._advance()
        assert len(times) >= 2 and times[0] == 0.0 and all(np.diff(times) > 0)
        ps = np.array(positions, float)
        if self.run is None and np.max(np.abs(ps[0] - self.q)) > 0.05:
            return False, "START_STATE_MISMATCH"
        ps[0] = self.q
        self.executed.append((list(times), np.array(positions, float)))
        self.run, self.st = (self.clock(), np.array(times), ps), "EXECUTING"
        return True, "ACCEPTED"

    def cancel(self):
        self._advance()
        self.cancels += 1
        if self.st == "EXECUTING":
            self.run, self.st = None, "ABORTED"
            return "CANCELLED"
        return "ALREADY_STOPPED"

    def status(self):
        self._advance()
        return self.st, (self.clock() - self.run[0]) if self.run else 0.0

    def set_estop(self, on):
        self.estop_calls.append(bool(on))
        self.estopped = bool(on)
        return True

    def gripper(self, w):
        if self.estopped:
            return False
        self.grip = float(w)
        return True

    def publish_twist(self, v):
        self.twists.append(np.array(v, float))

    def subscribe_joint_state(self, cb):
        pass  # no stream: the backend polls

    def close(self):
        self.closed = True


def make(q0=None, info=None, events=None):
    clock = Clock()
    link = FakeLink(clock, q0, info)
    ev = [] if events is None else events
    be = b.DimosBackend(env.DEFAULT, lambda k, dd: ev.append((k, dd)), link=link, autostart=False, clock=clock)
    return be, link, clock, ev


def run_for(be, clock, seconds, dt=0.01):
    for _ in range(int(round(seconds / dt))):
        clock.t += dt
        be._tick(clock.t)


# ---------------------------------------------------------------- pure helpers
def test_decimate_multi_point_within_tolerance():
    tr = b.plan_move(np.zeros(6), READY)
    idx = b.decimate(tr.q, tr.dt)
    assert idx[0] == 0 and idx[-1] == len(tr.q) - 1
    assert len(idx) >= 10 and len(idx) < len(tr.q) / 3  # multi-point, but far fewer than 100 Hz
    assert max(np.diff(idx)) * tr.dt <= b.TRAJ_MAX_GAP + 1e-9
    t = np.arange(len(tr.q)) * tr.dt
    for i in range(6):
        approx = np.interp(t, t[idx], tr.q[idx, i])
        assert np.max(np.abs(approx - tr.q[:, i])) <= b.TRAJ_TOL + 1e-9


def test_trajectory_points_refuses_single_point_and_nan():
    with pytest.raises(ValueError, match="single-point"):
        b.trajectory_points(b.Trajectory(np.zeros((1, 6))))
    bad = b.plan_move(np.zeros(6), READY)
    bad.q[3, 2] = np.nan
    with pytest.raises(ValueError, match="NaN"):
        b.trajectory_points(bad)
    times, pos, vel = b.trajectory_points(b.Trajectory(np.zeros((2, 6))))
    assert times == [0.0, 0.01] and np.all(vel == 0)


def test_bus_rules():
    e = {}
    assert b.dimos_bus("sim", e) == b.DIMOS_SIM_BUS and e["ZENOH_SCOUT_ADDR"] == b.DIMOS_SIM_BUS
    for bus in ("", "224.0.0.224:7446"):
        with pytest.raises(RuntimeError, match="default zenoh bus"):
            b.dimos_bus("sim", {"ZENOH_SCOUT_ADDR": bus})
    with pytest.raises(RuntimeError, match="REAL-arm bus"):
        b.dimos_bus("sim", {"ZENOH_SCOUT_ADDR": b.DIMOS_REAL_BUS})
    with pytest.raises(RuntimeError, match="needs ZENOH_SCOUT_ADDR"):
        b.dimos_bus("real", {"ZENOH_SCOUT_ADDR": b.DIMOS_SIM_BUS})
    with pytest.raises(RuntimeError, match="default zenoh bus"):
        b.dimos_bus("real", {})  # real never defaults
    assert b.dimos_bus("real", {"ZENOH_SCOUT_ADDR": b.DIMOS_REAL_BUS}) == b.DIMOS_REAL_BUS
    with pytest.raises(RuntimeError):
        b.dimos_bus("maybe", {})


def test_verify_coordinator():
    E = env.DEFAULT
    warns, adapter = b.verify_coordinator(sim_info(), E, "sim", b.DIMOS_SIM_BUS)
    assert adapter == "sim" and warns == []
    real = sim_info(adapters={"openyam": "openyam_damiao"}, simulation=False, can_port="nonexistent_can9")
    with pytest.raises(RuntimeError, match="REAL hardware"):
        b.verify_coordinator(real, E, "sim", b.DIMOS_SIM_BUS)
    assert b.verify_coordinator({**real, "scout_addr": b.DIMOS_REAL_BUS}, E, "real", b.DIMOS_REAL_BUS)[1] == "real"
    with pytest.raises(RuntimeError, match="is a simulation"):
        b.verify_coordinator(sim_info(scout_addr=b.DIMOS_REAL_BUS), E, "real", b.DIMOS_REAL_BUS)
    i = sim_info()
    del i["tasks"]["joint_trajectory"]
    with pytest.raises(RuntimeError, match="'joint_trajectory' missing"):
        b.verify_coordinator(i, E, "sim", b.DIMOS_SIM_BUS)
    i = sim_info()
    i["tasks"]["teleop"] = {"type": "teleop_ik", "priority": 30, "joints": [], "params": {}}
    with pytest.raises(RuntimeError, match="unexpected tasks"):
        b.verify_coordinator(i, E, "sim", b.DIMOS_SIM_BUS)
    i = sim_info()
    i["tasks"]["eef_twist_arm"]["params"]["command_limit_margin"] = 1e-4
    with pytest.raises(RuntimeError, match="command_limit_margin"):
        b.verify_coordinator(i, E, "sim", b.DIMOS_SIM_BUS)
    i = sim_info()
    i["tasks"]["eef_twist_arm"]["params"]["max_joint_velocity_rad_s"] = 1.0
    with pytest.raises(RuntimeError, match="joint speed"):
        b.verify_coordinator(i, E, "sim", b.DIMOS_SIM_BUS)
    with pytest.raises(RuntimeError, match="on bus"):
        b.verify_coordinator(sim_info(), E, "sim", "224.0.0.99:7000")
    with pytest.raises(RuntimeError, match="launcher"):
        b.verify_coordinator(sim_info(launcher="dimos run coordinator-openyam"), E, "sim", b.DIMOS_SIM_BUS)
    i = sim_info()
    i["tasks"]["joint_trajectory"]["params"]["velocity_limits"] = {j: 0.2 for j in b.DIMOS_ARM_JOINTS}
    assert "lag" in b.verify_coordinator(i, E, "sim", b.DIMOS_SIM_BUS)[0][0]


def test_constructor_refuses_bad_coordinator_and_closes_link():
    clock = Clock()
    link = FakeLink(clock, info=sim_info(tasks={}))
    with pytest.raises(RuntimeError, match="missing"):
        b.DimosBackend(env.DEFAULT, link=link, autostart=False, clock=clock)
    assert link.closed


def test_velocity_plan_ends_at_rest_within_caps():
    s, viol = b.velocity_plan(READY, np.zeros(6), [0.3, 0, 0, 0, 0, 0], 0.2, env.DEFAULT)
    v = np.diff(s, axis=0) / b.DT
    a = np.diff(v, axis=0) / b.DT
    assert not viol and np.all(np.abs(v) <= 0.3 + 1e-9) and np.all(np.abs(a) <= 1.0 + 1e-6)
    assert np.allclose(s[-1], s[-2])  # at rest at the end
    near = READY.copy()
    near[4] = env.DEFAULT.soft_upper[4] - 0.01
    s, viol = b.velocity_plan(near, np.zeros(6), [0, 0, 0, 0, 0.3, 0], 0.2, env.DEFAULT)
    assert viol and viol[0].joint == "j5" and np.max(s[:, 4]) <= env.DEFAULT.soft_upper[4] + 1e-9


def test_brake_along_path_stays_on_path():
    tr = b.plan_move(np.zeros(6), [1.0, 0.5, 0.8, 0, 0, 0])
    t = tr.duration * 0.4
    br = b.brake_along_path(tr.q, tr.dt, t, env.DEFAULT.acc_caps)
    k = np.arange(len(tr.q))
    for row in br:  # every braking sample is (an interpolation of) a point of the gated path
        kk = np.interp(row[0], tr.q[:, 0], k)
        assert np.allclose(row, b._interp_path(tr.q, kk), atol=1e-6)
    v = np.diff(br, axis=0) / b.DT
    assert np.max(np.abs(v[-1])) < 0.01 and np.all(np.abs(np.diff(v, axis=0)) / b.DT <= 1.0 + 0.05)
    assert b.brake_along_path(tr.q, tr.dt, tr.duration + 1, env.DEFAULT.acc_caps) is None


# ---------------------------------------------------------------- backend against the fake link
def test_state_keys_superset_of_mock():
    be, *_ = make()
    mock = b.MockBackend(autostart=False)
    assert set(mock.state()) <= set(be.state())
    st = be.state()
    assert st["adapter"] == "sim" and st["backend"] == "dimos" and st["torque"] is True
    assert "SIM" in be.description


def test_execute_reports_done_after_completion():
    be, link, clock, ev = make()
    tr = b.plan_move(np.zeros(6), READY, label="ready#1")
    be.execute(tr)
    assert be.state()["mode"] == "trajectory" and len(link.executed) == 1
    times, pos = link.executed[0]
    assert 2 < len(times) < len(tr.q) and times[-1] == pytest.approx(tr.duration)
    run_for(be, clock, tr.duration + 0.2)
    assert ("trajectory_done", {"label": "ready#1", "ok": True}) in ev
    st = be.state()
    assert st["mode"] == "hold" and np.allclose(st["q"], READY, atol=1e-6)


def test_execute_refused_by_dimos_raises():
    be, link, clock, ev = make()
    with pytest.raises(RuntimeError, match="START_STATE_MISMATCH"):
        be.execute(b.plan_move(READY, READY + 0.1))  # starts 0.5 rad away from where the arm is
    assert be.state()["mode"] == "hold"


def test_superseded_and_stopped_trajectories_report_not_ok():
    be, link, clock, ev = make()
    be.execute(b.plan_move(np.zeros(6), READY, label="a"))
    run_for(be, clock, 0.5)
    be.execute(b.plan_move(be.state()["q"], READY, label="b"))
    assert ("trajectory_done", {"label": "a", "ok": False}) in ev
    run_for(be, clock, 1.0)
    q_at_stop = np.array(be.state()["q"])
    be.stop()
    assert ("trajectory_done", {"label": "b", "ok": False}) in ev
    brake_t, brake_p = link.executed[-1]
    assert brake_t[-1] <= 0.3 + 0.05 + 1e-9  # v <= 0.3 rad/s at 1 rad/s^2
    run_for(be, clock, 0.6)
    st = be.state()
    assert st["mode"] == "hold" and not st["moving"]
    assert np.max(np.abs(np.array(st["q"]) - q_at_stop)) < 0.05  # stopped close to where it was


def test_twist_deadman_sends_zero_and_reports():
    be, link, clock, ev = make(q0=READY)
    be.set_twist({"x": 0.02}, 0.2)
    assert len(link.twists) == 1 and link.twists[0][0] > 0  # first setpoint goes out at once
    run_for(be, clock, 0.15)
    sent = len(link.twists)
    assert sent >= 7 and all(t[0] > 0 for t in link.twists)  # 50 Hz while live
    run_for(be, clock, 0.2)
    assert ("deadman", {}) in ev
    zeros = [t for t in link.twists[sent:] if not t.any()]
    assert len(zeros) == b.TWIST_ZERO_REPEATS and not link.twists[-1].any()
    assert be.state()["mode"] == "hold"


def test_twist_scaled_to_caps_and_refused_when_folded():
    be, link, clock, ev = make(q0=np.zeros(6))
    with pytest.raises(ValueError, match="do ready"):
        be.set_twist({"x": 0.02}, 0.2)
    be.set_twist({}, 0.2)  # a zero twist is fine anywhere
    be2, link2, *_ = make(q0=READY)
    be2.set_twist({"x": 1.0}, 0.2)
    v = link2.twists[-1]
    assert np.linalg.norm(v[:3]) <= b.TWIST_MAX_LIN + 1e-9
    qd = b.twist_qd(READY, v)
    assert np.max(np.abs(qd)) <= 0.3 + 1e-6


def test_joint_stream_rolling_plans_end_at_rest_and_deadman():
    be, link, clock, ev = make(q0=READY)
    for _ in range(20):  # 25 Hz setpoints for 0.8 s
        be.set_joint_velocity([0.2, 0, 0, 0, 0, 0], 0.2)
        run_for(be, clock, 0.04)
    n = len(link.executed)
    assert 4 <= n <= 20
    for times, pos in link.executed:
        # every plan ends at rest: its last segment is braking (mean speed <= acc * gap / 2)
        slope = np.abs(pos[-1] - pos[-2]) / (times[-1] - times[-2])
        assert len(times) >= 2 and np.max(slope) <= 1.0 * b.TRAJ_MAX_GAP / 2 + 0.01
        assert np.allclose(pos[:, 1:], READY[1:])  # only j1 moves
    q_mid = be.state()["q"][0]
    assert q_mid > READY[0] + 0.1
    run_for(be, clock, 0.8)
    assert ("deadman", {}) in ev
    st = be.state()
    assert st["mode"] == "hold" and not st["moving"]
    assert st["q"][0] - q_mid < 0.2 * 0.2 + 0.03  # ttl at 0.2 rad/s, then braking


def test_joint_stream_guard_at_soft_limit():
    q0 = READY.copy()
    q0[4] = env.DEFAULT.soft_upper[4] - 0.02
    be, link, clock, ev = make(q0=q0)
    for _ in range(25):
        be.set_joint_velocity([0, 0, 0, 0, 0.3, 0], 0.2)
        run_for(be, clock, 0.04)
    run_for(be, clock, 0.6)
    assert any(k == "guard" for k, _ in ev)
    assert be.state()["q"][4] <= env.DEFAULT.soft_upper[4] + 1e-6


def test_estop_latches_dimos_then_clears_on_next_motion():
    be, link, clock, ev = make(q0=READY)
    be.execute(b.plan_move(READY, READY + [0.3, 0, 0, 0, 0, 0], label="m"))
    run_for(be, clock, 0.5)
    be.estop()
    assert link.estop_calls == [True] and link.cancels >= 1
    assert ("trajectory_done", {"label": "m", "ok": False}) in ev
    assert any("E-STOP" in f for f in be.state()["faults"])
    be.gripper(0.7)  # the daemon only allows this after its own reset
    assert link.estop_calls == [True, False] and link.grip == 0.7 and be.state()["faults"] == []


def test_relax_refused_enable_ok():
    be, link, *_ = make()
    with pytest.raises(RuntimeError, match="disable_all"):
        be.relax()
    be.enable()
    assert be.state()["mode"] == "hold"


def test_trajectory_gripper_samples_are_forwarded():
    be, link, clock, ev = make()
    tr = b.plan_move(np.zeros(6), READY, label="g")
    tr.grip = np.linspace(0.0, 1.0, len(tr.q))
    be.execute(tr)
    run_for(be, clock, tr.duration + 0.2)
    assert link.grip == pytest.approx(1.0) and be.state()["grip_target"] == pytest.approx(1.0)


def test_daemon_drives_dimos_backend_end_to_end():
    """The real Daemon on top of DimosBackend (fake link, real time): gate, done, refusals."""
    async def main():
        link = FakeLink(lambda: __import__("time").monotonic(), q0=READY)
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
            await daemon.handle(c, {"op": "do", "id": 1, "cmd": "nudge", "joint": "j1", "delta": 0.05})
            assert inbox[-1]["op"] == "ack"
            t_end = asyncio.get_running_loop().time() + 5
            while not any(m["op"] == "done" for m in inbox) and asyncio.get_running_loop().time() < t_end:
                await asyncio.sleep(0.02)
            assert any(m["op"] == "done" and m["id"] == 1 for m in inbox)
            assert be.state()["q"][0] == pytest.approx(0.05, abs=1e-3)
            await daemon.handle(c, {"op": "do", "id": 2, "cmd": "move_joints", "q": [0, 0.5, 0.8, 1.6, 0, 0]})
            assert inbox[-1]["op"] == "refused" and "j4" in inbox[-1]["reason"]
            await daemon.handle(c, {"op": "do", "id": 3, "cmd": "relax"})
            assert inbox[-1]["op"] == "refused" and "disable_all" in inbox[-1]["reason"]
            assert daemon.state_msg()["adapter"] == "sim"
        finally:
            await daemon.shutdown()
            be.close()
    asyncio.run(main())


def test_dimos_messages_build():
    """Real DimOS message types (skipped where dimos is not installed, e.g. the repo .venv)."""
    pytest.importorskip("dimos")
    from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
    from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint

    times, pos, vel = b.trajectory_points(b.plan_move(np.zeros(6), READY))
    jt = JointTrajectory(joint_names=list(b.DIMOS_ARM_JOINTS), points=[
        TrajectoryPoint(time_from_start=t, positions=list(p), velocities=list(v)) for t, p, v in zip(times, pos, vel)])
    assert jt.duration == pytest.approx(times[-1])
    assert np.allclose(jt.sample(times[-1] / 2)[0], pos[np.searchsorted(times, times[-1] / 2)], atol=0.02)
