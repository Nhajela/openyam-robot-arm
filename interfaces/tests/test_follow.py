"""interfaces.follow: tracking + visual-servo following, against the synthetic wrist camera and a
velocity-streaming fake arm on a virtual clock, then end to end through the armctl daemon on the
mock backend (in-process, ephemeral loopback port). No camera, no CAN, no DimOS.

    .venv/bin/python -m pytest -q interfaces/tests/test_follow.py -s
"""
from __future__ import annotations

import asyncio
import contextlib
import json
import socket
import time

import numpy as np
import pytest

from armctl import envelope as envm
from armctl import kin
from interfaces.follow import sim
from interfaces.follow.control import FollowConfig, FollowController
from interfaces.follow.look import look_pose
from interfaces.follow.track import KalmanCV, QHistory, TrackConfig, Tracker
from interfaces.servo import table_margin
from interfaces.wristcam import synth
from interfaces.wristcam.locate import CupFinder, View

T_BT = synth.T_BASE_TABLE_TRUE
T_TC = synth.TRUE_T_TIP_CAM


# --------------------------------------------------------------------------- filter, q history
def test_kalman_learns_a_constant_velocity_and_leads_it():
    # a smooth mover: low process noise (the tracker's default, 0.6 m/s^2, follows hand-pushed
    # starts and stops, and so its velocity is noisier)
    kf = KalmanCV((0.0, 0.0), 0.0, sigma=0.002, acc_sigma=0.05)
    rng = np.random.default_rng(0)
    for k in range(1, 40):
        t = k / 15
        kf.update((0.04 * t + rng.normal(0, 0.001), -0.02 * t + rng.normal(0, 0.001)), 0.001, t)
    xy, v, _ = kf.at(39 / 15 + 0.2)
    assert np.allclose(v, [0.04, -0.02], atol=0.004)
    assert np.allclose(xy, [0.04 * (39 / 15 + 0.2), -0.02 * (39 / 15 + 0.2)], atol=0.003)


def test_kalman_late_measurement_is_moved_forward():
    kf = KalmanCV((0.0, 0.0), 0.0)
    for k in range(1, 30):
        kf.update((0.05 * k / 15, 0.0), 0.001, k / 15)
    t_now = kf.t
    before = kf.at(t_now)[0].copy()
    kf.update((0.05 * (t_now - 0.5), 0.0), 0.001, t_now - 0.5)   # an anchor from 0.5 s ago, consistent
    assert np.linalg.norm(kf.at(t_now)[0] - before) < 0.003
    assert kf.t == t_now


def test_kalman_answers_for_a_past_time_too():
    """A late anchor is compared with where the track was when its frame was taken."""
    kf = KalmanCV((0.0, 0.0), 0.0, acc_sigma=0.05)
    for k in range(1, 30):
        kf.update((0.08 * k / 15, 0.0), 0.0005, k / 15)
    xy_then, _, _ = kf.at(kf.t - 0.4)
    assert xy_then[0] == pytest.approx(0.08 * (kf.t - 0.4), abs=0.002)


def test_q_history_interpolates_at_capture_time():
    h = QHistory()
    h.add(1.0, np.zeros(6))
    h.add(2.0, np.ones(6))
    assert np.allclose(h.at(1.25), 0.25)
    assert np.allclose(h.at(5.0), 1.0) and np.allclose(h.at(0.0), 0.0)
    h.add(1.5, np.full(6, 9.0))          # out of order: ignored
    assert np.allclose(h.at(1.5), 0.5)


# --------------------------------------------------------------------------- the controller
def _ctrl(**kw):
    return FollowController(T_TC, T_BT, envm.DEFAULT.vel_caps, envm.DEFAULT.acc_caps, FollowConfig(**kw))


def _run_ctrl(c, q, target, n=300, dt=1 / 30):
    worst = 1.0
    prev = np.zeros(6)
    for _ in range(n):
        qd, info = c.step(q, target, dt)
        assert np.all(np.abs(qd) <= c.vel_caps + 1e-9)
        assert np.all(np.abs(qd - prev) <= envm.DEFAULT.acc_caps * dt + 1e-9)
        prev = qd
        q = q + qd * dt
        assert not envm.DEFAULT.check_point(q)
        worst = min(worst, table_margin(q))
    return q, info, worst


def test_controller_centres_the_camera_over_a_still_target():
    q0 = look_pose((0.0, -0.12), 0.08, 0.15, T_TC, T_BT)
    c = _ctrl()
    q, info, worst = _run_ctrl(c, q0, [0.05, -0.08, 0.08])          # 6 cm away
    assert c.centring_error(q, [0.05, -0.08, 0.08]) < 0.004
    assert info["e_pos"] < 0.004 and info["e_ang_deg"] < 2.0
    assert worst > 0.0
    Tc = c.camera(q)
    assert Tc[2, 3] - (T_BT @ [0.05, -0.08, 0.08, 1])[2] == pytest.approx(0.15, abs=0.005)


def test_controller_stops_at_the_soft_box_and_only_looks():
    """A box like the lab's site limit (j1 >= -0.6) and a target past it: j1 stops at the edge
    (lookahead), nothing leaves the box, and the camera turns toward the target, at most max_tilt."""
    env = envm.Envelope()
    env.soft_lower[0] = -0.6
    q0 = look_pose((-0.10, -0.12), 0.08, 0.15, T_TC, T_BT)
    assert q0 is not None and -0.6 < q0[0] < -0.45
    c = FollowController(T_TC, T_BT, env.vel_caps, env.acc_caps, FollowConfig(), envelope=env)
    q = q0.copy()
    for _ in range(400):
        qd, info = c.step(q, [-0.10, -0.32, 0.08], 1 / 30)
        q = q + qd / 30
        assert q[0] >= -0.6 - 1e-6 and not env.check_point(q)
    assert info["box"] or abs(qd[0]) < 1e-9
    axis = c.camera(q)[:3, 2]
    tilt = np.degrees(np.arccos(-axis[2]))
    assert info["e_ang_deg"] < 3 and 10 < tilt < c.cfg.max_tilt_deg + 3    # turned toward it, within max_tilt


def test_controller_clamps_far_targets_to_the_reach_annulus():
    c = _ctrl()
    q0 = look_pose((0.0, -0.12), 0.08, 0.15, T_TC, T_BT)
    p_des, _, clamped = c.desired(c.table_to_base([0.6, -0.1, 0.08]), q0)
    assert clamped and np.hypot(*p_des[:2]) == pytest.approx(c.cfg.reach[1], abs=1e-6)


# --------------------------------------------------------------------------- the tracker
def test_tracker_follows_a_still_cup_and_refuses_a_featureless_patch():
    world = synth.World(cups=[synth.Cup((0.0, -0.12))], board=False)
    cam = synth.TRUE_CAMERA.scaled((640, 360))
    q = look_pose((0.0, -0.12), 0.08, 0.15, T_TC, T_BT)
    view = View.from_q(q, cam, T_TC, T_BT)
    img = world.render(cam, kin.fk_vision(q) @ T_TC, ss=1)
    det = CupFinder().find(img, view)
    assert det is not None
    trk = Tracker(TrackConfig())
    assert trk.anchor(det, img, view, 0.0) == "started"
    for k in range(1, 6):
        m = trk.update(world.render(cam, kin.fk_vision(q) @ T_TC, ss=1, seed=k), view, k / 15)
        assert m is not None and m.score > 0.8
    est = trk.estimate(5 / 15)
    assert est.state == "tracking" and np.hypot(*(est.xy - [0.0, -0.12])) < 0.002
    # a flat, featureless frame: no template, no track
    flat = np.full_like(img, 128)
    t2 = Tracker(TrackConfig())
    assert t2.anchor(det, flat, view, 0.0) == "ignored" and t2.estimate(0.0) is None


def test_tracker_counts_a_frame_gap_as_coasting_not_lost():
    world = synth.World(cups=[synth.Cup((0.0, -0.12))], board=False)
    cam = synth.TRUE_CAMERA.scaled((640, 360))
    q = look_pose((0.0, -0.12), 0.08, 0.15, T_TC, T_BT)
    view = View.from_q(q, cam, T_TC, T_BT)
    img = world.render(cam, kin.fk_vision(q) @ T_TC, ss=1)
    trk = Tracker(TrackConfig())
    trk.anchor(CupFinder().find(img, view), img, view, 0.0)
    trk.update(img, view, 0.05)
    assert trk.update(img, view, 0.6) is not None       # 0.55 s with no frames, then a match
    assert trk.state == "tracking"
    empty = synth.World(board=False).render(cam, kin.fk_vision(q) @ T_TC, ss=1)
    for k in range(6):                                  # the cup is gone: misses -> lost
        trk.update(empty, view, 0.7 + k * 0.07)
    assert trk.state == "lost" and trk.estimate(1.2) is None


# --------------------------------------------------------------------------- closed loop (virtual clock)
def test_follow_sim_still_cup():
    r = sim.simulate(sim.SimConfig(path="static", duration=5.0, settle=2.0))
    print("\nstatic:", {k: r[k] for k in ("centring_median_mm", "centring_p95_mm", "tracked_frac")})
    assert r["centring_median_mm"] < 2.0 and r["tracked_frac"] == 1.0
    assert not r["box_violations"] and r["min_margin"] > 0


def test_follow_sim_circling_cup():
    r = sim.simulate(sim.SimConfig(path="circle", speed=0.04, duration=8.0))
    print("\ncircle:", {k: r[k] for k in ("centring_median_mm", "centring_p95_mm", "est_err_median_mm", "tracked_frac")})
    assert r["tracked_frac"] > 0.97
    assert r["est_err_median_mm"] < 2.0
    assert r["centring_median_mm"] < 8.0 and r["centring_p95_mm"] < 15.0
    assert not r["box_violations"] and r["min_margin"] > 0
    assert max(r["max_qd"]) <= 0.3 * FollowConfig().speed_scale + 1e-6


def test_follow_sim_vanished_cup_stops_the_arm():
    r = sim.simulate(sim.SimConfig(path="vanish", duration=6.0, vanish_at=3.0))
    print("\nvanish:", r["moved_after_vanish_rad"], r["track"])
    assert r["track"]["lost"] >= 1
    assert r["moved_after_vanish_rad"] < 0.01 and r["final_qd"] < 1e-6


def test_follow_sim_jump_is_found_again():
    r = sim.simulate(sim.SimConfig(path="jump", duration=7.0, settle=5.0))
    print("\njump:", {k: r[k] for k in ("centring_median_mm", "tracked_frac")}, r["track"])
    assert r["centring_median_mm"] < 4.0 and r["tracked_frac"] > 0.9


def test_follow_sim_ignores_an_identical_cup_nearby():
    r = sim.simulate(sim.SimConfig(path="circle", speed=0.03, duration=8.0, distractor=(-0.02, -0.25)))
    print("\ndistractor:", {k: r[k] for k in ("centring_median_mm", "est_err_p95_mm", "tracked_frac")}, r["track"])
    assert r["est_err_p95_mm"] < 5.0 and r["track"]["resets"] == 0


# --------------------------------------------------------------------------- end to end: daemon (mock)
@contextlib.asynccontextmanager
async def mock_daemon():
    import uvicorn

    from armctl import daemon as D
    from armctl.backend import MockBackend
    from openplay import server as S

    backend = MockBackend(q0=list(envm.POSES["ready"]))
    daemon = D.Daemon(backend, auto_approve=True)
    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    server = S._uvicorn_server(uvicorn.Config(D.build_app(daemon), log_level="warning", lifespan="on"))
    task = asyncio.create_task(server.serve(sockets=[sock]))
    try:
        end = time.monotonic() + 10
        while not server.started and time.monotonic() < end:
            await asyncio.sleep(0.02)
        yield f"ws://127.0.0.1:{port}/armctl"
    finally:
        server.should_exit = True
        with contextlib.suppress(Exception):
            await asyncio.wait_for(task, 5)
        backend.close()
        sock.close()


def test_follow_program_end_to_end_on_the_mock_daemon(tmp_path):
    from interfaces.follow import run as R

    log = tmp_path / "run.jsonl"

    async def go():
        async with mock_daemon() as url:
            a = R.build_parser().parse_args([
                "--url", url, "--source", "synthetic", "--sim-path", "circle", "--sim-speed", "0.025",
                "--expect", "0.0,-0.12", "--duration", "8", "--v-max", "0.06", "--log", str(log), "--settle", "0.5"])
            return await R.run(a)

    assert asyncio.run(go()) == 0
    ev = [json.loads(l) for l in log.read_text().splitlines()]
    kinds = {e["event"] for e in ev}
    assert {"start", "look_move", "seed", "frame", "control", "end"} <= kinds
    end = ev[-1]
    assert end["event"] == "end" and not end["refusals"] and end["control_ticks_sent"] > 100
    frames = [e for e in ev if e["event"] == "frame" and e["match"] is not None and e["true"]]
    err = [np.hypot(e["match"][0] - e["true"][0], e["match"][1] - e["true"][1]) for e in frames[5:]]
    print(f"\ne2e: {len(frames)} matched frames, match error median {np.median(err) * 1000:.1f} mm; {end['track']}")
    assert len(frames) > 20 and np.median(err) < 0.01
    ctrl = [e for e in ev if e["event"] == "control"]
    assert max(abs(x) for e in ctrl for x in e["qd"]) <= 0.3 * 0.5 + 1e-6     # speed-scale 0.5 of the caps
