"""openplay.eventlog: clock sync, gap statistics, JSONL writing, stream-episode attribution,
the armctl wire tracer, the summarize CLI, and the server relaying the daemon's id-less
stream refusals. No arm, no network.

    .venv/bin/python -m pytest -q openplay/tests/test_eventlog.py
"""
from __future__ import annotations

import asyncio
import datetime as dt
import io
import json

import pytest

from openplay import eventlog as EL


# ---------------------------------------------------------------- clock sync
def _sample(offset_ms, up_ms, down_ms, srv_t1=1_000_000.0, hold_ms=0.5):
    """One ping/pong with the phone clock ``offset_ms`` ahead of the server."""
    t0 = srv_t1 - up_ms + offset_ms           # phone clock when it sent
    t2 = srv_t1 + hold_ms                     # server: received at t1, replied at t2
    t3 = t2 + down_ms + offset_ms             # phone clock when the pong arrived
    return t0, srv_t1, t2, t3


def test_clock_sync_symmetric_is_exact():
    cs = EL.ClockSync()
    est = cs.add(*_sample(5000.0, 20.0, 20.0))
    assert est["offset_ms"] == pytest.approx(5000.0, abs=0.05)
    assert est["rtt_ms"] == pytest.approx(40.0, abs=0.05)
    assert est["unc_ms"] == pytest.approx(20.0, abs=0.05)


def test_clock_sync_asymmetry_stays_within_the_uncertainty_and_min_rtt_wins():
    cs = EL.ClockSync()
    cs.add(*_sample(-3200.0, 180.0, 20.0, srv_t1=1e6))       # slow uplink: offset error 80 ms
    cs.add(*_sample(-3200.0, 12.0, 4.0, srv_t1=1e6 + 5000))   # quick sample: error 4 ms
    cs.add(*_sample(-3200.0, 300.0, 300.0, srv_t1=1e6 + 9000))
    est = cs.estimate()
    assert est["rtt_ms"] == pytest.approx(16.0, abs=0.05)     # the min-RTT sample is used
    assert abs(est["offset_ms"] - (-3200.0)) <= est["unc_ms"]
    assert est["n"] == 3
    # a phone time converts onto the server clock
    assert cs.to_server_s(-3200.0 + 2_000_000.0) == pytest.approx(2000.0, abs=0.01)


def test_clock_sync_rejects_nonsense():
    cs = EL.ClockSync()
    assert cs.add("x", 1, 2, 3) is None
    assert cs.add(0, 10, 5, 20) is None                       # server replied before it received
    assert cs.add(100, 0, 0, 0) is None                       # negative rtt (a clock jumped)
    assert cs.add(float("nan"), 0, 0, 0) is None
    assert cs.estimate() is None and cs.to_server_s(1.0) is None


def test_clock_sync_window_forgets_old_samples():
    cs = EL.ClockSync(window=3)
    cs.add(*_sample(100.0, 1.0, 1.0))                         # best, then pushed out
    for k in range(3):
        cs.add(*_sample(150.0, 30.0, 30.0, srv_t1=2e6 + k))
    assert cs.estimate()["offset_ms"] == pytest.approx(150.0, abs=0.05)


# ---------------------------------------------------------------- gap stats
def test_percentile_and_histogram():
    xs = [66, 66, 67, 65, 70, 400, 66, 66, 66, 1200]
    assert EL.percentile(xs, 50) == pytest.approx(66.0)
    assert EL.percentile([1, 2, 3, 4], 50) == pytest.approx(2.5)
    assert EL.percentile([], 50) is None
    g = EL.gap_stats(xs, 300)
    assert g["n"] == 10 and g["max"] == 1200 and g["over_ttl"] == 2
    assert g["p95"] > 400
    h = g["hist"]
    assert h["50-100"] == 8 and h["400-500"] == 1 and h[">=1000"] == 1 and sum(h.values()) == 10


# ---------------------------------------------------------------- JSONL + ring
def test_eventlog_writes_json_lines_and_ring(tmp_path):
    p = tmp_path / "sub" / "events.jsonl"
    t = [1000.0]
    lg = EL.EventLog(p, ttl_ms=300, ring=5, clock=lambda: t[0])
    for k in range(7):
        t[0] += 0.1
        lg.emit("phone", "rx", cid="p1", pid="a" if k % 2 else "b", op="ping", seq=k, none_field=None)
    lg.close()
    lines = [json.loads(x) for x in p.read_text().splitlines()]
    assert [r["n"] for r in lines] == list(range(1, 8))
    assert all(r["lane"] == "phone" and r["kind"] == "rx" and "none_field" not in r for r in lines)
    assert lines[0]["t"] == pytest.approx(1000.1)
    assert len(lg.ring) == 5                              # bounded
    assert [r["seq"] for r in lg.since(4)] == [4, 5, 6]
    assert all(r["pid"] == "a" for r in lg.since(0, pid="a"))
    assert lg.since(0, op="nope") == []


def test_eventlog_unwritable_path_falls_back_to_the_ring(tmp_path):
    blocker = tmp_path / "file"
    blocker.write_text("x")
    lg = EL.EventLog(blocker / "events.jsonl")          # parent is a file: can't create
    lg.emit("server", "config", ttl_ms=300)
    assert len(lg.ring) == 1


def test_default_path_prefers_an_openplay_folder(tmp_path):
    now = dt.datetime(2026, 10, 29, 14, 5, 9)
    assert EL.default_path(now, tmp_path) == tmp_path / "2026-10-29-openplay/logs/openplay-events-20261029-140509.jsonl"
    (tmp_path / "2026-10-29-first-pick").mkdir()
    assert EL.default_path(now, tmp_path).parent.parent.name == "2026-10-29-first-pick"
    (tmp_path / "2026-10-29-openplay-gaps").mkdir()
    assert EL.default_path(now, tmp_path).parent.parent.name == "2026-10-29-openplay-gaps"


# ---------------------------------------------------------------- episodes
class Feed:
    """Builds records like the server does, on a fake clock."""

    def __init__(self, ttl_ms=300):
        self.t = 5000.0
        self.lg = EL.EventLog(None, ttl_ms=ttl_ms, clock=lambda: self.t)
        self.seq = 0
        self.tph = 9_000_000.0

    def stream(self, after_ms, phone_gap_ms=None, zero=False, ep=1, why=None, skip=None, act="forward", reason=None):
        self.t += after_ms / 1000.0
        self.tph += after_ms if phone_gap_ms is None else phone_gap_ms
        self.seq += 1
        return self.lg.emit("phone", "rx", cid="p1", pid="ana", op="stream", seq=self.seq, t_ph=self.tph,
                            ep=ep, zero=zero, why=why, skip=skip, act=act, reason=reason,
                            sum={"v": {} if zero else {"x": 0.3}})

    def stop(self, after_ms, cause="expired", after_zero=False):
        self.t += after_ms / 1000.0
        return self.lg.emit("arm", "stream_stop", cause=cause, after_zero=after_zero)


def test_episode_attributes_gaps_to_phone_network_and_gate():
    f = Feed()
    f.stream(0)
    for _ in range(5):
        f.stream(66)
    f.stop(330)                                    # deadman while the phone was silent
    f.stream(270, phone_gap_ms=600)                # gap 600 ms, phone sent 600 ms apart: phone
    f.stream(66)
    f.stop(330)
    f.stream(270, phone_gap_ms=66)                 # gap 600 ms, sent on time: network
    f.stream(66)
    f.stream(500, skip=7)                          # the tilt gate held 7 ticks back: phone-gate, no deadman
    f.stream(66, zero=True, why="up")
    eps = list(f.lg.tracker.done)
    assert len(eps) == 1
    e = eps[0]
    assert e["end"] == "release (up)" and e["n"] == 12
    causes = [(g["cause"], g["deadman"]) for g in e["gaps_over_ttl"]]
    assert causes == [("phone", True), ("network", True), ("phone-gate", False)]
    assert e["deadman_in_gaps"] == 2 and e["arrival"]["over_ttl"] == 3
    assert e["arrival"]["max"] == pytest.approx(600.0, abs=0.5)
    assert e["sent"]["max"] == pytest.approx(600.0, abs=0.5)
    # the expiry after a release is the benign "deadman" line the daemon logs on every lift
    f.stop(300, after_zero=True)
    t = f.lg.tracker.totals
    assert t["deadman_after_release"] == 1 and t["deadman_mid_episode"] == 2
    # the episode record went into the log too
    assert [r for r in f.lg.ring if r["kind"] == "episode"][0]["episode"] == e["episode"]


def test_episode_boundaries_new_hold_timeout_disconnect_and_refusals():
    f = Feed()
    f.stream(0, ep=1)
    f.stream(66, ep=1, act="refuse", reason="stream v: Cartesian streaming needs the arm unfolded (j2 0.2 < 0.3)")
    f.stream(66, ep=1, act="drop", reason="stream v: Cartesian streaming needs the arm unfolded (j2 0.2 < 0.3)")
    f.stream(66, ep=2)                                         # a new hold without a release
    f.t += 3.0
    f.lg.tick()                                                # 2 s silence: timeout
    f.stream(10, ep=3)
    f.lg.emit("conn", "close", cid="p1", code=1001)
    ends = [e["end"] for e in f.lg.tracker.done]
    assert ends == ["new hold", "timeout (no release)", "disconnect"]
    first = f.lg.tracker.done[0]
    assert first["refusals"] == {"needs the arm unfolded (j2/j3 too low)": 2}
    assert first["acts"] == {"forward": 1, "refuse": 1, "drop": 1}
    # a stray release outside a hold is counted, not an episode
    f.stream(10, zero=True, ep=3)
    assert f.lg.tracker.totals["zero_outside_episode"] == 1


def test_episode_without_ep_splits_on_long_gaps_and_old_pages_have_unknown_cause():
    lg = EL.EventLog(None, ttl_ms=300, clock=lambda: 0.0)
    for t in (10.0, 10.066, 10.5, 12.0):                       # no t_ph, no ep (an old phone page)
        lg.emit("phone", "rx", t=t, cid="p9", pid="old", op="stream", act="forward", sum={"v": {"x": 1}})
    lg.tracker.close_all()
    a, b = lg.tracker.done
    assert a["n"] == 3 and b["n"] == 1
    assert [g["cause"] for g in a["gaps_over_ttl"]] == ["unknown"]


# ---------------------------------------------------------------- the armctl wire
def test_arm_tracer_latency_state_sampling_and_stream_stop_causes():
    now = [100.0]
    recs = []
    tr = EL.ArmTracer(lambda lane, kind, **kw: recs.append({"kind": kind, **kw}), clock=lambda: now[0])
    tr("tx", {"op": "do", "cmd": "ready", "id": 7})
    now[0] += 0.012
    tr("rx", {"op": "ack", "id": 7})
    now[0] += 2.0
    tr("rx", {"op": "done", "id": 7, "cmd": "ready"})
    ack = next(r for r in recs if r.get("op") == "ack")
    done = next(r for r in recs if r.get("op") == "done")
    assert ack["ack_ms"] == pytest.approx(12.0) and done["done_ms"] == pytest.approx(2012.0) and done["req"] == "ready"
    assert 7 not in tr.sent

    def last_stop():
        return next(r for r in reversed(recs) if r["kind"] == "stream_stop")

    def state(streaming, dt_s=0.05):
        now[0] += dt_s
        tr("rx", {"op": "state", "q": [0, 0.5, 0.8, 0, 0, 0], "mode": "velocity", "streaming": streaming})

    n0 = len(recs)
    tr("tx", {"op": "stream", "v": {"x": 0.02}, "ttl_ms": 300})
    for _ in range(4):
        state(True)
    state(False)                                               # ttl ran out: a deadman
    stop = last_stop()
    assert stop["kind"] == "stream_stop" and stop["cause"] == "expired" and stop["after_zero"] is False
    assert sum(1 for r in recs[n0:] if r["kind"] == "state") <= 1  # 2 Hz samples, not 20 Hz
    tr("tx", {"op": "stream", "v": {"x": 0.0}, "ttl_ms": 300})
    state(True)
    state(False, 0.35)
    assert last_stop()["cause"] == "expired" and last_stop()["after_zero"] is True
    tr("tx", {"op": "stream", "v": {"x": 0.02}, "ttl_ms": 300})
    state(True)
    tr("rx", {"op": "refused", "id": None, "reason": "edge of reach: j3", "stream": True, "edge": True})
    state(False)
    assert last_stop()["cause"] == "refused" and "edge" in last_stop()["why"]
    tr("tx", {"op": "stream", "v": {"x": 0.02}, "ttl_ms": 300})
    state(True, 1.0)
    tr("tx", {"op": "do", "cmd": "stop"})
    state(False)
    assert last_stop()["cause"] == "do" and last_stop()["why"] == "stop"


# ---------------------------------------------------------------- summarize CLI
def test_summarize_cli_replays_a_file(tmp_path, capsys):
    p = tmp_path / "ev.jsonl"
    f = Feed()
    f.lg = EL.EventLog(p, ttl_ms=300, clock=lambda: f.t)
    f.lg.emit("server", "config", ttl_ms=300)
    f.lg.emit("conn", "sync", pid="ana", offset_ms=5003.1, unc_ms=6.0, rtt_ms=12.0, n=5)
    f.stream(0)
    f.stream(66)
    f.stop(330)
    f.stream(270, phone_gap_ms=66)
    f.stream(66, zero=True, why="hidden")
    f.lg.emit("phone", "diag", pid="ana", ev="hidden")
    f.lg.close()
    assert EL.main(["summarize", str(p)]) == 0
    out = capsys.readouterr().out
    assert "network 1" in out and "1 became deadman stops" in out and "release (hidden)" in out
    assert "offset 5003.1 ms" in out and "hidden 1" in out
    s = EL.summarize(EL.read_records(p))
    assert len(s["episodes"]) == 1 and s["totals"]["deadman_in_gaps"] == 1
    buf = io.StringIO()
    EL.print_summary(EL.summarize([]), out=buf)
    assert "no streaming episodes" in buf.getvalue()


# ---------------------------------------------------------------- server: relay the daemon's id-less stream refusals
def test_server_relays_unfolded_refusal_to_the_driver_once_a_second():
    from openplay.server import OpenPlay

    class C:
        def __init__(self):
            self.msgs, self.pid, self.alive, self.last_stream_refusal = [], None, True, 0.0

        def send(self, m):
            self.msgs.append(m)

    async def go():
        op = OpenPlay(events_log=None)
        c = C()
        await op.phone_msg(c, {"op": "hello", "pid": "p-ana", "name": "Ana"})
        await op.phone_msg(c, {"op": "join"})
        op._lobby_events(op.lobby.tick(can_start=True))
        assert op.lobby.active.pid == "p-ana"
        r = {"op": "refused", "id": None, "reason": "stream v: Cartesian streaming needs the arm unfolded "
             "(j2 0.28 < 0.3): DimOS's IK has no solution from the folded pose; send do ready first"}
        op._arm_event("refused", r)
        op._arm_event("refused", r)
        got = [m for m in c.msgs if m.get("op") == "refused"]
        assert len(got) == 1 and got[0]["reason"].startswith("Cartesian streaming needs the arm unfolded")
        assert any(x["kind"] == "relay" for x in op.trace.ring)
        # ping/pong carries the clock-sync stamps; sync feeds the estimate and the trace
        await op.phone_msg(c, {"op": "ping", "t": 123456.0})
        pong = c.msgs[-1]
        assert pong["op"] == "pong" and pong["t0"] == 123456.0 and "t1" in pong
        await op.phone_msg(c, {"op": "sync", "s": [1000.0, 2000.0, 2001.0, 1011.0]})
        assert any(x["kind"] == "sync" and x["offset_ms"] == pytest.approx(-995.0) for x in op.trace.ring)
        await op.phone_msg(c, {"op": "diag", "events": [{"t": 1005.0, "ev": "hidden", "streaming": True}, "junk"]})
        d = [x for x in op.trace.ring if x["kind"] == "diag"]
        assert len(d) == 1 and d[0]["ev"] == "hidden" and d[0]["t_ph_srv"] == pytest.approx(2.0, abs=1e-3)

    asyncio.run(go())


def test_display_listener_serves_the_viewer_and_stats():
    from fastapi.testclient import TestClient

    from openplay.server import OpenPlay

    op = OpenPlay(events_log=None)
    op.trace.emit("phone", "rx", cid="p1", pid="ana", op="stream", act="forward")
    op.trace.emit("phone", "rx", cid="p1", pid="ben", op="move", act="start")
    c = TestClient(op.display_app())
    assert "Open play events" in c.get("/events").text
    j = c.get("/api/events", params={"since": 0, "pid": "ana"}).json()
    assert j["ttl_ms"] == 300 and [r["pid"] for r in j["events"]] == ["ana"]
    s = c.get("/api/streamstats").json()
    assert s["ttl_ms"] == 300 and "episodes" in s and "sync" in s
