"""End to end, in one process: armctl Daemon on MockBackend (served on an ephemeral loopback
port), open play with its real ArmBridge, and a simulated phone with a clock 4.2 s ahead that
streams Tilt-style setpoints at 15 Hz with two 600 ms stalls (ttl 300 ms):

  hold 1: the phone's timer stalls (its ``t`` stamps jump too)   -> cause "phone"
  hold 2: the phone sends on time but the network holds 6 messages
          and delivers them in a burst (TCP head-of-line)        -> cause "network"

Both gaps must be detected, attributed, and matched to the daemon's deadman stop (seen as its
``streaming`` flag dropping); the release expiries after each lift must be counted as
harmless; the clock offset must be recovered within its uncertainty; and the JSONL file must
replay to the same answer through ``python -m openplay.eventlog summarize``.

Mock only, never hardware: no CAN, no DimOS, no zenoh, no fixed ports.

    .venv/bin/python -m pytest -q openplay/tests/test_stream_jitter_e2e.py     # about 10 s
"""
from __future__ import annotations

import asyncio
import contextlib
import json
import logging
import socket
import time

import pytest

from armctl import daemon as D
from armctl import envelope as env
from armctl.backend import MockBackend
from openplay import eventlog as EL
from openplay import server as S

PHONE_AHEAD_MS = 4200.0
RATE_S = 0.066
TTL_MS = 300.0
logging.getLogger("armctl").handlers.clear()


class SimConn(S.Conn):
    """A phone socket stand-in: what the server sends lands in ``msgs`` (wire-stamped)."""

    def __init__(self):
        super().__init__(None, "phone")
        self.msgs: list[dict] = []

    def send(self, msg: dict) -> None:
        self.msgs.append(self.wire(msg))


class Phone:
    def __init__(self, op: S.OpenPlay):
        self.op, self.c, self.seq = op, SimConn(), 0

    @staticmethod
    def now_ms() -> float:
        return time.time() * 1000.0 + PHONE_AHEAD_MS

    def stamp(self, m: dict, t: float | None = None) -> str:
        self.seq += 1
        return json.dumps({**m, "seq": self.seq, "t": self.now_ms() if t is None else t})

    async def deliver(self, text: str) -> None:
        await self.op.on_text(self.c, text, self.op.phone_msg)

    async def send(self, m: dict) -> None:
        await self.deliver(self.stamp(m))

    async def sync(self, up_s: float, down_s: float) -> None:
        t0 = self.now_ms()
        await asyncio.sleep(up_s)
        await self.deliver(self.stamp({"op": "ping"}, t0))
        pong = self.c.msgs[-1]
        assert pong["op"] == "pong" and pong["t0"] == pytest.approx(t0) and "t2" in pong
        await asyncio.sleep(down_s)
        await self.send({"op": "sync", "s": [pong["t0"], pong["t1"], pong["t2"], self.now_ms()]})

    async def hold(self, ep: int, before: int, after: int, stall: str) -> None:
        v = {"x": 0.3, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        for _ in range(before):
            await self.send({"op": "stream", "v": v, "ep": ep})
            await asyncio.sleep(RATE_S)
        if stall == "phone":          # the timer did not fire for 600 ms (page busy, throttled)
            await asyncio.sleep(0.6 - RATE_S)
        else:                         # sent on time, held up in the network, then a burst
            held = []
            for _ in range(6):
                held.append(self.stamp({"op": "stream", "v": v, "ep": ep}))
                await asyncio.sleep(0.6 / 6)
            for text in held:
                await self.deliver(text)
        for _ in range(after):
            await self.send({"op": "stream", "v": v, "ep": ep})
            await asyncio.sleep(RATE_S)
        await self.send({"op": "stream", "v": {k: 0.0 for k in v}, "ep": ep, "why": "up"})


async def wait_for(pred, timeout: float, what: str) -> None:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return
        await asyncio.sleep(0.02)
    raise AssertionError(f"timed out waiting for {what}")


@contextlib.asynccontextmanager
async def mock_daemon():
    import uvicorn

    backend = MockBackend(q0=list(env.POSES["ready"]))
    daemon = D.Daemon(backend, auto_approve=True)
    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))            # ephemeral port, loopback only
    port = sock.getsockname()[1]
    server = S._uvicorn_server(uvicorn.Config(D.build_app(daemon), log_level="warning", lifespan="on"))
    task = asyncio.create_task(server.serve(sockets=[sock]))
    try:
        await wait_for(lambda: server.started, 10, "the mock daemon")
        yield f"ws://127.0.0.1:{port}/armctl", daemon
    finally:
        server.should_exit = True
        with contextlib.suppress(Exception):
            await asyncio.wait_for(task, 5)
        backend.close()
        sock.close()


def test_jittery_phone_gaps_are_detected_and_attributed(tmp_path):
    trace = tmp_path / "openplay-events-test.jsonl"

    async def go():
        async with mock_daemon() as (url, daemon):
            op = S.OpenPlay(daemon_url=url, events_log=None, trace_log=trace, auto_play="on",
                            stream_ttl_ms=TTL_MS, chess_sensor="fake")
            await op.start()
            try:
                await wait_for(lambda: op.arm.ready and op.arm.state.get("q"), 10, "the lease")
                ph = Phone(op)
                await ph.send({"op": "hello", "pid": "p-jit", "name": "Jit", "rate_ms": 66})
                for k in range(5):  # asymmetric: 30 ms up, 5 ms down (offset error 12.5 ms <= rtt/2)
                    await ph.sync(0.03 if k % 2 else 0.01, 0.005)
                await ph.send({"op": "join"})
                await wait_for(lambda: op.lobby.active is not None and op.lobby.active.pid == "p-jit",
                               10, "the turn")
                await wait_for(lambda: not op.busy and not op.handover, 10, "the arm to be free")
                await ph.hold(1, before=10, after=5, stall="phone")
                await asyncio.sleep(0.6)    # the release expiry (harmless "deadman" line)
                await ph.hold(2, before=8, after=5, stall="network")
                await asyncio.sleep(0.6)
                await wait_for(lambda: len(op.trace.tracker.done) >= 2, 5, "two holds")
                stats = op.trace.stats()
            finally:
                await op.shutdown()
            # the daemon really did stop the arm twice mid-hold: its log says so
            dead = [line for line in daemon.recent if "deadman" in line]
            assert len(dead) >= 4 and all("300+ ms" in line for line in dead), dead
            return op, stats

    op, stats = asyncio.run(go())
    h1, h2 = stats["episodes"][:2]

    # clock sync: the phone runs 4.2 s ahead; recovered within the reported uncertainty
    est = op.syncs["p-jit"].estimate()
    assert abs(est["offset_ms"] - PHONE_AHEAD_MS) <= est["unc_ms"] + 3.0, est
    assert est["unc_ms"] < 40

    for h in (h1, h2):
        assert h["end"] == "release (up)" and h["acts"].get("forward", 0) == h["n"]
        assert h["arrival"]["p50"] < 120 and h["rate_hz"] > 8
        assert h["arrival"]["over_ttl"] == 1 and h["deadman_in_gaps"] == 1, h
        g = h["gaps_over_ttl"][0]
        assert 500 < g["gap_ms"] < 900 and g["deadman"] is True
    # who was late: the phone (it sent late) vs the network (sent on time, arrived late)
    assert h1["gaps_over_ttl"][0]["cause"] == "phone" and h1["sent"]["max"] > 500
    assert h2["gaps_over_ttl"][0]["cause"] == "network" and h2["sent"]["max"] < 200
    # each lift's ttl expiry is counted separately: harmless
    t = stats["totals"]
    assert t["deadman_mid_episode"] == 2 and t["deadman_after_release"] >= 2

    # one-way delay on the server clock: the burst shows up as ~0.5 s late messages
    recs = EL.read_records(trace)
    rx = [r for r in recs if r["kind"] == "rx" and r["lane"] == "phone" and r["op"] == "stream"]
    assert all(r.get("t_ph_srv") is not None for r in rx)
    assert max(r["owd_ms"] for r in rx) > 400 and min(r["owd_ms"] for r in rx) > -50
    assert {r["kind"] for r in recs} >= {"config", "hello", "sync", "rx", "tx", "state", "stream_stop", "episode", "event"}
    stops = [r for r in recs if r["kind"] == "stream_stop"]
    assert all(r["cause"] == "expired" for r in stops) and sum(1 for r in stops if not r["after_zero"]) == 2

    # the file replays to the same answer (episodes recomputed from raw records)
    s = EL.summarize(recs)
    EL.print_summary(s)  # shown with pytest -s
    assert [len(e["gaps_over_ttl"]) for e in s["episodes"]] == [1, 1]
    assert [e["gaps_over_ttl"][0]["cause"] for e in s["episodes"]] == ["phone", "network"]
    assert s["totals"]["deadman_in_gaps"] == 2
    assert "became deadman stops" in EL.verdict(s)
