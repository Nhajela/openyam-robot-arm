"""Daemon tests against MockBackend (real 100 Hz sim thread, real time).

Run: .venv/bin/python -m pytest armctl/tests
"""
import asyncio
import logging

import numpy as np
import pytest

from armctl import daemon as d
from armctl import envelope as env
from armctl.backend import MockBackend

READY = list(env.POSES["ready"])
logging.getLogger("armctl").handlers.clear()  # no file logging from tests


class Fake:
    """A connected program: records everything the daemon sends it."""

    def __init__(self, daemon, program):
        self.daemon = daemon
        self.program = program
        self.inbox = []
        self.client = d.Client(self._send, "test")

    async def _send(self, msg):
        self.inbox.append(msg)

    async def hello(self):
        await self.daemon.connect(self.client)
        await self.op("hello", program=self.program, name=f"{self.program} test")
        return self

    async def op(self, op, **kw):
        await self.daemon.handle(self.client, {"op": op, **kw})

    def last(self, op=None):
        ms = [m for m in self.inbox if op is None or m["op"] == op]
        return ms[-1] if ms else None

    def ops(self, op):
        return [m for m in self.inbox if m["op"] == op]


def run(coro_fn, **kw):
    async def main():
        backend = MockBackend(q0=kw.pop("q0", READY))
        daemon = d.Daemon(backend, **kw)
        await daemon.start()
        try:
            return await coro_fn(daemon, backend)
        finally:
            await daemon.shutdown()
            backend.close()
    return asyncio.run(main())


async def wait_for(pred, timeout=5.0):
    t = asyncio.get_running_loop().time() + timeout
    while asyncio.get_running_loop().time() < t:
        if pred():
            return True
        await asyncio.sleep(0.02)
    return False


def test_hello_required_and_reply():
    async def body(daemon, backend):
        a = Fake(daemon, "tilt")
        await daemon.connect(a.client)
        await a.op("acquire")
        assert a.last("refused")["reason"].startswith("say hello first")
        await a.op("hello", program="tilt", name="iPhone")
        h = a.last("hello")
        assert h["client"] == a.client.id and h["protocol"] == "armctl/0"
        assert h["envelope"]["soft_limits"]["j4"] == [-1.5581, 1.471]
    run(body)


def test_lease_needs_operator_and_refuses_non_holder():
    async def body(daemon, backend):
        a = await Fake(daemon, "tilt").hello()
        await a.op("acquire", id="1")
        assert a.last("ack")["pending"] is True
        assert daemon.holder is None and a.client.id in daemon.pending
        await a.op("do", id="2", cmd="nudge", joint="j1", delta=0.05)
        r = a.last("refused")
        assert r["id"] == "2" and "not the lease holder (holder: nobody)" in r["reason"]
        await daemon.grant(daemon.find_client(program="tilt"))
        assert daemon.holder is a.client and a.last("lease")["holder"] == "tilt"
        assert daemon.status()["pending"] == []
    run(body)


def test_switch_stops_then_grants():
    async def body(daemon, backend):
        a = await Fake(daemon, "joystick").hello()
        b = await Fake(daemon, "tilt").hello()
        await a.op("acquire")
        assert daemon.holder is a.client  # auto-approve
        await a.op("stream", qd=[0.3, 0, 0, 0, 0, 0], ttl_ms=500)
        await wait_for(lambda: backend.state()["qd"][0] > 0.25)
        await b.op("acquire")
        assert daemon.holder is b.client
        assert not backend.state()["moving"]  # stopped before granting
        await a.op("stream", qd=[0.3, 0, 0, 0, 0, 0])
        assert "not the lease holder (holder: tilt" in a.last("refused")["reason"]
        assert b.last("lease")["holder"] == "tilt"
    run(body, auto_approve=True)


def test_release_and_disconnect_free_lease():
    async def body(daemon, backend):
        a = await Fake(daemon, "tilt").hello()
        await a.op("acquire")
        await a.op("release", id="r")
        assert daemon.holder is None and a.last("ack")["cmd"] == "release"
        await a.op("acquire")
        assert daemon.holder is a.client
        await daemon.disconnect(a.client)
        assert daemon.holder is None
    run(body, auto_approve=True)


def test_out_of_envelope_refused_with_reason():
    async def body(daemon, backend):
        a = await Fake(daemon, "console").hello()
        await a.op("acquire")
        await a.op("do", id="m", cmd="move_joints", q=[0, 0.5, 0.8, 1.62, 0, 0])
        r = a.last("refused")
        assert r["id"] == "m"
        assert r["reason"] == "j4 target 1.620 > soft limit 1.471 (URDF 1.571 - margin 0.10, past URDF limit 1.571)"
        await a.op("do", id="n", cmd="nudge", joint="j2", delta=0.5)
        assert "exceeds 0.2" in a.last("refused")["reason"]
        await a.op("do", id="g", cmd="goto", name="nowhere")
        assert "unknown pose 'nowhere'" in a.last("refused")["reason"]
        await a.op("do", id="w", cmd="gripper", width=1.5)
        assert "outside 0 (closed) .. 1 (open)" in a.last("refused")["reason"]
        await a.op("do", id="x", cmd="dance")
        assert "unknown cmd 'dance'" in a.last("refused")["reason"]
        assert backend.state()["q"] == pytest.approx(READY, abs=1e-6)  # nothing moved
        assert len(daemon.status()["refusals"]) == 5
    run(body, auto_approve=True)


def test_do_nudge_ack_done_and_verify():
    async def body(daemon, backend):
        a = await Fake(daemon, "console").hello()
        await a.op("acquire")
        await a.op("do", id="n1", cmd="nudge", joint="j1", delta=0.05)
        ack = a.last("ack")
        assert ack["id"] == "n1" and ack["target"][0] == pytest.approx(0.05)
        await a.op("do", id="n2", cmd="nudge", joint="j1", delta=0.05)
        assert "busy" in a.last("refused")["reason"]
        assert await wait_for(lambda: a.last("done") is not None, 5)
        assert a.last("done")["id"] == "n1"
        assert backend.state()["q"][0] == pytest.approx(0.05, abs=1e-3)
    run(body, auto_approve=True)


def test_stream_deadman_stops_after_ttl():
    async def body(daemon, backend):
        a = await Fake(daemon, "joystick").hello()
        await a.op("acquire")
        await a.op("stream", qd=[0.2, 0, 0, 0, 0, 0], ttl_ms=200)
        assert daemon.streaming
        await asyncio.sleep(0.15)
        assert backend.state()["qd"][0] > 0.1
        # stop sending: within ttl (0.2 s) + braking (0.2 rad/s / 1 rad/s^2 = 0.2 s) it is still
        await asyncio.sleep(0.6)
        st = backend.state()
        assert st["qd"][0] == 0 and not st["moving"] and st["mode"] == "hold"
        assert not daemon.streaming
        q1 = st["q"][0]
        await asyncio.sleep(0.3)
        assert backend.state()["q"][0] == q1
    run(body, auto_approve=True)


def test_stream_ttl_is_capped_at_500ms():
    async def body(daemon, backend):
        a = await Fake(daemon, "joystick").hello()
        await a.op("acquire")
        await a.op("stream", v={"y": 0.05}, ttl_ms=10000)
        await asyncio.sleep(0.5 + 0.35 + 0.1)  # max ttl + braking + slack
        assert not backend.state()["moving"]
    run(body, auto_approve=True)


def test_stream_guard_slides_along_boundary():
    async def body(daemon, backend):
        a = await Fake(daemon, "joystick").hello()
        await a.op("acquire")
        for _ in range(40):  # 2 s of j4 + j1 at 20 Hz, starting 0.05 rad from j4's soft limit
            await a.op("stream", qd=[0.1, 0, 0, 0.3, 0, 0], ttl_ms=200)
            await asyncio.sleep(0.05)
        st = backend.state()
        assert st["q"][3] <= env.DEFAULT.soft_upper[3] + 1e-6
        assert st["q"][0] > 0.1  # j1 kept moving
        r = [m for m in a.ops("refused") if m.get("stream")]
        assert r and r[0]["joint"] == "j4" and "soft limit 1.471" in r[0]["reason"]
        assert len(r) <= 3  # rate-limited
    run(body, auto_approve=True, q0=[0, 0.5, 0.8, 1.42, 0, 0])


def test_estop_latches_and_overrides_lease():
    async def body(daemon, backend):
        a = await Fake(daemon, "tilt").hello()
        await a.op("acquire")
        await a.op("stream", qd=[0.3, 0, 0, 0, 0, 0], ttl_ms=500)
        await asyncio.sleep(0.2)
        await daemon.estop()
        assert not backend.state()["moving"] and daemon.holder is None
        await a.op("acquire")
        assert "E-STOP is latched" in a.last("refused")["reason"]
        await daemon.estop_reset()
        await a.op("acquire")
        assert daemon.holder is a.client
    run(body, auto_approve=True)


def test_relax_then_motion_refused_until_enable():
    async def body(daemon, backend):
        a = await Fake(daemon, "console").hello()
        await a.op("acquire")
        await a.op("do", id="r", cmd="relax")
        assert "support the arm" in a.last("ack")["warning"]
        await a.op("do", id="h", cmd="home")
        assert "torque is off" in a.last("refused")["reason"]
        await daemon.enable()
        await a.op("do", id="h2", cmd="ready")
        assert a.last("ack")["id"] == "h2"
    run(body, auto_approve=True)


def test_home_is_allowed_and_slow():
    async def body(daemon, backend):
        a = await Fake(daemon, "console").hello()
        await a.op("acquire")
        await a.op("do", id="h", cmd="home")
        ack = a.last("ack")
        assert ack["id"] == "h" and ack["target"] == [0, 0, 0, 0, 0, 0] and ack["duration"] > 4
        await a.op("do", id="s", cmd="stop")
        assert a.last("refused")["id"] == "h"  # interrupted
    run(body, auto_approve=True)


def test_playable_flow():
    pl = pytest.importorskip("armctl.playable")
    playable = {"name": "nod", "space": "joint", "start": "ready", "speed": 1.0, "loop": 1,
                "keyframes": [{"t": 0.0, "q": READY}, {"t": 1.5, "q": [0, 0.5, 0.8, 0.2, 0, 0], "ease": "cubic"},
                              {"t": 3.0, "q": READY, "ease": "cubic"}]}
    assert pl.validate(playable)["ok"], pl.validate(playable)

    async def body(daemon, backend):
        a = await Fake(daemon, "llm").hello()
        await a.op("playable.play", id="p0", name="nod")
        assert "not the lease holder" in a.last("refused")["reason"]
        await a.op("acquire")
        await a.op("playable.play", id="p1", name="nod")
        assert "not validated" in a.last("refused")["reason"]
        await a.op("playable.validate", id="v", playable=playable)
        assert a.last("playable.result")["ok"] is True
        bad = {**playable, "name": "bad", "keyframes": [{"t": 0.0, "q": READY}, {"t": 1.0, "q": [0, 0.5, 0.8, 1.6, 0, 0]}]}
        await a.op("playable.validate", id="vb", playable=bad)
        res = a.last("playable.result")
        assert res["ok"] is False and any("j4" in v for v in res["violations"])
        await a.op("playable.play", id="p2", name="nod")
        assert a.last("ack")["pending"] is True and daemon.pending_play["name"] == "nod"
        assert not backend.state()["moving"]  # nothing moves before the operator confirms
        await daemon.confirm_play("nod")
        assert a.last("ack")["playing"] == "nod"
        assert await wait_for(lambda: backend.state()["q"][3] > 0.1, 3)
        assert await wait_for(lambda: a.last("done") is not None, 8)
        assert backend.state()["q"] == pytest.approx(READY, abs=0.02)
    run(body, auto_approve=True)


def test_playables_missing_is_refused(monkeypatch):
    monkeypatch.setattr(d, "_playable_module", lambda: None)

    async def body(daemon, backend):
        a = await Fake(daemon, "llm").hello()
        await a.op("playable.validate", id="v", playable={"name": "x"})
        assert a.last("refused")["reason"].startswith("playables not available yet")
    run(body)
