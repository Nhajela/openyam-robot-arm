"""Daemon behaviour needed before the real arm: per-joint arrival tolerance, E-STOP reset clears
the backend latch, relax refused cleanly when a backend can't do it."""
from __future__ import annotations

import asyncio

import numpy as np

from armctl import envelope as env
from armctl.backend import MockBackend
from armctl.daemon import REACH_TOL_BY_BACKEND, Client, Daemon


class _Rec:
    def __init__(self):
        self.msgs = []

    async def __call__(self, m):
        self.msgs.append(m)


def test_reach_tolerance_per_backend():
    d = Daemon(MockBackend(env.DEFAULT), auto_approve=True)
    assert np.allclose(d.reach_tol, REACH_TOL_BY_BACKEND["mock"])
    real = REACH_TOL_BY_BACKEND["dimos"]
    assert real[1] >= 0.03 + 0.01 and real[3] >= 0.06 + 0.01   # 16 Oct sag: j2/j3 ~0.03, wrist ~0.06


def test_estop_reset_clears_backend_latch():
    b = MockBackend(env.DEFAULT)
    calls = []
    b.clear_estop = lambda: calls.append(1)
    d = Daemon(b, auto_approve=True)
    asyncio.run(d.estop_reset(by="test"))
    assert calls == [1] and not d.estopped


def test_relax_refused_cleanly():
    b = MockBackend(env.DEFAULT)

    def nope():
        raise RuntimeError("no torque-off RPC; run tools/disable_all.py with the arm supported")
    b.relax = nope
    d = Daemon(b, auto_approve=True)

    async def go():
        rec = _Rec()
        c = Client(rec, "test")
        await d.connect(c)
        await d.handle(c, {"op": "hello", "program": "t"})
        await d.handle(c, {"op": "acquire", "id": 1})
        await asyncio.sleep(0.05)
        await d.handle(c, {"op": "do", "cmd": "relax", "id": 2})
        return rec.msgs
    msgs = asyncio.run(go())
    ref = [m for m in msgs if m.get("op") == "refused" and m.get("id") == 2]
    assert ref and "relax:" in ref[0]["reason"] and "internal error" not in ref[0]["reason"]
