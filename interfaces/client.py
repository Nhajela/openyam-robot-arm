"""interfaces.client: a small asyncio client for the armctl daemon (protocol armctl/0).

    async with ArmClient(program="mirror", name="pose follow") as arm:
        await arm.acquire()                 # waits for the operator (or --auto-approve)
        await arm.do("ready")               # waits for done; raises Refused with the reason
        await arm.stream_qd([0.1, 0, 0, 0, 0, 0])
        print(arm.q)

Protocol reference: armctl/README.md. Nothing here bypasses the daemon's checks.
"""
from __future__ import annotations

import asyncio
import itertools
import json
import time
import urllib.request
from typing import Any

import numpy as np

DEFAULT_URL = "ws://127.0.0.1:8091/armctl"


class Refused(RuntimeError):
    """The daemon refused a request; ``str(e)`` is its reason."""


class ArmClient:
    def __init__(self, url: str = DEFAULT_URL, program: str = "script", name: str = ""):
        self.url, self.program, self.name = url, program, name or program
        self.state: dict[str, Any] = {}
        self.hello: dict[str, Any] = {}
        self.client_id: str | None = None
        self.holder: str | None = None
        self.refusals: list[dict] = []
        self._ids = itertools.count(1)
        self._waiters: dict[Any, asyncio.Future] = {}
        self._events: asyncio.Queue = asyncio.Queue()
        self._state_event = asyncio.Event()

    # ------------------------------------------------------------------ lifecycle
    async def __aenter__(self) -> "ArmClient":
        import websockets

        self.ws = await websockets.connect(self.url, max_size=None)
        self._rx_task = asyncio.create_task(self._rx())
        self.hello = await self.request({"op": "hello", "program": self.program, "name": self.name}, until="hello")
        self.client_id = self.hello["client"]
        self.holder = (self.hello.get("lease") or {}).get("client")
        return self

    async def __aexit__(self, *exc) -> None:
        try:
            if self.has_lease:
                await asyncio.wait_for(self.request({"op": "release"}, until="ack"), 3.0)
        except Exception:
            pass
        self._rx_task.cancel()
        await self.ws.close()

    @property
    def backend(self) -> str:
        return str(self.hello.get("backend", "?"))

    @property
    def envelope(self) -> dict:
        return self.hello.get("envelope", {})

    @property
    def has_lease(self) -> bool:
        return self.client_id is not None and self.holder == self.client_id

    @property
    def q(self) -> np.ndarray:
        return np.array(self.state.get("q", [np.nan] * 6), dtype=float)

    # ------------------------------------------------------------------ receive
    async def _rx(self) -> None:
        async for raw in self.ws:
            m = json.loads(raw)
            op = m.get("op")
            if op == "state":
                self.state = m
                self.holder = (m.get("lease") or {}).get("client")
                self._state_event.set()
                continue
            if op == "lease":
                self.holder = m.get("client")
            if op == "refused":
                self.refusals.append(m)
            fut = self._waiters.get(m.get("id"))
            if fut is not None and not fut.done():
                fut.set_result(m) if op != "refused" else fut.set_exception(Refused(m.get("reason", "refused")))
            await self._events.put(m)

    async def wait_state(self, timeout: float = 2.0) -> dict:
        self._state_event.clear()
        await asyncio.wait_for(self._state_event.wait(), timeout)
        return self.state

    async def send(self, msg: dict) -> None:
        await self.ws.send(json.dumps(msg))

    async def request(self, msg: dict, until: str = "ack", timeout: float = 10.0) -> dict:
        """Send with a fresh id; return the first reply with that id (raises Refused)."""
        mid = next(self._ids)
        fut = asyncio.get_running_loop().create_future()
        self._waiters[mid] = fut
        try:
            await self.send({**msg, "id": mid})
            m = await asyncio.wait_for(fut, timeout)
            if until != "ack" and m.get("op") != until:  # e.g. hello replies with op hello
                return await self._wait_op(until, mid, timeout)
            return m
        finally:
            self._waiters.pop(mid, None)

    async def _wait_op(self, op: str, mid: Any, timeout: float) -> dict:
        end = time.monotonic() + timeout
        while True:
            m = await asyncio.wait_for(self._events.get(), max(0.01, end - time.monotonic()))
            if m.get("id") == mid and m.get("op") == "refused":
                raise Refused(m.get("reason", "refused"))
            if m.get("op") == op and (mid is None or m.get("id") in (mid, None)):
                return m

    # ------------------------------------------------------------------ commands
    async def acquire(self, timeout: float = 120.0) -> None:
        """Ask for the lease; waits for the operator to approve in the console."""
        await self.request({"op": "acquire"})
        end = time.monotonic() + timeout
        while not self.has_lease:
            if time.monotonic() > end:
                raise TimeoutError("lease not granted (approve the program in the console, http://127.0.0.1:8090)")
            await asyncio.sleep(0.05)

    async def do(self, cmd: str, wait: bool = True, timeout: float = 60.0, **args: Any) -> dict:
        """``do`` a command (home, ready, goto, move_joints, nudge, gripper, stop, relax, save_pose).
        With ``wait``, returns the ``done`` message after execute-then-verify."""
        mid = next(self._ids)
        fut = asyncio.get_running_loop().create_future()
        self._waiters[mid] = fut
        try:
            await self.send({"op": "do", "cmd": cmd, "id": mid, **args})
            ack = await asyncio.wait_for(fut, 10.0)
            if not wait or cmd == "stop":
                return ack
            return await self._wait_op("done", mid, timeout)
        finally:
            self._waiters.pop(mid, None)

    async def stream_qd(self, qd, ttl_ms: int = 200, grip: float | None = None) -> None:
        msg: dict[str, Any] = {"op": "stream", "qd": [round(float(x), 5) for x in qd], "ttl_ms": ttl_ms}
        if grip is not None:
            msg["grip"] = round(float(grip), 3)
        await self.send(msg)

    async def stop(self) -> None:
        await self.send({"op": "do", "cmd": "stop"})

    async def validate(self, playable: dict) -> dict:
        return await self.request({"op": "playable.validate", "playable": playable}, until="playable.result")

    async def play(self, playable: dict, auto_confirm: bool = False, timeout: float = 90.0) -> dict:
        """Validate, ask to play, and wait for ``done``. The operator presses Play in the console;
        ``auto_confirm`` presses it for you, and only works against the mock backend."""
        r = await self.validate(playable)
        if not r.get("ok"):
            raise Refused("playable does not validate: " + "; ".join(r.get("violations", [])[:5]))
        mid = next(self._ids)
        fut = asyncio.get_running_loop().create_future()
        self._waiters[mid] = fut
        try:
            await self.send({"op": "playable.play", "name": r["name"], "id": mid})
            await asyncio.wait_for(fut, 10.0)
            if auto_confirm:
                self.operator("/playable/confirm", {"name": r["name"]})
            return await self._wait_op("done", mid, timeout)
        finally:
            self._waiters.pop(mid, None)

    def operator(self, path: str, body: dict | None = None) -> dict:
        """POST to the daemon's loopback operator API. Refuses unless the backend is mock:
        on the real arm a human presses the buttons in the console."""
        if self.backend != "mock":
            raise Refused(f"operator shortcuts are for the mock backend only (backend is {self.backend!r})")
        base = self.url.replace("ws://", "http://").rsplit("/armctl", 1)[0]
        req = urllib.request.Request(base + path, data=json.dumps(body or {}).encode(),
                                     headers={"Content-Type": "application/json"}, method="POST")
        with urllib.request.urlopen(req, timeout=5) as r:
            return json.loads(r.read() or b"{}")
