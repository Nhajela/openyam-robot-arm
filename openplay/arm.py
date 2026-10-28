"""openplay.arm: the one armctl connection open play holds (program "openplay").

    arm = ArmBridge("ws://127.0.0.1:8091/armctl", on_event=print)
    task = asyncio.create_task(arm.run())       # connect, acquire, reconnect forever
    await arm.stream(v={"x": 0.5}, grip=None)   # -1..1, scaled to the configured speeds
    await arm.do("ready")                        # waits for done; raises Refused
    await arm.play(playable, confirm=True)       # validate, play, press Play, wait for done

Why its own client and not ``interfaces.client.ArmClient``: open play runs for hours and
mixes 20 Hz streams with discrete actions and operator events, so replies are routed per
request id (no shared event queue to drain) and the connection heals itself: if the daemon
goes away, the lease is revoked or E-STOP latches, ``run()`` reconnects and re-acquires
with backoff. Nothing here bypasses the daemon: envelope gate, deadman and E-STOP apply.

Play confirmation: the daemon's Play button is ``POST /playable/confirm`` on its loopback
operator API. ``play(confirm=True)`` presses it; the server passes ``confirm=True`` only
when the operator has chosen auto-play, or has just approved that playable on the loopback
display. ``ArmClient.operator()`` still refuses non-mock backends; this does not touch it.
"""
from __future__ import annotations

import asyncio
import contextlib
import itertools
import json
import logging
import time
import urllib.error
import urllib.request
from typing import Any, Callable

log = logging.getLogger("openplay.arm")

DEFAULT_URL = "ws://127.0.0.1:8091/armctl"
LIN_MPS, ANG_RPS, JOINT_RPS = 0.08, 0.6, 0.5
STREAM_TTL_MS = 300
AXES_LIN, AXES_ANG = ("x", "y", "z"), ("roll", "pitch", "yaw")


class Refused(RuntimeError):
    """The daemon (or the bridge) refused; ``str(e)`` is the reason."""


def _clip(x: Any) -> float:
    try:
        f = float(x)
    except (TypeError, ValueError):
        return 0.0
    return 0.0 if f != f else max(-1.0, min(1.0, f))  # NaN -> 0


def scale_v(v: dict, lin_mps: float = LIN_MPS, ang_rps: float = ANG_RPS) -> dict[str, float]:
    """Normalized twist (-1..1 per axis) -> m/s and rad/s."""
    out = {k: round(_clip(v.get(k, 0)) * lin_mps, 5) for k in AXES_LIN}
    out.update({k: round(_clip(v.get(k, 0)) * ang_rps, 5) for k in AXES_ANG})
    return out


def scale_qd(qd: list, joint_rps: float = JOINT_RPS) -> list[float]:
    """Normalized joint velocities (-1..1, 6 of them) -> rad/s."""
    if not isinstance(qd, (list, tuple)) or len(qd) != 6:
        raise ValueError("qd needs 6 numbers in -1..1")
    return [round(_clip(x) * joint_rps, 5) for x in qd]


class ArmBridge:
    def __init__(self, url: str = DEFAULT_URL, *, name: str = "open play",
                 on_event: Callable[[str, dict], Any] | None = None,
                 lin_mps: float = LIN_MPS, ang_rps: float = ANG_RPS, joint_rps: float = JOINT_RPS):
        self.url, self.name = url, name
        self.http = url.replace("wss://", "https://").replace("ws://", "http://").rsplit("/armctl", 1)[0]
        self.on_event = on_event or (lambda kind, data: None)
        self.lin_mps, self.ang_rps, self.joint_rps = lin_mps, ang_rps, joint_rps
        self.state: dict[str, Any] = {}
        self.hello: dict[str, Any] = {}
        self.client_id: str | None = None
        self.holder: str | None = None
        self.estop = False
        self.fault: dict | None = None  # armctl state.fault: latched DimOS adapter fault (arm limp)
        self.connected = False
        self.waiting_approval = False
        self.ws = None
        self._ids = itertools.count(1)
        self._replies: dict[int, asyncio.Queue] = {}
        self._lost = asyncio.Event()
        self._stop = False
        self._denied = False

    # ------------------------------------------------------------------ status
    @property
    def has_lease(self) -> bool:
        return self.connected and self.client_id is not None and self.holder == self.client_id

    @property
    def backend(self) -> str | None:
        return self.hello.get("backend")

    @property
    def adapter(self) -> str | None:
        """"sim" | "real" from the daemon's state (DimosBackend), "mock" for the mock backend."""
        if self.backend == "mock":
            return "mock"
        return self.state.get("adapter") or self.hello.get("adapter")

    @property
    def ready(self) -> bool:
        return self.has_lease and not self.estop

    def unavailable_reason(self) -> str | None:
        if self.fault:
            return "ARM FAULTED: support the arm, run tools/disable_all.py"
        if self.estop:
            return "E-STOP is on"
        if not self.connected:
            return "the arm is not connected (waiting for the armctl daemon)"
        if not self.has_lease:
            return ("waiting for the operator to approve open play in the console"
                    if self.waiting_approval else "open play does not hold the arm right now")
        return None

    def speeds(self) -> dict[str, float]:
        return {"lin_mps": self.lin_mps, "ang_rps": self.ang_rps, "joint_rps": self.joint_rps}

    # ------------------------------------------------------------------ connection loop
    async def run(self) -> None:
        backoff = 1.0
        while not self._stop:
            try:
                await self._session()
                backoff = 1.0
            except asyncio.CancelledError:
                raise
            except Exception as e:
                log.info("armctl connection: %s: %s", type(e).__name__, e)
            self.connected = False
            self._fail_waiters("armctl connection lost")
            self.on_event("disconnected", {})
            if self._stop:
                break
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, 10.0)

    async def close(self) -> None:
        self._stop = True
        if self.ws is not None:
            with contextlib.suppress(Exception):
                if self.has_lease:
                    await asyncio.wait_for(self._request({"op": "release"}), 2.0)
            with contextlib.suppress(Exception):
                await self.ws.close()

    async def _session(self) -> None:
        import websockets

        self._lost.clear()
        async with websockets.connect(self.url, max_size=None, open_timeout=5) as ws:
            self.ws = ws
            rx = asyncio.create_task(self._rx(ws))
            try:
                h = await self._request({"op": "hello", "program": "openplay", "name": self.name}, until="hello")
                self.hello, self.client_id = h, h["client"]
                self.holder = (h.get("lease") or {}).get("client")
                self.estop = bool(h.get("estop"))
                self.fault = h.get("fault") or None
                self.connected = True
                self.on_event("connected", {"backend": self.backend})
                retry, first = 1.0, True
                while not rx.done():
                    if not self.has_lease and not self.estop:
                        if not first:  # lost it (revoked, E-STOP reset): don't fight the operator
                            await self._wait_or_lost(2.0)
                            if rx.done() or self.has_lease or self.estop:
                                continue
                        first = False
                        try:
                            await self._acquire()
                            retry = 1.0
                        except Refused as e:
                            log.info("acquire refused: %s", e)
                            await self._wait_or_lost(retry)
                            retry = min(retry * 2, 10.0)
                        continue
                    await self._wait_or_lost(0.5)
            finally:
                rx.cancel()
                with contextlib.suppress(asyncio.CancelledError, Exception):
                    await rx
                self.ws = None

    async def _wait_or_lost(self, s: float) -> None:
        with contextlib.suppress(asyncio.TimeoutError):
            await asyncio.wait_for(self._lost.wait(), s)

    async def _acquire(self) -> None:
        ack = await self._request({"op": "acquire"})
        self.waiting_approval = bool(ack.get("pending"))
        if self.waiting_approval:
            self.on_event("waiting_approval", {})
        self._denied = False
        while not self.has_lease and self.connected and not self.estop:
            await self._wait_or_lost(0.1)
            if self._lost.is_set():
                return
            if self._denied:  # the operator said no: ask again later, not in a loop
                self.waiting_approval = False
                self.on_event("denied", {})
                await self._wait_or_lost(30.0)
                return
        self.waiting_approval = False

    async def _rx(self, ws) -> None:
        try:
            async for raw in ws:
                try:
                    m = json.loads(raw)
                except ValueError:
                    continue
                op = m.get("op")
                if op == "state":
                    self.state = m
                    self.holder = (m.get("lease") or {}).get("client")
                    self.estop = bool(m.get("estop"))
                    f = m.get("fault") or None
                    if f and not self.fault:
                        self.on_event("fault", f)
                    self.fault = f
                    continue
                if op == "lease":
                    had = self.has_lease
                    self.holder = m.get("client")
                    if had != self.has_lease:
                        self.on_event("lease", {"ours": self.has_lease})
                elif op == "estop":
                    self.estop = bool(m.get("latched"))
                    self.on_event("estop", {"latched": self.estop})
                elif op == "fault":
                    if not self.fault:
                        self.fault = {k: v for k, v in m.items() if k != "op"}
                        self.on_event("fault", self.fault)
                q = self._replies.get(m.get("id")) if m.get("id") is not None else None
                if q is not None:
                    q.put_nowait(m)
                elif op == "refused":
                    if "denied the lease" in str(m.get("reason")):
                        self._denied = True
                    self.on_event("refused", m)  # stream guard refusals etc.
        finally:
            self.connected = False
            self._lost.set()

    def _fail_waiters(self, reason: str) -> None:
        for q in self._replies.values():
            q.put_nowait({"op": "refused", "reason": reason})

    # ------------------------------------------------------------------ requests
    async def _send(self, msg: dict) -> None:
        if self.ws is None:
            raise Refused(self.unavailable_reason() or "the arm is not connected")
        try:
            await self.ws.send(json.dumps(msg))
        except Exception as e:
            raise Refused(f"armctl connection lost ({type(e).__name__})") from None

    async def _request(self, msg: dict, until: str | None = None, timeout: float = 10.0,
                       then: str | None = None, then_timeout: float = 60.0,
                       between: Callable[[dict], Any] | None = None) -> dict:
        """Send with a fresh id. Returns the first reply (or the one with op ``until``).
        With ``then``, waits for a later reply with that op (e.g. ``done``) after calling
        ``between(first_reply)``. A ``refused`` with our id raises Refused."""
        mid = next(self._ids)
        q: asyncio.Queue = asyncio.Queue()
        self._replies[mid] = q
        try:
            await self._send({**msg, "id": mid})
            first = await self._next(q, until, timeout)
            if then is None:
                return first
            if between is not None:
                r = between(first)
                if asyncio.iscoroutine(r):
                    await r
            return await self._next(q, then, then_timeout)
        finally:
            self._replies.pop(mid, None)

    async def _next(self, q: asyncio.Queue, op: str | None, timeout: float) -> dict:
        end = time.monotonic() + timeout
        while True:
            left = end - time.monotonic()
            if left <= 0:
                raise Refused("the arm did not answer in time")
            try:
                m = await asyncio.wait_for(q.get(), left)
            except asyncio.TimeoutError:
                raise Refused("the arm did not answer in time") from None
            if m.get("op") == "refused":
                raise Refused(m.get("reason", "refused"))
            if op is None or m.get("op") == op:
                return m

    def _need(self) -> None:
        r = self.unavailable_reason()
        if r:
            raise Refused(r)

    # ------------------------------------------------------------------ commands
    async def stream(self, v: dict | None = None, qd: list | None = None, grip: float | None = None) -> None:
        """One live setpoint (normalized -1..1). The daemon's deadman stops the arm 300 ms
        after the last one."""
        self._need()
        msg: dict[str, Any] = {"op": "stream", "ttl_ms": STREAM_TTL_MS}
        if qd is not None:
            msg["qd"] = scale_qd(qd, self.joint_rps)
        else:
            msg["v"] = scale_v(v if isinstance(v, dict) else {}, self.lin_mps, self.ang_rps)
        if grip is not None:
            msg["grip"] = round(min(max(float(grip), 0.0), 1.0), 3)
        await self._send(msg)

    async def do(self, cmd: str, wait: bool = True, timeout: float = 60.0, **args: Any) -> dict:
        """A daemon ``do`` (ready, move_joints, gripper, ...). With ``wait`` returns ``done``."""
        if cmd != "stop":
            self._need()
        msg = {"op": "do", "cmd": cmd, **args}
        if not wait or cmd in ("stop", "gripper"):
            return await self._request(msg)
        return await self._request(msg, then="done", then_timeout=timeout)

    async def stop(self) -> None:
        if self.ws is not None:
            with contextlib.suppress(Refused):
                await self._send({"op": "do", "cmd": "stop"})

    async def validate(self, playable: dict) -> dict:
        self._need()
        return await self._request({"op": "playable.validate", "playable": playable}, until="playable.result")

    async def play(self, playable: dict, confirm: bool, timeout: float = 90.0) -> dict:
        """Validate, ask to play, press Play if ``confirm`` (see module doc), wait for done.
        Without ``confirm`` the daemon waits for Play in the armctl console."""
        r = await self.validate(playable)
        if not r.get("ok"):
            raise Refused("does not pass the safety check: " + "; ".join(r.get("violations", [])[:3]))
        name = r["name"]

        async def press(ack: dict) -> None:
            if confirm and ack.get("pending"):
                await self.confirm_play(name)

        return await self._request({"op": "playable.play", "name": name}, then="done",
                                   then_timeout=timeout, between=press)

    # ------------------------------------------------------------------ operator API (loopback)
    def _post(self, path: str, body: dict | None = None) -> dict:
        req = urllib.request.Request(self.http + path, data=json.dumps(body or {}).encode(),
                                     headers={"Content-Type": "application/json"}, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return json.loads(r.read() or b"{}")
        except urllib.error.HTTPError as e:
            try:
                detail = json.loads(e.read() or b"{}").get("detail")
            except Exception:
                detail = None
            raise Refused(str(detail or f"operator API {path}: HTTP {e.code}")) from None
        except (urllib.error.URLError, OSError) as e:
            raise Refused(f"operator API {path} unreachable ({type(e).__name__})") from None

    async def confirm_play(self, name: str) -> dict:
        return await asyncio.to_thread(self._post, "/playable/confirm", {"name": name})

    async def estop_on(self) -> dict:
        return await asyncio.to_thread(self._post, "/estop")

    async def estop_reset(self) -> dict:
        return await asyncio.to_thread(self._post, "/estop/reset")
