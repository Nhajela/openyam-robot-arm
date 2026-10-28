#!/usr/bin/env python3
"""armctl daemon: one control lease, a deadman for streams, the safety gate, and a log.

Run (mock backend, no hardware):
    .venv/bin/python -m armctl.daemon --backend mock            # ws://127.0.0.1:8091/armctl
    .venv/bin/python -m armctl.daemon --backend mock --auto-approve   # local dev: acquire is granted at once

Programs connect to ws://<host>:8091/armctl and speak the JSON protocol in
experiments/2026-10-16-linux-real-arm/ARMCTL-SPEC.md ("Programs"); see armctl/README.md for
the exact messages and the few extensions (done, pending, playable.result, hello reply).

Operator endpoints (HTTP, loopback clients only; the lab console on :8090 proxies them under /armctl/):
    GET  /status                 clients, lease, pending acquires, refusals, pending playable, recent log
    POST /lease {client|program} grant the lease (stop first, then grant); {"revoke": true} takes it back
    POST /lease/deny {client}    drop a pending acquire
    POST /estop                  stop now, revoke the lease, latch; every motion refused until reset
    POST /estop/reset            unlatch
    POST /enable                 torque on, holding position (after relax)
    POST /playable/confirm {name}   start the playable the lease holder asked to play
    POST /playable/cancel        drop a pending playable
"""
from __future__ import annotations

import argparse
import asyncio
import collections
import contextlib
import datetime as dt
import importlib
import inspect
import json
import logging
import re
import time
from pathlib import Path
from typing import Any, Awaitable, Callable

import numpy as np
from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse

from . import envelope as env
from .backend import BACKENDS, Backend, Trajectory, plan_home, plan_move

REPO = Path(__file__).resolve().parent.parent
DEFAULT_LOG = REPO / "experiments" / "2026-10-16-linux-real-arm" / "logs" / "armctl-daemon.log"
POSES_FILE = Path(__file__).resolve().parent / "poses.json"
PLAYABLES_DIR = REPO / "playables"
PROTOCOL = "armctl/0"

STATE_HZ = 20.0
TTL_DEFAULT_MS = 200
TTL_MAX_MS = 500
NUDGE_MAX = 0.2  # rad per nudge
REACH_TOL = 0.02  # rad, execute-then-verify (mock); see REACH_TOL_BY_BACKEND
# The real arm has no gravity feedforward: it sags ~0.03 rad on j2/j3 and ~0.06 rad on the
# wrist (2026-10-16 SUMMARY.md), so a single 0.02 rad check would refuse every real move.
REACH_TOL_BY_BACKEND = {"mock": (0.02,) * 6, "dimos": (0.02, 0.05, 0.05, 0.09, 0.09, 0.09)}
MAX_MSG = 256 * 1024
REFUSAL_REPEAT_S = 1.0  # same stream refusal at most once per second per client
SWITCH_STOP_TIMEOUT_S = 3.0
PROGRAM_RE = re.compile(r"[^a-zA-Z0-9_.:-]")
DISCRETE = ("home", "ready", "goto", "move_joints", "nudge", "gripper", "stop", "relax", "save_pose")
MOTION = ("home", "ready", "goto", "move_joints", "nudge")

log = logging.getLogger("armctl")
SendFn = Callable[[dict], Awaitable[None]]


def _now_iso() -> str:
    return dt.datetime.now().astimezone().isoformat(timespec="seconds")


def _clean(s: Any, n: int, pattern: re.Pattern | None = None) -> str:
    s = str(s if s is not None else "")
    s = "".join(ch for ch in s if ch.isprintable())[:n]
    return pattern.sub("", s) if pattern else s


class Refused(Exception):
    """An instruction refused with a human-readable reason."""


class Client:
    _n = 0

    def __init__(self, send: SendFn, remote: str = "?"):
        Client._n += 1
        self.id = f"c{Client._n}"
        self.send_raw = send
        self.remote = remote
        self.program: str | None = None
        self.name = ""
        self.connected_at = time.time()
        self.validated: dict[str, dict] = {}  # playable name -> playable (validated ok)
        self.last_stream_refusal: dict[str, float] = {}
        self.alive = True

    @property
    def tag(self) -> str:
        return f"{self.id} {self.program or '?'}"

    def info(self) -> dict:
        return {"id": self.id, "program": self.program, "name": self.name, "remote": self.remote,
                "connected_s": round(time.time() - self.connected_at, 1)}

    async def send(self, msg: dict) -> None:
        if not self.alive:
            return
        try:
            await asyncio.wait_for(self.send_raw(msg), timeout=1.0)
        except Exception:
            self.alive = False


# --------------------------------------------------------------------------
# Playable hooks (armctl/playable.py is written by another agent; import lazily)
# --------------------------------------------------------------------------
def _playable_module():
    try:
        return importlib.import_module("armctl.playable")
    except ModuleNotFoundError as e:
        if e.name == "armctl.playable":
            return None
        raise


def _call_flex(fn: Callable, *args: Any, **optional: Any) -> Any:
    """Call fn with the optional kwargs it accepts."""
    try:
        params = inspect.signature(fn).parameters
        has_var = any(p.kind == p.VAR_KEYWORD for p in params.values())
        kw = {k: v for k, v in optional.items() if has_var or k in params}
    except (TypeError, ValueError):
        kw = {}
    return fn(*args, **kw)


def _violation_text(v: Any) -> str:
    if isinstance(v, str):
        return v
    if isinstance(v, dict):
        return str(v.get("message") or v.get("msg") or v.get("reason") or v)
    return str(getattr(v, "message", v))


def _sync_playable_envelope(mod: Any, envelope: env.Envelope) -> None:
    """armctl.playable keeps its own envelope adapter (default: envelope.DEFAULT); give it
    the daemon's envelope when the daemon runs with non-default margins/caps."""
    if envelope is env.DEFAULT or not hasattr(mod, "set_envelope") or not hasattr(mod, "_Envelope"):
        return
    cur = mod.envelope() if hasattr(mod, "envelope") else None
    if getattr(cur, "impl", None) is not envelope:
        mod.set_envelope(mod._Envelope(impl=envelope))


def playable_validate(playable: dict, envelope: env.Envelope) -> tuple[bool, list[str]]:
    mod = _playable_module()
    if mod is None:
        raise Refused("playables not available yet (armctl/playable.py missing)")
    fn = getattr(mod, "validate", None) or getattr(mod, "validate_playable", None)
    if fn is None:
        raise Refused("playables not available yet (armctl.playable has no validate())")
    _sync_playable_envelope(mod, envelope)
    r = _call_flex(fn, playable, envelope=envelope)
    if isinstance(r, tuple) and len(r) == 2:
        ok, viol = r
    elif isinstance(r, dict):
        viol = r.get("violations") or r.get("errors") or []
        ok = r.get("ok", not viol)
    elif isinstance(r, list):
        viol, ok = r, not r
    else:
        viol = getattr(r, "violations", None) or getattr(r, "errors", None) or []
        ok = getattr(r, "ok", not viol)
    return bool(ok), [_violation_text(v) for v in viol]


def playable_trajectory(playable: dict, envelope: env.Envelope, dt_s: float) -> Trajectory:
    """Sample a validated playable into a joint Trajectory via armctl.playable."""
    mod = _playable_module()
    if mod is None:
        raise Refused("playables not available yet (armctl/playable.py missing)")
    fn = None
    for name in ("sample", "samples", "to_trajectory", "trajectory", "compile"):
        fn = getattr(mod, name, None)
        if callable(fn):
            break
    if fn is None:
        raise Refused("armctl.playable has no sample()/to_trajectory(): cannot play yet")
    _sync_playable_envelope(mod, envelope)
    r = _call_flex(fn, playable, envelope=envelope, rate=1.0 / dt_s, rate_hz=1.0 / dt_s, hz=1.0 / dt_s, dt=dt_s)
    grip = None
    t = None
    if isinstance(r, Trajectory):
        return r
    if isinstance(r, dict):
        q = r.get("q")
        grip = r.get("grip")
        t = r.get("t")
        dt_s = float(r.get("dt", dt_s))
    elif isinstance(r, tuple):
        if len(r) == 3:
            t, q, grip = r
        elif len(r) == 2:
            t, q = r
        else:
            raise Refused("armctl.playable sample() returned an unexpected tuple")
    else:
        q = r
    q = np.asarray(q, dtype=float)
    if q.ndim != 2 or q.shape[1] != env.N:
        raise Refused(f"playable samples have shape {q.shape}, expected (n, 6)")
    if t is not None and len(t) >= 2:
        t = np.asarray(t, dtype=float)
        dts = np.diff(t)
        if np.ptp(dts) > 1e-6:  # resample to uniform
            tu = np.arange(t[0], t[-1] + 1e-9, dt_s)
            q = np.stack([np.interp(tu, t, q[:, i]) for i in range(env.N)], axis=1)
            if grip is not None:
                grip = np.interp(tu, t, np.asarray(grip, dtype=float))
        else:
            dt_s = float(dts[0])
    g = None if grip is None else np.asarray(grip, dtype=float).reshape(-1)
    return Trajectory(q, dt_s, g, label=f"playable:{playable.get('name', '?')}")


# --------------------------------------------------------------------------
# Daemon core (transport-independent; the FastAPI app below wires WebSockets to it)
# --------------------------------------------------------------------------
class Daemon:
    def __init__(self, backend: Backend, *, auto_approve: bool = False, envelope: env.Envelope | None = None):
        self.backend = backend
        self.envelope = envelope or backend.envelope
        self.reach_tol = np.array(REACH_TOL_BY_BACKEND.get(getattr(backend, "name", ""), (REACH_TOL,) * 6), float)
        backend.on_event = self._on_backend_event
        self.auto_approve = auto_approve
        self.clients: dict[str, Client] = {}
        self.holder: Client | None = None
        self.switching = False
        self.pending: collections.OrderedDict[str, float] = collections.OrderedDict()
        self.estopped = False
        self.busy: dict | None = None
        self.streaming = False
        self.stream_deadline = 0.0
        self.stream_kind = ""
        self.pending_play: dict | None = None
        self.refusals: collections.deque[dict] = collections.deque(maxlen=40)
        self.recent: collections.deque[str] = collections.deque(maxlen=60)
        self.poses = self._load_poses()
        self.loop: asyncio.AbstractEventLoop | None = None
        self._tasks: list[asyncio.Task] = []
        self._seq = 0

    # ---- lifecycle
    async def start(self) -> None:
        self.loop = asyncio.get_running_loop()
        self._tasks.append(asyncio.create_task(self._ticker()))
        self.log(f"daemon up: backend={self.backend.name} auto_approve={self.auto_approve} "
                 f"soft margin={self.envelope.margin.tolist()} vel caps={self.envelope.vel_caps.tolist()}")

    async def shutdown(self) -> None:
        for t in self._tasks:
            t.cancel()
        with contextlib.suppress(Exception):
            self.backend.stop()
        self.log("daemon down (backend stopped)")

    # ---- logging
    def log(self, msg: str, level: int = logging.INFO) -> None:
        log.log(level, msg)
        self.recent.append(f"{dt.datetime.now().strftime('%H:%M:%S')} {msg}")

    def _refuse_record(self, client: Client | None, reason: str, what: str = "") -> None:
        self.refusals.append({"t": time.time(), "client": client.id if client else None,
                              "program": client.program if client else None, "what": what, "reason": reason})
        self.log(f"[{client.tag if client else 'operator'}] REFUSED {what}: {reason}")

    # ---- poses
    def _load_poses(self) -> dict[str, list[float]]:
        poses = {k: list(v) for k, v in env.POSES.items()}
        try:
            for k, v in json.loads(POSES_FILE.read_text()).items():
                poses.setdefault(k, list(map(float, v)))
        except FileNotFoundError:
            pass
        except (ValueError, OSError) as e:
            log.warning("poses.json unreadable: %s", e)
        return poses

    # ---- messages
    def lease_info(self) -> dict:
        h = self.holder
        return {"holder": h.program if h else None, "client": h.id if h else None,
                "name": h.name if h else None, "switching": self.switching}

    def state_msg(self) -> dict:
        s = self.backend.state()
        return {"op": "state", "t": round(time.time(), 3), **s, "lease": self.lease_info(),
                "estop": self.estopped, "busy": self.busy["label"] if self.busy else None,
                "streaming": self.streaming, "pending": list(self.pending),
                "pending_play": self.pending_play["name"] if self.pending_play else None}

    async def broadcast(self, msg: dict) -> None:
        cs = [c for c in self.clients.values() if c.program is not None]
        if cs:
            await asyncio.gather(*(c.send(msg) for c in cs))

    async def _ticker(self) -> None:
        period = 1.0 / STATE_HZ
        while True:
            try:
                now = time.monotonic()
                if self.streaming and now > self.stream_deadline:
                    self.streaming = False
                    self.log(f"[{self.holder.tag if self.holder else '?'}] deadman: no setpoint for "
                             f"{TTL_DEFAULT_MS}+ ms, decelerating to a stop")
                if self.busy and now > self.busy["deadline"]:
                    b = self.busy
                    self.busy = None
                    self.backend.stop()
                    await self._refuse(b["client"], f"{b['label']} did not finish in time; stopped",
                                       b["id"], "verify")
                await self.broadcast(self.state_msg())
            except asyncio.CancelledError:
                raise
            except Exception as e:  # keep broadcasting no matter what
                log.exception("ticker: %s", e)
            await asyncio.sleep(period)

    async def _refuse(self, client: Client | None, reason: str, mid: Any = None, what: str = "",
                      **extra: Any) -> None:
        self._refuse_record(client, reason, what)
        if client:
            await client.send({"op": "refused", "id": mid, "reason": reason, **extra})

    # ---- backend events (called from the backend thread)
    def _on_backend_event(self, kind: str, data: dict) -> None:
        if self.loop is None or self.loop.is_closed():
            return
        with contextlib.suppress(RuntimeError):
            self.loop.call_soon_threadsafe(lambda: asyncio.ensure_future(self._backend_event(kind, data)))

    async def _backend_event(self, kind: str, data: dict) -> None:
        if kind == "trajectory_done":
            b = self.busy
            if not b or b["label"] != data.get("label"):
                return
            self.busy = None
            if not data.get("ok"):
                await self._refuse(b["client"], f"{b['cmd']} interrupted (stopped)", b["id"], b["cmd"])
                return
            q = np.array(self.backend.state()["q"])
            err = np.abs(q - b["target"])
            if np.any(err > self.reach_tol):
                i = int(np.argmax(err / self.reach_tol))
                self.backend.stop()
                await self._refuse(b["client"], f"{b['cmd']} did not reach target: {env.JOINTS[i]} error "
                                   f"{err[i]:.3f} rad > {self.reach_tol[i]:.2f}; stopped", b["id"], b["cmd"])
                return
            self.log(f"[{b['client'].tag}] done {b['cmd']} (max error {err.max():.4f} rad)")
            await b["client"].send({"op": "done", "id": b["id"], "cmd": b["cmd"]})
        elif kind == "guard":
            c = self.holder
            for v in data.get("violations", []):
                key = f"{v.get('joint')}:{v.get('kind')}"
                now = time.monotonic()
                if c and now - c.last_stream_refusal.get(key, 0.0) >= REFUSAL_REPEAT_S:
                    c.last_stream_refusal[key] = now
                    await self._refuse(c, v["message"], None, "stream", stream=True, joint=v.get("joint"))
        elif kind == "deadman":
            pass  # logged by the ticker
        elif kind == "error":
            self.log(f"backend error: {data.get('error')}", logging.ERROR)

    # ---- connections
    async def connect(self, client: Client) -> None:
        self.clients[client.id] = client

    async def disconnect(self, client: Client) -> None:
        client.alive = False
        self.clients.pop(client.id, None)
        self.pending.pop(client.id, None)
        if client.program is not None:
            self.log(f"[{client.tag}] disconnected")
        if self.pending_play and self.pending_play["client"] is client:
            self.pending_play = None
        if self.holder is client:
            self.backend.stop()
            self.streaming = False
            self.holder = None
            self.busy = None
            self.log(f"[{client.tag}] lease holder disconnected: stopped, lease free")
            await self.broadcast({"op": "lease", **self.lease_info()})

    # ---- lease
    async def grant(self, client: Client, by: str = "operator") -> None:
        """Stop, wait until still, then grant."""
        prev = self.holder
        self.switching = True
        self.holder = None
        self.streaming = False
        self.busy = None
        self.pending_play = None
        self.backend.stop()
        t_end = time.monotonic() + SWITCH_STOP_TIMEOUT_S
        while self.backend.state()["moving"] and time.monotonic() < t_end:
            await asyncio.sleep(0.02)
        self.switching = False
        self.pending.pop(client.id, None)
        if client.id not in self.clients:
            self.log(f"[{client.tag}] grant cancelled: client gone")
            await self.broadcast({"op": "lease", **self.lease_info()})
            return
        self.holder = client
        self.log(f"[{client.tag}] lease granted by {by} (previous: {prev.tag if prev else 'none'}; stopped first)")
        await self.broadcast({"op": "lease", **self.lease_info()})

    async def revoke(self, by: str = "operator") -> None:
        h = self.holder
        self.backend.stop()
        self.streaming = False
        self.busy = None
        self.pending_play = None
        self.holder = None
        self.log(f"lease revoked by {by} (was {h.tag if h else 'none'}); stopped")
        await self.broadcast({"op": "lease", **self.lease_info()})

    def find_client(self, client_id: str | None = None, program: str | None = None) -> Client | None:
        if client_id:
            return self.clients.get(client_id)
        if program:
            for cid in self.pending:  # oldest pending request first
                c = self.clients.get(cid)
                if c and c.program == program:
                    return c
            cs = [c for c in self.clients.values() if c.program == program]
            return max(cs, key=lambda c: c.connected_at) if cs else None
        return None

    # ---- operator
    async def estop(self, by: str = "operator") -> None:
        self.backend.estop()
        self.estopped = True
        self.streaming = False
        b, self.busy = self.busy, None
        self.pending_play = None
        self.pending.clear()
        h, self.holder = self.holder, None
        self.log(f"E-STOP by {by}: halted, lease revoked (was {h.tag if h else 'none'}), latched", logging.WARNING)
        if b:
            await b["client"].send({"op": "refused", "id": b["id"], "reason": "E-STOP"})
        await self.broadcast({"op": "lease", **self.lease_info()})
        await self.broadcast({"op": "estop", "latched": True})

    async def estop_reset(self, by: str = "operator") -> None:
        self.estopped = False
        try:
            self.backend.clear_estop()
        except Exception as e:  # the daemon latch is cleared either way; say what the backend said
            self.log(f"backend clear_estop failed: {e}", logging.ERROR)
        self.log(f"E-STOP reset by {by}")
        await self.broadcast({"op": "estop", "latched": False})

    async def enable(self, by: str = "operator") -> None:
        self.backend.enable()
        self.log(f"torque enabled (holding) by {by}")

    async def confirm_play(self, name: str | None, by: str = "operator") -> dict:
        p = self.pending_play
        if not p:
            raise Refused("no playable is waiting for confirmation")
        if name and name != p["name"]:
            raise Refused(f"pending playable is {p['name']!r}, not {name!r}")
        c: Client = p["client"]
        self.pending_play = None
        if self.holder is not c:
            raise Refused(f"{c.tag} no longer holds the lease")
        self._motion_allowed()
        # Re-plan from where the arm is now (it may have moved since play was requested).
        traj = self._playable_with_approach(p["traj"])
        self._start_trajectory(c, p["id"], f"playable:{p['name']}", traj)
        self.log(f"[{c.tag}] playable {p['name']!r} confirmed by {by}: playing {traj.duration:.1f} s")
        await c.send({"op": "ack", "id": p["id"], "cmd": "playable.play", "playing": p["name"],
                      "duration": round(traj.duration, 2)})
        return {"ok": True, "name": p["name"], "duration": traj.duration}

    async def cancel_play(self, by: str = "operator") -> None:
        p, self.pending_play = self.pending_play, None
        if p:
            await self._refuse(p["client"], f"operator cancelled playable {p['name']!r}", p["id"], "playable.play")

    def status(self) -> dict:
        p = self.pending_play
        return {
            "state": self.state_msg(),
            "clients": [{**c.info(), "holder": c is self.holder, "pending": c.id in self.pending}
                        for c in self.clients.values() if c.program is not None],
            "lease": self.lease_info(),
            "pending": [{**self.clients[cid].info(), "since_s": round(time.time() - t, 1)}
                        for cid, t in self.pending.items() if cid in self.clients],
            "refusals": list(self.refusals)[-15:],
            "pending_play": ({"name": p["name"], "client": p["client"].id, "program": p["client"].program,
                              "duration": round(p["traj"].duration, 2)} if p else None),
            "estop": self.estopped,
            "auto_approve": self.auto_approve,
            "backend": self.backend.name,
            "log": list(self.recent)[-25:],
            "envelope": {"soft_limits": self.envelope.describe()["soft_limits"],
                         "vel_caps": self.envelope.vel_caps.tolist()},
        }

    # ---- instruction handling
    async def handle(self, client: Client, msg: Any) -> None:
        if not isinstance(msg, dict) or not isinstance(msg.get("op"), str):
            await self._refuse(client, "message must be a JSON object with an 'op'", None, "?")
            return
        op = msg["op"]
        mid = msg.get("id")
        if op != "hello" and client.program is None:
            await self._refuse(client, "say hello first: {\"op\":\"hello\",\"program\":...}", mid, op)
            return
        try:
            if op == "hello":
                await self._hello(client, msg)
            elif op == "acquire":
                await self._acquire(client, mid)
            elif op == "release":
                await self._release(client, mid)
            elif op == "do":
                await self._do(client, msg, mid)
            elif op == "stream":
                await self._stream(client, msg)
            elif op == "playable.validate":
                await self._validate(client, msg, mid)
            elif op == "playable.play":
                await self._play(client, msg, mid)
            elif op == "state":
                await client.send(self.state_msg())
            elif op == "ping":
                await client.send({"op": "pong", "id": mid})
            else:
                raise Refused(f"unknown op {op!r}")
        except Refused as e:
            await self._refuse(client, str(e), mid, op if op != "do" else f"do {msg.get('cmd')}")
        except Exception as e:  # never crash a connection on a bad message
            log.exception("handling %s", op)
            await self._refuse(client, f"internal error: {e}", mid, op)

    async def _hello(self, client: Client, msg: dict) -> None:
        program = _clean(msg.get("program"), 32, PROGRAM_RE)
        if not program:
            raise Refused("hello needs a 'program' (e.g. tilt, joystick, llm, console)")
        first = client.program is None
        client.program = program
        client.name = _clean(msg.get("name"), 64)
        if first:
            self.log(f"[{client.tag}] hello name={client.name!r} from {client.remote}")
        await client.send({"op": "hello", "id": msg.get("id"), "client": client.id, "protocol": PROTOCOL,
                           "lease": self.lease_info(), "estop": self.estopped,
                           "auto_approve": self.auto_approve, "backend": self.backend.name,
                           "envelope": self.envelope.describe(), "poses": self.poses,
                           "ttl_ms": {"default": TTL_DEFAULT_MS, "max": TTL_MAX_MS}})

    async def _acquire(self, client: Client, mid: Any) -> None:
        if self.holder is client:
            await client.send({"op": "ack", "id": mid, "cmd": "acquire", "lease": self.lease_info()})
            return
        if self.estopped:
            raise Refused("E-STOP is latched: the operator must reset it in the console")
        if self.auto_approve:
            self.log(f"[{client.tag}] acquire (auto-approved)")
            await client.send({"op": "ack", "id": mid, "cmd": "acquire", "granted": True})
            await self.grant(client, by="auto-approve")
            return
        self.pending[client.id] = time.time()
        self.log(f"[{client.tag}] acquire: waiting for operator approval")
        await client.send({"op": "ack", "id": mid, "cmd": "acquire", "pending": True,
                           "reason": "waiting for the operator to approve in the console"})

    async def _release(self, client: Client, mid: Any) -> None:
        self.pending.pop(client.id, None)
        if self.holder is client:
            await self.revoke(by=f"{client.tag} (release)")
        await client.send({"op": "ack", "id": mid, "cmd": "release"})

    def _require_lease(self, client: Client) -> None:
        if self.holder is not client:
            h = self.holder
            raise Refused(f"not the lease holder (holder: {h.program + ' ' + h.id if h else 'nobody'}); "
                          "send acquire and wait for the operator")

    def _motion_allowed(self) -> None:
        if self.estopped:
            raise Refused("E-STOP is latched: the operator must reset it in the console")
        st = self.backend.state()
        if not st["torque"]:
            raise Refused("torque is off (relaxed): the operator must re-enable it in the console")

    def _gate(self, traj: Trajectory, target: np.ndarray | None, what: str) -> None:
        home = traj.allow_home
        if target is not None and not home:
            viol = self.envelope.check_point(target)
            if viol:
                raise Refused("; ".join(v.message for v in viol))
        viol = self.envelope.check_path(traj.q, traj.dt, allow_home=home, from_outside=True)
        if viol:
            raise Refused(f"{what} path leaves the envelope: " + "; ".join(v.message for v in viol))

    def _start_trajectory(self, client: Client, mid: Any, cmd: str, traj: Trajectory) -> None:
        self._seq += 1
        traj.label = f"{cmd}#{self._seq}"
        self.backend.execute(traj)
        self.streaming = False
        self.busy = {"client": client, "id": mid, "cmd": cmd, "label": traj.label, "target": traj.q[-1].copy(),
                     "deadline": time.monotonic() + traj.duration + 2.0}

    async def _do(self, client: Client, msg: dict, mid: Any) -> None:
        cmd = msg.get("cmd")
        args = msg.get("args") if isinstance(msg.get("args"), dict) else msg
        if isinstance(cmd, dict) and len(cmd) == 1:  # {"cmd": {"goto": {"name": "ready"}}}
            (cmd, a), = cmd.items()
            args = a if isinstance(a, dict) else {"value": a}
        if cmd not in DISCRETE:
            raise Refused(f"unknown cmd {cmd!r} (one of {', '.join(DISCRETE)})")
        if cmd == "stop":  # any connected program may stop the arm
            self.backend.stop()
            self.streaming = False
            b, self.busy = self.busy, None
            self.log(f"[{client.tag}] do stop")
            if b:
                await b["client"].send({"op": "refused", "id": b["id"], "reason": f"{b['cmd']} interrupted: stopped by {client.tag}"})
            await client.send({"op": "ack", "id": mid, "cmd": "stop"})
            return
        self._require_lease(client)
        if cmd == "relax":
            try:
                self.backend.relax()
            except (NotImplementedError, RuntimeError) as e:
                raise Refused(f"relax: {e}") from None
            self.streaming = False
            self.busy = None
            self.log(f"[{client.tag}] do relax: torque off", logging.WARNING)
            await client.send({"op": "ack", "id": mid, "cmd": "relax",
                               "warning": "torque off: support the arm. The operator re-enables in the console."})
            return
        if cmd == "save_pose":
            name = _clean(args.get("name"), 32, PROGRAM_RE)
            if not name or name in env.POSES:
                raise Refused("save_pose needs a new 'name' (home/ready are fixed)")
            q = self.backend.state()["q"]
            viol = self.envelope.check_point(q)
            self.poses[name] = q
            saved = {k: v for k, v in self.poses.items() if k not in env.POSES}
            POSES_FILE.write_text(json.dumps(saved, indent=1) + "\n")
            self.log(f"[{client.tag}] save_pose {name} = {q}")
            await client.send({"op": "ack", "id": mid, "cmd": "save_pose", "name": name, "q": q,
                               "warning": ("; ".join(v.message for v in viol) + " (goto will refuse it)") if viol else None})
            return
        self._motion_allowed()
        if cmd == "gripper":
            if args.get("open") or args.get("value") == "open" or msg.get("width") == "open":
                width = 1.0
            elif args.get("close") or args.get("value") == "close" or msg.get("width") == "close":
                width = 0.0
            else:
                try:
                    width = float(args.get("width", args.get("value")))
                except (TypeError, ValueError):
                    raise Refused("gripper needs width (0 closed .. 1 open), open or close")
                if not 0.0 <= width <= 1.0:
                    raise Refused(f"gripper width {width} outside 0 (closed) .. 1 (open)")
            self.backend.gripper(width)
            self.log(f"[{client.tag}] do gripper {width:.2f}")
            await client.send({"op": "ack", "id": mid, "cmd": "gripper", "width": width})
            return
        # joint-space motion
        if self.busy:
            raise Refused(f"busy: {self.busy['cmd']} in progress (send do stop first)")
        q0 = np.array(self.backend.state()["q"])
        speed = args.get("speed")
        if speed is not None:
            try:
                speed = float(speed)
                if not speed > 0:
                    raise ValueError
            except (TypeError, ValueError):
                raise Refused(f"speed must be a positive number (rad/s), got {args.get('speed')!r}")
        capped = speed is not None and speed > float(self.envelope.vel_caps.min())
        if cmd == "home" or (cmd == "goto" and args.get("name") == "home"):
            cmd = "home"
            traj = plan_home(q0, self.envelope, speed)
            target = None
        else:
            if cmd == "ready":
                target = np.array(self.poses["ready"])
            elif cmd == "goto":
                name = args.get("name", args.get("value"))
                if name not in self.poses:
                    raise Refused(f"unknown pose {name!r} (known: {', '.join(self.poses)})")
                target = np.array(self.poses[name])
            elif cmd == "move_joints":
                try:
                    target = env.as_vector(args.get("q"))
                except (ValueError, TypeError) as e:
                    raise Refused(f"move_joints needs q (6 joint angles, rad): {e}")
            else:  # nudge
                try:
                    i = env.joint_index(args.get("joint"))
                    delta = float(args.get("delta"))
                except (ValueError, TypeError) as e:
                    raise Refused(f"nudge needs joint (j1..j6) and delta (rad): {e}")
                if abs(delta) > NUDGE_MAX:
                    raise Refused(f"nudge delta {delta:+.3f} rad exceeds {NUDGE_MAX} (use move_joints)")
                target = q0.copy()
                target[i] += delta
            if not np.all(np.isfinite(target)):
                raise Refused("target contains NaN or infinity")
            traj = plan_move(q0, target, self.envelope, speed)
        self._gate(traj, target, cmd)
        self._start_trajectory(client, mid, cmd, traj)
        self.log(f"[{client.tag}] do {cmd} -> {np.round(traj.q[-1], 3).tolist()} in {traj.duration:.2f} s"
                 + (" (speed capped)" if capped else ""))
        await client.send({"op": "ack", "id": mid, "cmd": cmd, "duration": round(traj.duration, 2),
                           "target": np.round(traj.q[-1], 4).tolist(),
                           **({"note": f"speed capped to {self.envelope.vel_caps.min():.2f} rad/s"} if capped else {})})

    async def _stream(self, client: Client, msg: dict) -> None:
        self._require_lease(client)
        self._motion_allowed()
        if self.busy:
            raise Refused(f"busy: {self.busy['cmd']} in progress (send do stop first)")
        try:
            ttl = float(msg.get("ttl_ms", TTL_DEFAULT_MS))
        except (TypeError, ValueError):
            raise Refused("ttl_ms must be a number")
        ttl = min(max(ttl, 0.0), TTL_MAX_MS) / 1000.0
        if msg.get("qd") is not None:
            try:
                qd = env.as_vector(msg["qd"])
            except (ValueError, TypeError) as e:
                raise Refused(f"stream qd: {e}")
            if not np.all(np.isfinite(qd)):
                raise Refused("stream qd contains NaN or infinity")
            self.backend.set_joint_velocity(qd, ttl)
            kind = "joint"
        else:
            v = msg.get("v") or {}
            if not isinstance(v, dict):
                raise Refused("stream v must be an object {x,y,z,roll,pitch,yaw}")
            try:
                self.backend.set_twist(v, ttl)
            except ValueError as e:
                raise Refused(f"stream v: {e}")
            kind = "cartesian"
        grip = msg.get("grip")
        if grip is not None:
            try:
                g = float(grip)
            except (TypeError, ValueError):
                raise Refused("grip must be a number 0 (closed) .. 1 (open)")
            self.backend.gripper(min(max(g, 0.0), 1.0))
        if not self.streaming:
            self.log(f"[{client.tag}] stream start ({kind}, ttl {ttl * 1000:.0f} ms)")
        self.streaming = True
        self.stream_kind = kind
        self.stream_deadline = time.monotonic() + ttl

    async def _validate(self, client: Client, msg: dict, mid: Any) -> None:
        playable = msg.get("playable")
        if playable is None and msg.get("name"):
            name = _clean(msg["name"], 64, PROGRAM_RE)
            path = PLAYABLES_DIR / f"{name}.json"
            try:
                playable = json.loads(path.read_text())
            except (OSError, ValueError) as e:
                raise Refused(f"cannot load playables/{name}.json: {e}")
        if not isinstance(playable, dict):
            raise Refused("playable.validate needs a playable object (or the name of one in playables/)")
        ok, viol = playable_validate(playable, self.envelope)
        name = str(playable.get("name") or "unnamed")
        if ok:
            client.validated[name] = playable
        self.log(f"[{client.tag}] playable.validate {name!r}: " + ("ok" if ok else f"{len(viol)} violations"))
        await client.send({"op": "playable.result", "id": mid, "name": name, "ok": ok, "violations": viol})

    def _playable_with_approach(self, traj: Trajectory) -> Trajectory:
        q0 = np.array(self.backend.state()["q"])
        approach = plan_move(q0, traj.q[0], self.envelope, label="approach")
        full = approach + traj
        full.allow_home = self.envelope.is_home(traj.q[-1])
        self._gate(full, None, "playable")
        return full

    async def _play(self, client: Client, msg: dict, mid: Any) -> None:
        self._require_lease(client)
        self._motion_allowed()
        name = str(msg.get("name") or "")
        playable = client.validated.get(name)
        if playable is None:
            raise Refused(f"playable {name!r} not validated by this program (send playable.validate first)")
        if self.busy:
            raise Refused(f"busy: {self.busy['cmd']} in progress (send do stop first)")
        ok, viol = playable_validate(playable, self.envelope)  # re-check: envelope may have changed
        if not ok:
            raise Refused(f"playable {name!r} no longer validates: " + "; ".join(viol))
        traj = playable_trajectory(playable, self.envelope, 1.0 / 100.0)
        # Start pose: if the playable names one, its first sample must be that pose.
        start = playable.get("start")
        if isinstance(start, str) and start in self.poses:
            if np.abs(traj.q[0] - np.array(self.poses[start])).max() > REACH_TOL:
                traj = plan_move(self.poses[start], traj.q[0], self.envelope) + traj
        if traj.duration > 60.0 + 1e-6:
            raise Refused(f"playable lasts {traj.duration:.1f} s > 60 s")
        viol2 = self.envelope.check_path(traj.q, traj.dt, allow_home=self.envelope.is_home(traj.q[-1]))
        if viol2:
            raise Refused("playable path leaves the envelope: " + "; ".join(v.message for v in viol2))
        self._playable_with_approach(traj)  # gate the approach from the current pose now too
        self.pending_play = {"client": client, "id": mid, "name": name, "traj": traj, "t": time.time()}
        self.log(f"[{client.tag}] playable.play {name!r} ({traj.duration:.1f} s): waiting for operator confirmation")
        await client.send({"op": "ack", "id": mid, "cmd": "playable.play", "pending": True, "name": name,
                           "reason": "waiting for the operator to confirm Play in the console"})


# --------------------------------------------------------------------------
# HTTP / WebSocket app
# --------------------------------------------------------------------------
def build_app(daemon: Daemon):
    @contextlib.asynccontextmanager
    async def lifespan(_app):
        await daemon.start()
        yield
        await daemon.shutdown()

    app = FastAPI(title="armctl", lifespan=lifespan, docs_url=None, redoc_url=None, openapi_url=None)

    def operator_only(request: Request) -> str:
        host = request.client.host if request.client else ""
        if host not in ("127.0.0.1", "::1", "localhost", "testclient"):
            raise HTTPException(403, "operator endpoints are loopback-only")
        return "console"

    async def body(request: Request) -> dict:
        try:
            b = await request.json()
            return b if isinstance(b, dict) else {}
        except Exception:
            return {}

    @app.get("/status")
    def status(request: Request):
        operator_only(request)
        return JSONResponse(daemon.status())

    @app.post("/lease")
    async def lease(request: Request):
        by = operator_only(request)
        b = await body(request)
        if b.get("revoke") or ("program" in b and b["program"] is None and not b.get("client")):
            await daemon.revoke(by=by)
            return {"ok": True, "lease": daemon.lease_info()}
        if daemon.estopped:
            raise HTTPException(409, "E-STOP is latched: reset it first")
        c = daemon.find_client(b.get("client"), b.get("program"))
        if c is None:
            raise HTTPException(404, f"no connected program {b.get('client') or b.get('program')!r}")
        await daemon.grant(c, by=by)
        return {"ok": True, "lease": daemon.lease_info()}

    @app.post("/lease/deny")
    async def deny(request: Request):
        operator_only(request)
        b = await body(request)
        cid = b.get("client")
        c = daemon.clients.get(cid) if cid else None
        daemon.pending.pop(cid, None)
        if c:
            await daemon._refuse(c, "the operator denied the lease request", None, "acquire")
        return {"ok": True}

    @app.post("/estop")
    async def estop(request: Request):
        await daemon.estop(by=operator_only(request))
        return {"ok": True, "estop": True}

    @app.post("/estop/reset")
    async def estop_reset(request: Request):
        await daemon.estop_reset(by=operator_only(request))
        return {"ok": True, "estop": False}

    @app.post("/enable")
    async def enable(request: Request):
        await daemon.enable(by=operator_only(request))
        return {"ok": True}

    @app.post("/playable/confirm")
    async def confirm(request: Request):
        by = operator_only(request)
        b = await body(request)
        try:
            return await daemon.confirm_play(b.get("name"), by=by)
        except Refused as e:
            daemon._refuse_record(None, str(e), "playable.confirm")
            raise HTTPException(409, str(e)) from None

    @app.post("/playable/cancel")
    async def cancel(request: Request):
        await daemon.cancel_play(by=operator_only(request))
        return {"ok": True}

    @app.websocket("/armctl")
    async def ws_endpoint(ws: WebSocket):
        await ws.accept()
        remote = ws.client.host if ws.client else "?"
        via = ws.headers.get("x-armctl-via")
        if via and remote in ("127.0.0.1", "::1"):
            remote = f"{_clean(ws.headers.get('x-forwarded-for'), 45)} via {_clean(via, 16)}"
        client = Client(ws.send_json, remote)
        await daemon.connect(client)
        try:
            while True:
                text = await ws.receive_text()
                if len(text) > MAX_MSG:
                    await daemon._refuse(client, f"message too large (> {MAX_MSG} bytes)", None, "?")
                    continue
                try:
                    msg = json.loads(text)
                except ValueError:
                    await daemon._refuse(client, "bad JSON", None, "?")
                    continue
                await daemon.handle(client, msg)
        except WebSocketDisconnect:
            pass
        finally:
            await daemon.disconnect(client)

    return app


def setup_logging(path: Path | None, quiet: bool = False) -> None:
    fmt = logging.Formatter("%(asctime)s %(levelname)s %(message)s", "%Y-%m-%d %H:%M:%S")
    root = logging.getLogger("armctl")
    root.setLevel(logging.INFO)
    root.handlers.clear()
    if path:
        path.parent.mkdir(parents=True, exist_ok=True)
        fh = logging.FileHandler(path)
        fh.setFormatter(fmt)
        root.addHandler(fh)
    if not quiet:
        sh = logging.StreamHandler()
        sh.setFormatter(fmt)
        root.addHandler(sh)
    root.propagate = False


def main() -> None:
    import uvicorn

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1", help="bind address (0.0.0.0 to expose on the LAN)")
    ap.add_argument("--port", type=int, default=8091)
    ap.add_argument("--backend", choices=sorted(BACKENDS), default="mock")
    ap.add_argument("--auto-approve", action="store_true", help="grant acquire requests without the operator (local dev)")
    ap.add_argument("--margin", type=float, default=env.DEFAULT_MARGIN, help="soft-limit margin, rad per side")
    ap.add_argument("--vel-cap", type=float, default=env.VEL_CAP, help="per-joint speed cap, rad/s")
    ap.add_argument("--acc-cap", type=float, default=env.ACC_CAP, help="per-joint acceleration cap, rad/s^2")
    ap.add_argument("--log", default=str(DEFAULT_LOG), help="log file ('' for none)")
    args = ap.parse_args()

    setup_logging(Path(args.log) if args.log else None)
    envelope = env.Envelope(args.margin, args.vel_cap, args.acc_cap)
    backend = BACKENDS[args.backend](envelope)
    daemon = Daemon(backend, auto_approve=args.auto_approve)
    log.info("armctl %s on ws://%s:%d/armctl (backend %s)", PROTOCOL, args.host, args.port, args.backend)
    uvicorn.run(build_app(daemon), host=args.host, port=args.port, log_level="warning",
                timeout_graceful_shutdown=1, ws_max_size=MAX_MSG + 1024)


if __name__ == "__main__":
    main()
