"""openplay.server: the open play station (phones queue up and take turns driving the arm).

    .venv/bin/python -m openplay.server [--daemon ws://127.0.0.1:8091/armctl] [--phone-port 8450]
        [--local-port 8451] [--key KEY] [--turn 600] [--auto-play on|off|auto] [--host-ip IP]
        [--phone-host 0.0.0.0] [--urdf-dir DIR] [--events-log PATH]

Two listeners in one asyncio process:
  * https://<phone-host>:8450  phones. Self-signed cert from dashboard/certs/ (phonecam's),
    every route needs the room key (?k=<key> once, then cookie op_key). "/" is
    openplay/static/play.html, /static/..., WebSocket /ws.
  * http://127.0.0.1:8451  the laptop display and operator, loopback only, no key. "/" is
    openplay/static/display.html, /static/..., /qr.png, /api/join, /urdf/..., WebSocket /display/ws.

The server is the only armctl program (``openplay``): it holds the daemon lease and decides
which player's commands reach the arm (openplay/arm.py). The daemon's envelope gate, deadman
and E-STOP still apply to everything. Contract: openplay/PROTOCOL.md. Queue and turn rules:
openplay/lobby.py. Events (no room keys) go to openplay/logs/events.jsonl.

Modules owned elsewhere (openplay.moves, interfaces.timeline.poses, interfaces.grasp.spots)
are imported lazily, so the server starts without them and reports the feature as off in
``welcome.features``.
"""
from __future__ import annotations

import argparse
import asyncio
import collections
import contextlib
import hmac
import io
import json
import logging
import os
import re
import secrets
import signal
import sys
import time
import uuid
from pathlib import Path
from typing import Any, Awaitable, Callable

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, PlainTextResponse, Response
from fastapi.staticfiles import StaticFiles

from openplay.arm import ANG_RPS, DEFAULT_URL, JOINT_RPS, LIN_MPS, ArmBridge, Refused
from openplay.lobby import Lobby

log = logging.getLogger("openplay")

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
STATIC = HERE / "static"
EVENTS_LOG = HERE / "logs" / "events.jsonl"
DEFAULT_URDF_DIR = Path.home() / ".local/share/dimos/repo/data/yam_description"  # as dashboard/server.py
COOKIE = "op_key"
PID_RE = re.compile(r"^[A-Za-z0-9_-]{1,64}$")
MAX_MSG = 8192
MAX_POSES = 12
SAY_PER_TURN = 6
SAY_TIMEOUT_S = 60.0
PENDING_TIMEOUT_S = 120.0
STREAM_REFUSAL_EVERY_S = 1.0
MOTION_OPS = {"stream", "move", "say", "grip", "ready", "stop", "pose.save", "pose.goto", "seq.play", "grab"}
DISCRETE = {"move", "ready", "pose.goto", "seq.play", "grab"}  # (say joins the busy slot once authored)
LOOPBACK = {"127.0.0.1", "::1", "localhost", "testclient"}

PLACEHOLDER = """<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>open play</title><body style="font:18px system-ui;padding:2em">
<h1>open play</h1><p>{what} is not built yet (openplay/static/{file}). The server is running.</p>"""


def _phonecam():
    """dashboard/phonecam.py (cert, IPs). dashboard/ is not a package: add it to sys.path."""
    d = str(REPO / "dashboard")
    if d not in sys.path:
        sys.path.insert(0, d)
    import phonecam

    return phonecam


def clean_name(s: Any, n: int = 24) -> str:
    s = re.sub(r"[\x00-\x1f\x7f<>]", "", str(s or ""))
    return re.sub(r"\s+", " ", s).strip()[:n]


class ActionError(RuntimeError):
    """A discrete action failed; ``str(e)`` is shown to the player."""


# --------------------------------------------------------------------------
# Connections: an outbound queue per socket, so a slow phone never blocks a broadcast
# --------------------------------------------------------------------------
class Conn:
    def __init__(self, ws, kind: str):
        self.ws, self.kind = ws, kind
        self.q: asyncio.Queue = asyncio.Queue(maxsize=300)
        self.pid: str | None = None
        self.alive = True
        self.last_stream_refusal = 0.0

    def send(self, msg: dict) -> None:
        if not self.alive:
            return
        try:
            self.q.put_nowait(msg)
        except asyncio.QueueFull:
            if msg.get("op") != "arm":  # arm updates are droppable; anything else: the socket is stuck
                self.alive = False  # the writer closes the socket

    async def writer(self) -> None:
        with contextlib.suppress(Exception):
            while self.alive:
                m = await self.q.get()
                await asyncio.wait_for(self.ws.send_text(json.dumps(m)), 5.0)
        self.alive = False
        with contextlib.suppress(Exception):
            await self.ws.close()


# --------------------------------------------------------------------------
# Optional modules
# --------------------------------------------------------------------------
def probe_features() -> tuple[dict, list[dict], list[dict]]:
    moves: list[dict] = []
    spots: list[dict] = []
    poses_ok = False
    try:
        from openplay import moves as M

        moves = [{k: m.get(k) for k in ("name", "title", "emoji", "duration_s", "blurb")} for m in M.catalogue()]
    except Exception as e:
        log.warning("moves unavailable: %s: %s", type(e).__name__, e)
    try:
        from interfaces.grasp import spots as S

        spots = [{"name": s["name"], "title": s.get("title") or s["name"]} for s in S.list_spots()]
    except Exception as e:
        log.warning("grab unavailable: %s: %s", type(e).__name__, e)
    try:
        from interfaces.timeline import poses as TP

        poses_ok = callable(TP.sequence_playable)
    except Exception as e:
        log.warning("pose sequences unavailable: %s: %s", type(e).__name__, e)
    say_ok = bool(moves) or bool(os.environ.get("ANTHROPIC_API_KEY"))
    features = {"say": say_ok, "grab": bool(spots), "tilt": True, "joystick": True, "poses": poses_ok}
    return features, moves, spots


def playable_info(playable: dict) -> tuple[float, list]:
    """(duration_s, keyframes) for captions and the display's ghost preview."""
    kf = playable.get("keyframes") or []
    try:
        from armctl import playable as P

        return round(float(P.sample(playable, rate=10)["duration"]), 1), kf
    except Exception:
        try:
            loop = int(playable.get("loop") or 1)
            return round(float(kf[-1]["t"]) * loop / float(playable.get("speed") or 1.0), 1), kf
        except Exception:
            return 0.0, kf


# --------------------------------------------------------------------------
# The station
# --------------------------------------------------------------------------
class OpenPlay:
    def __init__(self, *, daemon_url: str = DEFAULT_URL, turn_s: float = 600.0, auto_play: str = "auto",
                 join_url: str = "", urdf_dir: Path = DEFAULT_URDF_DIR, events_log: Path | None = EVENTS_LOG,
                 lin_mps: float = LIN_MPS, ang_rps: float = ANG_RPS, joint_rps: float = JOINT_RPS):
        self.lobby = Lobby(turn_s=turn_s)
        self.arm = ArmBridge(daemon_url, on_event=self._arm_event, lin_mps=lin_mps, ang_rps=ang_rps,
                             joint_rps=joint_rps)
        self.auto_play_mode = auto_play            # on | off | auto (on for mock, off otherwise)
        self.join_url = join_url
        self.urdf_dir = Path(urdf_dir)
        self.events_log = events_log
        self.features, self.moves, self.spots = probe_features()
        self.phones: dict[str, set[Conn]] = {}
        self.displays: set[Conn] = set()
        self.busy: dict | None = None               # {pid, kind, title, doing, task}
        self.pending: dict | None = None            # waiting for the operator (auto_play off)
        self.poses: dict[str, list[dict]] = {}       # pid -> [{i (stable id), q, grip}]
        self._pose_ids: dict[str, int] = {}
        self.say_count: dict[str, int] = collections.defaultdict(int)
        self.say_thinking: dict[str, asyncio.Task] = {}   # authoring (cancel on stop / turn end)
        self.say_busy: set[str] = set()                   # one say at a time per player
        self.say_now: dict[str, dict] = {}                # pid -> latest in-progress say (display form)
        self.handover = False
        self._offline_said = False
        self.recent: collections.deque[dict] = collections.deque(maxlen=20)
        self._tasks: list[asyncio.Task] = []
        self._qr: bytes | None = None
        self._n = 0

    # ------------------------------------------------------------------ lifecycle
    async def start(self) -> None:
        if self.events_log:
            self.events_log.parent.mkdir(parents=True, exist_ok=True)
        self._tasks = [asyncio.create_task(self.arm.run(), name="arm"),
                       asyncio.create_task(self._ticker(), name="ticker"),
                       asyncio.create_task(self._arm_loop(), name="arm-loop")]
        self._log({"kind": "server", "text": f"open play up (turn {self.lobby.turn_s:.0f} s, "
                   f"auto_play {self.auto_play_mode})"})

    async def shutdown(self) -> None:
        for t in self._tasks:
            t.cancel()
        with contextlib.suppress(Exception):
            await self.arm.stop()
        with contextlib.suppress(Exception):
            await asyncio.wait_for(self.arm.close(), 3.0)
        self._log({"kind": "server", "text": "open play down"})

    @property
    def auto_play(self) -> bool:
        if self.auto_play_mode == "auto":
            return self.arm.backend == "mock"
        return self.auto_play_mode == "on"

    # ------------------------------------------------------------------ output
    def _log(self, rec: dict) -> None:
        if not self.events_log:
            return
        try:
            with self.events_log.open("a", encoding="utf-8") as f:
                f.write(json.dumps({"t": round(time.time(), 3), **rec}, ensure_ascii=False) + "\n")
        except OSError as e:
            log.warning("events log: %s", e)

    def to_pid(self, pid: str | None, msg: dict) -> None:
        for c in self.phones.get(pid or "", ()):
            c.send(msg)

    def to_phones(self, msg: dict) -> None:
        for cs in self.phones.values():
            for c in cs:
                c.send(msg)

    def to_displays(self, msg: dict) -> None:
        for c in list(self.displays):
            c.send(msg)

    def event(self, kind: str, text: str, who: str | None = None) -> None:
        m = {"op": "event", "t": round(time.time(), 3), "who": who, "kind": kind, "text": text}
        self.recent.append(m)
        self.to_displays(m)
        self._log({"kind": kind, "who": who, "text": text})
        log.info("%s: %s", kind, text)

    def status(self, pid: str, text: str, kind: str = "info") -> None:
        self.to_pid(pid, {"op": "status", "text": text, "kind": kind})

    def refuse(self, c: Conn, reason: str) -> None:
        c.send({"op": "refused", "reason": reason})

    def push_lobby(self) -> None:
        for pid, cs in self.phones.items():
            v = {"op": "lobby", **self.lobby.view_for(pid)}
            for c in cs:
                c.send(v)
        self.to_displays({"op": "lobby", **self.lobby.view_display()})

    def push_config(self) -> None:
        self.to_displays({"op": "config", "auto_play": self.auto_play, "paused": self.lobby.paused,
                          "turn_s": self.lobby.turn_s})

    def push_pending(self) -> None:
        p = self.pending
        self.to_displays({"op": "pending", "pending": (
            {k: p[k] for k in ("id", "who", "title", "duration_s", "keyframes")} if p else None)})

    def name(self, pid: str | None) -> str:
        return self.lobby.name(pid) or "someone"

    def arm_msg(self, display: bool = False) -> dict | None:
        s = self.arm.state
        if not s:
            return None
        b = self.busy
        m = {"op": "arm", "q": s.get("q"), "grip": s.get("grip"), "moving": s.get("moving"),
             "busy": (b["title"] if b else None) or s.get("busy"), "mode": s.get("mode")}
        if display:
            m["estop"], m["lease"], m["adapter"] = self.arm.estop, self.arm.has_lease, self.arm.adapter
        return m

    def daemon_health(self) -> str:
        if not self.arm.connected:
            return "offline"
        if self.arm.estop:
            return "estop"
        return "connected" if self.arm.has_lease else "waiting for lease"

    def welcome(self, pid: str) -> dict:
        return {"op": "welcome", "pid": pid, "name": self.lobby.name(pid), "moves": self.moves,
                "spots": self.spots, "turn_s": self.lobby.turn_s, "features": self.features,
                "speeds": self.arm.speeds()}

    def display_hello(self) -> dict:
        h = self.arm.hello
        return {"op": "hello", "join_url": self.join_url, "backend": self.arm.backend, "adapter": self.arm.adapter,
                "turn_s": self.lobby.turn_s, "auto_play": self.auto_play, "paused": self.lobby.paused,
                "moves": self.moves, "spots": self.spots, "envelope": h.get("envelope"),
                "poses": h.get("poses"), "urdf": (self.urdf_dir / "i2rt" / "yam.urdf").is_file(),
                "lease": self.arm.has_lease, "daemon": self.daemon_health()}

    def qr_png(self) -> bytes | None:
        if self._qr is None:
            try:
                import qrcode

                buf = io.BytesIO()
                qrcode.make(self.join_url, border=2).save(buf, format="PNG")
                self._qr = buf.getvalue()
            except Exception as e:
                log.warning("QR: %s", e)
        return self._qr

    # ------------------------------------------------------------------ loops
    async def _ticker(self) -> None:
        last_push = 0.0
        while True:
            try:
                can_start = not self.handover and self.arm.ready
                if (can_start and self.lobby.active is None and not self.lobby.paused
                        and self.lobby.next_pid() and not self.busy and not self.near_ready()):
                    can_start = False  # e.g. the arm starts folded at home: ready first
                    self.handover = True
                    asyncio.create_task(self._prepare())
                evs = self.lobby.tick(can_start=can_start)
                if evs:
                    self._lobby_events(evs)
                now = time.monotonic()
                if now - last_push >= 1.0:
                    last_push = now
                    a = self.lobby.active
                    if a is not None:
                        rem = self.lobby.remaining() or 0.0
                        self.to_pid(a.pid, {"op": "turn", "remaining_s": round(rem),
                                            "warning": rem <= self.lobby.warn_s})
                    if a is not None or evs:
                        self.push_lobby()
            except asyncio.CancelledError:
                raise
            except Exception:
                log.exception("ticker")
            await asyncio.sleep(0.25)

    async def _arm_loop(self) -> None:
        n = 0
        while True:
            try:
                n += 1
                d = self.arm_msg(display=True) if self.arm.connected else None  # silence = disconnected
                if d is not None:
                    self.to_displays(d)
                    m = {k: v for k, v in d.items() if k != "estop"}
                    active = self.lobby.active.pid if self.lobby.active else None
                    for pid, cs in self.phones.items():
                        if (pid == active and n % 2 == 0) or n % 10 == 0:
                            for c in cs:
                                c.send(m)
            except asyncio.CancelledError:
                raise
            except Exception:
                log.exception("arm loop")
            await asyncio.sleep(0.05)

    def _lobby_events(self, evs: list[dict]) -> None:
        for e in evs:
            pid, who = e["pid"], self.name(e["pid"])
            if e["kind"] == "turn_start":
                self.say_count[pid] = 0
                self.to_pid(pid, {"op": "turn", "remaining_s": round(e["remaining_s"]), "warning": False})
                self.status(pid, "Your turn! You have the arm.", "info")
                self.event("turn", f"{who}'s turn", who)
            elif e["kind"] == "turn_end":
                self.handover = True  # synchronously: no new turn starts before the arm is at ready
                asyncio.create_task(self._turn_over(pid, e["reason"]))
            elif e["kind"] == "warning":
                self.to_pid(pid, {"op": "turn", "remaining_s": round(e["remaining_s"]), "warning": True})
                self.status(pid, f"{round(e['remaining_s'])} seconds left in your turn", "info")
            elif e["kind"] == "dropped":
                self.event("leave", f"{who} lost their place (away too long)", who)
        self.push_lobby()

    def near_ready(self, tol: float = 0.05) -> bool:
        q, ready = self.arm.state.get("q"), (self.arm.hello.get("poses") or {}).get("ready")
        if not q or not ready:
            return False
        return max(abs(float(x) - float(y)) for x, y in zip(q, ready)) <= tol

    async def _prepare(self) -> None:
        """Before the first turn: bring the arm to ready (the operator granted the lease)."""
        try:
            self.event("operator", "Getting the arm ready")
            self.busy = {"pid": None, "kind": "ready", "title": "ready", "doing": "going to ready", "task": None}
            await self.arm.do("ready", timeout=40.0)
        except Exception as e:
            log.warning("ready before the first turn: %s", e)
            await asyncio.sleep(2.0)
        finally:
            if self.busy and self.busy.get("pid") is None:
                self.busy = None
            self.handover = False

    async def _turn_over(self, pid: str, reason: str) -> None:
        who = self.name(pid)
        self.handover = True
        try:
            self.to_pid(pid, {"op": "turn_end", "reason": reason})
            text = {"time": "time's up", "left": "left", "idle": "no moves for a while",
                    "skipped": "skipped by the operator", "operator": "ended by the operator"}.get(reason, reason)
            self.event("turn_end", f"{who}'s turn is over ({text})", who)
            self._cancel_player_work(pid, "the turn ended")
            await self.arm.stop()
            b = self.busy
            if b and b.get("task"):
                with contextlib.suppress(Exception):
                    await asyncio.wait_for(asyncio.shield(b["task"]), 5.0)
            self.say_count.pop(pid, None)
            if self.arm.ready:
                with contextlib.suppress(Refused, Exception):
                    self.busy = {"pid": None, "kind": "ready", "title": "ready", "doing": "going to ready", "task": None}
                    await self.arm.do("ready", timeout=30.0)
        finally:
            if self.busy and self.busy.get("kind") == "ready" and self.busy.get("pid") is None:
                self.busy = None
            self.handover = False
            self.push_lobby()

    def _cancel_player_work(self, pid: str, why: str) -> None:
        t = self.say_thinking.pop(pid, None)
        if t:
            t.cancel()
        p = self.pending
        if p and p["pid"] == pid and not p["future"].done():
            p["future"].set_result((False, why))

    def _arm_event(self, kind: str, data: dict) -> None:
        if kind == "connected":
            self._offline_said = False
            self.event("operator", f"Connected to the arm ({data.get('backend')} backend); "
                       f"auto-play {'on' if self.auto_play else 'off'}")
            self.to_displays(self.display_hello())
            self.push_config()
        elif kind == "disconnected":
            if not self._offline_said:  # once per outage, not every retry
                self._offline_said = True
                self.event("operator", "Lost the connection to the arm; reconnecting")
                self.to_displays({"op": "error", "reason": "the armctl daemon is not reachable: reconnecting"})
                self.to_displays(self.display_hello())
        elif kind == "waiting_approval":
            self.event("operator", "Approve open play in the armctl console to start")
        elif kind == "denied":
            self.event("operator", "The console denied open play the arm; asking again in 30 s")
            self.to_displays({"op": "error", "reason": "the armctl console denied open play the lease"})
        elif kind == "lease":
            self.event("operator", "Open play has the arm" if data.get("ours") else "Open play lost the arm")
        elif kind == "estop":
            self.event("estop", "E-STOP is on" if data.get("latched") else "E-STOP reset")
        elif kind == "refused":  # stream guard ("j3 at the soft limit"): tell the driver, quietly
            a = self.lobby.active
            if a and data.get("stream"):
                self.status(a.pid, f"The arm can't go further that way ({data.get('reason', '')[:80]})", "info")

    # ------------------------------------------------------------------ phone protocol
    async def phone_msg(self, c: Conn, m: Any) -> None:
        if not isinstance(m, dict) or not isinstance(m.get("op"), str):
            return self.refuse(c, "messages are JSON objects with an 'op'")
        op = m["op"]
        if op == "ping":
            return c.send({"op": "pong"})
        if op == "hello":
            return self._hello(c, m)
        pid = c.pid
        if pid is None:
            return self.refuse(c, "say hello first")
        if op == "join":
            reason = self.lobby.join(pid)
            if reason:
                return self.refuse(c, reason)
            self.event("join", f"{self.name(pid)} joined the queue", self.name(pid))
            return self.push_lobby()
        if op == "leave":
            was = self.lobby.is_active(pid) or self.lobby.position(pid) is not None
            evs = self.lobby.leave(pid)
            if was:
                self.event("leave", f"{self.name(pid)} left", self.name(pid))
            self._lobby_events(evs)
            return
        if op in ("pose.delete", "pose.clear"):
            return self._pose_edit(pid, op, m)
        if op not in MOTION_OPS:
            return self.refuse(c, f"unknown op {op!r}")
        if op == "stream" and _zero_stream(m):
            # the page sends one all-zero stream on release: forward it when it can stop live
            # control, otherwise ignore it quietly (never a refusal, never stops a playing move)
            if (self.lobby.is_active(pid) and not self.handover and not self.busy and not self.pending
                    and self.arm.ready):
                with contextlib.suppress(Refused, ValueError, TypeError):
                    await self.arm.stream(qd=[0.0] * 6) if m.get("qd") is not None else await self.arm.stream(v={})
            return
        if not self.lobby.is_active(pid):
            if op != "stream" or self._stream_refusal_ok(c):
                self.refuse(c, "not your turn")
            return
        self.lobby.touch(pid)
        if op == "stop":
            return await self._stop(pid)
        if self.handover:
            return self.refuse(c, "your turn is ending")
        reason = self.arm.unavailable_reason()
        if reason:
            if op != "stream" or self._stream_refusal_ok(c):
                self.refuse(c, reason)
            return
        if op == "stream":
            return await self._stream(c, m)
        if op == "pose.save":
            return self._pose_save(c, pid)
        if op == "say":
            return self._say_start(c, pid, m)
        if self.busy or self.pending:
            return self.refuse(c, self.busy_reason())
        if op == "grip":
            try:
                v = min(max(float(m.get("value")), 0.0), 1.0)
            except (TypeError, ValueError):
                return self.refuse(c, "grip needs value 0 (closed) .. 1 (open)")
            try:
                await self.arm.do("gripper", width=v)
            except Refused as e:
                self.refuse(c, str(e))
            return
        self._start_discrete(c, pid, op, m)

    def _stream_refusal_ok(self, c: Conn) -> bool:
        now = time.monotonic()
        if now - c.last_stream_refusal >= STREAM_REFUSAL_EVERY_S:
            c.last_stream_refusal = now
            return True
        return False

    def busy_reason(self) -> str:
        if self.pending:
            return f"busy: waiting for the operator to approve {self.pending['title']}"
        if self.busy:
            return f"busy: {self.busy['doing']}"
        return "busy"

    def _hello(self, c: Conn, m: dict) -> None:
        pid = str(m.get("pid") or "")
        if not PID_RE.match(pid):
            return self.refuse(c, "hello needs a pid (1-64 letters, digits, _ or -)")
        if c.pid and c.pid != pid:
            self._detach(c)
        name = clean_name(m.get("name"))
        if not name and pid not in self.lobby.players:
            self._n += 1
            name = f"Player {self._n}"
        c.pid = pid
        self.phones.setdefault(pid, set()).add(c)
        self.lobby.hello(pid, name)
        c.send(self.welcome(pid))
        c.send({"op": "poses", "list": self._pose_list(pid)})
        if self.lobby.is_active(pid):
            rem = self.lobby.remaining() or 0.0
            c.send({"op": "turn", "remaining_s": round(rem), "warning": rem <= self.lobby.warn_s})
        a = self.arm_msg()
        if a:
            c.send(a)
        self.push_lobby()

    def _detach(self, c: Conn) -> None:
        pid = c.pid
        if pid is None:
            return
        cs = self.phones.get(pid)
        if cs is not None:
            cs.discard(c)
            if not cs:
                del self.phones[pid]
                self.lobby.disconnect(pid)
                if not self.lobby.is_active(pid) and self.lobby.position(pid) is None:
                    self.poses.pop(pid, None)
                    self._pose_ids.pop(pid, None)
                    self.lobby.forget(pid)
        c.pid = None

    def phone_gone(self, c: Conn) -> None:
        self._detach(c)
        self.push_lobby()

    async def _stream(self, c: Conn, m: dict) -> None:
        if self.busy or self.pending:
            if self._stream_refusal_ok(c):
                self.refuse(c, self.busy_reason() + " (send stop first)")
            return
        try:
            grip = m.get("grip")
            grip = None if grip is None else float(grip)
            if m.get("qd") is not None:
                await self.arm.stream(qd=m["qd"], grip=grip)
            else:
                await self.arm.stream(v=m.get("v") or {}, grip=grip)
        except (ValueError, TypeError) as e:
            if self._stream_refusal_ok(c):
                self.refuse(c, f"stream: {e}")
        except Refused as e:
            if self._stream_refusal_ok(c):
                self.refuse(c, str(e))

    async def _stop(self, pid: str) -> None:
        self._cancel_player_work(pid, "stopped")
        await self.arm.stop()
        self.status(pid, "Stopped", "done")
        self.event("stop", f"{self.name(pid)} pressed stop", self.name(pid))

    # ---- poses
    def _pose_list(self, pid: str) -> list[dict]:
        return [dict(p) for p in self.poses.get(pid, [])]

    def _pose_find(self, pid: str, i: Any) -> tuple[int, dict] | None:
        """(position, pose) for the stable id ``i`` (ids survive deletes)."""
        try:
            i = int(i)
        except (TypeError, ValueError):
            return None
        return next(((k, p) for k, p in enumerate(self.poses.get(pid, [])) if p["i"] == i), None)

    def _pose_save(self, c: Conn, pid: str) -> None:
        s = self.arm.state
        if not s or s.get("q") is None:
            return self.refuse(c, "no arm state yet")
        lst = self.poses.setdefault(pid, [])
        if len(lst) >= MAX_POSES:
            return self.refuse(c, f"you can save up to {MAX_POSES} poses (delete one first)")
        grip = s.get("grip_target", s.get("grip"))
        self._pose_ids[pid] = self._pose_ids.get(pid, -1) + 1
        lst.append({"i": self._pose_ids[pid], "q": [round(float(x), 4) for x in s["q"]],
                    "grip": round(float(grip or 0.0), 3)})
        c.send({"op": "poses", "list": self._pose_list(pid)})
        self.status(pid, f"Saved pose {len(lst)}", "done")

    def _pose_edit(self, pid: str, op: str, m: dict) -> None:
        lst = self.poses.setdefault(pid, [])
        if op == "pose.clear":
            lst.clear()
        else:
            found = self._pose_find(pid, m.get("i"))
            if found:
                del lst[found[0]]
        self.to_pid(pid, {"op": "poses", "list": self._pose_list(pid)})

    # ---- discrete actions
    def _start_discrete(self, c: Conn, pid: str, op: str, m: dict) -> None:
        if op == "move":
            name = str(m.get("name") or "")
            if not any(x["name"] == name for x in self.moves):
                return self.refuse(c, f"unknown move {name!r}")
            title = next(x.get("title") or name for x in self.moves if x["name"] == name)
            self._begin(pid, "move", title, f"playing {title}", self._do_move(pid, name, title))
        elif op == "ready":
            self._begin(pid, "ready", "ready", "going to ready", self._do_ready(pid))
        elif op == "pose.goto":
            found = self._pose_find(pid, m.get("i"))
            if found is None:
                return self.refuse(c, "no such pose")
            k, pose = found
            self._begin(pid, "pose.goto", f"pose {k + 1}", f"going to pose {k + 1}", self._do_goto(pid, k, pose))
        elif op == "seq.play":
            if not self.features["poses"]:
                return self.refuse(c, "pose sequences are not available")
            poses = [{"q": p["q"], "grip": p["grip"]} for p in self.poses.get(pid, [])]
            if not poses:
                return self.refuse(c, "save a pose first")
            style = m.get("style") if m.get("style") in ("gentle", "lively") else "lively"
            self._begin(pid, "seq", "my sequence", "playing a pose sequence", self._do_seq(pid, poses, style))
        elif op == "grab":
            if not self.features["grab"]:
                return self.refuse(c, "grab is not available")
            src, dst = str(m.get("from") or ""), str(m.get("to") or "")
            names = {s["name"]: s["title"] for s in self.spots}
            if src not in names or dst not in names or src == dst:
                return self.refuse(c, "grab needs two different spots (from, to)")
            title = f"{names[src]} to {names[dst]}"
            self._begin(pid, "grab", "grab", f"moving the block from {title}", self._do_grab(pid, src, dst, title))

    def _begin(self, pid: str, kind: str, title: str, doing: str, coro: Awaitable) -> None:
        """Take the busy slot synchronously (no race with the next message), then run."""
        token = {"pid": pid, "kind": kind, "title": title, "doing": doing, "task": None}
        self.busy = token

        async def run():
            try:
                await coro
            except (ActionError, Refused) as e:
                reason = str(e) if isinstance(e, ActionError) else _friendly(str(e))
                self.status(pid, reason, "error")
                if reason != "Stopped" and "said no" not in reason:
                    self.to_displays({"op": "error", "reason": f"{self.name(pid)}: {reason}"})
            except asyncio.CancelledError:
                pass
            except Exception as e:
                log.exception("action %s", kind)
                self.status(pid, f"something went wrong ({type(e).__name__})", "error")
            finally:
                if self.busy is token:
                    self.busy = None

        token["task"] = asyncio.create_task(run())

    async def _play_flow(self, pid: str, kind: str, title: str, playable: dict,
                         on_start: Callable[[], None] | None = None) -> None:
        """Validate, wait for the operator if auto_play is off, then play to done."""
        who = self.name(pid)
        dur, keyframes = playable_info(playable)
        r = await self.arm.validate(playable)
        if not r.get("ok"):
            raise ActionError(f"{title} doesn't pass the safety check: " + "; ".join(r.get("violations", [])[:2]))
        if not self.auto_play:
            fut = asyncio.get_running_loop().create_future()
            self.pending = {"id": uuid.uuid4().hex[:8], "who": who, "title": title, "duration_s": dur,
                            "keyframes": keyframes, "pid": pid, "future": fut}
            if self.busy and self.busy["pid"] == pid:
                self.busy["doing"] = f"waiting for the operator to approve {title}"
            self.push_pending()
            self.status(pid, f"Waiting for the operator to approve {title}", "busy")
            self.event(kind if kind in ("move", "say", "seq", "grab") else "move",
                       f"{title} is waiting for the operator", who)
            try:
                ok, why = await asyncio.wait_for(fut, PENDING_TIMEOUT_S)
            except asyncio.TimeoutError:
                ok, why = False, "the operator didn't answer"
            finally:
                self.pending = None
                self.push_pending()
            if not ok:
                raise ActionError(f"{title}: {why}")
            if self.busy and self.busy["pid"] == pid:
                self.busy["doing"] = f"playing {title}"
        self.status(pid, f"Playing {title} ({dur:.0f} s)", "busy")
        self.event(kind if kind in ("move", "say", "seq", "grab") else "move", f"Playing {title} ({dur:.0f} s)", who)
        if on_start:
            on_start()
        # confirm=True: either the operator chose auto-play, or they just approved this
        # playable on the loopback display. Nothing else presses the daemon's Play.
        await self.arm.play(playable, confirm=True, timeout=dur + 20.0)
        self.status(pid, f"Done: {title}", "done")

    async def _do_move(self, pid: str, name: str, title: str) -> None:
        from openplay import moves as M

        try:
            playable = await asyncio.to_thread(M.load, name)
        except Exception as e:
            raise ActionError(f"can't load {title}: {e}") from None
        await self._play_flow(pid, "move", title, playable)

    async def _do_ready(self, pid: str) -> None:
        self.status(pid, "Going to ready", "busy")
        await self.arm.do("ready", timeout=30.0)
        self.status(pid, "Ready", "done")

    async def _do_goto(self, pid: str, i: int, pose: dict) -> None:
        self.status(pid, f"Going to pose {i + 1}", "busy")
        await self.arm.do("move_joints", q=pose["q"], timeout=30.0)
        await self.arm.do("gripper", width=pose.get("grip", 0.0))
        self.status(pid, f"At pose {i + 1}", "done")

    async def _do_seq(self, pid: str, poses: list[dict], style: str) -> None:
        from interfaces.timeline import poses as TP

        who = self.name(pid)
        try:
            playable, notes = await asyncio.to_thread(TP.sequence_playable, poses, name="my-sequence", style=style)
        except Exception as e:
            reason = str(e) if isinstance(e, ValueError) else f"{type(e).__name__}: {e}"
            self.to_pid(pid, {"op": "seq", "stage": "error", "duration_s": None, "notes": [reason]})
            raise ActionError(f"Can't play your poses: {reason}") from None
        dur, _ = playable_info(playable)
        self.event("seq", f"{who} is playing {len(poses)} saved pose{'s' if len(poses) != 1 else ''}", who)
        self.to_pid(pid, {"op": "seq", "stage": "playing", "duration_s": dur, "notes": list(notes)})
        try:
            await self._play_flow(pid, "seq", f"{len(poses)} saved poses", playable)
        except (ActionError, Refused) as e:
            self.to_pid(pid, {"op": "seq", "stage": "error", "duration_s": dur, "notes": [_friendly(str(e))]})
            raise
        self.to_pid(pid, {"op": "seq", "stage": "done", "duration_s": dur, "notes": list(notes)})

    async def _do_grab(self, pid: str, src: str, dst: str, title: str) -> None:
        from interfaces.grasp import spots as S

        who = self.name(pid)
        try:
            playable, _info = await asyncio.to_thread(S.grab_playable, src, dst)
        except Exception as e:
            raise ActionError(f"Can't grab {title}: {e}") from None
        self.event("grab", f"{who} asked to move the block from {title}", who)
        await self._play_flow(pid, "grab", f"Grab {title}", playable)

    # ---- say
    def _say_start(self, c: Conn, pid: str, m: dict) -> None:
        text = clean_name(m.get("text"), 200)
        if not text:
            return self.refuse(c, "say what? (type something)")
        if not self.features["say"]:
            return self.refuse(c, "Say it is not available")
        if pid in self.say_busy:
            return self.refuse(c, "one at a time: your last one is still going")
        if self.say_count[pid] >= SAY_PER_TURN:
            return self.refuse(c, f"that's {SAY_PER_TURN} this turn: try the moves or poses")
        self.say_count[pid] += 1
        self.say_busy.add(pid)
        self.say_thinking[pid] = asyncio.create_task(self._say(pid, text))

    def _say_msg(self, pid: str, stage: str, text: str, title: str = "", duration_s: float | None = None,
                 note: str = "", keyframes: list | None = None) -> None:
        msg = {"op": "say", "stage": stage, "text": text, "title": title, "duration_s": duration_s, "note": note}
        self.to_pid(pid, msg)
        d = {**msg, "who": self.name(pid)}
        if keyframes is not None:
            d["keyframes"] = keyframes
        self.to_displays(d)
        if stage in ("done", "error"):  # remembered for a display that connects mid-way
            self.say_now.pop(pid, None)
        else:
            self.say_now[pid] = d

    async def _say(self, pid: str, text: str) -> None:
        from openplay import say as SAY

        who = self.name(pid)
        title = SAY.make_title(text)
        task = asyncio.current_task()
        try:
            self.event("say", f'{who} asked: "{text}"', who)
            self._say_msg(pid, "thinking", text, title)
            claude = bool(os.environ.get("ANTHROPIC_API_KEY"))
            self.event("say", "Claude is choreographing…" if claude else "Picking a move from the library…", who)
            try:
                res = await SAY.author(text, timeout_s=SAY_TIMEOUT_S)
            except SAY.SayError as e:
                self._say_msg(pid, "error", text, title, note=str(e))
                self.status(pid, str(e), "error")
                return
            if self.say_thinking.get(pid) is not task or not self.lobby.is_active(pid) or self.handover:
                return  # stopped, or the turn ended while authoring
            self.say_thinking.pop(pid, None)
            for _ in range(80):  # something else running: wait for it (up to 20 s)
                if not (self.busy or self.pending):
                    break
                await asyncio.sleep(0.25)
            if self.busy or self.pending or not self.lobby.is_active(pid) or self.handover:
                self._say_msg(pid, "error", text, res["title"], note=self.busy_reason())
                return
            playable, title = res["playable"], res["title"]
            dur, keyframes = playable_info(playable)
            done = asyncio.get_running_loop().create_future()

            async def play():
                try:
                    await self._play_flow(pid, "say", title, playable, on_start=lambda: self._say_msg(
                        pid, "playing", text, title, dur, res.get("note", ""), keyframes))
                    self._say_msg(pid, "done", text, title, dur, res.get("note", ""))
                except (ActionError, Refused) as e:
                    self._say_msg(pid, "error", text, title, dur, _friendly(str(e)))
                    raise
                finally:
                    if not done.done():
                        done.set_result(None)

            self._begin(pid, "say", title, f"playing {title}", play())
            await done
        except asyncio.CancelledError:
            self._say_msg(pid, "error", text, title, note="stopped")
        finally:
            self.say_busy.discard(pid)
            if self.say_thinking.get(pid) is task:
                self.say_thinking.pop(pid, None)

    # ------------------------------------------------------------------ display protocol
    async def display_msg(self, c: Conn, m: Any) -> None:
        if not isinstance(m, dict) or not isinstance(m.get("op"), str):
            return c.send({"op": "refused", "reason": "messages are JSON objects with an 'op'"})
        op = m["op"]
        self._log({"kind": "operator_cmd", "op": op, **{k: v for k, v in m.items() if k in ("id", "s", "on", "pid")}})
        if op in ("approve", "deny"):
            p = self.pending
            if not p or p["id"] != m.get("id"):
                return c.send({"op": "refused", "reason": "nothing with that id is waiting"})
            if not p["future"].done():
                p["future"].set_result((op == "approve", "the operator said no" if op == "deny" else ""))
            self.event("operator", f"Operator {'approved' if op == 'approve' else 'declined'} {p['title']}", p["who"])
        elif op == "skip":
            a = self.lobby.active
            if a:
                self._lobby_events(self.lobby.skip())
        elif op == "extend":
            try:
                s = max(-3600.0, min(3600.0, float(m.get("s"))))
            except (TypeError, ValueError):
                return c.send({"op": "refused", "reason": "extend needs s (seconds)"})
            a = self.lobby.active
            if a:
                self.lobby.extend(s)
                self.event("operator", f"Operator {'added' if s >= 0 else 'took'} {abs(s):.0f} s "
                           f"{'to' if s >= 0 else 'from'} {self.name(a.pid)}'s turn", self.name(a.pid))
                self.push_lobby()
        elif op == "kick":
            pid = str(m.get("pid") or "")
            if pid in self.lobby.players:
                who = self.name(pid)
                evs = self.lobby.kick(pid)
                self.to_pid(pid, {"op": "status", "text": "The operator took you out of the queue", "kind": "info"})
                self.event("operator", f"Operator removed {who}", who)
                self._lobby_events(evs)
        elif op == "auto_play":
            self.auto_play_mode = "on" if m.get("on") else "off"
            self.event("operator", "Auto-play on: moves play without approval" if self.auto_play
                       else "Auto-play off: the operator approves each move")
            self.push_config()
        elif op == "pause":
            self.lobby.set_paused(bool(m.get("on")))
            self.event("operator", "Paused: no new turns start" if self.lobby.paused else "Resumed")
            self.push_config()
            self.push_lobby()
        elif op == "turn_s":
            try:
                self.lobby.set_turn_s(float(m.get("s")))
            except (TypeError, ValueError):
                return c.send({"op": "refused", "reason": "turn_s needs s (seconds)"})
            self.event("operator", f"Turns are now {self.lobby.turn_s:.0f} s")
            self.push_config()
        elif op in ("estop", "estop_reset"):
            try:
                await (self.arm.estop_on() if op == "estop" else self.arm.estop_reset())
            except Refused as e:
                return c.send({"op": "refused", "reason": str(e)})
            if op == "estop":
                self.event("estop", "E-STOP pressed on the laptop")
                if self.lobby.active:
                    self._cancel_player_work(self.lobby.active.pid, "E-STOP")
        elif op == "ping":
            c.send({"op": "pong"})
        else:
            c.send({"op": "refused", "reason": f"unknown op {op!r}"})

    def display_joined(self, c: Conn) -> None:
        self.displays.add(c)
        c.send(self.display_hello())
        c.send({"op": "lobby", **self.lobby.view_display()})
        p = self.pending
        c.send({"op": "pending", "pending": ({k: p[k] for k in ("id", "who", "title", "duration_s", "keyframes")}
                                             if p else None)})
        for d in list(self.say_now.values()):
            c.send(d)
        c.send({"op": "config", "auto_play": self.auto_play, "paused": self.lobby.paused, "turn_s": self.lobby.turn_s})
        for e in self.recent:
            c.send(e)

    # ------------------------------------------------------------------ apps
    def phone_app(self, key: str):
        app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None)

        def key_ok(q: str, c: str) -> tuple[bool, bool]:
            return bool(q) and hmac.compare_digest(q, key), bool(c) and hmac.compare_digest(c, key)

        @app.middleware("http")
        async def auth(request: Request, call_next):
            ok_q, ok_c = key_ok(request.query_params.get("k", ""), request.cookies.get(COOKIE, ""))
            if not (ok_q or ok_c):
                return PlainTextResponse("403: scan the QR code on the open play screen to join.", status_code=403)
            resp = await call_next(request)
            if ok_q and not ok_c:
                resp.set_cookie(COOKIE, key, max_age=12 * 3600, httponly=True, secure=True, samesite="strict")
            return resp

        @app.get("/")
        def page():
            return HTMLResponse(_page("play.html", "The phone page"), headers={"Cache-Control": "no-store"})

        @app.get("/favicon.ico")
        def favicon():
            return Response(status_code=204)

        @app.websocket("/ws")
        async def ws_endpoint(ws: WebSocket):
            ok_q, ok_c = key_ok(ws.query_params.get("k", ""), ws.cookies.get(COOKIE, ""))
            origin, host = ws.headers.get("origin", ""), ws.headers.get("host", "")
            if not (ok_q or ok_c) or (origin and origin.split("://", 1)[-1] != host):
                await ws.close(code=4403)
                return
            await ws.accept()
            await self._serve_ws(ws, "phone", self.phone_msg, None, self.phone_gone, WebSocketDisconnect)

        app.mount("/static", StaticFiles(directory=STATIC, check_dir=False), name="static")
        return app

    def display_app(self):
        app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None)

        def loopback(host: str | None) -> None:
            if (host or "") not in LOOPBACK:
                raise HTTPException(403, "the display is loopback-only")

        @app.get("/")
        def page(request: Request):
            loopback(request.client.host if request.client else None)
            return HTMLResponse(_page("display.html", "The display page"), headers={"Cache-Control": "no-store"})

        @app.get("/qr.png")
        def qr(request: Request):
            loopback(request.client.host if request.client else None)
            png = self.qr_png()
            if png is None:
                raise HTTPException(404, "no QR (qrcode/PIL missing)")
            return Response(png, media_type="image/png", headers={"Cache-Control": "no-store"})

        @app.get("/api/join")
        def join(request: Request):
            loopback(request.client.host if request.client else None)
            return {"url": self.join_url}

        @app.get("/favicon.ico")
        def favicon():
            return Response(status_code=204)

        @app.websocket("/display/ws")
        async def ws_endpoint(ws: WebSocket):
            remote = ws.client.host if ws.client else ""
            origin, host = ws.headers.get("origin", ""), ws.headers.get("host", "")
            # loopback only, and no other site's page in the laptop browser may drive the operator API
            if remote not in LOOPBACK or (origin and origin.split("://", 1)[-1] != host):
                await ws.close(code=4403)
                return
            await ws.accept()
            await self._serve_ws(ws, "display", self.display_msg, self.display_joined,
                                 lambda c: self.displays.discard(c), WebSocketDisconnect)

        if self.urdf_dir.is_dir():
            app.mount("/urdf", StaticFiles(directory=self.urdf_dir), name="urdf")
        app.mount("/static", StaticFiles(directory=STATIC, check_dir=False), name="static")
        return app

    async def _serve_ws(self, ws, kind: str, handle: Callable, joined: Callable | None, gone: Callable,
                        disconnect_exc: type) -> None:
        c = Conn(ws, kind)
        writer = asyncio.create_task(c.writer())
        if joined:
            joined(c)
        try:
            while c.alive:
                text = await ws.receive_text()
                if len(text) > MAX_MSG:
                    c.send({"op": "refused", "reason": "message too large"})
                    continue
                try:
                    m = json.loads(text)
                except ValueError:
                    c.send({"op": "refused", "reason": "bad JSON"})
                    continue
                try:
                    await handle(c, m)
                except Exception as e:  # never drop a connection over one bad message
                    log.exception("%s message", kind)
                    c.send({"op": "refused", "reason": f"internal error ({type(e).__name__})"})
        except (disconnect_exc, RuntimeError):
            pass
        finally:
            c.alive = False
            writer.cancel()
            gone(c)


def _zero_stream(m: dict) -> bool:
    """An all-zero stream (v all 0 or qd all 0, no grip): the page's "finger lifted"."""
    if m.get("grip") is not None:
        return False
    try:
        if m.get("qd") is not None:
            return all(float(x) == 0.0 for x in m["qd"])
        v = m.get("v") or {}
        return isinstance(v, dict) and all(float(x) == 0.0 for x in v.values())
    except (TypeError, ValueError):
        return False


def _friendly(reason: str) -> str:
    if "interrupted" in reason or "stopped by" in reason:
        return "Stopped"
    if reason.startswith("E-STOP") or reason == "E-STOP":
        return "E-STOP is on"
    return reason


def _page(file: str, what: str) -> str:
    p = STATIC / file
    try:
        return p.read_text(encoding="utf-8")
    except OSError:
        return PLACEHOLDER.format(what=what, file=file)


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------
def _uvicorn_server(config):
    import uvicorn

    class Server(uvicorn.Server):
        @contextlib.contextmanager
        def capture_signals(self):  # two servers in one loop: main() handles the signals
            yield

    return Server(config)


async def serve(args) -> None:
    import uvicorn

    pc = _phonecam()
    ips = pc.local_ips()
    host_ip = args.host_ip or pc.primary_ip()
    pc.ensure_cert(sorted(set(ips + [host_ip])))
    key = args.key or secrets.token_urlsafe(6)
    join_url = f"https://{host_ip}:{args.phone_port}/?k={key}"
    events = Path(args.events_log) if args.events_log else None
    op = OpenPlay(daemon_url=args.daemon, turn_s=args.turn, auto_play=args.auto_play, join_url=join_url,
                  urdf_dir=Path(args.urdf_dir), events_log=events, lin_mps=args.lin_mps,
                  ang_rps=args.ang_rps, joint_rps=args.joint_rps)
    # The room key is a secret: only on an interactive terminal (and on the loopback display).
    shown = join_url if os.isatty(1) else join_url.split("?k=")[0] + f"?k=<key: see http://127.0.0.1:{args.local_port}>"
    print(f"open play: phones join at {shown}", flush=True)
    print(f"open play: display at http://127.0.0.1:{args.local_port}  (daemon {args.daemon}, "
          f"turn {args.turn:.0f} s, auto-play {args.auto_play})", flush=True)
    if args.auto_play == "on":
        print("open play: auto-play is ON: playables play without the operator pressing Play", flush=True)

    servers = [
        _uvicorn_server(uvicorn.Config(op.phone_app(key), host=args.phone_host, port=args.phone_port,
                                       ssl_certfile=str(pc.CERT), ssl_keyfile=str(pc.KEY), log_level="warning",
                                       timeout_graceful_shutdown=1, ws_max_size=MAX_MSG * 2)),
        _uvicorn_server(uvicorn.Config(op.display_app(), host="127.0.0.1", port=args.local_port,
                                       log_level="warning", timeout_graceful_shutdown=1, ws_max_size=MAX_MSG * 2)),
    ]
    loop = asyncio.get_running_loop()

    def stop() -> None:
        for s in servers:
            if s.should_exit:
                s.force_exit = True
            s.should_exit = True

    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop)
    await op.start()
    try:
        results = await asyncio.gather(*(s.serve() for s in servers), return_exceptions=True)
        for r in results:
            if isinstance(r, BaseException) and not isinstance(r, asyncio.CancelledError):
                log.error("listener stopped: %r", r)
    finally:
        await op.shutdown()


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--daemon", default=DEFAULT_URL, help="armctl daemon WebSocket")
    ap.add_argument("--phone-host", default="0.0.0.0", help="phone listener bind address")
    ap.add_argument("--phone-port", type=int, default=8450)
    ap.add_argument("--local-port", type=int, default=8451, help="display + operator listener (127.0.0.1)")
    ap.add_argument("--key", default=None, help="room key (default: random per start)")
    ap.add_argument("--turn", type=float, default=600.0, help="turn length, s")
    ap.add_argument("--auto-play", choices=["on", "off", "auto"], default="auto",
                    help="press the daemon's Play for each playable (auto: on for the mock backend only)")
    ap.add_argument("--host-ip", default=None, help="IP in the join URL (default: detected)")
    ap.add_argument("--urdf-dir", default=str(DEFAULT_URDF_DIR), help="yam_description directory for /urdf")
    ap.add_argument("--events-log", default=str(EVENTS_LOG), help="JSON-lines event log ('' for none)")
    ap.add_argument("--lin-mps", type=float, default=LIN_MPS, help="stream v=1 linear speed, m/s")
    ap.add_argument("--ang-rps", type=float, default=ANG_RPS, help="stream v=1 angular speed, rad/s")
    ap.add_argument("--joint-rps", type=float, default=JOINT_RPS, help="stream qd=1 joint speed, rad/s")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s %(message)s",
                        datefmt="%H:%M:%S")
    asyncio.run(serve(args))


if __name__ == "__main__":
    main()
