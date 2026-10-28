"""openplay.lobby: the queue and turn rules for open play, with no I/O.

    lobby = Lobby(turn_s=600, clock=time.monotonic)
    lobby.hello("p1", "Asha"); lobby.join("p1")
    for ev in lobby.tick():            # call a few times a second
        ...                            # {"kind": "turn_start"|"turn_end"|"warning"|"dropped", "pid", ...}

Why a separate module: the rules (who plays next, when a turn ends, what the queue
shows) are where fairness bugs hide, so they are plain Python with an injectable clock
and are unit-tested without sockets or an arm (openplay/tests/test_lobby.py).

Rules (openplay/PROTOCOL.md, "Server rules"):
* players are keyed by a client-made ``pid``; reconnecting with the same pid keeps the
  place in the queue and the running turn.
* a turn ends at time-out (``time``), on leave or after ``disconnect_s`` disconnected
  (``left``), after ``idle_s`` without a command (``idle``), or by the operator
  (``skipped`` for skip, ``operator`` for kick).
* ``pause`` stops new turns from starting; the running one finishes.
* the next turn never starts in the tick that ended the previous one, and the server
  holds it back (``tick(can_start=False)``) while it returns the arm to ready.
* the next turn goes to the first *connected* player in the queue; a queued player who
  stays disconnected for ``drop_s`` loses their place.
"""
from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable

Clock = Callable[[], float]

TURN_S = 600.0
IDLE_S = 60.0
DISCONNECT_S = 20.0
WARN_S = 30.0
DROP_S = 120.0       # queued and disconnected this long: dropped from the queue
KICK_COOLDOWN_S = 300.0
MIN_TURN_S, MAX_TURN_S = 5.0, 3600.0


@dataclass
class Player:
    pid: str
    name: str
    connected: bool = True
    disconnected_at: float | None = None
    kicked_until: float = 0.0


@dataclass
class Turn:
    pid: str
    started: float
    ends_at: float
    last_cmd: float
    warned: bool = False


@dataclass
class Lobby:
    turn_s: float = TURN_S
    idle_s: float = IDLE_S
    disconnect_s: float = DISCONNECT_S
    warn_s: float = WARN_S
    drop_s: float = DROP_S
    clock: Clock = time.monotonic
    paused: bool = False
    players: dict[str, Player] = field(default_factory=dict)
    queue: list[str] = field(default_factory=list)       # waiting, in order (active not included)
    active: Turn | None = None
    _history: deque = field(default_factory=lambda: deque(maxlen=8))  # recent turn lengths (s)

    # ------------------------------------------------------------------ players
    def hello(self, pid: str, name: str) -> Player:
        p = self.players.get(pid)
        if p is None:
            p = self.players[pid] = Player(pid, name)
        else:
            p.name = name or p.name
        p.connected, p.disconnected_at = True, None
        return p

    def disconnect(self, pid: str) -> None:
        p = self.players.get(pid)
        if p and p.connected:
            p.connected, p.disconnected_at = False, self.clock()

    def name(self, pid: str | None) -> str | None:
        p = self.players.get(pid) if pid else None
        return p.name if p else None

    def connected_count(self) -> int:
        return sum(1 for p in self.players.values() if p.connected)

    # ------------------------------------------------------------------ queue
    def join(self, pid: str) -> str | None:
        """Enter the queue. Returns None, or a refusal reason."""
        p = self.players.get(pid)
        if p is None:
            return "say hello first"
        if self.clock() < p.kicked_until:
            return "the operator removed you from the queue; try again in a few minutes"
        if self.is_active(pid) or pid in self.queue:
            return None
        self.queue.append(pid)
        return None

    def leave(self, pid: str) -> list[dict]:
        """Leave the queue; leaving during your turn ends it (reason ``left``)."""
        if pid in self.queue:
            self.queue.remove(pid)
        if self.is_active(pid):
            return [self._end("left")]
        return []

    def kick(self, pid: str, cooldown_s: float = KICK_COOLDOWN_S) -> list[dict]:
        p = self.players.get(pid)
        if p:
            p.kicked_until = self.clock() + cooldown_s
        if pid in self.queue:
            self.queue.remove(pid)
        if self.is_active(pid):
            return [self._end("operator")]
        return []

    def forget(self, pid: str) -> None:
        """Drop a player entirely (no connection, not queued, not active)."""
        if not self.is_active(pid) and pid not in self.queue:
            self.players.pop(pid, None)

    # ------------------------------------------------------------------ turn
    def is_active(self, pid: str | None) -> bool:
        return self.active is not None and pid is not None and self.active.pid == pid

    def touch(self, pid: str) -> None:
        """A command from ``pid``: resets the idle timer if it's their turn."""
        if self.is_active(pid):
            self.active.last_cmd = self.clock()

    def remaining(self) -> float | None:
        if self.active is None:
            return None
        return max(0.0, self.active.ends_at - self.clock())

    def extend(self, s: float) -> None:
        if self.active is not None:
            now = self.clock()
            self.active.ends_at = max(now + 1.0, self.active.ends_at + float(s))
            if self.active.ends_at - now > self.warn_s:
                self.active.warned = False

    def skip(self) -> list[dict]:
        return [self._end("skipped")] if self.active else []

    def end_operator(self) -> list[dict]:
        return [self._end("operator")] if self.active else []

    def set_turn_s(self, s: float) -> None:
        """New turn length (applies to turns that start from now on)."""
        self.turn_s = min(max(float(s), MIN_TURN_S), MAX_TURN_S)

    def set_paused(self, on: bool) -> None:
        self.paused = bool(on)

    def _end(self, reason: str) -> dict:
        t = self.active
        assert t is not None
        self.active = None
        self._history.append(max(0.0, self.clock() - t.started))
        return {"kind": "turn_end", "pid": t.pid, "reason": reason}

    def _start(self, pid: str) -> dict:
        now = self.clock()
        self.queue.remove(pid)
        self.active = Turn(pid, now, now + self.turn_s, now)
        return {"kind": "turn_start", "pid": pid, "remaining_s": self.turn_s}

    def next_pid(self) -> str | None:
        for pid in self.queue:
            p = self.players.get(pid)
            if p and p.connected:
                return pid
        return None

    def tick(self, can_start: bool = True) -> list[dict]:
        """Apply the time rules. Returns events in the order they happened."""
        now = self.clock()
        out: list[dict] = []
        t = self.active
        if t is not None:
            p = self.players.get(t.pid)
            if now >= t.ends_at:
                out.append(self._end("time"))
            elif p is None or (not p.connected and p.disconnected_at is not None
                               and now - p.disconnected_at >= self.disconnect_s):
                out.append(self._end("left"))
            elif now - t.last_cmd >= self.idle_s:
                out.append(self._end("idle"))
            elif not t.warned and t.ends_at - now <= self.warn_s:
                t.warned = True
                out.append({"kind": "warning", "pid": t.pid, "remaining_s": t.ends_at - now})
        for pid in list(self.queue):
            p = self.players.get(pid)
            if p is None or (not p.connected and p.disconnected_at is not None
                             and now - p.disconnected_at >= self.drop_s):
                self.queue.remove(pid)
                out.append({"kind": "dropped", "pid": pid})
        # A turn that just ended never hands over in the same tick: the server first stops
        # the arm and returns it to ready, then calls tick() again.
        ended = any(e["kind"] == "turn_end" for e in out)
        if self.active is None and can_start and not self.paused and not ended:
            pid = self.next_pid()
            if pid is not None:
                out.append(self._start(pid))
        return out

    # ------------------------------------------------------------------ views
    def avg_turn_s(self) -> float:
        """Expected length of a turn: recent turns (people often leave early), else turn_s."""
        if not self._history:
            return self.turn_s
        return min(self.turn_s, sum(self._history) / len(self._history))

    def position(self, pid: str) -> int | None:
        return self.queue.index(pid) + 1 if pid in self.queue else None

    def eta(self, pid: str) -> float | None:
        """Seconds until ``pid``'s turn starts (None if not queued, or paused)."""
        pos = self.position(pid)
        if pos is None or self.paused:
            return None
        ahead = self.queue[:pos - 1]
        connected_ahead = sum(1 for q in ahead if (p := self.players.get(q)) and p.connected)
        rem = self.remaining() or 0.0
        return round(rem + connected_ahead * self.avg_turn_s(), 1)

    def view_for(self, pid: str | None) -> dict:
        """The phone ``lobby`` message body."""
        a = self.active
        return {
            "queue": [{"name": self.name(q), "you": q == pid} for q in self.queue],
            "position": self.position(pid) if pid else None,
            "active": ({"name": self.name(a.pid), "remaining_s": round(self.remaining() or 0.0)} if a else None),
            "you_active": self.is_active(pid),
            "eta_s": self.eta(pid) if pid else None,
        }

    def view_display(self) -> dict:
        """The display ``lobby`` message body."""
        a = self.active
        return {
            "queue": [{"pid": q, "name": self.name(q)} for q in self.queue],
            "active": ({"pid": a.pid, "name": self.name(a.pid), "remaining_s": round(self.remaining() or 0.0)}
                       if a else None),
            "players": self.connected_count(),
        }
