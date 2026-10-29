"""openplay.eventlog: a time-synced event stream for open play (phone -> server -> arm).

Why: on the real arm Tilt and Sticks "often disconnect" while buttons work. The daemon logs
"deadman: no setpoint for ... ms" and stream refusals, but not which phone message was late,
whether the phone sent it late or the network delivered it late, or what the server did with
it. This module records every step on one clock (the server's) so a stream gap can be pinned
on the phone, the network, the server or the arm.

    log = EventLog(path, ttl_ms=300)          # JSON lines + an in-memory ring
    log.emit("phone", "rx", cid=..., op="stream", ...)
    log.episodes                              # per streaming episode stats (StreamTracker)

    .venv/bin/python -m openplay.eventlog summarize experiments/<day>/logs/openplay-events-<ts>.jsonl

Record format: one JSON object per line, always ``n`` (sequence in this file), ``t`` (server
unix time, s), ``lane`` (``phone`` | ``server`` | ``arm`` | ``conn``) and ``kind``. The rest
depends on the kind; see openplay/PROTOCOL.md "Event log". Nothing here sends anything to the
arm; it only watches.
"""
from __future__ import annotations

import argparse
import collections
import datetime as dt
import json
import logging
import math
import sys
import time
from pathlib import Path
from typing import Any, Callable, Iterable

log = logging.getLogger("openplay.eventlog")

REPO = Path(__file__).resolve().parent.parent
RING = 20000
EPISODE_IDLE_S = 2.0        # an episode with no stream for this long is closed ("timeout")
EPISODE_SPLIT_MS = 1000.0   # without the phone's ``ep``: a gap this long starts a new episode
STATE_SAMPLE_S = 0.5        # arm state samples at 2 Hz
GAP_BINS_MS = (50, 100, 150, 200, 300, 400, 500, 1000)
PHONE_LATE_FRAC = 0.8       # a gap whose phone-side send gap is >= 0.8 x ttl: the phone sent late
LATENCY_KEEP_S = 300.0


# --------------------------------------------------------------------------
# Clock sync (NTP style)
# --------------------------------------------------------------------------
class ClockSync:
    """Offset of a phone clock against the server clock from ping/pong samples.

    A sample is (t0 phone send, t1 server receive, t2 server send, t3 phone receive), phone
    times in the phone's ms, server times in server unix ms. offset = phone - server:
    ``((t0 - t1) + (t3 - t2)) / 2``; rtt = ``(t3 - t0) - (t2 - t1)``. The estimate is the
    sample with the smallest rtt of the last ``window`` (NTP's clock filter): its error is at
    most rtt/2, which is the reported uncertainty."""

    def __init__(self, window: int = 16):
        self.samples: collections.deque[tuple[float, float, float]] = collections.deque(maxlen=window)

    def add(self, t0: Any, t1: Any, t2: Any, t3: Any) -> dict | None:
        try:
            t0, t1, t2, t3 = (float(x) for x in (t0, t1, t2, t3))
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(x) for x in (t0, t1, t2, t3)):
            return None
        rtt = (t3 - t0) - (t2 - t1)
        if rtt < -1.0 or t2 < t1 or rtt > 60000:  # a clock jumped mid-sample, or nonsense
            return None
        rtt = max(rtt, 0.0)
        self.samples.append((rtt, ((t0 - t1) + (t3 - t2)) / 2.0, t1))
        return self.estimate()

    def estimate(self) -> dict | None:
        if not self.samples:
            return None
        rtt, off, t1 = min(self.samples, key=lambda s: s[0])
        return {"offset_ms": round(off, 1), "rtt_ms": round(rtt, 1), "unc_ms": round(rtt / 2.0, 1),
                "n": len(self.samples), "at": round(t1 / 1000.0, 3)}

    @property
    def offset_ms(self) -> float | None:
        e = self.estimate()
        return None if e is None else e["offset_ms"]

    def to_server_s(self, t_phone_ms: Any) -> float | None:
        """A phone timestamp (ms) on the server clock (unix s), or None without an estimate."""
        e = self.estimate()
        try:
            return None if e is None else (float(t_phone_ms) - e["offset_ms"]) / 1000.0
        except (TypeError, ValueError):
            return None


# --------------------------------------------------------------------------
# Gap statistics
# --------------------------------------------------------------------------
def percentile(xs: Iterable[float], p: float) -> float | None:
    """Linear-interpolated percentile (p in 0..100) of ``xs``; None for no data."""
    s = sorted(float(x) for x in xs)
    if not s:
        return None
    k = (len(s) - 1) * p / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    return s[lo] if lo == hi else s[lo] + (s[hi] - s[lo]) * (k - lo)


def histogram(xs: Iterable[float], bins: tuple = GAP_BINS_MS) -> dict[str, int]:
    """Counts per bin: ``"<50"``, ``"50-100"``, ..., ``">=1000"``."""
    labels = [f"<{bins[0]}"] + [f"{a}-{b}" for a, b in zip(bins, bins[1:])] + [f">={bins[-1]}"]
    out = dict.fromkeys(labels, 0)
    for x in xs:
        i = next((k for k, b in enumerate(bins) if x < b), len(bins))
        out[labels[i]] += 1
    return out


def gap_stats(gaps_ms: list[float], ttl_ms: float) -> dict:
    r = lambda v: None if v is None else round(v, 1)  # noqa: E731
    return {"n": len(gaps_ms), "p50": r(percentile(gaps_ms, 50)), "p95": r(percentile(gaps_ms, 95)),
            "max": r(max(gaps_ms) if gaps_ms else None),
            "over_ttl": sum(1 for g in gaps_ms if g > ttl_ms), "hist": histogram(gaps_ms)}


# --------------------------------------------------------------------------
# Streaming episodes (pointer down -> up), built from records only (live and replay agree)
# --------------------------------------------------------------------------
class Episode:
    def __init__(self, eid: int, rec: dict):
        self.id = eid
        self.cid, self.pid, self.ep = rec.get("cid"), rec.get("pid"), rec.get("ep")
        self.kind = "qd" if (rec.get("sum") or {}).get("qd") is not None else "v"
        self.t0 = self.t_last = float(rec["t"])
        self.t_ph_last: float | None = None
        self.n = 0
        self.arrival: list[float] = []
        self.sent: list[float] = []
        self.big: list[dict] = []
        self.acts: collections.Counter = collections.Counter()
        self.refusals: collections.Counter = collections.Counter()
        self.seq_gaps = 0
        self.skipped = 0
        self.owd: list[float] = []
        self.deadman_pending: dict | None = None
        self.deadman_in_gaps = 0
        self.deadman_other = 0
        self.rate_ms = rec.get("rate_ms")

    def add(self, rec: dict, ttl_ms: float) -> None:
        t = float(rec["t"])
        tph = rec.get("t_ph")
        if self.n:
            gap = (t - self.t_last) * 1000.0
            self.arrival.append(gap)
            pg = None
            if tph is not None and self.t_ph_last is not None:
                pg = float(tph) - self.t_ph_last
                self.sent.append(pg)
            dead = self.deadman_pending
            if gap > ttl_ms or dead:
                skip = int(rec.get("skip") or 0)
                if skip:
                    cause = "phone-gate"      # the phone chose not to send (tilt start-pose gate)
                elif pg is None:
                    cause = "unknown"         # an old phone page: no ``t``
                elif pg >= PHONE_LATE_FRAC * ttl_ms:
                    cause = "phone"           # the phone sent late (timer throttled, page hidden, busy)
                else:
                    cause = "network"         # sent on time, arrived late (Wi-Fi, TCP retransmit)
                self.big.append({"t": round(self.t_last, 3), "gap_ms": round(gap, 1),
                                 "phone_gap_ms": None if pg is None else round(pg, 1), "cause": cause,
                                 "deadman": bool(dead), "seq": rec.get("seq"), "skip": skip})
                if dead:
                    self.deadman_in_gaps += 1
            self.deadman_pending = None
        self.n += 1
        self.t_last = t
        if tph is not None:
            self.t_ph_last = float(tph)
        act = rec.get("act") or "?"
        self.acts[act] += 1
        if act in ("refuse", "drop") and rec.get("reason"):
            self.refusals[_short(rec["reason"])] += 1
        self.seq_gaps += int(rec.get("seq_gap") or 0)
        self.skipped += int(rec.get("skip") or 0)
        if rec.get("owd_ms") is not None:
            self.owd.append(float(rec["owd_ms"]))

    def summary(self, reason: str, ttl_ms: float) -> dict:
        dur = max(self.t_last - self.t0, 1e-6)
        trailing = 1 if self.deadman_pending else 0
        return {
            "episode": self.id, "cid": self.cid, "pid": self.pid, "ep": self.ep, "stream": self.kind,
            "t0": round(self.t0, 3), "t1": round(self.t_last, 3), "dur_s": round(self.t_last - self.t0, 3),
            "n": self.n, "rate_hz": round((self.n - 1) / dur, 1) if self.n > 1 else None,
            "phone_rate_ms": self.rate_ms, "ttl_ms": ttl_ms,
            "arrival": gap_stats(self.arrival, ttl_ms), "sent": gap_stats(self.sent, ttl_ms),
            "gaps_over_ttl": self.big, "deadman_in_gaps": self.deadman_in_gaps,
            "deadman_trailing": trailing, "deadman_other": self.deadman_other,
            "acts": dict(self.acts), "refusals": dict(self.refusals), "seq_gaps": self.seq_gaps,
            "skipped_ticks": self.skipped,
            "owd_ms": {"p50": _r1(percentile(self.owd, 50)), "p95": _r1(percentile(self.owd, 95)),
                       "max": _r1(max(self.owd) if self.owd else None)} if self.owd else None,
            "end": reason,
        }


class StreamTracker:
    """Feeds on event records; returns ``episode`` summaries as episodes end.

    An episode is one hold of a live control: phone ``stream`` messages with the same ``ep``
    (the phone's hold counter; without it, a > 1 s gap splits), ended by the all-zero release
    (``why`` says why the phone let go), a new ``ep``, the connection closing, or 2 s of silence.
    ``arm/stream_stop`` records (the daemon's ``streaming`` flag dropping) with cause
    ``expired`` are deadman stops: while an episode is open they are pinned on the gap in
    progress; after a release they are the benign "release expiry" the daemon also logs as
    "deadman"."""

    def __init__(self, ttl_ms: float = 300.0):
        self.ttl_ms = float(ttl_ms)
        self.open: dict[Any, Episode] = {}
        self.rate_ms: dict[Any, Any] = {}
        self.done: collections.deque[dict] = collections.deque(maxlen=500)
        self.totals = collections.Counter()
        self._ids = 0
        self._last_cid: Any = None

    def observe(self, rec: dict) -> list[dict]:
        out = self.tick(float(rec.get("t") or time.time()))
        lane, kind = rec.get("lane"), rec.get("kind")
        if lane == "conn" and kind == "hello":
            self.rate_ms[rec.get("cid")] = rec.get("rate_ms")
        elif lane == "conn" and kind == "close":
            out += self._close(rec.get("cid"), "disconnect")
        elif lane == "phone" and kind == "rx" and rec.get("op") == "stream":
            out += self._stream(rec)
        elif lane == "arm" and kind == "stream_stop":
            self.totals[f"stream_stop:{rec.get('cause')}"] += 1
            ep = self.open.get(self._last_cid)
            if rec.get("cause") == "expired":
                if ep is not None:
                    ep.deadman_pending = {"t": rec["t"]}
                    self.totals["deadman_mid_episode"] += 1
                else:
                    self.totals["deadman_after_release"] += 1
            elif ep is not None:
                ep.deadman_other += 1
        return out

    def _stream(self, rec: dict) -> list[dict]:
        out: list[dict] = []
        cid = rec.get("cid")
        ep = self.open.get(cid)
        zero = bool(rec.get("zero"))
        if ep is not None:
            new_ep = rec.get("ep") is not None and ep.ep is not None and rec.get("ep") != ep.ep
            split = rec.get("ep") is None and (float(rec["t"]) - ep.t_last) * 1000.0 > EPISODE_SPLIT_MS
            if new_ep or split:
                out += self._close(cid, "new hold")
                ep = None
        if ep is None:
            if zero:
                self.totals["zero_outside_episode"] += 1
                return out
            self._ids += 1
            ep = Episode(self._ids, {**rec, "rate_ms": self.rate_ms.get(cid)})
            self.open[cid] = ep
        ep.add(rec, self.ttl_ms)
        self._last_cid = cid
        if zero:
            out += self._close(cid, f"release ({rec.get('why') or 'up'})")
        return out

    def tick(self, now: float) -> list[dict]:
        out: list[dict] = []
        for cid, ep in list(self.open.items()):
            if now - ep.t_last > EPISODE_IDLE_S:
                out += self._close(cid, "timeout (no release)")
        return out

    def close_all(self, reason: str = "end of log") -> list[dict]:
        out: list[dict] = []
        for cid in list(self.open):
            out += self._close(cid, reason)
        return out

    def _close(self, cid: Any, reason: str) -> list[dict]:
        ep = self.open.pop(cid, None)
        if ep is None:
            return []
        s = ep.summary(reason, self.ttl_ms)
        self.done.append(s)
        self.totals["episodes"] += 1
        self.totals["gaps_over_ttl"] += len(s["gaps_over_ttl"])
        self.totals["deadman_in_gaps"] += s["deadman_in_gaps"]
        for g in s["gaps_over_ttl"]:
            self.totals[f"cause:{g['cause']}"] += 1
        return [s]

    def stats(self) -> dict:
        return {"ttl_ms": self.ttl_ms, "totals": dict(self.totals), "open": len(self.open),
                "episodes": list(self.done)[-50:]}


# --------------------------------------------------------------------------
# The armctl wire: latencies, state samples, inferred deadman stops
# --------------------------------------------------------------------------
class ArmTracer:
    """Watches every armctl message open play sends and receives (ArmBridge ``trace`` hook)
    and turns them into records: ``tx`` / ``rx`` (with ``ack_ms`` / ``done_ms`` latency per id),
    ``state`` (q, mode, streaming at 2 Hz) and ``stream_stop`` when the daemon's ``streaming``
    flag drops. The daemon does not tell the client about deadman stops, so they are inferred:
    cause ``expired`` (the ttl ran out: a deadman), ``refused`` (a stream refusal just before)
    or ``do`` (a discrete command, e.g. stop, just before). ``after_zero`` says the last
    setpoint was all zeros (the phone's release: harmless)."""

    def __init__(self, emit: Callable[..., Any], clock: Callable[[], float] = time.time):
        self.emit, self.clock = emit, clock
        self.sent: dict[Any, dict] = {}
        self.streaming = False
        self.last_stream: tuple[float, bool] | None = None    # (t, zero)
        self.last_refusal: tuple[float, str] | None = None
        self.last_do: tuple[float, str] | None = None
        self.last_sample = 0.0
        self.n_stream_tx = 0

    def __call__(self, direction: str, msg: dict) -> None:
        try:
            (self.tx if direction == "tx" else self.rx)(msg)
        except Exception:  # never break the arm connection over a log
            log.exception("arm trace")

    def tx(self, msg: dict) -> None:
        now = self.clock()
        op = msg.get("op")
        if op == "stream":
            vals = msg.get("qd") if msg.get("qd") is not None else list((msg.get("v") or {}).values())
            zero = all(float(x) == 0.0 for x in vals) and msg.get("grip") is None
            self.last_stream = (now, zero)
            self.n_stream_tx += 1
            self.emit("arm", "tx", op="stream", ttl_ms=msg.get("ttl_ms"), zero=zero,
                      sum=_stream_sum(msg))
            return
        if op == "do":
            self.last_do = (now, str(msg.get("cmd")))
        if msg.get("id") is not None:
            self.sent[msg["id"]] = {"t": now, "op": op, "cmd": msg.get("cmd")}
            if len(self.sent) > 200:
                for k in [k for k, v in self.sent.items() if now - v["t"] > LATENCY_KEEP_S]:
                    self.sent.pop(k, None)
        self.emit("arm", "tx", op=op, id=msg.get("id"), cmd=msg.get("cmd"),
                  **({"name": msg["name"]} if isinstance(msg.get("name"), str) else {}))

    def rx(self, msg: dict) -> None:
        now = self.clock()
        op = msg.get("op")
        if op == "state":
            s = bool(msg.get("streaming"))
            if self.streaming and not s:
                self._stream_stop(now)
            self.streaming = s
            if now - self.last_sample >= STATE_SAMPLE_S:
                self.last_sample = now
                q = msg.get("q")
                self.emit("arm", "state", q=[round(float(x), 3) for x in q] if isinstance(q, list) else q,
                          mode=msg.get("mode"), streaming=s, busy=msg.get("busy"), estop=msg.get("estop"),
                          fault=bool(msg.get("fault")), t_daemon=msg.get("t"))
            return
        rec: dict[str, Any] = {"op": op}
        mid = msg.get("id")
        if op == "refused":
            rec["reason"] = msg.get("reason")
            for k in ("stream", "edge", "joint"):
                if msg.get(k) is not None:
                    rec[k] = msg[k]
            if msg.get("stream") or mid is None:
                self.last_refusal = (now, str(msg.get("reason")))
        if op == "done":
            rec["cmd"] = msg.get("cmd")
        if mid is not None:
            rec["id"] = mid
            sent = self.sent.get(mid)
            if sent is not None:
                rec["req"] = sent["cmd"] or sent["op"]
                lat = round((now - sent["t"]) * 1000.0, 1)
                if op == "done":
                    rec["done_ms"] = lat
                    self.sent.pop(mid, None)
                else:
                    rec["ack_ms"] = lat
                    if op == "refused" or sent["op"] not in ("do", "playable.play"):
                        self.sent.pop(mid, None)
        elif op in ("lease", "estop", "fault"):
            rec.update({k: v for k, v in msg.items() if k in ("client", "latched", "reason", "message")})
        self.emit("arm", "rx", **rec)

    def _stream_stop(self, now: float) -> None:
        since = None if self.last_stream is None else round((now - self.last_stream[0]) * 1000.0, 1)
        zero = bool(self.last_stream and self.last_stream[1])
        if self.last_refusal and now - self.last_refusal[0] < 0.6:
            cause, why = "refused", self.last_refusal[1]
        elif self.last_do and now - self.last_do[0] < 0.6:
            cause, why = "do", self.last_do[1]
        else:
            cause, why = "expired", None
        self.emit("arm", "stream_stop", cause=cause, why=why, after_zero=zero, since_last_ms=since)


# --------------------------------------------------------------------------
# The log
# --------------------------------------------------------------------------
class EventLog:
    """JSON lines (one file per server start) + an in-memory ring + the stream tracker."""

    def __init__(self, path: Path | str | None = None, *, ttl_ms: float = 300.0, ring: int = RING,
                 on_episode: Callable[[dict], Any] | None = None, clock: Callable[[], float] = time.time):
        self.path = Path(path) if path else None
        self.clock = clock
        self.ring: collections.deque[dict] = collections.deque(maxlen=ring)
        self.tracker = StreamTracker(ttl_ms)
        self.on_episode = on_episode
        self.n = 0
        self._f = None
        self._warned = False
        if self.path:
            try:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                self._f = self.path.open("a", encoding="utf-8", buffering=1)
            except OSError as e:
                log.warning("event log %s: %s (ring buffer only)", self.path, e)

    def emit(self, lane: str, kind: str, t: float | None = None, **fields: Any) -> dict:
        self.n += 1
        rec = {"n": self.n, "t": round(self.clock() if t is None else t, 4), "lane": lane, "kind": kind,
               **{k: v for k, v in fields.items() if v is not None}}
        self._write(rec)
        if kind != "episode":
            for s in self.tracker.observe(rec):
                self._episode(s)
        return rec

    def tick(self) -> None:
        for s in self.tracker.tick(self.clock()):
            self._episode(s)

    def _episode(self, s: dict) -> None:
        self.n += 1
        self._write({"n": self.n, "t": round(self.clock(), 4), "lane": "server", "kind": "episode", **s})
        if self.on_episode:
            with _suppress():
                self.on_episode(s)

    def _write(self, rec: dict) -> None:
        self.ring.append(rec)
        if self._f is not None:
            try:
                self._f.write(json.dumps(rec, ensure_ascii=False, default=str) + "\n")
            except (OSError, ValueError) as e:
                if not self._warned:
                    self._warned = True
                    log.warning("event log write: %s", e)

    def since(self, n: int = 0, limit: int = 2000, pid: str | None = None, op: str | None = None) -> list[dict]:
        out = []
        for r in self.ring:
            if r["n"] <= n:
                continue
            if pid and r.get("pid") not in (None, pid):
                continue
            if op and op not in (str(r.get("op") or ""), r["kind"]):
                continue
            out.append(r)
        return out[-limit:]

    def stats(self) -> dict:
        return {"file": str(self.path) if self.path else None, "records": self.n, **self.tracker.stats()}

    def close(self) -> None:
        for s in self.tracker.close_all("server stopped"):
            self._episode(s)
        if self._f is not None:
            with _suppress():
                self._f.close()
            self._f = None


def default_path(now: dt.datetime | None = None, experiments: Path = REPO / "experiments") -> Path:
    """experiments/<today>-*/logs/openplay-events-<HHMMSS>.jsonl: today's experiment folder
    with "openplay" in its name, else the most recently changed one dated today, else a new
    ``<today>-openplay``."""
    now = now or dt.datetime.now()
    day = now.strftime("%Y-%m-%d")
    dirs = [d for d in experiments.glob(f"{day}-*") if d.is_dir()] if experiments.is_dir() else []
    pick = next((d for d in sorted(dirs) if "openplay" in d.name), None)
    if pick is None and dirs:
        pick = max(dirs, key=lambda d: d.stat().st_mtime)
    if pick is None:
        pick = experiments / f"{day}-openplay"
    return pick / "logs" / f"openplay-events-{now.strftime('%Y%m%d-%H%M%S')}.jsonl"


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------
class _suppress:
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        if exc[0] is not None and issubclass(exc[0], Exception):
            log.debug("suppressed", exc_info=exc)
            return True
        return False


def _r1(v: float | None) -> float | None:
    return None if v is None else round(v, 1)


def _short(reason: Any, n: int = 70) -> str:
    s = str(reason)
    if "needs the arm unfolded" in s:
        return "needs the arm unfolded (j2/j3 too low)"
    if "edge of reach" in s:
        return "edge of reach"
    return s[:n]


def _stream_sum(m: dict) -> dict:
    """A compact payload summary for a stream message."""
    out: dict[str, Any] = {}
    if m.get("qd") is not None:
        try:
            out["qd"] = [round(float(x), 2) for x in m["qd"]]
        except (TypeError, ValueError):
            out["qd"] = "?"
    else:
        v = m.get("v") if isinstance(m.get("v"), dict) else {}
        try:
            out["v"] = {k: round(float(x), 2) for k, x in v.items() if float(x) != 0.0}
        except (TypeError, ValueError):
            out["v"] = "?"
    if m.get("grip") is not None:
        out["grip"] = m["grip"]
    return out


def payload_summary(m: dict) -> dict | None:
    op = m.get("op")
    if op == "stream":
        return _stream_sum(m)
    keep = {k: m[k] for k in ("name", "value", "i", "style", "from", "to", "uci", "opponent", "color")
            if k in m and not isinstance(m[k], (dict, list))}
    if op == "say":
        keep["text"] = str(m.get("text") or "")[:60]
    return keep or None


# --------------------------------------------------------------------------
# CLI: summarize a log
# --------------------------------------------------------------------------
def read_records(path: Path | str) -> list[dict]:
    out = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                with _suppress():
                    out.append(json.loads(line))
    return out


def summarize(records: list[dict], ttl_ms: float | None = None) -> dict:
    """Replay the records through a fresh StreamTracker (episodes recomputed, not trusted
    from the file) and collect connection, sync and refusal facts."""
    if ttl_ms is None:
        ttl_ms = next((float(r["ttl_ms"]) for r in records if r.get("kind") == "config" and r.get("ttl_ms")), 300.0)
    tr = StreamTracker(ttl_ms)
    eps: list[dict] = []
    closes, syncs, diags, lags = [], {}, collections.Counter(), []
    refusals: collections.Counter = collections.Counter()
    for r in records:
        if r.get("kind") == "episode":
            continue
        eps += tr.observe(r)
        lane, kind = r.get("lane"), r.get("kind")
        if lane == "conn" and kind == "close":
            closes.append(r)
        elif lane == "conn" and kind == "sync":
            syncs[r.get("pid") or r.get("cid")] = r
        elif lane == "phone" and kind == "diag":
            diags[str(r.get("ev"))] += 1
        elif lane == "server" and kind == "loop_lag":
            lags.append(r.get("lag_ms"))
        elif lane == "arm" and kind == "rx" and r.get("op") == "refused":
            refusals[_short(r.get("reason"))] += 1
    eps += tr.close_all()
    arr = [g for e in eps for g in _all_gaps(e)]
    return {"ttl_ms": ttl_ms, "records": len(records), "episodes": eps, "totals": dict(tr.totals),
            "closes": closes, "sync": syncs, "diag": dict(diags), "loop_lag_ms": lags,
            "arm_refusals": dict(refusals), "all_big_gaps": arr}


def _all_gaps(e: dict) -> list[dict]:
    return [{**g, "episode": e["episode"], "pid": e["pid"]} for g in e["gaps_over_ttl"]]


def verdict(s: dict) -> str:
    t = s["totals"]
    big = t.get("gaps_over_ttl", 0)
    if not s["episodes"]:
        return "no streaming episodes in this log"
    lines = []
    if big == 0:
        lines.append(f"no stream gap exceeded the ttl ({s['ttl_ms']:.0f} ms): jitter is not what stops the arm here")
    else:
        causes = {k.split(":", 1)[1]: v for k, v in t.items() if k.startswith("cause:")}
        top = max(causes, key=causes.get)
        lines.append(f"{big} gaps > ttl {s['ttl_ms']:.0f} ms over {t.get('episodes', 0)} holds; "
                     f"{t.get('deadman_in_gaps', 0)} became deadman stops; causes " +
                     ", ".join(f"{k} {v}" for k, v in sorted(causes.items(), key=lambda kv: -kv[1])) +
                     f" (mostly {top})")
    ends = collections.Counter(e["end"].split(" (")[0] if e["end"].startswith("release") else e["end"]
                               for e in s["episodes"])
    why = collections.Counter(e["end"] for e in s["episodes"] if e["end"].startswith("release"))
    lines.append("holds ended by: " + ", ".join(f"{k} {v}" for k, v in ends.most_common()) +
                 ("; releases: " + ", ".join(f"{k} {v}" for k, v in why.most_common()) if why else ""))
    if s["arm_refusals"]:
        lines.append("daemon stream refusals: " + ", ".join(f"{k} x{v}" for k, v in
                                                            collections.Counter(s["arm_refusals"]).most_common(4)))
    rel = t.get("deadman_after_release", 0)
    if rel:
        lines.append(f"{rel} daemon 'deadman' lines are release expiries (the arm was already told 0): harmless")
    return "\n".join("  " + x for x in lines)


def print_summary(s: dict, out=None, top: int = 12) -> None:
    out = out or sys.stdout
    p = lambda *a: print(*a, file=out)  # noqa: E731
    p(f"records {s['records']}, ttl {s['ttl_ms']:.0f} ms, episodes {len(s['episodes'])}")
    if s["sync"]:
        p("\nclock sync (phone - server, min-RTT sample):")
        for who, r in s["sync"].items():
            p(f"  {who}: offset {r.get('offset_ms')} ms ± {r.get('unc_ms')} ms (rtt {r.get('rtt_ms')} ms, n {r.get('n')})")
    if s["episodes"]:
        p("\nholds (one line each): arrival gaps at the server vs send gaps on the phone, ms")
        p(f"  {'#':>4} {'pid':10} {'kind':4} {'dur s':>6} {'n':>4} {'Hz':>5} {'arr p50/p95/max':>18} "
          f"{'sent p50/p95/max':>18} {'>ttl':>4} {'dead':>4}  end")
        for e in s["episodes"]:
            a, sn = e["arrival"], e["sent"]
            f = lambda g: f"{_fmt(g['p50'])}/{_fmt(g['p95'])}/{_fmt(g['max'])}"  # noqa: E731
            p(f"  {e['episode']:>4} {str(e['pid'])[:10]:10} {e['stream']:4} {e['dur_s']:>6.1f} {e['n']:>4} "
              f"{_fmt(e['rate_hz']):>5} {f(a):>18} {f(sn) if sn['n'] else '-':>18} "
              f"{len(e['gaps_over_ttl']):>4} {e['deadman_in_gaps']:>4}  {e['end']}"
              + (f"  refused: {e['refusals']}" if e["refusals"] else ""))
        arr = [e["arrival"] for e in s["episodes"]]
        n_all = sum(x["n"] for x in arr)
        p(f"\n  {n_all} stream gaps in all; histogram of arrival gaps (ms):")
        hist = collections.Counter()
        for x in arr:
            hist.update(x["hist"])
        p("  " + "  ".join(f"{k}:{hist.get(k, 0)}" for k in histogram([]).keys()))
    if s["all_big_gaps"]:
        p(f"\ngaps > ttl (worst {top}):")
        for g in sorted(s["all_big_gaps"], key=lambda g: -g["gap_ms"])[:top]:
            ts = dt.datetime.fromtimestamp(g["t"]).strftime("%H:%M:%S.%f")[:-3]
            p(f"  {ts} hold {g['episode']:>3} gap {g['gap_ms']:>7.1f} ms (phone sent {_fmt(g['phone_gap_ms'])} ms apart)"
              f" -> {g['cause']}{'  DEADMAN' if g['deadman'] else ''}")
    if s["closes"]:
        p("\nconnections closed:")
        for c in s["closes"][-top:]:
            ts = dt.datetime.fromtimestamp(c["t"]).strftime("%H:%M:%S")
            p(f"  {ts} {c.get('ckind', '?')} {str(c.get('pid'))[:10]} code {c.get('code')} by {c.get('by')} "
              f"{c.get('reason') or ''} after {c.get('dur_s')} s")
    if s["diag"]:
        p("\nphone-side events: " + ", ".join(f"{k} {v}" for k, v in sorted(s["diag"].items())))
    if s["loop_lag_ms"]:
        p(f"\nserver event-loop stalls > 100 ms: {len(s['loop_lag_ms'])} (max {max(s['loop_lag_ms'])} ms)")
    p("\nverdict:")
    p(verdict(s))


def _fmt(v: Any) -> str:
    if v is None:
        return "-"
    return f"{v:.0f}" if isinstance(v, (int, float)) else str(v)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m openplay.eventlog", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sp = sub.add_parser("summarize", help="stream-gap analysis of an openplay-events-*.jsonl file")
    sp.add_argument("file")
    sp.add_argument("--ttl-ms", type=float, default=None, help="override the ttl (default: from the log's config record)")
    sp.add_argument("--json", action="store_true", help="print the summary as JSON")
    a = ap.parse_args(argv)
    s = summarize(read_records(a.file), a.ttl_ms)
    if a.json:
        print(json.dumps(s, indent=1, default=str))
    else:
        print_summary(s)
    return 0


if __name__ == "__main__":
    sys.exit(main())
