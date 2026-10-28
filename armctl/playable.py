"""Playables: validated, joint-space motion sequences for the OpenYAM arm.

Spec: experiments/2026-10-16-linux-real-arm/ARMCTL-SPEC.md, section "Playables".

Nothing in this module moves anything. It parses, interpolates, validates and
retimes playables; the armctl daemon decides whether (and how) to play one.

Schema (JSON)::

    {"name": "dance-1", "author": "llm:claude", "created": "2026-10-16T15:30:00+05:30",
     "space": "joint",          # "joint" (rad) | "cartesian" (parsed, not supported yet)
     "start": "ready",          # named pose the arm is moved to first
     "keyframes": [{"t": 0.0, "q": [0, 0.5, 0.8, 0, 0, 0], "grip": 0.0},
                   {"t": 1.5, "q": [0.4, 0.6, 0.9, 0.3, 0, 0.5], "grip": 0.5, "ease": "cubic"}],
     "loop": 1, "speed": 0.5}

Semantics chosen here (the spec leaves them open):

* ``t`` is seconds in the authored timeline; the first keyframe must be at t=0 and
  t must be strictly increasing. Playback time = authored time / ``speed``
  (speed 0.5 plays twice as slowly). ``speed`` in (0, 2].
* ``ease`` on keyframe k shapes the segment from keyframe k-1 to k:
  ``cubic`` (default): smoothstep 3s^2-2s^3, zero velocity at both ends.
  ``linear``: constant velocity. Starting/stopping a linear segment is a velocity
  step, so the accel check flags it unless the neighbours match its velocity.
  ``hold``: stay at keyframe k-1 for the whole segment; keyframe k's q must equal
  k-1's (otherwise it would be a jump, reported as ``hold_jump``).
  ``flow``: a cubic Hermite that does not stop at a keyframe whose segments on both
  sides are ``flow``: there each joint (and the gripper) passes with a monotone
  (Fritsch-Butland, scipy-PCHIP) tangent from its neighbours, or zero velocity where that
  joint changes direction. Everywhere else (first/last keyframe, next to a cubic, linear
  or hold segment, a loop seam) the velocity is zero, so a lone ``flow`` segment is
  exactly ``cubic``. Monotone tangents never overshoot: each joint stays between its two
  keyframe values, so in-box keyframes give an in-box path. The path is not the straight
  joint-space line, so check table clearance on samples (``coach.clearance``). An optional
  ``"stop": true`` on a keyframe makes the arm come to rest there anyway. Format 1.1.
* ``grip`` is 0 (closed) .. 1 (open), optional; omitted carries the previous value
  (first default 0). Gripper is interpolated with the same ease.
* ``loop`` (1..10) repeats the keyframes. For loop > 1 the last keyframe must equal
  the first (within 0.01 rad), otherwise ``loop_discontinuity``.
* If the first keyframe is not within ``START_TOL`` of the start pose, validation
  prepends a cubic transition from the start pose sized to half the caps; this is
  reported in ``transition`` and counts towards the 60 s limit. The transition may
  leave the start pose monotonically even if the start (e.g. home, on the j2/j3
  hard limit) is outside the soft box.

Limits, caps and named poses come from ``armctl.envelope`` when that module exists
(another agent owns it); otherwise from the constants below. See ``_Envelope``.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

REPO = Path(__file__).resolve().parents[1]
PLAYABLES_DIR = REPO / "playables"

JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]
RATE_HZ = 100.0
MAX_DURATION_S = 60.0
START_TOL = 0.05  # rad: first keyframe this close to the start pose needs no transition
LOOP_TOL = 0.01
EASES = ("linear", "cubic", "hold", "flow")

# --- fallback envelope (used only if armctl.envelope is missing or lacks a name) ----
# URDF limits (rad), yam_description/i2rt/yam.urdf; soft box = URDF shrunk by 0.1 rad.
_URDF_LIMITS = [(-2.61799, 3.14159), (0.0, 3.66519), (0.0, 3.14159),
                (-1.69297, 1.5708), (-1.5708, 1.5708), (-2.0944, 2.0944)]
_MARGIN = 0.1
_SOFT = [(lo + _MARGIN, hi - _MARGIN) for lo, hi in _URDF_LIMITS]
# Caps for playables. URDF velocity limit is 1 rad/s; we stay below it.
_VEL_CAP = 0.8   # rad/s per joint
_ACC_CAP = 3.0   # rad/s^2 per joint
_GRIP_VEL_CAP = 3.0  # fraction of full stroke per second
_POSES = {"home": [0.0] * 6, "ready": [0.0, 0.5, 0.8, 0.0, 0.0, 0.0]}


class _Envelope:
    """Adapter over ``armctl.envelope`` (the shared safety envelope).

    Uses ``armctl.envelope.DEFAULT`` (soft box, per-joint vel/acc caps) and
    ``armctl.envelope.POSES`` when that module imports; otherwise the fallback constants
    above. ``impl`` is the envelope object, used for an independent cross-check of the
    sampled path (``check_path``). Pass overrides for tests or a live-configured daemon.
    """

    def __init__(self, impl: Any = None, *, vel_cap: float | list[float] | None = None,
                 acc_cap: float | list[float] | None = None, grip_vel_cap: float | None = None,
                 use_module: bool = True) -> None:
        self.source = "fallback constants in armctl/playable.py"
        self.impl = None
        self.soft = [tuple(x) for x in _SOFT]
        self.vel_cap = [_VEL_CAP] * 6
        self.acc_cap = [_ACC_CAP] * 6
        self.grip_vel_cap = _GRIP_VEL_CAP
        self.poses = {k: list(v) for k, v in _POSES.items()}
        mod = None
        if use_module:
            try:
                from armctl import envelope as mod  # type: ignore
            except Exception:
                mod = None
        if mod is not None or impl is not None:
            impl = impl if impl is not None else mod.DEFAULT
            self.impl = impl
            self.source = "armctl.envelope" + ("" if impl is getattr(mod, "DEFAULT", None) else " (custom)")
            self.soft = [(float(a), float(b)) for a, b in zip(impl.soft_lower, impl.soft_upper)]
            self.vel_cap = [float(x) for x in impl.vel_caps]
            self.acc_cap = [float(x) for x in impl.acc_caps]
            if mod is not None:
                self.poses = {k: [float(x) for x in v] for k, v in mod.POSES.items()}
        if vel_cap is not None:
            self.vel_cap = [float(vel_cap)] * 6 if isinstance(vel_cap, (int, float)) else list(vel_cap)
        if acc_cap is not None:
            self.acc_cap = [float(acc_cap)] * 6 if isinstance(acc_cap, (int, float)) else list(acc_cap)
        if grip_vel_cap is not None:
            self.grip_vel_cap = float(grip_vel_cap)

    def describe(self) -> dict[str, Any]:
        return {"source": self.source, "soft_limits": self.soft, "vel_cap": self.vel_cap,
                "acc_cap": self.acc_cap, "grip_vel_cap": self.grip_vel_cap,
                "named_poses": self.poses}


_ENV: _Envelope | None = None


def envelope() -> _Envelope:
    global _ENV
    if _ENV is None:
        _ENV = _Envelope()
    return _ENV


def set_envelope(env: _Envelope | None) -> None:
    """Replace the envelope (tests, or a daemon with live-configured limits)."""
    global _ENV
    _ENV = env


# --- load / save ---------------------------------------------------------------------

NAME_RE = re.compile(r"^[a-z0-9][a-z0-9-]{0,63}$")


def slugify(text: str) -> str:
    s = re.sub(r"[^a-z0-9]+", "-", text.lower()).strip("-")
    return (s or "playable")[:64]


def load(name_or_path: str | Path) -> dict[str, Any]:
    p = Path(name_or_path)
    if not p.suffix:
        p = PLAYABLES_DIR / f"{name_or_path}.json"
    return json.loads(p.read_text())


def save(playable: dict[str, Any], directory: Path | None = None) -> Path:
    d = directory or PLAYABLES_DIR
    d.mkdir(parents=True, exist_ok=True)
    name = playable.get("name", "")
    if not NAME_RE.match(name):
        raise ValueError(f"bad playable name {name!r} (use lowercase letters, digits, '-')")
    p = d / f"{name}.json"
    p.write_text(json.dumps(playable, indent=2) + "\n")
    return p


def list_playables(directory: Path | None = None) -> list[str]:
    d = directory or PLAYABLES_DIR
    return sorted(p.stem for p in d.glob("*.json")) if d.exists() else []


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


# --- parsing ---------------------------------------------------------------------------

class SchemaError(ValueError):
    pass


@dataclass
class _Key:
    t: float
    q: np.ndarray
    grip: float
    ease: str
    stop: bool = False   # "stop": true -> zero velocity here even between two flow segments


def _parse(playable: dict[str, Any]) -> tuple[list[_Key], list[dict[str, Any]]]:
    """Return keyframes and schema violations (empty list if the schema is fine)."""
    v: list[dict[str, Any]] = []

    def bad(msg: str, t: float | None = None, kind: str = "schema") -> None:
        v.append({"t": t, "joint": None, "kind": kind, "value": None, "limit": None, "msg": msg})

    if not isinstance(playable, dict):
        raise SchemaError("playable must be a JSON object")
    if not isinstance(playable.get("name"), str) or not NAME_RE.match(playable["name"]):
        bad(f"name {playable.get('name')!r} must match {NAME_RE.pattern}")
    space = playable.get("space", "joint")
    if space == "cartesian":
        bad("space 'cartesian' is parsed but not supported yet; use 'joint'", kind="unsupported")
    elif space != "joint":
        bad(f"space must be 'joint' or 'cartesian', got {space!r}")
    start = playable.get("start", "ready")
    if start not in envelope().poses:
        bad(f"start {start!r} is not a named pose ({', '.join(envelope().poses)})")
    loop = playable.get("loop", 1)
    if not isinstance(loop, int) or isinstance(loop, bool) or not 1 <= loop <= 10:
        bad(f"loop must be an integer 1..10, got {loop!r}")
    speed = playable.get("speed", 1.0)
    if not isinstance(speed, (int, float)) or isinstance(speed, bool) or not 0 < speed <= 2:
        bad(f"speed must be a number in (0, 2], got {speed!r}")

    kfs = playable.get("keyframes")
    keys: list[_Key] = []
    if not isinstance(kfs, list) or len(kfs) < 2:
        bad("keyframes must be a list of at least 2 keyframes")  # spec v1 §JSON format
        return keys, v
    grip = 0.0
    for i, k in enumerate(kfs):
        if not isinstance(k, dict):
            bad(f"keyframe {i} is not an object")
            continue
        t = k.get("t")
        q = k.get("q")
        if not isinstance(t, (int, float)) or isinstance(t, bool) or not math.isfinite(t):
            bad(f"keyframe {i}: t must be a number")
            continue
        if (not isinstance(q, list) or len(q) != 6
                or not all(isinstance(x, (int, float)) and not isinstance(x, bool)
                           and math.isfinite(x) for x in q)):
            bad(f"keyframe {i} (t={t}): q must be a list of 6 numbers (rad)", t)
            continue
        if "grip" in k and k["grip"] is not None:
            g = k["grip"]
            if not isinstance(g, (int, float)) or not 0.0 <= g <= 1.0:
                bad(f"keyframe {i} (t={t}): grip must be 0..1, got {g!r}", t)
                continue
            grip = float(g)
        ease = k.get("ease", "cubic") or "cubic"
        if ease not in EASES:
            bad(f"keyframe {i} (t={t}): ease must be one of {EASES}, got {ease!r}", t)
            ease = "cubic"
        keys.append(_Key(float(t), np.array(q, dtype=float), grip, ease, k.get("stop") is True))
    if keys and abs(keys[0].t) > 1e-9:
        bad(f"first keyframe must be at t=0, got t={keys[0].t}", keys[0].t)
    for a, b in zip(keys, keys[1:]):
        if b.t <= a.t:
            v.append({"t": b.t, "joint": None, "kind": "non_monotonic_t", "value": b.t,
                      "limit": a.t, "msg": f"keyframe t={b.t} is not after previous t={a.t}"})
    return keys, v


# --- timeline ----------------------------------------------------------------------------

@dataclass
class Segment:
    t0: float          # playback seconds
    t1: float
    q0: np.ndarray
    q1: np.ndarray
    g0: float
    g1: float
    ease: str
    label: str         # "transition", or "loop L: kf i->j"
    authored_t0: float | None  # authored-time start (None for transition)
    speed: float
    loop: int
    seg_index: int | None  # index of the destination keyframe (k) in the authored list
    m0: np.ndarray | None = None   # "flow": joint velocity at t0 (rad per playback s)
    m1: np.ndarray | None = None   # "flow": joint velocity at t1
    gm0: float = 0.0               # "flow": grip velocity at t0 and t1 (per playback s)
    gm1: float = 0.0


def flow_tangents(ts: Any, X: Any, eases: list[str], stops: Any = None) -> np.ndarray:
    """Velocities at each keyframe for ``flow`` playback: ``ts`` (n,) times, ``X`` (n, c)
    values, ``eases[i]`` the ease of the segment *into* keyframe i (``eases[0]`` unused),
    ``stops[i]`` True to come to rest at keyframe i anyway.

    Non-zero only at an interior keyframe, not a stop, whose incoming and outgoing
    segments are both ``flow``. There, per channel, the Fritsch-Butland weighted harmonic mean of the two
    secant slopes (scipy's PCHIP rule), or 0 if the channel reverses or rests. It is at
    most 3x either secant, which keeps every segment monotone (no overshoot).
    """
    ts = np.asarray(ts, float)
    X = np.asarray(X, float).reshape(len(ts), -1)
    M = np.zeros_like(X)
    for i in range(1, len(ts) - 1):
        if eases[i] != "flow" or eases[i + 1] != "flow" or (stops is not None and stops[i]):
            continue
        h0, h1 = ts[i] - ts[i - 1], ts[i + 1] - ts[i]
        if h0 <= 0 or h1 <= 0:
            continue
        d0 = (X[i] - X[i - 1]) / h0
        d1 = (X[i + 1] - X[i]) / h1
        w1, w2 = 2 * h1 + h0, h1 + 2 * h0
        same = (d0 * d1) > 0
        with np.errstate(divide="ignore", invalid="ignore"):
            m = (w1 + w2) / (w1 / d0 + w2 / d1)
        M[i] = np.where(same, m, 0.0)
    return M


def _transition_duration(q0: np.ndarray, q1: np.ndarray) -> float:
    env = envelope()
    d = np.abs(q1 - q0)
    vel = np.array(env.vel_cap) * 0.5
    acc = np.array(env.acc_cap) * 0.5
    # smoothstep: peak vel 1.5 d/T, peak acc 6 d/T^2
    T = max(float(np.max(1.5 * d / vel)), float(np.max(np.sqrt(6 * d / acc))), 0.5)
    return round(T + 0.05, 2)


def timeline(playable: dict[str, Any]) -> tuple[list[Segment], dict[str, Any] | None, list[dict]]:
    """Expand a playable into playback-time segments. Returns (segments, transition, schema_violations)."""
    keys, viol = _parse(playable)
    if not keys or any(x["kind"] in ("schema", "non_monotonic_t", "unsupported") for x in viol):
        return [], None, viol
    speed = float(playable.get("speed", 1.0))
    loops = int(playable.get("loop", 1))
    start_q = np.array(envelope().poses[playable.get("start", "ready")], dtype=float)
    segs: list[Segment] = []
    t = 0.0
    transition = None
    if np.max(np.abs(keys[0].q - start_q)) > START_TOL:
        T = _transition_duration(start_q, keys[0].q)
        segs.append(Segment(0.0, T, start_q, keys[0].q, keys[0].grip, keys[0].grip, "cubic",
                            "transition from start pose", None, 1.0, 0, None))
        transition = {"from": playable.get("start", "ready"), "duration": T,
                      "max_delta": float(np.max(np.abs(keys[0].q - start_q)))}
        t = T
    for L in range(loops):
        if L > 0:
            gap = float(np.max(np.abs(keys[-1].q - keys[0].q)))
            if gap > LOOP_TOL:
                viol.append({"t": keys[-1].t, "joint": None, "kind": "loop_discontinuity",
                             "value": gap, "limit": LOOP_TOL,
                             "msg": f"loop={loops} but the last keyframe differs from the first "
                                    f"by {gap:.3f} rad; make them equal"})
                return segs, transition, viol
        # flow tangents per authored second -> per playback second
        Mf = flow_tangents([k.t for k in keys], [list(k.q) + [k.grip] for k in keys],
                           [k.ease for k in keys], [k.stop for k in keys]) * speed
        for i in range(1, len(keys)):
            a, b = keys[i - 1], keys[i]
            dt = (b.t - a.t) / speed
            segs.append(Segment(t, t + dt, a.q, b.q, a.grip, b.grip, b.ease,
                                f"loop {L + 1}: keyframe {i - 1}->{i} (t {a.t:g}->{b.t:g})",
                                a.t, speed, L, i, Mf[i - 1, :6], Mf[i, :6],
                                float(Mf[i - 1, 6]), float(Mf[i, 6])))
            t += dt
    if len(keys) == 1:  # single keyframe: a 0-length hold
        segs.append(Segment(t, t, keys[0].q, keys[0].q, keys[0].grip, keys[0].grip, "hold",
                            "single keyframe", 0.0, speed, 0, 0))
    return segs, transition, viol


def _shape(ease: str, s: np.ndarray) -> np.ndarray:
    if ease == "linear":
        return s
    if ease == "hold":
        return np.where(s >= 1.0, 1.0, 0.0)
    return s * s * (3 - 2 * s)


def _eval(segs: list[Segment], ts: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Positions (N,6), grip (N,), segment index (N,) at playback times ts."""
    starts = np.array([s.t0 for s in segs])
    idx = np.clip(np.searchsorted(starts, ts, side="right") - 1, 0, len(segs) - 1)
    Q = np.zeros((len(ts), 6))
    G = np.zeros(len(ts))
    for k, sg in enumerate(segs):
        m = idx == k
        if not m.any():
            continue
        span = sg.t1 - sg.t0
        s = np.ones(m.sum()) if span <= 0 else np.clip((ts[m] - sg.t0) / span, 0.0, 1.0)
        w = _shape(sg.ease, s)
        Q[m] = sg.q0 + np.outer(w, sg.q1 - sg.q0)
        G[m] = sg.g0 + w * (sg.g1 - sg.g0)
        if sg.ease == "flow" and span > 0 and sg.m0 is not None:
            # cubic Hermite = smoothstep (h01) + the tangent terms h10, h11
            h10 = s * (1 - s) ** 2 * span
            h11 = s * s * (s - 1) * span
            Q[m] += np.outer(h10, sg.m0) + np.outer(h11, sg.m1)
            G[m] += h10 * sg.gm0 + h11 * sg.gm1
    return Q, G, idx


def sample(playable: dict[str, Any], rate: float = RATE_HZ) -> dict[str, Any]:
    """Interpolate the whole playback (transition + loops) at ``rate`` Hz.

    Returns {"t": [...], "q": [[6]...], "grip": [...], "duration": s}. Raises SchemaError
    if the playable can't be expanded.
    """
    segs, _, viol = timeline(playable)
    if not segs:
        raise SchemaError("; ".join(x["msg"] for x in viol) or "empty playable")
    ts, Q, G, _ = _sample_arrays(segs, rate)
    return {"t": ts.round(4).tolist(), "q": Q.round(5).tolist(), "grip": G.round(4).tolist(),
            "duration": float(ts[-1]), "rate": rate}


def _sample_arrays(segs: list[Segment], rate: float):
    D = segs[-1].t1
    n = int(math.floor(D * rate + 1e-9))
    ts = np.arange(n + 1) / rate
    if ts[-1] < D - 1e-9:
        ts = np.append(ts, D)
    Q, G, idx = _eval(segs, ts)
    return ts, Q, G, idx


# --- validation ----------------------------------------------------------------------------

def _runs(mask: np.ndarray) -> list[tuple[int, int]]:
    """Contiguous True runs as (start, end_inclusive)."""
    out = []
    i, n = 0, len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j + 1 < n and mask[j + 1]:
                j += 1
            out.append((i, j))
            i = j + 1
        else:
            i += 1
    return out


def _authored_t(sg: Segment, t_play: float) -> float | None:
    if sg.authored_t0 is None:
        return None
    return round(sg.authored_t0 + (t_play - sg.t0) * sg.speed, 3)


# Named cap presets a playable may declare with "caps": (vel rad/s, acc rad/s^2).
# "dance" stays well under DimOS's own 1 rad/s clamp; anything else needs a code change.
CAP_PRESETS: dict[str, tuple[float, float] | None] = {"default": None, "dance": (0.6, 2.0)}


# Interchange format (public spec: openyam-playground site/spec/v1.md). The lab accepts any
# 1.x; minor versions only add optional fields and unknown fields are ignored. A playable
# without format/version is legacy and treated as 1.0.
FORMAT = "openyam-playable"
SUPPORTED_MAJOR = 1


def compat(playable: dict[str, Any]) -> tuple[bool, str]:
    """(accepted, note) for the playable's format/version fields."""
    fmt, ver = playable.get("format"), playable.get("version")
    if fmt is None and ver is None:
        return True, "legacy (no format/version), treated as 1.0"
    if fmt != FORMAT:
        return False, f"format {fmt!r} is not {FORMAT!r}"
    if not isinstance(ver, str) or not re.fullmatch(r"\d+\.\d+", ver):
        return False, f"version {ver!r} must be a 'MAJOR.MINOR' string, e.g. '1.0'"
    if int(ver.split(".")[0]) != SUPPORTED_MAJOR:
        return False, f"version {ver} not supported: this lab reads {SUPPORTED_MAJOR}.x"
    return True, f"{FORMAT} {ver}"


def _schema_refusal(kind: str, value: Any, limit: Any, msg: str) -> dict[str, Any]:
    return {"ok": False, "duration": None, "max_vel": None, "max_acc": None, "transition": None,
            "envelope": envelope().source,
            "violations": [{"t": None, "t_play": None, "joint": None, "kind": kind, "value": value,
                            "limit": limit, "msg": msg, "segment": None}]}


def validate(playable: dict[str, Any]) -> dict[str, Any]:
    """Validate with the cap preset the playable declares ("caps", default "default")."""
    ok, note = compat(playable)
    if not ok:
        return _schema_refusal("version", playable.get("version"), f"{FORMAT} {SUPPORTED_MAJOR}.x", note)
    preset = playable.get("caps", "default")
    if preset not in CAP_PRESETS:
        return _schema_refusal("schema", preset, sorted(CAP_PRESETS), f"unknown caps preset {preset!r}")
    caps = CAP_PRESETS[preset]
    if caps is None:
        return _validate(playable)
    from armctl.envelope import Envelope
    previous = _ENV
    set_envelope(_Envelope(Envelope(vel_caps=caps[0], acc_caps=caps[1])))
    try:
        return _validate(playable)
    finally:
        set_envelope(previous)


def _validate(playable: dict[str, Any]) -> dict[str, Any]:
    """Check a playable against the soft envelope, caps, start pose, duration and t order.

    Returns {ok, violations:[{t, t_play, joint, kind, value, limit, msg, segment}],
    duration, max_vel[6], max_acc[6], transition, envelope}. ``t`` is authored time
    (what you edit), ``t_play`` playback seconds.
    """
    env = envelope()
    segs, transition, viol = timeline(playable)
    result: dict[str, Any] = {"ok": False, "violations": viol, "duration": None,
                              "max_vel": None, "max_acc": None, "transition": transition,
                              "envelope": env.source}
    if not segs:
        return result
    for x in viol:
        x.setdefault("t_play", None)
        x.setdefault("segment", None)

    keys, _ = _parse(playable)
    lo = np.array([a for a, _ in env.soft])
    hi = np.array([b for _, b in env.soft])
    start_q = np.array(env.poses[playable.get("start", "ready")], dtype=float)

    # keyframes themselves
    for i, k in enumerate(keys):
        for j in range(6):
            if k.q[j] < lo[j] or k.q[j] > hi[j]:
                lim = lo[j] if k.q[j] < lo[j] else hi[j]
                side = "<" if k.q[j] < lo[j] else ">"
                viol.append({"t": k.t, "t_play": None, "joint": JOINT_NAMES[j],
                             "kind": "keyframe_out_of_range", "value": round(float(k.q[j]), 4),
                             "limit": round(float(lim), 4), "segment": f"keyframe {i}",
                             "msg": f"keyframe {i} (t={k.t:g}): {JOINT_NAMES[j]} {k.q[j]:.3f} "
                                    f"{side} soft limit {lim:.3f}"})
    for i in range(1, len(keys)):
        if keys[i].ease == "hold":
            d = float(np.max(np.abs(keys[i].q - keys[i - 1].q)))
            if d > 1e-6:
                viol.append({"t": keys[i].t, "t_play": None, "joint": None, "kind": "hold_jump",
                             "value": round(d, 4), "limit": 0.0, "segment": f"keyframe {i}",
                             "msg": f"keyframe {i} (t={keys[i].t:g}) has ease 'hold' but q differs "
                                    f"from keyframe {i - 1} by {d:.3f} rad (would be a jump); "
                                    f"repeat the previous q or use 'cubic'"})

    ts, Q, G, idx = _sample_arrays(segs, RATE_HZ)
    dt = 1.0 / RATE_HZ
    D = float(ts[-1])
    result["duration"] = round(D, 3)
    if D > MAX_DURATION_S:
        viol.append({"t": None, "t_play": round(D, 2), "joint": None, "kind": "too_long",
                     "value": round(D, 2), "limit": MAX_DURATION_S, "segment": None,
                     "msg": f"playback lasts {D:.1f} s (incl. transition, loops, speed) > "
                            f"{MAX_DURATION_S:.0f} s"})

    # envelope on every sample; the transition may leave an out-of-box start pose
    lo_t = np.minimum(lo, start_q)
    hi_t = np.maximum(hi, start_q)
    in_trans = np.array([segs[i].authored_t0 is None for i in idx])
    LO = np.where(in_trans[:, None], lo_t, lo)
    HI = np.where(in_trans[:, None], hi_t, hi)

    # velocity (central differences) and acceleration (second differences)
    V = np.gradient(Q, ts, axis=0)
    A = np.zeros_like(Q)
    if len(ts) > 2:
        A[1:-1] = (Q[2:] - 2 * Q[1:-1] + Q[:-2]) / dt**2
        # starting/ending at rest: velocity jump from 0 at the ends
        A[0] = (Q[1] - Q[0]) / dt**2
        A[-1] = (Q[-2] - Q[-1]) / dt**2
    Gv = np.gradient(G, ts) if len(ts) > 1 else np.zeros_like(G)
    result["max_vel"] = [round(float(x), 3) for x in np.max(np.abs(V), axis=0)]
    result["max_acc"] = [round(float(x), 3) for x in np.max(np.abs(A), axis=0)]
    result["max_grip_vel"] = round(float(np.max(np.abs(Gv))), 3)

    def add_runs(mask: np.ndarray, vals: np.ndarray, j: int | None, kind: str, limit,
                 fmt: str) -> None:
        for a, b in _runs(mask):
            w = a + int(np.argmax(np.abs(vals[a:b + 1])))
            sg = segs[idx[w]]
            lim = float(limit[w]) if isinstance(limit, np.ndarray) else float(limit)
            jn = JOINT_NAMES[j] if j is not None else "grip"
            at = _authored_t(sg, float(ts[w]))
            when = f"t={at:g}" if at is not None else f"playback {ts[w]:.2f}s"
            viol.append({"t": at, "t_play": round(float(ts[w]), 2),
                         "t_play_span": [round(float(ts[a]), 2), round(float(ts[b]), 2)],
                         "joint": jn, "kind": kind,
                         "value": round(float(vals[w]), 4), "limit": round(lim, 4),
                         "segment": sg.label,
                         "msg": fmt.format(jn=jn, v=float(vals[w]), lim=lim, when=when,
                                           seg=sg.label,
                                           dur=round(float(ts[b] - ts[a]), 2))})

    vcap = np.array(env.vel_cap)
    acap = np.array(env.acc_cap)
    for j in range(6):
        out_lo = Q[:, j] < LO[:, j] - 1e-9
        out_hi = Q[:, j] > HI[:, j] + 1e-9
        add_runs(out_lo, Q[:, j], j, "path_out_of_range", LO[:, j],
                 "{jn} path reaches {v:.3f} < soft limit {lim:.3f} at {when} ({seg}, for {dur}s)")
        add_runs(out_hi, Q[:, j], j, "path_out_of_range", HI[:, j],
                 "{jn} path reaches {v:.3f} > soft limit {lim:.3f} at {when} ({seg}, for {dur}s)")
        add_runs(np.abs(V[:, j]) > vcap[j] + 1e-9, V[:, j], j, "velocity", vcap[j],
                 "{jn} velocity {v:.2f} rad/s exceeds cap {lim:.2f} at {when} ({seg})")
        add_runs(np.abs(A[:, j]) > acap[j] + 1e-9, A[:, j], j, "acceleration", acap[j],
                 "{jn} acceleration {v:.1f} rad/s^2 exceeds cap {lim:.1f} at {when} ({seg})")
    add_runs(np.abs(Gv) > env.grip_vel_cap + 1e-9, Gv, None, "grip_velocity", env.grip_vel_cap,
             "gripper speed {v:.2f}/s exceeds cap {lim:.2f}/s at {when} ({seg})")

    # independent cross-check with the shared envelope (armctl.envelope.check_path);
    # only adds what our own checks did not already report for that joint and kind
    if env.impl is not None:
        n_grid = int(math.floor(D * RATE_HZ + 1e-9)) + 1
        start_outside = bool(np.any((start_q < lo - 1e-9) | (start_q > hi + 1e-9)))
        mine = {(x.get("joint"), {"path_out_of_range": "range", "keyframe_out_of_range": "range"}
                 .get(x["kind"], x["kind"])) for x in viol}
        for ev in env.impl.check_path(Q[:n_grid], dt, from_outside=start_outside):
            k = {"soft_lower": "range", "soft_upper": "range"}.get(ev.kind, ev.kind)
            if (ev.joint, k) in mine:
                continue
            viol.append({"t": None, "t_play": ev.t, "joint": ev.joint, "kind": f"envelope_{ev.kind}",
                         "value": ev.value, "limit": ev.limit, "segment": None,
                         "msg": f"armctl.envelope: {ev.message}"})

    result["ok"] = not viol
    return result


SPEED_KINDS = {"velocity", "acceleration", "grip_velocity"}


def retime(playable: dict[str, Any], max_iter: int = 30, margin: float = 1.03) -> dict[str, Any]:
    """Return a copy slowed down just enough to satisfy the speed caps.

    Stretches only the authored segments that violate velocity/acceleration caps,
    by the factor the worst violation in that segment needs (velocity scales 1/f,
    acceleration 1/f^2). Raises ValueError if non-speed violations remain or it
    can't converge (e.g. a 'linear' segment starting from rest: the velocity step
    can't be cured by slowing down; use 'cubic').
    """
    import copy

    p = copy.deepcopy(playable)
    for _ in range(max_iter):
        r = validate(p)
        if r["ok"]:
            return p
        other = [v for v in r["violations"] if v["kind"] not in SPEED_KINDS]
        if other:
            raise ValueError("retime only fixes speed violations; also: "
                             + "; ".join(v["msg"] for v in other[:5]))
        need: dict[int, float] = {}
        kfs = p["keyframes"]
        for v in r["violations"]:
            if v["t"] is None:  # transition: sized by us from caps, shouldn't happen
                continue
            # destination keyframe of the segment containing v["t"]
            k = next((i for i in range(1, len(kfs)) if kfs[i]["t"] >= v["t"] - 1e-9), len(kfs) - 1)
            ratio = abs(v["value"]) / v["limit"]
            f = ratio if v["kind"] in ("velocity", "grip_velocity") else math.sqrt(ratio)
            need[k] = max(need.get(k, 1.0), f * margin)
            # an acceleration spike at a keyframe boundary involves the next segment too
            if v["kind"] == "acceleration" and k + 1 < len(kfs) and abs(kfs[k]["t"] - v["t"]) < 0.02:
                need[k + 1] = max(need.get(k + 1, 1.0), f * margin)
        if not need:
            raise ValueError("retime: no stretchable segment found")
        if max(need.values()) > 50:
            raise ValueError("retime: needs >50x slow-down (velocity step from a 'linear' or "
                             "'hold' segment?); fix the eases instead")
        old_t = [k["t"] for k in kfs]
        new_t = [0.0]
        for i in range(1, len(kfs)):
            new_t.append(new_t[-1] + (old_t[i] - old_t[i - 1]) * need.get(i, 1.0))
        for k, t in zip(kfs, new_t):
            k["t"] = round(t, 3)
    r = validate(p)
    if r["ok"]:
        return p
    raise ValueError("retime did not converge: " + "; ".join(v["msg"] for v in r["violations"][:5]))


def report(r: dict[str, Any], max_lines: int = 30) -> str:
    """Human-readable validation report."""
    lines = [f"{'OK' if r['ok'] else 'REJECTED'}  duration {r['duration']} s  "
             f"(envelope: {r.get('envelope')})"]
    if r.get("transition"):
        tr = r["transition"]
        lines.append(f"  transition from '{tr['from']}' prepended: {tr['duration']} s "
                     f"(max delta {tr['max_delta']:.2f} rad)")
    if r.get("max_vel"):
        lines.append("  max |vel| rad/s: " + " ".join(f"{n}={x:.2f}" for n, x in zip(JOINT_NAMES, r["max_vel"])))
        lines.append("  max |acc| rad/s2: " + " ".join(f"{n}={x:.1f}" for n, x in zip(JOINT_NAMES, r["max_acc"])))
    for v in r["violations"][:max_lines]:
        lines.append(f"  - [{v['kind']}] {v['msg']}")
    if len(r["violations"]) > max_lines:
        lines.append(f"  ... and {len(r['violations']) - max_lines} more")
    return "\n".join(lines)


if __name__ == "__main__":  # .venv/bin/python -m armctl.playable [name ...]
    import sys

    names = sys.argv[1:] or list_playables()
    bad = 0
    for n in names:
        r = validate(load(n))
        bad += not r["ok"]
        print(f"{n}: " + report(r))
    sys.exit(1 if bad else 0)
