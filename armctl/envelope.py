"""armctl.envelope: the safety envelope every OpenYAM motion goes through.

Other modules (daemon, playables, LLM authoring) import this. The API is kept small and
stable; add to it, don't change it.

Units: joints in rad, joint velocities in rad/s, accelerations in rad/s^2. Joint order is
always j1..j6 (DimOS names yam_joint1..yam_joint6). The gripper is normalised 0 (closed)
.. 1 (open); its mapping to motor radians is not characterised yet.

Rules (ARMCTL-SPEC.md, "The gate"):
* Soft box = URDF limits shrunk by ``margin`` (default 0.1 rad per side, per-joint configurable).
* Home (all zeros) sits ON the j2/j3 hard lower limit. It is a special allowed target:
  ``check_point(HOME)`` passes, and a path planned *to* home may use j2/j3 down to 0
  (``check_path(..., allow_home=True)``).
* Out-of-envelope requests are REFUSED with a reason, never clamped silently. Streaming is
  the one exception: ``clamp_step`` zeroes the velocity components that would leave the box
  and returns the reasons, so jogging slides along the boundary instead of jamming.

Public API
----------
JOINTS, URDF_LIMITS, DEFAULT_MARGIN, VEL_CAP, ACC_CAP, GRIP_RANGE, POSES, POSE_NOTES
Violation                     one problem, with a precise human-readable ``message``
Envelope(margin, vel_caps, acc_caps)
    .soft_lower / .soft_upper / .hard_lower / .hard_upper / .vel_caps / .acc_caps  (np arrays)
    .check_point(q)            -> list[Violation]
    .check_path(samples, dt=None, *, allow_home=False, from_outside=False, t0=0.0) -> list[Violation]
    .clamp_step(q, qd, dt)     -> (qd_safe, list[Violation])
    .cap_velocity(qd)          -> (qd_scaled, scale)   uniform scaling to the speed caps
    .is_home(q)                -> bool
    .describe()                -> JSON-able dict (limits, caps, poses) for clients / LLM prompts
Module-level shortcuts use DEFAULT (margin 0.1): check_point, check_path, clamp_step, as_vector.
"""
from __future__ import annotations

from pathlib import Path

import os

import json

import math
from dataclasses import asdict, dataclass
from typing import Any, Iterable, Mapping, Sequence

import numpy as np

JOINTS: tuple[str, ...] = ("j1", "j2", "j3", "j4", "j5", "j6")
N = len(JOINTS)

# Limits of the model armctl plans with (yam_description/i2rt/yam.urdf), rad.
I2RT_URDF_LIMITS: dict[str, tuple[float, float]] = {
    "j1": (-2.618, 3.142),
    "j2": (0.0, 3.665),
    "j3": (0.0, 3.142),
    "j4": (-1.693, 1.571),
    "j5": (-1.571, 1.571),
    "j6": (-2.094, 2.094),
}
# Limits the DimOS Damiao adapter enforces on the REAL arm (yam_description/urdf/
# yam_gripper_gravity.urdf, its kinematic model): a joint reading more than 0.05 rad past one
# latches a feedback fault and the arm goes limp. On 2026-10-28 armctl let j1 go to +1.62
# (its own limit was +3.04) and the arm dropped. DimosBackend re-reads the installed file and
# refuses to start if these pinned values are looser than it (armctl/dimos_limits.py).
DIMOS_ADAPTER_LIMITS: dict[str, tuple[float, float]] = {
    "j1": (-3.92699, 1.5708),
    "j2": (0.0, 3.66519),
    "j3": (0.0, 4.01426),
    "j4": (-1.65806, 1.65806),
    "j5": (-1.5708, 1.5708),
    "j6": (-2.35619, 1.8326),
}
# Hard limits = the intersection of both: nothing armctl allows can fault DimOS.
URDF_LIMITS: dict[str, tuple[float, float]] = {
    j: (round(max(I2RT_URDF_LIMITS[j][0], DIMOS_ADAPTER_LIMITS[j][0]), 4),
        round(min(I2RT_URDF_LIMITS[j][1], DIMOS_ADAPTER_LIMITS[j][1]), 4))
    for j in I2RT_URDF_LIMITS
}
# Optional per-site limits (obstacles around where the arm is mounted), intersected too:
# armctl/site_limits.json, or ARMCTL_SITE_LIMITS=path ('' disables). Only ever tightens.
SITE_LIMITS_FILE = Path(os.environ.get("ARMCTL_SITE_LIMITS", Path(__file__).with_name("site_limits.json")))
SITE_LIMITS: dict[str, tuple[float, float]] = {}
if str(SITE_LIMITS_FILE) and SITE_LIMITS_FILE.is_file():
    SITE_LIMITS = {j: (float(lo), float(hi)) for j, (lo, hi) in json.loads(SITE_LIMITS_FILE.read_text())["limits"].items()}
    URDF_LIMITS = {j: (max(lo, SITE_LIMITS.get(j, (lo, hi))[0]), min(hi, SITE_LIMITS.get(j, (lo, hi))[1]))
                   for j, (lo, hi) in URDF_LIMITS.items()}
DEFAULT_MARGIN = 0.1  # rad, per side
VEL_CAP = 0.3  # rad/s per joint (conservative start; raise per joint after hardware tests)
ACC_CAP = 1.0  # rad/s^2 per joint
HOME_TOL = 0.02  # rad: a measured state within this of all-zeros counts as "at home"
GRIP_RANGE = (0.0, 1.0)  # normalised: 0 closed, 1 open

POSES: dict[str, tuple[float, ...]] = {
    "home": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    "ready": (0.0, 0.5, 0.8, 0.0, 0.0, 0.0),
}
POSE_NOTES: dict[str, str] = {
    "home": "folded on the stops; j2/j3 on their hard lower limit; approached slowly",
    "ready": "clear of every limit, start teleop from here. TO BE VERIFIED ON HARDWARE",
}

_EPS = 1e-9


@dataclass(frozen=True)
class Violation:
    """One envelope problem. ``message`` is what a human (or an LLM) should read."""

    joint: str | None  # "j1".."j6", or None for whole-sample problems
    kind: str  # soft_lower | soft_upper | velocity | acceleration | shape | not_finite
    value: float | None
    limit: float | None
    message: str
    t: float | None = None

    def __str__(self) -> str:
        return self.message

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def as_vector(q: Sequence[float] | Mapping[str, float] | np.ndarray) -> np.ndarray:
    """6-vector from a sequence or a mapping keyed j1..j6 / yam_joint1..6 / joint1..6.

    Raises ValueError with a readable reason on a wrong shape or unknown names.
    A mapping must name all six joints (use ``with_defaults`` style merging yourself).
    """
    if isinstance(q, Mapping):
        out = np.full(N, np.nan)
        for k, v in q.items():
            i = joint_index(k)
            out[i] = float(v)
        missing = [JOINTS[i] for i in range(N) if math.isnan(out[i])]
        if missing:
            raise ValueError(f"missing joints {missing}")
        return out
    arr = np.asarray(q, dtype=float).reshape(-1)
    if arr.shape != (N,):
        raise ValueError(f"expected {N} joint values (j1..j6), got {arr.size}")
    return arr


def joint_index(name: str | int) -> int:
    """0-based index from 'j4', 'yam_joint4', 'joint4', 4 (1-based int) -> 3."""
    if isinstance(name, (int, np.integer)) and not isinstance(name, bool):
        i = int(name) - 1
    else:
        s = str(name).strip().lower()
        for p in ("yam_joint", "joint", "j"):
            if s.startswith(p) and s[len(p):].isdigit():
                i = int(s[len(p):]) - 1
                break
        else:
            raise ValueError(f"unknown joint {name!r} (use j1..j6)")
    if not 0 <= i < N:
        raise ValueError(f"unknown joint {name!r} (use j1..j6)")
    return i


def _per_joint(v: float | Sequence[float] | Mapping[str, float], what: str) -> np.ndarray:
    if isinstance(v, Mapping):
        out = np.full(N, np.nan)
        for k, x in v.items():
            out[joint_index(k)] = float(x)
        if np.isnan(out).any():
            raise ValueError(f"{what}: give all six joints")
        return out
    if np.isscalar(v):
        return np.full(N, float(v))  # type: ignore[arg-type]
    arr = np.asarray(v, dtype=float).reshape(-1)
    if arr.shape != (N,):
        raise ValueError(f"{what}: expected {N} values")
    return arr


def _f(x: float) -> str:
    return f"{x:+.3f}"


class Envelope:
    """Soft joint box + speed/accel caps. Immutable after construction."""

    def __init__(
        self,
        margin: float | Sequence[float] | Mapping[str, float] = DEFAULT_MARGIN,
        vel_caps: float | Sequence[float] | Mapping[str, float] = VEL_CAP,
        acc_caps: float | Sequence[float] | Mapping[str, float] = ACC_CAP,
    ):
        self.hard_lower = np.array([URDF_LIMITS[j][0] for j in JOINTS])
        self.hard_upper = np.array([URDF_LIMITS[j][1] for j in JOINTS])
        self.margin = _per_joint(margin, "margin")
        if (self.margin < 0).any():
            raise ValueError("margin must be >= 0")
        self.soft_lower = self.hard_lower + self.margin
        self.soft_upper = self.hard_upper - self.margin
        if (self.soft_lower >= self.soft_upper).any():
            raise ValueError("margin too large: soft box is empty")
        self.vel_caps = _per_joint(vel_caps, "vel_caps")
        self.acc_caps = _per_joint(acc_caps, "acc_caps")
        if (self.vel_caps <= 0).any() or (self.acc_caps <= 0).any():
            raise ValueError("caps must be > 0")
        # Path box when the target is home: j2/j3 may go down to their hard lower limit (0).
        self._home_lower = self.soft_lower.copy()
        for j in ("j2", "j3"):
            i = joint_index(j)
            self._home_lower[i] = min(self.soft_lower[i], self.hard_lower[i] - HOME_TOL)
        for name, pose in POSES.items():
            if name != "home" and self.check_point(pose):
                raise ValueError(f"named pose {name!r} is outside this envelope")

    # ------------------------------------------------------------------ helpers
    def is_home(self, q: Any, tol: float = HOME_TOL) -> bool:
        v = as_vector(q)
        return bool(np.all(np.abs(v) <= tol))

    def _outside(self, v: np.ndarray, lower: np.ndarray) -> np.ndarray:
        """Per-joint distance outside [lower, soft_upper] (0 inside)."""
        return np.maximum(lower - v, 0.0) + np.maximum(v - self.soft_upper, 0.0)

    def _limit_msg(self, i: int, v: float, lower: np.ndarray, what: str) -> tuple[str, float, str]:
        j = JOINTS[i]
        lo, hi = URDF_LIMITS[j]
        if v > self.soft_upper[i]:
            hard = f", past URDF limit {hi:.3f}" if v > hi else ""
            return "soft_upper", float(self.soft_upper[i]), (
                f"{j} {what} {v:.3f} > soft limit {self.soft_upper[i]:.3f} (URDF {hi:.3f} - margin {self.margin[i]:.2f}{hard})"
            )
        hard = f", past URDF limit {lo:.3f}" if v < lo else ""
        return "soft_lower", float(lower[i]), (
            f"{j} {what} {v:.3f} < soft limit {lower[i]:.3f} (URDF {lo:.3f} + margin {self.margin[i]:.2f}{hard})"
        )

    # ------------------------------------------------------------------ checks
    def check_point(self, q: Any) -> list[Violation]:
        """Violations for one joint target. Home (all zeros, within HOME_TOL) is allowed."""
        try:
            v = as_vector(q)
        except ValueError as e:
            return [Violation(None, "shape", None, None, str(e))]
        if not np.all(np.isfinite(v)):
            return [Violation(None, "not_finite", None, None, "target contains NaN or infinity")]
        if self.is_home(v):
            return []
        out: list[Violation] = []
        for i in range(N):
            if v[i] < self.soft_lower[i] - _EPS or v[i] > self.soft_upper[i] + _EPS:
                kind, lim, msg = self._limit_msg(i, float(v[i]), self.soft_lower, "target")
                out.append(Violation(JOINTS[i], kind, float(v[i]), lim, msg))
        return out

    def check_path(
        self,
        samples: Iterable[Any],
        dt: float | None = None,
        *,
        allow_home: bool = False,
        from_outside: bool = False,
        t0: float = 0.0,
    ) -> list[Violation]:
        """Check every sample of a joint path (N x 6, equally spaced by ``dt`` seconds).

        * Every sample must be inside the soft box. ``allow_home=True`` (the path ends at
          home) lowers the j2/j3 floor to their hard limit 0.
        * ``from_outside=True``: the first sample is the measured current state and may be
          outside the box (e.g. leaving home, or recovering a joint past its limit). Later
          samples may stay outside only while, per joint, they move monotonically back
          towards the box.
        * With ``dt``: per-joint velocity and acceleration (finite differences) under the caps.

        Returns at most one Violation per (joint, kind): the first time it happens, the worst
        value, and how many samples were affected.
        """
        try:
            arr = np.array([as_vector(s) for s in samples], dtype=float)
        except ValueError as e:
            return [Violation(None, "shape", None, None, f"bad path sample: {e}")]
        if arr.size == 0:
            return [Violation(None, "shape", None, None, "empty path")]
        if not np.all(np.isfinite(arr)):
            return [Violation(None, "not_finite", None, None, "path contains NaN or infinity")]
        lower = self._home_lower if allow_home else self.soft_lower
        ts = (t0 + np.arange(len(arr)) * dt) if dt else None
        tstr = (lambda k: f" at t={ts[k]:.2f} s") if ts is not None else (lambda k: f" at sample {k}")

        out: list[Violation] = []
        prev_out = self._outside(arr[0], lower)
        bad: dict[tuple[int, str], list] = {}  # (joint, kind) -> [first_k, worst_k, worst_val, count]
        for k in range(len(arr)):
            o = self._outside(arr[k], lower)
            for i in range(N):
                if o[i] <= _EPS:
                    continue
                if from_outside and (k == 0 or o[i] <= prev_out[i] + _EPS):
                    continue  # still recovering towards the box
                kind = "soft_upper" if arr[k, i] > self.soft_upper[i] else "soft_lower"
                rec = bad.setdefault((i, kind), [k, k, arr[k, i], 0])
                rec[3] += 1
                if o[i] > self._outside(arr[rec[1]], lower)[i]:
                    rec[1], rec[2] = k, arr[k, i]
            prev_out = o
        for (i, kind), (first, worst, val, count) in sorted(bad.items()):
            _, lim, msg = self._limit_msg(i, float(val), lower, "reaches")
            out.append(Violation(
                JOINTS[i], kind, float(val), lim,
                f"{msg}{tstr(worst)} (first leaves the box{tstr(first)}, {count} samples)",
                float(ts[first]) if ts is not None else None,
            ))

        if dt and len(arr) >= 2:
            vel = np.diff(arr, axis=0) / dt
            for i in range(N):
                over = np.abs(vel[:, i]) > self.vel_caps[i] * (1 + 1e-3) + _EPS
                if over.any():
                    k = int(np.argmax(np.abs(vel[:, i])))
                    out.append(Violation(
                        JOINTS[i], "velocity", float(vel[k, i]), float(self.vel_caps[i]),
                        f"{JOINTS[i]} velocity {abs(vel[k, i]):.3f} rad/s{tstr(k)} > cap {self.vel_caps[i]:.3f} rad/s "
                        f"({int(over.sum())} samples over)",
                        float(ts[int(np.argmax(over))]),
                    ))
            if len(arr) >= 3:
                acc = np.diff(vel, axis=0) / dt
                for i in range(N):
                    over = np.abs(acc[:, i]) > self.acc_caps[i] * (1 + 1e-2) + _EPS
                    if over.any():
                        k = int(np.argmax(np.abs(acc[:, i])))
                        out.append(Violation(
                            JOINTS[i], "acceleration", float(acc[k, i]), float(self.acc_caps[i]),
                            f"{JOINTS[i]} acceleration {abs(acc[k, i]):.3f} rad/s^2{tstr(k)} > cap "
                            f"{self.acc_caps[i]:.3f} rad/s^2 ({int(over.sum())} samples over)",
                            float(ts[int(np.argmax(over))]),
                        ))
        return out

    def clamp_step(self, q: Any, qd: Any, dt: float) -> tuple[np.ndarray, list[Violation]]:
        """Streaming guard: predict ``q + qd*dt``; zero each velocity component that would take
        its joint further outside the soft box, and say why. Components moving back towards
        the box (e.g. leaving home) are allowed. Use a ``dt`` that covers the braking distance
        (see ``lookahead``), not just one control tick.
        """
        v = as_vector(q)
        w = as_vector(qd).copy()
        nxt = v + w * dt
        o_now = self._outside(v, self.soft_lower)
        o_next = self._outside(nxt, self.soft_lower)
        out: list[Violation] = []
        for i in range(N):
            if w[i] != 0.0 and o_next[i] > o_now[i] + _EPS:
                _, lim, msg = self._limit_msg(i, float(nxt[i]), self.soft_lower, "would reach")
                out.append(Violation(
                    JOINTS[i], "soft_upper" if w[i] > 0 else "soft_lower", float(nxt[i]), lim,
                    f"{msg} at {_f(w[i])} rad/s (now {v[i]:.3f}); {JOINTS[i]} motion zeroed",
                ))
                w[i] = 0.0
        return w, out

    def lookahead(self) -> float:
        """Prediction horizon (s) for clamp_step: one 10 ms tick plus braking time at the caps."""
        return 0.01 + float(np.max(self.vel_caps / (2.0 * self.acc_caps)))

    def cap_velocity(self, qd: Any) -> tuple[np.ndarray, float]:
        """Scale a joint velocity uniformly so every component is under its cap. Returns (qd, scale<=1)."""
        w = as_vector(qd)
        ratio = float(np.max(np.abs(w) / self.vel_caps)) if w.any() else 0.0
        if ratio <= 1.0:
            return w.copy(), 1.0
        return w / ratio, 1.0 / ratio

    def describe(self) -> dict[str, Any]:
        """Everything a client (or an LLM writing playables) needs to know, JSON-able."""
        return {
            "joints": list(JOINTS),
            "units": {"q": "rad", "qd": "rad/s", "acc": "rad/s^2", "grip": "0 closed .. 1 open"},
            "urdf_limits": {j: list(URDF_LIMITS[j]) for j in JOINTS},
            "soft_limits": {j: [round(float(self.soft_lower[i]), 4), round(float(self.soft_upper[i]), 4)]
                            for i, j in enumerate(JOINTS)},
            "margin": {j: float(self.margin[i]) for i, j in enumerate(JOINTS)},
            "vel_caps": {j: float(self.vel_caps[i]) for i, j in enumerate(JOINTS)},
            "acc_caps": {j: float(self.acc_caps[i]) for i, j in enumerate(JOINTS)},
            "poses": {k: list(v) for k, v in POSES.items()},
            "pose_notes": dict(POSE_NOTES),
            "home_rule": "home (all zeros) is allowed although j2/j3 sit on their hard lower limit; "
                         "it is approached slowly",
        }


DEFAULT = Envelope()


def check_point(q: Any) -> list[Violation]:
    """``DEFAULT.check_point``."""
    return DEFAULT.check_point(q)


def check_path(samples: Iterable[Any], dt: float | None = None, **kw: Any) -> list[Violation]:
    """``DEFAULT.check_path``."""
    return DEFAULT.check_path(samples, dt, **kw)


def clamp_step(q: Any, qd: Any, dt: float) -> tuple[np.ndarray, list[Violation]]:
    """``DEFAULT.clamp_step``."""
    return DEFAULT.clamp_step(q, qd, dt)
