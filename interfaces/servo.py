"""interfaces.servo: turn a stream of joint *targets* into armctl velocity setpoints.

Any "follow something live" interface (pose mirroring, hand tracking, a slider) produces a
target q at camera rate. The daemon only takes velocities (``stream qd``) with a deadman, so
this module closes the loop: qd = gain * (target - q), limited per joint to the speed cap and
rate-limited to the acceleration cap. When targets stop arriving, it stops sending and the
daemon's deadman brings the arm to rest.

``OneEuro`` smooths noisy targets (pose landmarks jitter by a few degrees per frame) with
little lag on fast moves (Casiez et al. 2012).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


class OneEuro:
    """One Euro filter on a vector. ``min_cutoff`` (Hz) sets smoothing at rest, ``beta``
    how quickly the cutoff rises with speed (less lag on fast moves)."""

    def __init__(self, min_cutoff: float = 1.0, beta: float = 0.5, d_cutoff: float = 1.0):
        self.min_cutoff, self.beta, self.d_cutoff = min_cutoff, beta, d_cutoff
        self.x: np.ndarray | None = None
        self.dx: np.ndarray | None = None
        self.t: float | None = None

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def reset(self) -> None:
        self.x = self.dx = self.t = None

    def __call__(self, x, t: float) -> np.ndarray:
        x = np.asarray(x, float)
        if self.x is None or self.t is None or t <= self.t:
            self.x, self.dx, self.t = x.copy(), np.zeros_like(x), t
            return x.copy()
        dt = t - self.t
        dx = (x - self.x) / dt
        a_d = self._alpha(self.d_cutoff, dt)
        self.dx = a_d * dx + (1 - a_d) * self.dx
        cutoff = self.min_cutoff + self.beta * np.abs(self.dx)
        a = np.array([self._alpha(c, dt) for c in np.atleast_1d(cutoff)])
        self.x = a * x + (1 - a) * self.x
        self.t = t
        return self.x.copy()


@dataclass
class TargetServo:
    """P-controller from target q to a capped, acceleration-limited qd."""

    vel_caps: np.ndarray
    acc_caps: np.ndarray
    gain: float = 3.0           # 1/s: 0.1 rad error -> 0.3 rad/s
    deadband: float = 0.005     # rad: closer than this counts as there
    qd: np.ndarray = field(default_factory=lambda: np.zeros(6))

    def step(self, q, target, dt: float) -> np.ndarray:
        q, target = np.asarray(q, float), np.asarray(target, float)
        err = target - q
        err[np.abs(err) < self.deadband] = 0.0
        want = np.clip(self.gain * err, -self.vel_caps, self.vel_caps)
        # brake early enough to stop on the target at the acceleration cap: v <= sqrt(2 a d)
        want = np.sign(want) * np.minimum(np.abs(want), np.sqrt(2 * self.acc_caps * np.abs(err)))
        dv = np.clip(want - self.qd, -self.acc_caps * dt, self.acc_caps * dt)
        self.qd = self.qd + dv
        return self.qd.copy()

    def reset(self) -> None:
        self.qd = np.zeros(6)


def table_margin(q) -> float:
    """Metres above the table rule (armctl.kin): negative = too low."""
    from armctl import kin

    tz, lz, _ = kin.clearance(q)
    return min(tz - kin.MIN_TIP_Z, lz - kin.MIN_LINK_Z)


def table_guard(q, qd, horizon: float = 0.3) -> tuple[np.ndarray, bool]:
    """Targets are checked for table clearance, but the path *between* two safe targets can
    dip (each joint servos on its own). Predict ``q + qd * horizon``; if that is too low and
    lower than now, zero the joint velocities that take the arm down, worst first.
    Returns (qd, changed)."""
    q, qd = np.asarray(q, float), np.asarray(qd, float).copy()
    now = table_margin(q)
    floor = min(0.0, now)
    nxt = table_margin(q + qd * horizon)
    changed = False
    while nxt < floor - 1e-4 and np.any(qd != 0):
        best_j, best_m = None, nxt
        for j in np.nonzero(qd)[0]:
            trial = qd.copy()
            trial[j] = 0.0
            m = table_margin(q + trial * horizon)
            if m > best_m:
                best_j, best_m = j, m
        if best_j is None:
            qd[:] = 0.0
            nxt = now
        else:
            qd[best_j] = 0.0
            nxt = best_m
        changed = True
    return qd, changed


def caps_from_envelope(envelope: dict, scale: float = 1.0) -> tuple[np.ndarray, np.ndarray]:
    """(vel caps, acc caps) from the daemon's hello ``envelope``, optionally scaled down."""
    joints = envelope.get("joints", [f"j{i}" for i in range(1, 7)])
    v = np.array([envelope.get("vel_caps", {}).get(j, 0.3) for j in joints], float) * scale
    a = np.array([envelope.get("acc_caps", {}).get(j, 1.0) for j in joints], float) * scale
    return v, a
