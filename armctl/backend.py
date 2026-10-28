"""armctl.backend: what the daemon drives. ``Backend`` interface, ``MockBackend`` (kinematic
simulation, no hardware) and ``DimosBackend`` (a running DimOS ``armctl.dimos_coordinator``).

The daemon owns policy (lease, envelope gate, deadman bookkeeping, logging). A backend owns
motion: it executes already-checked trajectories, follows velocity setpoints that expire on
their own (deadman, enforced here too as defence in depth), applies the streaming guard each
tick, and reports state.

All joint vectors are j1..j6 in rad (see armctl.envelope); the gripper is normalised 0..1.
"""
from __future__ import annotations

import abc
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable

import numpy as np

from . import envelope as env
from . import kin

RATE_HZ = 100.0
DT = 1.0 / RATE_HZ
HOME_FINAL_SPEED = 0.05  # rad/s for the last approach onto the j2/j3 stops
HOME_APPROACH = 0.1  # rad: j2/j3 height where the slow final approach starts
GRIP_SPEED = 0.5  # normalised range per second


# --------------------------------------------------------------------------
# Trajectories
# --------------------------------------------------------------------------
@dataclass
class Trajectory:
    """Joint path sampled at ``dt`` (first sample = start). ``grip`` optional per-sample."""

    q: np.ndarray  # (N, 6)
    dt: float = DT
    grip: np.ndarray | None = None
    label: str = ""
    allow_home: bool = False  # the path ends at home: j2/j3 may go down to 0

    @property
    def duration(self) -> float:
        return (len(self.q) - 1) * self.dt

    @property
    def target(self) -> np.ndarray:
        return self.q[-1]

    def __add__(self, other: "Trajectory") -> "Trajectory":
        grip = None
        if self.grip is not None or other.grip is not None:
            g1 = self.grip if self.grip is not None else np.full(len(self.q), np.nan)
            g2 = other.grip if other.grip is not None else np.full(len(other.q), np.nan)
            grip = np.concatenate([g1, g2[1:]])
        return Trajectory(np.vstack([self.q, other.q[1:]]), self.dt, grip,
                          self.label or other.label, self.allow_home or other.allow_home)


def _min_jerk(s: np.ndarray) -> np.ndarray:
    return 10 * s**3 - 15 * s**4 + 6 * s**5


def plan_move(
    start: Any, target: Any, envelope: env.Envelope = env.DEFAULT, speed: float | None = None,
    dt: float = DT, label: str = "",
) -> Trajectory:
    """Minimum-jerk joint move. Duration is chosen so every joint stays under
    min(speed, its velocity cap) and its acceleration cap (peak v = 1.875 D/T,
    peak a = 5.774 D/T^2). Does NOT check the envelope: the caller gates it."""
    q0, q1 = env.as_vector(start), env.as_vector(target)
    d = np.abs(q1 - q0)
    vcap = envelope.vel_caps if speed is None else np.minimum(envelope.vel_caps, float(speed))
    if (vcap <= 0).any():
        raise ValueError("speed must be > 0")
    T = max(float(np.max(1.875 * d / vcap)), float(np.max(np.sqrt(5.7735 * d / envelope.acc_caps))), 2 * dt)
    n = int(np.ceil(T / dt))
    s = _min_jerk(np.linspace(0.0, 1.0, n + 1))
    return Trajectory(q0 + np.outer(s, q1 - q0), dt, None, label)


def plan_home(start: Any, envelope: env.Envelope = env.DEFAULT, speed: float | None = None) -> Trajectory:
    """Home: move to (0, a, a, 0, 0, 0) with a = HOME_APPROACH (or lower if already lower),
    then the final approach onto the j2/j3 stops at HOME_FINAL_SPEED."""
    q0 = env.as_vector(start)
    pre = np.zeros(env.N)
    for j in ("j2", "j3"):
        i = env.joint_index(j)
        pre[i] = min(max(q0[i], 0.0), HOME_APPROACH)
    a = plan_move(q0, pre, envelope, speed, label="home")
    b = plan_move(pre, np.zeros(env.N), envelope, HOME_FINAL_SPEED, label="home")
    t = a + b
    t.allow_home = True
    return t


# --------------------------------------------------------------------------
# Cartesian twists
# --------------------------------------------------------------------------
# A twist is {x, y, z (m/s), roll, pitch, yaw (rad/s)} in the BASE frame: linear velocity of
# ``gripper_tip`` and angular velocity of the tip frame about the base x/y/z axes (the same
# convention as armctl.kin.jacobian_geometric). Zero angular components mean "keep the
# fingers' orientation". The mock solves it with damped least squares on the real Jacobian
# every tick (MockBackend); the DimOS backend sends it to DimOS's eef-twist IK task instead.
TWIST_DAMPING = 0.03  # DLS damping lambda (mixed units: ~m and ~rad)

# LEGACY, kept for compatibility (``twist_to_qd``): rows j1..j6, columns x, y, z (m/s), roll,
# pitch, yaw (rad/s). NOT a real Jacobian; the mock no longer uses it (2026-10-28).
J_APPROX = np.array([
    #  x     y     z    roll pitch  yaw
    [0.0,  3.0,  0.0,  0.0, 0.0,  1.0],   # j1 base yaw: lateral motion and yaw
    [3.0,  0.0,  2.0,  0.0, 0.0,  0.0],   # j2 shoulder: forward reach and up
    [1.0,  0.0, -2.0,  0.0, 0.0,  0.0],   # j3 elbow: reach out, fold down
    [0.0,  0.0,  0.0,  0.0, 1.0,  0.0],   # j4 wrist pitch
    [0.0,  0.0,  0.0,  0.0, 0.0,  0.5],   # j5 wrist yaw
    [0.0,  0.0,  0.0,  1.0, 0.0,  0.0],   # j6 wrist roll
])
TWIST_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")


def twist_vector(v: Any) -> np.ndarray:
    """[x, y, z, roll, pitch, yaw] from a twist dict or 6-vector. Raises ValueError on unknown
    keys, a wrong shape, or NaN/infinity."""
    if isinstance(v, dict):
        unknown = set(v) - set(TWIST_KEYS)
        if unknown:
            raise ValueError(f"unknown twist keys {sorted(unknown)} (use {', '.join(TWIST_KEYS)})")
        vec = np.array([float(v.get(k, 0.0) or 0.0) for k in TWIST_KEYS])
    else:
        vec = np.asarray(v, dtype=float).reshape(6)
    if not np.all(np.isfinite(vec)):
        raise ValueError("twist contains NaN or infinity")
    return vec


def twist_to_qd(v: Any) -> np.ndarray:
    """LEGACY approximate joint velocity via ``J_APPROX`` (no kinematics). Use
    ``twist_qd(q, v)`` for the real thing."""
    return J_APPROX @ twist_vector(v)


def twist_qd(q: Any, v: Any, damping: float = TWIST_DAMPING, blocked: Any = None) -> np.ndarray:
    """Joint velocity that gives the base-frame twist ``v`` at joint position ``q``: damped
    least squares on ``kin.jacobian_geometric``. ``blocked`` (6 bools) removes joints from the
    solve (they get 0 and the others take over as far as they can). Not capped or gated."""
    J = kin.jacobian_geometric(q)
    if blocked is not None:
        J[:, np.asarray(blocked, bool)] = 0.0
    return kin.dls_velocity(q, twist_vector(v), damping, J)


# --------------------------------------------------------------------------
# Interface
# --------------------------------------------------------------------------
EventFn = Callable[[str, dict], None]


class Backend(abc.ABC):
    """What armctl needs from a robot. Implementations must be thread-safe; the daemon calls
    them from its asyncio loop and every call must return quickly (no blocking on motion).

    Events are reported via ``on_event(kind, data)`` (called from the backend's thread):
      "trajectory_done" {"label", "ok"}       a trajectory finished (or was cancelled, ok=False)
      "guard"           {"violations": [...]} the per-tick streaming guard zeroed something
      "deadman"         {}                     a velocity setpoint expired; decelerating
    """

    name = "abstract"

    def __init__(self, envelope: env.Envelope = env.DEFAULT, on_event: EventFn | None = None):
        self.envelope = envelope
        self.on_event: EventFn = on_event or (lambda kind, data: None)

    @abc.abstractmethod
    def state(self) -> dict:
        """{"q": [6], "qd": [6], "grip": float, "torque": bool, "mode": str, "moving": bool,
        "faults": [str], "label": str}. mode: hold | trajectory | velocity | relaxed | estop."""

    @abc.abstractmethod
    def execute(self, traj: Trajectory) -> None:
        """Start an envelope-checked trajectory (first sample ~= current state). Non-blocking."""

    def move_joints(self, target: Any, speed: float | None = None) -> Trajectory:
        """Plan a min-jerk move from the current state and start it. NOT gated: the daemon
        plans with ``plan_move``, checks with the envelope, then calls ``execute``."""
        traj = plan_move(self.state()["q"], target, self.envelope, speed)
        self.execute(traj)
        return traj

    @abc.abstractmethod
    def set_joint_velocity(self, qd: Any, ttl_s: float) -> None:
        """Follow a joint velocity until ``ttl_s`` from now, then decelerate to a stop.
        The backend applies envelope.clamp_step every tick."""

    @abc.abstractmethod
    def set_twist(self, v: dict, ttl_s: float) -> None:
        """Cartesian twist {x,y,z (m/s), roll,pitch,yaw (rad/s)}, same expiry rule."""

    @abc.abstractmethod
    def gripper(self, width: float) -> None:
        """Gripper target, normalised 0 (closed) .. 1 (open)."""

    @abc.abstractmethod
    def stop(self) -> None:
        """Cancel any trajectory/stream, decelerate within the accel caps, hold position."""

    @abc.abstractmethod
    def estop(self) -> None:
        """Emergency: halt now (hold position, torque stays on so the arm doesn't drop)."""

    @abc.abstractmethod
    def relax(self) -> None:
        """Torque off. The arm falls unless supported."""

    @abc.abstractmethod
    def enable(self) -> None:
        """Torque on, holding the current position."""

    def close(self) -> None:
        pass


# --------------------------------------------------------------------------
# Mock
# --------------------------------------------------------------------------
class MockBackend(Backend):
    """Kinematic simulation at 100 Hz. Starts at home with torque on. Velocity and
    acceleration caps from the envelope; URDF limits act as physical stops.

    Cartesian twists (``set_twist``): every tick the twist is turned into a joint velocity by
    damped least squares on the real Jacobian at the current q (``twist_qd``), scaled
    uniformly to the velocity caps (``cap_velocity``, so the tip keeps its direction), then
    goes through the same deadman, ``clamp_step`` guard and acceleration limit as a joint
    velocity stream. A joint the guard zeroes is left out of the solve until the next command
    (the other joints carry on with the twist as far as they can), mirroring how a zeroed
    joint-velocity component stays zero until the next ``set_joint_velocity``."""

    name = "mock"

    def __init__(self, envelope: env.Envelope = env.DEFAULT, on_event: EventFn | None = None,
                 q0: Any = None, rate_hz: float = RATE_HZ, autostart: bool = True):
        super().__init__(envelope, on_event)
        self.dt = 1.0 / rate_hz
        self.lock = threading.RLock()
        self.q = env.as_vector(q0) if q0 is not None else np.zeros(env.N)
        self.qd = np.zeros(env.N)
        self.grip = 0.0
        self.grip_target = 0.0
        self.torque = True
        self.mode = "hold"
        self.traj: Trajectory | None = None
        self.traj_t0 = 0.0
        self.cmd_qd = np.zeros(env.N)
        self.cmd_expiry = 0.0
        self.deadman_fired = True
        self.cmd_twist: np.ndarray | None = None  # live twist (base frame), else joint-velocity mode
        self.twist_blocked = np.zeros(env.N, bool)  # joints the guard zeroed since the last set_twist
        self.twist_damping = TWIST_DAMPING
        self.label = ""
        self._stop_evt = threading.Event()
        self._thread: threading.Thread | None = None
        if autostart:
            self.start()

    # ---- lifecycle
    def start(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(target=self._loop, name="mock-backend", daemon=True)
            self._thread.start()

    def close(self) -> None:
        self._stop_evt.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    def _loop(self) -> None:
        nxt = time.monotonic()
        while not self._stop_evt.is_set():
            nxt += self.dt
            try:
                self.step(time.monotonic())
            except Exception as e:  # never let the sim thread die silently
                self.on_event("error", {"error": repr(e)})
            sl = nxt - time.monotonic()
            if sl > 0:
                time.sleep(sl)
            else:
                nxt = time.monotonic()

    # ---- simulation step (public so tests can drive it deterministically)
    def step(self, now: float) -> None:
        events: list[tuple[str, dict]] = []
        with self.lock:
            dt = self.dt
            if self.mode == "trajectory" and self.traj is not None:
                k = int((now - self.traj_t0) / self.traj.dt)
                tr = self.traj
                if k >= len(tr.q) - 1:
                    new_q = tr.q[-1].copy()
                    self.qd = np.zeros(env.N)
                    self.q = new_q
                    if tr.grip is not None and not np.isnan(tr.grip[-1]):
                        self.grip_target = float(tr.grip[-1])
                    self.mode, self.traj = "hold", None
                    events.append(("trajectory_done", {"label": tr.label, "ok": True}))
                else:
                    new_q = tr.q[k]
                    self.qd = (tr.q[k + 1] - tr.q[k]) / tr.dt
                    self.q = new_q.copy()
                    if tr.grip is not None and not np.isnan(tr.grip[k]):
                        self.grip_target = float(tr.grip[k])
            elif self.mode == "velocity":
                live = now < self.cmd_expiry
                if not live and not self.deadman_fired:
                    self.deadman_fired = True
                    events.append(("deadman", {}))
                if live and self.cmd_twist is not None:
                    self.cmd_qd = self._twist_qd_locked()
                target = self.cmd_qd if live else np.zeros(env.N)
                target, viol = self.envelope.clamp_step(self.q, target, self.envelope.lookahead())
                if viol and live and self.cmd_twist is not None:
                    target, viol = self._twist_reblock_locked(target, viol)
                if viol:
                    self.cmd_qd = np.where(target == 0.0, 0.0, self.cmd_qd)
                    events.append(("guard", {"violations": [v.to_dict() for v in viol]}))
                step = np.clip(target - self.qd, -self.envelope.acc_caps * dt, self.envelope.acc_caps * dt)
                self.qd = self.qd + step
                self.q = self.q + self.qd * dt
                if not live and not self.qd.any():
                    self.mode = "hold"
            # physical stops
            lo, hi = self.envelope.hard_lower - 0.005, self.envelope.hard_upper
            hit = (self.q < lo) | (self.q > hi)
            if hit.any():
                self.q = np.clip(self.q, lo, hi)
                self.qd[hit] = 0.0
            # gripper
            if self.torque:
                g = self.grip_target - self.grip
                self.grip += float(np.clip(g, -GRIP_SPEED * dt, GRIP_SPEED * dt))
        for kind, data in events:
            self.on_event(kind, data)

    # ---- interface
    def state(self) -> dict:
        with self.lock:
            moving = bool(np.any(np.abs(self.qd) > 1e-4)) or self.mode == "trajectory"
            return {
                "q": [round(float(x), 5) for x in self.q],
                "qd": [round(float(x), 5) for x in self.qd],
                "grip": round(self.grip, 4),
                "grip_target": round(self.grip_target, 4),
                "torque": self.torque,
                "mode": self.mode if self.torque else "relaxed",
                "moving": moving,
                "faults": [],
                "label": self.label if self.mode == "trajectory" else "",
                "backend": self.name,
            }

    def _cancel_traj_locked(self) -> None:
        if self.mode == "trajectory" and self.traj is not None:
            label = self.traj.label
            self.traj = None
            self.mode = "hold"
            threading.Thread(target=self.on_event, args=("trajectory_done", {"label": label, "ok": False}),
                             daemon=True).start()

    def execute(self, traj: Trajectory) -> None:
        with self.lock:
            if not self.torque:
                raise RuntimeError("torque is off")
            self._cancel_traj_locked()
            self.traj = traj
            self.traj_t0 = time.monotonic()
            self.mode = "trajectory"
            self.label = traj.label

    def set_joint_velocity(self, qd: Any, ttl_s: float) -> None:
        w = env.as_vector(qd)
        w, _ = self.envelope.cap_velocity(w)
        with self.lock:
            if not self.torque:
                raise RuntimeError("torque is off")
            if self.mode == "trajectory":
                raise RuntimeError("a trajectory is running")
            self.cmd_twist = None
            self.cmd_qd = w
            self.cmd_expiry = time.monotonic() + max(0.0, float(ttl_s))
            self.deadman_fired = False
            self.mode = "velocity"

    def set_twist(self, v: dict, ttl_s: float) -> None:
        """Cartesian twist {x,y,z (m/s), roll,pitch,yaw (rad/s)}, base frame, tip point; same
        expiry rule as ``set_joint_velocity``. Re-solved against the real Jacobian every tick.
        Raises ValueError for a bad twist, RuntimeError if torque is off or a trajectory runs."""
        vec = twist_vector(v)
        with self.lock:
            if not self.torque:
                raise RuntimeError("torque is off")
            if self.mode == "trajectory":
                raise RuntimeError("a trajectory is running")
            self.cmd_twist = vec
            self.twist_blocked = np.zeros(env.N, bool)
            self.cmd_qd = self._twist_qd_locked()
            self.cmd_expiry = time.monotonic() + max(0.0, float(ttl_s))
            self.deadman_fired = False
            self.mode = "velocity"

    def _twist_qd_locked(self) -> np.ndarray:
        """Capped joint velocity for the live twist at the current q (blocked joints excluded)."""
        assert self.cmd_twist is not None
        qd = twist_qd(self.q, self.cmd_twist, self.twist_damping, self.twist_blocked)
        qd[self.twist_blocked] = 0.0
        qd, _ = self.envelope.cap_velocity(qd)
        return qd

    def _twist_reblock_locked(self, target: np.ndarray, viol: list) -> tuple[np.ndarray, list]:
        """The guard zeroed some joints: leave them out of the solve and re-solve so the other
        joints keep the twist going; re-check with the guard each round. Returns the final
        guarded target and every violation seen (all reported, as for joint streams)."""
        allv = list(viol)
        for _ in range(env.N):
            newly = (target == 0.0) & (self.cmd_qd != 0.0) & ~self.twist_blocked
            if not newly.any():
                break
            self.twist_blocked |= newly
            self.cmd_qd = self._twist_qd_locked()
            target, more = self.envelope.clamp_step(self.q, self.cmd_qd, self.envelope.lookahead())
            allv += more
            if not more:
                break
        return target, allv

    def gripper(self, width: float) -> None:
        with self.lock:
            self.grip_target = float(np.clip(width, *env.GRIP_RANGE))

    def stop(self) -> None:
        with self.lock:
            self._cancel_traj_locked()
            if self.torque and self.qd.any():
                self.cmd_qd = np.zeros(env.N)
                self.cmd_expiry = 0.0
                self.deadman_fired = True
                self.mode = "velocity"  # decelerate within acc caps, then hold
            elif self.torque:
                self.mode = "hold"
            self.grip_target = self.grip

    def estop(self) -> None:
        with self.lock:
            self._cancel_traj_locked()
            self.qd = np.zeros(env.N)
            self.cmd_qd = np.zeros(env.N)
            self.cmd_expiry = 0.0
            self.deadman_fired = True
            self.grip_target = self.grip
            if self.torque:
                self.mode = "hold"

    def relax(self) -> None:
        with self.lock:
            self._cancel_traj_locked()
            self.qd = np.zeros(env.N)
            self.cmd_qd = np.zeros(env.N)
            self.cmd_expiry = 0.0
            self.torque = False
            self.mode = "hold"

    def enable(self) -> None:
        with self.lock:
            self.torque = True
            self.qd = np.zeros(env.N)
            self.mode = "hold"
            self.grip_target = self.grip


# --------------------------------------------------------------------------
# DimOS
# --------------------------------------------------------------------------
import collections as _collections  # noqa: E402
import logging as _logging  # noqa: E402
import os as _os  # noqa: E402
import sys as _sys  # noqa: E402

_dlog = _logging.getLogger("armctl")

# zenoh buses (CLAUDE.md "Hardware safety"): real-arm runs and sims never share one.
DIMOS_REAL_BUS = "224.0.0.123:7449"
DIMOS_SIM_BUS = "224.0.0.124:7450"
DIMOS_DEFAULT_BUSES = ("", "224.0.0.224:7446")  # the LAN-wide default: always refused
DIMOS_ARM_JOINTS = tuple(f"yam_joint{i}" for i in range(1, 7))  # = env.JOINTS j1..j6, in order
DIMOS_GRIPPER_JOINT = "arm/gripper"  # DimOS reports/commands it as a calibrated opening 0..1
DIMOS_TWIST_TASK = "eef_twist_arm"
DIMOS_TRAJ_TASK = "joint_trajectory"
DIMOS_GRIPPER_TASK = "openyam_gripper"
DIMOS_SIM_ADAPTER = "mock_whole_body"
DIMOS_LAUNCHER = "armctl.dimos_coordinator/"
DIMOS_EXPECTED_TASKS = {  # name -> (type, priority)
    DIMOS_TWIST_TASK: ("eef_twist", 10),
    DIMOS_TRAJ_TASK: ("trajectory", 20),
    DIMOS_GRIPPER_TASK: ("gripper", 20),
}

TRAJ_TOL = 5e-4  # rad: max deviation of the decimated (piecewise-linear) path from our samples
TRAJ_MAX_GAP = 0.1  # s: longest gap between two kept points
JSTREAM_PERIOD = 0.04  # s: joint-velocity streams re-issue a short trajectory at most at 25 Hz...
JSTREAM_REFRESH = 0.1  # s: ...when the setpoint changes, or when less than this cruise is left
JSTREAM_CRUISE = 0.3  # s: each one follows the setpoint this long (at most the ttl) then brakes to rest
JSTREAM_CMD_EPS = 0.005  # rad/s: a setpoint change smaller than this does not trigger a re-issue
TWIST_PUBLISH_PERIOD = 0.02  # s: twist setpoints to DimOS at 50 Hz (its own timeout is 0.3 s)
TWIST_ZERO_REPEATS = 3  # zero twists sent when a twist stream ends (zenoh is best effort)
TWIST_MAX_LIN = 0.10  # m/s: twist magnitude caps (then scaled so the joints stay under the caps)
TWIST_MAX_ANG = 0.6  # rad/s
TWIST_MIN_POSE = {"j2": 0.3, "j3": 0.5}  # below these, Pink IK has no feasible step (SUMMARY.md)
STATUS_PERIOD = 0.05  # s: get_status polling while a trajectory runs
SETTLE_TOL = 0.01  # rad: after DimOS says COMPLETED, wait until measured q is this close...
SETTLE_TIMEOUT = 1.5  # s: ...or this long (PD sag on the real arm), then report done
STATE_POLL_AFTER_S = 0.02  # s: no joint-state message for this long -> poll (>= 50 Hz fallback)
QD_WINDOW_S = 0.05  # s: qd is a finite difference over this window
MOVING_QD = 0.01  # rad/s: |qd| above this counts as moving
RPC_TIMEOUT_S = 1.0  # s per motion RPC (DimOS's default is 120 s, which would freeze the daemon)


def dimos_bus(expect: str = "sim", environ: dict | None = None) -> str:
    """Pick and check the zenoh bus BEFORE any dimos import. Unset -> the sim bus (written back
    to ``environ``). The LAN default is refused; ``expect="sim"`` refuses the real-arm bus and
    ``expect="real"`` requires it. Returns the bus. Raises RuntimeError."""
    e = _os.environ if environ is None else environ
    if expect not in ("sim", "real"):
        raise RuntimeError(f"ARMCTL_DIMOS_EXPECT must be 'sim' or 'real', not {expect!r}")
    bus = e.get("ZENOH_SCOUT_ADDR", "").strip()
    if "ZENOH_SCOUT_ADDR" not in e and expect == "sim":
        bus = e["ZENOH_SCOUT_ADDR"] = DIMOS_SIM_BUS
    if bus in DIMOS_DEFAULT_BUSES:
        raise RuntimeError(
            f"DimosBackend refuses the default zenoh bus {bus or '(empty)'!r}: anything on the LAN could "
            f"attach. export ZENOH_SCOUT_ADDR={DIMOS_SIM_BUS} (sim) or {DIMOS_REAL_BUS} (real arm)")
    if expect == "sim" and bus == DIMOS_REAL_BUS:
        raise RuntimeError(f"ZENOH_SCOUT_ADDR={bus} is the REAL-arm bus but ARMCTL_DIMOS_EXPECT=sim: use "
                           f"{DIMOS_SIM_BUS} for sims, or ARMCTL_DIMOS_EXPECT=real with the person at the arm")
    if expect == "real" and bus != DIMOS_REAL_BUS:
        raise RuntimeError(f"ARMCTL_DIMOS_EXPECT=real needs ZENOH_SCOUT_ADDR={DIMOS_REAL_BUS} (got {bus!r})")
    return bus


def verify_coordinator(info: Any, envelope: env.Envelope, expect: str, bus: str) -> tuple[list[str], str]:
    """Check ``armctl_info()`` from the coordinator against what armctl needs. Returns
    (warnings, adapter "sim" | "real"); raises RuntimeError naming every problem."""
    if not isinstance(info, dict):
        raise RuntimeError(f"armctl_info() returned {type(info).__name__}: not an armctl.dimos_coordinator")
    errs: list[str] = []
    warns: list[str] = []
    if not str(info.get("launcher", "")).startswith(DIMOS_LAUNCHER):
        errs.append(f"launcher is {info.get('launcher')!r}, expected {DIMOS_LAUNCHER}*")
    adapters = info.get("adapters") or {}
    sim = bool(info.get("simulation")) and bool(adapters) and all(a == DIMOS_SIM_ADAPTER for a in adapters.values())
    adapter = "sim" if sim else "real"
    if expect == "sim" and not sim:
        errs.append(f"coordinator drives REAL hardware (adapters {adapters}, can {info.get('can_port')!r}) but "
                    "ARMCTL_DIMOS_EXPECT=sim; only with the person at the arm: ARMCTL_DIMOS_EXPECT=real")
    if expect == "real" and sim:
        errs.append(f"ARMCTL_DIMOS_EXPECT=real but the coordinator is a simulation (adapters {adapters})")
    if info.get("scout_addr") and str(info["scout_addr"]).strip() != bus:
        errs.append(f"coordinator is on bus {info['scout_addr']!r}, we are on {bus!r}")
    tasks = info.get("tasks") or {}
    for name, (typ, prio) in DIMOS_EXPECTED_TASKS.items():
        t = tasks.get(name)
        if t is None:
            errs.append(f"task {name!r} missing (tasks: {sorted(tasks)})")
            continue
        if t.get("type") != typ or t.get("priority") != prio:
            errs.append(f"task {name!r} is type {t.get('type')!r} priority {t.get('priority')}, expected {typ!r} {prio}")
    extra = sorted(set(tasks) - set(DIMOS_EXPECTED_TASKS))
    if extra:
        errs.append(f"unexpected tasks {extra}: something else could command the arm")
    for name in (DIMOS_TWIST_TASK, DIMOS_TRAJ_TASK):
        t = tasks.get(name)
        if t is not None and list(t.get("joints", [])) != list(DIMOS_ARM_JOINTS):
            errs.append(f"task {name!r} joints {t.get('joints')} != {list(DIMOS_ARM_JOINTS)}")
    tw = (tasks.get(DIMOS_TWIST_TASK) or {}).get("params", {})
    if tw:
        margin = float(tw.get("command_limit_margin", 0.0))
        if margin < float(envelope.margin.max()) - 1e-9:
            errs.append(f"eef twist command_limit_margin {margin} < armctl soft margin {envelope.margin.max():.3f} "
                        "(start the coordinator with a matching --ik-margin)")
        jv = float(tw.get("max_joint_velocity_rad_s", 99.0))
        if jv > float(envelope.vel_caps.min()) + 1e-9:
            errs.append(f"eef twist joint speed {jv} rad/s > armctl velocity cap {envelope.vel_caps.min():.3f} "
                        "(start the coordinator with --joint-vel <= the cap)")
        if tw.get("target_frame") != "gripper_tip":
            errs.append(f"eef twist target_frame {tw.get('target_frame')!r} != 'gripper_tip'")
        if float(tw.get("timeout", 0.0)) <= 0.0 or float(tw.get("timeout", 1.0)) > 0.5:
            warns.append(f"eef twist timeout {tw.get('timeout')} s (expected 0 < t <= 0.5)")
    tr = (tasks.get(DIMOS_TRAJ_TASK) or {}).get("params", {})
    vl = tr.get("velocity_limits") or {}
    if vl and min(float(v) for v in vl.values()) < float(envelope.vel_caps.max()) - 1e-9:
        warns.append(f"trajectory velocity backstop {min(vl.values())} rad/s < armctl cap "
                     f"{envelope.vel_caps.max():.3f}: trajectories will lag and may fail verification")
    if errs:
        raise RuntimeError("DimOS coordinator check failed: " + "; ".join(errs))
    return warns, adapter


def decimate(q: np.ndarray, dt: float, tol: float = TRAJ_TOL, max_gap: float = TRAJ_MAX_GAP) -> list[int]:
    """Indices of ``q`` (N, 6) samples to keep so linear interpolation between kept samples stays
    within ``tol`` rad of every dropped sample and no gap exceeds ``max_gap`` s. Always keeps the
    first and last sample (so >= 2 points for N >= 2)."""
    n = len(q)
    if n < 2:
        raise ValueError("a trajectory needs at least 2 samples (never send single-point targets)")
    max_k = max(1, int(round(max_gap / dt)))
    keep = [0]
    i = 0
    while i < n - 1:
        best = i + 1
        for j in range(i + 2, min(i + max_k, n - 1) + 1):
            s = (np.arange(i, j + 1) - i)[:, None] / (j - i)
            line = q[i] + s * (q[j] - q[i])
            if np.max(np.abs(q[i:j + 1] - line)) > tol:
                break
            best = j
        keep.append(best)
        i = best
    return keep


def trajectory_points(traj: Trajectory, tol: float = TRAJ_TOL, max_gap: float = TRAJ_MAX_GAP
                      ) -> tuple[list[float], np.ndarray, np.ndarray]:
    """Our 100 Hz samples -> (times, positions (K, 6), velocities (K, 6)) for a DimOS
    JointTrajectory. Multi-point (>= 2, with intermediate points every <= max_gap s); velocities
    are finite differences of the full-rate path, zero at both ends. Raises ValueError."""
    q = np.asarray(traj.q, float)
    if q.ndim != 2 or q.shape[1] != env.N:
        raise ValueError(f"trajectory samples must be (N, {env.N}), got {q.shape}")
    if not np.all(np.isfinite(q)):
        raise ValueError("trajectory contains NaN or infinity")
    if len(q) < 2 or traj.duration <= 0.0:
        raise ValueError("refusing a single-point trajectory (DimOS jumps at up to 1 rad/s with no ramp)")
    idx = decimate(q, traj.dt, tol, max_gap)
    grad = np.gradient(q, traj.dt, axis=0)
    vel = grad[idx]
    vel[0] = 0.0
    vel[-1] = 0.0
    return [round(i * traj.dt, 6) for i in idx], q[idx], vel


def _interp_path(q: np.ndarray, k: float) -> np.ndarray:
    k = min(max(k, 0.0), len(q) - 1.0)
    i = min(int(k), len(q) - 2)
    a = k - i
    return q[i] * (1.0 - a) + q[i + 1] * a


def path_velocity(q: np.ndarray, dt: float, t: float) -> np.ndarray:
    k = t / dt
    if k >= len(q) - 1 or len(q) < 2:
        return np.zeros(q.shape[1])
    i = min(int(k), len(q) - 2)
    return (q[i + 1] - q[i]) / dt


def brake_along_path(q: np.ndarray, dt: float, t_elapsed: float, acc_caps: np.ndarray,
                     out_dt: float = DT) -> np.ndarray | None:
    """Samples (at ``out_dt``) that keep following the already-gated path ``q`` from
    ``t_elapsed`` while slowing its time-scaling linearly to zero within the acceleration caps.
    Stays on the path, so it stays inside the envelope. None if there is nothing to brake."""
    v = np.abs(path_velocity(q, dt, t_elapsed))
    if v.max() < 1e-3:
        return None
    tb = max(float(np.max(v / acc_caps)), 0.05)
    ts = np.arange(0.0, tb + out_dt / 2, out_dt)
    tau = t_elapsed + ts - ts**2 / (2 * tb)
    return np.array([_interp_path(q, x / dt) for x in tau])


def velocity_plan(q0: Any, qd0: Any, cmd: Any, t_live: float, envelope: env.Envelope,
                  cruise: float = JSTREAM_CRUISE, dt: float = DT, max_t: float = 3.0
                  ) -> tuple[np.ndarray, list]:
    """Joint-velocity streaming as a short trajectory: from (q0, qd0) follow ``cmd`` for
    min(t_live, cruise) s, then brake to rest, with the MockBackend rules every 10 ms tick
    (``clamp_step`` guard over the braking lookahead, acceleration caps). Every plan ends at
    rest, so if armctl dies the arm stops by itself. Returns (samples (n, 6), guard violations
    seen on the first tick)."""
    q = env.as_vector(q0).copy()
    v = env.as_vector(qd0).copy()
    c = env.as_vector(cmd)
    t_cmd = max(0.0, min(float(t_live), cruise))
    la = envelope.lookahead()
    acc = envelope.acc_caps * dt
    out = [q.copy()]
    first: list = []
    k = 0
    while True:
        t = k * dt
        target = c if t < t_cmd else np.zeros(env.N)
        target, viol = envelope.clamp_step(q, target, la)
        if k == 0:
            first = viol
        v = v + np.clip(target - v, -acc, acc)
        v[np.abs(v) < 1e-12] = 0.0
        q = q + v * dt
        out.append(q.copy())
        k += 1
        if (t >= t_cmd and not v.any()) or t > max_t:
            break
    return np.array(out), first


def scale_twist(q: Any, vec: np.ndarray, envelope: env.Envelope) -> tuple[np.ndarray, np.ndarray]:
    """Twist capped to TWIST_MAX_LIN / TWIST_MAX_ANG, then scaled uniformly so the joint
    velocity it needs at ``q`` (DLS on armctl.kin) is under the caps: DimOS's IK then never has
    to saturate, so its integrated target cannot run ahead of the arm. Returns (twist, qd)."""
    v = np.asarray(vec, float).copy()
    lin, ang = float(np.linalg.norm(v[:3])), float(np.linalg.norm(v[3:]))
    s = min(1.0, TWIST_MAX_LIN / lin if lin > 0 else 1.0, TWIST_MAX_ANG / ang if ang > 0 else 1.0)
    v *= s
    qd = twist_qd(q, v)
    qd, s2 = envelope.cap_velocity(qd)
    return v * s2, qd


class DimosLink:
    """The DimOS 0.0.14 client side: RPCs to ``ControlCoordinator`` plus the two zenoh topics.
    Everything DimOS-specific lives here, so DimosBackend can be tested with a fake link."""

    def __init__(self, connect_timeout: float = 5.0, rpc_timeout: float = RPC_TIMEOUT_S):
        _os.environ.setdefault("DIMOS_TRANSPORT", "zenoh")
        if "dimos.core.global_config" in _sys.modules:  # imported earlier: it read the bus then
            from dimos.core.global_config import global_config
            got = (getattr(global_config, "zenoh_scout_addr", "") or "").strip()
            if got and got != _os.environ["ZENOH_SCOUT_ADDR"].strip():
                raise RuntimeError(f"dimos was imported on bus {got!r} before ZENOH_SCOUT_ADDR was checked")
        from dimos.core.transport_factory import make_transport
        from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
        from dimos.msgs.sensor_msgs.JointState import JointState
        from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
        from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
        from dimos.porcelain.dimos import Dimos

        self._JT, self._TP, self._TS = JointTrajectory, TrajectoryPoint, TwistStamped
        self.rpc_timeout = rpc_timeout
        try:
            self.app = Dimos.connect(timeout=connect_timeout)
            names = [d.rpc_name or d.class_name for d in self.app._source.list_module_descriptors()]
        except Exception as e:
            raise RuntimeError(f"no DimOS coordinator answered on bus {_os.environ['ZENOH_SCOUT_ADDR']}: {e!r}") from e
        if "ControlCoordinator" not in names:
            self.app.stop()
            raise RuntimeError(f"no ControlCoordinator on bus {_os.environ['ZENOH_SCOUT_ADDR']} (modules: {names})")
        self.coord = self.app.ControlCoordinator
        self._twist_tr = make_transport("/ee_twist_command", TwistStamped)
        self._js_tr = make_transport("/coordinator_joint_state", JointState)
        self._unsubs: list = []

    def _call(self, method: str, *args: Any, timeout: float | None = None, **kw: Any) -> Any:
        """RPC with our own timeout. Calls the RpcCall's transport directly: the proxy's
        __call__ has a 120 s timeout and keeps one closure per call forever (a leak at 25 Hz)."""
        rc = getattr(self.coord, method)
        rpc, remote, name = getattr(rc, "_rpc", None), getattr(rc, "_remote_name", None), getattr(rc, "_name", None)
        if rpc is None or remote is None or name is None or not hasattr(rpc, "call_sync"):
            return rc(*args, **kw)
        res, unsub = rpc.call_sync(f"{remote}/{name}", (args, kw), rpc_timeout=timeout or self.rpc_timeout)
        try:
            unsub()
        except Exception:
            pass
        return res

    def info(self) -> dict:
        return self._call("armctl_info", timeout=5.0)

    def joint_positions(self) -> dict[str, float]:
        return self._call("get_joint_positions")

    def execute(self, times: list[float], positions: np.ndarray, velocities: np.ndarray) -> tuple[bool, str]:
        pts = [self._TP(time_from_start=float(t), positions=[float(x) for x in p], velocities=[float(x) for x in v])
               for t, p, v in zip(times, positions, velocities)]
        r = self._call("execute_trajectory", self._JT(joint_names=list(DIMOS_ARM_JOINTS), points=pts))
        st = getattr(getattr(r, "status", None), "name", str(r))
        return st == "ACCEPTED", f"{st} {getattr(r, 'message', '') or ''}".strip()

    def cancel(self) -> str:
        r = self._call("cancel_trajectory")
        return getattr(getattr(r, "status", None), "name", str(r))

    def status(self) -> tuple[str, float]:
        """(trajectory task state name, time_elapsed of its run in s)."""
        r = self._call("task_invoke", DIMOS_TRAJ_TASK, "get_status", {"t_now": None})
        return getattr(getattr(r, "state", None), "name", str(r)), float(getattr(r, "time_elapsed", 0.0) or 0.0)

    def set_estop(self, on: bool) -> Any:
        return self._call("set_estop", bool(on))

    def gripper(self, width: float) -> bool:
        return bool(self._call("task_invoke", DIMOS_GRIPPER_TASK, "set_normalized",
                               {"values": [float(width)], "t_now": None}))

    def publish_twist(self, v: np.ndarray) -> None:
        self._twist_tr.publish(self._TS(linear=[float(x) for x in v[:3]], angular=[float(x) for x in v[3:]]))

    def subscribe_joint_state(self, cb: Callable[[dict[str, float]], None]) -> None:
        def on_msg(m: Any) -> None:
            cb(dict(zip(list(m.name), [float(x) for x in m.position])))
        u = self._js_tr.subscribe(on_msg)
        if callable(u):
            self._unsubs.append(u)

    def close(self) -> None:
        for u in self._unsubs:
            try:
                u()
            except Exception:
                pass
        for tr in (self._twist_tr, self._js_tr):
            try:
                tr.stop()
            except Exception:
                pass
        try:
            self.app.stop()  # closes this client only; the coordinator keeps running
        except Exception:
            pass


class DimosBackend(Backend):
    """Backend that drives a running ``armctl.dimos_coordinator`` (DimOS 0.0.14 ControlCoordinator
    with tasks eef_twist_arm prio 10, joint_trajectory prio 20, openyam_gripper).

    Try it in ``--simulation`` on the sim bus first; never on hardware without the person at the
    arm agreeing (CLAUDE.md). Configuration comes from the environment (the daemon builds
    backends as ``BACKENDS[name](envelope)``):

    * ``ZENOH_SCOUT_ADDR``: the coordinator's bus. Unset -> the sim bus 224.0.0.124:7450. The LAN
      default is refused; the real-arm bus 224.0.0.123:7449 is refused unless
      ``ARMCTL_DIMOS_EXPECT=real`` (default ``sim``), which in turn requires that bus.
    * Right after connecting it calls ``armctl_info()`` and refuses to start unless the
      launcher, the three tasks (names, types, priorities, arm joints), the IK margin (>= our soft
      margin), the IK joint speed (<= our caps), the bus and sim/real all match. The result is
      logged, kept in ``self.info`` and shown as ``state()["adapter"]`` ("sim" | "real") and
      ``self.description``.

    How each call maps onto DimOS:

    * ``state()``: ``/coordinator_joint_state`` (~100 Hz subscription), polling
      ``get_joint_positions`` (50-100 Hz) whenever no message came for 20 ms. ``qd`` is a 50 ms finite difference
      (the sim reports zero velocity). Keys as MockBackend plus ``adapter``/``adapter_type``.
    * ``execute(traj)``: the daemon's gated 100 Hz samples -> a DimOS ``JointTrajectory``,
      decimated to a piecewise-linear path within 0.5 mrad of every sample (gap <= 0.1 s;
      always >= 2 points, never a single-point target). DimOS's trajectory sampler is a linear
      scan per joint per tick, so 100 Hz points for a 60 s playable would load its tick loop.
      Completion: ``get_status`` at 20 Hz; on COMPLETED it waits until measured q is within
      0.01 rad of the target (or 1.5 s: PD sag on hardware) and reports ``trajectory_done``.
      ABORTED / FAULT / superseded / stopped -> ``trajectory_done ok=False``. Gripper samples in
      the trajectory are sent to the gripper task as they come up.
    * ``set_twist``: published on ``/ee_twist_command`` (TwistStamped, base frame, tip
      ``gripper_tip``: the same convention as armctl) at 50 Hz by a local 100 Hz thread that is
      the deadman: when the ttl expires it sends zero twists. The IK's own 0.3 s timeout is a third
      layer. Each setpoint is capped (0.1 m/s, 0.6 rad/s) and scaled so armctl's Jacobian says the
      joints stay under the caps, so DimOS's integrated target cannot run ahead of the arm.
      Refused (ValueError) below j2 0.3 / j3 0.5: Pink has no feasible step from the folded
      pose ("command envelope is empty"); send ``do ready`` first. The soft box itself is
      enforced by the IK (``command_limit_margin``); armctl's guard reports when it bites.
    * ``set_joint_velocity``: emulated with short rolling trajectories (DimOS's velocity task
      emits VELOCITY commands, which the OpenYAM whole-body adapter drops). Each plan follows the
      setpoint for min(ttl left, 0.3 s), guarded by ``envelope.clamp_step`` and the accel caps
      every 10 ms like MockBackend, then brakes to rest; DimOS continues it from its commanded
      position. A new plan goes out when the setpoint changes (at most 25 Hz) or when < 0.1 s of
      cruise is left (about 10 Hz for a steady stream with ttl 200 ms). DimOS holds the first
      10 ms tick of every new trajectory, so each plan starts at DimOS's commanded position (its
      tick-quantised ``time_elapsed`` on the previous plan) and skips its own first step. Every plan ends at rest, so the arm stops
      even if armctl dies.
    * ``stop``: a trajectory brakes along its own (gated) path within the accel caps; streams
      decelerate (joint) or get zero twists (Cartesian). ``cancel_trajectory`` when idle.
    * ``estop``: ``set_estop(True)`` (latches the twist and gripper tasks) + ``cancel_trajectory``
      (the trajectory task has no E-STOP of its own): halts at once, holding position. The DimOS
      latch is cleared by ``enable()`` or by the next motion command, which the daemon only
      allows after the operator reset its own E-STOP.
    * ``gripper``: ``task_invoke(openyam_gripper, set_normalized)``. armctl and DimOS both use
      0 = closed .. 1 = open. ASSUMPTION (from the DimOS 0.0.14 source, not yet measured):
      on the real arm ``can_motor_control.Gripper(opening_direction="decreasing_position")``
      calibrates during activation (the gripper opens fully at startup; motor about -2.9 rad =
      open, about 0 = closed) and DimOS's ``arm/gripper`` joint is that calibrated opening, so no
      sign flip or motor-radian mapping is needed here. Check on hardware: width 1 opens.
    * ``relax``: refused (RuntimeError). There is no clean torque-off RPC; dropping torque is a
      human step: support the arm, stop the coordinator, ``tools/disable_all.py``, check LEDs.
    * ``torque``: True in sim. TODO(real): DimOS does not report it and ``is_enabled`` is not
      trustworthy; read the Damiao reply status nibble (``b0 >> 4``, see dashboard) instead.
    """

    name = "dimos"

    def __init__(self, envelope: env.Envelope = env.DEFAULT, on_event: EventFn | None = None, *,
                 link: Any = None, expect: str | None = None, autostart: bool = True,
                 clock: Callable[[], float] = time.monotonic):
        super().__init__(envelope, on_event)
        self.clock = clock
        self.expect = expect or _os.environ.get("ARMCTL_DIMOS_EXPECT", "sim").strip().lower() or "sim"
        if link is None:
            self.bus = dimos_bus(self.expect)  # before any dimos import
            link = DimosLink()
        else:
            self.bus = _os.environ.get("ZENOH_SCOUT_ADDR", DIMOS_SIM_BUS)
        self.link = link
        try:
            self.info = link.info()
            warns, self.adapter = verify_coordinator(self.info, envelope, self.expect, self.bus)
        except Exception:
            link.close()
            raise
        hw = self.info.get("adapters") or {}
        self.adapter_type = ",".join(sorted(set(hw.values()))) or "?"
        self.description = (f"DimOS {'SIM' if self.adapter == 'sim' else 'REAL'} ({self.adapter_type}) on "
                            f"{self.bus}, coordinator pid {self.info.get('pid')}, can {self.info.get('can_port')}")
        _dlog.info("DimosBackend connected: %s; tasks %s", self.description,
                   {k: (v.get("type"), v.get("priority")) for k, v in (self.info.get("tasks") or {}).items()})
        _dlog.info("DimosBackend armctl_info: %s", self.info)
        for w in warns:
            _dlog.warning("DimosBackend: %s", w)
        if self.adapter == "real":
            _dlog.warning("DimosBackend: REAL ARM. torque is reported as True (not measured: TODO status nibble)")

        self.lock = threading.RLock()  # serialises motion commands to DimOS (held across their RPCs)
        self._slock = threading.Lock()  # joint state only; never held across an RPC
        self._q = np.zeros(env.N)
        self._grip = 0.0
        self._t_state = -1e9
        self._hist: _collections.deque = _collections.deque(maxlen=64)
        self._faults: dict[str, str] = {}
        self.grip_target = 0.0
        self.mode = "hold"
        self.label = ""
        self._stream: str | None = None  # None | "twist" | "joint"
        self._expiry = 0.0
        self._deadman_fired = True
        self._twist = np.zeros(6)
        self._twist_last_pub = -1e9
        self._zero_left = 0
        self._cmd_qd = np.zeros(env.N)
        self._jplan: tuple[np.ndarray, float, float] | None = None  # (samples, dt, t_issue)
        self._jlast_issue = -1e9
        self._jcruise_end = -1e9  # when the current plan stops following its setpoint
        self._jcmd_sent = np.zeros(env.N)  # the setpoint the current plan follows
        self._traj: Trajectory | None = None
        self._traj_t0 = 0.0
        self._traj_status_t = 0.0
        self._settle_t0: float | None = None
        self._grip_sent: float | None = None
        self._brake_until = 0.0
        self._estop_latched = False
        self._stop_evt = threading.Event()
        self._thread: threading.Thread | None = None

        self._update_joints(link.joint_positions())
        try:
            link.subscribe_joint_state(self._on_joint_state)
        except Exception as e:  # polling still works
            _dlog.warning("DimosBackend: joint-state subscription failed (%r); polling instead", e)
        with self._slock:
            self.grip_target = self._grip
        if autostart:
            self.start()

    # ---- lifecycle
    def start(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(target=self._loop, name="dimos-backend", daemon=True)
            self._thread.start()

    def close(self) -> None:
        self._stop_evt.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        try:
            with self.lock:
                if self._stream == "twist":
                    self.link.publish_twist(np.zeros(6))
        except Exception:
            pass
        self.link.close()

    def _loop(self) -> None:
        nxt = self.clock()
        while not self._stop_evt.is_set():
            nxt += DT
            try:
                self._tick(self.clock())
            except Exception as e:  # never let the worker die silently
                self._fault("worker", repr(e))
                self.on_event("error", {"error": f"dimos backend: {e!r}"})
            sl = nxt - self.clock()
            if sl > 0:
                time.sleep(sl)
            else:
                nxt = self.clock()

    # ---- joint state
    def _update_joints(self, d: dict[str, float], now: float | None = None) -> None:
        if not d or any(j not in d for j in DIMOS_ARM_JOINTS):
            return
        t = self.clock() if now is None else now
        q = np.array([float(d[j]) for j in DIMOS_ARM_JOINTS])
        with self._slock:
            self._q = q
            if DIMOS_GRIPPER_JOINT in d:
                self._grip = float(d[DIMOS_GRIPPER_JOINT])
            self._t_state = t
            self._hist.append((t, q))

    def _on_joint_state(self, d: dict[str, float]) -> None:
        self._update_joints(d)

    def _measured(self) -> tuple[np.ndarray, np.ndarray, float, float]:
        """(q, qd, grip, age_s)."""
        with self._slock:
            q, grip, t = self._q.copy(), self._grip, self._t_state
            hist = list(self._hist)
        qd = np.zeros(env.N)
        if len(hist) >= 2:
            t1, q1 = hist[-1]
            for t0, q0 in reversed(hist[:-1]):
                if t1 - t0 >= QD_WINDOW_S:
                    break
            if t1 - t0 > 1e-6:
                qd = (q1 - q0) / (t1 - t0)
        return q, qd, grip, self.clock() - t

    def _fault(self, key: str, msg: str | None) -> None:
        if msg is None:
            self._faults.pop(key, None)
        elif self._faults.get(key) != msg:
            if key not in self._faults:
                _dlog.warning("DimosBackend fault %s: %s", key, msg)
            self._faults[key] = msg

    # ---- interface
    def state(self) -> dict:
        q, qd, grip, age = self._measured()
        if age > 0.5:
            self._fault("state", f"joint state is {age:.1f} s old (coordinator gone?)")
        mode = self.mode
        moving = bool(np.any(np.abs(qd) > MOVING_QD)) or mode == "trajectory" or (
            mode == "velocity" and self._busy_streaming(self.clock()))
        return {
            "q": [round(float(x), 5) for x in q],
            "qd": [round(float(x), 5) for x in qd],
            "grip": round(float(grip), 4),
            "grip_target": round(float(self.grip_target), 4),
            "torque": True,  # TODO(real): Damiao status nibble (b0 >> 4), not is_enabled
            "mode": mode,
            "moving": moving,
            "faults": list(self._faults.values()),
            "label": self.label if mode == "trajectory" else "",
            "backend": self.name,
            "adapter": self.adapter,
            "adapter_type": self.adapter_type,
        }

    def _busy_streaming(self, now: float) -> bool:
        if now < self._expiry:
            return True
        if self._jplan is not None:
            samples, dt, t0 = self._jplan
            return now < t0 + (len(samples) - 1) * dt
        return now < self._brake_until

    def _clear_estop_locked(self) -> None:
        if self._estop_latched:
            self.link.set_estop(False)
            self._estop_latched = False
            self._fault("estop", None)
            _dlog.info("DimosBackend: DimOS E-STOP latch cleared (armctl allowed motion again)")

    def _end_traj_locked(self, ok: bool, events: list) -> None:
        if self._traj is not None:
            events.append(("trajectory_done", {"label": self._traj.label, "ok": ok}))
        self._traj = None
        self._settle_t0 = None
        if self.mode == "trajectory":
            self.mode = "hold"

    def _stop_twist_locked(self) -> None:
        if self._stream == "twist":
            self._twist = np.zeros(6)
            self.link.publish_twist(self._twist)
            self._zero_left = TWIST_ZERO_REPEATS - 1
        self._stream = None
        self._expiry = 0.0

    def _emit(self, events: list) -> None:
        for kind, data in events:
            self.on_event(kind, data)

    def execute(self, traj: Trajectory) -> None:
        times, pos, vel = trajectory_points(traj)
        events: list = []
        with self.lock:
            self._clear_estop_locked()
            self._stop_twist_locked()
            self._jplan = None
            self._end_traj_locked(False, events)  # superseded
            ok, msg = self.link.execute(times, pos, vel)
            if not ok:
                self.mode = "hold"
                self._emit(events)
                raise RuntimeError(f"DimOS refused the trajectory: {msg}")
            now = self.clock()
            self._traj, self._traj_t0, self._traj_status_t = traj, now, now
            self._grip_sent = None
            self._brake_until = 0.0
            self.mode = "trajectory"
            self.label = traj.label
            _dlog.info("DimosBackend: %s -> DimOS trajectory %d points (from %d samples), %.2f s",
                       traj.label or "trajectory", len(times), len(traj.q), traj.duration)
        self._emit(events)

    def set_joint_velocity(self, qd: Any, ttl_s: float) -> None:
        w, _ = self.envelope.cap_velocity(env.as_vector(qd))
        with self.lock:
            if self.mode == "trajectory":
                raise RuntimeError("a trajectory is running")
            self._clear_estop_locked()
            if self._stream == "twist":
                self._stop_twist_locked()
            fresh = self._stream != "joint"
            self._stream = "joint"
            self._cmd_qd = w
            self._expiry = self.clock() + max(0.0, float(ttl_s))
            self._deadman_fired = False
            self.mode = "velocity"
            if fresh:
                self._issue_jplan_locked(self.clock(), [])

    def set_twist(self, v: dict, ttl_s: float) -> None:
        vec = twist_vector(v)
        q = self._measured()[0]
        low = [f"{j} {q[env.joint_index(j)]:.2f} < {m}" for j, m in TWIST_MIN_POSE.items()
               if q[env.joint_index(j)] < m]
        if low and vec.any():
            raise ValueError("Cartesian streaming needs the arm unfolded (" + ", ".join(low) +
                             "): DimOS's IK has no solution from the folded pose; send do ready first")
        with self.lock:
            if self.mode == "trajectory":
                raise RuntimeError("a trajectory is running")
            self._clear_estop_locked()
            if self._stream == "joint":
                self._cmd_qd = np.zeros(env.N)
                self._issue_jplan_locked(self.clock(), [])  # brake the joint stream first
            fresh = self._stream != "twist"
            self._stream = "twist"
            self._twist = vec
            self._expiry = self.clock() + max(0.0, float(ttl_s))
            self._deadman_fired = False
            self._zero_left = 0
            self.mode = "velocity"
            if fresh:  # first setpoint goes out now, not on the next tick
                self._publish_twist_locked(self.clock(), [])

    def gripper(self, width: float) -> None:
        w = float(np.clip(width, *env.GRIP_RANGE))
        with self.lock:
            if abs(w - self.grip_target) < 1e-3 and not self._estop_latched:
                return  # streams repeat the same grip at 20-30 Hz: no RPC for those
            self._clear_estop_locked()
            if not self.link.gripper(w):
                raise RuntimeError("DimOS gripper task refused the target (E-STOP latched?)")
            self.grip_target = w

    def stop(self) -> None:
        events: list = []
        with self.lock:
            now = self.clock()
            if self.mode == "trajectory" and self._traj is not None:
                tr = self._traj
                brake = brake_along_path(np.asarray(tr.q, float), tr.dt, now - self._traj_t0, self.envelope.acc_caps)
                self._end_traj_locked(False, events)
                if brake is not None and len(brake) >= 2:
                    bt = Trajectory(brake, DT, label="stop")
                    ok, msg = self.link.execute(*trajectory_points(bt))
                    if ok:
                        self._brake_until = now + bt.duration
                        self.mode = "velocity"
                    else:
                        _dlog.warning("DimosBackend: braking trajectory refused (%s); cancelling", msg)
                        self.link.cancel()
                        self.mode = "hold"
                else:
                    self.link.cancel()
                    self.mode = "hold"
            elif self._stream == "joint":
                self._cmd_qd = np.zeros(env.N)
                self._expiry = 0.0
                self._deadman_fired = True
                self._issue_jplan_locked(now, events)
            elif self._stream == "twist":
                self._stop_twist_locked()
                self._deadman_fired = True
            elif not self._busy_streaming(now):
                self.link.cancel()
                self.mode = "hold"
            g = self._measured()[2]
            if abs(self.grip_target - g) > 0.02 and not self._estop_latched:
                if self.link.gripper(float(np.clip(g, 0.0, 1.0))):
                    self.grip_target = g
        self._emit(events)

    def estop(self) -> None:
        events: list = []
        with self.lock:
            try:
                self.link.set_estop(True)
                self._estop_latched = True
                self._fault("estop", "DimOS E-STOP latched (cleared by enable or the next allowed motion)")
            finally:
                self.link.cancel()
                self._end_traj_locked(False, events)
                if self._stream == "twist":
                    self.link.publish_twist(np.zeros(6))
                self._stream = None
                self._expiry = 0.0
                self._deadman_fired = True
                self._jplan = None
                self._brake_until = 0.0
                self.mode = "hold"
        _dlog.warning("DimosBackend: E-STOP (set_estop + cancel_trajectory)")
        self._emit(events)

    def relax(self) -> None:
        raise RuntimeError(
            "relax is not available on the DimOS backend: DimOS has no clean torque-off RPC. "
            "A person must support the arm, stop the coordinator, run tools/disable_all.py and check every LED is steady red")

    def enable(self) -> None:
        with self.lock:
            self._clear_estop_locked()
            if self.mode != "trajectory" and not self._busy_streaming(self.clock()):
                self.mode = "hold"

    # ---- worker (100 Hz): deadman, rolling joint streams, twist publishing, trajectory monitor
    def _issue_jplan_locked(self, now: float, events: list) -> None:
        """Send the next rolling trajectory. It starts where DimOS's commanded position is now:
        DimOS restarts every trajectory's clock at its first tick (so each re-issue costs one
        held tick) and continues it from its commanded position, so the reference is the current
        plan sampled at DimOS's own ``time_elapsed`` (not our clock, which runs a tick ahead)."""
        if self._jplan is not None:
            samples, dt, t0 = self._jplan
            el = now - t0
            try:
                st, el_d = self.link.status()
                if st == "EXECUTING":  # DimOS's commanded position is its value at the last tick
                    el = np.floor(el_d / dt + 1e-6) * dt
                elif st in ("COMPLETED", "ABORTED", "IDLE"):
                    el = (len(samples) - 1) * dt
            except Exception:
                pass
            q0 = _interp_path(samples, el / dt)
            qd0 = path_velocity(samples, dt, el)
        else:
            q0, qd0 = self._measured()[0], np.zeros(env.N)
        t_live = max(0.0, self._expiry - now)
        self._jcmd_sent = self._cmd_qd.copy()
        samples, viol = velocity_plan(q0, qd0, self._cmd_qd, t_live, self.envelope)
        if viol:
            zeroed = np.array([any(v.joint == j for v in viol) for j in env.JOINTS])
            self._cmd_qd = np.where(zeroed, 0.0, self._cmd_qd)  # stays zero until the next setpoint
            events.append(("guard", {"violations": [v.to_dict() for v in viol]}))
        self._jlast_issue = now
        self._jcruise_end = now + min(t_live, JSTREAM_CRUISE)
        if np.max(np.abs(samples - samples[0])) < 1e-7:
            self._jplan = None  # at rest and nothing to do: send nothing
            return
        if len(samples) >= 3:
            # DimOS holds the first tick of every new trajectory at its commanded position
            # (elapsed 0 = point 0); dropping our first step makes the next tick catch up, so the
            # stream keeps its average speed instead of losing one tick per re-issue.
            samples = np.vstack([samples[:1], samples[2:]])
        ok, msg = self.link.execute(*trajectory_points(Trajectory(samples, DT)))
        if ok:
            self._jplan = (samples, DT, self.clock())
            self._fault("jstream", None)
        else:
            self._fault("jstream", f"DimOS refused a joint-stream step: {msg}")
            self._jplan = None

    def _publish_twist_locked(self, now: float, events: list) -> None:
        q = self._measured()[0]
        v, qd = scale_twist(q, self._twist, self.envelope)
        _, viol = self.envelope.clamp_step(q, qd, self.envelope.lookahead())
        if viol:  # the IK's command_limit_margin enforces the box; we just say so
            events.append(("guard", {"violations": [x.to_dict() for x in viol]}))
        self.link.publish_twist(v)
        self._twist_last_pub = now

    def _tick(self, now: float) -> None:
        events: list = []
        q, qd, grip, age = self._measured()
        if age > STATE_POLL_AFTER_S:
            try:
                self._update_joints(self.link.joint_positions(), now)
                self._fault("state", None)
                self._fault("rpc", None)
            except Exception as e:
                self._fault("rpc", f"get_joint_positions failed: {e!r}")
        else:
            self._fault("state", None)
        with self.lock:
            live = now < self._expiry
            if self._stream is not None and not live and not self._deadman_fired:
                self._deadman_fired = True
                events.append(("deadman", {}))
            if self._stream == "twist":
                if live:
                    if now - self._twist_last_pub >= TWIST_PUBLISH_PERIOD - 1e-4:
                        self._publish_twist_locked(now, events)
                else:
                    self._stop_twist_locked()
            elif self._stream == "joint":
                due = (np.max(np.abs(self._cmd_qd - self._jcmd_sent)) > JSTREAM_CMD_EPS
                       or self._jcruise_end - now < JSTREAM_REFRESH)
                if live and due and now - self._jlast_issue >= JSTREAM_PERIOD - 1e-4:
                    self._issue_jplan_locked(now, events)
                elif not live and self._jplan is None:
                    self._stream = None
            if self._zero_left > 0:
                self.link.publish_twist(np.zeros(6))
                self._zero_left -= 1
            if self._jplan is not None:
                samples, dt, t0 = self._jplan
                if now > t0 + (len(samples) - 1) * dt + 0.05 and not live:
                    self._jplan = None
                    if self._stream == "joint":
                        self._stream = None
            if self.mode == "velocity" and self._stream is None and self._jplan is None \
                    and now >= self._brake_until and not np.any(np.abs(qd) > MOVING_QD):
                self.mode = "hold"
            if self.mode == "trajectory" and self._traj is not None:
                self._monitor_traj_locked(now, q, events)
        self._emit(events)

    def _monitor_traj_locked(self, now: float, q: np.ndarray, events: list) -> None:
        tr = self._traj
        assert tr is not None
        if tr.grip is not None:  # gripper samples ride along with the trajectory
            k = int(min(max((now - self._traj_t0) / tr.dt, 0), len(tr.grip) - 1))
            g = tr.grip[k]
            if not np.isnan(g) and (self._grip_sent is None or abs(g - self._grip_sent) > 0.02):
                if self.link.gripper(float(np.clip(g, 0.0, 1.0))):
                    self._grip_sent = self.grip_target = float(g)
        if self._settle_t0 is not None:
            err = float(np.max(np.abs(q - tr.q[-1])))
            if err <= SETTLE_TOL or now - self._settle_t0 >= SETTLE_TIMEOUT:
                if tr.grip is not None and not np.isnan(tr.grip[-1]) and self.link.gripper(float(np.clip(tr.grip[-1], 0, 1))):
                    self.grip_target = float(tr.grip[-1])
                self._end_traj_locked(True, events)
            return
        if now - self._traj_status_t < STATUS_PERIOD:
            return
        self._traj_status_t = now
        try:
            st, _ = self.link.status()
            self._fault("rpc", None)
        except Exception as e:
            self._fault("rpc", f"get_status failed: {e!r}")
            return
        if st == "COMPLETED":
            self._settle_t0 = now
        elif st in ("ABORTED", "FAULT", "IDLE"):
            _dlog.warning("DimosBackend: %s ended %s in DimOS", tr.label, st)
            self._end_traj_locked(False, events)
        elif now - self._traj_t0 > tr.duration + 3.0:
            _dlog.warning("DimosBackend: %s still %s %.1f s after its end; cancelling", tr.label, st,
                          now - self._traj_t0 - tr.duration)
            self.link.cancel()
            self._end_traj_locked(False, events)


BACKENDS: dict[str, type[Backend]] = {"mock": MockBackend, "dimos": DimosBackend}
