"""armctl.backend: what the daemon drives. ``Backend`` interface, ``MockBackend`` (kinematic
simulation, no hardware) and a ``DimosBackend`` stub.

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
# DimOS (stub)
# --------------------------------------------------------------------------
class DimosBackend(Backend):
    """NOT IMPLEMENTED. Backend that drives a running DimOS 0.0.14 ControlCoordinator.

    To be wired by the parent agent; reuse the logic in
    ``experiments/2026-10-16-linux-real-arm/armctl_sim.py`` (``ArmCtl``: connect, state,
    move_joints, home, ready, jog; min-jerk trajectories at 50 Hz with soft-limit checks) and
    the blueprint in ``armctl_blueprints.py`` (``ArmTwistCoordinator``: eef twist task prio 10
    + joint_trajectory prio 20 + planner). Try it with ``dimos --simulation`` first; never on
    hardware without the person at the arm agreeing (CLAUDE.md).

    What it needs:
    * Transport: DimOS 0.0.14 uses zenoh; discovery is multicast (default 224.0.0.224:7446).
      Set ``ZENOH_SCOUT_ADDR`` to the SAME value as the coordinator it targets before any
      dimos import (armctl_sim uses a private group 224.0.0.123:7449 for sims), then verify
      the coordinator's tasks/modules (joint_trajectory, eef twist) before commanding.
    * Client: ``Dimos.connect()`` -> ``app.ControlCoordinator`` RPCs:
      ``get_joint_positions()`` (dict yam_joint1.. -> rad), ``execute_trajectory(JointTrajectory)``,
      ``cancel_trajectory()`` (holds the last commanded position), ``task_invoke(
      "joint_trajectory", "get_status", {"t_now": None})``, ``set_estop``. Joint state is also
      published on ``/coordinator_joint_state`` at ~100 Hz (subscribe for ``state()``).
    * ``execute(traj)``: convert our 100 Hz samples (already gated by the daemon) into a
      DimOS ``JointTrajectory`` of ``TrajectoryPoint``s with time_from_start. DimOS does NOT
      check position limits on trajectories, so the daemon's gate is the only gate. Never
      send a single-point target (DimOS jumps at up to 1 rad/s).
    * ``set_twist`` / ``set_joint_velocity``: the eef twist task (Pink differential IK).
      Configure it with ``command_limit_margin=0.1`` and ``max_joint_velocity_rad_s`` = our
      caps so the IK itself respects the soft box. The deadman must live here too: a local
      100 Hz thread that sends zero twist when the setpoint's ttl expires. Joint-velocity
      streaming can be emulated with short rolling trajectories (or a velocity task if one
      exists) and the per-tick envelope.clamp_step guard.
    * ``stop``: cancel_trajectory + zero twist. ``estop``: set_estop. ``relax``: there is no
      clean torque-off RPC known; tools/disable_all.py sends Damiao disables on can0 and
      must only be run with the arm supported (CLAUDE.md). ``state()["torque"]`` should come
      from the CAN reply status nibble (the dashboard decodes it), not ``is_enabled``.
    * Gripper: range and direction not characterised (opening DEcreases motor position).
    """

    name = "dimos"

    def __init__(self, *a: Any, **kw: Any):
        raise NotImplementedError(
            "DimosBackend is a stub: see its docstring and experiments/2026-10-16-linux-real-arm/armctl_sim.py"
        )

    def state(self) -> dict: raise NotImplementedError  # noqa: E704
    def execute(self, traj: Trajectory) -> None: raise NotImplementedError  # noqa: E704
    def set_joint_velocity(self, qd: Any, ttl_s: float) -> None: raise NotImplementedError  # noqa: E704
    def set_twist(self, v: dict, ttl_s: float) -> None: raise NotImplementedError  # noqa: E704
    def gripper(self, width: float) -> None: raise NotImplementedError  # noqa: E704
    def stop(self) -> None: raise NotImplementedError  # noqa: E704
    def estop(self) -> None: raise NotImplementedError  # noqa: E704
    def relax(self) -> None: raise NotImplementedError  # noqa: E704
    def enable(self) -> None: raise NotImplementedError  # noqa: E704


BACKENDS: dict[str, type[Backend]] = {"mock": MockBackend, "dimos": DimosBackend}
