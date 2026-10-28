"""interfaces.mirror.retarget: a person's arm -> OpenYAM joint targets.

Two mappings, both "mirror" (the arm faces the person, like a reflection: a hand moving to
the right of the camera image moves the arm's tip to the right of the image too):

* ``joints`` (puppet): the person's shoulder/elbow angles drive j1..j4 directly.
    j1 (base yaw)   <- where the upper arm points horizontally (toward the camera = 0)
    j2 (shoulder)   <- how high the upper arm is raised (hanging .. overhead)
    j3 (elbow)      <- elbow opening (folded .. straight)
    j4 (wrist)      <- hand bend up/down vs the forearm (off by default: noisy)
  Reads as "the arm copies my elbow and posture" even when the tip is not where the hand is.
* ``hand`` (IK): the wrist position relative to the shoulder, scaled into a box in front of
  the arm, solved with armctl.kin.ik. The tip goes where the hand goes; elbows are the
  solver's choice.

Every target is clipped to the soft box and checked for table clearance (armctl.kin); a
target that can't be made safe is dropped (the arm keeps its last target).

Joint facts used (armctl.kin, checked with FK): j2 is the upper arm's angle from
"flat, pointing back" (pi/2 = vertical, ~pi = flat forward); j3 is the elbow opening
(0 folded, ~pi straight); +j1 swings the tip toward the arm's left (+y); +j4 tilts the
fingers up.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from armctl import envelope as envm
from armctl import kin

from .pose import SIDE, Body

MIN_VIS = 0.5


@dataclass
class Mapping:
    """Tunable mapping constants (rad unless noted)."""

    j1_gain: float = 0.8           # base yaw per rad of upper-arm yaw
    j1_limit: float = 1.4          # never swing further than this from facing the person
    j2_down: float = 3.0           # j2 when the arm hangs straight down
    j2_up: float = 1.2             # j2 when the arm points straight up
    j3_gain: float = 0.95          # j3 = gain * elbow opening
    j3_min: float = 0.35
    j3_max: float = 2.9            # stay off full stretch (j3 past 3.14 locks DimOS out)
    wrist: bool = False
    j4_gain: float = 0.8
    yaw_hold_below: float = 0.35   # |horizontal part| of the upper arm under which yaw is held
    # hand mode: tip box (base frame, m)
    hand_center: tuple = (0.30, 0.0, 0.32)
    hand_scale: float = 0.30
    hand_box: tuple = ((0.12, 0.55), (-0.40, 0.40), (0.12, 0.62))
    rest: tuple = tuple(envm.POSES["ready"])


@dataclass
class Frame:
    """Torso frame from world landmarks: R image-right, U up, C toward the camera."""

    R: np.ndarray
    U: np.ndarray
    C: np.ndarray


def torso_frame(body: Body) -> Frame:
    W = body.world
    sh = (W[11] + W[12]) / 2
    hip = (W[23] + W[24]) / 2
    U = _unit(sh - hip) if np.linalg.norm(sh - hip) > 0.05 else np.array([0.0, -1.0, 0.0])
    C0 = np.array([0.0, 0.0, -1.0])
    C = _unit(C0 - np.dot(C0, U) * U)
    R = _unit(np.cross(U, C))
    return Frame(R, U, C)


def _unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


def arm_angles(body: Body, side: str = "right") -> dict | None:
    """Human arm angles in the torso frame, or None when the arm isn't seen well enough."""
    s = SIDE[side]
    if body.vis(s["shoulder"], s["elbow"], s["wrist"]) < MIN_VIS:
        return None
    W = body.world
    F = torso_frame(body)
    u = _unit(W[s["elbow"]] - W[s["shoulder"]])
    f = _unit(W[s["wrist"]] - W[s["elbow"]])
    elev = math.acos(float(np.clip(np.dot(u, -F.U), -1, 1)))         # 0 hanging .. pi overhead
    h = u - np.dot(u, F.U) * F.U
    horiz = float(np.linalg.norm(h))
    yaw = math.atan2(float(np.dot(h, F.R)), float(np.dot(h, F.C)))      # 0 at camera, + image right
    opening = math.acos(float(np.clip(np.dot(-u, f), -1, 1)))           # pi straight .. 0 folded
    out = {"elev": elev, "yaw": yaw, "horiz": horiz, "opening": opening, "wrist_pitch": None}
    if body.vis(s["index"]) >= MIN_VIS:
        hand = _unit(W[s["index"]] - W[s["wrist"]])
        p = F.U - np.dot(F.U, f) * f
        if np.linalg.norm(p) > 0.2:
            p = _unit(p)
            out["wrist_pitch"] = math.atan2(float(np.dot(hand, p)), float(np.dot(hand, f)))
    return out


def clutch_engaged(body: Body, side: str = "right") -> bool:
    """True while the *other* hand is raised above its shoulder (a deadman gesture)."""
    other = SIDE["left" if side == "right" else "right"]
    if body.vis(other["wrist"], other["shoulder"]) < MIN_VIS:
        return False
    F = torso_frame(body)
    return float(np.dot(body.world[other["wrist"]] - body.world[other["shoulder"]], F.U)) > 0.05


@dataclass
class Retargeter:
    mode: str = "joints"          # joints | hand
    side: str = "right"
    m: Mapping = field(default_factory=Mapping)
    envelope: envm.Envelope = envm.DEFAULT
    last: np.ndarray | None = None
    last_yaw: float = 0.0

    def __call__(self, body: Body | None) -> tuple[np.ndarray | None, dict]:
        """-> (target q or None, info). None means: nothing safe to follow, keep still."""
        if body is None:
            return None, {"why": "no person"}
        ang = arm_angles(body, self.side)
        if ang is None:
            return None, {"why": f"{self.side} arm not visible"}
        q = self._joints(ang) if self.mode == "joints" else self._hand(body)
        info = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in ang.items()}
        if q is None:
            info["why"] = "IK failed"
            return None, info
        q, why = self.make_safe(q)
        if q is None:
            info["why"] = why
            return None, info
        if why:
            info["adjusted"] = why
        self.last = q
        return q, info

    # ------------------------------------------------------------------ mappings
    def _joints(self, a: dict) -> np.ndarray:
        m = self.m
        if a["horiz"] >= m.yaw_hold_below:
            self.last_yaw = a["yaw"]
        q = np.array(self.last if self.last is not None else m.rest, float)
        q[0] = float(np.clip(-m.j1_gain * self.last_yaw, -m.j1_limit, m.j1_limit))
        q[1] = m.j2_down + (m.j2_up - m.j2_down) * a["elev"] / math.pi
        q[2] = float(np.clip(m.j3_gain * a["opening"], m.j3_min, m.j3_max))
        q[3] = float(np.clip(m.j4_gain * a["wrist_pitch"], -1.2, 1.2)) if (m.wrist and a["wrist_pitch"] is not None) else 0.0
        q[4] = 0.0
        q[5] = 0.0
        return q

    def _hand(self, body: Body) -> np.ndarray | None:
        m = self.m
        s = SIDE[self.side]
        W = body.world
        F = torso_frame(body)
        L = np.linalg.norm(W[s["elbow"]] - W[s["shoulder"]]) + np.linalg.norm(W[s["wrist"]] - W[s["elbow"]])
        rel = (W[s["wrist"]] - W[s["shoulder"]]) / max(L, 0.2)
        cx, cy, cz = m.hand_center
        p = np.array([cx + m.hand_scale * float(np.dot(rel, F.C)),
                      cy - m.hand_scale * float(np.dot(rel, F.R)),
                      cz + m.hand_scale * float(np.dot(rel, F.U))])
        p = np.array([np.clip(p[i], *m.hand_box[i]) for i in range(3)])
        seed = self.last if self.last is not None else np.array(m.rest)
        r = kin.ik(p, seed, envelope=self.envelope, max_iters=40, restarts=1, pos_tol=0.005)
        return r.q if r.err_pos < 0.03 else None

    # ------------------------------------------------------------------ safety
    def make_safe(self, q: np.ndarray) -> tuple[np.ndarray | None, str]:
        """Clip into the soft box; if the arm would come near the table, raise the upper arm."""
        env = self.envelope
        q = np.clip(q, env.soft_lower, env.soft_upper)
        why = ""
        for _ in range(40):
            if kin.clear_of_table(q):
                return q, why
            q[1] = max(env.soft_lower[1], q[1] - 0.05)   # lower j2 = raise the upper arm
            why = "raised to clear the table"
        return None, "can't clear the table"
