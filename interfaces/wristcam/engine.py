"""interfaces.wristcam.engine: closed-loop look -> centre -> descend -> grasp -> verify, as a state machine.

Not one look then a blind move: the camera keeps looking until the estimate is good, and every
decision is logged (JSON lines) so a run can be replayed and the thresholds tuned.

    SEARCH   camera above the expected area (and a few offsets around it) until the finder sees it
    CENTER   move the camera centred over the estimate and closer (look_heights, camera above the
             object's top: 0.18 -> 0.12 -> 0.08 m by default; exactly-down views only reach tip
             z ~0.21 m, so higher looks don't exist), re-estimate; converged when at the last
             height the object is within ``center_px_tol`` of the image centre AND the estimate moved
             less than ``change_tol``. Gives up after ``max_center_iters`` or when the object is lost
             twice in a row.
    APPROACH tip over the object, fingers open, ``approach_clear`` above the rim
    DESCEND  down in ``descent_step`` steps to the grasp height; while the rim is still visible,
             re-check alignment: small offsets are corrected, a large one aborts back to CENTER
    CLOSE    close to the object's width minus ``squeeze``; HOLDING if the measured grip stopped
             short (``interfaces.grasp.grasp.holding``), else MISS
    LIFT     up by ``lift``; VERIFY: grip still stalled AND the finder no longer sees the object on
             the table at its old position (else MISS)
    MISS     open, back off to the approach height, re-centre, retry once (``retries``)
    PLACE    carry to ``place_table`` at travel height, down, open, up; READY; DONE

Every motion is one ``move_joints`` (through the armctl daemon on the real arm: soft box, speed
caps, lease, E-STOP), checked here first (``check_step``: soft box + table clearance on the
joint-space path; the tip may go below armctl.kin.MIN_TIP_Z only in DESCEND/CLOSE/PLACE-down,
never below the workcell's ``min_tip_z``) and with the camera kept ``cam_rim_clearance`` above
the rim when it is over the object.

The ``Arm`` protocol is what the engine needs: ``q() grip() move_joints(q, speed) gripper(g)
frame()``. ``DaemonArm`` implements it over ``interfaces.client.ArmClient`` + a frame source;
``synth.SimArm`` implements it over the synthetic world for tests.
"""
from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Protocol

import cv2
import numpy as np

from armctl import envelope as envm
from armctl import kin

from interfaces.grasp.grasp import Workcell, _set_finger_yaw, holding

from .camera import finger_dir_for_cam_axis, CameraModel, inv_T
from .locate import Detection, Hint, ObjectFinder, View, annotate


class Arm(Protocol):
    def q(self) -> np.ndarray: ...
    def grip(self) -> float: ...
    def move_joints(self, q, speed: float | None = None) -> None: ...
    def gripper(self, g: float) -> None: ...
    def frame(self) -> np.ndarray: ...


class Unsafe(RuntimeError):
    pass


@dataclass
class GraspConfig:
    obj: str = "cup"
    width: float = 0.06               # m across the fingers (cup: diameter)
    height: float = 0.08              # m, object top above the table
    expected_table: tuple = (0.0, -0.15)   # where to look first (table frame)
    place_table: tuple | None = (0.08, -0.05)
    look_heights: tuple = (0.18, 0.12, 0.08)   # camera above the object's top
    search_offsets: tuple = ((0, 0), (0.06, 0), (-0.06, 0), (0, 0.06), (0, -0.06))
    center_px_tol: float = 25.0
    change_tol: float = 0.002
    max_center_iters: int = 7
    approach_clear: float = 0.035     # tip above the rim before descending
    descent_step: float = 0.02
    realign_min: float = 0.0015
    realign_max: float = 0.008
    lift: float = 0.08
    grasp_frac: float = 0.6           # grip height as a fraction of the object height (cup with flared fingertips: ~0.85, just below the rim)
    retries: int = 1
    speed: float = 0.25               # rad/s for move_joints (daemon caps at 0.3)
    slow_speed: float = 0.12          # descending / near the object
    cam_rim_clearance: float = 0.01
    finger_yaw: float | None = None   # base-frame closing direction; None keeps j6 = 0
    settle_s: float = 0.0             # wait before grabbing a frame (real arm: ~1 s)


@dataclass
class Observation:
    q: list
    det: dict | None
    px_err: float | None
    frame: str | None = None


def check_step(q0, q1, floor: float, n: int = 16) -> str:
    """'' if the straight joint-space path q0 -> q1 stays in the soft box and clear of the table
    (tip >= floor, links >= armctl.kin.MIN_LINK_Z), else the reason."""
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    for s in np.linspace(0, 1, n):
        q = q0 + s * (q1 - q0)
        if s > 0:
            viol = envm.DEFAULT.check_point(q)
            if viol:
                return f"at {s:.0%} of the move: " + "; ".join(v.message for v in viol)
        tz, lz, ln = kin.clearance(q)
        if tz < floor - 1e-4 or lz < kin.MIN_LINK_Z:
            return f"at {s:.0%} of the move: too low (tip z {tz:.3f} m < {floor:.3f}, {ln} z {lz:.3f} m)"
    return ""


class GraspEngine:
    def __init__(self, arm: Arm, finder: ObjectFinder, cam: CameraModel, T_tip_cam, wc: Workcell,
                 cfg: GraspConfig | None = None, log_path: str | Path | None = None,
                 frames_dir: str | Path | None = None, verify_finder: ObjectFinder | None = None):
        self.arm, self.finder, self.cam = arm, finder, cam
        self.T_tip_cam = np.asarray(T_tip_cam, float)
        self.wc, self.cfg = wc, cfg or GraspConfig()
        self.verify_finder = verify_finder or finder
        self.log_path = Path(log_path) if log_path else None
        self.frames_dir = Path(frames_dir) if frames_dir else None
        if self.frames_dir:
            self.frames_dir.mkdir(parents=True, exist_ok=True)
        if self.log_path:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            self.log_path.write_text("")
        self.events: list[dict] = []
        self.t0 = time.time()
        self.state = "search"
        self.est: np.ndarray | None = None      # table xy of the object
        self.est_sigma = None
        self.level = 0
        self.iters = 0
        self.lost = 0
        self.retries_left = self.cfg.retries
        self.n_obs = 0
        self.reason = ""
        self.j6 = 0.0
        self.measured_width = None

    # ------------------------------------------------------------------ logging
    def log(self, event: str, **kw):
        e = {"t": round(time.time() - self.t0, 3), "state": self.state, "event": event}
        for k, v in kw.items():
            if isinstance(v, np.ndarray):
                v = v.round(5).tolist()
            elif isinstance(v, (np.floating, np.integer)):
                v = v.item()
            e[k] = v
        self.events.append(e)
        if self.log_path:
            with self.log_path.open("a") as f:
                f.write(json.dumps(e, default=float) + "\n")

    # ------------------------------------------------------------------ geometry
    @property
    def top_z(self) -> float:
        return self.cfg.height

    def _to_base(self, xyz_table) -> np.ndarray:
        return self.wc.to_base(xyz_table)

    def _ik_down(self, xyz_base, seed, look: bool = False) -> np.ndarray | None:
        """Fingers straight down (grasp), or with ``look`` the CAMERA straight down: the wrist
        camera sits ~60 deg off the finger axis (measured), so the fingers tilt outward."""
        if look:
            f = finger_dir_for_cam_axis([0.0, 0.0, -1.0], xyz_base, self.T_tip_cam)
            r = kin.ik(xyz_base, seed, approach_dir=f, check_table=False, restarts=3, max_iters=200, corr=kin.vision_correction())
        else:
            r = kin.ik(xyz_base, seed, down=True, check_table=False, restarts=3, max_iters=200, corr=kin.vision_correction())
        if r.err_pos > 0.004 or r.err_rot > 0.08:
            return None
        q = r.q.copy()
        if look:
            q[5] = 0.0  # keep the camera in the arm's vertical plane
        elif self.cfg.finger_yaw is not None:
            q = _set_finger_yaw(q, self.cfg.finger_yaw)
        else:
            q[5] = self.j6
        return q

    def tip_over(self, xy_table, z_table, seed=None, look: bool = False) -> np.ndarray | None:
        p = self._to_base([xy_table[0], xy_table[1], z_table])
        return self._ik_down(p, self.arm.q() if seed is None else seed, look=look)

    def camera_over(self, xy_table, h_above_top) -> tuple[np.ndarray | None, float]:
        """Tip pose (camera looking straight down) whose optical axis passes through (xy, top) at the camera
        height h above the object's top. Lowers h in 1 cm steps if it can't be reached."""
        h = h_above_top
        while h >= 0.05:
            target = np.array([xy_table[0], xy_table[1]], float)
            z_cam = self.top_z + h
            tip_xy = target.copy()
            q = None
            seed = self.arm.q()
            for _ in range(4):
                # tip z so the camera sits at z_cam: camera height above tip ~ -R t (z part)
                tip_z = z_cam - self._cam_above_tip(seed)
                q = self.tip_over(tip_xy, tip_z, seed, look=True)
                if q is None:
                    break
                hit = self._axis_hit(q, self.top_z)
                if hit is None:
                    q = None
                    break
                err = target - hit
                tip_xy = tip_xy + err
                seed = q
                if np.linalg.norm(err) < 3e-4:
                    break
            if q is not None and not self._check(self.arm.q(), q, kin.MIN_TIP_Z, quiet=True):
                return q, h
            h -= 0.01
        return None, h

    def _table_z(self, z_base: float) -> float:
        """A base-frame height as a table-frame height (the table is ~level)."""
        return float(z_base - np.asarray(self.wc.T_base_table, float)[2, 3])

    def _cam_above_tip(self, q) -> float:
        T = kin.fk_vision(q)
        return float((T[:3, :3] @ self.T_tip_cam[:3, 3])[2])

    def view(self, q) -> View:
        return View.from_q(q, self.cam, self.T_tip_cam, self.wc.T_base_table)

    def _axis_hit(self, q, z_table):
        v = self.view(q)
        c = self.cam.K[:2, 2]
        P = v.pixel_to_table([c], z_table)[0]
        return None if not np.isfinite(P).all() else P[:2]

    def tip_table(self, q=None) -> np.ndarray:
        T = inv_T(self.wc.T_base_table) @ kin.fk_vision(self.arm.q() if q is None else q)
        return T[:3, 3]

    # ------------------------------------------------------------------ motion
    def _check(self, q0, q1, floor, quiet=False) -> str:
        why = check_step(q0, q1, floor)
        if why and not quiet:
            raise Unsafe(why)
        return why

    def move(self, q, why: str, floor: float = kin.MIN_TIP_Z, slow: bool = False):
        q0 = self.arm.q()
        self._check(q0, q, floor)
        speed = self.cfg.slow_speed if slow else self.cfg.speed
        self.log("move", why=why, q_from=np.asarray(q0), q_to=np.asarray(q), speed=speed,
                 tip_table=self.tip_table(q))
        self.arm.move_joints(np.asarray(q, float), speed)

    def grip_to(self, g: float, why: str):
        self.log("gripper", why=why, target=round(float(g), 4))
        self.arm.gripper(float(g))

    # ------------------------------------------------------------------ observation
    def observe(self, hint: Hint | None = None, finder: ObjectFinder | None = None) -> tuple[Detection | None, View, np.ndarray]:
        if self.cfg.settle_s:
            time.sleep(self.cfg.settle_s)
        q = self.arm.q()
        frame = self.arm.frame()
        view = self.view(q)
        view.cam = self.cam.for_image(frame)
        det = (finder or self.finder).find(frame, view, hint)
        self.n_obs += 1
        px_err = None
        if det is not None and det.pixel is not None:
            px_err = float(np.linalg.norm(np.array(det.pixel) - view.cam.K[:2, 2]))
        name = None
        if self.frames_dir is not None:
            name = f"obs_{self.n_obs:03d}_{self.state}.jpg"
            cv2.imwrite(str(self.frames_dir / name), annotate(frame, view, det, f"{self.state} #{self.n_obs}"))
        self.log("observe", q=np.asarray(q), det=det.to_dict() if det else None, px_err=px_err, frame=name)
        return det, view, frame

    # ------------------------------------------------------------------ states
    def st_search(self):
        c = self.cfg
        for k, (dx, dy) in enumerate(c.search_offsets):
            xy = (c.expected_table[0] + dx, c.expected_table[1] + dy)
            q, h = self.camera_over(xy, c.look_heights[0])
            if q is None:
                self.log("skip_search_view", xy=list(xy), reason="unreachable")
                continue
            self.move(q, f"search view {k + 1}: camera {h:.2f} m above the object top over table {xy}")
            det, view, _ = self.observe()
            if det is not None:
                self.est = np.array(det.xyz_table[:2])
                self.est_sigma = det.sigma_xy
                self.log("decision", decision="found", est=self.est, sigma=det.sigma_xy)
                return "center"
        self.reason = f"object not found in {len(c.search_offsets)} search views around table {c.expected_table}"
        return "failed"

    def st_center(self):
        c = self.cfg
        while True:
            self.iters += 1
            if self.iters > c.max_center_iters:
                self.reason = f"no convergence after {c.max_center_iters} centring iterations"
                return "failed"
            h = c.look_heights[self.level]
            q, h_used = self.camera_over(self.est, h)
            if q is None:
                self.reason = f"can't put the camera over table {self.est.round(3).tolist()} at any height"
                return "failed"
            self.move(q, f"centre camera {h_used:.2f} m above the object (level {self.level})")
            hint = Hint(tuple(self.est), radius=max(0.03, 3 * (self.est_sigma or 0.01)))
            det, view, _ = self.observe(hint)
            if det is None:
                self.lost += 1
                self.log("decision", decision="lost", lost=self.lost)
                if self.lost >= 2:
                    self.reason = "object lost twice in a row while centring"
                    return "failed"
                self.level = max(0, self.level - 1)
                continue
            self.lost = 0
            new = np.array(det.xyz_table[:2])
            change = float(np.linalg.norm(new - self.est))
            self.est, self.est_sigma = new, det.sigma_xy
            if det.diameter:
                self.measured_width = det.diameter
            last = self.level == len(c.look_heights) - 1
            px_err = float(np.linalg.norm(np.array(det.pixel) - view.cam.K[:2, 2])) if det.pixel else 1e9
            converged = last and px_err < c.center_px_tol and change < c.change_tol
            self.log("decision", decision="converged" if converged else "closer" if px_err < 3 * c.center_px_tol else "recentre",
                     level=self.level, px_err=px_err, change_mm=change * 1000, sigma_mm=det.sigma_xy * 1000, est=self.est)
            if converged:
                return "approach"
            if px_err < 3 * c.center_px_tol and not last:
                self.level += 1

    def _grasp_geometry(self):
        c, wc = self.cfg, self.wc
        z_grasp = max(wc.min_tip_z, c.height * c.grasp_frac - wc.tip_to_pad)
        # never approach below the general table guard (short objects: descend from there)
        z_above = max(c.height + c.approach_clear, self._table_z(kin.MIN_TIP_Z + 0.008))
        width = self.measured_width if (self.measured_width and abs(self.measured_width - c.width) < 0.25 * c.width) else c.width
        g_open = wc.grip_for(width + wc.open_margin)
        g_closed = wc.grip_for(max(0.0, width - wc.squeeze))
        return z_grasp, z_above, g_open, g_closed, width

    def st_approach(self):
        z_grasp, z_above, g_open, g_closed, width = self._grasp_geometry()
        if width + self.wc.open_margin > self.wc.max_width:
            self.reason = f"object {width * 100:.1f} cm wide: too wide for the gripper"
            return "failed"
        q = self.tip_over(self.est, z_above)
        if q is None:
            self.reason = f"approach pose over table {self.est.round(3).tolist()} unreachable"
            return "failed"
        self.grip_to(g_open, "open before the approach")
        if self._check(self.arm.q(), q, kin.MIN_TIP_Z, quiet=True):
            # the joint-space path from the camera pose sags below the table guard (short
            # objects): turn the fingers down higher up first, then go straight down
            for dz in (0.04, 0.08):
                q_hi = self.tip_over(self.est, z_above + dz)
                if q_hi is not None and not self._check(self.arm.q(), q_hi, kin.MIN_TIP_Z, quiet=True):
                    self.move(q_hi, f"fingers down {dz * 100:.0f} cm above the approach height", slow=True)
                    q = self.tip_over(self.est, z_above, seed=q_hi)
                    break
        self.move(q, f"tip {z_above:.3f} m (table) over the object, fingers open {g_open:.2f}", slow=True)
        return "descend"

    def st_descend(self):
        c = self.cfg
        z_grasp, z_above, g_open, _, width = self._grasp_geometry()
        clear = g_open * self.wc.max_width / 2 - width / 2
        z = z_above
        while z > z_grasp + 1e-4:
            z = max(z_grasp, z - c.descent_step)
            # look first (rim visible while the camera is well above it)
            det, view, _ = self.observe(Hint(tuple(self.est), 0.02))
            if det is not None and det.extra.get("coverage", 1) >= 0.6:
                tip = self.tip_table()[:2]
                off = np.array(det.xyz_table[:2]) - tip
                d = float(np.linalg.norm(off))
                self.log("decision", decision="align", offset_mm=off * 1000, finger_clearance_mm=clear * 1000)
                if d > min(c.realign_max, clear - 0.003):
                    self.log("decision", decision="abort_descent", offset_mm=d * 1000)
                    self.est = np.array(det.xyz_table[:2])
                    q_up = self.tip_over(self.est, z_above)
                    self.move(q_up, "back up to re-centre", floor=self.wc.floor_base(), slow=True)
                    self.level = len(c.look_heights) - 1
                    self.iters = 0
                    return "center"
                if d > c.realign_min:
                    self.est = np.array(det.xyz_table[:2])
            q = self.tip_over(self.est, z)
            if q is None:
                self.reason = f"descent pose at z {z:.3f} unreachable"
                return "failed"
            cam_z = float((inv_T(self.wc.T_base_table) @ kin.fk_vision(q) @ self.T_tip_cam)[2, 3])
            if cam_z < c.height + c.cam_rim_clearance:
                self.reason = (f"camera would be {cam_z:.3f} m above the table, below the rim + clearance "
                               f"({c.height + c.cam_rim_clearance:.3f}): grasp lower is impossible with this mount")
                return "failed"
            self.move(q, f"descend: tip {z:.3f} m (table)", floor=self.wc.floor_base(), slow=True)
        return "close"

    def st_close(self):
        _, _, _, g_closed, _ = self._grasp_geometry()
        self.grip_to(g_closed, "close on the object")
        g = self.arm.grip()
        ok = holding(g, g_closed)
        self.log("decision", decision="holding" if ok else "miss", grip_measured=g, grip_target=g_closed,
                 width_mm=g * self.wc.max_width * 1000)
        return "lift" if ok else "miss"

    def st_lift(self):
        z_grasp, *_ = self._grasp_geometry()
        tip = self.tip_table()
        q = self.tip_over(tip[:2], tip[2] + self.cfg.lift)
        self.move(q, f"lift {self.cfg.lift:.2f} m", floor=self.wc.floor_base(), slow=True)
        return "verify"

    def st_verify(self):
        _, _, _, g_closed, _ = self._grasp_geometry()
        g = self.arm.grip()
        still = holding(g, g_closed)
        det, _, _ = self.observe(Hint(tuple(self.est), 0.03), self.verify_finder)
        on_table = det is not None and abs((det.diameter or self.cfg.width) - self.cfg.width) < 0.2 * self.cfg.width
        self.log("decision", decision="verified" if still and not on_table else "miss", grip_measured=g,
                 still_holding=still, object_still_on_table=on_table)
        return "place" if (still and not on_table) else "miss"

    def st_miss(self):
        if self.retries_left <= 0:
            self.reason = "grasp missed (and the retry missed too)"
            return "failed"
        self.retries_left -= 1
        z_grasp, z_above, g_open, *_ = self._grasp_geometry()
        self.grip_to(g_open, "open after a miss")
        tip = self.tip_table()
        q = self.tip_over(tip[:2], max(tip[2], z_above) + 0.02)
        self.move(q, "back off above the object", floor=self.wc.floor_base(), slow=True)
        self.level = len(self.cfg.look_heights) - 1
        self.iters = 0
        self.lost = 0
        return "center"

    def st_place(self):
        c, wc = self.cfg, self.wc
        if c.place_table is None:
            return "ready"
        _, _, g_open, g_closed, _ = self._grasp_geometry()
        tip = self.tip_table()
        grasp_off = tip[2]   # tip height above the object's base while held (base back at 0)
        z_travel = max(wc.travel_z, c.height + c.approach_clear + 0.03)
        xy = np.array(c.place_table[:2], float)
        q_up = self.tip_over(tip[:2], max(tip[2], z_travel))
        self.move(q_up, "up to travel height", floor=wc.floor_base())
        q_over = self.tip_over(xy, max(tip[2], z_travel))
        if q_over is None:
            self.reason = f"place {xy.tolist()} unreachable"
            return "failed"
        self.move(q_over, f"over the place target table {xy.round(3).tolist()}")
        z_rel = max(wc.min_tip_z, c.height * 0.6 - wc.tip_to_pad + 0.003)
        q_down = self.tip_over(xy, z_rel)
        self.move(q_down, "down to release", floor=wc.floor_base(), slow=True)
        self.grip_to(g_open, "release")
        q_lift = self.tip_over(xy, self._grasp_geometry()[1])   # clear of the object and of the table guard
        self.move(q_lift, "lift clear of the object", floor=wc.floor_base(), slow=True)
        self.log("decision", decision="placed", grasp_offset_z=grasp_off)
        return "ready"

    def st_ready(self):
        self.move(np.array(envm.POSES["ready"], float), "ready")
        return "done"

    # ------------------------------------------------------------------ run
    def run(self) -> dict:
        self.log("start", config=asdict(self.cfg), T_tip_cam=self.T_tip_cam, T_base_table=np.asarray(self.wc.T_base_table),
                 camera=self.cam.to_dict())
        trace = []
        try:
            while self.state not in ("done", "failed"):
                trace.append(self.state)
                nxt = getattr(self, f"st_{self.state}")()
                self.log("transition", to=nxt)
                self.state = nxt
        except Unsafe as e:
            self.reason = f"refused an unsafe motion: {e}"
            self.state = "failed"
        trace.append(self.state)
        res = {"ok": self.state == "done", "reason": self.reason, "trace": trace,
               "estimate_table": None if self.est is None else self.est.round(5).tolist(),
               "observations": self.n_obs, "measured_width": self.measured_width}
        self.log("end", **res)
        return res


# --------------------------------------------------------------------------- the real arm
class DaemonArm:
    """The ``Arm`` protocol over the armctl daemon (ArmClient) + a frame source. Every motion
    is a ``move_joints`` (or ``gripper``) the daemon validates; with ``confirm`` each one waits
    for the operator to type y at the terminal. Never used by the tests."""

    def __init__(self, url: str | None = None, source: str = "dashboard", program: str = "wristcam",
                 speed: float = 0.25, confirm: bool = True, grip_settle: float = 1.5):
        import asyncio
        from interfaces.client import DEFAULT_URL
        self.url = url or DEFAULT_URL
        self.source_spec, self.program = source, program
        self.speed, self.confirm, self.grip_settle = speed, confirm, grip_settle
        self.loop = asyncio.new_event_loop()

    def _run(self, coro):
        return self.loop.run_until_complete(coro)

    def __enter__(self):
        import sys
        from interfaces.client import ArmClient
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "vision"))
        from sources import open_source  # noqa: E402
        self.client = ArmClient(self.url, program=self.program, name=self.program)
        self._run(self.client.__aenter__())
        print(f"connected to {self.url} (backend {self.client.backend}); asking for the lease: approve it in the console")
        self._run(self.client.acquire())
        self.src = open_source(self.source_spec, new_only=False) if self.source_spec in ("dashboard",) or \
            str(self.source_spec).startswith("http") else open_source(self.source_spec)
        return self

    def __exit__(self, *exc):
        try:
            self._run(self.client.__aexit__(*exc))
        finally:
            self.src.close()
            self.loop.close()

    def _ok(self, what: str) -> None:
        if not self.confirm:
            return
        a = input(f"\n>>> {what}\n    type y + Enter to do it, anything else aborts: ").strip().lower()
        if a != "y":
            raise Unsafe(f"operator declined: {what}")

    def q(self) -> np.ndarray:
        self._run(self.client.wait_state(2.0))
        return self.client.q

    def grip(self) -> float:
        self._run(self.client.wait_state(2.0))
        return float(self.client.state.get("grip", np.nan))

    def move_joints(self, q, speed: float | None = None, label: str = "") -> None:
        q = [round(float(x), 4) for x in q]
        q0 = self.q()
        d = np.abs(np.array(q) - q0)
        tip = kin.tip(q, kin.vision_correction())
        self._ok(f"MOVE {label}: joints by up to {d.max():.3f} rad (j{int(d.argmax()) + 1}); tip -> "
                 f"({tip[0]:.3f}, {tip[1]:.3f}, {tip[2]:.3f}) m; q={q}; speed {speed or self.speed:.2f} rad/s")
        self._run(self.client.do("move_joints", q=q, speed=min(speed or self.speed, 0.3), timeout=120))

    def gripper(self, g: float) -> None:
        self._ok(f"GRIPPER -> {g:.2f} (0 closed .. 1 open)")
        self._run(self.client.do("gripper", width=round(float(g), 3)))
        time.sleep(self.grip_settle)

    def frame(self) -> np.ndarray:
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "vision"))
        from sources import grab_one  # noqa: E402
        time.sleep(0.3)
        f = grab_one(self.src)
        if f is None:
            raise RuntimeError(f"no frame from {self.source_spec}")
        return f

    def ready(self) -> None:
        self._ok("MOVE to ready")
        self._run(self.client.do("ready", timeout=120))


def read_q_from_daemon(url: str | None = None) -> np.ndarray:
    """Read-only: the arm's current joint angles (hello + one state; no lease, no motion)."""
    import asyncio
    from interfaces.client import DEFAULT_URL, ArmClient

    async def go():
        async with ArmClient(url or DEFAULT_URL, program="wristcam-read", name="read q") as c:
            await c.wait_state(3.0)
            return c.q
    return asyncio.run(go())
