"""interfaces.wristcam.synth: a ray-cast synthetic wrist camera, and a fake arm, for tests.

Every pixel's ray (from the lens model, so fisheye / rational / pinhole all render exactly) is
intersected with the table plane (a texture: wood-ish grey, the taped calibration sheet, loose
tags) and with simple solids: open-top cups (outer wall, rim, inner wall, inner bottom) and boxes
(optionally a tag on the top face). Supersampled, Lambert-shaded, then blur, noise, JPEG, and the
jaws drawn into the lower image corners (they move with the camera).

``SimArm`` is the fake arm the closed-loop engine is tested against: joint moves are instant
(and checked against the envelope), the camera renders from FK(q) * T_tip_cam_true, closing the
gripper on a cup (or on a box, at its width across the fingers) stalls there (that is how
``grasp.holding`` detects a grasp), a held object moves with the tip, opening drops it. Fingers
that would hit a cup or a box are recorded in ``world.collisions``.

    .venv/bin/python -m interfaces.wristcam.synth --out /tmp/wristcam_synth    # demo frames
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

from armctl import envelope as envm
from armctl import kin

from .board import BoardSpec
from .camera import JAW_POLYGONS, T_TIP_CAM_NOMINAL, CameraModel, inv_T, make_T, rot_axis

LIGHT = np.array([-0.3, -0.4, 1.0]) / np.linalg.norm([-0.3, -0.4, 1.0])
AMBIENT = 0.45
TABLE_BGR = (112, 128, 140)

# Ground truth used by the tests: a wide-angle fisheye (~105 deg HFOV) mounted 6 cm behind the
# tip, a few mm off the axis and tilted ~3 deg.
TRUE_CAMERA = CameraModel("fisheye", [[700.0, 0, 646.0], [0, 699.0, 357.0], [0, 0, 1]],
                          [0.045, -0.018, 0.006, -0.0012], (1280, 720))
# "True" mounting for the synthetic tests: the MEASURED nominal (camera ~60 deg off the finger
# axis, ~6 cm behind the tip; camera.T_TIP_CAM_NOMINAL) plus a few degrees and mm of error that
# hand-eye has to recover.
TRUE_T_TIP_CAM = make_T(rot_axis([0.3, 1.0, 0.2], 3.0) @ T_TIP_CAM_NOMINAL[:3, :3],
                        T_TIP_CAM_NOMINAL[:3, 3] + np.array([0.004, -0.007, 0.002]))
T_BASE_TABLE_TRUE = make_T(rot_axis([0, 0, 1], 4.0) @ rot_axis([1, 0.3, 0], 0.4), [0.305, 0.012, 0.002])


@dataclass
class Cup:
    xy: tuple                       # centre, table frame (m)
    radius: float = 0.03            # outer radius at the rim
    height: float = 0.08
    wall: float = 0.0025
    bottom: float = 0.005
    z0: float = 0.0                 # base height (table frame); > 0 while carried
    color: tuple = (70, 80, 205)    # BGR, a red paper cup


@dataclass
class Box:
    xy: tuple
    size: tuple = (0.05, 0.05, 0.04)   # lx, ly, h (m)
    yaw: float = 0.0
    color: tuple = (60, 170, 70)       # BGR, green
    tag_id: int | None = None
    tag_mm: float = 40.0
    z0: float = 0.0


@dataclass
class FlatTag:
    xy: tuple
    id: int
    yaw: float = 0.0
    size_mm: float = 40.0


def _bilinear(tex, u, v):
    """Sample tex (H,W,3 float) at float pixel coords (u=col, v=row); outside -> edge."""
    H, W = tex.shape[:2]
    u = np.clip(u, 0, W - 1.001)
    v = np.clip(v, 0, H - 1.001)
    u0, v0 = np.floor(u).astype(int), np.floor(v).astype(int)
    fu, fv = (u - u0)[:, None], (v - v0)[:, None]
    return ((1 - fu) * (1 - fv) * tex[v0, u0] + fu * (1 - fv) * tex[v0, u0 + 1]
            + (1 - fu) * fv * tex[v0 + 1, u0] + fu * fv * tex[v0 + 1, u0 + 1])


def _tag_texture(dict_name, tag_id, px=240, border_frac=0.25):
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    m = cv2.aruco.generateImageMarker(d, int(tag_id), px, borderBits=1)
    b = int(px * border_frac)
    img = np.full((px + 2 * b, px + 2 * b), 240, np.uint8)
    img[b:b + px, b:b + px] = np.where(m > 128, 240, 25)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR).astype(np.float32), (px + 2 * b) / px


@dataclass
class World:
    T_base_table: np.ndarray = field(default_factory=lambda: T_BASE_TABLE_TRUE.copy())
    spec: BoardSpec = field(default_factory=BoardSpec)
    board: bool = True
    cups: list = field(default_factory=list)
    boxes: list = field(default_factory=list)
    tags: list = field(default_factory=list)
    extent: tuple = (-0.32, 0.34, -0.36, 0.30)   # table-frame x0, x1, y0, y1 of the texture
    px_per_mm: float = 4.0
    collisions: list = field(default_factory=list)
    _tex: np.ndarray | None = field(default=None, repr=False)
    _rays: dict = field(default_factory=dict, repr=False)

    # ------------------------------------------------------------------ texture
    def texture(self) -> np.ndarray:
        if self._tex is not None:
            return self._tex
        x0, x1, y0, y1 = self.extent
        s = self.px_per_mm * 1000
        W, H = int(round((x1 - x0) * s)), int(round((y1 - y0) * s))
        rng = np.random.default_rng(7)
        low = cv2.resize(rng.normal(0, 1, (H // 160 + 2, W // 160 + 2)).astype(np.float32), (W, H),
                         interpolation=cv2.INTER_CUBIC)
        grain = cv2.GaussianBlur(rng.normal(0, 1, (H, W)).astype(np.float32), (0, 0), 3)
        base = np.array(TABLE_BGR, np.float32)
        T = base[None, None, :] * (1 + 0.08 * low[..., None] + 0.05 * grain[..., None])

        def to_px(x, y):
            return (x - x0) * s, (y1 - y) * s

        def paste(img_bgr, cx, cy, size_m, yaw=0.0):
            """Paste a square/rect image centred at table (cx, cy), x along table x rotated by yaw."""
            h, w = img_bgr.shape[:2]
            sx = size_m[0] / w if isinstance(size_m, tuple) else size_m / w
            sy = size_m[1] / h if isinstance(size_m, tuple) else size_m / h
            c, sn = np.cos(yaw), np.sin(yaw)
            # image px (u, v) -> table (x, y): x right, y up the image
            A = np.array([[c * sx, sn * sy, 0], [sn * sx, -c * sy, 0]], float)
            A[:, 2] = [cx, cy] - A[:, :2] @ [w / 2, h / 2]
            Ttex = np.array([[s, 0, -x0 * s], [0, -s, y1 * s]], float)
            M = Ttex @ np.vstack([A, [0, 0, 1]])
            warped = cv2.warpAffine(img_bgr, M, (W, H), flags=cv2.INTER_AREA, borderValue=(0, 0, 0))
            mask = cv2.warpAffine(np.ones((h, w), np.float32), M, (W, H), flags=cv2.INTER_LINEAR)
            T[:] = T * (1 - mask[..., None]) + warped * mask[..., None]

        if self.board:
            spec = self.spec
            sheet = np.full((int(210 * 4), int(297 * 4), 3), 238, np.float32)
            bw, bh = spec.squares[0] * spec.square_mm, spec.squares[1] * spec.square_mm
            bimg = spec.board().generateImage((int(round(bw * 4)), int(round(bh * 4))), marginSize=0, borderBits=1)
            bimg = np.where(bimg > 128, 238, 22).astype(np.float32)
            oy, ox = int(round((210 - bh) / 2 * 4)), int(round((297 - bw) / 2 * 4))
            sheet[oy:oy + bimg.shape[0], ox:ox + bimg.shape[1]] = bimg[..., None]
            # sheet centred on the board centre = table origin, page "up" = table +y
            paste(sheet, 0.0, 0.0, (0.297, 0.210))
        for t in self.tags:
            img, k = _tag_texture(self.spec.tag_dict, t.id)
            paste(img, t.xy[0], t.xy[1], t.size_mm / 1000 * k, t.yaw)
        self._tex = np.clip(T, 0, 255).astype(np.float32)
        return self._tex

    # ------------------------------------------------------------------ rays
    def _cam_rays(self, cam: CameraModel, ss: int) -> np.ndarray:
        key = (cam.model, tuple(cam.K.ravel()), tuple(cam.D), cam.size, ss)
        if key not in self._rays:
            w, h = cam.size
            u = (np.arange(w * ss) + 0.5) / ss - 0.5
            v = (np.arange(h * ss) + 0.5) / ss - 0.5
            uu, vv = np.meshgrid(u, v)
            r = cam.rays(np.c_[uu.ravel(), vv.ravel()]).astype(np.float32)
            self._rays = {key: r}
        return self._rays[key]

    # ------------------------------------------------------------------ render
    def render(self, cam: CameraModel, T_base_cam, *, ss: int = 2, noise: float = 2.0, blur: float = 0.5,
               jaws: bool = True, gain: float = 1.0, seed: int = 0, jpeg_q: int = 92) -> np.ndarray:
        w, h = cam.size
        rays = self._cam_rays(cam, ss)
        T_tc = inv_T(self.T_base_table) @ np.asarray(T_base_cam, float)
        C = T_tc[:3, 3].astype(np.float32)
        d = rays @ T_tc[:3, :3].T.astype(np.float32)
        N = len(d)
        dz = d[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            t_best = np.where(dz < -1e-6, -C[2] / dz, np.inf).astype(np.float32)
        P = C[None, :] + t_best[:, None] * d
        x0, x1, y0, y1 = self.extent
        s = self.px_per_mm * 1000
        mapx = ((P[:, 0] - x0) * s).reshape(h * ss, w * ss).astype(np.float32)
        mapy = ((y1 - P[:, 1]) * s).reshape(h * ss, w * ss).astype(np.float32)
        tex = self.texture()
        col = cv2.remap(tex, mapx, mapy, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                        borderValue=TABLE_BGR).reshape(-1, 3)
        col *= AMBIENT + (1 - AMBIENT) * LIGHT[2]
        col[~np.isfinite(t_best)] = (90, 90, 90)
        for cup in self.cups:
            self._cup(cup, C, d, t_best, col)
        for box in self.boxes:
            self._box(box, C, d, t_best, col)
        img = col.reshape(h * ss, w * ss, 3)
        img = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA) * gain
        if jaws:
            for poly in JAW_POLYGONS:
                p = np.array([[x * w, y * h] for x, y in poly], np.int32)
                cv2.fillPoly(img, [p], (38, 38, 42), cv2.LINE_AA)
        rng = np.random.default_rng(seed)
        if blur > 0:
            img = cv2.GaussianBlur(img, (0, 0), blur)
        if noise > 0:
            img = img + rng.normal(0, noise, img.shape).astype(np.float32)
        img = np.clip(img, 0, 255).astype(np.uint8)
        if jpeg_q:
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, jpeg_q])
            img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        return img

    @staticmethod
    def _shade(base, normals):
        lam = AMBIENT + (1 - AMBIENT) * np.clip(normals @ LIGHT, 0, 1)
        return np.asarray(base, np.float32)[None, :] * lam[:, None].astype(np.float32)

    def _cup(self, cup: Cup, C, d, t_best, col):
        cx, cy = cup.xy
        R, Ri, H = cup.radius, cup.radius - cup.wall, cup.z0 + cup.height
        zb = cup.z0 + cup.bottom
        if C[2] <= H:
            return  # renderer assumes the camera is above the rim
        with np.errstate(divide="ignore", invalid="ignore"):
            t_rim = (H - C[2]) / d[:, 2]
        cand = np.where((d[:, 2] < -1e-6) & (t_rim > 0) & (t_rim < t_best))[0]
        if not len(cand):
            return
        dd = d[cand]
        ox, oy = C[0] - cx, C[1] - cy
        tr = t_rim[cand]
        rx, ry = ox + tr * dd[:, 0], oy + tr * dd[:, 1]
        r_rim = np.hypot(rx, ry)
        a = dd[:, 0] ** 2 + dd[:, 1] ** 2
        b = 2 * (ox * dd[:, 0] + oy * dd[:, 1])
        tnew = np.full(len(cand), np.inf, np.float32)
        cnew = np.zeros((len(cand), 3), np.float32)
        # rim annulus
        m = (r_rim >= Ri) & (r_rim <= R)
        tnew[m] = tr[m]
        cnew[m] = self._shade(np.minimum(np.array(cup.color) * 1.15 + 25, 255), np.tile([0, 0, 1.0], (m.sum(), 1)))
        # outer wall (ray outside R at rim height, then hits the wall below)
        mo = r_rim > R
        c0 = ox * ox + oy * oy - R * R
        disc = b * b - 4 * a * c0
        mo &= disc > 0
        t_o = (-b - np.sqrt(np.maximum(disc, 0))) / (2 * np.maximum(a, 1e-12))
        z_o = C[2] + t_o * dd[:, 2]
        mo &= (t_o > tr) & (z_o >= cup.z0) & (z_o <= H)
        if mo.any():
            px, py = ox + t_o[mo] * dd[mo, 0], oy + t_o[mo] * dd[mo, 1]
            n = np.c_[px / R, py / R, np.zeros(mo.sum())]
            tnew[mo] = t_o[mo]
            cnew[mo] = self._shade(cup.color, n)
        # through the opening: inner wall or inner bottom
        mi = r_rim < Ri
        if mi.any():
            ci = ox * ox + oy * oy - Ri * Ri
            disc_i = b * b - 4 * a * ci
            t_i = (-b + np.sqrt(np.maximum(disc_i, 0))) / (2 * np.maximum(a, 1e-12))
            z_i = C[2] + t_i * dd[:, 2]
            wall = mi & (z_i >= zb)
            bot = mi & ~wall
            if wall.any():
                px, py = ox + t_i[wall] * dd[wall, 0], oy + t_i[wall] * dd[wall, 1]
                n = np.c_[-px / Ri, -py / Ri, np.zeros(wall.sum())]
                tnew[wall] = t_i[wall]
                cnew[wall] = self._shade(np.array(cup.color) * 0.8, n) * 0.85
            if bot.any():
                t_b = (zb - C[2]) / dd[bot, 2]
                tnew[bot] = t_b
                cnew[bot] = self._shade(np.array(cup.color) * 0.8, np.tile([0, 0, 1.0], (bot.sum(), 1))) * 0.6
        hit = tnew < t_best[cand]
        idx = cand[hit]
        t_best[idx] = tnew[hit]
        col[idx] = cnew[hit]

    def _box(self, box: Box, C, d, t_best, col):
        c, s = np.cos(box.yaw), np.sin(box.yaw)
        Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], np.float32)
        ctr = np.array([box.xy[0], box.xy[1], box.z0 + box.size[2] / 2], np.float32)
        Cl = Rz.T @ (C - ctr)
        dl = d @ Rz  # = (Rz^T d^T)^T
        half = np.array(box.size, np.float32) / 2
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (-half[None, :] - Cl[None, :]) / dl
            t2 = (half[None, :] - Cl[None, :]) / dl
        tmin = np.nanmax(np.minimum(t1, t2), axis=1)
        tmax = np.nanmin(np.maximum(t1, t2), axis=1)
        hit = (tmax >= tmin) & (tmin > 0) & (tmin < t_best)
        idx = np.where(hit)[0]
        if not len(idx):
            return
        tm = tmin[idx]
        face = np.argmax(np.minimum(t1[idx], t2[idx]), axis=1)
        sign = np.sign(-dl[idx, face])
        nl = np.zeros((len(idx), 3), np.float32)
        nl[np.arange(len(idx)), face] = sign
        n = nl @ Rz.T
        cnew = self._shade(box.color, n)
        if box.tag_id is not None:
            top = face == 2
            if top.any():
                Pl = Cl[None, :] + tm[top, None] * dl[idx[top]]
                tex, k = _tag_texture(self.spec.tag_dict, box.tag_id)
                side = box.tag_mm / 1000 * k
                u = (Pl[:, 0] / side + 0.5) * tex.shape[1] - 0.5
                v = (-Pl[:, 1] / side + 0.5) * tex.shape[0] - 0.5
                inside = (np.abs(Pl[:, 0]) < side / 2) & (np.abs(Pl[:, 1]) < side / 2)
                sampled = _bilinear(tex, u, v) * (AMBIENT + (1 - AMBIENT) * LIGHT[2])
                sub = cnew[top]
                sub[inside] = sampled[inside]
                cnew[top] = sub
        t_best[idx] = tm
        col[idx] = cnew


class SimArm:
    """Fake arm + wrist camera over a ``World`` (instant moves, gripper stall on a cup)."""

    def __init__(self, world: World, cam: CameraModel = TRUE_CAMERA, T_tip_cam=TRUE_T_TIP_CAM,
                 q0=None, max_width: float = 0.094, joint_noise: float = 0.0, miss_first: bool = False,
                 seed: int = 0, ss: int = 2):
        self.world, self.cam, self.T_tip_cam = world, cam, np.asarray(T_tip_cam, float)
        self._q = np.array(envm.POSES["ready"] if q0 is None else q0, float)
        self._grip = 0.0
        self.max_width = max_width
        self.held: tuple | None = None   # (cup or box, offset table xyz from tip)
        self.rng = np.random.default_rng(seed)
        self.joint_noise = joint_noise
        self.miss_first = miss_first
        self.n_frames = 0
        self.ss = ss
        self.moves: list = []

    # Arm protocol ------------------------------------------------------
    def q(self) -> np.ndarray:
        return self._q + self.rng.normal(0, self.joint_noise, 6) if self.joint_noise else self._q.copy()

    def grip(self) -> float:
        return float(self._grip)

    def _tip_table(self, q=None):
        T = inv_T(self.world.T_base_table) @ kin.fk_vision(self._q if q is None else q)
        return T

    def move_joints(self, q, speed: float | None = None) -> None:
        q = np.asarray(q, float)
        viol = envm.DEFAULT.check_point(q)
        if viol:
            raise RuntimeError("sim refused: " + "; ".join(v.message for v in viol))
        self.moves.append((self._q.copy(), q.copy(), speed))
        self._q = q.copy()
        T = self._tip_table()
        if self.held:
            cup, off = self.held
            cup.xy = (T[0, 3] + off[0], T[1, 3] + off[1])
            cup.z0 = T[2, 3] + off[2]
        else:
            self._check_collision(T)

    def _finger_points(self, T):
        fdir = T[:3, 1].copy()
        fdir[2] = 0
        fdir /= max(np.linalg.norm(fdir), 1e-9)
        half = self._grip * self.max_width / 2
        return [T[:3, 3] + half * fdir, T[:3, 3] - half * fdir]

    def _check_collision(self, T):
        for box in self.world.boxes:
            if T[2, 3] >= box.z0 + box.size[2]:
                continue
            for f in self._finger_points(T):
                lx, ly = _box_local(box, f[0], f[1])
                if abs(lx) < box.size[0] / 2 + 0.003 and abs(ly) < box.size[1] / 2 + 0.003:
                    self.world.collisions.append({"q": self._q.round(4).tolist(), "box": list(box.xy),
                                                  "finger_local": [round(float(lx), 4), round(float(ly), 4)],
                                                  "tip_z": round(float(T[2, 3]), 4)})
        for cup in self.world.cups:
            if T[2, 3] >= cup.z0 + cup.height:
                continue
            for f in self._finger_points(T):
                r = np.hypot(f[0] - cup.xy[0], f[1] - cup.xy[1])
                if r < cup.radius + 0.003:
                    self.world.collisions.append({"q": self._q.round(4).tolist(), "finger_r": round(float(r), 4),
                                                  "tip_z": round(float(T[2, 3]), 4)})

    def gripper(self, g: float) -> None:
        g = float(np.clip(g, 0, 1))
        T = self._tip_table()
        if g < self._grip and not self.held:   # closing
            for cup in self.world.cups:
                off = np.hypot(T[0, 3] - cup.xy[0], T[1, 3] - cup.xy[1])
                zrel = T[2, 3] - cup.z0
                stall = 2 * cup.radius / self.max_width
                if off < 0.5 * cup.radius and 0.005 < zrel < cup.height - 0.003 and g < stall:
                    if self.miss_first:
                        self.miss_first = False
                        break
                    self._grip = stall + self.rng.normal(0, 0.004)
                    self.held = (cup, np.array([cup.xy[0] - T[0, 3], cup.xy[1] - T[1, 3], cup.z0 - T[2, 3]]))
                    return
            fdir = T[:3, 1]
            for box in self.world.boxes:
                lx, ly = _box_local(box, T[0, 3], T[1, 3])
                zrel = T[2, 3] - box.z0
                a = np.arctan2(fdir[1], fdir[0]) - box.yaw
                width = abs(box.size[0] * np.cos(a)) + abs(box.size[1] * np.sin(a))   # across the fingers
                stall = width / self.max_width
                if (abs(lx) < 0.35 * box.size[0] and abs(ly) < 0.35 * box.size[1] and 0.005 < zrel < box.size[2] - 0.003
                        and g < stall <= 1.0):
                    if self.miss_first:
                        self.miss_first = False
                        break
                    self._grip = stall + self.rng.normal(0, 0.004)
                    self.held = (box, np.array([box.xy[0] - T[0, 3], box.xy[1] - T[1, 3], box.z0 - T[2, 3]]))
                    return
            self._grip = g
            return
        if g > self._grip and self.held:      # opening: drop it where it is
            obj, _ = self.held
            obj.z0 = 0.0
            self.held = None
        self._grip = g

    def frame(self) -> np.ndarray:
        self.n_frames += 1
        return self.world.render(self.cam, kin.fk_vision(self._q) @ self.T_tip_cam, seed=self.n_frames, ss=self.ss)


def _box_local(box: Box, x: float, y: float) -> tuple[float, float]:
    """Table-frame (x, y) in the box's own frame (centre, axes along its sides)."""
    c, s = np.cos(box.yaw), np.sin(box.yaw)
    dx, dy = x - box.xy[0], y - box.xy[1]
    return c * dx + s * dy, -s * dx + c * dy


def demo_world() -> World:
    return World(cups=[Cup((-0.01, -0.16))], boxes=[Box((0.03, 0.17), tag_id=2, yaw=0.4)],
                 tags=[FlatTag((0.2, -0.05), 3)])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    world = demo_world()
    for name, xyz in (("board", (0.30, 0.0, 0.20)), ("cup", (0.29, -0.15, 0.17)), ("box", (0.32, 0.16, 0.17))):
        from .camera import finger_dir_for_cam_axis
        r = kin.ik(xyz, approach_dir=finger_dir_for_cam_axis([0, 0, -1], xyz), check_table=False, corr=kin.vision_correction())
        img = world.render(TRUE_CAMERA, kin.fk_vision(r.q) @ TRUE_T_TIP_CAM)
        cv2.imwrite(str(out / f"{name}.jpg"), img)
        print("wrote", out / f"{name}.jpg", "q", r.q.round(3).tolist())


if __name__ == "__main__":
    main()
