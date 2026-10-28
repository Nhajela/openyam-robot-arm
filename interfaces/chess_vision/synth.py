"""Synthetic renders of a chess board with ArUco corner markers, for tests and demos (no camera).

A pinhole camera looks at the board plane (rendered by a homography from a texture) and the
pieces, drawn as shaded two-tier cylinders (a wide base and a narrower body, heights by piece
type) with the painter's algorithm, so tall pieces lean into and hide their neighbours exactly as
a real oblique camera sees them. Post-processing adds a lighting change (gain, gradient, tint),
blur, noise and JPEG compression, and optionally a hand covering one corner marker.

    .venv/bin/python -m interfaces.chess_vision.synth --out /tmp/chess_synth   # writes demo frames
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from pathlib import Path

import chess
import cv2
import numpy as np

from .board_geom import CORNERS, BoardSpec

LIGHT_SQ = (160, 200, 222)   # BGR
DARK_SQ = (62, 98, 142)
FRAME = (95, 125, 150)
TABLE = (118, 128, 132)
WHITE_PC = (222, 230, 236)
BLACK_PC = (40, 40, 46)
SKIN = (120, 150, 205)
# piece type -> (base radius, base height, body radius, total height), in squares
SHAPES = {chess.PAWN: (0.31, 0.22, 0.19, 1.0), chess.ROOK: (0.35, 0.25, 0.26, 1.2),
          chess.KNIGHT: (0.35, 0.25, 0.23, 1.45), chess.BISHOP: (0.34, 0.25, 0.19, 1.7),
          chess.QUEEN: (0.37, 0.27, 0.21, 1.95), chess.KING: (0.37, 0.27, 0.21, 2.15)}
LIGHT_DIR = np.array([-0.35, -0.45, 1.0]) / np.linalg.norm([-0.35, -0.45, 1.0])
AMBIENT = 0.38


@dataclass
class Cam:
    name: str
    C: np.ndarray                 # camera centre, board mm
    target: np.ndarray            # look-at point, board mm
    f: float = 900.0
    size: tuple = (1280, 720)

    @property
    def K(self) -> np.ndarray:
        w, h = self.size
        return np.array([[self.f, 0, w / 2 - 0.5], [0, self.f, h / 2 - 0.5], [0, 0, 1]], float)

    @property
    def R(self) -> np.ndarray:
        z = self.target - self.C
        z = z / np.linalg.norm(z)
        up = np.array([0, 0, 1.0])
        x = np.cross(z, up)
        if np.linalg.norm(x) < 1e-6:
            x = np.cross(z, [0, 1.0, 0])
        x = x / np.linalg.norm(x)
        y = np.cross(z, x)
        return np.vstack([x, y, z])

    def project(self, P: np.ndarray, K: np.ndarray | None = None) -> np.ndarray:
        K = self.K if K is None else K
        Pc = (np.asarray(P, float).reshape(-1, 3) - self.C) @ self.R.T
        q = Pc @ K.T
        return q[:, :2] / q[:, 2:3]

    def ground_H(self, K: np.ndarray | None = None) -> np.ndarray:
        """Board mm (x, y, 1) on z=0 -> image px."""
        K = self.K if K is None else K
        R = self.R
        t = -R @ self.C
        return K @ np.c_[R[:, 0], R[:, 1], t]

    def true_img2board(self) -> np.ndarray:
        return np.linalg.inv(self.ground_H())


def standard_cameras(spec: BoardSpec) -> dict[str, Cam]:
    """Overhead (slightly off-axis), a side camera behind white, and one from the a-file side."""
    c = np.array([spec.size_mm / 2, spec.size_mm / 2, 0.0])
    return {
        "top": Cam("top", c + [15, -40, 720], c + [0, 5, 0], f=1000),
        "side": Cam("side", c + [40, -480, 520], c + [0, 15, 0], f=1050),
        "left": Cam("left", c + [-470, 60, 560], c + [10, 0, 0], f=1050),
    }


@dataclass
class Lighting:
    gain: float = 1.0
    grad: tuple = (0.0, 0.0)      # relative gain change across the image width / height
    tint: tuple = (1.0, 1.0, 1.0)  # per-channel BGR gain

    @staticmethod
    def random(rng: np.random.Generator) -> "Lighting":
        return Lighting(float(rng.uniform(0.65, 1.25)), tuple(rng.uniform(-0.3, 0.3, 2)),
                        tuple(rng.uniform(0.9, 1.1, 3)))


@dataclass
class Scene:
    spec: BoardSpec = field(default_factory=BoardSpec)
    tex_px_per_mm: float = 4.0
    marker_offset_mm: dict = field(default_factory=dict)   # corner -> (dx, dy) placement error
    marker_rot: dict = field(default_factory=dict)         # corner -> k (x 90 degrees)
    frame_mm: float = 18.0                                 # wooden border around the squares

    def texture(self) -> tuple[np.ndarray, np.ndarray]:
        s, S, tr = self.spec.square_mm, self.spec.size_mm, self.tex_px_per_mm
        ext = self.spec.gap_mm + self.spec.marker_mm * 1.3 + 40
        x0, y1 = -ext, S + ext
        n = int(round((S + 2 * ext) * tr))
        T = np.empty((n, n, 3), np.uint8)
        T[:] = TABLE

        def rect(xa, ya, xb, yb, col):
            u0, u1 = int(round((xa - x0) * tr)), int(round((xb - x0) * tr))
            v0, v1 = int(round((y1 - yb) * tr)), int(round((y1 - ya) * tr))
            T[v0:v1, u0:u1] = col

        fm = self.frame_mm
        rect(-fm, -fm, S + fm, S + fm, FRAME)
        for i in range(64):
            f, r = i % 8, i // 8
            rect(f * s, r * s, (f + 1) * s, (r + 1) * s, DARK_SQ if (f + r) % 2 == 0 else LIGHT_SQ)
        d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.spec.dict_name))
        m = self.spec.marker_mm
        for corner in CORNERS:
            cx, cy = self.spec.marker_center(corner) + np.array(self.marker_offset_mm.get(corner, (0, 0)))
            q = m * 0.62  # white paper around the marker
            rect(cx - q, cy - q, cx + q, cy + q, (245, 245, 245))
            side = int(round(m * tr))
            img = cv2.aruco.generateImageMarker(d, self.spec.ids[corner], side, borderBits=1)
            img = np.rot90(img, -self.marker_rot.get(corner, 0))
            u0, v0 = int(round((cx - m / 2 - x0) * tr)), int(round((y1 - (cy + m / 2)) * tr))
            T[v0:v0 + side, u0:u0 + side] = img[..., None]
        A = np.array([[1 / tr, 0, x0], [0, -1 / tr, y1], [0, 0, 1]], float)  # texture px -> board mm
        return T, A


_TEX_CACHE: dict = {}


def _tex(scene: Scene):
    key = (repr(scene.spec), scene.tex_px_per_mm, repr(sorted(scene.marker_offset_mm.items())),
           repr(sorted(scene.marker_rot.items())), scene.frame_mm)
    if key not in _TEX_CACHE:
        _TEX_CACHE.clear()
        _TEX_CACHE[key] = scene.texture()
    return _TEX_CACHE[key]


def _shade(col, k):
    return tuple(int(min(255, c * k)) for c in col)


def _cylinder(img, cam, K, center, r, z0, z1, col, n=28):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.c_[np.cos(a), np.sin(a)]
    bot = np.c_[center + r * ring, np.full(n, z0)]
    top = np.c_[center + r * ring, np.full(n, z1)]
    pb, pt = cam.project(bot, K), cam.project(top, K)
    view = cam.C[:2] - center
    facets = []
    for i in range(n):
        j = (i + 1) % n
        nm = (ring[i] + ring[j]) / 2
        mid = center + r * nm
        if np.dot(nm, cam.C[:2] - mid) <= 0:
            continue
        lam = AMBIENT + (1 - AMBIENT) * max(0.0, float(np.dot(np.r_[nm, 0] / np.linalg.norm(nm), LIGHT_DIR)))
        depth = -np.dot(nm, view)
        facets.append((depth, np.array([pb[i], pb[j], pt[j], pt[i]]), lam))
    for _, quad, lam in sorted(facets, key=lambda f: f[0]):
        cv2.fillConvexPoly(img, np.round(quad * 16).astype(np.int32), _shade(col, lam), cv2.LINE_AA, 4)
    lam_top = AMBIENT + (1 - AMBIENT) * LIGHT_DIR[2]
    cv2.fillConvexPoly(img, np.round(pt * 16).astype(np.int32), _shade(col, lam_top * 1.03), cv2.LINE_AA, 4)


def render(scene: Scene, cam: Cam, board: chess.Board | None = None, *, lighting: Lighting | None = None,
           hand_over: str | list | None = None, seed: int = 0, jitter_sq: float = 0.07, noise: float = 2.5,
           blur: float = 0.7, jpeg_q: int = 88, ss: int = 2) -> np.ndarray:
    """Render ``board`` (None = empty) seen by ``cam``. Returns a BGR uint8 image."""
    rng = np.random.default_rng(seed)
    spec = scene.spec
    w, h = cam.size
    Kss = cam.K.copy()
    Kss[:2] *= ss
    Kss[0, 2] += (ss - 1) / 2
    Kss[1, 2] += (ss - 1) / 2
    T, A = _tex(scene)
    lam_floor = AMBIENT + (1 - AMBIENT) * LIGHT_DIR[2]
    Tl = np.clip(T.astype(np.float32) * lam_floor, 0, 255).astype(np.uint8)
    img = cv2.warpPerspective(Tl, cam.ground_H(Kss) @ A, (w * ss, h * ss), flags=cv2.INTER_AREA,
                              borderMode=cv2.BORDER_CONSTANT, borderValue=tuple(int(c * lam_floor) for c in TABLE))
    if board is not None:
        s = spec.square_mm
        pieces = []
        for sq, pc in board.piece_map().items():
            c = spec.square_center(sq) + rng.uniform(-jitter_sq, jitter_sq, 2) * s
            pieces.append((np.linalg.norm(cam.C[:2] - c) + 0.0, c, pc))
        for _, c, pc in sorted(pieces, key=lambda p: -p[0]):
            rb, hb, rt, ht = SHAPES[pc.piece_type]
            col = WHITE_PC if pc.color == chess.WHITE else BLACK_PC
            _cylinder(img, cam, Kss, c, rb * s, 0.0, hb * s, col)
            _cylinder(img, cam, Kss, c, rt * s, hb * s, ht * s, col)
            # a small head so tops are not flat discs
            _cylinder(img, cam, Kss, c, rt * 1.25 * s, ht * s * 0.86, ht * s, _shade(col, 0.97))
    for corner in ([hand_over] if isinstance(hand_over, str) else (hand_over or [])):
        mc = np.r_[spec.marker_center(corner), 30.0]
        p = cam.project([mc], Kss)[0]
        span = np.linalg.norm(cam.project([mc + [spec.marker_mm, 0, 0]], Kss)[0] - p)
        out = p - cam.project([np.r_[spec.size_mm / 2, spec.size_mm / 2, 30.0]], Kss)[0]
        out = out / (np.linalg.norm(out) + 1e-9)
        cv2.ellipse(img, tuple(int(v) for v in p), (int(span * 1.0), int(span * 0.75)),
                    float(np.degrees(np.arctan2(out[1], out[0]))), 0, 360, SKIN, -1, cv2.LINE_AA)
        arm = np.array([p + out * span * 0.5 + np.array([-out[1], out[0]]) * span * k for k in (-0.55, 0.55)])
        far = arm + out * 4000
        cv2.fillConvexPoly(img, np.round(np.vstack([arm, far[::-1]])).astype(np.int32), SKIN, cv2.LINE_AA)
    img = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA).astype(np.float32)
    L = lighting or Lighting()
    xs = (np.arange(w) / w - 0.5)[None, :]
    ys = (np.arange(h) / h - 0.5)[:, None]
    field_ = L.gain * (1 + L.grad[0] * xs + L.grad[1] * ys)
    img = img * field_[..., None] * np.array(L.tint, np.float32)
    if blur > 0:
        img = cv2.GaussianBlur(img, (0, 0), blur)
    if noise > 0:
        img = img + rng.normal(0, noise, img.shape).astype(np.float32)
    img = np.clip(img, 0, 255).astype(np.uint8)
    if jpeg_q:
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, jpeg_q])
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    return img


class FrameBox:
    """A source whose current frame the caller sets (tests, replay). ``read()`` returns it."""
    live = True

    def __init__(self, name: str, frame: np.ndarray | None = None):
        self.name, self.frame = name, frame

    def read(self):
        return self.frame

    def close(self):
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--fen", default=chess.STARTING_FEN)
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    scene = Scene()
    for name, cam in standard_cameras(scene.spec).items():
        (out / name).mkdir(exist_ok=True)
        cv2.imwrite(str(out / name / "0_empty.jpg"), render(scene, cam, None))
        cv2.imwrite(str(out / name / "1_start.jpg"), render(scene, cam, chess.Board()))
        cv2.imwrite(str(out / name / "2_position.jpg"), render(scene, cam, chess.Board(a.fen), lighting=Lighting(0.8, (0.2, -0.1))))
    print("wrote", out, "(one folder per camera: empty, start, position; usable as calibrate/watch sources)")


if __name__ == "__main__":
    main()
