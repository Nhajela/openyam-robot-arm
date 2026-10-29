"""interfaces.wristcam.locate: find an object in one wrist-camera frame taken at a known q.

Every finder implements the ``ObjectFinder`` protocol:

    det = finder.find(frame, view, hint=None)  ->  Detection | None

``view`` (``View.from_q``) carries the lens model, the camera pose ``T_base_cam`` (FK(q) @
T_tip_cam from hand-eye) and the table frame. A ``Detection`` gives the object's grasp-relevant
point in the base and table frames (cup: rim centre; tagged object: tag centre on its top face),
its yaw, size, and an uncertainty ``sigma_xy`` (m).

Finders:

* ``CupFinder`` (primary, tag-free): an open-top cylinder seen from above. The frame is resampled
  onto the RIM PLANE (table plane raised by the cup height: ray-plane intersection per pixel, so
  the fisheye distortion and the view tilt are gone), where the rim is an exact circle of the
  cup's radius. Hough circles with a radius window propose centres, a RANSAC + least-squares
  circle fit on the edge points refines them, and the candidate with the widest angular support
  wins (the jaws hiding the lower image corners only remove part of an arc). Gives the rim
  centre, a diameter estimate (from the circle at the known rim height) and sigma (fit noise +
  rim-height uncertainty x how obliquely the rim is seen).
* ``TagFinder``: ArUco tags (ids and sizes configurable, default the 40 mm DICT_4X4_50 ids 2-4 of
  print/wristcam_tags_A4.pdf) stuck on top of objects: full 6-DoF pose by PnP.
* ``ColorBlobFinder``: the largest blob in an HSV range, resampled onto a plane at a given height.
* ``ClaudeVisionFinder``: the seam for a Claude-vision finder (no API calls here): it would ask for
  the object's pixel and turn it into a table point with ``View.pixel_to_table``.

CLI (offline, on a saved frame):

    .venv/bin/python -m interfaces.wristcam.locate --image f.png --q -0.4,1.7,1.59,-1.46,0,0 --finder cup
"""
from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Protocol, runtime_checkable

import cv2
import numpy as np

from armctl import kin

from .board import BoardSpec
from .camera import (DEFAULT_HANDEYE, DEFAULT_INTRINSICS, T_TIP_CAM_NOMINAL, CameraModel, inv_T, load_handeye,
                     make_T, nominal_camera)


@dataclass
class View:
    cam: CameraModel
    T_base_cam: np.ndarray
    T_base_table: np.ndarray
    q: list | None = None

    @classmethod
    def from_q(cls, q, cam: CameraModel, T_tip_cam=None, T_base_table=None) -> "View":
        T_tc = T_TIP_CAM_NOMINAL if T_tip_cam is None else np.asarray(T_tip_cam, float)
        T_bt = np.eye(4) if T_base_table is None else np.asarray(T_base_table, float)
        return cls(cam, kin.fk_vision(q) @ T_tc, T_bt, [float(x) for x in q])

    @property
    def T_table_cam(self) -> np.ndarray:
        return inv_T(self.T_base_table) @ self.T_base_cam

    def table_to_base(self, P) -> np.ndarray:
        P = np.asarray(P, float).reshape(-1, 3)
        return P @ self.T_base_table[:3, :3].T + self.T_base_table[:3, 3]

    def base_to_table(self, P) -> np.ndarray:
        T = inv_T(self.T_base_table)
        return np.asarray(P, float).reshape(-1, 3) @ T[:3, :3].T + T[:3, 3]

    def project_table(self, P_table) -> np.ndarray:
        T = inv_T(self.T_table_cam)
        P = np.asarray(P_table, float).reshape(-1, 3)
        return self.cam.project(P @ T[:3, :3].T + T[:3, 3])

    def pixel_to_table(self, uv, z: float = 0.0) -> np.ndarray:
        """Pixels -> points on the table-frame plane at height z (table frame, NaN if no hit)."""
        T = self.T_table_cam
        d = self.cam.rays(np.asarray(uv, float).reshape(-1, 2)) @ T[:3, :3].T
        C = T[:3, 3]
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (z - C[2]) / d[:, 2]
        t = np.where(t > 0, t, np.nan)
        return C[None, :] + t[:, None] * d

    def centre_px(self) -> np.ndarray:
        return self.cam.K[:2, 2].copy()

    def ortho(self, img, z: float, center=None, half=None, res: float | None = None, mask=None):
        """Resample ``img`` onto the table-frame plane at height z: returns (ortho, valid, grid)
        where grid maps ortho px (u, v) -> table (x0 + u*res, y1 - v*res). Defaults: the part of the
        plane the camera sees, at about the camera's own resolution there."""
        cam = self.cam.for_image(img)
        h, w = img.shape[:2]
        C = self.T_table_cam[:3, 3]
        if center is None or half is None:
            u = np.linspace(0, w - 1, 9)
            v = np.linspace(0, h - 1, 7)
            border = np.array([[a, b] for a in u for b in v])
            P = View(cam, self.T_base_cam, self.T_base_table).pixel_to_table(border, z)
            P = P[np.isfinite(P).all(1)]
            P = P[np.hypot(P[:, 0] - C[0], P[:, 1] - C[1]) < 0.45]
            if not len(P):
                return None, None, None
            lo, hi = P[:, :2].min(0), P[:, :2].max(0)
            center = (lo + hi) / 2 if center is None else np.asarray(center, float)
            half = float(np.max(hi - lo) / 2) if half is None else half
        center = np.asarray(center, float)
        if res is None:
            dist = max(C[2] - z, 0.03)
            res = max(0.0004, min(dist / cam.K[0, 0], 2 * half / 900))
        n = int(np.ceil(2 * half / res))
        x0, y1 = center[0] - half, center[1] + half
        xs = x0 + (np.arange(n) + 0.5) * res
        ys = y1 - (np.arange(n) + 0.5) * res
        X, Y = np.meshgrid(xs, ys)
        P = np.c_[X.ravel(), Y.ravel(), np.full(X.size, z)]
        uv = View(cam, self.T_base_cam, self.T_base_table).project_table(P)
        mapx = uv[:, 0].reshape(n, n).astype(np.float32)
        mapy = uv[:, 1].reshape(n, n).astype(np.float32)
        bad = ~np.isfinite(mapx) | ~np.isfinite(mapy)
        mapx[bad], mapy[bad] = -1, -1
        out = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        m = cam.jaw_mask() if mask is None else mask
        valid = cv2.remap(m, mapx, mapy, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        valid[bad] = 0
        grid = {"x0": x0, "y1": y1, "res": res, "n": n, "z": z}
        return out, valid, grid


@dataclass
class Detection:
    kind: str                       # cup | tag | blob | claude
    xyz_base: list
    xyz_table: list
    yaw_table: float = 0.0          # rad, object's long axis in the table frame
    diameter: float | None = None   # m (cup)
    height: float | None = None     # m above the table (top / rim)
    sigma_xy: float = 0.005         # m, 1-sigma horizontal uncertainty
    pixel: list | None = None       # image px of the reference point
    score: float = 0.0
    id: int | None = None
    extra: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        d = asdict(self)
        for k in ("xyz_base", "xyz_table", "pixel"):
            if d[k] is not None:
                d[k] = [round(float(v), 5) for v in d[k]]
        d["yaw_table"] = round(float(self.yaw_table), 4)
        d["sigma_xy"] = round(float(self.sigma_xy), 5)
        return d


@dataclass
class Hint:
    near_table: tuple | None = None   # (x, y) table frame: only look around here
    radius: float = 0.05              # m


@runtime_checkable
class ObjectFinder(Protocol):
    def find(self, frame: np.ndarray, view: View, hint: Hint | None = None) -> Detection | None: ...


# --------------------------------------------------------------------------- circle fitting
def _fit_circle_ls(pts):
    """Algebraic (Kasa) then a few Gauss-Newton steps of the geometric fit -> (cx, cy, r, rms)."""
    x, y = pts[:, 0], pts[:, 1]
    A = np.c_[2 * x, 2 * y, np.ones(len(x))]
    b = x * x + y * y
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = sol[0], sol[1]
    r = np.sqrt(max(sol[2] + cx * cx + cy * cy, 1e-9))
    for _ in range(5):
        dx, dy = x - cx, y - cy
        d = np.hypot(dx, dy) + 1e-12
        res = d - r
        J = np.c_[-dx / d, -dy / d, -np.ones(len(x))]
        step, *_ = np.linalg.lstsq(J, -res, rcond=None)
        cx, cy, r = cx + step[0], cy + step[1], r + step[2]
    rms = float(np.sqrt(np.mean((np.hypot(x - cx, y - cy) - r) ** 2)))
    return cx, cy, r, rms


def _circle3(p):
    (x1, y1), (x2, y2), (x3, y3) = p
    d = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
    if abs(d) < 1e-9:
        return None
    s1, s2, s3 = x1 * x1 + y1 * y1, x2 * x2 + y2 * y2, x3 * x3 + y3 * y3
    cx = (s1 * (y2 - y3) + s2 * (y3 - y1) + s3 * (y1 - y2)) / d
    cy = (s1 * (x3 - x2) + s2 * (x1 - x3) + s3 * (x2 - x1)) / d
    return cx, cy, np.hypot(x1 - cx, y1 - cy)


def ransac_circle(pts, r_lo, r_hi, thr=1.2, iters=300, rng=None):
    rng = rng or np.random.default_rng(0)
    best, best_in = None, None
    if len(pts) < 8:
        return None
    for _ in range(iters):
        c = _circle3(pts[rng.choice(len(pts), 3, replace=False)])
        if c is None or not (r_lo <= c[2] <= r_hi):
            continue
        inl = np.abs(np.hypot(pts[:, 0] - c[0], pts[:, 1] - c[1]) - c[2]) < thr
        if best_in is None or inl.sum() > best_in.sum():
            best, best_in = c, inl
    if best is None or best_in.sum() < 8:
        return None
    cx, cy, r, rms = _fit_circle_ls(pts[best_in])
    inl = np.abs(np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) - r) < thr
    if inl.sum() >= 8:
        cx, cy, r, rms = _fit_circle_ls(pts[inl])
    return cx, cy, r, rms, inl


def angular_coverage(pts, cx, cy, bins=48) -> float:
    a = np.arctan2(pts[:, 1] - cy, pts[:, 0] - cx)
    h, _ = np.histogram(a, bins=bins, range=(-np.pi, np.pi))
    return float((h > 0).mean())


# --------------------------------------------------------------------------- cup
@dataclass
class CupFinder:
    """Open-top cup (cylinder) from above: rim circle on the rim plane."""
    diameter: float = 0.06          # m, outer rim diameter (workcell "cup")
    height: float = 0.08            # m, rim above the table
    diam_tol: float = 0.25          # accept circles within +-25% of the diameter
    height_sigma: float = 0.005     # m, how well the rim height is known
    min_coverage: float = 0.45      # fraction of the rim circle with edge support
    canny: tuple = (40, 110)
    min_density: float = 0.7        # edge points per px of the visible rim circumference
    max_ring_std: float = 22.0      # grey levels: the band next to the rim must be uniform
    calib_sigma: float = 0.001      # m, floor for calibration error

    @classmethod
    def from_workcell(cls, obj: str = "cup", path=None, **kw) -> "CupFinder":
        from interfaces.grasp.grasp import WORKCELL
        raw = json.loads(Path(path or WORKCELL).read_text())
        o = raw.get("objects", {}).get(obj, {})
        return cls(diameter=float(o.get("width", 0.06)), height=float(o.get("height", 0.08)), **kw)

    def edges(self, ortho, valid):
        """Canny edges on brightness and saturation, plus the gradient (of whichever channel is
        stronger at each pixel) so rim points can be required to have a RADIAL gradient."""
        g = cv2.GaussianBlur(cv2.cvtColor(ortho, cv2.COLOR_BGR2GRAY), (0, 0), 1.2)
        s = cv2.GaussianBlur(cv2.cvtColor(ortho, cv2.COLOR_BGR2HSV)[..., 1], (0, 0), 1.2)
        e = cv2.Canny(g, *self.canny) | cv2.Canny(s, *self.canny)
        e[cv2.erode(valid, np.ones((7, 7), np.uint8)) == 0] = 0
        gx1, gy1 = cv2.Sobel(g, cv2.CV_32F, 1, 0), cv2.Sobel(g, cv2.CV_32F, 0, 1)
        gx2, gy2 = cv2.Sobel(s, cv2.CV_32F, 1, 0), cv2.Sobel(s, cv2.CV_32F, 0, 1)
        use2 = np.hypot(gx2, gy2) > np.hypot(gx1, gy1)
        gx, gy = np.where(use2, gx2, gx1), np.where(use2, gy2, gy1)
        return e, g, gx, gy

    @staticmethod
    def _ring_uniformity(gray, valid, cx, cy, r, offsets=(-3.0, -2.0, 2.0, 3.0)) -> float:
        """Brightness spread just inside / just outside the fitted circle (the smaller one). A cup
        rim is a uniform band; a circle hallucinated on clutter crosses dark and light."""
        a = np.linspace(0, 2 * np.pi, 180, endpoint=False)
        best = 1e9
        h, w = gray.shape
        for dr in offsets:
            u = np.clip(np.round(cx + (r + dr) * np.cos(a)).astype(int), 0, w - 1)
            v = np.clip(np.round(cy + (r + dr) * np.sin(a)).astype(int), 0, h - 1)
            ok = valid[v, u] > 0
            if ok.sum() < 30:
                continue
            vals = np.sort(gray[v[ok], u[ok]].astype(float))
            k = len(vals) // 10
            best = min(best, float(np.std(vals[k:len(vals) - k])))
        return best

    @staticmethod
    def _radial(pts, gxy, cx, cy):
        rx, ry = pts[:, 0] - cx, pts[:, 1] - cy
        rn = np.hypot(rx, ry) + 1e-9
        gn = np.hypot(gxy[:, 0], gxy[:, 1]) + 1e-9
        return np.abs(rx * gxy[:, 0] + ry * gxy[:, 1]) / (rn * gn)

    def find(self, frame, view: View, hint: Hint | None = None) -> Detection | None:
        R = self.diameter / 2
        if hint is not None and hint.near_table is not None:
            center, half = hint.near_table, hint.radius + 1.6 * R
        else:
            center, half = None, None
        ortho, valid, grid = view.ortho(frame, self.height, center, half)
        if ortho is None:
            return None
        res = grid["res"]
        if ortho.shape[0] > 1100:   # very wide footprint: coarser grid
            ortho, valid, grid = view.ortho(frame, self.height, center, half, res=res * ortho.shape[0] / 1000)
            res = grid["res"]
        edges, gray, gx, gy = self.edges(ortho, valid)
        Rpx = R / res
        r_lo, r_hi = Rpx * (1 - self.diam_tol), Rpx * (1 + self.diam_tol)
        circles = cv2.HoughCircles(cv2.GaussianBlur(gray, (0, 0), 1.5), cv2.HOUGH_GRADIENT, dp=1.5, minDist=Rpx * 0.5,
                                   param1=110, param2=16, minRadius=int(r_lo), maxRadius=int(np.ceil(r_hi)))
        ys, xs = np.nonzero(edges)
        pts = np.c_[xs, ys].astype(np.float64)
        gxy = np.c_[gx[ys, xs], gy[ys, xs]]
        if circles is None or len(pts) < 10:
            return None
        best = None
        thr = max(1.0, 0.6e-3 / res)
        for cx0, cy0, r0 in circles[0][:20]:
            d = np.hypot(pts[:, 0] - cx0, pts[:, 1] - cy0)
            sel = (d > r_lo * 0.95) & (d < r_hi * 1.05)
            sel[sel] &= self._radial(pts[sel], gxy[sel], cx0, cy0) > 0.8   # rim edges are radial
            band, gband = pts[sel], gxy[sel]
            fit = ransac_circle(band, r_lo, r_hi, thr=thr)
            if fit is None:
                continue
            cx, cy, r, rms, inl = fit
            keep = inl & (self._radial(band, gband, cx, cy) > 0.9)
            if keep.sum() < 12:
                continue
            cx, cy, r, rms = _fit_circle_ls(band[keep])
            cov = angular_coverage(band[keep], cx, cy)
            density = keep.sum() / (2 * np.pi * r * max(cov, 1e-3))   # edge points per px of SEEN circumference
            ring_std = self._ring_uniformity(gray, valid, cx, cy, r)
            if density < self.min_density or ring_std > self.max_ring_std:
                continue   # sparse support or a ring that crosses black/white: clutter (e.g. the board)
            score = cov * min(density, 2.0)
            if best is None or score > best[0]:
                best = (score, cx, cy, r, rms, int(keep.sum()), cov)
        if best is None or best[6] < self.min_coverage:
            return None
        score, cx, cy, r, rms, n_in, cov = best
        x = grid["x0"] + (cx + 0.5) * res
        y = grid["y1"] - (cy + 0.5) * res
        p_t = np.array([x, y, self.height])
        p_b = view.table_to_base(p_t)[0]
        C = view.T_table_cam[:3, 3]
        oblique = np.hypot(C[0] - x, C[1] - y) / max(C[2] - self.height, 0.02)
        sigma = float(np.sqrt((rms * res / np.sqrt(max(n_in * cov, 1))) ** 2 * 4 + (self.height_sigma * oblique) ** 2
                              + self.calib_sigma ** 2))
        uv = view.project_table([p_t])[0]
        return Detection("cup", p_b.tolist(), p_t.tolist(), 0.0, float(2 * r * res), self.height, sigma,
                         uv.tolist() if np.isfinite(uv).all() else None, float(score),
                         extra={"coverage": round(cov, 3), "inliers": n_in, "fit_rms_mm": round(rms * res * 1000, 3),
                                "ortho_res_mm": round(res * 1000, 3), "oblique": round(float(oblique), 3)})


# --------------------------------------------------------------------------- tags
@dataclass
class TagFinder:
    ids: tuple | None = (2, 3, 4)
    size_m: float = 0.040
    sizes: dict = field(default_factory=dict)
    dict_name: str = "DICT_4X4_50"

    def __post_init__(self):
        p = cv2.aruco.DetectorParameters()
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.dict_name)), p)

    @classmethod
    def from_tags_json(cls, ids=None, **kw) -> "TagFinder":
        """Sizes from vision/targets/tags.json (40 mm page) unless given."""
        spec = BoardSpec.load()
        return cls(ids=tuple(ids) if ids is not None else spec.tag_ids, size_m=spec.tag_mm / 1000,
                   dict_name=spec.tag_dict, **kw)

    def detect_all(self, frame, view: View) -> list[Detection]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        corners, ids, _ = self.det.detectMarkers(gray)
        if ids is None:
            return []
        cam = view.cam.for_image(frame)
        out = []
        for c, i in zip(corners, ids.ravel()):
            i = int(i)
            if self.ids is not None and i not in self.ids:
                continue
            s = self.sizes.get(i, self.size_m)
            obj = np.array([[-s / 2, s / 2, 0], [s / 2, s / 2, 0], [s / 2, -s / 2, 0], [-s / 2, -s / 2, 0]], np.float64)
            img = c.reshape(4, 2).astype(np.float64)
            n = cam.normalize(img)
            ok, rv, tv = cv2.solvePnP(obj, n, np.eye(3), None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            T_cam_tag = make_T(cv2.Rodrigues(rv)[0], tv.ravel())
            T_bt = view.T_base_cam @ T_cam_tag
            T_tt = inv_T(view.T_base_table) @ T_bt
            dist = float(np.linalg.norm(tv))
            sigma = 0.001 + 0.004 * dist   # lateral; PnP depth is worse but the top height is known-ish
            out.append(Detection("tag", T_bt[:3, 3].tolist(), T_tt[:3, 3].tolist(),
                                 float(np.arctan2(T_tt[1, 0], T_tt[0, 0])), None, float(T_tt[2, 3]), sigma,
                                 img.mean(0).tolist(), 1.0, i, extra={"T_table_tag": T_tt.round(5).tolist(),
                                                                      "tilt_deg": round(float(np.degrees(np.arccos(np.clip(T_tt[2, 2], -1, 1)))), 2)}))
        return out

    def find(self, frame, view: View, hint: Hint | None = None) -> Detection | None:
        dets = self.detect_all(frame, view)
        if hint is not None and hint.near_table is not None:
            dets = [d for d in dets if np.hypot(d.xyz_table[0] - hint.near_table[0], d.xyz_table[1] - hint.near_table[1]) < hint.radius]
        return dets[0] if dets else None


# --------------------------------------------------------------------------- colour blob
@dataclass
class ColorBlobFinder:
    """Largest blob in an HSV range, located on the plane at ``plane_height`` (table frame).
    For a box use its top-face height; for a flat object 0."""
    hsv_lo: tuple = (40, 80, 40)
    hsv_hi: tuple = (85, 255, 255)
    plane_height: float = 0.0
    min_area_m2: float = 4e-4

    def find(self, frame, view: View, hint: Hint | None = None) -> Detection | None:
        center = half = None
        if hint is not None and hint.near_table is not None:
            center, half = hint.near_table, hint.radius + 0.05
        ortho, valid, grid = view.ortho(frame, self.plane_height, center, half)
        if ortho is None:
            return None
        hsv = cv2.cvtColor(ortho, cv2.COLOR_BGR2HSV)
        m = cv2.inRange(hsv, np.array(self.hsv_lo), np.array(self.hsv_hi))
        m[valid == 0] = 0
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        n, lab, stats, cents = cv2.connectedComponentsWithStats(m)
        if n <= 1:
            return None
        k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        res = grid["res"]
        area = stats[k, cv2.CC_STAT_AREA] * res * res
        if area < self.min_area_m2:
            return None
        ys, xs = np.nonzero(lab == k)
        rect = cv2.minAreaRect(np.c_[xs, ys].astype(np.float32))
        cx, cy = cents[k]
        p_t = np.array([grid["x0"] + (cx + 0.5) * res, grid["y1"] - (cy + 0.5) * res, self.plane_height])
        yaw = -np.radians(rect[2])
        uv = view.project_table([p_t])[0]
        return Detection("blob", view.table_to_base(p_t)[0].tolist(), p_t.tolist(), float(yaw), None,
                         self.plane_height, 0.003, uv.tolist() if np.isfinite(uv).all() else None, float(area),
                         extra={"area_m2": round(float(area), 6), "size_m": [round(float(v) * res, 4) for v in rect[1]]})


# --------------------------------------------------------------------------- Claude vision seam
@dataclass
class ClaudeVisionFinder:
    """Seam for a Claude-vision finder (not implemented: no API calls from this module). The
    implemented prompt -> pixel path is ``interfaces.wristcam.target.ClaudeLocator`` (tap-to-pick).

    Intended implementation: encode ``frame`` (JPEG, maybe with the image centre marked), ask a
    Claude vision model for the pixel (u, v) of the object described by ``prompt`` (and its rough
    size/top height if unknown), validate the answer is inside the image and outside the jaws,
    then ``view.pixel_to_table([[u, v]], z=height)`` gives the table point; ``sigma_xy`` from the
    model's stated pixel uncertainty times the ground resolution there. Return a Detection with
    kind="claude". The closed-loop engine then re-centres over it and the CupFinder / TagFinder
    can take over at close range. Inject a client callable ``ask(frame_jpeg, prompt) -> dict``.
    """
    prompt: str = "the paper cup"
    height: float = 0.0
    ask: object | None = None

    def find(self, frame, view: View, hint: Hint | None = None) -> Detection | None:
        if self.ask is None:
            raise NotImplementedError("ClaudeVisionFinder needs an `ask` callable (no API calls in this module)")
        ok, buf = cv2.imencode(".jpg", frame)
        ans = self.ask(buf.tobytes(), self.prompt)  # expected {"u": .., "v": .., "sigma_px": ..}
        if not ans or "u" not in ans:
            return None
        p_t = view.pixel_to_table([[ans["u"], ans["v"]]], self.height)[0]
        if not np.isfinite(p_t).all():
            return None
        dist = float(np.linalg.norm(view.T_table_cam[:3, 3] - p_t))
        sigma = float(ans.get("sigma_px", 10)) * dist / view.cam.K[0, 0]
        return Detection("claude", view.table_to_base(p_t)[0].tolist(), p_t.tolist(), 0.0, None, self.height, sigma,
                         [ans["u"], ans["v"]], 0.5)


# --------------------------------------------------------------------------- drawing
def annotate(frame, view: View, dets, text: str = "") -> np.ndarray:
    vis = frame.copy()
    c = tuple(int(v) for v in view.centre_px())
    cv2.drawMarker(vis, c, (255, 255, 0), cv2.MARKER_CROSS, 30, 2)
    for d in [x for x in (dets if isinstance(dets, (list, tuple)) else [dets]) if x is not None]:
        if d.kind == "cup" and d.diameter:
            a = np.linspace(0, 2 * np.pi, 72)
            ring = np.c_[d.xyz_table[0] + d.diameter / 2 * np.cos(a), d.xyz_table[1] + d.diameter / 2 * np.sin(a),
                         np.full(72, d.xyz_table[2])]
            uv = view.project_table(ring)
            uv = uv[np.isfinite(uv).all(1)]
            if len(uv):
                cv2.polylines(vis, [uv.astype(np.int32)], True, (0, 255, 0), 2, cv2.LINE_AA)
        if d.pixel is not None:
            p = tuple(int(v) for v in d.pixel)
            cv2.drawMarker(vis, p, (0, 255, 0), cv2.MARKER_TILTED_CROSS, 18, 2)
            label = f"{d.kind}{'' if d.id is None else ' ' + str(d.id)} table({d.xyz_table[0]:+.3f},{d.xyz_table[1]:+.3f}) " \
                    f"+-{d.sigma_xy * 1000:.1f}mm"
            cv2.putText(vis, label, (p[0] + 10, p[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(vis, label, (p[0] + 10, p[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
    if text:
        for k, line in enumerate(text.splitlines()):
            cv2.putText(vis, line, (10, 28 + 24 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(vis, line, (10, 28 + 24 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    return vis


def load_view_inputs(calib=DEFAULT_INTRINSICS, handeye=DEFAULT_HANDEYE):
    """(camera model, T_tip_cam, notes) from the calibration files, falling back to nominal."""
    notes = []
    try:
        cam = CameraModel.load(calib)
    except FileNotFoundError:
        cam = nominal_camera()
        notes.append("NO lens calibration: nominal fisheye guess (positions off by cm)")
    T = load_handeye(handeye)
    if T is None:
        T = T_TIP_CAM_NOMINAL
        notes.append("NO hand-eye calibration: camera assumed at gripper_tip looking along the fingers")
    return cam, T, notes


def make_finder(kind: str, **kw) -> ObjectFinder:
    if kind == "cup":
        return CupFinder.from_workcell(**kw)
    if kind == "tags":
        return TagFinder.from_tags_json(**kw)
    if kind == "blob":
        return ColorBlobFinder(**kw)
    if kind == "claude":
        return ClaudeVisionFinder(**kw)
    raise ValueError(f"unknown finder {kind!r} (cup, tags, blob, claude)")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--image", required=True)
    ap.add_argument("--q", required=True, help="joint angles when the frame was taken, comma separated (rad)")
    ap.add_argument("--finder", default="cup", choices=("cup", "tags", "blob"))
    ap.add_argument("--calib", default=str(DEFAULT_INTRINSICS))
    ap.add_argument("--handeye", default=str(DEFAULT_HANDEYE))
    ap.add_argument("--out", default=None, help="annotated image")
    a = ap.parse_args(argv)
    from interfaces.grasp.grasp import Workcell
    cam, T_tc, notes = load_view_inputs(a.calib, a.handeye)
    wc = Workcell.load()
    img = cv2.imread(a.image, cv2.IMREAD_COLOR)
    view = View.from_q([float(x) for x in a.q.split(",")], cam.for_image(img), T_tc, wc.T_base_table)
    det = make_finder(a.finder).find(img, view)
    for n in notes:
        print("note:", n)
    print(json.dumps(det.to_dict() if det else None, indent=1))
    if a.out:
        cv2.imwrite(a.out, annotate(img, view, det))
        print("wrote", a.out)


if __name__ == "__main__":
    main()
