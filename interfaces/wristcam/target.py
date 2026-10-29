"""interfaces.wristcam.target: "pick up THAT": a tap or a prompt -> a tracked target -> the closed loop.

A player looks at a still wrist-camera frame taken at a known q and either taps the object or
types what they want ("the cup on the table"). This module turns that into something the
closed-loop ``engine.GraspEngine`` can home in on and grasp:

    TargetRequest   tap pixel (u, v) or prompt text, on frame ``frame_id`` taken at ``q``
    ClaudeLocator   prompt + frame -> {found, point, box, label, confidence, reason, candidates}
                    (Claude vision, strict JSON, pixel coords rescaled to the full frame)
    resolve()       pixel -> object region (table-colour model + GrabCut seeded at the pixel) ->
                    footprint and height on the table plane (single-view prism fit, below) ->
                    ``Target``: table xy, height, footprint (rect or circle), grasp width/yaw, outline
    SeededFinder    the engine's ``ObjectFinder`` for "whatever was tapped": re-finds the target
                    near its predicted position in every look (NCC on the top face resampled
                    onto the plane at the object's height, then re-segmentation; cups go through
                    the rim-circle ``CupFinder`` seeded there), so the same look -> centre ->
                    closer -> descend -> close -> verify -> retry loop works for anything
    pick_config()   GraspConfig for a target (width, height, finger yaw across the narrow side)

Footprint and height from ONE view. Every pixel of the object's silhouette is a ray; their hits
on the table plane z=0 form the "shadow" S0 seen from the camera centre C (nadir N, height H).
A vertical prism with footprint F and height h casts the convex hull of F and of its top face
pushed out from the nadir, N + (F - N) H/(H-h). ``fit_prism`` fits F (a rectangle or a circle)
and h to S0's hull (symmetric-difference area). Off-nadir objects (the side walls show) give a
clear optimum; straight below the camera the height is unobservable (flat cost curve) and the
prior height is used, marked ``height_known: false``. (Silhouette-only "largest consistent
footprint" carving does NOT work: along every ray from N any h up to the true one re-casts S0
exactly; the shape prior, top face = footprint, is what fixes h.) The engine then centres the
camera over the object, where the xy no longer depends on the height.

Grasp: top-down, closing across the footprint's NARROW side (min-area rect). Refused, with a
message for the player, if the narrow side + the approach margin is wider than the gripper opens
(``Workcell.max_width``, 94 mm from the URDF; the real i2rt crank_4310 stroke may be only ~71 mm:
set ``PickConfig.max_open``), flatter than ``min_height`` or taller than ``max_height``.

No API calls happen unless a ``ClaudeLocator`` is used without an injected client; tests always
inject a fake client.
"""
from __future__ import annotations

import base64
import json
import os
import re
import time
import uuid
from dataclasses import asdict, dataclass, field
from pathlib import Path

import cv2
import numpy as np

from interfaces.grasp.grasp import Workcell

from .camera import CameraModel
from .locate import CupFinder, Detection, Hint, View

HERE = Path(__file__).resolve().parent
PICK_CONFIG = HERE / "pick_config.json"     # optional overrides of PickConfig (not required)
DEFAULT_MODEL = "claude-opus-5-5"


class TargetRefused(ValueError):
    """The request can't become a target; ``str(e)`` is shown to the player."""

    def __init__(self, message: str, code: str = "refused"):
        super().__init__(message)
        self.code = code


@dataclass
class PickConfig:
    max_object: float = 0.16        # m: biggest object we look for around a tap
    min_area: float = 3e-4          # m^2 footprint (about 1.7 x 1.7 cm)
    min_height: float = 0.015       # m: flatter than this can't be grabbed from above
    max_height: float = 0.16        # m
    default_height: float = 0.06    # m: when the height can't be seen (object right under the camera)
    reach: tuple = (0.12, 0.46)     # m from the base axis (horizontal) where the arm can pick
    table_extent: tuple | None = None   # table-frame (x0, x1, y0, y1) of the usable surface, if known
    max_open: float | None = None   # m the fingers open (None: workcell max_width)
    look_xy: tuple = (0.0, -0.15)   # table frame: where the look pose points the camera
    look_h: float = 0.25            # m camera above the table at the look pose (lowered if unreachable)
    search_radius: float = 0.04     # m: SeededFinder search radius without a hint
    hull_res: float = 0.001         # m per px of the footprint grid
    model: str = ""                 # Claude model id ("" -> $WRISTCAM_PICK_MODEL or DEFAULT_MODEL)

    @classmethod
    def load(cls, path: Path | str | None = None) -> "PickConfig":
        p = Path(path) if path else PICK_CONFIG
        if p.exists():
            raw = json.loads(p.read_text())
            raw = {k: tuple(v) if isinstance(v, list) else v for k, v in raw.items() if k in cls.__dataclass_fields__}
            return cls(**raw)
        return cls()


# --------------------------------------------------------------------------- requests and frames
@dataclass
class LookFrame:
    """A still frame and the joint angles it was taken at (the q is what makes a tap a ray)."""
    image: np.ndarray
    q: list
    id: str = field(default_factory=lambda: uuid.uuid4().hex[:10])
    t: float = field(default_factory=time.time)

    def view(self, cam: CameraModel, T_tip_cam, T_base_table) -> View:
        v = View.from_q(self.q, cam, T_tip_cam, T_base_table)
        v.cam = cam.for_image(self.image)
        return v

    @property
    def size(self) -> tuple[int, int]:
        return int(self.image.shape[1]), int(self.image.shape[0])


@dataclass
class TargetRequest:
    frame_id: str
    q: list
    kind: str                         # tap | prompt
    pixel: tuple | None = None        # full-frame px (u, v)
    prompt: str | None = None
    box: tuple | None = None          # full-frame px (u0, v0, u1, v1), e.g. from Claude
    label: str = ""

    def to_dict(self) -> dict:
        d = asdict(self)
        d["q"] = [round(float(x), 5) for x in self.q]
        for k in ("pixel", "box"):
            if d[k] is not None:
                d[k] = [round(float(x), 1) for x in d[k]]
        return d


@dataclass
class Target:
    xy_table: tuple                   # footprint centre, table frame (m)
    height: float                     # m, top above the table
    size: tuple                       # (long, short) sides of the footprint's min-area rect (m)
    yaw_table: float                  # rad, long axis in the table frame
    pixel: tuple                      # top centre in the request frame (px)
    outline_px: list                  # silhouette outline in the request frame (px), for the phone
    footprint_table: list             # footprint rect corners, table frame (m)
    height_known: bool = True
    sigma_xy: float = 0.003
    label: str = "object"
    kind: str = "object"              # object | cup (round, open top: the rim finder helps)
    refusal: str | None = None        # why it can't be picked (too wide, ...): shown, confirm refused
    request: dict | None = None
    template: dict | None = None      # NCC template of the top face (not serialised)
    fit: dict | None = None           # prism fit diagnostics (model, cost, height curve, segmentation)
    id: str = field(default_factory=lambda: uuid.uuid4().hex[:8])

    @property
    def width(self) -> float:          # across the fingers
        return float(self.size[1])

    @property
    def grasp_yaw_table(self) -> float:
        """Closing direction (table frame): across the narrow side = along the short axis."""
        return float(self.yaw_table + np.pi / 2)

    def to_dict(self, frame_size: tuple | None = None) -> dict:
        d = {"id": self.id, "xy_table": [round(float(v), 4) for v in self.xy_table], "height": round(float(self.height), 4),
             "size": [round(float(v), 4) for v in self.size], "yaw_table": round(float(self.yaw_table), 4),
             "pixel": [round(float(v), 1) for v in self.pixel], "height_known": self.height_known,
             "sigma_xy": round(float(self.sigma_xy), 4), "label": self.label, "kind": self.kind,
             "refusal": self.refusal, "width_mm": round(self.width * 1000), "height_mm": round(self.height * 1000),
             "outline_px": [[round(float(u), 1), round(float(v), 1)] for u, v in self.outline_px],
             "footprint_table": [[round(float(x), 4), round(float(y), 4)] for x, y in self.footprint_table],
             "request": self.request}
        if frame_size:   # normalised 0..1 for the phone (any display size)
            w, h = frame_size
            d["outline"] = [[round(u / w, 4), round(v / h, 4)] for u, v in self.outline_px]
            d["point"] = [round(self.pixel[0] / w, 4), round(self.pixel[1] / h, 4)]
        return d


# --------------------------------------------------------------------------- segmentation
def _roi(uv, half, w, h):
    u0, v0 = int(max(0, uv[0] - half)), int(max(0, uv[1] - half))
    u1, v1 = int(min(w, uv[0] + half + 1)), int(min(h, uv[1] + half + 1))
    return u0, v0, u1, v1


def segment(frame, view: View, uv, box=None, cfg: PickConfig | None = None) -> tuple[np.ndarray, dict]:
    """Object region around pixel ``uv``: pixels unlike the surrounding table (robust Lab colour
    model from the region's border), the connected part at the tap, refined by GrabCut. Returns
    (full-frame uint8 mask, info). Raises TargetRefused when there is nothing object-like there."""
    cfg = cfg or PickConfig()
    h, w = frame.shape[:2]
    if not (np.isfinite(uv).all() and 0 <= uv[0] < w and 0 <= uv[1] < h):
        raise TargetRefused("that's outside the picture", "outside")
    cam = view.cam.for_image(frame)
    jaw = cam.jaw_mask()
    p0 = view.pixel_to_table([uv], 0.0)[0]
    C = view.T_table_cam[:3, 3]
    dist = float(np.linalg.norm(C - p0)) if np.isfinite(p0).all() else 0.4
    ppm = cam.K[0, 0] / max(dist, 0.05)              # px per metre near the tap
    if box is not None:
        bu0, bv0, bu1, bv1 = box
        half = 0.5 * max(bu1 - bu0, bv1 - bv0) * 1.5 + 12
        centre = ((bu0 + bu1) / 2, (bv0 + bv1) / 2)
    else:
        half, centre = float(np.clip(0.6 * cfg.max_object * ppm, 60, 420)), uv
    u0, v0, u1, v1 = _roi(centre, half, w, h)
    sub = frame[v0:v1, u0:u1]
    sj = jaw[v0:v1, u0:u1]
    lab = cv2.cvtColor(sub, cv2.COLOR_BGR2LAB).astype(np.float32)
    H_, W_ = sub.shape[:2]
    b = int(max(6, 0.1 * min(H_, W_)))
    band = np.zeros((H_, W_), bool)
    band[:b], band[-b:], band[:, :b], band[:, -b:] = True, True, True, True
    band &= sj > 0
    if band.sum() < 50:
        raise TargetRefused("I can't see enough table around that spot", "no_context")
    bg = lab[band]
    med = np.median(bg, axis=0)
    mad = 1.4826 * np.median(np.abs(bg - med), axis=0) + np.array([5.0, 2.5, 2.5])
    z = np.abs(lab - med) / mad
    score = np.maximum(z[..., 0] / 1.8, np.maximum(z[..., 1], z[..., 2]))
    non = ((score > 3.0) & (sj > 0)).astype(np.uint8)
    non = cv2.morphologyEx(non, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    non = cv2.morphologyEx(non, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n, lab_cc, stats, _ = cv2.connectedComponentsWithStats(non)
    su, sv = int(round(uv[0] - u0)), int(round(uv[1] - v0))
    r_seed = int(max(4, 0.012 * ppm))
    ys, xs = np.mgrid[max(0, sv - r_seed):min(H_, sv + r_seed + 1), max(0, su - r_seed):min(W_, su + r_seed + 1)]
    near = lab_cc[ys, xs]
    d2 = (ys - sv) ** 2 + (xs - su) ** 2
    cand = near > 0
    if not cand.any():
        raise TargetRefused("I don't see an object there: that looks like bare table", "bare_table")
    k = int(near[cand][np.argmin(d2[cand])])
    seed = (int(xs[cand][np.argmin(d2[cand])]), int(ys[cand][np.argmin(d2[cand])]))
    cc = (lab_cc == k).astype(np.uint8)
    area_px = int(stats[k, cv2.CC_STAT_AREA])
    if area_px < 30:
        raise TargetRefused("that's too small to pick up (or just a mark on the table)", "too_small")
    # GrabCut, started from the colour model: the object's side walls (darker) stay in
    gm = np.full((H_, W_), cv2.GC_PR_BGD, np.uint8)
    gm[cc > 0] = cv2.GC_PR_FGD
    cv2.circle(gm, seed, 2, cv2.GC_FGD, -1)
    gm[sj == 0] = cv2.GC_BGD
    gm[:3], gm[-3:], gm[:, :3], gm[:, -3:] = cv2.GC_BGD, cv2.GC_BGD, cv2.GC_BGD, cv2.GC_BGD
    try:
        bgd, fgd = np.zeros((1, 65), np.float64), np.zeros((1, 65), np.float64)
        cv2.setRNGSeed(0)   # GrabCut seeds its GMMs from OpenCV's global RNG: same frame, same mask
        cv2.grabCut(sub, gm, None, bgd, fgd, 4, cv2.GC_INIT_WITH_MASK)
        fg = ((gm == cv2.GC_FGD) | (gm == cv2.GC_PR_FGD)).astype(np.uint8)
    except cv2.error:
        fg = cc.copy()
    fg &= cv2.dilate(cc, np.ones((9, 9), np.uint8))   # no leaking into the table
    fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    n2, lab2, stats2, _ = cv2.connectedComponentsWithStats(fg)
    k2 = int(lab2[seed[1], seed[0]])
    if k2 == 0:
        ov = [(int(np.sum((lab2 == j) & (cc > 0))), j) for j in range(1, n2)]
        k2 = max(ov)[1] if ov and max(ov)[0] > 0 else 0
    region = (lab2 == k2).astype(np.uint8) if k2 else cc
    # holes (a cup's dark inside) belong to the object
    cnts, _ = cv2.findContours(region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    region = np.zeros_like(region)
    cv2.drawContours(region, [max(cnts, key=cv2.contourArea)], -1, 1, -1)
    edge = np.r_[region[0], region[-1], region[:, 0], region[:, -1]]
    full = np.zeros((h, w), np.uint8)
    full[v0:v1, u0:u1] = region * 255
    info = {"roi": [u0, v0, u1, v1], "area_px": int(region.sum()), "ppm": round(ppm, 1),
            "touches_roi": bool(edge.mean() > 0.01),
            "touches_image": bool(full[0].any() or full[-1].any() or full[:, 0].any() or full[:, -1].any()),
            "touches_jaw": bool((cv2.dilate(full, np.ones((5, 5), np.uint8)) & (255 - jaw)).any())}
    if info["touches_roi"] and box is None:
        raise TargetRefused("I can't tell where that object ends (too big, or touching something else)", "no_edges")
    return full, info


# --------------------------------------------------------------------------- footprint (single-view prism fit)
def _hull(P) -> np.ndarray:
    return cv2.convexHull(np.asarray(P, np.float32).reshape(-1, 1, 2)).reshape(-1, 2)


def _area(P) -> float:
    return float(cv2.contourArea(np.asarray(P, np.float32))) if len(P) >= 3 else 0.0


def _inter(A, B) -> tuple[float, np.ndarray | None]:
    a, poly = cv2.intersectConvexConvex(np.asarray(A, np.float32), np.asarray(B, np.float32))
    return float(max(a, 0.0)), (None if poly is None else poly.reshape(-1, 2))


def _base_pts(model: str, p) -> np.ndarray:
    if model == "circle":
        cx, cy, r = p
        a = np.linspace(0, 2 * np.pi, 40, endpoint=False)
        return np.c_[cx + abs(r) * np.cos(a), cy + abs(r) * np.sin(a)]
    cx, cy, L, W, yaw = p
    c, s = np.cos(yaw), np.sin(yaw)
    u, v = np.array([c, s]) * abs(L) / 2, np.array([-s, c]) * abs(W) / 2
    return np.array([[cx, cy] + u + v, [cx, cy] - u + v, [cx, cy] - u - v, [cx, cy] + u - v])


def prism_silhouette(model: str, p, h: float, N, H: float) -> np.ndarray:
    """Convex silhouette on the table plane of a vertical prism (footprint ``model``/``p``, height h)
    seen from a camera centre at (N, H): hull of the footprint and the top face cast onto z=0."""
    B = _base_pts(model, p)
    k = (H - h) / H
    top = np.asarray(N)[None, :] + (B - np.asarray(N)[None, :]) / k
    return _hull(np.r_[B, top])


def fit_prism(S0, N, H: float, heights, models=("rect", "circle")) -> dict:
    """Best prism (model, footprint params, height) whose silhouette matches the convex polygon
    ``S0`` (table plane, m). Cost: symmetric-difference area / area(S0). Returns {model, p, height,
    cost, curve: [(h, cost)], observable, near: (h_lo, h_hi)}."""
    from scipy.optimize import minimize, minimize_scalar

    # work in mm: cv2's convex clipping is happier with numbers ~1..1000 than ~1e-3
    S0 = _hull(np.asarray(S0, float) * 1000.0)
    aS = _area(S0)
    N = np.asarray(N, float) * 1000.0
    H = float(H) * 1000.0
    heights = [float(h) * 1000.0 for h in heights]

    def cost(model, p, h):
        if (model == "circle" and p[2] <= 1.0) or (model == "rect" and min(p[2], p[3]) <= 1.0):
            return 10.0
        Sm = prism_silhouette(model, p, h, N, H)
        aM = _area(Sm)
        i, _ = _inter(S0, Sm)
        i = min(i, aS, aM)        # cv2 can misreport on near-degenerate polygons
        return max(0.0, (aS + aM - 2 * i) / aS)

    def init(model, h):
        k = (H - h) / H
        shrunk = N[None, :] + (S0 - N[None, :]) * k
        _, F = _inter(S0, shrunk)
        F = S0 if F is None or len(F) < 3 else F
        if model == "circle":
            M = cv2.moments(F.astype(np.float32))
            cx, cy = (M["m10"] / M["m00"], M["m01"] / M["m00"]) if M["m00"] > 0 else F.mean(0)
            return np.array([cx, cy, np.sqrt(_area(F) / np.pi)])
        (cx, cy), (a, b), ang = cv2.minAreaRect(F.astype(np.float32))
        return np.array([cx, cy, a, b, np.radians(ang)])

    def solve(model, p0, h):
        steps = np.array([4.0, 4.0, 4.0] if model == "circle" else [4.0, 4.0, 6.0, 6.0, 0.15])
        p0 = np.asarray(p0, float)
        sim = np.vstack([p0] + [p0 + np.eye(len(p0))[i] * steps[i] for i in range(len(p0))])
        r = minimize(lambda x: cost(model, x, h), p0, method="Nelder-Mead",
                     options={"initial_simplex": sim, "xatol": 0.02, "fatol": 1e-5, "maxiter": 500})
        return r.x, float(r.fun)

    best, curve = None, {m: [] for m in models}
    for m in models:
        for hh in heights:
            if (H - hh) / H <= 0.25:
                continue
            p, c = solve(m, init(m, hh), hh)
            curve[m].append((float(hh), c, p))
            if best is None or c < best["cost"]:
                best = {"model": m, "p": p, "height": float(hh), "cost": c}
    if best is None:
        raise TargetRefused("the camera is too low to judge that object", "too_close")
    m = best["model"]
    hs = [h for h, _, _ in curve[m]]
    if len(hs) > 2:   # refine the height between the grid points
        j = hs.index(best["height"])
        lo, hi = hs[max(0, j - 1)], hs[min(len(hs) - 1, j + 1)]
        cache = {}
        p_best = best["p"]

        def f(hh):
            p, c = solve(m, p_best, hh)
            cache[hh] = p
            return c
        r = minimize_scalar(f, bounds=(lo, hi), method="bounded", options={"xatol": 0.5})
        if r.fun < best["cost"]:
            best.update(height=float(r.x), cost=float(r.fun), p=cache[r.x])
    cmin = min(c for _, c, _ in curve[m])
    tol = max(0.004, 0.3 * cmin)
    near = [h for h, c, _ in curve[m] if c <= cmin + tol]
    observable = len(hs) == 1 or (max(near) - min(near)) <= 30.0
    p = np.array(best["p"], float)
    p[:2] /= 1000.0
    if m == "circle":
        p[2] /= 1000.0
    else:
        p[2:4] /= 1000.0
    best.update(p=p, height=best["height"] / 1000.0, observable=bool(observable),
                curve=[(round(h / 1000.0, 4), round(c, 4)) for h, c, _ in curve[m]],
                near=(min(near) / 1000.0, max(near) / 1000.0),
                costs={mm: round(min(c for _, c, _ in curve[mm]), 4) for mm in models if curve[mm]})
    return best


def contour_table(mask, view: View) -> tuple[np.ndarray, np.ndarray]:
    """The mask's outer contour (px) and its rays' hits on the table plane (m)."""
    cnts, _ = cv2.findContours((mask > 0).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    c = max(cnts, key=cv2.contourArea).reshape(-1, 2).astype(np.float64)
    P = view.pixel_to_table(c, 0.0)
    if not np.isfinite(P).all():
        raise TargetRefused("that's not on the table (it's above the horizon)", "off_table")
    if np.ptp(P[:, 0]) > 0.5 or np.ptp(P[:, 1]) > 0.5:
        raise TargetRefused("that's not on the table (too far away)", "off_table")
    return c, P[:, :2]


def footprint(mask, view: View, heights, models=("rect", "circle")) -> dict:
    """Footprint + height of the prism whose silhouette is ``mask`` (see the module doc).
    ``heights``: candidate heights (one value = the height is known). Adds xy, size (long,
    short), yaw (long axis) and corners (table frame) to ``fit_prism``'s result."""
    _, P = contour_table(mask, view)
    C = view.T_table_cam[:3, 3]
    fit = fit_prism(P, C[:2], float(C[2]), list(heights), models)
    B = _base_pts(fit["model"], fit["p"])
    if fit["model"] == "circle":
        r = abs(fit["p"][2])
        fit.update(xy=np.array(fit["p"][:2], float), size=(2 * r, 2 * r), yaw=0.0, corners=B[::5])
    else:
        cx, cy, L, W, yaw = fit["p"]
        L, W = abs(L), abs(W)
        if W > L:
            L, W, yaw = W, L, yaw + np.pi / 2
        yaw = (yaw + np.pi / 2) % np.pi - np.pi / 2
        fit.update(xy=np.array([cx, cy], float), size=(float(L), float(W)), yaw=float(yaw), corners=B)
    return fit

def _template(frame, view: View, xy, height, size, res) -> dict:
    half = 0.5 * size[0] + 0.012
    full = np.full(frame.shape[:2], 255, np.uint8)
    o, valid, grid = view.ortho(frame, height, xy, half, res=res, mask=full)
    g = cv2.cvtColor(o, cv2.COLOR_BGR2GRAY).astype(np.float32)
    lab = cv2.cvtColor(o, cv2.COLOR_BGR2LAB)
    return {"gray": g, "valid": valid, "res": res, "half": half, "lab_mean": lab.reshape(-1, 3).mean(0).tolist()}


# --------------------------------------------------------------------------- resolve
def check_pixel(frame, view: View, uv, wc: Workcell, cfg: PickConfig) -> np.ndarray:
    """Refuse taps outside the image, on the jaws, off the table or out of reach. Returns the
    tap's ray hit on the table plane (table frame)."""
    h, w = frame.shape[:2]
    u, v = float(uv[0]), float(uv[1])
    if not (0 <= u < w and 0 <= v < h):
        raise TargetRefused("that's outside the picture", "outside")
    jaw = view.cam.for_image(frame).jaw_mask()
    if jaw[int(v), int(u)] == 0:
        raise TargetRefused("that's the gripper's own jaw: tap the object on the table", "jaw")
    p = view.pixel_to_table([[u, v]], 0.0)[0]
    C = view.T_table_cam[:3, 3]
    if not np.isfinite(p).all() or np.linalg.norm(p - C) > 0.9:
        raise TargetRefused("that's not on the table", "off_table")
    if cfg.table_extent is not None:
        x0, x1, y0, y1 = cfg.table_extent
        if not (x0 <= p[0] <= x1 and y0 <= p[1] <= y1):
            raise TargetRefused("that's off the table", "off_table")
    b = wc.to_base(p)
    r = float(np.hypot(b[0], b[1]))
    if r < cfg.reach[0]:
        raise TargetRefused("that's the arm itself (or too close to its base)", "arm")
    if r > cfg.reach[1] + 0.05:
        raise TargetRefused(f"that's out of the arm's reach ({r * 100:.0f} cm from its base)", "reach")
    return p


def resolve(frame, view: View, req: TargetRequest, wc: Workcell, cfg: PickConfig | None = None,
            max_open: float | None = None) -> Target:
    """A tap (or a Claude point/box) -> ``Target``. Raises TargetRefused for taps that aren't
    an object on the table; returns a Target with ``refusal`` set for objects the gripper can't
    take (too wide / flat / tall), so the player still sees what was found."""
    cfg = cfg or PickConfig()
    if req.pixel is None:
        raise TargetRefused("no point to look at", "no_point")
    uv = np.array(req.pixel, float)
    check_pixel(frame, view, uv, wc, cfg)
    mask, info = segment(frame, view, uv, req.box, cfg)
    return target_from_mask(frame, view, mask, wc, cfg, req=req, info=info, max_open=max_open)


def target_from_mask(frame, view: View, mask, wc: Workcell, cfg: PickConfig | None = None, req: TargetRequest | None = None,
                     info: dict | None = None, label: str = "", max_open: float | None = None) -> Target:
    """An object's image mask (from any segmenter: the tap GrabCut above, YOLO-E, Claude + GrabCut)
    -> ``Target``: footprint and height on the table plane (prism fit), grasp width and yaw, refusal.
    Raises TargetRefused when the mask is cut off by the picture's edge, too small or out of reach."""
    cfg = cfg or PickConfig()
    mask = (np.asarray(mask) > 0).astype(np.uint8) * 255
    if info is None:
        h, w = mask.shape[:2]
        ys, xs = np.nonzero(mask)
        if not len(xs):
            raise TargetRefused("nothing there to pick up", "empty")
        edge = xs.min() <= 1 or ys.min() <= 1 or xs.max() >= w - 2 or ys.max() >= h - 2
        info = {"touches_image": bool(edge), "touches_jaw": False, "source": "mask"}
    if info["touches_image"]:
        raise TargetRefused("that object is cut off at the edge of the picture: pick one nearer the middle", "edge")
    if req is None:
        ys, xs = np.nonzero(mask)
        req = TargetRequest("mask", list(view.q or []), "prompt", (float(xs.mean()), float(ys.mean())), label=label)
    hs = np.arange(0.0, cfg.max_height + 1e-9, 0.01)
    fp = footprint(mask, view, hs)
    height, known = fp["height"], fp["observable"]
    if not known:   # straight under the camera: the prior, if the data allows it
        lo, hi = fp["near"]
        height = float(np.clip(cfg.default_height, lo, hi))
        fp = footprint(mask, view, [height])
    xy, size, yaw, corners = fp["xy"], fp["size"], fp["yaw"], fp["corners"]
    if size[0] * size[1] < cfg.min_area:
        raise TargetRefused("that's too small to pick up", "too_small")
    b = wc.to_base([xy[0], xy[1], 0.0])
    r = float(np.hypot(b[0], b[1]))
    if not (cfg.reach[0] <= r <= cfg.reach[1]):
        raise TargetRefused(f"that's out of the arm's reach ({r * 100:.0f} cm from its base)", "reach")
    top_px = view.project_table([[xy[0], xy[1], height]])[0]
    cnts, _ = cv2.findContours((mask > 0).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    outline = cv2.approxPolyDP(max(cnts, key=cv2.contourArea), 1.5, True).reshape(-1, 2).tolist()
    kind = "cup" if fp["model"] == "circle" and height > 0.03 else "object"
    label = label or req.label or ("cup" if kind == "cup" else "object")
    sigma = float(np.hypot(0.002 + 0.01 * fp["cost"], 0.0 if known else 0.004))
    t = Target(tuple(float(v) for v in xy), float(height), tuple(size), float(yaw), tuple(float(v) for v in top_px),
               outline, np.asarray(corners).tolist(), bool(known), sigma, label, kind, None, req.to_dict())
    t.template = _template(frame, view, xy, height, size, cfg.hull_res)
    t.refusal = grasp_refusal(t, wc, cfg, max_open)
    t.fit = {"model": fp["model"], "cost": round(fp["cost"], 4), "costs": fp["costs"], "curve": fp["curve"],
             "observable": fp["observable"], "seg": info}
    return t


def grasp_refusal(t: Target, wc: Workcell, cfg: PickConfig, max_open: float | None = None) -> str | None:
    opening = max_open or cfg.max_open or wc.max_width
    if t.width + wc.open_margin > opening + 1e-9:
        return (f"too wide to grab: {t.width * 100:.1f} cm across at its narrowest, and the gripper opens to "
                f"{opening * 100:.1f} cm (it needs {wc.open_margin * 100:.0f} cm to spare)")
    if t.height < cfg.min_height:
        return f"too flat to grab from above ({t.height * 100:.1f} cm tall)"
    if t.height > cfg.max_height:
        return f"too tall for a top-down grab ({t.height * 100:.0f} cm)"
    return None


# --------------------------------------------------------------------------- Claude
PROMPT = """You are the eyes of a small robot arm. The image is from a camera on its wrist, looking down at a table.
The player asked: "{prompt}"

Find the one physical object they mean. Answer with ONLY a JSON object, no prose, no code fence:
{{"found": true|false, "point": [u, v], "box": [u0, v0, u1, v1], "label": "short name", "confidence": 0..1,
  "reason": "one short sentence", "candidates": [{{"point": [u, v], "box": [u0, v0, u1, v1], "label": "..."}}]}}
Pixel coordinates are in THIS image ({w} x {h} px, origin top-left, u right, v down). "point" is the centre of
the object's top surface, "box" its bounding box. If nothing matches, found=false and say why in "reason".
If several objects could match, found=false and list them in "candidates" (most likely first) so the player
can tap one. The dark shapes in the bottom corners are the gripper's own jaws, never an answer."""


class LocatorUnavailable(RuntimeError):
    pass


@dataclass
class LocateAnswer:
    found: bool
    point: tuple | None = None        # full-frame px
    box: tuple | None = None          # full-frame px
    label: str = ""
    confidence: float = 0.0
    reason: str = ""
    candidates: list = field(default_factory=list)   # [{point, box, label}] full-frame px
    raw: str = ""

    def to_dict(self) -> dict:
        d = asdict(self)
        d.pop("raw")
        return d


def _parse_json(text: str) -> dict:
    text = text.strip()
    m = re.search(r"\{.*\}", text, re.S)
    if not m:
        raise ValueError("no JSON object in the answer")
    return json.loads(m.group(0))


class ClaudeLocator:
    """Prompt + frame -> ``LocateAnswer`` through the Anthropic API (vision). The frame is sent
    downscaled (longest side ``max_side`` px, JPEG); coordinates come back in that image and are
    rescaled. Inject ``client`` (anything with ``messages.create``) for tests: no API calls then.
    Model: ``model`` or $WRISTCAM_PICK_MODEL or claude-opus-5-5; key: $ANTHROPIC_API_KEY."""

    def __init__(self, client=None, model: str | None = None, max_side: int = 768, timeout: float = 45.0,
                 min_confidence: float = 0.5):
        self._client, self.max_side, self.timeout = client, max_side, timeout
        self.model = model or os.environ.get("WRISTCAM_PICK_MODEL") or DEFAULT_MODEL
        self.min_confidence = min_confidence

    @property
    def client(self):
        if self._client is None:
            if not os.environ.get("ANTHROPIC_API_KEY"):
                raise LocatorUnavailable("prompts need Claude, and ANTHROPIC_API_KEY isn't set: tap the object instead")
            import anthropic
            self._client = anthropic.Anthropic(timeout=self.timeout)
        return self._client

    def encode(self, frame) -> tuple[bytes, float, tuple]:
        h, w = frame.shape[:2]
        s = min(1.0, self.max_side / max(w, h))
        small = cv2.resize(frame, (int(round(w * s)), int(round(h * s))), interpolation=cv2.INTER_AREA) if s < 1 else frame
        ok, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return buf.tobytes(), s, (small.shape[1], small.shape[0])

    def locate(self, frame, prompt: str) -> LocateAnswer:
        jpg, s, (sw, sh) = self.encode(frame)
        msg = self.client.messages.create(
            model=self.model, max_tokens=600,
            messages=[{"role": "user", "content": [
                {"type": "image", "source": {"type": "base64", "media_type": "image/jpeg",
                                             "data": base64.b64encode(jpg).decode("ascii")}},
                {"type": "text", "text": PROMPT.format(prompt=prompt.strip()[:200], w=sw, h=sh)}]}])
        text = "".join(getattr(b, "text", "") for b in getattr(msg, "content", []) or [])
        return self.parse(text, s, (sw, sh))

    def parse(self, text: str, s: float, small_size: tuple) -> LocateAnswer:
        try:
            d = _parse_json(text)
        except (ValueError, json.JSONDecodeError) as e:
            return LocateAnswer(False, reason=f"couldn't read Claude's answer ({e})", raw=text)
        sw, sh = small_size

        def pt(p):
            try:
                u, v = float(p[0]), float(p[1])
            except (TypeError, ValueError, IndexError):
                return None
            if not (0 <= u <= sw and 0 <= v <= sh):
                return None
            return (u / s, v / s)

        def bx(b):
            try:
                u0, v0, u1, v1 = (float(x) for x in b)
            except (TypeError, ValueError):
                return None
            u0, u1 = sorted((min(max(u0, 0), sw), min(max(u1, 0), sw)))
            v0, v1 = sorted((min(max(v0, 0), sh), min(max(v1, 0), sh)))
            if u1 - u0 < 2 or v1 - v0 < 2:
                return None
            return (u0 / s, v0 / s, u1 / s, v1 / s)

        cands = []
        for c in (d.get("candidates") or [])[:6]:
            if isinstance(c, dict):
                p, b = pt(c.get("point")), bx(c.get("box")) if c.get("box") is not None else None
                if p is None and b is not None:
                    p = ((b[0] + b[2]) / 2, (b[1] + b[3]) / 2)
                if p is not None:
                    cands.append({"point": p, "box": b, "label": str(c.get("label") or "")[:40]})
        point = pt(d.get("point")) if d.get("point") is not None else None
        box = bx(d.get("box")) if d.get("box") is not None else None
        if point is None and box is not None:
            point = ((box[0] + box[2]) / 2, (box[1] + box[3]) / 2)
        try:
            conf = float(d.get("confidence", 0.0))
        except (TypeError, ValueError):
            conf = 0.0
        found = bool(d.get("found")) and point is not None
        ans = LocateAnswer(found, point, box, str(d.get("label") or "")[:40], conf, str(d.get("reason") or "")[:200],
                           cands, text)
        if found and conf < self.min_confidence:   # not sure: let the player choose
            ans.found = False
            ans.candidates = [{"point": point, "box": box, "label": ans.label}] + [c for c in cands if c["point"] != point]
            ans.reason = ans.reason or "not sure which one you mean"
        if bool(d.get("found")) and point is None:
            ans.reason = "Claude's answer had no usable point"
        return ans


def request_from_answer(ans: LocateAnswer, frame_id: str, q, prompt: str) -> TargetRequest:
    return TargetRequest(frame_id, list(q), "prompt", ans.point, prompt, ans.box, ans.label)


# --------------------------------------------------------------------------- the tracker
class SeededFinder:
    """``ObjectFinder`` that keeps one tapped target across the engine's looks.

    Each ``find``: predict where the top centre is (the last estimate, or the engine's hint),
    resample the frame onto the plane at the target's height around it, NCC-match the top-face
    template taken from the tap frame (the plane resampling makes it the same size from any
    camera distance), re-segment seeded at the match and take the footprint centroid (height
    fixed, refined when the view is oblique enough), and for a cup the rim circle from
    ``CupFinder``. A candidate that moved more than the gate or doesn't look like the target
    (size, NCC) is rejected: the engine sees "lost" rather than a different object."""

    def __init__(self, target: Target, cfg: PickConfig | None = None, min_ncc: float = 0.45,
                 size_tol: float = 0.35, refine_height: bool = True):
        self.t, self.cfg = target, cfg or PickConfig()
        self.xy = np.array(target.xy_table, float)
        self.height = float(target.height)
        self.size = tuple(target.size)
        self.min_ncc, self.size_tol = min_ncc, size_tol
        self.refine_height = refine_height and not target.height_known   # the tap frame saw it: keep it
        self.models = ("circle",) if target.kind == "cup" else ("rect", "circle")
        self.history: list[dict] = []

    def _ncc(self, frame, view: View, centre, radius):
        tp = self.t.template
        if tp is None:
            return None
        res, half_t = tp["res"], tp["half"]
        full = np.full(frame.shape[:2], 255, np.uint8)
        o, valid, grid = view.ortho(frame, self.height, centre, radius + half_t, res=res, mask=full)
        if o is None:
            return None
        g = cv2.cvtColor(o, cv2.COLOR_BGR2GRAY).astype(np.float32)
        T = tp["gray"]
        if g.shape[0] < T.shape[0] or g.shape[1] < T.shape[1]:
            return None
        g[valid == 0] = float(np.median(g[valid > 0])) if (valid > 0).any() else 0.0
        r = cv2.matchTemplate(g, T, cv2.TM_CCOEFF_NORMED)
        _, score, _, loc = cv2.minMaxLoc(r)
        cu, cv_ = loc[0] + T.shape[1] / 2, loc[1] + T.shape[0] / 2
        xy = np.array([grid["x0"] + cu * res, grid["y1"] - cv_ * res])
        return float(score), xy

    def _coverage(self, frame, view: View, xy) -> float:
        a = np.linspace(0, 2 * np.pi, 24, endpoint=False)
        r = 0.45 * self.size[1]
        P = np.c_[xy[0] + r * np.cos(a), xy[1] + r * np.sin(a), np.full(24, self.height)]
        uv = view.project_table(P)
        h, w = frame.shape[:2]
        jaw = view.cam.for_image(frame).jaw_mask()
        ok = np.isfinite(uv).all(1)
        ok[ok] &= (uv[ok, 0] >= 0) & (uv[ok, 0] < w) & (uv[ok, 1] >= 0) & (uv[ok, 1] < h)
        ok[ok] = jaw[uv[ok, 1].astype(int), uv[ok, 0].astype(int)] > 0
        return float(ok.mean())

    def find(self, frame, view: View, hint: Hint | None = None) -> Detection | None:
        centre = np.array(hint.near_table if hint is not None and hint.near_table is not None else self.xy, float)
        radius = float(hint.radius if hint is not None else self.cfg.search_radius)
        rec = {"centre": centre.round(4).tolist(), "radius": radius}
        m = self._ncc(frame, view, centre, radius)
        if m is None:
            self.history.append({**rec, "result": "outside the view"})
            return None
        score, xy_ncc = m
        rec["ncc"] = round(score, 3)
        xy, src = None, None
        uv = view.project_table([[xy_ncc[0], xy_ncc[1], self.height]])[0]
        try:
            if not np.isfinite(uv).all():
                raise TargetRefused("not in view")
            mask, info = segment(frame, view, uv, None, self.cfg)
            hs = [self.height]
            C = view.T_table_cam[:3, 3]
            if self.refine_height and np.hypot(C[0] - xy_ncc[0], C[1] - xy_ncc[1]) > 0.8 * self.size[0]:
                hs = list(np.arange(max(0.0, self.height - 0.02), self.height + 0.0201, 0.01))
            fp = footprint(mask, view, hs, self.models)
            if len(hs) > 1 and fp["observable"]:
                self.height = 0.5 * self.height + 0.5 * fp["height"]
                fp = footprint(mask, view, [self.height], self.models)
            xy_seg, size = fp["xy"], fp["size"]
            ok_size = all(abs(a - b) <= self.size_tol * b + 0.004 for a, b in zip(size, self.size))
            rec.update(seg=xy_seg.round(4).tolist(), seg_size=[round(v, 4) for v in size], touches=info["touches_image"] or info["touches_jaw"])
            if ok_size and np.linalg.norm(xy_seg - xy_ncc) < 0.015 and not (info["touches_image"] or info["touches_jaw"]):
                xy, src = xy_seg, "segment"
        except TargetRefused as e:
            rec["seg_error"] = str(e)
        if xy is None and score >= self.min_ncc + 0.15:
            xy, src = xy_ncc, "ncc"
        if xy is not None and self.t.kind == "cup":
            cup = CupFinder(diameter=self.size[1], height=self.height).find(frame, view, Hint(tuple(xy), 0.02))
            if cup is not None and np.hypot(cup.xyz_table[0] - xy[0], cup.xyz_table[1] - xy[1]) < 0.012:
                xy, src = np.array(cup.xyz_table[:2]), "cup-rim"
        if xy is None or score < self.min_ncc:
            self.history.append({**rec, "result": "lost"})
            return None
        if np.linalg.norm(xy - centre) > radius + 0.01:
            self.history.append({**rec, "result": "outside the gate", "xy": xy.round(4).tolist()})
            return None
        self.xy = np.array(xy, float)
        p_t = np.array([xy[0], xy[1], self.height])
        uvp = view.project_table([p_t])[0]
        dist = float(np.linalg.norm(view.T_table_cam[:3, 3] - p_t))
        sigma = float(np.hypot(0.0015, 1.5 * dist / view.cam.K[0, 0]))
        cov = self._coverage(frame, view, xy)
        rec.update(result=src, xy=self.xy.round(4).tolist(), height=round(self.height, 4))
        self.history.append(rec)
        return Detection("target", view.table_to_base(p_t)[0].tolist(), p_t.tolist(), float(self.t.yaw_table),
                         float(self.size[1]), self.height, sigma, uvp.tolist() if np.isfinite(uvp).all() else None,
                         float(score), extra={"coverage": round(cov, 3), "source": src, "ncc": round(score, 3)})


# --------------------------------------------------------------------------- name it: open-vocabulary
def resolve_prompt(frame, view: View, prompt: str | None, detector, wc: Workcell, cfg: PickConfig | None = None,
                   max_open: float | None = None, max_size: float | None = None) -> tuple[Target | None, list[dict]]:
    """"Pick up the blue marker": the detector's masks (``interfaces.wristcam.detect``) ->
    ``Target`` via the same geometry as a tap (``target_from_mask``). Returns (best, candidates):
    best = the highest-scoring detection that makes a target the gripper can take (else the best
    one with a ``refusal``, so the player sees why); candidates = every detection with its xy,
    score and why it was or wasn't chosen. ``prompt=None``: everything the detector sees.
    Raises TargetRefused("not_found") when nothing matches."""
    cfg = cfg or PickConfig()
    _sync_view(detector, view)
    dets = sorted(detector.detect(frame, [prompt] if prompt else None), key=lambda d: -d.score)
    cands, ok, refused = [], [], []
    for d in dets:
        c = {"label": d.label, "score": round(float(d.score), 3), "box": [round(float(v), 1) for v in d.box]}
        try:
            t = target_from_mask(frame, view, d.mask, wc, cfg, label=d.label, max_open=max_open)
        except TargetRefused as e:
            c.update(error=str(e), code=e.code)
            cands.append(c)
            continue
        t.fit["detector"] = {"name": getattr(detector, "name", "?"), "score": float(d.score), "box": list(d.box)}
        c.update(xy=[round(float(v), 4) for v in t.xy_table], height=round(t.height, 4),
                 size=[round(float(v), 4) for v in t.size], refusal=t.refusal)
        if max_size is not None and not t.refusal and max(t.size) > max_size:
            # 29 Oct, chess: one box covered a pawn and the piece behind it (63 x 23 mm)
            t.refusal = f"bigger than one object ({max(t.size) * 100:.1f} cm > {max_size * 100:.1f} cm): probably two together"
            c["refusal"] = t.refusal
        cands.append(c)
        (refused if t.refusal else ok).append(t)
    best = ok[0] if ok else (refused[0] if refused else None)
    if best is None:
        what = f"'{prompt}'" if prompt else "anything I can pick up"
        why = "; ".join(f"{c['label']}: {c['error']}" for c in cands if "error" in c)[:300]
        raise TargetRefused(f"I can't see {what} here" + (f" ({why})" if why else ""), "not_found")
    return best, cands


def _sync_view(detector, view: View) -> None:
    """The synthetic detector renders ground truth and needs the frame's camera pose."""
    if hasattr(detector, "set_view"):
        detector.set_view(view.cam, view.T_base_cam)


class DetectorFinder:
    """``ObjectFinder`` that re-detects the named object at every look of the engine's closed loop
    (the open-vocabulary detector, not a template), and keeps the one nearest the prediction with
    the target's size. Falls back to ``SeededFinder`` (template + re-segment) when the detector
    misses, so one bad look doesn't lose it."""

    def __init__(self, target: Target, detector, prompt: str | None = None, cfg: PickConfig | None = None,
                 gate: float = 0.04, size_tol: float = 0.4, fallback: bool = True):
        self.t, self.detector, self.cfg = target, detector, cfg or PickConfig()
        self.prompt = prompt or target.label
        self.xy = np.array(target.xy_table, float)
        self.height, self.size = float(target.height), tuple(target.size)
        self.gate, self.size_tol = gate, size_tol
        self.models = ("circle",) if target.kind == "cup" else ("rect", "circle")
        self.fallback = SeededFinder(target, cfg) if fallback else None
        self.history: list[dict] = []

    def find(self, frame, view: View, hint: Hint | None = None) -> Detection | None:
        centre = np.array(hint.near_table if hint is not None and hint.near_table is not None else self.xy, float)
        radius = float(hint.radius if hint is not None else self.gate)
        _sync_view(self.detector, view)
        best, rec = None, {"centre": centre.round(4).tolist(), "radius": radius, "dets": 0}
        try:
            dets = self.detector.detect(frame, [self.prompt])
        except Exception as e:           # a detector failure is a miss, not a crash of the pick
            dets, rec["error"] = [], str(e)[:200]
        rec["dets"] = len(dets)
        for d in dets:
            try:
                fp = footprint(d.mask, view, [self.height], self.models)
            except Exception:
                continue
            xy, size = np.asarray(fp["xy"], float), fp["size"]
            dist = float(np.linalg.norm(xy - centre))
            ok_size = all(abs(a - b) <= self.size_tol * b + 0.006 for a, b in zip(size, self.size))
            if dist <= radius + 0.01 and ok_size and (best is None or dist < best[0]):
                best = (dist, xy, d, fp)
        if best is None:
            if self.fallback is not None:
                self.fallback.xy = self.xy.copy()
                det = self.fallback.find(frame, view, hint)
                rec["result"] = "fallback " + ("found" if det is not None else "lost")
                self.history.append(rec)
                if det is not None:
                    self.xy = np.array(det.xyz_table[:2], float)
                    det.extra["source"] = "template (detector missed)"
                return det
            rec["result"] = "lost"
            self.history.append(rec)
            return None
        _, xy, d, fp = best
        self.xy = xy
        p_t = np.array([xy[0], xy[1], self.height])
        uvp = view.project_table([p_t])[0]
        dist_cam = float(np.linalg.norm(view.T_table_cam[:3, 3] - p_t))
        sigma = float(np.hypot(0.002, 2.0 * dist_cam / view.cam.K[0, 0]))
        rec.update(result="detector", xy=xy.round(4).tolist(), score=round(float(d.score), 3))
        self.history.append(rec)
        return Detection("target", view.table_to_base(p_t)[0].tolist(), p_t.tolist(), float(fp["yaw"]),
                         float(self.size[1]), self.height, sigma, uvp.tolist() if np.isfinite(uvp).all() else None,
                         float(d.score), extra={"source": "detector", "label": d.label})


# --------------------------------------------------------------------------- engine glue
def pick_config(t: Target, wc: Workcell, place_table=None, **kw):
    """GraspConfig for a target: its width and height, look first where it was seen, close across
    its narrow side, put it back (``place_table``; default: where it was)."""
    from .engine import GraspConfig
    return GraspConfig(obj=t.label, width=float(t.width), height=float(t.height),
                       expected_table=tuple(float(v) for v in t.xy_table),
                       place_table=tuple(place_table) if place_table is not None else tuple(float(v) for v in t.xy_table),
                       search_offsets=((0.0, 0.0), (0.03, 0.0), (-0.03, 0.0)),
                       **{"finger_yaw": wc.yaw_to_base(t.grasp_yaw_table), **kw})


class _QArm:
    def __init__(self, q):
        self._q = np.asarray(q, float)

    def q(self):
        return self._q.copy()


def look_pose(wc: Workcell, cam: CameraModel, T_tip_cam, q_seed, xy=None, h=None, cfg: PickConfig | None = None):
    """Joint angles that put the camera straight down over table ``xy`` (default the config's
    look point) ``h`` m above the table (lowered in 1 cm steps until reachable), and the height
    used. None if nothing is reachable."""
    from .engine import GraspConfig, GraspEngine
    cfg = cfg or PickConfig()
    xy = np.array(cfg.look_xy if xy is None else xy, float)
    eng = GraspEngine(_QArm(q_seed), None, cam, T_tip_cam, wc, GraspConfig(height=0.0))
    q, used = eng.camera_over(xy, cfg.look_h if h is None else h)
    return q, used


def run_pick(arm, target: Target, cam: CameraModel, T_tip_cam, wc: Workcell, cfg: PickConfig | None = None,
             place_table=None, on_event=None, log_path=None, frames_dir=None, finder=None, **grasp_kw) -> dict:
    """The closed loop on ``target``: GraspEngine + a finder that keeps the target across looks
    (default SeededFinder; ``DetectorFinder`` for a named object). ``on_event(state, event, data)``
    is called for every engine log line (progress for a UI)."""
    from .engine import GraspEngine

    if target.refusal:
        return {"ok": False, "reason": target.refusal, "trace": ["refused"]}
    finder = finder or SeededFinder(target, cfg)

    class _Engine(GraspEngine):
        def log(self, event, **kw):
            super().log(event, **kw)
            if on_event is not None:
                on_event(self.state, event, self.events[-1])

    gcfg = pick_config(target, wc, place_table, **grasp_kw)
    eng = _Engine(arm, finder, cam, T_tip_cam, wc, gcfg, log_path=log_path, frames_dir=frames_dir)
    res = eng.run()
    res["finder_history"] = finder.history
    res["height"] = round(finder.height, 4)
    return res


def calibration_refusal(wc: Workcell, intrinsics=None, handeye=None) -> str | None:
    """None when tap-to-pick may run: workcell calibrated, lens and hand-eye files present."""
    from .camera import DEFAULT_HANDEYE, DEFAULT_INTRINSICS
    if not wc.calibrated:
        return "camera not calibrated yet (workcell.json has calibrated: false)"
    if not Path(handeye or DEFAULT_HANDEYE).exists():
        return "camera not calibrated yet (no hand-eye calibration: interfaces/wristcam/calib/handeye.json)"
    if not Path(intrinsics or DEFAULT_INTRINSICS).exists():
        return "camera not calibrated yet (no lens calibration: interfaces/wristcam/calib/wristcam.json)"
    return None
