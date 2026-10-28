"""Board localisation from four ArUco corner markers, and the canonical top-down warp.

Board frame (millimetres): origin at the outer corner of a1's playing square, x along the files
(a -> h), y along the ranks (1 -> 8), z up. Square (file f, rank r) spans [f*s, (f+1)*s] x
[r*s, (r+1)*s], s = ``square_mm``. This is the same file/rank convention as
``interfaces.grasp.chess_arm.BoardGeometry``; the arm-side transform comes from the corner-touch
calibration (CHESS.md section 2), not from here.

Markers: DICT_4X4_50 (the dictionary already printed by ``vision/make_targets.py``), IDs **10-13**
(0-9 are the table/gripper/object tags in ``vision/targets/tags.json``), 40 mm by default:

    id 13 (a8 corner) .......... id 12 (h8 corner)
         :      black side (rank 8)      :
         :                               :
         :      white side (rank 1)      :
    id 10 (a1 corner) .......... id 11 (h1 corner)

Each marker sits diagonally outside its corner, edges parallel to the board edges, its inner
corner ``gap_mm`` from the playing-area corner along both axes. The marker's own rotation does not
matter (each marker's corner order is solved for), so you only need the right ID at the right
corner. Placement errors of a few mm are absorbed by the grid refinement during calibration
(``refine_grid``), as long as the markers stay taped to the board afterwards.

``BoardLocator.locate(frame)`` returns a ``BoardView`` with the homography image -> board mm:
  - 4 markers: fresh homography from all 16 marker corners (sub-pixel refined).
  - 3 markers (a hand over one): reuse the last 4-marker homography if it is younger than
    ``staleness_s`` and the 3 visible markers still agree with it (camera/board not bumped);
    otherwise a fresh 3-marker homography (12 corners).
  - 1-2 markers: reuse the last good homography under the same age/consistency check, else fail.
  - 0 markers: reuse only if younger than ``staleness_s / 4`` (it cannot be checked).
"""
from __future__ import annotations

import itertools
import time
from dataclasses import asdict, dataclass, field

import cv2
import numpy as np

CORNERS = ("a1", "h1", "h8", "a8")
DEFAULT_IDS = {"a1": 10, "h1": 11, "h8": 12, "a8": 13}
FILES = "abcdefgh"


def square_names() -> list[str]:
    """a1, b1, ..., h1, a2, ..., h8 (python-chess square index order)."""
    return [f"{FILES[i % 8]}{i // 8 + 1}" for i in range(64)]


SQUARES = square_names()


@dataclass
class BoardSpec:
    square_mm: float = 35.0
    marker_mm: float = 40.0
    gap_mm: float = 5.0
    ids: dict = field(default_factory=lambda: dict(DEFAULT_IDS))
    dict_name: str = "DICT_4X4_50"

    @property
    def size_mm(self) -> float:
        return 8 * self.square_mm

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "BoardSpec":
        d = dict(d)
        if "ids" in d:
            d["ids"] = {k: int(v) for k, v in d["ids"].items()}
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})

    def marker_center(self, corner: str) -> np.ndarray:
        o, S = self.gap_mm + self.marker_mm / 2, self.size_mm
        return np.array({"a1": (-o, -o), "h1": (S + o, -o), "h8": (S + o, S + o), "a8": (-o, S + o)}[corner], float)

    def marker_corners(self, corner: str) -> np.ndarray:
        """4x2 board mm, in ArUco order (TL, TR, BR, BL) for a marker printed upright (its top towards +y)."""
        c, h = self.marker_center(corner), self.marker_mm / 2
        return c + np.array([[-h, h], [h, h], [h, -h], [-h, -h]])

    def corner_of_id(self) -> dict[int, str]:
        return {int(v): k for k, v in self.ids.items()}

    def square_center(self, sq: str | int) -> np.ndarray:
        i = SQUARES.index(sq) if isinstance(sq, str) else int(sq)
        return np.array([(i % 8 + 0.5) * self.square_mm, (i // 8 + 0.5) * self.square_mm])

    def grid_points(self) -> np.ndarray:
        """81x2 board mm, the square corners (i*s, j*s)."""
        s = self.square_mm
        return np.array([(i * s, j * s) for j in range(9) for i in range(9)], float)


def aruco_detector(dict_name: str = "DICT_4X4_50"):
    p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.cornerRefinementWinSize = 5
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name)), p)


def apply_h(H: np.ndarray, pts) -> np.ndarray:
    pts = np.asarray(pts, float).reshape(-1, 2)
    q = np.c_[pts, np.ones(len(pts))] @ H.T
    return q[:, :2] / q[:, 2:3]


def guess_K(w: int, h: int) -> np.ndarray:
    """Same guess as vision/detect_tags.py when there is no calibration: f = 0.9 x width."""
    f = 0.9 * w
    return np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]], float)


@dataclass
class BoardView:
    ok: bool
    H: np.ndarray | None = None              # image px -> board mm (refined)
    n_markers: int = 0
    seen: tuple = ()                         # corner names whose markers were seen
    reproj_px: float = 0.0
    age_s: float = 0.0                       # > 0 when the homography was reused
    quality: float = 0.0                     # 0..1, used as the camera's fusion weight
    cam_pos: np.ndarray | None = None        # camera centre in board mm (x, y, z up)
    image_size: tuple = (0, 0)
    reason: str = ""
    markers: dict = field(default_factory=dict)  # corner -> 4x2 image px (for drawing)


class BoardLocator:
    def __init__(self, spec: BoardSpec, K: np.ndarray | None = None, staleness_s: float = 30.0,
                 max_reproj_px: float = 3.0, fix: np.ndarray | None = None):
        self.spec = spec
        self.K = K
        self.staleness_s = staleness_s
        self.max_reproj_px = max_reproj_px
        self.fix = np.eye(3) if fix is None else np.asarray(fix, float)  # nominal board mm -> true board mm
        self.detector = aruco_detector(spec.dict_name)
        self.shift_cache: dict[str, int] = {}
        self.last: dict | None = None        # {"H": nominal H, "t", "n", "cam_pos", "markers"}

    # ------------------------------------------------------------------ detection
    def detect(self, frame: np.ndarray) -> dict[str, np.ndarray]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        corners, ids, _ = self.detector.detectMarkers(gray)
        out: dict[str, np.ndarray] = {}
        if ids is None:
            return out
        by_id = self.spec.corner_of_id()
        for c, i in zip(corners, ids.ravel()):
            name = by_id.get(int(i))
            if name is not None and name not in out:
                out[name] = c.reshape(4, 2).astype(float)
        return out

    def _fit(self, markers: dict[str, np.ndarray]):
        """Homography image -> nominal board mm, solving each marker's rotation (cyclic corner shift)."""
        names = sorted(markers)
        img = np.concatenate([markers[n] for n in names])

        def solve(shifts):
            brd = np.concatenate([np.roll(self.spec.marker_corners(n), -k, axis=0) for n, k in zip(names, shifts)])
            H, _ = cv2.findHomography(img, brd, 0)
            if H is None:
                return None, np.inf
            err = np.linalg.norm(apply_h(np.linalg.inv(H), brd) - img, axis=1)
            return H, float(np.sqrt(np.mean(err ** 2)))

        cached = [self.shift_cache.get(n) for n in names]
        if all(c is not None for c in cached):
            H, e = solve(cached)
            if e <= self.max_reproj_px:
                return H, e
        best = (None, np.inf, None)
        for shifts in itertools.product(range(4), repeat=len(names)):
            H, e = solve(shifts)
            if e < best[1]:
                best = (H, e, shifts)
        if best[0] is not None and best[1] <= self.max_reproj_px:
            self.shift_cache.update(dict(zip(names, best[2])))
        return best[0], best[1]

    def _consistent(self, H_nom: np.ndarray, markers: dict[str, np.ndarray]) -> bool:
        if not markers:
            return True
        Hi = np.linalg.inv(H_nom)
        errs = []
        for n, m in markers.items():
            pred_c = apply_h(Hi, [self.spec.marker_center(n)])[0]
            size_px = np.linalg.norm(m[0] - m[2])
            errs.append(np.linalg.norm(m.mean(axis=0) - pred_c) / max(size_px, 1.0))
        return max(errs) < 0.15

    def _cam_pos(self, H_nom: np.ndarray, size: tuple) -> np.ndarray | None:
        w, h = size
        K = self.K if self.K is not None else guess_K(w, h)
        brd = np.concatenate([self.spec.marker_corners(c) for c in CORNERS])
        brd_true = apply_h(self.fix, brd)
        img = apply_h(np.linalg.inv(H_nom), brd)
        obj = np.c_[brd_true, np.zeros(len(brd_true))].astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(obj, img.astype(np.float32), K, None, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            return None
        R, _ = cv2.Rodrigues(rvec)
        C = (-R.T @ tvec).ravel()
        if C[2] < 0:   # mirrored solution: the camera is always above the board
            C[2] = -C[2]
        return C

    # ------------------------------------------------------------------ main entry
    def locate(self, frame: np.ndarray, t: float | None = None) -> BoardView:
        t = time.monotonic() if t is None else t
        size = (frame.shape[1], frame.shape[0])
        markers = self.detect(frame)
        n = len(markers)
        last = self.last
        age = (t - last["t"]) if last else np.inf

        def reuse(why: str, quality_scale: float) -> BoardView:
            q = last["q"] * quality_scale * (1.0 - 0.5 * age / self.staleness_s)
            return BoardView(True, self.fix @ last["H"], n, tuple(sorted(markers)), last["reproj"], age, max(q, 0.05),
                             last["cam_pos"], size, why, markers)

        if n == 4:
            H, e = self._fit(markers)
            if H is not None and e <= self.max_reproj_px:
                cam = self._cam_pos(H, size)
                q = float(np.exp(-e / 2.0))
                self.last = {"H": H, "t": t, "n": 4, "reproj": e, "cam_pos": cam, "q": q}
                return BoardView(True, self.fix @ H, 4, tuple(sorted(markers)), e, 0.0, q, cam, size,
                                 "4 markers", markers)
        if n >= 1 and last is not None and age <= self.staleness_s and self._consistent(last["H"], markers):
            return reuse(f"{n} markers, reused homography from {age:.1f} s ago", 0.9 if n == 3 else 0.75)
        if n == 0 and last is not None and age <= self.staleness_s / 4:
            return reuse(f"no markers, reused homography from {age:.1f} s ago (unchecked)", 0.5)
        if n == 3:
            H, e = self._fit(markers)
            if H is not None and e <= self.max_reproj_px:
                cam = self._cam_pos(H, size)
                q = 0.7 * float(np.exp(-e / 2.0))
                if last is None or last["n"] < 4 or age > self.staleness_s:
                    self.last = {"H": H, "t": t, "n": 3, "reproj": e, "cam_pos": cam, "q": q}
                return BoardView(True, self.fix @ H, 3, tuple(sorted(markers)), e, 0.0, q, cam, size,
                                 "3 markers (fresh)", markers)
        why = f"only {n} of 4 corner markers" if n < 3 else "marker geometry inconsistent"
        if last is not None and age > self.staleness_s:
            why += f"; last good homography is {age:.0f} s old (> {self.staleness_s:.0f} s)"
        elif last is not None:
            why += "; visible markers disagree with the last homography (board or camera moved?)"
        return BoardView(False, None, n, tuple(sorted(markers)), 0.0, 0.0, 0.0, None, size, why, markers)


# ---------------------------------------------------------------------- canonical warp
class Canon:
    """Top-down image of the board: ``pps`` px per square, ``margin_sq`` squares of margin, white at the
    bottom (rank 8 at the top row), a-file on the left."""

    def __init__(self, spec: BoardSpec, pps: int = 64, margin_sq: float = 0.5):
        self.spec, self.pps, self.margin = spec, int(pps), float(margin_sq)
        self.size = int(round((8 + 2 * self.margin) * self.pps))
        k = self.pps / spec.square_mm
        m = self.margin * self.pps
        # board mm -> warp px: u = x*k + m, v = (8s - y)*k + m
        self.B2W = np.array([[k, 0, m], [0, -k, 8 * self.pps + m], [0, 0, 1]], float)

    def board_to_px(self, pts) -> np.ndarray:
        return apply_h(self.B2W, pts)

    def px_to_board(self, pts) -> np.ndarray:
        return apply_h(np.linalg.inv(self.B2W), pts)

    def square_center_px(self, i: int) -> np.ndarray:
        return self.board_to_px([self.spec.square_center(i)])[0]

    def warp(self, frame: np.ndarray, H_img2board: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(warped BGR, valid mask uint8) where valid marks pixels that came from inside the frame."""
        M = self.B2W @ H_img2board
        sz = (self.size, self.size)
        img = cv2.warpPerspective(frame, M, sz, flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        ones = np.full(frame.shape[:2], 255, np.uint8)
        valid = cv2.warpPerspective(ones, M, sz, flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)
        valid = cv2.erode(valid, np.ones((3, 3), np.uint8))
        return img, valid


def refine_grid(empty_warp: np.ndarray, canon: Canon) -> tuple[np.ndarray | None, dict]:
    """Find the 7x7 inner square corners of an EMPTY board in the warped image and return the
    correction homography nominal board mm -> true board mm (so the warp grid lands on the real
    squares even when the markers were placed a few mm off). Returns (None, info) if not found."""
    gray = cv2.cvtColor(empty_warp, cv2.COLOR_BGR2GRAY) if empty_warp.ndim == 3 else empty_warp
    found, pts = cv2.findChessboardCornersSB(gray, (7, 7), flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_ACCURACY)
    if not found:
        found, pts = cv2.findChessboardCorners(gray, (7, 7), flags=cv2.CALIB_CB_ADAPTIVE_THRESH)
        if found:
            pts = cv2.cornerSubPix(gray, pts, (5, 5), (-1, -1),
                                   (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.01))
    if not found:
        return None, {"found": False}
    s = canon.spec.square_mm
    nominal = canon.px_to_board(pts.reshape(-1, 2))           # where the corners appear, nominal board mm
    ideal = np.round(nominal / s) * s                        # nearest true grid point
    ok = (np.abs(nominal - ideal).max(axis=1) < 0.35 * s) & (ideal.min(axis=1) > 0) & (ideal.max(axis=1) < 8 * s)
    uniq = len({tuple(p) for p in ideal[ok]})
    if ok.sum() < 40 or uniq != ok.sum():
        return None, {"found": True, "matched": int(ok.sum()), "unique": uniq}
    F, _ = cv2.findHomography(nominal[ok], ideal[ok], 0)
    resid = np.linalg.norm(apply_h(F, nominal[ok]) - ideal[ok], axis=1)
    shift = np.linalg.norm(nominal[ok] - ideal[ok], axis=1)
    return F, {"found": True, "matched": int(ok.sum()), "resid_mm": float(resid.mean()),
               "max_shift_mm": float(shift.max())}
