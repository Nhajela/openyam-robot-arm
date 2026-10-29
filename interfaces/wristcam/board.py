"""interfaces.wristcam.board: the one printed sheet that serves all three wrist-camera calibrations.

``print/wristcam_calib_A4.pdf`` is a ChArUco board (DICT_5X5_100, 9 x 6 squares of 28 mm,
21 mm markers, 252 x 168 mm) on A4 landscape, with a 50 mm scale bar. It is **taped flat to
the table** and the arm moves the camera around it:

1. intrinsics: many views of the board (the arm moves, not the board),
2. hand-eye: board pose per view + FK of the arm -> camera relative to ``gripper_tip``,
3. table frame: the board's pose in the base frame **is** the table frame.

Table frame (what ``interfaces/grasp/workcell.json`` ``T_base_table`` means after calibration):
origin at the **centre of the board**, x along the long edge (the printed "+x" arrow), y toward
the top edge of the print ("+y"), z up out of the paper. Tape the sheet with +x pointing straight
away from the arm and the centre ~30 cm in front of the base: then the table axes are parallel to
the base axes (same as the placeholder).

OpenCV's board frame: origin at the board's top-left corner, x right, y DOWN the page, z into the
paper. ``T_BOARD_TABLE`` converts.

``print/wristcam_tags_A4.pdf``: three 40 mm DICT_4X4_50 tags (ids 2, 3, 4: the object ids of
vision/targets/tags.json, a different dictionary from the board's) for tagged objects.

    .venv/bin/python -m interfaces.wristcam.board              # (re)writes print/*.pdf, *.png, *.json
    .venv/bin/python -m interfaces.wristcam.board --screen 1920x1080 --pixel-mm 0.1792
        # a screen version: shows the board on a laptop; prints the square size in mm to pass as
        # --square-mm (pixel pitch = screen width in mm / horizontal pixels)
"""
from __future__ import annotations

import argparse
import json
import os
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

from .camera import make_T

HERE = Path(__file__).resolve().parent
PRINT_DIR = HERE / "print"
# The board actually on the table. WRISTCAM_BOARD overrides (28 Oct the lab printed
# vision/targets/charuco_board_A4.pdf, 7x5 squares of 29.97 mm: print/printed_vision_board.json).
# WRISTCAM_SQUARE_MM overrides the square size of that sheet (the print came out smaller; measure it).
SHEET_JSON = Path(os.environ.get("WRISTCAM_BOARD", PRINT_DIR / "wristcam_calib_board.json"))

DPI = 300
PX_PER_MM = DPI / 25.4
A4_LANDSCAPE_MM = (297.0, 210.0)


@dataclass
class BoardSpec:
    dict_name: str = "DICT_5X5_100"
    squares: tuple = (9, 6)          # columns, rows
    square_mm: float = 28.0
    marker_mm: float = 21.0
    tag_dict: str = "DICT_4X4_50"
    tag_ids: tuple = (2, 3, 4)
    tag_mm: float = 40.0
    extra: dict = field(default_factory=dict)
    # kind "tags": a printed page of ArUco tags at known positions used as the board (e.g. the
    # 40 mm page of vision/targets/aruco_tags_A4.pdf). square_mm is then the tag side and
    # tag_centers_mm the tag centres in page mm (x right, y down), both already at printed scale.
    kind: str = "charuco"
    tag_centers_mm: dict | None = None

    def _tag_layout(self):
        """(origin_mm, size_mm): bounding box of the tag grid in page mm."""
        c = np.array(list(self.tag_centers_mm.values()), float)
        half = self.square_mm / 2
        lo, hi = c.min(0) - half, c.max(0) + half
        return lo, hi - lo

    def tag_corners(self) -> dict[int, np.ndarray]:
        """id -> 4x3 board-frame corners (metres; x right, y down the page, z into it), in
        OpenCV's marker corner order (TL, TR, BR, BL of the printed, upright tag)."""
        lo, _ = self._tag_layout()
        h = self.square_mm / 2
        out = {}
        for k, (cx, cy) in self.tag_centers_mm.items():
            x, y = cx - lo[0], cy - lo[1]
            out[int(k)] = np.array([[x - h, y - h, 0], [x + h, y - h, 0], [x + h, y + h, 0], [x - h, y + h, 0]]) / 1000
        return out

    @property
    def size_m(self) -> tuple[float, float]:
        if self.kind == "tags":
            _, size = self._tag_layout()
            return float(size[0]) / 1000, float(size[1]) / 1000
        return self.squares[0] * self.square_mm / 1000, self.squares[1] * self.square_mm / 1000

    def board(self):
        if self.kind == "tags":
            return None
        d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.dict_name))
        return cv2.aruco.CharucoBoard(tuple(self.squares), self.square_mm / 1000, self.marker_mm / 1000, d)

    def with_measured(self, square_mm: float | None = None, marker_mm: float | None = None) -> "BoardSpec":
        """Measured printed sizes (from the scale bar or the screen pixel pitch)."""
        s = BoardSpec(**{**self.__dict__})
        if self.kind == "tags":
            if square_mm:
                k = square_mm / self.square_mm
                s.square_mm = square_mm
                s.tag_centers_mm = {i: [v[0] * k, v[1] * k] for i, v in self.tag_centers_mm.items()}
            return s
        if square_mm:
            s.marker_mm = marker_mm or self.marker_mm * square_mm / self.square_mm
            s.square_mm = square_mm
        elif marker_mm:
            s.marker_mm = marker_mm
        return s

    @property
    def T_board_table(self) -> np.ndarray:
        """Table frame in board coordinates: centre of the board, y up the page, z out of it."""
        w, h = self.size_m
        return make_T(np.diag([1.0, -1.0, -1.0]), [w / 2, h / 2, 0.0])

    def to_dict(self) -> dict:
        return {"dict": self.dict_name, "squares": list(self.squares), "square_mm": self.square_mm,
                "marker_mm": self.marker_mm, "tag_dict": self.tag_dict, "tag_ids": list(self.tag_ids),
                "tag_mm": self.tag_mm, "table_frame": "origin at the board centre, x = long edge "
                "(+x arrow), y = toward the top of the print, z up"}

    @classmethod
    def load(cls, path=SHEET_JSON) -> "BoardSpec":
        p = Path(path)
        if not p.exists():
            return cls()
        d = json.loads(p.read_text())
        if d.get("kind") == "tags":
            k = float(d.get("print_scale", 1.0))
            spec = cls(d["dict"], (1, 1), float(d["tag_mm"]) * k, 0.0, d["dict"], tuple(int(i) for i in d["centers_mm"]),
                       float(d["tag_mm"]) * k, kind="tags",
                       tag_centers_mm={int(i): [c[0] * k, c[1] * k] for i, c in d["centers_mm"].items()})
            sq = os.environ.get("WRISTCAM_SQUARE_MM")
            return spec.with_measured(float(sq)) if sq else spec
        spec = cls(d["dict"], tuple(d["squares"]), d["square_mm"], d["marker_mm"], d.get("tag_dict", "DICT_4X4_50"),
                   tuple(d.get("tag_ids", (2, 3, 4))), d.get("tag_mm", 40.0))
        # WRISTCAM_SQUARE_MM: the MEASURED printed square (a print that was scaled; marker scales with it)
        sq = os.environ.get("WRISTCAM_SQUARE_MM")
        return spec.with_measured(float(sq)) if sq and Path(path) == SHEET_JSON else spec


def table_points(spec: BoardSpec, obj_board) -> np.ndarray:
    """Board-frame points (N,3) -> table-frame points."""
    T = np.linalg.inv(spec.T_board_table)
    P = np.asarray(obj_board, float).reshape(-1, 3)
    return P @ T[:3, :3].T + T[:3, 3]


# --------------------------------------------------------------------------- detection
class BoardDetector:
    def __init__(self, spec: BoardSpec | None = None):
        self.spec = spec or BoardSpec.load()
        self.board = self.spec.board()
        if self.spec.kind == "tags":
            dp = cv2.aruco.DetectorParameters()
            dp.adaptiveThreshWinSizeMin, dp.adaptiveThreshWinSizeMax, dp.adaptiveThreshWinSizeStep = 3, 53, 4
            dp.minMarkerPerimeterRate = 0.01
            dp.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.spec.dict_name))
            self.tag_det = cv2.aruco.ArucoDetector(d, dp)
            self.corners_by_id = self.spec.tag_corners()
            return
        cp = cv2.aruco.CharucoParameters()
        cp.minMarkers = 1
        cp.tryRefineMarkers = True
        # Tuned on the real wrist camera (28 Oct): the defaults decoded 6/17 markers of a board
        # ~190 px wide at 1280x720; these decode 17/17 (24/24 ChArUco corners).
        dp = cv2.aruco.DetectorParameters()
        dp.adaptiveThreshWinSizeMin, dp.adaptiveThreshWinSizeMax, dp.adaptiveThreshWinSizeStep = 3, 53, 4
        dp.minMarkerPerimeterRate = 0.01
        dp.polygonalApproxAccuracyRate = 0.05
        dp.errorCorrectionRate = 0.8
        dp.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        self.det = cv2.aruco.CharucoDetector(self.board, cp, dp)

    def detect(self, img, mask=None):
        """-> (obj Nx3 board metres, img Nx2 px, ids N) or None. ``mask`` (255 = usable) drops
        corners hidden by the jaws."""
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
        if self.spec.kind == "tags":
            corners, ids, _ = self.tag_det.detectMarkers(gray)
            if ids is None:
                return None
            obj, px, pid = [], [], []
            for c, i in zip(corners, ids.ravel()):
                if int(i) in self.corners_by_id:
                    obj.append(self.corners_by_id[int(i)]); px.append(c.reshape(4, 2)); pid += [int(i) * 4 + k for k in range(4)]
            if not obj:
                return None
            obj, cc, ci = np.vstack(obj), np.vstack(px).astype(np.float64), np.array(pid)
            if mask is not None:
                h, w = mask.shape
                u = np.clip(cc[:, 0].round().astype(int), 0, w - 1)
                v = np.clip(cc[:, 1].round().astype(int), 0, h - 1)
                keep = mask[v, u] > 0
                obj, cc, ci = obj[keep], cc[keep], ci[keep]
            return (obj, cc, ci) if len(ci) >= 4 else None
        cc, ci, _, _ = self.det.detectBoard(gray)
        if ci is None or len(ci) < 4:
            return None
        cc = cc.reshape(-1, 2).astype(np.float64)
        ci = ci.ravel()
        if mask is not None:
            h, w = mask.shape
            u = np.clip(cc[:, 0].round().astype(int), 0, w - 1)
            v = np.clip(cc[:, 1].round().astype(int), 0, h - 1)
            keep = mask[v, u] > 0
            cc, ci = cc[keep], ci[keep]
            if len(ci) < 4:
                return None
        obj = self.board.getChessboardCorners()[ci].astype(np.float64)
        return obj, cc, ci


def board_pose(cam, obj, img_pts, min_points: int = 6):
    """T_cam_board from detected corners, model-agnostic (PnP on undistorted rays).
    Returns (T, rms_px) or (None, inf)."""
    if obj is None or len(obj) < min_points:
        return None, float("inf")
    n = cam.normalize(img_pts)
    ok, rv, tv = cv2.solvePnP(obj, n, np.eye(3), None, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None, float("inf")
    rv, tv = cv2.solvePnPRefineLM(obj, n, np.eye(3), None, rv, tv)
    T = make_T(cv2.Rodrigues(rv)[0], tv.ravel())
    proj = cam.project(obj @ T[:3, :3].T + T[:3, 3])
    rms = float(np.sqrt(np.nanmean(np.sum((proj - img_pts) ** 2, axis=1))))
    return T, rms


# --------------------------------------------------------------------------- printing
def _mm(v):
    return int(round(v * PX_PER_MM))


def _font(size_px):
    from PIL import ImageFont
    for name in ("DejaVuSans.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, size_px)
        except OSError:
            pass
    return ImageFont.load_default()


def _scale_bar(draw, x_mm, y_mm, font):
    draw.rectangle([_mm(x_mm), _mm(y_mm), _mm(x_mm + 50), _mm(y_mm + 2.5)], fill=0)
    for k in range(6):
        draw.line([_mm(x_mm + 10 * k), _mm(y_mm - 2), _mm(x_mm + 10 * k), _mm(y_mm + 4.5)], fill=0, width=3)
    draw.text((_mm(x_mm), _mm(y_mm + 5)), "50 mm: must measure 50.0 mm", fill=0, font=font)
    draw.text((_mm(x_mm), _mm(y_mm + 9)), "(print at 100%, no fit-to-page)", fill=0, font=font)


def _arrow(draw, x0, y0, x1, y1, label, font):
    draw.line([_mm(x0), _mm(y0), _mm(x1), _mm(y1)], fill=0, width=8)
    d = np.array([x1 - x0, y1 - y0], float)
    d /= np.linalg.norm(d)
    nrm = np.array([-d[1], d[0]])
    tip = np.array([x1, y1])
    a, b = tip - 4 * d + 2 * nrm, tip - 4 * d - 2 * nrm
    draw.polygon([(_mm(tip[0]), _mm(tip[1])), (_mm(a[0]), _mm(a[1])), (_mm(b[0]), _mm(b[1]))], fill=0)
    draw.text((_mm(x1 + 1.5 * d[0] + 1), _mm(y1 + 1.5 * d[1] - 2)), label, fill=0, font=font)


def make_calib_sheet(spec: BoardSpec | None = None):
    """PIL image (A4 landscape, 300 dpi) of the calibration sheet."""
    from PIL import Image, ImageDraw
    spec = spec or BoardSpec()
    W, H = _mm(A4_LANDSCAPE_MM[0]), _mm(A4_LANDSCAPE_MM[1])
    page = Image.new("L", (W, H), 255)
    bw, bh = spec.squares[0] * spec.square_mm, spec.squares[1] * spec.square_mm
    x0, y0 = (A4_LANDSCAPE_MM[0] - bw) / 2, 10.0
    sq_px = spec.square_mm * PX_PER_MM
    img = spec.board().generateImage((int(round(spec.squares[0] * sq_px)), int(round(spec.squares[1] * sq_px))),
                                     marginSize=0, borderBits=1)
    page.paste(Image.fromarray(img), (_mm(x0), _mm(y0)))
    d = ImageDraw.Draw(page)
    f, fs = _font(_mm(3.2)), _font(_mm(2.6))
    cx, cy = x0 + bw / 2, y0 + bh / 2
    for (xa, ya, xb, yb) in ((cx, y0 - 1, cx, y0 - 6), (cx, y0 + bh + 1, cx, y0 + bh + 6),
                             (x0 - 1, cy, x0 - 6, cy), (x0 + bw + 1, cy, x0 + bw + 6, cy)):
        d.line([_mm(xa), _mm(ya), _mm(xb), _mm(yb)], fill=0, width=4)   # centre ticks = table origin
    yb = y0 + bh + 9
    _scale_bar(d, x0, yb + 3, fs)
    ax, ay = x0 + 175, yb + 15
    _arrow(d, ax, ay, ax + 18, ay, "+x (away from the arm)", fs)
    _arrow(d, ax, ay, ax, ay - 11, "+y", fs)
    for k, line in enumerate(("OpenYAM wrist-camera calibration sheet",
                              f"ChArUco {spec.dict_name} {spec.squares[0]}x{spec.squares[1]}, square {spec.square_mm:g} mm, "
                              f"marker {spec.marker_mm:g} mm",
                              "Tape FLAT to the table. Table origin = board centre (ticks).")):
        d.text((_mm(x0 + 60), _mm(yb + 0.5 + 4 * k)), line, fill=0, font=fs)
    return page


def make_tag_sheet(spec: BoardSpec | None = None):
    from PIL import Image, ImageDraw
    spec = spec or BoardSpec()
    W, H = _mm(A4_LANDSCAPE_MM[0]), _mm(A4_LANDSCAPE_MM[1])
    page = Image.new("L", (W, H), 255)
    d = ImageDraw.Draw(page)
    f = _font(_mm(3.5))
    dic = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec.tag_dict))
    side = _mm(spec.tag_mm)
    for k, tid in enumerate(spec.tag_ids):
        x, y = 30 + k * 85, 50
        img = cv2.aruco.generateImageMarker(dic, int(tid), side, borderBits=1)
        page.paste(Image.fromarray(img), (_mm(x), _mm(y)))
        m = 8   # dashed cut line, keep the white margin
        for t in range(0, int(spec.tag_mm + 2 * m), 4):
            for (xa, ya, xb, yb) in ((x - m + t, y - m, x - m + t + 2, y - m), (x - m + t, y + spec.tag_mm + m, x - m + t + 2, y + spec.tag_mm + m),
                                     (x - m, y - m + t, x - m, y - m + t + 2), (x + spec.tag_mm + m, y - m + t, x + spec.tag_mm + m, y - m + t + 2)):
                d.line([_mm(xa), _mm(ya), _mm(min(xb, x + spec.tag_mm + m)), _mm(min(yb, y + spec.tag_mm + m))], fill=0, width=2)
        d.text((_mm(x), _mm(y + spec.tag_mm + 10)), f"id {tid}  ({spec.tag_dict}, {spec.tag_mm:g} mm)", fill=0, font=f)
    d.text((_mm(30), _mm(20)), "Object tags for the wrist camera. Stick one flat on the TOP of an object, centred.", fill=0, font=f)
    _scale_bar(d, 30, 160, _font(_mm(2.6)))
    return page


def write_prints(out: Path = PRINT_DIR, spec: BoardSpec | None = None) -> list[Path]:
    spec = spec or BoardSpec()
    out.mkdir(parents=True, exist_ok=True)
    paths = []
    for name, page in (("wristcam_calib_A4", make_calib_sheet(spec)), ("wristcam_tags_A4", make_tag_sheet(spec))):
        page.save(out / f"{name}.png", dpi=(DPI, DPI))
        page.save(out / f"{name}.pdf", resolution=DPI)
        paths += [out / f"{name}.pdf", out / f"{name}.png"]
    (out / SHEET_JSON.name).write_text(json.dumps(spec.to_dict(), indent=1) + "\n")
    return paths + [out / SHEET_JSON.name]


def screen_board(spec: BoardSpec, screen=(1920, 1080), pixel_mm: float = 0.18) -> tuple[np.ndarray, float]:
    """Board image for a laptop screen (full screen, no scaling!) and its square size in mm."""
    sq_px = int(min(screen[0] * 0.95 / spec.squares[0], screen[1] * 0.95 / spec.squares[1]))
    img = np.full((screen[1], screen[0]), 255, np.uint8)
    b = spec.board().generateImage((sq_px * spec.squares[0], sq_px * spec.squares[1]), marginSize=0, borderBits=1)
    y0, x0 = (screen[1] - b.shape[0]) // 2, (screen[0] - b.shape[1]) // 2
    img[y0:y0 + b.shape[0], x0:x0 + b.shape[1]] = b
    return img, sq_px * pixel_mm


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=str(PRINT_DIR))
    ap.add_argument("--screen", default=None, help="WxH: write a screen version instead of the prints")
    ap.add_argument("--pixel-mm", type=float, default=None, help="screen pixel pitch in mm (screen width mm / px)")
    a = ap.parse_args()
    spec = BoardSpec()
    if a.screen:
        if not a.pixel_mm:
            ap.error("--screen needs --pixel-mm (measure the visible screen width in mm, divide by its pixels)")
        w, h = (int(v) for v in a.screen.lower().split("x"))
        img, sq_mm = screen_board(spec, (w, h), a.pixel_mm)
        p = Path(a.out) / f"wristcam_calib_screen_{w}x{h}.png"
        cv2.imwrite(str(p), img)
        print(f"wrote {p}: show it full screen at 100% zoom; square = {sq_mm:.3f} mm -> pass --square-mm {sq_mm:.3f}")
        return
    for p in write_prints(Path(a.out), spec):
        print("wrote", p)


if __name__ == "__main__":
    main()
