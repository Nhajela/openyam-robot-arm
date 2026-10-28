"""CameraBoardSensor: the game's ``BoardSensor`` backed by one or more cameras around the board.

    sensor = CameraBoardSensor(["top=phone:top", "side=phone:side"], "interfaces/chess_vision/calib")
    snap = sensor.snapshot()          # Snapshot(t, occupancy, colour, frames)

Sources (``sources`` list entries):
  - a ``vision/sources.py`` spec: ``/dev/video0``, ``dashboard``, ``phone:top``, ``http://.../x.jpg``,
    an image file/folder/glob; optionally prefixed with a camera name, ``top=phone:top``. Without a
    name it is derived (``phone:top`` -> ``top``, ``/dev/video2`` -> ``video2``, a path -> its stem).
  - a ``(name, spec_or_source)`` tuple, or any object with ``read() -> BGR ndarray`` and ``name``.
Each camera must have been calibrated under the same name (``python -m interfaces.chess_vision.calibrate``).

Fusion: per square, each camera's occupancy logit is averaged with weight
``quality (markers, reprojection, staleness) x view_w (falls with obliqueness) x visibility``;
colour likewise, additionally weighted by that camera's P(occupied). A weighted mean of logits,
not a sum, so two cameras that see the same thing do not double their confidence. Squares no
camera sees get 0.5.
"""
from __future__ import annotations

import json
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from .board_geom import BoardLocator, BoardSpec, BoardView, Canon, SQUARES, apply_h, refine_grid
from .occupancy import CameraModel, Logistic, Regions, sample_regions
from .sensor import Snapshot

REPO = Path(__file__).resolve().parents[2]
CALIB_JSON = "chess_vision.json"
DEFAULT_CALIB = Path(__file__).resolve().parent / "calib"


# ---------------------------------------------------------------------- sources
@dataclass
class PosedFrame:
    """A frame that arrives with a known camera pose. A source's ``read()`` may return one instead
    of a bare BGR array. This is the seam for **arm scan mode** (planned, see README): the wrist
    camera between the gripper jaws taken to 3-4 top-down scan poses, where the pose is
    FK(gripper tip) x hand-eye transform expressed in the board frame.

    - ``cam_pos_mm``: camera centre in the board frame (x along files, y along ranks, z up, mm).
      Used instead of the marker-PnP estimate for the oblique-sampling shift and fusion weight.
    - ``H_img2board``: image px (after undistortion) -> board mm, from the pose and intrinsics.
      Used only when the corner markers are not visible (a close-up scan pose sees part of the
      board); markers, when seen, still win because they absorb pose error.
    Each scan pose is calibrated as its own named camera (its own empty-board reference), since a
    wrist camera's view of the board differs per pose."""
    image: np.ndarray
    cam_pos_mm: np.ndarray | None = None
    H_img2board: np.ndarray | None = None
    quality: float = 0.6


def _vision_sources():
    vdir = str(REPO / "vision")
    if vdir not in sys.path:
        sys.path.insert(0, vdir)
    import sources  # noqa: E402  (vision/ is a script folder, not a package)
    return sources


def split_named(entry: str) -> tuple[str | None, str]:
    m = re.match(r"^([A-Za-z0-9_\-]+)=(.+)$", entry)
    if m and "/" not in m.group(1) and ":" not in m.group(1):
        return m.group(1), m.group(2)
    return None, entry


def default_name(spec: str) -> str:
    if spec.startswith("phone:"):
        return spec.split(":", 1)[1]
    if spec.startswith(("v4l2:", "v4l2+ffmpeg:")):
        spec = spec.split(":", 1)[1]
    if spec.startswith("/dev/"):
        return Path(spec).name
    if spec.isdigit():
        return f"video{spec}"
    if spec.startswith(("http://", "https://")):
        return re.sub(r"\W+", "_", spec.split("//", 1)[1]).strip("_")[-40:]
    return Path(spec.rstrip("/")).stem or spec


def open_spec(spec: str, size=(1280, 720)):
    src = _vision_sources()
    if spec.startswith(("phone:", "http://", "https://")) or spec == "dashboard":
        return src.open_source(spec, new_only=False)   # a snapshot wants the latest frame, even if unchanged
    return src.open_source(spec, size=size)


def resolve_sources(sources) -> list[tuple[str, object, str | None]]:
    """-> [(name, source object, spec or None)]"""
    out = []
    for e in sources:
        if isinstance(e, tuple):
            name, s = e
            out.append((name, open_spec(s) if isinstance(s, str) else s, s if isinstance(s, str) else None))
        elif isinstance(e, str):
            name, spec = split_named(e)
            out.append((name or default_name(spec), open_spec(spec), spec))
        else:
            out.append((getattr(e, "name", f"cam{len(out)}"), e, None))
    return out


class _Grabber:
    """Wraps a source: flushes stale buffered frames from V4L2 and keeps the last frame of
    exhausted file sources (so a folder of one image acts as a still camera)."""

    def __init__(self, src):
        self.src = src
        self.last: np.ndarray | None = None

    def grab(self, timeout: float = 5.0) -> np.ndarray | None:
        cap = getattr(self.src, "cap", None)
        if cap is not None:
            for _ in range(2):
                cap.grab()
        t0 = time.time()
        while True:
            f = self.src.read()
            if f is not None:
                self.last = f
                return f
            if not getattr(self.src, "live", True) or time.time() - t0 > timeout:
                return self.last
            time.sleep(0.05)

    def close(self):
        c = getattr(self.src, "close", None)
        if c:
            c()


# ---------------------------------------------------------------------- intrinsics
def load_intrinsics(path: str | Path):
    """vision/calib/<camera>.yaml -> (K, dist)"""
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise FileNotFoundError(path)
    K = fs.getNode("camera_matrix").mat()
    dist = fs.getNode("dist_coeffs").mat().ravel()
    fs.release()
    return K, dist


def undistort(frame, intr):
    if intr is None:
        return frame
    K, dist = intr
    return cv2.undistort(frame, K, dist) if dist is not None and np.any(dist) else frame


# ---------------------------------------------------------------------- calibration
def calib_json_path(path: str | Path) -> Path:
    p = Path(path)
    return p if p.suffix == ".json" else p / CALIB_JSON


def calibrate_camera(name: str, empty_frames: list, start_frames: list, spec: BoardSpec, canon: Canon,
                     intr=None, refine: bool = True) -> dict:
    """Fit one camera from frames of the empty board and of the start position.
    Returns {"model": CameraModel, "fix": 3x3, "info": {...}}. Raises ValueError with a reason."""
    K = intr[0] if intr is not None else None
    loc = BoardLocator(spec, K)

    def warps(frames, what, reuse=False):
        # The camera must not move between the empty-board and start-position frames: pieces may
        # hide a marker in the start position (a tall rook in front of a8 from the side), so those
        # frames reuse the empty board's homography when the visible markers still agree with it.
        acc, valid, view = None, None, None
        for k, f in enumerate(frames):
            f = undistort(f, intr)
            view = loc.locate(f, t=0.0)
            if not view.ok or (view.n_markers < 4 and not (reuse and view.age_s == 0.0 and "reused" in view.reason)):
                raise ValueError(f"{name}: {what} frame {k}: need all 4 corner markers in view ({view.reason}; "
                                 f"seen {list(view.seen)})")
            w, v = canon.warp(f, view.H)
            acc = w.astype(np.float32) if acc is None else acc + w
            valid = v if valid is None else np.minimum(valid, v)
            if not reuse:
                loc.last = None   # every empty-board frame is located fresh
        return (acc / len(frames)).astype(np.uint8), valid, view

    empty, valid_e, view = warps(empty_frames, "empty-board")
    info: dict = {"reproj_px": round(view.reproj_px, 3)}
    if refine:
        F, rinfo = refine_grid(empty, canon)
        info["grid"] = rinfo
        if F is not None:
            loc.fix = F
            empty, valid_e, view = warps(empty_frames, "empty-board")
    loc.locate(undistort(empty_frames[-1], intr), t=0.0)  # remember the empty board's homography
    start, valid_s, view_s = warps(start_frames, "start-position", reuse=True)
    regions = sample_regions(canon, view_s.cam_pos)
    model = CameraModel(canon, empty, valid_e)
    info.update(model.fit(start, np.minimum(valid_s, valid_e), regions))
    info["cam_pos_mm"] = None if view.cam_pos is None else [round(float(v), 1) for v in view.cam_pos]
    info["mean_lean"] = round(float(regions.lean.mean()), 3)
    return {"model": model, "fix": loc.fix, "info": info, "start": start}


def save_calibration(path: str | Path, spec: BoardSpec, canon: Canon, cams: dict) -> Path:
    """cams: name -> {"model", "fix", "info", optional "source", "intrinsics", "start"}"""
    jp = calib_json_path(path)
    jp.parent.mkdir(parents=True, exist_ok=True)
    doc = {"version": 1, "saved": time.strftime("%Y-%m-%d %H:%M:%S"), "board": spec.to_dict(),
           "pps": canon.pps, "margin_sq": canon.margin, "cameras": {}}
    for name, c in cams.items():
        m: CameraModel = c["model"]
        cv2.imwrite(str(jp.parent / f"{name}_empty.png"), m.ref.astype(np.uint8))
        cv2.imwrite(str(jp.parent / f"{name}_valid.png"), m.ref_valid)
        if c.get("start") is not None:
            cv2.imwrite(str(jp.parent / f"{name}_start.png"), c["start"])
        doc["cameras"][name] = {"source": c.get("source"), "intrinsics": c.get("intrinsics"),
                                "fix": np.asarray(c["fix"]).tolist(), "occ_model": m.occ.to_dict(),
                                "col_model": m.col.to_dict(), "empty": f"{name}_empty.png",
                                "valid": f"{name}_valid.png", "info": c.get("info", {})}
    jp.write_text(json.dumps(doc, indent=2) + "\n")
    return jp


def load_calibration(path: str | Path):
    jp = calib_json_path(path)
    doc = json.loads(jp.read_text())
    spec = BoardSpec.from_dict(doc["board"])
    canon = Canon(spec, doc.get("pps", 64), doc.get("margin_sq", 0.5))
    cams = {}
    for name, c in doc["cameras"].items():
        ref = cv2.imread(str(jp.parent / c["empty"]), cv2.IMREAD_COLOR)
        valid = cv2.imread(str(jp.parent / c["valid"]), cv2.IMREAD_GRAYSCALE)
        if ref is None or valid is None:
            raise FileNotFoundError(f"{jp.parent / c['empty']} (calibration images missing)")
        model = CameraModel(canon, ref, valid, Logistic.from_dict(c["occ_model"]), Logistic.from_dict(c["col_model"]))
        cams[name] = {"model": model, "fix": np.array(c["fix"]), "intrinsics": c.get("intrinsics"),
                      "source": c.get("source"), "info": c.get("info", {})}
    return spec, canon, cams


# ---------------------------------------------------------------------- the sensor
def _logit(p):
    p = np.clip(p, 1e-4, 1 - 1e-4)
    return np.log(p / (1 - p))


class CameraBoardSensor:
    def __init__(self, sources, calibration_path: str | Path = DEFAULT_CALIB, *, staleness_s: float = 30.0,
                 thumb_width: int = 480, intrinsics: dict | None = None, annotate_frames: bool = True):
        self.spec, self.canon, cals = load_calibration(calibration_path)
        self.thumb_width = thumb_width
        self.annotate_frames = annotate_frames
        self.cams = []
        for name, src, spec in resolve_sources(sources):
            if name not in cals:
                raise KeyError(f"camera {name!r} is not in the calibration {calib_json_path(calibration_path)} "
                               f"(calibrated: {sorted(cals)})")
            c = cals[name]
            ipath = (intrinsics or {}).get(name) or c.get("intrinsics")
            intr = load_intrinsics(ipath) if ipath else None
            loc = BoardLocator(self.spec, intr[0] if intr else None, staleness_s=staleness_s, fix=c["fix"])
            self.cams.append({"name": name, "grabber": _Grabber(src), "loc": loc, "model": c["model"], "intr": intr})
        self.last: dict = {}

    # one camera -> per-square arrays
    def _analyse(self, cam, frame, t):
        posed = frame if isinstance(frame, PosedFrame) else None
        frame = undistort(posed.image if posed else frame, cam["intr"])
        view: BoardView = cam["loc"].locate(frame, t)
        if not view.ok and posed is not None and posed.H_img2board is not None:
            view = BoardView(True, np.asarray(posed.H_img2board, float), view.n_markers, view.seen, 0.0, 0.0,
                             posed.quality, posed.cam_pos_mm, view.image_size, "homography from the camera pose",
                             view.markers)
        d = {"view": view, "frame": frame}
        if not view.ok:
            return d
        warp, valid = self.canon.warp(frame, view.H)
        cam_pos = posed.cam_pos_mm if posed is not None and posed.cam_pos_mm is not None else view.cam_pos
        regions: Regions = sample_regions(self.canon, None if cam_pos is None else np.asarray(cam_pos, float))
        p_occ, p_white, vis, feats = cam["model"].predict(warp, valid, regions)
        w = view.quality * regions.view_w * np.clip((vis - 0.6) / 0.4, 0, 1)
        d.update(warp=warp, regions=regions, p_occ=p_occ, p_white=p_white, vis=vis, w=w, feats=feats)
        return d

    def snapshot(self) -> Snapshot:
        t = time.time()
        tm = time.monotonic()
        L_occ = np.zeros(64)
        W_occ = np.zeros(64)
        L_col = np.zeros(64)
        W_col = np.zeros(64)
        frames: dict[str, bytes] = {}
        per = {}
        for cam in self.cams:
            frame = cam["grabber"].grab()
            if frame is None:
                per[cam["name"]] = {"view": BoardView(False, reason="no frame")}
                continue
            d = self._analyse(cam, frame, tm)
            per[cam["name"]] = d
            if "w" in d:
                L_occ += d["w"] * _logit(d["p_occ"])
                W_occ += d["w"]
                wc = d["w"] * d["p_occ"]
                L_col += wc * _logit(d["p_white"])
                W_col += wc
            frames[cam["name"]] = self._thumb(d)
        occ = np.where(W_occ > 1e-6, 1 / (1 + np.exp(-L_occ / np.maximum(W_occ, 1e-9))), 0.5)
        col = np.where(W_col > 1e-6, 1 / (1 + np.exp(-L_col / np.maximum(W_col, 1e-9))), 0.5)
        self.last = {"t": t, "cameras": per, "occ": occ, "col": col, "w_occ": W_occ}
        return Snapshot(t, {s: float(occ[i]) for i, s in enumerate(SQUARES)},
                        {s: float(col[i]) for i, s in enumerate(SQUARES)}, frames)

    # ------------------------------------------------------------------ drawing
    def annotate(self, d) -> np.ndarray:
        img = d["frame"].copy()
        view: BoardView = d["view"]
        for corner, m in view.markers.items():
            cv2.polylines(img, [np.round(m).astype(np.int32)], True, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(img, corner, tuple(int(v) for v in m[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        if view.ok:
            Hi = np.linalg.inv(view.H)
            s = self.spec.square_mm
            for k in range(9):
                for a, b in (((0, k * s), (8 * s, k * s)), ((k * s, 0), (k * s, 8 * s))):
                    p = apply_h(Hi, [a, b])
                    cv2.line(img, tuple(np.round(p[0]).astype(int)), tuple(np.round(p[1]).astype(int)),
                             (255, 200, 0), 1, cv2.LINE_AA)
            if "p_occ" in d:
                for i in range(64):
                    c = apply_h(Hi, [self.spec.square_center(i)])[0]
                    if d["p_occ"][i] > 0.5:
                        col = (255, 255, 255) if d["p_white"][i] > 0.5 else (0, 0, 0)
                        cv2.circle(img, tuple(np.round(c).astype(int)), 6, col, -1, cv2.LINE_AA)
                        cv2.circle(img, tuple(np.round(c).astype(int)), 6, (0, 0, 255), 1, cv2.LINE_AA)
        msg = f"{view.reason}" + (f"  q={view.quality:.2f}" if view.ok else "")
        cv2.putText(img, msg, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(img, msg, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)
        return img

    def _thumb(self, d) -> bytes:
        img = self.annotate(d) if self.annotate_frames else d["frame"]
        h, w = img.shape[:2]
        if w > self.thumb_width:
            img = cv2.resize(img, (self.thumb_width, int(h * self.thumb_width / w)), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return buf.tobytes() if ok else b""

    def board_image(self, pps: int = 48) -> np.ndarray:
        """Summary of the last snapshot: 8x8, grey level = P(occupied), dot colour = P(white)."""
        n = 8 * pps
        img = np.zeros((n, n, 3), np.uint8)
        occ, col = self.last.get("occ", np.full(64, 0.5)), self.last.get("col", np.full(64, 0.5))
        for i in range(64):
            f, r = i % 8, i // 8
            x0, y0 = f * pps, (7 - r) * pps
            img[y0:y0 + pps, x0:x0 + pps] = (150, 180, 200) if (f + r) % 2 else (90, 120, 150)
            if occ[i] > 0.5:
                g = int(255 * col[i])
                cv2.circle(img, (x0 + pps // 2, y0 + pps // 2), int(pps * 0.35), (g, g, g), -1, cv2.LINE_AA)
            cv2.putText(img, f"{occ[i]:.2f}", (x0 + 3, y0 + pps - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
        return img

    def close(self):
        for c in self.cams:
            c["grabber"].close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def ascii_board(snap: Snapshot, thr: float = 0.5) -> str:
    rows = []
    for r in range(7, -1, -1):
        row = f"{r + 1} "
        for f in range(8):
            s = SQUARES[r * 8 + f]
            row += (" " + ("W" if snap.colour[s] > 0.5 else "B")) if snap.occupancy[s] > thr else " ."
        rows.append(row)
    rows.append("   a b c d e f g h")
    return "\n".join(rows)
