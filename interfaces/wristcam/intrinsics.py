"""interfaces.wristcam.intrinsics: lens calibration of the wide-angle wrist camera from ChArUco views.

The board is the taped calibration sheet (``board.py``; or the older vision/ board with
``--board vision``, or a board shown on a laptop screen). Three lens models are fitted and the
one with the lowest reprojection error wins; a model with more parameters must beat a simpler
one by more than 10% (fisheye: 4 coefficients, pinhole: 5, rational: 8):

* ``fisheye``  cv2.fisheye (equidistant, k1..k4): usually right for this ~100 deg lens
* ``rational`` cv2.calibrateCamera + CALIB_RATIONAL_MODEL
* ``pinhole``  cv2.calibrateCamera, k1 k2 p1 p2 k3

CLI (nothing here moves the arm):

    # grab N frames while the board moves in front of the camera (or the arm moves the camera
    # over the taped board: then use handeye.py collect, which saves frames AND joint angles)
    .venv/bin/python -m interfaces.wristcam.intrinsics capture --source dashboard --n 25 --out interfaces/wristcam/data/intr
    # solve from a folder of frames (also accepts a handeye collect folder)
    .venv/bin/python -m interfaces.wristcam.intrinsics solve --dir interfaces/wristcam/data/intr
    # board shown on a laptop screen (board.py --screen prints the square size in mm):
    .venv/bin/python -m interfaces.wristcam.intrinsics solve --dir ... --square-mm 27.43
    #   or give the pixel pitch and the square size in screen pixels:
    .venv/bin/python -m interfaces.wristcam.intrinsics solve --dir ... --screen-pixel-mm 0.1792 --screen-square-px 153

Writes ``interfaces/wristcam/calib/wristcam.json`` (CameraModel) with the coverage report in
``meta``. Calibrate at the stream size you use (1280x720 from the console by default).
"""
from __future__ import annotations

import argparse
import datetime as dt
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from .board import BoardDetector, BoardSpec
from .camera import DEFAULT_INTRINSICS, CameraModel

REPO = Path(__file__).resolve().parents[2]
NPARAMS = {"fisheye": 4, "pinhole": 5, "rational": 8}
IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def board_views(frames, spec: BoardSpec | None = None, mask=None, min_corners: int = 12):
    """frames: iterable of BGR images -> (views [(obj, img)], image size, used indices)."""
    det = BoardDetector(spec)
    views, used, size = [], [], None
    for i, f in enumerate(frames):
        if f is None:
            continue
        size = size or (f.shape[1], f.shape[0])
        d = det.detect(f, mask)
        if d is not None and len(d[0]) >= min_corners:
            views.append((d[0].astype(np.float64), d[1].astype(np.float64)))
            used.append(i)
    return views, size, used


def coverage(views, size, grid=(8, 5)) -> dict:
    """Which image cells and how much of the image radius the board corners reached."""
    w, h = size
    hit = np.zeros(grid[::-1], int)
    rmax = 0.0
    for _, p in views:
        gx = np.clip((p[:, 0] / w * grid[0]).astype(int), 0, grid[0] - 1)
        gy = np.clip((p[:, 1] / h * grid[1]).astype(int), 0, grid[1] - 1)
        np.add.at(hit, (gy, gx), 1)
        rmax = max(rmax, float(np.max(np.hypot(p[:, 0] - w / 2, p[:, 1] - h / 2))))
    ascii_map = "\n".join("".join("#" if c >= 20 else "+" if c > 0 else "." for c in row) for row in hit)
    return {"cells_hit": float((hit > 0).mean()), "radius_reached": rmax / float(np.hypot(w / 2, h / 2)),
            "map": ascii_map, "grid": list(grid), "n_views": len(views)}


def _fit(views, size, model):
    obj = [v[0].reshape(-1, 1, 3) for v in views]
    img = [v[1].reshape(-1, 1, 2) for v in views]
    crit = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 200, 1e-10)
    if model == "fisheye":
        flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
        K0 = np.array([[0.55 * size[0], 0, size[0] / 2], [0, 0.55 * size[0], size[1] / 2], [0, 0, 1]])
        rms, K, D, rv, tv = cv2.fisheye.calibrate(obj, img, size, K0.copy(), np.zeros((4, 1)),
                                                  flags=flags | cv2.fisheye.CALIB_USE_INTRINSIC_GUESS, criteria=crit)
    else:
        flags = cv2.CALIB_RATIONAL_MODEL if model == "rational" else 0
        rms, K, D, rv, tv = cv2.calibrateCamera([o.astype(np.float32) for o in obj],
                                                [i.astype(np.float32) for i in img], size, None, None,
                                                flags=flags, criteria=crit)
    cam = CameraModel(model, K, np.ravel(D), size, float(rms))
    per_view = []
    for (o, p), r, t in zip(views, rv, tv):
        R = cv2.Rodrigues(np.asarray(r, float))[0]
        proj = cam.project(o @ R.T + np.ravel(t))
        per_view.append(float(np.sqrt(np.nanmean(np.sum((proj - p) ** 2, axis=1)))))
    return cam, np.array(per_view)


def calibrate_model(views, size, model: str, reject_factor: float = 3.0) -> CameraModel:
    """Fit one model; drops views worse than ``reject_factor`` x median once."""
    if len(views) < 5:
        raise ValueError(f"need at least 5 board views, have {len(views)}")
    cam, pv = _fit(views, size, model)
    med = float(np.median(pv))
    bad = [i for i, e in enumerate(pv) if e > reject_factor * med and e > 0.5]
    if bad and len(views) - len(bad) >= 6:
        views = [v for i, v in enumerate(views) if i not in bad]
        cam, pv = _fit(views, size, model)
    cam.rms = float(np.sqrt(np.mean(pv ** 2)))
    cam.meta = {"per_view_rms": pv.round(3).tolist(), "dropped_views": bad, "n_views": len(views)}
    return cam


def solve(views, size, models=("fisheye", "rational", "pinhole")) -> tuple[CameraModel, dict]:
    """Fit every model, pick by reprojection error (simpler model wins ties within 10%)."""
    results, errors = {}, {}
    for m in models:
        try:
            results[m] = calibrate_model(views, size, m)
        except cv2.error as e:
            errors[m] = str(e).strip().splitlines()[-1][:200]
    if not results:
        raise RuntimeError(f"no lens model converged: {errors}")
    order = sorted(results, key=lambda m: (NPARAMS[m], results[m].rms))
    best = order[0]
    for m in order[1:]:
        if results[m].rms < 0.9 * results[best].rms:
            best = m
    cam = results[best]
    cam.meta.update({"chosen_by": "lowest reprojection error (more parameters must win by > 10%)",
                     "candidates": {m: round(c.rms, 4) for m, c in results.items()}, "failed": errors,
                     "coverage": {k: v for k, v in coverage(views, size).items() if k != "map"},
                     "calibrated_at": dt.datetime.now().isoformat(timespec="seconds")})
    return cam, {"results": results, "errors": errors, "coverage": coverage(views, size)}


def report(cam: CameraModel, info: dict) -> str:
    cov = info["coverage"]
    L = [f"chosen model: {cam.model}   RMS {cam.rms:.3f} px   (" +
         ", ".join(f"{m} {c.rms:.3f}" for m, c in info["results"].items()) + ")"]
    for m, e in info["errors"].items():
        L.append(f"  {m} failed: {e}")
    K = cam.K
    L.append(f"fx {K[0, 0]:.1f}  fy {K[1, 1]:.1f}  cx {K[0, 2]:.1f}  cy {K[1, 2]:.1f}  D {np.round(cam.D, 4).tolist()}")
    L.append(f"horizontal FOV {cam.hfov_deg():.1f} deg; views {cov['n_views']}; image cells hit "
             f"{cov['cells_hit']:.0%}; reached {cov['radius_reached']:.0%} of the corner radius")
    L.append("coverage (8x5 cells: # many corners, + some, . none):\n" + cov["map"])
    if cam.rms > 1.0:
        L.append("WARNING: RMS > 1 px: blur, a bent/raised sheet or wrong --square-mm; recapture")
    if cov["cells_hit"] < 0.7 or cov["radius_reached"] < 0.8:
        L.append("WARNING: the board never reached the image edges/corners: the lens edges are extrapolated. "
                 "Add views with the board near the edges (aim the camera beside it).")
    return "\n".join(L)


def read_frames(folder) -> list:
    p = Path(folder)
    paths = sorted(q for q in p.iterdir() if q.suffix.lower() in IMAGE_EXTS)
    return [cv2.imread(str(q), cv2.IMREAD_COLOR) for q in paths]


def _spec_from_args(a) -> BoardSpec:
    if a.board == "vision":
        sys.path.insert(0, str(REPO / "vision"))
        import json
        j = json.loads((REPO / "vision/targets/charuco_board.json").read_text())
        spec = BoardSpec(j["dict"], tuple(j["squares"]), j["square_mm"], j["marker_mm"])
    else:
        spec = BoardSpec.load(a.board) if a.board else BoardSpec.load()
    sq = a.square_mm
    if a.screen_pixel_mm and a.screen_square_px:
        sq = a.screen_pixel_mm * a.screen_square_px
    return spec.with_measured(sq, a.marker_mm)


def cmd_capture(a):
    sys.path.insert(0, str(REPO / "vision"))
    from sources import open_source  # noqa: E402
    from calibrate_intrinsics import DiversityGate, rough_pose  # noqa: E402
    spec = _spec_from_args(a)
    det = BoardDetector(spec)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    gate, views, size = DiversityGate(a.min_angle, a.min_shift), [], None
    src = open_source(a.source)
    print(f"capturing {a.n} views from {a.source} into {out} (Ctrl-C to stop early)")
    try:
        last = 0.0
        while len(views) < a.n:
            f = src.read()
            if f is None:
                time.sleep(0.05)
                continue
            size = size or (f.shape[1], f.shape[0])
            d = det.detect(f)
            if d is None or len(d[0]) < a.min_corners or time.time() - last < a.interval:
                continue
            pose = rough_pose(d[0].astype(np.float32), d[1].astype(np.float32), size)
            if not gate.novel(pose):
                continue
            gate.add(pose)
            views.append((d[0], d[1]))
            last = time.time()
            cv2.imwrite(str(out / f"{dt.datetime.now():%Y%m%d-%H%M%S-%f}.png"), f)
            cov = coverage(views, size)
            print(f"  view {len(views)}: {len(d[0])} corners; cells {cov['cells_hit']:.0%}, "
                  f"radius {cov['radius_reached']:.0%}")
    except KeyboardInterrupt:
        pass
    finally:
        src.close()
    if views:
        print(coverage(views, size)["map"])


def cmd_solve(a):
    spec = _spec_from_args(a)
    frames = read_frames(a.dir)
    from .camera import JAW_POLYGONS  # noqa: F401
    views, size, used = board_views(frames, spec, min_corners=a.min_corners)
    print(f"{len(views)} of {len(frames)} frames have >= {a.min_corners} board corners")
    models = ("fisheye", "rational", "pinhole") if a.model == "auto" else (a.model,)
    cam, info = solve(views, size, models)
    cam.meta["board"] = spec.to_dict()
    cam.meta["source_dir"] = str(a.dir)
    print(report(cam, info))
    p = cam.save(a.out)
    print("wrote", p)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("capture", "solve"):
        s = sub.add_parser(name)
        s.add_argument("--board", default=None, help="'vision' for vision/targets/charuco_board.json, or a board json "
                                                     "(default: the wristcam sheet)")
        s.add_argument("--square-mm", type=float, default=None, help="measured printed square size")
        s.add_argument("--marker-mm", type=float, default=None)
        s.add_argument("--screen-pixel-mm", type=float, default=None, help="board on a screen: pixel pitch, mm")
        s.add_argument("--screen-square-px", type=float, default=None, help="board on a screen: square size, px")
        s.add_argument("--min-corners", type=int, default=12)
    c = sub.choices["capture"]
    c.add_argument("--source", default="dashboard", help="vision/sources.py spec (default: the lab console camera)")
    c.add_argument("--n", type=int, default=25)
    c.add_argument("--out", required=True)
    c.add_argument("--interval", type=float, default=0.7)
    c.add_argument("--min-angle", type=float, default=10.0)
    c.add_argument("--min-shift", type=float, default=0.10)
    s = sub.choices["solve"]
    s.add_argument("--dir", required=True)
    s.add_argument("--model", default="auto", choices=("auto", "fisheye", "rational", "pinhole"))
    s.add_argument("--out", default=str(DEFAULT_INTRINSICS))
    a = ap.parse_args(argv)
    {"capture": cmd_capture, "solve": cmd_solve}[a.cmd](a)


if __name__ == "__main__":
    main()
