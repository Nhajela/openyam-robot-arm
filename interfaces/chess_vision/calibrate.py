"""Record the board-vision calibration: an empty board, then the start position, per camera.

    .venv/bin/python -m interfaces.chess_vision.calibrate --source top=phone:top --source side=phone:side \\
        --out interfaces/chess_vision/calib --square-mm 35 --marker-mm 40 --gap-mm 5

Steps (it prompts for each; ``--yes`` skips the prompts):
  1. Clear the board: only the four corner markers (IDs 10-13) and the empty squares in view.
  2. Set up the start position, white on the a1/h1 (IDs 10/11) side.
It then fits each camera (grid refinement on the empty board, classifiers on the start position),
writes ``chess_vision.json`` + reference images to ``--out``, and a ``<camera>_check.jpg`` per
camera showing what it sees. Re-run whenever a camera, the board or the markers move, or the
lighting changes a lot.

Image-file sources work too (offline / tests): a folder whose sorted images are the empty-board
frame(s) followed by the start-position frame(s), ``--frames`` of each.

Nothing here talks to the arm; it only reads cameras.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2

from .board_geom import BoardSpec, Canon
from .camera import DEFAULT_CALIB, _Grabber, calibrate_camera, load_intrinsics, resolve_sources, save_calibration


def parse_kv(items) -> dict:
    out = {}
    for it in items or []:
        k, v = it.split("=", 1)
        out[k] = v
    return out


def grab_frames(grabbers, n: int, spacing: float) -> dict:
    frames = {name: [] for name, _ in grabbers}
    for k in range(n):
        for name, g in grabbers:
            f = g.grab()
            if f is None:
                raise SystemExit(f"{name}: no frame")
            frames[name].append(f)
        if k + 1 < n and spacing > 0:
            time.sleep(spacing)
    return frames


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", action="append", required=True,
                    help="camera source, optionally named: top=phone:top, side=/dev/video2, a folder ... (repeatable)")
    ap.add_argument("--out", default=str(DEFAULT_CALIB), help="calibration folder (default %(default)s)")
    ap.add_argument("--square-mm", type=float, default=35.0, help="playing square size")
    ap.add_argument("--marker-mm", type=float, default=40.0, help="printed marker size (outer black edge)")
    ap.add_argument("--gap-mm", type=float, default=5.0,
                    help="marker inner corner to playing-area corner, along each edge (rough is fine: refined)")
    ap.add_argument("--ids", default="10,11,12,13", help="marker IDs at the a1,h1,h8,a8 corners")
    ap.add_argument("--frames", type=int, default=None, help="frames averaged per step (default 3 live, 1 files)")
    ap.add_argument("--intrinsics", action="append", help="name=vision/calib/<camera>.yaml (optional, repeatable)")
    ap.add_argument("--pps", type=int, default=64, help="warp pixels per square")
    ap.add_argument("--no-refine", action="store_true", help="skip the empty-board grid refinement")
    ap.add_argument("--yes", action="store_true", help="no prompts (file sources, scripted runs)")
    a = ap.parse_args(argv)

    ids = [int(v) for v in a.ids.split(",")]
    spec = BoardSpec(a.square_mm, a.marker_mm, a.gap_mm, dict(zip(("a1", "h1", "h8", "a8"), ids)))
    canon = Canon(spec, a.pps)
    intr_paths = parse_kv(a.intrinsics)
    srcs = resolve_sources(a.source)
    grabbers = [(name, _Grabber(src)) for name, src, _ in srcs]
    live = any(getattr(src, "live", True) for _, src, _ in srcs)
    n = a.frames or (3 if live else 1)

    def step(msg):
        print(msg)
        if not a.yes:
            input("  ... press Enter when ready ")

    step("1/2  Clear the board: only the 4 corner markers and empty squares. Arm parked out of view.")
    empty = grab_frames(grabbers, n, 0.3 if live else 0)
    step("2/2  Set up the START position, white on the a1/h1 (ID %d/%d) side." % (ids[0], ids[1]))
    start = grab_frames(grabbers, n, 0.3 if live else 0)

    cams, failed = {}, []
    out = Path(a.out)
    for name, src, spec_str in srcs:
        ip = intr_paths.get(name)
        intr = load_intrinsics(ip) if ip else None
        try:
            c = calibrate_camera(name, empty[name], start[name], spec, canon, intr, refine=not a.no_refine)
        except ValueError as e:
            failed.append(name)
            print(f"FAILED {e}", file=sys.stderr)
            continue
        c["source"], c["intrinsics"] = spec_str, ip
        cams[name] = c
        info = c["info"]
        grid = info.get("grid", {})
        print(f"{name}: markers reproj {info['reproj_px']:.2f} px, grid {'refined' if grid.get('found') and 'resid_mm' in grid else 'NOT refined'}"
              f"{' (max shift %.1f mm)' % grid['max_shift_mm'] if 'max_shift_mm' in grid else ''}, "
              f"camera at {info['cam_pos_mm']} mm (lean {info['mean_lean']}), "
              f"start-position fit: occupancy {info['occ_train_acc']:.0%}, colour {info['col_train_acc']:.0%}")
        if info["occ_train_acc"] < 0.95:
            print(f"  warning: {name} cannot separate occupied/empty squares well; move it higher or more overhead",
                  file=sys.stderr)
    for _, g in grabbers:
        g.close()
    if not cams:
        raise SystemExit("no camera calibrated")
    jp = save_calibration(out, spec, canon, cams)
    for name, c in cams.items():
        cv2.imwrite(str(out / f"{name}_check.jpg"), c["start"])
    print("wrote", jp)
    if failed:
        print("not calibrated:", ", ".join(failed), file=sys.stderr)
    return jp


if __name__ == "__main__":
    main()
