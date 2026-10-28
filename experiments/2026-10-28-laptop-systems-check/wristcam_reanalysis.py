"""Re-solve the wrist-camera lens per data subset and check it against he1 (offline; nothing moves).

    WRISTCAM_BOARD=interfaces/wristcam/print/printed_vision_board.json \\
        .venv/bin/python experiments/2026-10-28-laptop-systems-check/wristcam_reanalysis.py interfaces/wristcam/data

For each subset (intr1 arm-driven, intr3 hand-held flat, he1, and combinations) and each lens model:
RMS, fx/fy/cx/cy, D, horizontal FOV, and the PnP camera-board distance on the he1 views with the
board's declared square size. Then, per lens, hand-eye on he1 with ``handeye.solve_handeye`` and the
board scale the arm's kinematics imply (``estimate_board_scale``). See LOG.md 28 Oct, 17:00.
"""
import sys
import warnings
from pathlib import Path

import numpy as np

from interfaces.wristcam import handeye as he
from interfaces.wristcam import intrinsics as intr
from interfaces.wristcam.board import BoardSpec, board_pose

warnings.filterwarnings("ignore")
D = Path(sys.argv[1] if len(sys.argv) > 1 else "interfaces/wristcam/data")
spec = BoardSpec.load()
size = (1280, 720)
views = {s: intr.board_views(intr.read_frames(D / s), spec, min_corners=8)[0]
         for s in ("intr1", "intr3", "he1") if (D / s).exists()}
combos = {"intr1": ["intr1"], "intr3": ["intr3"], "he1": ["he1"], "intr1+intr3": ["intr1", "intr3"],
          "intr1+intr3+he1": ["intr1", "intr3", "he1"]}
lenses = {}
for name, subs in combos.items():
    if not all(s in views for s in subs):
        continue
    v = sum((views[s] for s in subs), [])
    cov = intr.coverage(v, size)
    print(f"\n{name}: {len(v)} views, cells {cov['cells_hit']:.0%}, radius {cov['radius_reached']:.0%}")
    for m in ("fisheye", "rational", "pinhole"):
        try:
            c = intr.calibrate_model(v, size, m)
        except Exception as e:  # noqa: BLE001
            print(f"  {m:9s} failed: {str(e).splitlines()[-1][:80]}")
            continue
        d = [np.linalg.norm(T[:3, 3]) for T, _ in (board_pose(c, o, p) for o, p in views.get("he1", [])) if T is not None]
        print(f"  {m:9s} rms {c.rms:.3f} fx {c.K[0, 0]:6.1f} fy {c.K[1, 1]:6.1f} c ({c.K[0, 2]:.0f},{c.K[1, 2]:.0f}) "
              f"D {np.round(c.D[:5], 3).tolist()} hfov {c.hfov_deg():.0f} he1 PnP dist {np.median(d):.2f} m")
        lenses[(name, m)] = c
if (D / "he1" / "samples.json").exists():
    print("\nhand-eye on he1 per lens (declared square", spec.square_mm, "mm):")
    for key in (("intr1", "fisheye"), ("intr1+intr3", "fisheye"), ("intr1", "rational")):
        if key not in lenses:
            continue
        cam = lenses[key]
        s = he.load_samples_dir(D / "he1", cam, spec)
        r = he.solve_handeye(s, cam, spec)
        k, sd = he.estimate_board_scale(s, cam, r)
        print(f"  {key}: {r.method} {r.rms_px:.2f} px, spread {r.spread_mm:.1f} mm/{r.spread_deg:.2f} deg; "
              f"kinematic board scale x{k:.3f}±{sd:.3f} -> square {spec.square_mm * k:.2f} mm")
