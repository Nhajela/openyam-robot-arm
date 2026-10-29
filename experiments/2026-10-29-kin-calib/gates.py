"""Re-run the PIPELINE's hand-eye solve and sanity gates (interfaces.wristcam.handeye) on he1 / he2
with and without a kinematic correction file. Offline: reads frames + samples.json only.

    .venv/bin/python -I experiments/2026-10-29-kin-calib/gates.py --corr armctl/kin_corrections.json \
        [--he1-square 20.24] [--out experiments/2026-10-29-kin-calib/handeye_he2_kincorr.json]

The correction is passed through ARMCTL_KIN_CORRECTIONS (what kin.fk_vision reads), so this is the
same code path the grasp/vision code will use. Writes only the --out file (never the main calib or
interfaces/grasp/workcell.json).
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO))

from armctl import kin  # noqa: E402
from interfaces.wristcam import handeye as he  # noqa: E402
from interfaces.wristcam.board import BoardSpec  # noqa: E402
from interfaces.wristcam.camera import CameraModel  # noqa: E402

PRINT = REPO / "interfaces" / "wristcam" / "print"


def solve(folder, spec, cam, table_z):
    samples = he.load_samples_dir(folder, cam, spec)
    res = he.solve_handeye(samples, cam, spec)
    k, sd = he.estimate_board_scale(samples, cam, res)
    gates = he.sanity_gates(res, table_z=table_z, board_scale=k)
    res.extra.update({"board_scale_fit": {"factor": round(k, 4), "sigma": round(sd, 4)}, "gates": gates,
                      "data": str(folder)})
    return res, gates, k, sd


def show(label, res, gates, k, sd):
    t = res.T_tip_cam[:3, 3] * 1000
    o = res.T_base_table[:3, 3]
    fing = np.degrees(np.arccos(np.clip(-res.T_tip_cam[2, 2], -1, 1)))
    print(f"\n[{label}] {res.method}: {res.rms_px:.2f} px, spread {res.spread_mm:.2f} mm / {res.spread_deg:.2f} deg, "
          f"t_tc ({t[0]:.1f}, {t[1]:.1f}, {t[2]:.1f}) mm, axis {fing:.1f} deg from fingers, table origin "
          f"({o[0]:.3f}, {o[1]:.3f}, {o[2]:.3f}), board scale x{k:.3f}±{sd:.3f}")
    for g in gates:
        print(f"   {'PASS' if g['ok'] else 'FAIL'}  {g['text']}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(REPO / "interfaces" / "wristcam" / "data"))
    ap.add_argument("--corr", required=True, help="kin_corrections.json to evaluate (vs nominal)")
    ap.add_argument("--he1-square", type=float, default=22.9, help="he1 ChArUco square, mm (fitted)")
    ap.add_argument("--he2-table-z", type=float, default=-0.02)
    ap.add_argument("--he1-table-z", type=float, default=0.0)
    ap.add_argument("--out", default=None, help="write the corrected he2 hand-eye here (this experiment folder)")
    a = ap.parse_args()
    cam = CameraModel.load(REPO / "interfaces" / "wristcam" / "calib" / "wristcam.json")
    sets = {"he2": (BoardSpec.load(PRINT / "printed_tags_25mm.json"), a.he2_table_z),
            "he1": (BoardSpec.load(PRINT / "printed_vision_board.json").with_measured(a.he1_square), a.he1_table_z)}
    summary = {}
    best = None
    for mode, env in (("nominal FK", "off"), ("corrected FK", str(Path(a.corr).resolve()))):
        os.environ["ARMCTL_KIN_CORRECTIONS"] = env
        kin.reload_vision_correction()
        for name, (spec, tz) in sets.items():
            res, gates, k, sd = solve(Path(a.data) / name, spec, cam, tz)
            show(f"{name}, {mode}", res, gates, k, sd)
            summary[f"{name} {mode}"] = {"rms_px": round(res.rms_px, 2), "spread_mm": round(res.spread_mm, 2),
                                         "spread_deg": round(res.spread_deg, 3), "board_scale": round(k, 4),
                                         "gates": {g["name"]: g["ok"] for g in gates}}
            if name == "he2" and mode == "corrected FK":
                best = res
    (HERE / "logs" / "gates_summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    if a.out and best is not None:
        best.extra["note"] = ("he2 hand-eye with the kinematic correction (kin_corrections.json); experiment "
                              "2026-10-29-kin-calib. NOT the main calib: copy only after review.")
        p = he.write_handeye(best, a.out)
        print("wrote", p)


if __name__ == "__main__":
    main()
