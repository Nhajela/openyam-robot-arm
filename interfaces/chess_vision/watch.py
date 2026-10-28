"""Debug view of the board sensor, headless: writes annotated images to a folder and prints the board.

    .venv/bin/python -m interfaces.chess_vision.watch --source top=phone:top --source side=phone:side \\
        --calib interfaces/chess_vision/calib --out /tmp/chess_watch --interval 1

Per snapshot it writes (overwriting ``latest_*``; ``--keep`` also keeps numbered copies):
  latest_<camera>.jpg       the camera frame: detected corner markers, the projected 8x8 grid,
                            dots on squares it thinks are occupied (white/black = piece colour)
  latest_<camera>_warp.jpg  that camera's top-down warp with the sampled discs, green = occupied
  latest_board.png          the fused board: P(occupied) per square, dot shade = P(white)
and prints an ASCII board plus the squares that changed since the previous snapshot.
Open the folder in an image viewer that reloads, or ``python -m http.server`` it. Read-only:
cameras only, nothing talks to the arm.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

from .board_geom import SQUARES
from .camera import DEFAULT_CALIB, CameraBoardSensor, ascii_board
from .calibrate import parse_kv


def warp_debug(d) -> np.ndarray:
    img = d["warp"].copy()
    reg = d["regions"]
    for i, (u, v) in enumerate(reg.centers):
        occ = d["p_occ"][i] > 0.5
        cv2.circle(img, (int(round(u)), int(round(v))), int(reg.radius), (0, 255, 0) if occ else (0, 0, 255),
                   2 if occ else 1, cv2.LINE_AA)
        cv2.putText(img, f"{d['p_occ'][i]:.2f}", (int(u) - 14, int(v) + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                    (0, 255, 255), 1, cv2.LINE_AA)
    return img


def changes(prev, snap, thr=0.5) -> list[str]:
    out = []
    for s in SQUARES:
        a, b = prev.occupancy[s], snap.occupancy[s]
        if (a > thr) != (b > thr):
            out.append(f"{s}:{'+' if b > thr else '-'}")
        elif a > thr and b > thr and (prev.colour[s] > 0.5) != (snap.colour[s] > 0.5):
            out.append(f"{s}:{'W' if snap.colour[s] > 0.5 else 'B'}")
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", action="append", required=True, help="as for calibrate (repeatable)")
    ap.add_argument("--calib", default=str(DEFAULT_CALIB))
    ap.add_argument("--out", required=True)
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--count", type=int, default=0, help="stop after N snapshots (0 = run until Ctrl-C)")
    ap.add_argument("--keep", action="store_true", help="also keep numbered copies")
    ap.add_argument("--intrinsics", action="append", help="name=vision/calib/<camera>.yaml")
    ap.add_argument("--json", action="store_true", help="print each snapshot's probabilities as JSON")
    a = ap.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    sensor = CameraBoardSensor(a.source, a.calib, intrinsics=parse_kv(a.intrinsics))
    prev = None
    k = 0
    try:
        while True:
            t0 = time.time()
            snap = sensor.snapshot()
            imgs = {"board.png": sensor.board_image()}
            for name, d in sensor.last["cameras"].items():
                if "frame" in d:
                    imgs[f"{name}.jpg"] = sensor.annotate(d)
                if "warp" in d:
                    imgs[f"{name}_warp.jpg"] = warp_debug(d)
            for fn, img in imgs.items():
                cv2.imwrite(str(out / f"latest_{fn}"), img)
                if a.keep:
                    cv2.imwrite(str(out / f"{k:05d}_{fn}"), img)
            views = {n: d["view"].reason for n, d in sensor.last["cameras"].items()}
            print(time.strftime("%H:%M:%S"), views)
            print(ascii_board(snap))
            if prev is not None:
                print("changed:", " ".join(changes(prev, snap)) or "nothing")
            if a.json:
                print(json.dumps({"t": snap.t, "occupancy": snap.occupancy, "colour": snap.colour}))
            prev = snap
            k += 1
            if a.count and k >= a.count:
                break
            time.sleep(max(0.0, a.interval - (time.time() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        sensor.close()
    return prev


if __name__ == "__main__":
    main()
