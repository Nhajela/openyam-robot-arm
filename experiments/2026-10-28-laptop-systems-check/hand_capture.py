#!/usr/bin/env python3
"""Hand-held ChArUco capture for the wrist-camera lens calibration (the arm holds still).

A person moves the printed board under the parked camera. Every --period seconds this grabs a
frame from the lab console (/camera.jpg), detects the board, and saves the frame when the board
is somewhere new (another part of the image, another size or another tilt). It keeps a coverage
map of where board corners have landed (a 6x4 grid over the image) and writes a status file
with the cells still missing, so the person can be told where to move the board next.

    WRISTCAM_BOARD=interfaces/wristcam/print/printed_vision_board.json \
    .venv/bin/python experiments/2026-10-28-laptop-systems-check/hand_capture.py \
        --out interfaces/wristcam/data/intr2 --secs 300

Then: .venv/bin/python -m interfaces.wristcam.intrinsics solve --dir interfaces/wristcam/data/intr2
Nothing here commands the arm.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from interfaces.wristcam.board import BoardDetector  # noqa: E402

GX, GY = 6, 4
NAMES_X = ["far left", "left", "centre-left", "centre-right", "right", "far right"]
NAMES_Y = ["top", "upper middle", "lower middle", "bottom"]


def grab(url: str) -> np.ndarray | None:
    try:
        b = urllib.request.urlopen(url, timeout=2).read()
    except Exception:
        return None
    return cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_COLOR)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--secs", type=float, default=300)
    ap.add_argument("--period", type=float, default=0.5)
    ap.add_argument("--min-corners", type=int, default=10)
    ap.add_argument("--url", default="http://127.0.0.1:8090/camera.jpg")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    det = BoardDetector()
    cover = np.zeros((GY, GX), int)
    saved: list[dict] = []
    status_path = out / "status.json"
    t_end = time.time() + a.secs
    last_seen = 0.0
    while time.time() < t_end:
        img = grab(a.url)
        if img is None:
            time.sleep(a.period)
            continue
        h, w = img.shape[:2]
        r = det.detect(img)
        if r is not None and len(r[2]) >= a.min_corners:
            last_seen = time.time()
            _, px, ids = r
            c = px.mean(0)
            size = float(np.sqrt(cv2.contourArea(cv2.convexHull(px.astype(np.float32)))))
            # tilt proxy: aspect of the corner cloud's principal axes
            ev = np.linalg.eigvalsh(np.cov((px - c).T))
            aspect = float(np.sqrt(ev[-1] / max(ev[0], 1e-9)))
            new = all(np.hypot(*(c - s["c"])) > 0.08 * w or abs(size - s["size"]) > 0.25 * s["size"]
                      or abs(aspect - s["aspect"]) > 0.35 for s in saved)
            if new:
                name = f"hand_{len(saved):02d}.png"
                cv2.imwrite(str(out / name), img)
                saved.append({"file": name, "c": c, "size": size, "aspect": aspect, "n": int(len(ids))})
                gx = np.clip((px[:, 0] / w * GX).astype(int), 0, GX - 1)
                gy = np.clip((px[:, 1] / h * GY).astype(int), 0, GY - 1)
                for x, y in set(zip(gx.tolist(), gy.tolist())):
                    cover[y, x] += 1
        missing = [f"{NAMES_Y[y]} {NAMES_X[x]}" for y in range(GY) for x in range(GX) if cover[y, x] == 0]
        status = {"t": round(time.time(), 1), "saved": len(saved), "board_visible_now": time.time() - last_seen < 1.0,
                  "coverage_pct": round(100 * float((cover > 0).mean()), 0),
                  "grid": ["".join("#" if v >= 3 else ("+" if v else ".") for v in row) for row in cover],
                  "missing": missing}
        status_path.write_text(json.dumps(status, indent=1))
        time.sleep(a.period)
    (out / "samples.json").write_text(json.dumps(
        [{k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in s.items()} for s in saved], indent=1))
    print(f"saved {len(saved)} frames, coverage {status['coverage_pct']}%: {status['grid']}")


if __name__ == "__main__":
    main()
