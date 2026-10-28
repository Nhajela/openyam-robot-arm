"""Printable A4 sheet with the four chess-board corner markers (DICT_4X4_50, IDs 10-13).

    .venv/bin/python -m interfaces.chess_vision.make_markers [--size-mm 40]

Writes interfaces/chess_vision/markers/board_markers_A4.pdf and .png (300 dpi). Print at 100% /
"Actual size" and check the 50 mm ruler. The size you pass to calibrate (--marker-mm) is the outer
edge of the black square.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "vision"))
from make_targets import DPI, PX_PER_MM, dashed_rect, font, page, px, ruler  # noqa: E402

from .board_geom import DEFAULT_IDS  # noqa: E402

PLACES = {"a1": "a1 corner (white's left)", "h1": "h1 corner (white's right)",
          "h8": "h8 corner (black's left)", "a8": "a8 corner (black's right)"}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--size-mm", type=float, default=40.0)
    ap.add_argument("--out", default=str(HERE / "markers"))
    a = ap.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    tag_px = px(a.size_mm)
    exact = tag_px / PX_PER_MM
    im = page()
    d = ImageDraw.Draw(im)
    d.text((px(15), px(8)), "OpenYAM chess board corner markers", font=font(14, True), fill=0)
    d.text((px(15), px(15)), f"DICT_4X4_50, marker = outer black edge = {exact:.2f} mm at 100%. "
           "Cut on the dashed lines, keep the white margin.", font=font(8), fill=0)
    d.text((px(15), px(19.5)), "Tape each diagonally outside its corner, edges parallel to the board, inner corner",
           font=font(8), fill=0)
    d.text((px(15), px(24)), "~5 mm from the squares (pass the measured gap as --gap-mm). See interfaces/chess_vision/README.md.",
           font=font(8), fill=0)
    cell = 80.0
    gx, gy = (210 - 2 * cell) / 2, 35.0
    order = ["a8", "h8", "a1", "h1"]   # laid out as seen from white's side
    d2 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for k, corner in enumerate(order):
        r, c = divmod(k, 2)
        x0, y0 = gx + c * cell, gy + r * (cell + 15)
        dashed_rect(d, (px(x0), px(y0), px(x0 + cell), px(y0 + cell)))
        tx, ty = px(x0 + cell / 2) - tag_px // 2, px(y0 + 10)
        im.paste(Image.fromarray(cv2.aruco.generateImageMarker(d2, DEFAULT_IDS[corner], tag_px, borderBits=1)), (tx, ty))
        ly = ty + tag_px + px(3)
        d.text((px(x0 + cell / 2), ly), f"ID {DEFAULT_IDS[corner]}", font=font(10, True), fill=0, anchor="ma")
        d.text((px(x0 + cell / 2), ly + px(5)), PLACES[corner], font=font(8), fill=0, anchor="ma")
    ruler(d, 15, 284)
    im.save(out / "board_markers_A4.png", dpi=(DPI, DPI))
    im.save(out / "board_markers_A4.pdf", resolution=float(DPI))
    print("wrote", out / "board_markers_A4.pdf", f"({exact:.2f} mm markers)")


if __name__ == "__main__":
    main()
