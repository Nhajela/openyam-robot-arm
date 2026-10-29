"""interfaces.grasp.place_spots: put the Grab spots and the chess board where they really are.

The spots and the board in ``workcell.json`` are table-frame coordinates that were made up before
the table was calibrated. Since 29 Oct the table frame is real (its origin is the tag sheet, and
its x axis points about 97 deg from the base's), so the old spots land 0.5 m out, past the reach.
Nothing here guesses the table: you show the arm where things are, and this records it.

    # what is reachable now (nothing moves; reads workcell.json only)
    .venv/bin/python -m interfaces.grasp.place_spots check
    # a paper layout every pair can reach (left = the base's +y side); --apply writes it as provisional
    .venv/bin/python -m interfaces.grasp.place_spots suggest --side left [--apply]

    # teach: park the fingertip over a taped mark (open play Joints mode, or the console), then
    .venv/bin/python -m interfaces.grasp.place_spots teach A --title "Spot A (left)"
    # chess: fingertip over the centre of a1, run "board a1"; over h8, run "board h8"; the board's
    # centre, square size and direction come from the two
    .venv/bin/python -m interfaces.grasp.place_spots board a1
    .venv/bin/python -m interfaces.grasp.place_spots board h8

``teach`` and ``board`` read the arm's joint angles from the daemon (hello + one state: no lease,
no motion), take the fingertip's FK (vision-corrected, like the grasp planner), put it into the
table frame and save it into ``workcell.json``. The old file is backed up next to it first.
Then ``check`` runs every spot pair and every board square through the real planner.
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
import time
from pathlib import Path

import numpy as np

from armctl import kin

from .grasp import WORKCELL, Unreachable, Workcell, pick_place

BOARD_TEACH = Path(__file__).with_name(".board_teach.json")   # a1 / h8 between the two runs (local)


def tip_table(q, wc: Workcell) -> np.ndarray:
    """Fingertip position in the table frame for joint angles q."""
    T = np.linalg.inv(np.array(wc.T_base_table, float)) @ kin.fk_vision(q)
    return T[:3, 3]


def _read_q(url: str | None):
    from interfaces.wristcam.engine import read_q_from_daemon
    return np.asarray(read_q_from_daemon(url), float)


def _save(raw: dict, path: Path) -> Path:
    bak = path.with_name(path.stem + time.strftime(".before-spots-%Y%m%d-%H%M%S") + path.suffix)
    shutil.copy2(path, bak)
    path.write_text(json.dumps(raw, indent=2) + "\n")
    return bak


def check(path: Path = WORKCELL, obj: str = "block") -> dict:
    """Every spot pair (pick at one, place at another) and every chess square through pick_place."""
    from .chess_arm import Board, piece_dims
    import chess

    raw = json.loads(path.read_text())
    wc = Workcell.load(path)
    spots = raw.get("spots") or {}
    o = (raw.get("objects") or {}).get(obj, {"height": 0.04, "width": 0.04})
    out = {"calibrated": wc.calibrated, "floor_base_z": round(wc.floor_base(), 4), "spots": {}, "pairs": {}, "board": {}}
    T = np.array(wc.T_base_table, float)
    for name, s in spots.items():
        b = T @ np.r_[np.asarray(s["xyz"], float), 1.0]
        out["spots"][name] = {"table": s["xyz"], "base_r": round(float(np.hypot(b[0], b[1])), 3),
                              "base_j1_deg": round(math.degrees(math.atan2(b[1], b[0])), 1)}
    for a in spots:
        for c in spots:
            if a == c:
                continue
            try:
                pick_place(wc, spots[a]["xyz"], spots[c]["xyz"], height=float(o["height"]), width=float(o["width"]))
                out["pairs"][f"{a}->{c}"] = "ok"
            except Unreachable as e:
                out["pairs"][f"{a}->{c}"] = str(e)
    board = Board.from_workcell(wc)
    pawn = chess.Piece(chess.PAWN, chess.BLACK)
    h, w = piece_dims(board, pawn)
    bad = []
    for sq in chess.SQUARES:
        xy = board.xy(sq)
        try:
            pick_place(wc, [xy[0], xy[1], 0.0], [xy[0], xy[1], 0.0], height=h, width=w)
        except Unreachable as e:
            bad.append(f"{chess.square_name(sq)}: {e}")
    out["board"] = {"center": list(board.center), "square": board.square, "rank_dir_deg": round(math.degrees(board.rank_dir), 1),
                    "reachable_squares": 64 - len(bad), "unreachable": bad[:8]}
    return out


def suggest_spots(side: str = "left", obj: str = "block", r: float = 0.30, apply: bool = False,
                  path: Path = WORKCELL) -> dict:
    """Three spots A/B/C that every pick-and-place pair can reach, on one side of the arm (base
    frame: left = +y). A layout on paper: it knows the arm's reach, not what is on the table. With
    ``apply`` they are written as PROVISIONAL (title says so) until ``teach`` replaces them."""
    raw = json.loads(path.read_text())
    wc = Workcell.load(path)
    if not wc.calibrated:
        raise SystemExit("the table frame isn't calibrated: spots would be made up twice over")
    o = (raw.get("objects") or {}).get(obj, {"height": 0.04, "width": 0.04})
    Ti = np.linalg.inv(np.array(wc.T_base_table, float))
    z_tab = float(np.array(wc.T_base_table, float)[2, 3])
    sign = 1.0 if side == "left" else -1.0
    angles = {"A": 0.30, "B": 0.62, "C": 0.46}          # rad from straight ahead, toward `side`
    radii = {"A": r, "B": r, "C": r - 0.08}
    spots = {}
    for name in ("A", "B", "C"):
        th = sign * angles[name]
        p = Ti @ np.r_[radii[name] * math.cos(th), radii[name] * math.sin(th), z_tab, 1.0]
        spots[name] = {"title": f"Spot {name} ({side}, provisional)", "xyz": [round(float(p[0]), 4), round(float(p[1]), 4), 0.0],
                       "provisional": True, "base_r": radii[name], "base_j1_rad": round(th, 3)}
    pairs = {}
    for a in spots:
        for c in spots:
            if a != c:
                try:
                    pick_place(wc, spots[a]["xyz"], spots[c]["xyz"], height=float(o["height"]), width=float(o["width"]))
                    pairs[f"{a}->{c}"] = "ok"
                except Unreachable as e:
                    pairs[f"{a}->{c}"] = str(e)
    out = {"spots": spots, "pairs": pairs}
    if apply:
        if not all(v == "ok" for v in pairs.values()):
            raise SystemExit("not every pair is reachable: not writing them\n" + json.dumps(pairs, indent=1))
        raw["spots"] = {k: {kk: vv for kk, vv in v.items() if kk not in ("base_r", "base_j1_rad")} for k, v in spots.items()}
        out["backup"] = str(_save(raw, path))
    return out


def teach_spot(name: str, title: str | None, url: str | None = None, path: Path = WORKCELL) -> dict:
    raw = json.loads(path.read_text())
    wc = Workcell.load(path)
    if not wc.calibrated:
        raise SystemExit("the table frame isn't calibrated (workcell.json calibrated: false): spots would be made up")
    q = _read_q(url)
    p = tip_table(q, wc)
    spots = raw.setdefault("spots", {})
    spots[name] = {"title": title or spots.get(name, {}).get("title") or f"Spot {name}",
                   "xyz": [round(float(p[0]), 4), round(float(p[1]), 4), 0.0],
                   "taught": {"q": [round(float(x), 4) for x in q], "tip_above_table_m": round(float(p[2]), 4),
                              "at": time.strftime("%Y-%m-%dT%H:%M:%S%z")}}
    bak = _save(raw, path)
    return {"spot": name, **spots[name], "backup": str(bak)}


def teach_board(corner: str, url: str | None = None, path: Path = WORKCELL) -> dict:
    """a1 and h8 square centres (fingertip over each, one run each) -> board centre, square, direction."""
    if corner not in ("a1", "h8"):
        raise SystemExit("board corner must be a1 or h8")
    wc = Workcell.load(path)
    if not wc.calibrated:
        raise SystemExit("the table frame isn't calibrated (workcell.json calibrated: false)")
    p = tip_table(_read_q(url), wc)[:2]
    seen = json.loads(BOARD_TEACH.read_text()) if BOARD_TEACH.exists() else {}
    seen[corner] = [round(float(p[0]), 4), round(float(p[1]), 4)]
    BOARD_TEACH.write_text(json.dumps(seen))
    if not ("a1" in seen and "h8" in seen):
        return {"taught": corner, "xy": seen[corner], "next": "now put the fingertip over h8" if corner == "a1" else "now a1"}
    a1, h8 = np.array(seen["a1"]), np.array(seen["h8"])
    d = h8 - a1                                   # 7 squares along the rank direction + 7 along the files
    diag = float(np.linalg.norm(d))
    square = diag / (7 * math.sqrt(2))
    # rank direction r and file direction f = (r_y, -r_x): d = 7 s (r + f) -> r is d rotated by +45 deg
    ang = math.atan2(d[1], d[0]) + math.pi / 4
    raw = json.loads(path.read_text())
    raw["chess"] = {**(raw.get("chess") or {}), "center": [round(float(v), 4) for v in (a1 + h8) / 2],
                    "square": round(square, 4), "rank_dir": round(ang, 4)}
    bak = _save(raw, path)
    BOARD_TEACH.unlink(missing_ok=True)
    return {"chess": raw["chess"], "a1": seen["a1"], "h8": seen["h8"], "backup": str(bak),
            "note": "check the square size against a ruler (a printed or real board's square)"}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("check", help="reachability of every spot pair and chess square (nothing moves)")
    c.add_argument("--object", default="block")
    t = sub.add_parser("teach", help="spot NAME = where the fingertip is now (reads q; nothing moves)")
    t.add_argument("name")
    t.add_argument("--title", default=None)
    g = sub.add_parser("suggest", help="three spots every pair can reach, on one side (paper layout; --apply writes them as provisional)")
    g.add_argument("--side", choices=("left", "right"), default="left")
    g.add_argument("--object", default="block")
    g.add_argument("--apply", action="store_true")
    b = sub.add_parser("board", help="chess board from the fingertip over a1, then over h8")
    b.add_argument("corner", choices=("a1", "h8"))
    for p in (t, b):
        p.add_argument("--url", default=None, help="armctl daemon WebSocket (default ws://127.0.0.1:8091/armctl)")
    a = ap.parse_args(argv)
    if a.cmd == "check":
        res = check(obj=a.object)
    elif a.cmd == "suggest":
        res = suggest_spots(a.side, a.object, apply=a.apply)
    elif a.cmd == "teach":
        res = teach_spot(a.name, a.title, a.url)
    else:
        res = teach_board(a.corner, a.url)
    print(json.dumps(res, indent=1, default=float))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
