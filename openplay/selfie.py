"""openplay.selfie: the arm takes a selfie of the player (arcade 🤳 Selfie, PROTOCOL.md "Selfie").

1. **Aim:** ``aim_pose`` turns the wrist camera toward where the player stands
   (``armctl/audience.json``: the people watching, base frame) from the ready pose, by turning
   the base (j1) and tilting the wrist (j4) only, inside the daemon's soft limits. The server
   moves there with ``move_joints`` (the daemon's envelope and speed caps; the operator approves
   it first when auto-play is off).
2. **Snap:** the server takes the freshest full-resolution camera frame and saves it on the
   laptop (``SelfieStore``); the phone gets it, puts its frame and filter on it, offers it to the
   player to save, and uploads that framed copy, which is saved next to the original.

Photos of people stay on the laptop, outside the repo (default ``~/Pictures/openyam-selfies``):
never commit them. ``selfies.jsonl`` in that folder indexes them (who, when, pose, files).
"""
from __future__ import annotations

import json
import math
import os
import re
import secrets
import threading
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
AUDIENCE_FILE = Path(os.environ.get("ARMCTL_AUDIENCE", REPO / "armctl" / "audience.json"))
DEFAULT_DIR = Path(os.environ.get("OPENPLAY_SELFIE_DIR", Path.home() / "Pictures" / "openyam-selfies"))
READY = (0.0, 0.5, 0.8, 0.0, 0.0, 0.0)
J4_RANGE = (-0.6, 0.9)        # rad: tilt the camera between "a bit down" and "well up"
LIMIT_MARGIN = 0.1            # rad inside the daemon's soft limits
MAX_FRAMED_BYTES = 4_000_000
ID_RE = re.compile(r"^[0-9]{8}-[0-9]{6}-[a-f0-9]{6}$")


def audience_eye(where: str | None = None) -> tuple[np.ndarray, str]:
    """(eye xyz in the base frame, label). ``where``: a preset name in audience.json, 'x,y,z', or
    None for the file's default."""
    if where and "," in where:
        xyz = np.array([float(v) for v in where.split(",")], float)
        if xyz.shape != (3,):
            raise ValueError("selfie position must be x,y,z (m, base frame)")
        return xyz, f"at {where} m"
    cfg = json.loads(AUDIENCE_FILE.read_text())
    name = where or cfg.get("default", "standing")
    p = cfg["presets"][name]
    return np.array(p["eye"], float), p.get("label", name)


def aim_pose(eye, limits: dict | None = None, start=READY) -> tuple[list[float], float]:
    """Joint angles that point the camera (it looks along the fingers) at ``eye``, from ``start``
    with only j1 and j4 changed, and the angle (deg) still between the camera axis and the eye.
    ``limits``: the daemon's soft limits {"j1": [lo, hi], ...} (rad)."""
    from armctl import kin

    eye = np.asarray(eye, float)
    lim = limits or {}
    lo1, hi1 = (lim.get("j1") or [-2.6, 3.1])
    lo1, hi1 = lo1 + LIMIT_MARGIN, hi1 - LIMIT_MARGIN
    lo4, hi4 = (lim.get("j4") or [-1.69, 1.57])
    lo4, hi4 = max(J4_RANGE[0], lo4 + LIMIT_MARGIN), min(J4_RANGE[1], hi4 - LIMIT_MARGIN)
    best, best_err = list(start), math.inf

    def err(q):
        d = eye - kin.tip(q)
        a = kin.approach(q)
        c = float(np.dot(a, d) / (np.linalg.norm(d) + 1e-9))
        return math.degrees(math.acos(max(-1.0, min(1.0, c))))

    # coarse grid, then refine around the best (cheap: FK is a handful of 4x4 products)
    for step in (0.05, 0.01):
        if step == 0.05:
            a1, b1, a4, b4 = lo1, hi1, lo4, hi4
        else:
            c1, c4 = best[0], best[3]
            a1, b1, a4, b4 = max(lo1, c1 - 0.06), min(hi1, c1 + 0.06), max(lo4, c4 - 0.06), min(hi4, c4 + 0.06)
        for j1 in np.arange(a1, b1 + 1e-9, step):
            for j4 in np.arange(a4, b4 + 1e-9, step):
                q = list(start)
                q[0], q[3] = float(j1), float(j4)
                e = err(q)
                if e < best_err:
                    best, best_err = q, e
    if not kin.clear_of_table(best):
        raise ValueError("the selfie pose would bring the arm too close to the table")
    return [round(v, 4) for v in best], round(best_err, 1)


def safe_name(name: str) -> str:
    s = re.sub(r"[^A-Za-z0-9]+", "-", name or "").strip("-").lower()
    return s[:24] or "player"


class SelfieStore:
    """Saves selfies on the laptop: ``<id>-<name>.jpg`` (the camera's original, full size) and
    ``<id>-<name>-framed.jpg`` (the phone's framed copy), indexed in ``selfies.jsonl``."""

    def __init__(self, folder: Path | str | None = None):
        self.dir = Path(folder) if folder else DEFAULT_DIR
        self.lock = threading.Lock()
        self.items: dict[str, dict] = {}

    def new(self, jpeg: bytes, pid: str, who: str, q: list | None) -> dict:
        if not jpeg.startswith(b"\xff\xd8"):
            raise ValueError("the camera didn't send a JPEG")
        self.dir.mkdir(parents=True, exist_ok=True)
        sid = time.strftime("%Y%m%d-%H%M%S") + "-" + secrets.token_hex(3)
        base = f"{sid}-{safe_name(who)}"
        (self.dir / f"{base}.jpg").write_bytes(jpeg)
        rec = {"id": sid, "t": round(time.time(), 3), "who": who, "pid": pid, "q": q,
               "original": f"{base}.jpg", "framed": None}
        with self.lock:
            self.items[sid] = rec
        self._index(rec)
        return rec

    def framed(self, sid: str, pid: str, jpeg: bytes) -> dict:
        rec = self.items.get(sid)
        if rec is None or rec["pid"] != pid:
            raise KeyError("no such selfie")
        if not jpeg.startswith(b"\xff\xd8") or len(jpeg) > MAX_FRAMED_BYTES:
            raise ValueError("expected a JPEG under 4 MB")
        name = rec["original"][:-4] + "-framed.jpg"
        (self.dir / name).write_bytes(jpeg)
        rec["framed"] = name
        self._index({"id": sid, "framed": name})
        return rec

    def path(self, sid: str, kind: str = "original") -> Path | None:
        rec = self.items.get(sid) if ID_RE.match(sid or "") else None
        f = rec and rec.get(kind)
        return self.dir / f if f else None

    def files(self) -> list[str]:
        """Saved images in the folder, newest first (the laptop's gallery)."""
        if not self.dir.is_dir():
            return []
        return sorted((p.name for p in self.dir.glob("*.jpg")), reverse=True)

    def _index(self, rec: dict) -> None:
        with self.lock, (self.dir / "selfies.jsonl").open("a", encoding="utf-8") as f:
            f.write(json.dumps({k: v for k, v in rec.items() if k != "pid"}) + "\n")
