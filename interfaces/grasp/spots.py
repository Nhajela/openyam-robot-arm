"""interfaces.grasp.spots: named places on the table, and "grab from A, put on B".

The spots and the objects the arm knows how to hold live in ``workcell.json``:

    "spots":   {"A": {"title": "Spot A (left)", "xyz": [x, y, 0]}, ...}      # TABLE frame, metres
    "objects": {"block": {"title": ..., "height": 0.04, "width": 0.04}, ...}  # metres

Positions are in the table frame (``grasp.Workcell.T_base_table``), which is a placeholder
until the table is calibrated: the spots are taped marks around the table origin, within top-down
reach. ``grab_playable`` plans a ``grasp.pick_place`` between two spots, at "dance" caps when that
keeps it valid and within 60 s, otherwise at "default" caps.

    from interfaces.grasp.spots import list_spots, grab_playable
    p, info = grab_playable("A", "B", obj="block")
"""
from __future__ import annotations

import dataclasses
import json
from pathlib import Path

from armctl import playable as pl

from ..timeline import coach
from .grasp import WORKCELL, Unreachable, Workcell, pick_place

__all__ = ["list_spots", "list_objects", "grab_playable", "Unreachable"]


def _raw(path: Path = WORKCELL) -> dict:
    return json.loads(Path(path).read_text()) if Path(path).exists() else {}


def _workcell(raw: dict) -> Workcell:
    """Workcell from the JSON, ignoring the keys Workcell doesn't know (spots, objects)."""
    known = {f.name for f in dataclasses.fields(Workcell)}
    return Workcell(**{k: v for k, v in raw.items() if k in known})


def _spots(raw: dict) -> dict:
    return raw.get("spots") or {}


def list_spots(path: Path = WORKCELL) -> list[dict]:
    """[{name, title, xyz_table}] from workcell.json "spots", sorted by name."""
    return [{"name": n, "title": s.get("title", f"Spot {n}"), "xyz_table": [float(x) for x in s["xyz"]]}
            for n, s in sorted(_spots(_raw(path)).items())]


def list_objects(path: Path = WORKCELL) -> list[dict]:
    """[{name, title, height, width}] from workcell.json "objects"."""
    return [{"name": n, "title": o.get("title", n), "height": float(o["height"]), "width": float(o["width"])}
            for n, o in sorted((_raw(path).get("objects") or {}).items())]


def grab_playable(src: str, dst: str, obj: str = "block", path: Path = WORKCELL) -> tuple[dict, dict]:
    """Pick ``obj`` up at spot ``src`` and put it down at spot ``dst`` -> (playable, info).

    Raises ValueError for an unknown spot or object or src == dst, and ``grasp.Unreachable``
    (a ValueError) if the arm can't reach a spot with the fingers down. ``info`` is
    pick_place's info plus {src, dst, obj, caps, duration_s}.
    """
    raw = _raw(path)
    spots = _spots(raw)
    objects = raw.get("objects") or {}
    names = ", ".join(sorted(spots)) or "none set up"
    for s in (src, dst):
        if s not in spots:
            raise ValueError(f"unknown spot {s!r} (spots: {names})")
    if src == dst:
        raise ValueError(f"pick and place are both spot {src}: choose two different spots")
    if obj not in objects:
        raise ValueError(f"unknown object {obj!r} (objects: {', '.join(sorted(objects)) or 'none set up'})")
    o = objects[obj]
    wc = _workcell(raw)
    height, width = float(o["height"]), float(o["width"])
    if width + wc.open_margin > wc.max_width:
        raise ValueError(f"{obj} is {width * 100:.1f} cm wide: too wide for the gripper "
                         f"(opens {wc.max_width * 100:.1f} cm, needs {wc.open_margin * 100:.0f} cm spare)")
    name = pl.slugify(f"grab-{obj}-{src}-to-{dst}")
    last = None
    for caps in ("dance", "default"):
        p, info = pick_place(wc, spots[src]["xyz"], spots[dst]["xyz"], height=height, width=width,
                             name=name, caps=caps)
        p["notes"] = f"{o.get('title', obj)} from {spots[src].get('title', src)} to {spots[dst].get('title', dst)}. " \
                     + p.get("notes", "")
        r = coach.explain(p)
        last = r
        if r["ok"] and (r["duration"] or 0) <= pl.MAX_DURATION_S:
            info.update({"src": src, "dst": dst, "obj": obj, "caps": caps, "duration_s": r["duration"]})
            return p, info
    raise ValueError(f"can't plan a safe grab from {src} to {dst}: " + "; ".join((last or {}).get("summary", [])[1:4]))
