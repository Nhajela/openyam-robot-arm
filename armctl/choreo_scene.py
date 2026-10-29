"""armctl.choreo_scene: does a choreography request need a look at the real scene, and the look itself.

Router (per prompt): "wave at the person in front of you", "point at the cup", "look at whoever is
closest", "dance toward the window" refer to the real world, so the visual author ALSO gets a live
wrist-camera snapshot; "happy dance", "act surprised" are self-contained and don't.

    route("point at the cup")      -> {"use_camera": True,  "method": "rules", "reason": "..."}
    route("act surprised")         -> {"use_camera": False, "method": "rules", "reason": "..."}
    route("do the thing", classify=fn) -> rules unsure: a small model call decides (``classify``)

``mode`` overrides: "always" / "never" / "auto" (default). Every decision carries its reason, which
the caller logs.

Snapshot: the latest frame from the lab console (http://127.0.0.1:8090/camera.jpg, the same URL as
vision/sources.py 'dashboard') plus the arm's joint angles from its /api/state at that moment. If the
wrist camera is calibrated (interfaces/wristcam/calib/handeye.json present AND
interfaces/grasp/workcell.json "calibrated": true), a labelled grid is drawn on the frame and every grid
point is mapped to a base-frame direction and a table point, so "point at the cup in cell B2" becomes a
j1 yaw and a reach target. Uncalibrated, only the pose and the camera's nominal viewing direction are
given (approximate). Reading the console is read-only; nothing here moves the arm.
"""
from __future__ import annotations

import json
import math
import os
import re
import time
import urllib.request
from pathlib import Path
from typing import Any, Callable

import numpy as np

REPO = Path(__file__).resolve().parents[1]
CAMERA_URL = os.environ.get("VISION_DASHBOARD_URL", "http://127.0.0.1:8090/camera.jpg")
HANDEYE = REPO / "interfaces/wristcam/calib/handeye.json"
INTRINSICS = REPO / "interfaces/wristcam/calib/wristcam.json"
WORKCELL = REPO / "interfaces/grasp/workcell.json"
MODES = ("auto", "always", "never")

# --- router ---------------------------------------------------------------------------------

_WORLD = [
    (r"\b(in front of|next to|behind|beside|near|nearest|closest|farthest|furthest|over there|right there)\b",
     "spatial reference to the scene"),
    (r"\b(point|look|stare|glance|gesture|nod|wave|reach|turn|face|dance|bow|wink)\w*\s+(at|to|towards?|toward)\s+"
     r"(?!(the )?(music|beat|rhythm|song|camera's)\b)\w+", "directed at something/someone"),
    (r"\b(whoever|whatever|whichever|someone|somebody|anyone|anybody|everyone|everybody)\b",
     "refers to people present"),
    (r"\b(cup|mug|bottle|glass|window|door|wall|laptop|screen|phone|block|piece|ball|box|chair|plant|"
     r"board|chessboard|pen|book|bag|hat|toy|object|thing on|that thing|this thing)s?\b", "names an object"),
    (r"\b(person|people|man|woman|kid|kids|child|children|guy|girl|boy|crowd|visitor|visitors|"
     r"stranger|friend|audience)\b", "refers to a person"),
    (r"\b(see|find|spot|follow|track|count|what('?s| is) (on|in front))\b", "needs perception"),
    (r"\b(left|right) of\b", "relative position"),
]
_SELF = [
    r"\b(dance|wave|nod|bow|shrug|shake|celebrate|cheer|clap|stretch|wiggle|sway|spin|twirl|groove|boogie)\b",
    r"\b(happy|sad|surprised|angry|scared|sleepy|tired|curious|shy|proud|excited|bored|confused|"
    r"relieved|nervous|silly|grumpy|joyful|calm)\b",
    r"\b(act|pretend|be|feel|show|express|imitate|like a|as if)\b",
]


def route_rules(text: str) -> tuple[bool | None, str]:
    """(True/False, reason) when the rules are confident, (None, why) when not."""
    t = " " + re.sub(r"\s+", " ", str(text or "").lower()) + " "
    hits = [why for pat, why in _WORLD if re.search(pat, t)]
    if hits:
        return True, "rules: " + "; ".join(dict.fromkeys(hits))
    if any(re.search(p, t) for p in _SELF):
        return False, "rules: self-contained gesture/emotion, no reference to the scene"
    return None, "rules: no clear cue either way"


def route(text: str, mode: str = "auto", classify: Callable[[str], tuple[bool, str]] | None = None
          ) -> dict[str, Any]:
    """Decide whether to include a live camera snapshot. Never raises."""
    if mode not in MODES:
        raise ValueError(f"camera mode must be one of {MODES}")
    if mode != "auto":
        return {"use_camera": mode == "always", "method": "override", "reason": f"--camera {mode}"}
    yes, why = route_rules(text)
    if yes is not None:
        return {"use_camera": yes, "method": "rules", "reason": why}
    if classify is None:
        return {"use_camera": False, "method": "rules", "reason": why + "; no classifier: default no camera"}
    try:
        yes, cwhy = classify(text)
        return {"use_camera": bool(yes), "method": "model", "reason": f"{why}; model: {cwhy}"}
    except Exception as e:  # a failed classifier must not block authoring
        return {"use_camera": False, "method": "rules",
                "reason": f"{why}; classifier failed ({type(e).__name__}): default no camera"}


ROUTER_SCHEMA = {
    "type": "object",
    "properties": {"needs_camera": {"type": "boolean"}, "reason": {"type": "string"}},
    "required": ["needs_camera", "reason"],
    "additionalProperties": False,
}
ROUTER_SYSTEM = (
    "A desk robot arm turns short English requests into expressive motions. Decide whether doing the "
    "request well needs a live camera snapshot of the real scene around the arm: yes if it refers to "
    "people, objects, places or directions in the room (\"wave at the person in front of you\", \"point at "
    "the cup\", \"look at whoever is closest\", \"dance toward the window\"); no if it is self-contained "
    "(\"happy dance\", \"act surprised then relieved\", \"nod three times\"). Reply with JSON: "
    "needs_camera and a short reason.")


def make_classifier(client: Any, model: str) -> Callable[[str], tuple[bool, str]]:
    """A small structured-output call (a cheap model is plenty)."""
    def classify(text: str) -> tuple[bool, str]:
        resp = client.messages.create(
            model=model, max_tokens=1024, system=ROUTER_SYSTEM,
            messages=[{"role": "user", "content": f"Request: {text!r}"}],
            output_config={"effort": "low", "format": {"type": "json_schema", "schema": ROUTER_SCHEMA}},
        )
        if resp.stop_reason == "refusal":
            raise RuntimeError("router refused")
        d = json.loads(next(b.text for b in resp.content if b.type == "text"))
        return bool(d["needs_camera"]), str(d.get("reason", ""))[:200]
    return classify


# --- snapshot ---------------------------------------------------------------------------------

def _get(url: str, timeout: float) -> bytes:
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.read()


def _state_url(camera_url: str) -> str:
    m = re.match(r"(https?://[^/]+)", camera_url)
    return (m.group(1) if m else "http://127.0.0.1:8090") + "/api/state"


def read_pose(state: dict) -> list[float] | None:
    """Joint angles j1..j6 (rad) from the console's /api/state, or None."""
    motors = state.get("motors") or []
    by = {m.get("name"): m.get("pos") for m in motors}
    q = [by.get(f"joint{i}") for i in range(1, 7)]
    return [float(x) for x in q] if all(isinstance(x, (int, float)) for x in q) else None


def calibration_status() -> tuple[bool, str]:
    if not HANDEYE.exists():
        return False, f"no hand-eye calibration ({HANDEYE.relative_to(REPO)} missing)"
    try:
        wc = json.loads(WORKCELL.read_text())
    except (OSError, ValueError):
        return False, "workcell.json unreadable"
    if not wc.get("calibrated"):
        return False, "workcell.json not calibrated (table frame unknown)"
    return True, "hand-eye + table calibrated"


def grid_map(img: np.ndarray, q, cols: str = "ABCDE", rows: int = 4, calibrated: bool = True
             ) -> tuple[np.ndarray, list[dict[str, Any]]]:
    """Draw a labelled grid on a copy of ``img`` and map each grid point to the base frame:
    direction (unit), j1 yaw that faces it, and where its ray hits the table (if it does)."""
    import cv2

    from interfaces.wristcam import camera as C
    from armctl import kin  # noqa: F401  (cam_pose uses it)
    cam = C.CameraModel.load(INTRINSICS) if INTRINSICS.exists() else C.nominal_camera()
    cam = cam.for_image(img)
    T_tip_cam = C.load_handeye(HANDEYE) if calibrated else None
    T = C.cam_pose(q, T_tip_cam)
    table_z = 0.0
    try:
        table_z = float(np.asarray(json.loads(WORKCELL.read_text())["T_base_table"])[2, 3])
    except Exception:
        pass
    h, w = img.shape[:2]
    out = img.copy()
    pts = []
    for ri in range(rows):
        for ci, cl in enumerate(cols):
            u, v = (ci + 0.5) * w / len(cols), (ri + 0.5) * h / rows
            pts.append((f"{cl}{ri + 1}", u, v))
    rays = cam.rays([[u, v] for _, u, v in pts])
    dirs = rays @ T[:3, :3].T
    hits = C.ray_plane(T[:3, 3], dirs, [0, 0, table_z], [0, 0, 1])
    table = []
    for (lab, u, v), d, hit in zip(pts, dirs, hits):
        cv2.circle(out, (int(u), int(v)), 6, (0, 255, 255), -1)
        cv2.putText(out, lab, (int(u) + 8, int(v) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 4)
        cv2.putText(out, lab, (int(u) + 8, int(v) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
        row = {"cell": lab, "dir": np.round(d, 3).tolist(), "yaw": round(math.atan2(d[1], d[0]), 2)}
        if np.all(np.isfinite(hit)) and np.linalg.norm(hit[:2]) < 2.0:
            row["table_xyz"] = np.round(hit, 3).tolist()
            row["yaw"] = round(math.atan2(hit[1], hit[0]), 2)
        table.append(row)
    return out, table


def snapshot(camera_url: str = CAMERA_URL, timeout: float = 3.0, fetch: Callable[[str, float], bytes] = _get
             ) -> dict[str, Any]:
    """{jpeg: bytes, q, t, calibrated, calib_note, grid: [...], text}. Raises RuntimeError if no frame."""
    import cv2

    t0 = time.time()
    try:
        raw = fetch(camera_url, timeout)
    except Exception as e:
        raise RuntimeError(f"camera snapshot failed: {camera_url}: {e}") from None
    q = None
    try:
        q = read_pose(json.loads(fetch(_state_url(camera_url), timeout)))
    except Exception:
        pass
    img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError("camera snapshot: not a JPEG")
    h, w = img.shape[:2]
    if w > 1280:
        img = cv2.resize(img, (1280, int(h * 1280 / w)))
    ok, note = calibration_status()
    grid: list[dict[str, Any]] = []
    lines = [f"Live wrist-camera frame taken {time.strftime('%H:%M:%S', time.localtime(t0))}."]
    if q is not None:
        lines.append("Arm pose when it was taken (rad): " + json.dumps([round(x, 3) for x in q]) +
                     ". The camera is on the wrist, looking roughly where the fingers point.")
        try:
            img, grid = grid_map(img, q, calibrated=ok)
            qual = "calibrated" if ok else "UNCALIBRATED nominal estimate, +-15 deg"
            lines.append(f"Grid cells drawn on the frame map to the arm's base frame ({qual}; x forward down "
                         "the table, y left toward the audience, z up; j1 = yaw turns the body to face it):")
            for r in grid:
                lines.append(f"  {r['cell']}: face with j1 ~ {r['yaw']:+.2f} rad" +
                             (f", table point {r['table_xyz']} m" if "table_xyz" in r else ", above the table"))
        except Exception as e:
            lines.append(f"(no grid mapping: {type(e).__name__}: {e})")
    else:
        lines.append("Arm pose unknown (console state unavailable): directions are not mapped.")
    lines.append(f"Calibration: {note}.")
    ok_enc, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return {"jpeg": buf.tobytes() if ok_enc else raw, "q": q, "t": t0, "calibrated": ok, "calib_note": note,
            "grid": grid, "text": "\n".join(lines)}
