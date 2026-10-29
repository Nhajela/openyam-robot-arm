"""interfaces.wristcam.viz: a live "what the arm is thinking" page for the look-and-grasp engine.

    # one run, or a folder of runs (picker, newest first)
    .venv/bin/python -m interfaces.wristcam.viz experiments/2026-10-29-first-pick/cup-pick-real/run5
    .venv/bin/python -m interfaces.wristcam.viz experiments/2026-10-29-first-pick --port 8097
    # follow whichever run under DIR is newest (start this, then start look_and_grasp)
    .venv/bin/python -m interfaces.wristcam.viz --latest-under interfaces/wristcam/runs

Then open http://127.0.0.1:8097/. It reads ``run.jsonl`` and ``frames/`` that ``engine.GraspEngine``
writes (``look_and_grasp --simulate/--execute --out DIR``) and polls them every 0.5 s, so it works
during a run and as a replay afterwards. It shows the latest frame with a detection overlay (box,
outline, label, image-centre crosshair and the pixel error line, projected from the logged q and
camera model), the state strip, the estimate and how it converged, the last decision and move, a
top-down table map and the event log.

Read-only: it opens files and nothing else. It never talks to the daemon, DimOS or CAN, binds
127.0.0.1 only, and refuses the console/daemon ports (8090, 8091).
"""
from __future__ import annotations

import argparse
import json
import re
import time
from functools import lru_cache
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

STATES = ["search", "center", "approach", "descend", "close", "lift", "verify", "miss", "place", "ready", "done"]
RESERVED_PORTS = {8090, 8091}         # lab console, armctl daemon
FRAME_RE = re.compile(r"^[\w.-]+\.(jpg|jpeg|png)$")
LIVE_S = 120.0                        # a run with no end event and a log touched this recently is "live"


# ---------------------------------------------------------------------------------------------- runs
def find_runs(root: Path, depth: int = 4) -> list[Path]:
    """Run dirs (containing run.jsonl) at or under root, oldest first (by run.jsonl mtime)."""
    root = Path(root)
    if (root / "run.jsonl").exists():
        return [root]
    out = []
    for p in root.rglob("run.jsonl"):
        rel = p.relative_to(root)
        if len(rel.parts) <= depth + 1 and not any(s.startswith(".") for s in rel.parts):
            out.append(p.parent)
    return sorted(out, key=lambda d: (d / "run.jsonl").stat().st_mtime)


def run_name(root: Path, d: Path) -> str:
    return d.name if d == root else str(d.relative_to(root))


def read_events(path: Path) -> list[dict]:
    """Events from a run.jsonl that may still be being written (a torn last line is skipped)."""
    out = []
    try:
        text = Path(path).read_text()
    except FileNotFoundError:
        return out
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    return out


# ---------------------------------------------------------------------------------------------- overlay
def _fmt(v, nd=3):
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    if isinstance(v, list) and v and all(isinstance(x, (int, float)) for x in v):
        return "(" + ", ".join(f"{x:+.{nd}f}" if isinstance(x, float) else str(x) for x in v) + ")"
    return str(v)


@lru_cache(maxsize=512)
def _image_size(path: str, mtime: float) -> tuple[int, int] | None:
    img = cv2.imread(path, cv2.IMREAD_REDUCED_GRAYSCALE_2)
    if img is None:
        return None
    return img.shape[1] * 2, img.shape[0] * 2


def _camera(start: dict | None):
    """CameraModel for the run: the one logged in the start event, else the repo's calibration."""
    from .camera import CameraModel
    if start and start.get("camera"):
        try:
            return CameraModel.from_dict(start["camera"])
        except Exception:
            pass
    try:
        from .locate import load_view_inputs
        return load_view_inputs()[0]
    except Exception:
        return None


def outline_table(det: dict, width: float | None, n: int = 48) -> np.ndarray | None:
    """Table-frame (N,3) outline of a detection: the rim ring (cup), the blob's rotated
    rectangle, or a circle of the configured width."""
    xyz = det.get("xyz_table")
    if not xyz:
        return None
    x, y, z = (list(xyz) + [0.0, 0.0])[:3]
    size = (det.get("extra") or {}).get("size_m")
    if det.get("kind") == "blob" and size:
        yaw = float(det.get("yaw_table") or 0.0)
        c, s = np.cos(yaw), np.sin(yaw)
        hx, hy = size[0] / 2, size[1] / 2
        pts = []
        for ax, ay in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
            pts.append([x + c * ax - s * ay, y + s * ax + c * ay, z])
        return np.array(pts)
    d = det.get("diameter") or width
    if not d:
        return None
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.c_[x + d / 2 * np.cos(a), y + d / 2 * np.sin(a), np.full(n, z)]


def overlay_for(det: dict | None, q, cam, T_tip_cam, T_base_table, img_size, width=None) -> dict:
    """Pixel-space overlay for one observation: image centre, detection pixel, outline polygon,
    bounding box, label, and any candidate boxes the finder logged."""
    ov = {"size": list(img_size) if img_size else None, "centre": None, "pixel": None,
          "poly": None, "box": None, "label": None, "candidates": []}
    view = None
    if cam is not None and q is not None:
        try:
            from .locate import View
            c = cam.scaled(tuple(img_size)) if img_size and tuple(img_size) != cam.size else cam
            view = View.from_q(q, c, T_tip_cam, T_base_table)
            ov["centre"] = [round(float(v), 1) for v in view.centre_px()]
            if not ov["size"]:
                ov["size"] = list(c.size)
        except Exception:
            view = None
    if det is None:
        return ov
    if det.get("pixel"):
        ov["pixel"] = [round(float(v), 1) for v in det["pixel"]]
    if view is not None:
        P = outline_table(det, width)
        if P is not None:
            uv = view.project_table(P)
            uv = uv[np.isfinite(uv).all(1)]
            if len(uv) >= 3:
                ov["poly"] = uv.round(1).tolist()
                ov["box"] = [round(float(uv[:, 0].min()), 1), round(float(uv[:, 1].min()), 1),
                             round(float(uv[:, 0].max()), 1), round(float(uv[:, 1].max()), 1)]
    if ov["box"] is None and ov["pixel"]:
        u, v = ov["pixel"]
        ov["box"] = [u - 40, v - 40, u + 40, v + 40]
    sig = det.get("sigma_xy")
    w = det.get("diameter") or ((det.get("extra") or {}).get("size_m") or [None])[0]
    lab = det.get("kind") or "object"
    if det.get("id") is not None:
        lab += f" {det['id']}"
    if sig is not None:
        lab += f" σ{sig * 1000:.1f}mm"
    if w:
        lab += f" {w * 1000:.0f}mm"
    ov["label"] = lab
    for cnd in (det.get("extra") or {}).get("candidates") or []:
        if isinstance(cnd, dict) and cnd.get("box"):
            ov["candidates"].append({"box": [float(v) for v in cnd["box"][:4]], "label": str(cnd.get("label", ""))})
    return ov


# ---------------------------------------------------------------------------------------------- summary
def _event_text(e: dict) -> str:
    ev = e.get("event")
    if ev == "start":
        c = e.get("config", {})
        return f"start: {c.get('obj')} {c.get('width')} m wide, expected table {_fmt(c.get('expected_table'))}"
    if ev == "move":
        return f"move: {e.get('why', '')}"
    if ev == "observe":
        d = e.get("det")
        if not d:
            return f"observe {e.get('frame') or ''}: nothing found"
        return (f"observe {e.get('frame') or ''}: {d.get('kind')} at table {_fmt(d.get('xyz_table', [])[:2])} "
                f"σ{(d.get('sigma_xy') or 0) * 1000:.1f}mm, {e.get('px_err') or 0:.0f} px off centre")
    if ev == "decision":
        return "decision: " + decision_text(e)
    if ev == "transition":
        return f"→ {e.get('to')}"
    if ev == "gripper":
        return f"gripper {e.get('target')}: {e.get('why', '')}"
    if ev == "skip_search_view":
        return f"skip search view {_fmt(e.get('xy'))}: {e.get('reason')}"
    if ev == "end":
        return ("end: OK" if e.get("ok") else f"end: FAILED: {e.get('reason')}")
    rest = {k: v for k, v in e.items() if k not in ("t", "state", "event")}
    return f"{ev}: {json.dumps(rest, default=str)[:200]}"


def decision_text(e: dict) -> str:
    parts = [str(e.get("decision"))]
    for k, v in e.items():
        if k in ("t", "state", "event", "decision"):
            continue
        if k in ("px_err",) and isinstance(v, (int, float)):
            parts.append(f"px_err {v:.1f}")
        elif k.endswith("_mm") and isinstance(v, (int, float)):
            parts.append(f"{k} {v:.2f}")
        else:
            parts.append(f"{k} {_fmt(v, 4)}")
    return ", ".join(parts)


def summarize(events: list[dict], run_dir: Path | None = None, with_overlay: bool = True) -> dict:
    """Everything the page draws, from a run's events (and its frames dir for image sizes)."""
    start = next((e for e in events if e.get("event") == "start"), None)
    cfg = (start or {}).get("config", {})
    T_tip_cam = (start or {}).get("T_tip_cam")
    T_bt = (start or {}).get("T_base_table")
    T_bt_np = np.asarray(T_bt, float) if T_bt else np.eye(4)
    cam = _camera(start) if with_overlay else None
    end = next((e for e in reversed(events) if e.get("event") == "end"), None)

    trace, state = [], "search"
    for e in events:
        s = e.get("state")
        if s and (not trace or trace[-1] != s):
            trace.append(s)
        if e.get("event") == "transition":
            state = e.get("to") or state
            if not trace or trace[-1] != state:
                trace.append(state)
        elif s:
            state = s
    if end:
        state = "done" if end.get("ok") else "failed"
        trace = end.get("trace") or trace

    obs, decisions, moves = [], [], []
    est = est_sigma = None
    width = None
    for i, e in enumerate(events):
        ev = e.get("event")
        if ev == "observe":
            d = e.get("det")
            o = {"n": len(obs) + 1, "t": e.get("t"), "state": e.get("state"), "frame": e.get("frame"),
                 "px_err": e.get("px_err"), "change_mm": None, "decision": None, "det": None, "q": e.get("q")}
            if d:
                o["det"] = {"kind": d.get("kind"), "xy_table": (d.get("xyz_table") or [None, None])[:2],
                            "xyz_table": d.get("xyz_table"), "xy_base": (d.get("xyz_base") or [None, None])[:2],
                            "sigma_mm": (d.get("sigma_xy") or 0) * 1000,
                            "width_mm": (d.get("diameter") or ((d.get("extra") or {}).get("size_m") or [None])[0] or 0) * 1000 or None,
                            "pixel": d.get("pixel"), "score": d.get("score"), "extra": d.get("extra")}
            if with_overlay:
                size = None
                if run_dir is not None and e.get("frame"):
                    fp = Path(run_dir) / "frames" / e["frame"]
                    if fp.exists():
                        size = _image_size(str(fp), fp.stat().st_mtime)
                if size is None and cam is not None:
                    size = cam.size
                o["overlay"] = overlay_for(d, e.get("q"), cam, T_tip_cam, T_bt, size, cfg.get("width"))
            obs.append(o)
        elif ev == "decision":
            dd = {"t": e.get("t"), "state": e.get("state"), "decision": e.get("decision"), "text": decision_text(e)}
            decisions.append(dd)
            if obs and obs[-1]["decision"] is None and e.get("state") == obs[-1]["state"]:
                obs[-1]["decision"] = e.get("decision")
                if e.get("change_mm") is not None:
                    obs[-1]["change_mm"] = e.get("change_mm")
            if e.get("est") is not None:
                est = e["est"][:2]
                s = e.get("sigma") if e.get("sigma") is not None else (e.get("sigma_mm") or 0) / 1000 or None
                if s is not None:
                    est_sigma = s
            if e.get("width_mm"):
                width = e["width_mm"] / 1000
        elif ev == "move":
            qf, qt = np.asarray(e.get("q_from") or [], float), np.asarray(e.get("q_to") or [], float)
            dq_j, dq = None, None
            if len(qf) and len(qf) == len(qt):
                dd = np.abs(qt - qf)
                dq_j, dq = int(dd.argmax()) + 1, float(np.degrees((qt - qf)[dd.argmax()]))
            moves.append({"t": e.get("t"), "state": e.get("state"), "why": e.get("why"), "tip_table": e.get("tip_table"),
                          "speed": e.get("speed"), "joint": dq_j, "dq_deg": dq, "index": i})
    if end:
        if end.get("estimate_table"):
            est = end["estimate_table"][:2]
        if end.get("measured_width"):
            width = end["measured_width"]
    if width is None:
        for o in reversed(obs):
            if o["det"] and o["det"]["width_mm"]:
                width = o["det"]["width_mm"] / 1000
                break
    if est_sigma is None:
        for o in reversed(obs):
            if o["det"]:
                est_sigma = o["det"]["sigma_mm"] / 1000
                break
    est_base = None
    if est is not None:
        z = next((o["det"]["xyz_table"][2] for o in reversed(obs) if o["det"] and o["det"].get("xyz_table")), 0.0)
        est_base = (T_bt_np @ np.array([est[0], est[1], z, 1.0]))[:3].round(5).tolist()

    last_move = moves[-1] if moves else None
    pending = bool(last_move and end is None and last_move["index"] == max(
        (i for i, e in enumerate(events) if e.get("event") in ("move", "observe")), default=-1))
    exp = cfg.get("expected_table")
    spots = [[exp[0] + dx, exp[1] + dy] for dx, dy in (cfg.get("search_offsets") or [])] if exp else []
    skipped = [e.get("xy") for e in events if e.get("event") == "skip_search_view"]
    base_in_table = None
    if T_bt:
        Ti = np.linalg.inv(T_bt_np)
        base_in_table = Ti[:3, 3].round(4).tolist()
    board = None
    meta_board = ((start or {}).get("camera") or {}).get("meta", {}).get("board")
    if meta_board and meta_board.get("squares") and meta_board.get("square_mm"):
        sq = meta_board["square_mm"] / 1000
        board = [meta_board["squares"][0] * sq, meta_board["squares"][1] * sq]

    return {
        "obj": cfg.get("obj"), "config": cfg, "state": state, "trace": trace, "states": STATES,
        "ended": end is not None, "ok": (end or {}).get("ok"), "reason": (end or {}).get("reason"),
        "t": events[-1].get("t") if events else None, "n_events": len(events),
        "estimate": {"table": est, "base": est_base, "sigma_mm": est_sigma * 1000 if est_sigma else None,
                     "width_mm": width * 1000 if width else None, "expected_width_mm": (cfg.get("width") or 0) * 1000 or None},
        "observations": obs,
        "last_decision": decisions[-1] if decisions else None,
        "last_move": last_move, "move_pending": pending,
        "map": {"expected": exp, "search_spots": spots, "skipped": skipped, "place": cfg.get("place_table"),
                "detections": [{"n": o["n"], "xy": o["det"]["xy_table"], "sigma_mm": o["det"]["sigma_mm"],
                                "state": o["state"]} for o in obs if o["det"]],
                "estimate": est, "width": width or cfg.get("width"),
                "tip_path": [m["tip_table"] for m in moves if m.get("tip_table")],
                "base": base_in_table, "board": board},
        "tolerances": {"px": cfg.get("center_px_tol"), "change_mm": (cfg.get("change_tol") or 0) * 1000 or None},
        "events": [{"t": e.get("t"), "state": e.get("state"), "event": e.get("event"), "text": _event_text(e)}
                   for e in events],
    }


# ---------------------------------------------------------------------------------------------- server
class Viewer:
    def __init__(self, root: Path, follow: bool = False):
        self.root = Path(root).resolve()
        self.follow = follow
        self._cache: dict[str, tuple[tuple, dict]] = {}

    def runs(self) -> list[dict]:
        out = []
        for d in find_runs(self.root):
            st = (d / "run.jsonl").stat()
            out.append({"name": run_name(self.root, d), "mtime": st.st_mtime})
        return out[::-1]          # newest first

    def run_dir(self, name: str | None) -> Path | None:
        dirs = find_runs(self.root)
        if not dirs:
            return None
        if not name:
            return dirs[-1]
        for d in dirs:
            if run_name(self.root, d) == name:
                return d
        return None

    def summary(self, d: Path) -> dict:
        p = d / "run.jsonl"
        st = p.stat()
        key = (st.st_mtime_ns, st.st_size)
        hit = self._cache.get(str(d))
        if hit and hit[0] == key:
            s = hit[1]
        else:
            s = summarize(read_events(p), d)
            s["version"] = f"{st.st_mtime_ns}-{st.st_size}"
            self._cache[str(d)] = (key, s)
        s["live"] = (not s["ended"]) and (time.time() - st.st_mtime) < LIVE_S
        s["name"] = run_name(self.root, d)
        return s


def make_handler(viewer: Viewer):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body: bytes, ctype: str):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj, default=float).encode(), "application/json")

        def do_GET(self):
            u = urlparse(self.path)
            qs = {k: v[0] for k, v in parse_qs(u.query).items()}
            if u.path == "/":
                return self._send(200, PAGE.replace("__FOLLOW__", "true" if viewer.follow else "false").encode(),
                                  "text/html; charset=utf-8")
            if u.path == "/api/runs":
                return self._json({"root": str(viewer.root), "follow": viewer.follow, "runs": viewer.runs()})
            if u.path == "/api/run":
                d = viewer.run_dir(qs.get("run"))
                if d is None:
                    return self._json({"error": "no such run"}, 404)
                s = viewer.summary(d)
                if qs.get("v") and qs["v"] == s.get("version"):
                    return self._json({"unchanged": True, "version": s["version"], "live": s["live"], "name": s["name"]})
                return self._json(s)
            if u.path == "/frame":
                d = viewer.run_dir(qs.get("run"))
                f = qs.get("f", "")
                if d is None or not FRAME_RE.match(f):
                    return self._send(404, b"not found", "text/plain")
                fp = d / "frames" / f
                if not fp.is_file():
                    return self._send(404, b"not found", "text/plain")
                return self._send(200, fp.read_bytes(), "image/png" if f.endswith(".png") else "image/jpeg")
            self._send(404, b"not found", "text/plain")

    return H


def serve(root: Path, port: int = 8097, follow: bool = False) -> ThreadingHTTPServer:
    if port in RESERVED_PORTS:
        raise SystemExit(f"port {port} is the lab console/armctl daemon's: pick another (default 8097)")
    srv = ThreadingHTTPServer(("127.0.0.1", port), make_handler(Viewer(root, follow)))
    srv.daemon_threads = True
    return srv


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir", nargs="?", help="a run dir (has run.jsonl) or a folder of runs")
    ap.add_argument("--latest-under", metavar="DIR", help="follow whichever run under DIR is newest")
    ap.add_argument("--port", type=int, default=8097)
    a = ap.parse_args(argv)
    root = a.latest_under or a.run_dir
    if not root:
        ap.error("give a run dir or --latest-under DIR")
    root = Path(root)
    if not root.is_dir():
        if a.latest_under:
            root.mkdir(parents=True, exist_ok=True)
        else:
            ap.error(f"{root} is not a directory")
    srv = serve(root, a.port, follow=bool(a.latest_under))
    n = len(find_runs(root))
    print(f"wristcam viz: {root} ({n} run{'s' if n != 1 else ''}) on http://127.0.0.1:{a.port}/  (Ctrl-C to stop)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Wristcam Pick Viewer</title>
<style>
:root{--bg:#0e1116;--panel:#161b22;--line:#2a313c;--ink:#e6edf3;--ink2:#9aa7b4;--ink3:#6b7785;
 --acc:#3fb6ff;--det:#3ddc84;--warn:#ffcc4d;--bad:#ff6b6b;--est:#ff9f43;--place:#c084fc;--tip:#7aa2ff}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);font:13px/1.4 ui-sans-serif,system-ui,-apple-system,"Segoe UI",sans-serif}
header{display:flex;flex-wrap:wrap;gap:12px;align-items:center;padding:10px 16px;border-bottom:1px solid var(--line)}
header h1{font-size:15px;margin:0;font-weight:600}
select,button{background:var(--panel);color:var(--ink);border:1px solid var(--line);border-radius:6px;padding:4px 8px;font:inherit}
button:hover{border-color:var(--acc)}
.pill{padding:2px 8px;border-radius:10px;font-size:12px;border:1px solid var(--line);color:var(--ink2)}
.pill.live{color:#000;background:var(--det);border-color:var(--det)}
.pill.ok{color:#000;background:var(--det)} .pill.fail{color:#000;background:var(--bad)}
main{display:grid;grid-template-columns:minmax(0,1.7fr) minmax(320px,1fr);gap:12px;padding:12px 16px}
@media (max-width:900px){main{grid-template-columns:1fr}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:10px 12px;min-width:0}
.card h2{font-size:11px;letter-spacing:.08em;text-transform:uppercase;color:var(--ink3);margin:0 0 8px;font-weight:600}
.col{display:flex;flex-direction:column;gap:12px;min-width:0}
#strip{display:flex;flex-wrap:wrap;gap:4px}
#strip span{padding:4px 9px;border-radius:5px;border:1px solid var(--line);color:var(--ink3);font-size:12px}
#strip span.seen{color:var(--ink2);border-color:#3b4553}
#strip span.cur{background:var(--acc);color:#000;border-color:var(--acc);font-weight:600}
#strip span.cur.fail{background:var(--bad);border-color:var(--bad)}
#strip span.cur.done{background:var(--det);border-color:var(--det)}
.framewrap{position:relative;width:100%;background:#000;border-radius:6px;overflow:hidden;aspect-ratio:16/9}
.framewrap img,.framewrap svg{position:absolute;inset:0;width:100%;height:100%}
.framewrap img{object-fit:contain}
.framebar{display:flex;gap:8px;align-items:center;margin-top:8px;flex-wrap:wrap;color:var(--ink2)}
.kv{display:grid;grid-template-columns:auto 1fr;gap:3px 12px;font-variant-numeric:tabular-nums}
.kv b{color:var(--ink3);font-weight:500}
.mono{font-family:ui-monospace,SFMono-Regular,Menlo,monospace;font-size:12px}
#log{height:220px;overflow:auto;font-family:ui-monospace,SFMono-Regular,Menlo,monospace;font-size:12px}
#log div{padding:1px 0;border-bottom:1px solid #1d232c;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
#log .t{color:var(--ink3);display:inline-block;width:56px}
#log .s{color:var(--acc);display:inline-block;width:70px}
#log .decision{color:var(--warn)} #log .end{color:var(--bad)} #log .observe{color:var(--det)}
.charts{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.chart svg{width:100%;height:120px;display:block}
.chart .cap{color:var(--ink2);font-size:12px}
.reason{font-size:14px;margin:2px 0 6px}
.muted{color:var(--ink3)}
#map svg{width:100%;height:auto;display:block}
.legend{display:flex;flex-wrap:wrap;gap:10px;color:var(--ink2);font-size:12px;margin-top:6px}
.legend i{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:4px;vertical-align:-1px}
</style></head><body>
<header>
 <h1>Wristcam pick: what the arm is thinking</h1>
 <select id="runsel" title="run"></select>
 <label class="muted"><input type="checkbox" id="follow"> follow newest run</label>
 <span id="status" class="pill">loading</span>
 <span id="clock" class="muted mono"></span>
</header>
<main>
 <div class="col">
  <div class="card"><div id="strip"></div></div>
  <div class="card">
   <div class="framewrap"><img id="frame" alt=""><svg id="ov" viewBox="0 0 1280 720" preserveAspectRatio="xMidYMid meet"></svg></div>
   <div class="framebar">
    <button id="prev">&larr;</button><button id="next">&rarr;</button><button id="latest">latest</button>
    <label><input type="checkbox" id="ovon" checked> overlay</label>
    <span id="frameinfo" class="mono"></span>
   </div>
  </div>
  <div class="card"><h2>Event log</h2><div id="log"></div></div>
 </div>
 <div class="col">
  <div class="card"><h2>Last decision</h2><div id="dec" class="reason"></div><div id="endreason" class="muted"></div></div>
  <div class="card"><h2>Estimate</h2><div id="est" class="kv"></div></div>
  <div class="card"><h2>Convergence</h2><div class="charts">
   <div class="chart"><div class="cap">px error from image centre</div><svg id="c_px"></svg></div>
   <div class="chart"><div class="cap">estimate change, mm</div><svg id="c_ch"></svg></div></div></div>
  <div class="card"><h2 id="movetitle">Last move</h2><div id="move" class="kv"></div></div>
  <div class="card" id="map"><h2>Table map (table frame, m)</h2><svg id="mapsvg"></svg>
   <div class="legend"><span><i style="background:var(--ink3)"></i>expected / search views</span>
   <span><i style="background:var(--det)"></i>detections (size = &sigma;)</span><span><i style="background:var(--est)"></i>estimate</span>
   <span><i style="background:var(--place)"></i>place</span><span><i style="background:var(--tip)"></i>tip path</span></div></div>
 </div>
</main>
<script>
const FOLLOW_DEFAULT = __FOLLOW__;
let cur = null, ver = null, S = null, sel = null /* selected obs index, null = latest */, runsKnown = [];
const $ = id => document.getElementById(id);
const esc = s => String(s ?? "").replace(/[&<>"]/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[c]));
const f = (v, n=3) => v == null || !isFinite(v) ? "–" : Number(v).toFixed(n);
const xy = (a, n=3) => a ? `(${f(a[0],n)}, ${f(a[1],n)})` : "–";
$("follow").checked = FOLLOW_DEFAULT;

async function loadRuns(){
  const r = await (await fetch("/api/runs")).json();
  const names = r.runs.map(x => x.name);
  if (names.join("|") !== runsKnown.join("|")){
    runsKnown = names;
    $("runsel").innerHTML = r.runs.map(x => `<option>${esc(x.name)}</option>`).join("");
    $("runsel").style.display = names.length > 1 ? "" : "none";
  }
  if (!cur || ($("follow").checked && names[0] && names[0] !== cur)) { setRun(names[0] || null); }
  $("runsel").value = cur;
}
function setRun(name){ cur = name; ver = null; sel = null; }
$("runsel").onchange = e => { $("follow").checked = false; setRun(e.target.value); tick(); };

async function tick(){
  if (!cur) { $("status").textContent = "no runs yet"; return; }
  try{
    const r = await (await fetch(`/api/run?run=${encodeURIComponent(cur)}` + (ver ? `&v=${ver}` : ""))).json();
    if (r.error) { $("status").textContent = r.error; return; }
    if (!r.unchanged){ S = r; ver = r.version; render(); }
    else if (S) { S.live = r.live; status(); }
  } catch(e){ $("status").textContent = "viewer offline"; $("status").className = "pill fail"; }
}
function status(){
  const st = $("status");
  if (S.ended){ st.textContent = S.ok ? "done: OK" : "FAILED"; st.className = "pill " + (S.ok ? "ok" : "fail"); }
  else if (S.live){ st.textContent = "LIVE"; st.className = "pill live"; }
  else { st.textContent = "stopped (no end event)"; st.className = "pill"; }
  $("clock").textContent = `t = ${f(S.t,1)} s, ${S.n_events} events`;
}

function render(){
  status();
  // state strip
  const states = S.states.slice(); if (S.state === "failed") states.push("failed");
  $("strip").innerHTML = states.map(s => {
    let c = S.trace.includes(s) ? "seen" : "";
    if (s === S.state) c += " cur" + (s === "failed" ? " fail" : s === "done" ? " done" : "");
    return `<span class="${c}">${s}</span>`; }).join("");
  renderFrame();
  // decision
  const d = S.last_decision;
  $("dec").innerHTML = d ? `<b>${esc(d.decision)}</b> <span class="muted">(${esc(d.state)}, t ${f(d.t,1)} s)</span><div class="mono muted">${esc(d.text)}</div>` : "<span class='muted'>none yet</span>";
  $("endreason").textContent = S.ended ? (S.ok ? "run finished OK" : `failed: ${S.reason}`) : "";
  // estimate
  const E = S.estimate;
  $("est").innerHTML = `<b>object</b><span>${esc(S.obj)}</span><b>table xy</b><span class="mono">${xy(E.table, 4)} m</span>
    <b>base xyz</b><span class="mono">${E.base ? `(${E.base.map(v=>f(v,4)).join(", ")})` : "–"} m</span>
    <b>&sigma;</b><span>${f(E.sigma_mm,2)} mm</span>
    <b>width</b><span>${f(E.width_mm,1)} mm measured <span class="muted">(${f(E.expected_width_mm,0)} mm expected)</span></span>
    <b>observations</b><span>${S.observations.length} (${S.observations.filter(o=>o.det).length} with a detection)</span>`;
  chart("c_px", S.observations.map(o => o.px_err), S.tolerances.px, "px");
  chart("c_ch", S.observations.map(o => o.change_mm), S.tolerances.change_mm, "mm");
  // move
  const m = S.last_move;
  $("movetitle").textContent = S.move_pending ? "Moving now" : "Last move";
  $("move").innerHTML = m ? `<b>why</b><span>${esc(m.why)}</span><b>state</b><span>${esc(m.state)} (t ${f(m.t,1)} s)</span>
    <b>tip target</b><span class="mono">${m.tip_table ? `(${m.tip_table.map(v=>f(v,3)).join(", ")})` : "–"} m table</span>
    <b>biggest joint</b><span>${m.joint ? `j${m.joint} ${m.dq_deg >= 0 ? "+" : ""}${f(m.dq_deg,1)}°` : "–"}</span>
    <b>speed</b><span>${f(m.speed,2)} rad/s</span>` : "<span class='muted'>none yet</span>";
  renderMap();
  // log
  const lg = $("log"), atBottom = lg.scrollTop + lg.clientHeight >= lg.scrollHeight - 30;
  lg.innerHTML = S.events.map(e => `<div class="${esc(e.event)}" title="${esc(e.text)}"><span class="t">${f(e.t,1)}</span><span class="s">${esc(e.state)}</span>${esc(e.text)}</div>`).join("");
  if (atBottom || S.live) lg.scrollTop = lg.scrollHeight;
}

function obsIndex(){ const n = S.observations.length; return sel == null ? n - 1 : Math.min(sel, n - 1); }
function renderFrame(){
  const n = S.observations.length, i = obsIndex(), ov = $("ov");
  if (i < 0){ $("frame").removeAttribute("src"); ov.innerHTML = ""; $("frameinfo").textContent = "no observation yet"; return; }
  const o = S.observations[i];
  if (o.frame){ const src = `/frame?run=${encodeURIComponent(cur)}&f=${encodeURIComponent(o.frame)}`; if ($("frame").getAttribute("src") !== src) $("frame").src = src; }
  $("frameinfo").textContent = `obs ${o.n}/${n}  ${o.frame || "(no frame)"}  ${o.decision ? "→ " + o.decision : ""}${sel == null ? "  [latest]" : ""}`;
  ov.innerHTML = $("ovon").checked ? overlaySvg(o) : "";
}
function overlaySvg(o){
  const O = o.overlay || {}, W = (O.size || [1280,720])[0], H = (O.size || [1280,720])[1], k = W / 1280;
  $("ov").setAttribute("viewBox", `0 0 ${W} ${H}`);
  let s = "";
  const fs = 22 * k, sw = 3 * k;
  for (const c of O.candidates || []){ const [a,b,c2,d] = c.box;
    s += `<rect x="${a}" y="${b}" width="${c2-a}" height="${d-b}" fill="none" stroke="#fff" stroke-opacity=".45" stroke-dasharray="${8*k} ${6*k}" stroke-width="${2*k}"/>`;
    if (c.label) s += `<text x="${a}" y="${b - 6*k}" fill="#fff" fill-opacity=".6" font-size="${16*k}">${esc(c.label)}</text>`; }
  if (O.centre){ const [cx, cy] = O.centre, r = 26 * k;
    s += `<g stroke="#3fb6ff" stroke-width="${2*k}"><line x1="${cx-r}" y1="${cy}" x2="${cx+r}" y2="${cy}"/><line x1="${cx}" y1="${cy-r}" x2="${cx}" y2="${cy+r}"/><circle cx="${cx}" cy="${cy}" r="${r*0.45}" fill="none"/></g>`; }
  if (O.poly) s += `<polygon points="${O.poly.map(p=>p.join(",")).join(" ")}" fill="#3ddc84" fill-opacity=".12" stroke="#3ddc84" stroke-width="${2*k}"/>`;
  if (O.box){ const [a,b,c,d] = O.box, w = c-a, h = d-b, L = Math.min(w, h) * 0.25;
    s += `<rect x="${a}" y="${b}" width="${w}" height="${h}" fill="none" stroke="#3ddc84" stroke-width="${1.5*k}" stroke-opacity=".7"/>`;
    s += `<path d="M${a},${b+L}V${b}H${a+L} M${c-L},${b}H${c}V${b+L} M${c},${d-L}V${d}H${c-L} M${a+L},${d}H${a}V${d-L}" fill="none" stroke="#3ddc84" stroke-width="${5*k}" stroke-linecap="square"/>`;
    if (O.label){ const tw = O.label.length * fs * 0.56 + 14 * k, ty = Math.max(b - fs - 10*k, 0);
      s += `<rect x="${a - 2.5*k}" y="${ty}" width="${tw}" height="${fs + 10*k}" fill="#3ddc84"/>`;
      s += `<text x="${a + 5*k}" y="${ty + fs + 1*k}" fill="#000" font-size="${fs}" font-weight="700" font-family="ui-monospace,Menlo,monospace">${esc(O.label)}</text>`; } }
  if (O.pixel){ const [u, v] = O.pixel;
    if (O.centre){ const [cx, cy] = O.centre, mx = (u+cx)/2, my = (v+cy)/2;
      s += `<line x1="${cx}" y1="${cy}" x2="${u}" y2="${v}" stroke="#ffcc4d" stroke-width="${2.5*k}" stroke-dasharray="${7*k} ${5*k}"/>`;
      s += `<text x="${mx + 10*k}" y="${my - 8*k}" fill="#ffcc4d" font-size="${18*k}" font-weight="700" stroke="#000" stroke-width="${4*k}" paint-order="stroke">${f(o.px_err,0)} px</text>`; }
    s += `<circle cx="${u}" cy="${v}" r="${6*k}" fill="#ffcc4d" stroke="#000" stroke-width="${2*k}"/>`; }
  if (!o.det) s += `<text x="${W/2}" y="${H/2 + 70*k}" text-anchor="middle" fill="#ff6b6b" font-size="${30*k}" font-weight="700" stroke="#000" stroke-width="${5*k}" paint-order="stroke">NO DETECTION</text>`;
  // HUD bottom-left (the saved frame already carries its own label top-left)
  const hud = [`${(o.state || "").toUpperCase()}  obs #${o.n}  t ${f(o.t,1)} s`,
               o.det ? `table ${xy(o.det.xy_table)}  σ ${f(o.det.sigma_mm,1)} mm` : "nothing found",
               o.decision ? `decision: ${o.decision}` : ""].filter(Boolean);
  const hh = hud.length * 26 * k + 16 * k;
  s += `<rect x="${12*k}" y="${H - hh - 12*k}" width="${470*k}" height="${hh}" rx="${6*k}" fill="#000" fill-opacity=".6"/>`;
  hud.forEach((t, j) => s += `<text x="${24*k}" y="${H - hh - 12*k + 30*k + j*26*k}" fill="${j ? "#e6edf3" : "#3fb6ff"}" font-size="${20*k}" font-weight="${j ? 400 : 700}" font-family="ui-monospace,Menlo,monospace">${esc(t)}</text>`);
  return s;
}
$("prev").onclick = () => { if (!S) return; sel = Math.max(0, obsIndex() - 1); renderFrame(); };
$("next").onclick = () => { if (!S) return; const i = obsIndex() + 1; sel = i >= S.observations.length - 1 ? null : i; renderFrame(); };
$("latest").onclick = () => { sel = null; if (S) renderFrame(); };
$("ovon").onchange = () => S && renderFrame();

function chart(id, vals, tol, unit){
  const el = $(id), W = 300, H = 120, pl = 34, pr = 8, pt = 8, pb = 18;
  el.setAttribute("viewBox", `0 0 ${W} ${H}`);
  const pts = vals.map((v, i) => [i, v]).filter(p => p[1] != null && isFinite(p[1]));
  const n = Math.max(vals.length, 2), ymax = Math.max(1e-9, tol || 0, ...pts.map(p => p[1])) * 1.15;
  const X = i => pl + (W - pl - pr) * (n === 1 ? 0.5 : i / (n - 1)), Y = v => pt + (H - pt - pb) * (1 - v / ymax);
  let s = `<line x1="${pl}" y1="${Y(0)}" x2="${W-pr}" y2="${Y(0)}" stroke="#2a313c"/>`;
  s += `<text x="${pl-4}" y="${Y(ymax/1.15)+4}" fill="#6b7785" font-size="10" text-anchor="end">${f(ymax/1.15, ymax > 10 ? 0 : 1)}</text>`;
  s += `<text x="${pl-4}" y="${Y(0)+4}" fill="#6b7785" font-size="10" text-anchor="end">0</text>`;
  if (tol){ s += `<line x1="${pl}" y1="${Y(tol)}" x2="${W-pr}" y2="${Y(tol)}" stroke="#ffcc4d" stroke-dasharray="4 3" stroke-opacity=".7"/>`;
    s += `<text x="${W-pr}" y="${Y(tol)-3}" fill="#ffcc4d" font-size="10" text-anchor="end">tol ${tol} ${unit}</text>`; }
  if (pts.length > 1) s += `<polyline points="${pts.map(p => `${X(p[0])},${Y(p[1])}`).join(" ")}" fill="none" stroke="#3fb6ff" stroke-width="2"/>`;
  const si = obsIndex();
  pts.forEach(p => { const o = S.observations[p[0]];
    s += `<circle cx="${X(p[0])}" cy="${Y(p[1])}" r="${p[0] === si ? 5.5 : 4}" fill="${p[0] === si ? "#ffcc4d" : "#3fb6ff"}" stroke="#161b22" stroke-width="2"/>`;
    s += `<circle cx="${X(p[0])}" cy="${Y(p[1])}" r="11" fill="transparent" style="cursor:pointer" data-i="${p[0]}"><title>obs ${o.n} (${o.state}): ${f(p[1],2)} ${unit}${o.decision ? " → " + o.decision : ""}</title></circle>`; });
  vals.forEach((v, i) => { s += `<text x="${X(i)}" y="${H-4}" fill="#6b7785" font-size="10" text-anchor="middle">${i+1}</text>`; });
  if (!pts.length) s += `<text x="${W/2}" y="${H/2}" fill="#6b7785" font-size="11" text-anchor="middle">no data yet</text>`;
  el.innerHTML = s;
  el.querySelectorAll("[data-i]").forEach(c => c.onclick = () => { sel = +c.dataset.i; render(); });
}

function renderMap(){
  const M = S.map, el = $("mapsvg"), pts = [];
  const add = p => p && p[0] != null && pts.push(p);
  add(M.expected); M.search_spots.forEach(add); add(M.place); add(M.estimate); M.detections.forEach(d => add(d.xy)); M.tip_path.forEach(add);
  if (!pts.length){ el.innerHTML = ""; return; }
  let x0 = Math.min(...pts.map(p=>p[0])), x1 = Math.max(...pts.map(p=>p[0])), y0 = Math.min(...pts.map(p=>p[1])), y1 = Math.max(...pts.map(p=>p[1]));
  const pad = 0.04; x0 -= pad; x1 += pad; y0 -= pad; y1 += pad;
  const span = Math.max(x1 - x0, y1 - y0, 0.12), cxm = (x0+x1)/2, cym = (y0+y1)/2;
  x0 = cxm - span/2; x1 = cxm + span/2; y0 = cym - span/2; y1 = cym + span/2;
  const W = 400, H = 400, sc = W / span, X = x => (x - x0) * sc, Y = y => H - (y - y0) * sc;
  let s = "";
  // grid every 5 cm
  for (let g = Math.ceil(x0/0.05)*0.05; g <= x1; g += 0.05) s += `<line x1="${X(g)}" y1="0" x2="${X(g)}" y2="${H}" stroke="#1f262f"/><text x="${X(g)+2}" y="${H-3}" fill="#4b5563" font-size="9">${g.toFixed(2)}</text>`;
  for (let g = Math.ceil(y0/0.05)*0.05; g <= y1; g += 0.05) s += `<line x1="0" y1="${Y(g)}" x2="${W}" y2="${Y(g)}" stroke="#1f262f"/><text x="2" y="${Y(g)-2}" fill="#4b5563" font-size="9">${g.toFixed(2)}</text>`;
  if (M.board){ const [bw, bh] = M.board; s += `<rect x="${X(-bw/2)}" y="${Y(bh/2)}" width="${bw*sc}" height="${bh*sc}" fill="none" stroke="#3b4553" stroke-dasharray="3 3"/><text x="${X(-bw/2)+3}" y="${Y(bh/2)+11}" fill="#4b5563" font-size="10">board</text>`; }
  s += `<g stroke="#4b5563"><line x1="${X(0)}" y1="${Y(0)}" x2="${X(0.03)}" y2="${Y(0)}"/><line x1="${X(0)}" y1="${Y(0)}" x2="${X(0)}" y2="${Y(0.03)}"/></g><text x="${X(0.031)}" y="${Y(0)+3}" fill="#6b7785" font-size="9">x</text><text x="${X(0)-3}" y="${Y(0.032)}" fill="#6b7785" font-size="9">y</text>`;
  if (M.base){ // arrow at the edge toward the robot base
    const dx = M.base[0] - cxm, dy = M.base[1] - cym, L = Math.hypot(dx, dy) || 1, ex = cxm + dx/L * span*0.44, ey = cym + dy/L * span*0.44;
    s += `<text x="${X(ex)}" y="${Y(ey)}" fill="#6b7785" font-size="11" text-anchor="middle">◆ base (${f(L,2)} m)</text>`; }
  if (M.tip_path.length){ s += `<polyline points="${M.tip_path.map(p=>`${X(p[0])},${Y(p[1])}`).join(" ")}" fill="none" stroke="#7aa2ff" stroke-width="1.5" stroke-opacity=".8"/>`;
    M.tip_path.forEach((p, i) => s += `<circle cx="${X(p[0])}" cy="${Y(p[1])}" r="${i === M.tip_path.length-1 ? 4 : 2.5}" fill="#7aa2ff"><title>move ${i+1}: tip (${p.map(v=>f(v,3)).join(", ")})</title></circle>`); }
  M.search_spots.forEach((p, i) => { const sk = M.skipped.some(q => q && Math.hypot(q[0]-p[0], q[1]-p[1]) < 1e-4);
    s += `<rect x="${X(p[0])-5}" y="${Y(p[1])-5}" width="10" height="10" fill="none" stroke="${sk ? "#ff6b6b" : "#6b7785"}" stroke-width="1.5"><title>search view ${i+1}${sk ? " (skipped: unreachable)" : ""}</title></rect>`; });
  if (M.expected) s += `<g stroke="#9aa7b4" stroke-width="1.5"><line x1="${X(M.expected[0])-8}" y1="${Y(M.expected[1])}" x2="${X(M.expected[0])+8}" y2="${Y(M.expected[1])}"/><line x1="${X(M.expected[0])}" y1="${Y(M.expected[1])-8}" x2="${X(M.expected[0])}" y2="${Y(M.expected[1])+8}"/></g><text x="${X(M.expected[0])+9}" y="${Y(M.expected[1])-9}" fill="#9aa7b4" font-size="10">expected</text>`;
  if (M.place) s += `<circle cx="${X(M.place[0])}" cy="${Y(M.place[1])}" r="${Math.max(6, (M.width||0.04)/2*sc)}" fill="none" stroke="#c084fc" stroke-width="1.5" stroke-dasharray="4 3"><title>place spot ${xy(M.place)}</title></circle><text x="${X(M.place[0])-8}" y="${Y(M.place[1])+18}" fill="#c084fc" font-size="10" text-anchor="end">place</text>`;
  M.detections.forEach(d => { s += `<circle cx="${X(d.xy[0])}" cy="${Y(d.xy[1])}" r="${Math.max(2.5, d.sigma_mm/1000*sc)}" fill="#3ddc84" fill-opacity=".35" stroke="#3ddc84" stroke-width="1"><title>obs ${d.n} (${d.state}): ${xy(d.xy,4)} σ ${f(d.sigma_mm,1)} mm</title></circle>`; });
  if (M.estimate){ const r = (M.width || 0.04)/2*sc;
    s += `<circle cx="${X(M.estimate[0])}" cy="${Y(M.estimate[1])}" r="${r}" fill="none" stroke="#ff9f43" stroke-width="2"/><circle cx="${X(M.estimate[0])}" cy="${Y(M.estimate[1])}" r="3.5" fill="#ff9f43"><title>estimate ${xy(M.estimate,4)}</title></circle>`; }
  el.setAttribute("viewBox", `0 0 ${W} ${H}`); el.innerHTML = s;
}

async function loop(){ try { await loadRuns(); await tick(); } catch(e) {} setTimeout(loop, 500); }
loop();
</script></body></html>
"""

if __name__ == "__main__":
    main()
