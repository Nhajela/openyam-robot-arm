"""The look-and-grasp viewer (interfaces/wristcam/viz.py): a run.jsonl -> the summary the page draws,
and the read-only HTTP endpoints. No daemon, no camera, no arm; the server binds 127.0.0.1 on a free port.

    .venv/bin/python -m pytest -q interfaces/tests/test_wristcam_viz.py
"""
from __future__ import annotations

import json
import urllib.request
from threading import Thread

import numpy as np
import pytest

from interfaces.wristcam import viz

CAM = {"model": "pinhole", "K": [[540.0, 0, 640.0], [0, 540.0, 360.0], [0, 0, 1]], "D": [0, 0, 0, 0, 0],
       "size": [1280, 720], "meta": {"board": {"squares": [7, 5], "square_mm": 29.972}}}
CFG = {"obj": "cup", "width": 0.074, "height": 0.08, "expected_table": [-0.14, -0.031], "place_table": [0.08, -0.05],
       "search_offsets": [[0, 0], [0.06, 0]], "center_px_tol": 25.0, "change_tol": 0.002}
Q = [0.33856, 1.39029, 0.836, -0.17529, 0.02155, -0.01392]


def _det(x, y, px):
    return {"kind": "cup", "xyz_base": [0.36, 0.12, 0.047], "xyz_table": [x, y, 0.08], "yaw_table": 0.0,
            "diameter": 0.0853, "height": 0.08, "sigma_xy": 0.00107, "pixel": px, "score": 1.0, "id": None, "extra": {}}


EVENTS = [
    {"t": 0.0, "state": "search", "event": "start", "config": CFG, "camera": CAM,
     "T_tip_cam": np.eye(4).tolist(), "T_base_table": np.eye(4).tolist()},
    {"t": 0.15, "state": "search", "event": "skip_search_view", "xy": [-0.08, -0.031], "reason": "unreachable"},
    {"t": 0.2, "state": "search", "event": "move", "why": "search view 1", "q_from": Q, "q_to": [q + (0.2 if i == 3 else 0) for i, q in enumerate(Q)],
     "speed": 0.25, "tip_table": [-0.14, -0.056, 0.176]},
    {"t": 13.2, "state": "search", "event": "observe", "q": Q, "det": _det(-0.1394, -0.0308, [683.7, 381.4]),
     "px_err": 20.4, "frame": "obs_001_search.jpg"},
    {"t": 13.2, "state": "search", "event": "decision", "decision": "found", "est": [-0.1394, -0.0308], "sigma": 0.00107},
    {"t": 13.2, "state": "search", "event": "transition", "to": "center"},
    {"t": 13.3, "state": "center", "event": "move", "why": "centre camera 0.18 m above the object (level 0)",
     "q_from": Q, "q_to": Q, "speed": 0.25, "tip_table": [-0.140, -0.0559, 0.176]},
    {"t": 30.1, "state": "center", "event": "observe", "q": Q, "det": _det(-0.1391, -0.0335, [689.0, 398.1]),
     "px_err": 37.6, "frame": "obs_002_center.jpg"},
    {"t": 30.1, "state": "center", "event": "decision", "decision": "closer", "level": 0, "px_err": 37.6,
     "change_mm": 2.91, "sigma_mm": 1.09, "est": [-0.1391, -0.0335]},
    {"t": 31.0, "state": "center", "event": "observe", "q": Q, "det": None, "px_err": None, "frame": None},
]


def _write(d, events, torn=False):
    d.mkdir(parents=True, exist_ok=True)
    text = "".join(json.dumps(e) + "\n" for e in events)
    (d / "run.jsonl").write_text(text + ('{"t": 40.0, "sta' if torn else ""))


def test_summary_of_a_run_in_progress(tmp_path):
    _write(tmp_path / "r1", EVENTS, torn=True)          # the engine is mid-write
    s = viz.summarize(viz.read_events(tmp_path / "r1" / "run.jsonl"), tmp_path / "r1")
    assert s["state"] == "center" and s["trace"] == ["search", "center"] and not s["ended"]
    assert [o["px_err"] for o in s["observations"]] == [20.4, 37.6, None]
    assert [o["change_mm"] for o in s["observations"]] == [None, 2.91, None]
    assert [o["decision"] for o in s["observations"]] == ["found", "closer", None]
    assert s["estimate"]["table"] == [-0.1391, -0.0335]
    assert s["estimate"]["sigma_mm"] == pytest.approx(1.09)
    assert s["estimate"]["width_mm"] == pytest.approx(85.3)
    assert s["estimate"]["base"][:2] == pytest.approx([-0.1391, -0.0335])     # identity T_base_table
    assert s["last_decision"]["decision"] == "closer" and "change_mm 2.91" in s["last_decision"]["text"]
    assert s["observations"][-1]["frame"] is None and s["observations"][-1]["det"] is None
    m = s["last_move"]
    assert m["why"].startswith("centre camera") and not s["move_pending"]
    assert s["map"]["search_spots"] == [[-0.14, -0.031], [pytest.approx(-0.08), -0.031]]
    assert s["map"]["skipped"] == [[-0.08, -0.031]] and s["map"]["place"] == [0.08, -0.05]
    assert len(s["map"]["detections"]) == 2 and len(s["map"]["tip_path"]) == 2
    assert s["map"]["board"] == pytest.approx([0.2098, 0.1499], abs=1e-3)
    assert s["tolerances"] == {"px": 25.0, "change_mm": 2.0}
    assert s["events"][1]["text"].startswith("skip search view")
    first = viz.summarize(EVENTS[:3])
    assert first["last_move"]["joint"] == 4 and first["last_move"]["dq_deg"] == pytest.approx(11.46, abs=0.01)
    assert first["move_pending"]
    ov = s["observations"][0]["overlay"]          # projected from q + camera model
    assert ov["label"] == "cup σ1.1mm 85mm" and ov["pixel"] == [683.7, 381.4]
    assert ov["centre"] == [640.0, 360.0] and ov["size"] == [1280, 720]


def test_end_event_and_overlay_shapes():
    ev = EVENTS[:5] + [{"t": 45.6, "state": "failed", "event": "end", "ok": False, "reason": "lost twice",
                        "trace": ["search", "center", "failed"], "estimate_table": [-0.139, -0.034],
                        "observations": 2, "measured_width": 0.0906}]
    s = viz.summarize(ev, with_overlay=False)
    assert s["ended"] and s["ok"] is False and s["state"] == "failed" and s["reason"] == "lost twice"
    assert s["estimate"]["table"] == [-0.139, -0.034] and s["estimate"]["width_mm"] == pytest.approx(90.6)
    ring = viz.outline_table(_det(0.0, 0.0, None), None)
    assert np.allclose(np.hypot(ring[:, 0], ring[:, 1]), 0.0853 / 2)
    blob = {"kind": "blob", "xyz_table": [0.0, 0.0, 0.02], "yaw_table": np.pi / 2, "extra": {"size_m": [0.04, 0.02]}}
    rect = viz.outline_table(blob, None)
    assert rect.shape == (4, 3) and np.ptp(rect[:, 0]) == pytest.approx(0.02) and np.ptp(rect[:, 1]) == pytest.approx(0.04)


def test_server_lists_runs_and_serves_summary(tmp_path):
    _write(tmp_path / "a" / "run1", EVENTS[:5])
    _write(tmp_path / "a" / "run2", EVENTS)
    (tmp_path / "a" / "run2" / "frames").mkdir()
    (tmp_path / "a" / "run2" / "frames" / "obs_001_search.jpg").write_bytes(b"\xff\xd8jpg")
    with pytest.raises(SystemExit):
        viz.serve(tmp_path, 8091)                    # never the armctl daemon's port
    srv = viz.serve(tmp_path, 0)
    Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"
    get = lambda p: urllib.request.urlopen(base + p, timeout=5).read()
    try:
        runs = json.loads(get("/api/runs"))["runs"]
        assert [r["name"] for r in runs] == ["a/run2", "a/run1"]          # newest first
        s = json.loads(get("/api/run"))                                     # default: newest
        assert s["name"] == "a/run2" and len(s["observations"]) == 3 and s["live"]
        assert json.loads(get(f"/api/run?run=a/run2&v={s['version']}"))["unchanged"]
        assert get("/frame?run=a/run2&f=obs_001_search.jpg") == b"\xff\xd8jpg"
        with pytest.raises(urllib.error.HTTPError):
            get("/frame?run=a/run2&f=../run.jsonl")
        assert b"Wristcam Pick Viewer" in get("/")
    finally:
        srv.shutdown()
        srv.server_close()


def test_live_camera_proxy_fails_gracefully_and_only_gets(tmp_path):
    with pytest.raises(ValueError):
        viz.check_get_url("http://127.0.0.1:1/armctl/lease")      # only /armctl/status may be read
    assert viz.check_get_url("http://127.0.0.1:1/armctl/status")
    _write(tmp_path / "r", EVENTS)
    calls = []

    def down(url, timeout):                                         # the console is not there
        calls.append(url)
        raise OSError("connection refused")

    srv = viz.serve(tmp_path, 0, live=viz.LiveSource("http://127.0.0.1:1/camera.jpg", "http://127.0.0.1:1/armctl/status", fetch=down))
    Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"
    try:
        with pytest.raises(urllib.error.HTTPError) as e:
            urllib.request.urlopen(base + "/live.jpg", timeout=5)
        assert e.value.code == 503 and b"connection refused" in e.value.read()
        r = json.loads(urllib.request.urlopen(base + "/api/live", timeout=5).read())
        assert r["q"] is None and "connection refused" in r["q_error"] and r["overlay"] is None
        assert b"Wristcam Pick Viewer" in urllib.request.urlopen(base + "/", timeout=5).read()   # page still up
    finally:
        srv.shutdown()
        srv.server_close()
    assert set(calls) == {"http://127.0.0.1:1/camera.jpg", "http://127.0.0.1:1/armctl/status"}

    # console up: the frame passes through with its age, and the estimate projects at the live q
    up = lambda url, timeout: b"\xff\xd8live" if url.endswith(".jpg") else json.dumps({"state": {"q": Q}}).encode()
    live = viz.LiveSource("http://x/camera.jpg", "http://x/armctl/status", fetch=up)
    b, age, err = live.frame()
    assert b == b"\xff\xd8live" and age < 1 and err is None
    v = viz.Viewer(tmp_path, live=live)
    d = v.run_dir(None)
    ov = viz.live_overlay(v.summary(d), v._start[str(d)], live.q()[0], (1280, 720))
    assert ov["label"].startswith("cup estimate") and ov["centre"] == [640.0, 360.0]
    assert viz.LiveSource(None, None).frame()[0] is None
