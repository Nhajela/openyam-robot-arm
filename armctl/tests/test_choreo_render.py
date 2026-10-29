"""Render sheets (offscreen): tile counts, multiple sheets, thinning, metrics. Nothing moves."""
import json
import math

import cv2
import pytest

from armctl import choreo_render as R
from armctl import playable as P

READY = [0.0, 0.5, 0.8, 0.0, 0.0, 0.0]


def _pl(keys):
    """keys: list of (t, q, ease)."""
    return {"name": "t", "space": "joint", "start": "ready", "loop": 1, "speed": 1.0,
            "keyframes": [{"t": 0, "q": READY, "grip": 0.0, "ease": "cubic"}] +
                         [{"t": t, "q": q, "grip": 0.5, "ease": e} for t, q, e in keys]}


def test_pose_list_keyframes_midpoints_quarters_and_holds():
    a = [0.3, 0.6, 1.0, 0.2, 0.0, 0.0]
    p = _pl([(1.5, a, "cubic"), (2.5, a, "hold"), (5.0, READY, "cubic")])
    poses = R.pose_list(p)
    kinds = [x["kind"] for x in poses]
    # 4 keys; seg1 (1.5 s): mid; seg2 hold: none; seg3 (2.5 s > 2 s): 3 quarter-ish points
    assert kinds.count("key") == 4 and kinds.count("mid") == 2 and kinds.count("quarter") == 2
    assert [x["t"] for x in poses] == sorted(x["t"] for x in poses)
    mid = next(x for x in poses if x["kind"] == "mid")
    assert mid["t"] == 0.75 and 0.0 < mid["q"][0] < 0.3  # on the interpolated path


def test_render_sheets_tile_counts_and_size(tmp_path):
    p = P.load("wave-hello")
    poses = R.pose_list(p)
    n_keys = len(p["keyframes"])
    assert len([x for x in poses if x["kind"] == "key"]) == n_keys
    assert len([x for x in poses if x["kind"] != "key"]) == n_keys - 1  # all segments short, none hold
    paths = R.render_sheets(p, tmp_path)
    assert paths[0].name == "overview.png"
    assert len(paths) == 1 + math.ceil(len(poses) / R.POSES_PER_SHEET)
    for path in paths:
        img = cv2.imread(str(path))
        assert img is not None and 1500 <= img.shape[1] <= 2000 and img.shape[0] > 800


def test_long_playable_splits_and_thins(tmp_path):
    keys, t = [], 0.0
    for i in range(30):
        t += 2.5
        keys.append((t, [0.3 if i % 2 else -0.2, 0.6, 1.0, 0.0, 0.0, 0.0], "cubic"))
    p = _pl(keys)
    full = R.pose_list(p, max_sheets=100)
    assert len(full) > 4 * R.POSES_PER_SHEET
    thin = R.pose_list(p, max_sheets=2)
    assert len(thin) <= 2 * R.POSES_PER_SHEET and all(x.get("thinned") for x in thin)
    paths = R.render_sheets(p, tmp_path, max_sheets=2, overview=False)
    assert len(paths) == 2


def test_metrics_flag_invisible_wave_and_audience_config(tmp_path, monkeypatch):
    txt = R.metrics_text(P.load("wave-hello"))
    assert "barely visible from the audience" in txt and "Speed headroom" in txt
    cfg = tmp_path / "aud.json"
    cfg.write_text(json.dumps({"default": "x", "presets": {"x": {"eye": [1.0, 0.0, 0.5], "label": "test"}}}))
    monkeypatch.setattr(R, "AUDIENCE_FILE", cfg)
    v = R.load_audience()
    assert v.label.startswith("AUDIENCE (x)") and v.eye.tolist() == [1.0, 0.0, 0.5]
    assert R.load_audience("0.5,1.0,0.8").eye.tolist() == [0.5, 1.0, 0.8]
    with pytest.raises(ValueError):
        R.load_audience("nowhere")


def test_invalid_playable_still_renders(tmp_path):
    p = _pl([(0.3, [0.9, 0.5, 0.8, 0, 0, 0], "cubic")])  # far too fast
    assert not P.validate(p)["ok"]
    paths = R.render_sheets(p, tmp_path)
    assert all(x.exists() for x in paths)
