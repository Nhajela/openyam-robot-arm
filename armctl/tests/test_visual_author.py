"""Visual multi-pass author with a fake Claude client: no network, no camera, nothing moves."""
import copy
import json
from pathlib import Path

import pytest

from armctl import choreo_render as R
from armctl import choreo_scene as S
from armctl import llm_author as L
from armctl import playable as P
from armctl import visual_author as V

READY = [0.0, 0.5, 0.8, 0.0, 0.0, 0.0]
SCORES = {k: 4 for k in V.SCORE_KEYS}


def _pl(j1=0.4, t1=3.0):
    return {"name": "fake", "space": "joint", "start": "ready", "loop": 1, "speed": 1.0, "notes": "x",
            "keyframes": [{"t": 0, "q": READY, "grip": 0, "ease": "cubic"},
                          {"t": t1, "q": [j1, 0.6, 1.0, 0.2, 0, 0], "grip": 1, "ease": "cubic"},
                          {"t": t1 * 2, "q": READY, "grip": 0, "ease": "cubic"}]}


def _reply(pl, satisfied=False, scores=SCORES):
    return {"critique": "c", "scores": scores, "changelog": ["did a thing"], "satisfied": satisfied, "playable": pl}


class Block:
    def __init__(self, type, **kw):
        self.type = type
        self.__dict__.update(kw)

    def model_dump(self, **_):
        return dict(self.__dict__)


class Resp:
    def __init__(self, data, stop_reason="end_turn"):
        self.stop_reason = stop_reason
        self.stop_details = None
        self.model = "claude-fable-5-1"
        self.usage = {"input_tokens": 10, "output_tokens": 20}
        self.content = [Block("thinking", thinking="", signature="sig"),
                        Block("text", text=json.dumps(data))]


class FakeClient:
    """Records every request; replies from a queue."""

    def __init__(self, replies, router=None):
        self.replies, self.calls, self.router_calls = list(replies), [], []
        outer = self

        class _Stream:
            def __init__(self, r):
                self.r = r

            def __enter__(self):
                return self

            def __exit__(self, *a):
                return False

            def get_final_message(self):
                return self.r

        class _Beta:
            class messages:  # noqa: N801
                @staticmethod
                def stream(**kw):
                    outer.calls.append(copy.deepcopy(kw))
                    r = outer.replies.pop(0)
                    return _Stream(r if isinstance(r, Resp) else Resp(r))

        class _Msgs:
            @staticmethod
            def create(**kw):
                outer.router_calls.append(kw)
                return Resp(router or {"needs_camera": False, "reason": "self-contained"})

        self.beta = _Beta()
        self.messages = _Msgs()


@pytest.fixture(autouse=True)
def quiet(monkeypatch, tmp_path):
    monkeypatch.setattr(L, "LOG_PATH", tmp_path / "llm.log")


@pytest.fixture
def fast_render(monkeypatch):
    """Rendering is tested in test_choreo_render; here write tiny stand-in PNGs."""
    def fake(p, out, **kw):
        out = Path(out)
        out.mkdir(parents=True, exist_ok=True)
        paths = []
        for n in ("overview.png", "poses-1.png"):
            (out / n).write_bytes(b"\x89PNG fake")
            paths.append(out / n)
        return paths
    monkeypatch.setattr(R, "render_sheets", fake)


def _types(msg):
    return [b["type"] for b in msg["content"]]


def test_stops_on_satisfaction_and_sends_images(fast_render, tmp_path):
    c = FakeClient([_reply(_pl()), _reply(_pl(), satisfied=True), _reply(_pl(0.5))])
    res = V.author_visual("a happy wave", client=c, passes=3, camera="never", sheets_dir=tmp_path / "s",
                          echo=lambda *a: None)
    assert res["ok"] and res["stop_reason"] == "satisfied" and len(c.calls) == 2
    review = c.calls[1]["messages"][-1]
    assert review["role"] == "user" and _types(review).count("image") == 2
    assert "REVIEW pass 1 of 3" in review["content"][0]["text"]
    assert "VALID" in review["content"][-1]["text"] and "Motion metrics" in review["content"][-1]["text"]
    # assistant turn echoed back unchanged (thinking block kept) between the user turns
    assert [m["role"] for m in c.calls[1]["messages"]] == ["user", "assistant", "user"]
    assert c.calls[1]["messages"][1]["content"][0]["type"] == "thinking"
    assert P.validate(res["playable"])["ok"]
    for f in ("pass-0.json", "pass-1.json", "session.json", "CHANGELOG.md"):
        assert (tmp_path / "s" / f).exists()
    assert "| 1 | yes |" in (tmp_path / "s" / "CHANGELOG.md").read_text()


def test_draft_satisfied_does_not_stop_before_a_visual_review(fast_render):
    c = FakeClient([_reply(_pl(), satisfied=True), _reply(_pl(), satisfied=True)])
    res = V.author_visual("x", client=c, passes=2, camera="never", echo=lambda *a: None)
    assert len(c.calls) == 2 and res["stop_reason"] == "satisfied"


def test_validator_failures_fed_back(fast_render):
    bad = _pl(j1=5.0)  # outside the soft limits
    c = FakeClient([_reply(bad), _reply(_pl(), satisfied=True)])
    res = V.author_visual("x", client=c, passes=2, camera="never", echo=lambda *a: None)
    txt = c.calls[1]["messages"][-1]["content"][-1]["text"]
    assert "REJECTED" in txt and "keyframe_out_of_range j1" in txt
    assert res["ok"] and res["passes"][0]["ok"] is False and res["passes"][1]["ok"] is True


def test_never_returns_an_invalid_playable(fast_render):
    bad = _pl(j1=5.0)
    c = FakeClient([_reply(_pl()), _reply(bad), _reply(bad)])
    res = V.author_visual("x", client=c, passes=2, camera="never", echo=lambda *a: None)
    assert res["ok"] and res["playable"]["keyframes"][1]["q"][0] == 0.4  # the last VALID one (draft)
    c = FakeClient([_reply(bad), _reply(bad), _reply(bad)])
    res = V.author_visual("x", client=c, passes=2, camera="never", echo=lambda *a: None)
    assert not res["ok"] and res["playable"] is None and "error" in res


def test_speed_only_violations_are_retimed_not_relaxed(fast_render):
    fast = _pl(j1=0.4, t1=0.6)
    assert {v["kind"] for v in P.validate(L._normalize(fast, "x", "m"))["violations"]} <= P.SPEED_KINDS
    c = FakeClient([_reply(fast), _reply(fast, satisfied=True)])
    res = V.author_visual("x", client=c, passes=1, camera="never", echo=lambda *a: None)
    assert res["ok"] and res["retimed"] and P.validate(res["playable"])["ok"]
    assert res["playable"]["keyframes"][1]["t"] > 0.6


def test_model_error_returns_best_so_far(fast_render):
    c = FakeClient([_reply(_pl()), Resp({}, stop_reason="refusal")])
    res = V.author_visual("x", client=c, passes=2, camera="never", echo=lambda *a: None)
    assert res["ok"] and "model call failed" in res["stop_reason"]


def test_request_shape(fast_render):
    c = FakeClient([_reply(_pl()), _reply(_pl(), satisfied=True)])
    V.author_visual("x", client=c, passes=1, camera="never", model="claude-fable-5-1", effort="high",
                    echo=lambda *a: None)
    kw = c.calls[0]
    assert kw["model"] == "claude-fable-5-1" and kw["fallbacks"] == "default"
    assert kw["betas"] == [V.FALLBACK_BETA] and "thinking" not in kw and "temperature" not in kw
    assert kw["output_config"]["effort"] == "high"
    assert kw["output_config"]["format"]["schema"]["required"] == V.VISUAL_SCHEMA["required"]
    assert "Choreography guidelines" in kw["system"] and "AUDIENCE" in kw["system"]
    img = next(b for b in c.calls[1]["messages"][-1]["content"] if b["type"] == "image")
    assert img["source"]["type"] == "base64" and img["source"]["media_type"] == "image/png"
    c2 = FakeClient([_reply(_pl())])
    V.author_visual("x", client=c2, passes=0, camera="never", model="claude-haiku-5-5", echo=lambda *a: None)
    assert "fallbacks" not in c2.calls[0]


def test_camera_routing_and_snapshot(fast_render):
    snap = {"jpeg": b"\xff\xd8fake", "q": READY, "text": "Live wrist-camera frame. Grid A1: face with j1 ~ +0.30"}
    taken = []

    def snap_fn():
        taken.append(1)
        return snap

    c = FakeClient([_reply(_pl()), _reply(_pl(), satisfied=True)])
    res = V.author_visual("point at the cup", client=c, passes=1, snapshot_fn=snap_fn, echo=lambda *a: None)
    assert taken and res["route"]["use_camera"] and res["route"]["method"] == "rules"
    first = c.calls[0]["messages"][0]
    assert first["content"][0]["source"]["media_type"] == "image/jpeg" and "Grid A1" in first["content"][1]["text"]
    assert not c.router_calls  # rules were sure: no classifier call

    taken.clear()
    c = FakeClient([_reply(_pl()), _reply(_pl(), satisfied=True)])
    res = V.author_visual("act surprised then relieved", client=c, passes=1, snapshot_fn=snap_fn,
                          echo=lambda *a: None)
    assert not taken and not res["route"]["use_camera"] and "image" not in _types(c.calls[0]["messages"][0])

    c = FakeClient([_reply(_pl()), _reply(_pl(), satisfied=True)], router={"needs_camera": True, "reason": "r"})
    res = V.author_visual("do the thing", client=c, passes=1, snapshot_fn=snap_fn, echo=lambda *a: None)
    assert res["route"]["method"] == "model" and res["route"]["use_camera"] and c.router_calls
    assert c.router_calls[0]["model"] == V.ROUTER_MODEL

    def broken():
        raise RuntimeError("console down")
    c = FakeClient([_reply(_pl()), _reply(_pl(), satisfied=True)])
    res = V.author_visual("point at the cup", client=c, passes=1, camera="always", snapshot_fn=broken,
                          echo=lambda *a: None)
    assert res["ok"] and "console down" in res["route"]["snapshot_error"]


def test_router_rules_and_overrides():
    assert S.route("wave at the person in front of you")["use_camera"]
    assert S.route("look at whoever is closest")["use_camera"]
    assert S.route("dance toward the window")["use_camera"]
    assert not S.route("happy dance")["use_camera"]
    assert not S.route("act surprised then relieved")["use_camera"]
    assert S.route("happy dance", "always") == {"use_camera": True, "method": "override", "reason": "--camera always"}
    assert not S.route("point at the cup", "never")["use_camera"]
    r = S.route("hmm", classify=lambda t: (_ for _ in ()).throw(RuntimeError("x")))
    assert r["use_camera"] is False and "classifier failed" in r["reason"]


def test_snapshot_reads_pose_without_a_camera(monkeypatch):
    import cv2
    import numpy as np
    ok, jpg = cv2.imencode(".jpg", np.zeros((72, 128, 3), np.uint8))
    state = {"motors": [{"name": f"joint{i}", "pos": 0.1 * i} for i in range(1, 7)]}

    def fetch(url, timeout):
        return json.dumps(state).encode() if url.endswith("/api/state") else jpg.tobytes()

    monkeypatch.setattr(S, "calibration_status", lambda: (False, "test: not calibrated"))
    s = S.snapshot("http://127.0.0.1:1/camera.jpg", fetch=fetch)
    assert s["q"] == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 0.6]) and s["jpeg"][:2] == b"\xff\xd8"
    assert "UNCALIBRATED" in s["text"] and "not calibrated" in s["text"] and len(s["grid"]) == 20
    with pytest.raises(RuntimeError):
        S.snapshot("http://127.0.0.1:1/camera.jpg", fetch=lambda u, t: (_ for _ in ()).throw(OSError("refused")))


def test_echo_content_drops_pre_fallback_thinking():
    blocks = [{"type": "thinking", "thinking": "", "signature": "a"}, {"type": "text", "text": "partial"},
              {"type": "fallback"}, {"type": "thinking", "thinking": "", "signature": "b"},
              {"type": "text", "text": "{}"}]
    out = V.echo_content(blocks)
    assert [b["type"] for b in out] == ["text", "fallback", "thinking", "text"]


def test_cli_visual_writes_out(monkeypatch, tmp_path):
    pl = L._normalize(_pl(), "a happy wave", "m")
    monkeypatch.setattr(V, "author_visual", lambda *a, **k: {
        "ok": True, "playable": pl, "report": P.validate(pl), "passes": [{}], "latency_s": 1.0,
        "model": k["model"], "stop_reason": "satisfied"})
    out = tmp_path / "happy.json"
    assert L.main(["a happy wave", "--visual", "--passes", "3", "--model", "claude-fable-5-1",
                   "--out", str(out)]) == 0
    assert P.validate(json.loads(out.read_text()))["ok"]
