"""Tap-to-pick and prompt-to-pick (interfaces/wristcam/target.py) on synthetic renders: a red cup and a
green box on the table, a still look frame, a tap or a (mocked) Claude answer -> table point, the tracker
across the engine's homing looks, and the closed loop picking the tapped box (not the cup).
No daemon, no camera, no arm, no API calls (Claude is a fake client).

    .venv/bin/python -m pytest -q interfaces/tests/test_wristcam_target.py -s
"""
from __future__ import annotations

import json

import numpy as np
import pytest

from interfaces.grasp.grasp import Workcell
from interfaces.wristcam import engine, synth
from interfaces.wristcam import target as T

BOX_XY, BOX_SIZE, BOX_YAW = (0.06, -0.08), (0.05, 0.035, 0.04), 0.5
CUP_XY = (-0.02, -0.20)
LOOK_XY = (0.02, -0.13)
CAM, T_TC = synth.TRUE_CAMERA, synth.TRUE_T_TIP_CAM


def make_world():
    return synth.World(board=False, cups=[synth.Cup(CUP_XY)],
                       boxes=[synth.Box(BOX_XY, size=BOX_SIZE, yaw=BOX_YAW)])


def workcell(world, calibrated=True):
    return Workcell(T_base_table=world.T_base_table.tolist(), calibrated=calibrated)


@pytest.fixture(scope="module")
def look():
    """The still frame a player sees: camera straight down 0.25 m over the table, both objects in view."""
    world = make_world()
    arm = synth.SimArm(world, ss=1)
    wc = workcell(world)
    q, h = T.look_pose(wc, CAM, T_TC, arm.q(), LOOK_XY, 0.25)
    assert q is not None and abs(h - 0.25) < 1e-9
    arm.move_joints(q)
    lf = T.LookFrame(arm.frame(), [float(x) for x in q])
    return world, wc, lf, lf.view(CAM, T_TC, world.T_base_table)


def _err_mm(xy, true):
    return float(np.hypot(xy[0] - true[0], xy[1] - true[1]) * 1000)


def test_tap_on_the_box_and_the_cup(look):
    world, wc, lf, view = look
    uv = view.project_table([[BOX_XY[0], BOX_XY[1], BOX_SIZE[2]]])[0] + [6, -4]   # a finger is not exact
    t = T.resolve(lf.image, view, T.TargetRequest(lf.id, lf.q, "tap", tuple(uv)), wc)
    print(f"\nbox: error {_err_mm(t.xy_table, BOX_XY):.2f} mm, height {t.height * 1000:.1f} mm "
          f"(known {t.height_known}), size {np.round(np.array(t.size) * 1000, 1)} mm, yaw {t.yaw_table:.3f}, fit {t.fit['costs']}")
    assert _err_mm(t.xy_table, BOX_XY) < 3.0
    assert t.refusal is None and t.kind == "object"
    assert abs(t.height - BOX_SIZE[2]) < 0.012
    assert abs(t.size[0] - BOX_SIZE[0]) < 0.006 and abs(t.size[1] - BOX_SIZE[1]) < 0.006
    assert abs(((t.yaw_table - BOX_YAW) + np.pi / 2) % np.pi - np.pi / 2) < 0.06
    d = t.to_dict(lf.size)
    assert len(d["outline"]) >= 4 and all(0 <= u <= 1 and 0 <= v <= 1 for u, v in d["outline"])
    json.dumps(d)  # serialisable for the phone

    uv = view.project_table([[CUP_XY[0], CUP_XY[1], 0.08]])[0]
    c = T.resolve(lf.image, view, T.TargetRequest(lf.id, lf.q, "tap", tuple(uv)), wc)
    print(f"cup: error {_err_mm(c.xy_table, CUP_XY):.2f} mm, height {c.height * 1000:.1f} mm, diameter {c.size[1] * 1000:.1f} mm")
    assert c.kind == "cup" and _err_mm(c.xy_table, CUP_XY) < 3.0
    assert abs(c.height - 0.08) < 0.01 and abs(c.width - 0.06) < 0.006


class FakeClaude:
    """Stands in for anthropic.Anthropic(): records the request, answers with canned text."""

    def __init__(self, answer):
        self.answer, self.calls = answer, []
        self.messages = self

    def create(self, **kw):
        self.calls.append(kw)
        text = self.answer(kw) if callable(self.answer) else self.answer

        class Block:
            def __init__(self, t):
                self.type, self.text = "text", t

        class Msg:
            content = [Block(text)]
        return Msg()


def _small(view, xyz, s):
    uv = view.project_table([xyz])[0]
    return [round(float(uv[0] * s), 1), round(float(uv[1] * s), 1)]


def test_prompt_with_mocked_claude(look):
    world, wc, lf, view = look
    s = 768 / 1280

    def answer(kw):
        p = _small(view, [BOX_XY[0], BOX_XY[1], BOX_SIZE[2]], s)
        return ('Sure! {"found": true, "point": %s, "box": [%f, %f, %f, %f], "label": "green box", '
                '"confidence": 0.9, "reason": "the only box", "candidates": []}'
                % (p, p[0] - 45, p[1] - 45, p[0] + 45, p[1] + 45))
    fake = FakeClaude(answer)
    loc = T.ClaudeLocator(client=fake, model="claude-test")
    ans = loc.locate(lf.image, "the green box on the table")
    kw = fake.calls[0]
    assert kw["model"] == "claude-test"
    img = kw["messages"][0]["content"][0]
    assert img["type"] == "image" and img["source"]["media_type"] == "image/jpeg"
    assert "768 x 432" in kw["messages"][0]["content"][1]["text"]
    assert ans.found and ans.label == "green box"
    true_uv = view.project_table([[BOX_XY[0], BOX_XY[1], BOX_SIZE[2]]])[0]
    assert np.hypot(*(np.array(ans.point) - true_uv)) < 2.0          # rescaled to the full frame
    req = T.request_from_answer(ans, lf.id, lf.q, "the green box on the table")
    t = T.resolve(lf.image, view, req, wc)
    print(f"\nprompt -> box: error {_err_mm(t.xy_table, BOX_XY):.2f} mm, label {t.label}")
    assert _err_mm(t.xy_table, BOX_XY) < 3.0 and t.label == "green box"


def test_claude_not_found_ambiguous_and_junk():
    s, size = 0.6, (768, 432)
    loc = T.ClaudeLocator(client=FakeClaude(""))
    a = loc.parse('{"found": false, "reason": "no cup in view", "candidates": []}', s, size)
    assert not a.found and a.reason == "no cup in view" and a.candidates == []
    a = loc.parse('{"found": false, "reason": "two cups", "candidates": [{"point": [100, 50], "label": "red cup"},'
                  ' {"box": [300, 200, 340, 260], "label": "blue cup"}, {"point": [9999, 5]}]}', s, size)
    assert not a.found and len(a.candidates) == 2
    assert np.allclose(a.candidates[0]["point"], (100 / s, 50 / s)) and np.allclose(a.candidates[1]["point"], (320 / s, 230 / s))
    a = loc.parse('{"found": true, "point": [10, 10], "label": "cup", "confidence": 0.2}', s, size)
    assert not a.found and a.candidates and a.candidates[0]["label"] == "cup"   # unsure -> the player taps
    a = loc.parse("I think it is the cup on the left", s, size)
    assert not a.found and "couldn't read" in a.reason
    a = loc.parse('{"found": true, "point": [5000, 10]}', s, size)
    assert not a.found


def test_claude_needs_a_key(monkeypatch):
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    with pytest.raises(T.LocatorUnavailable):
        T.ClaudeLocator().client
    monkeypatch.setenv("WRISTCAM_PICK_MODEL", "claude-other")
    assert T.ClaudeLocator().model == "claude-other"
    monkeypatch.delenv("WRISTCAM_PICK_MODEL")
    assert T.ClaudeLocator().model == "claude-opus-5-5"


def test_tracker_keeps_the_box_over_three_homing_looks(look):
    """The engine's CENTER: camera over the estimate at 0.18, 0.12, 0.08 m above the top; the tracker
    must stay on the box (the cup is 13 cm away) and converge on its centre."""
    world, wc, lf, view = look
    uv = view.project_table([[BOX_XY[0], BOX_XY[1], BOX_SIZE[2]]])[0]
    t = T.resolve(lf.image, view, T.TargetRequest(lf.id, lf.q, "tap", tuple(uv)), wc)
    finder = T.SeededFinder(t)
    arm = synth.SimArm(make_world(), ss=1)
    eng = engine.GraspEngine(arm, finder, CAM, T_TC, wc, T.pick_config(t, wc))
    est = np.array(t.xy_table)
    errs = []
    for h in (0.18, 0.12, 0.08):
        q, _ = eng.camera_over(est, h)
        assert q is not None
        arm.move_joints(q)
        det, v, _ = eng.observe(locate_hint(est))
        assert det is not None, finder.history[-1]
        est = np.array(det.xyz_table[:2])
        errs.append(_err_mm(est, BOX_XY))
        assert _err_mm(est, CUP_XY) > 100
    print("\ntracker errors (mm) at 0.18/0.12/0.08 m:", np.round(errs, 2), "sources:",
          [h.get("result") for h in finder.history], "height", round(finder.height, 4))
    assert max(errs) < 3.0
    assert abs(finder.height - BOX_SIZE[2]) < 0.015   # from the tap frame; looks straight down keep it


def locate_hint(xy):
    from interfaces.wristcam.locate import Hint
    return Hint(tuple(xy), 0.03)


def test_engine_picks_the_tapped_box_not_the_cup(look, tmp_path):
    world0, wc, lf, view = look
    uv = view.project_table([[BOX_XY[0], BOX_XY[1], BOX_SIZE[2]]])[0]
    t = T.resolve(lf.image, view, T.TargetRequest(lf.id, lf.q, "tap", tuple(uv)), wc)
    world = make_world()
    box, cup = world.boxes[0], world.cups[0]
    arm = synth.SimArm(world, ss=1, joint_noise=0.0005)
    arm.move_joints(lf.q)
    seen = []
    lifted = []

    def on_event(state, event, e):
        seen.append((state, event, e.get("decision")))
        if e.get("decision") == "verified":
            lifted.append(box.z0)
    res = T.run_pick(arm, t, CAM, T_TC, wc, on_event=on_event, log_path=tmp_path / "run.jsonl",
                     frames_dir=tmp_path / "frames")
    print("\npick:", {k: res[k] for k in ("ok", "reason", "trace", "estimate_table", "observations", "height")})
    assert res["ok"], res["reason"]
    assert lifted and lifted[0] > 0.05                 # the box was in the air when verified
    assert _err_mm(box.xy, BOX_XY) < 6 and box.z0 == 0.0   # put back where it was
    assert tuple(cup.xy) == CUP_XY and cup.z0 == 0.0       # the cup never moved
    assert not world.collisions
    decisions = [d for _, ev, d in seen if ev == "decision"]
    assert "converged" in decisions and "holding" in decisions and "verified" in decisions
    assert _err_mm(res["estimate_table"], BOX_XY) < 3.0
    # every move was a checked move_joints inside the soft box, under the daemon's cap
    from armctl import envelope as envm
    for line in (tmp_path / "run.jsonl").read_text().splitlines():
        e = json.loads(line)
        if e["event"] == "move":
            assert not envm.DEFAULT.check_point(e["q_to"]) and e["speed"] <= 0.3


def test_refusals(look, tmp_path):
    world, wc, lf, view = look
    # uncalibrated workcell / no hand-eye / no lens
    he, cam = tmp_path / "he.json", tmp_path / "cam.json"
    assert "not calibrated" in T.calibration_refusal(workcell(world, calibrated=False), cam, he)
    assert "hand-eye" in T.calibration_refusal(wc, cam, he)
    he.write_text(json.dumps({"T_tip_cam": T_TC.tolist()}))
    assert "lens" in T.calibration_refusal(wc, cam, he)
    CAM.save(cam)
    assert T.calibration_refusal(wc, cam, he) is None
    req = lambda uv: T.TargetRequest(lf.id, lf.q, "tap", tuple(uv))  # noqa: E731
    # the jaws (lower corners)
    with pytest.raises(T.TargetRefused, match="jaw"):
        T.resolve(lf.image, view, req((15, 712)), wc)
    # bare table
    with pytest.raises(T.TargetRefused, match="bare table"):
        T.resolve(lf.image, view, req((640, 120)), wc)
    # off the table (a known table extent that excludes the cup)
    uv_cup = view.project_table([[CUP_XY[0], CUP_XY[1], 0.08]])[0]
    with pytest.raises(T.TargetRefused, match="off the table") as e:
        T.resolve(lf.image, view, req(uv_cup), wc, T.PickConfig(table_extent=(0.0, 0.3, -0.15, 0.2)))
    assert e.value.code == "off_table"
    # outside the picture
    with pytest.raises(T.TargetRefused, match="outside"):
        T.resolve(lf.image, view, req((1400, 100)), wc)
    # too wide: with the real gripper's ~71 mm stroke, the 3.5 cm box + 2 cm margin fits but a cup doesn't
    uv_box = view.project_table([[BOX_XY[0], BOX_XY[1], BOX_SIZE[2]]])[0]
    assert T.resolve(lf.image, view, req(uv_box), wc, max_open=0.071).refusal is None
    c = T.resolve(lf.image, view, req(uv_cup), wc, max_open=0.071)
    assert c.refusal and "too wide" in c.refusal and "7.1 cm" in c.refusal
    assert T.run_pick(None, c, CAM, T_TC, wc)["ok"] is False


def test_too_wide_box_is_refused_with_a_message():
    world = synth.World(board=False, boxes=[synth.Box((0.04, -0.1), size=(0.12, 0.09, 0.05), yaw=0.2)])
    arm = synth.SimArm(world, ss=1)
    wc = workcell(world)
    q, _ = T.look_pose(wc, CAM, T_TC, arm.q(), (0.04, -0.12), 0.25)
    arm.move_joints(q)
    lf = T.LookFrame(arm.frame(), [float(x) for x in q])
    view = lf.view(CAM, T_TC, world.T_base_table)
    uv = view.project_table([[0.04, -0.1, 0.05]])[0]
    t = T.resolve(lf.image, view, T.TargetRequest(lf.id, lf.q, "tap", tuple(uv)), wc)
    print("\nwide box:", np.round(np.array(t.size) * 1000, 1), t.refusal)
    assert t.size[1] > 0.08 and t.refusal and t.refusal.startswith("too wide to grab: ")
