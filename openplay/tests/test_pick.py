"""Open play "Pick it up" (openplay/pick_play.py) in-process: the server with a fake ArmBridge whose
moves drive the synthetic wrist-camera world (interfaces/wristcam/synth.py: a red cup and a green
box). Look -> tap (or a prompt answered by a FAKE Claude client) -> "Pick this?" -> the closed loop
picks the box up, lifts it, puts it back. No daemon, no camera, no network, no API calls.

    .venv/bin/python -m pytest -q openplay/tests/test_pick.py -s
"""
from __future__ import annotations

import asyncio
import json

import numpy as np
import pytest

from armctl import playable as pl
from interfaces.grasp.grasp import Workcell
from interfaces.wristcam import synth
from interfaces.wristcam import target as T
from openplay.server import OpenPlay

BOX_XY, BOX_SIZE = (0.06, -0.08), (0.05, 0.035, 0.04)
CUP_XY = (-0.02, -0.20)
CAM, T_TC = synth.TRUE_CAMERA, synth.TRUE_T_TIP_CAM


class SimBridge:
    """Stands in for openplay.arm.ArmBridge over synth.SimArm: move_joints / gripper really move
    the fake arm (instantly), state follows it."""

    def __init__(self, sim: synth.SimArm):
        self.sim = sim
        self.hello = {"poses": {"ready": list(sim.q())}}
        self.state = {}
        self._sync()
        self.connected = self.ready = self.has_lease = True
        self.estop = False
        self.cmds: list[tuple] = []
        self.stopped = 0

    backend = property(lambda self: "mock")
    adapter = property(lambda self: "mock")

    def _sync(self):
        self.state = {"q": [float(x) for x in self.sim.q()], "grip": float(self.sim.grip()), "moving": False}

    def unavailable_reason(self):
        return None

    def speeds(self):
        return {"lin_mps": 0.08, "ang_rps": 0.6, "joint_rps": 0.5}

    async def validate(self, p):
        return pl.validate(p)

    async def play(self, p, confirm, timeout=90.0):
        return {"op": "done"}

    async def stop(self):
        self.stopped += 1

    async def do(self, cmd, **kw):
        self.cmds.append((cmd, kw))
        if cmd == "move_joints":
            assert kw["speed"] <= 0.3
            self.sim.move_joints(kw["q"], kw["speed"])
        elif cmd == "gripper":
            self.sim.gripper(kw["width"])
        elif cmd == "ready":
            self.sim.move_joints(self.hello["poses"]["ready"])
        await asyncio.sleep(0)
        self._sync()
        return {"op": "done"}


class Conn:
    def __init__(self):
        self.msgs: list[dict] = []
        self.pid = None
        self.alive = True
        self.last_stream_refusal = 0.0

    def send(self, m):
        self.msgs.append(m)

    def last(self, op):
        return next((m for m in reversed(self.msgs) if m.get("op") == op), None)

    def refusals(self):
        return [m["reason"] for m in self.msgs if m.get("op") == "refused"]


class FakeClaude:
    def __init__(self, answer):
        self.answer, self.calls = answer, []
        self.messages = self

    def create(self, **kw):
        self.calls.append(kw)
        text = self.answer(kw) if callable(self.answer) else self.answer

        class Block:
            type, text_ = "text", None

        b = Block()
        b.text = text

        class Msg:
            content = [b]
        return Msg()


def make_world():
    return synth.World(board=False, cups=[synth.Cup(CUP_XY)], boxes=[synth.Box(BOX_XY, size=BOX_SIZE, yaw=0.5)])


async def station(tmp_path, calibrated=True, auto_play="on", answer=None):
    world = make_world()
    sim = synth.SimArm(world, ss=1)
    wcp = tmp_path / "workcell.json"
    Workcell(T_base_table=world.T_base_table.tolist(), calibrated=calibrated).save(wcp)
    he, cam = tmp_path / "handeye.json", tmp_path / "wristcam.json"
    he.write_text(json.dumps({"T_tip_cam": T_TC.tolist()}))
    CAM.save(cam)
    locator = T.ClaudeLocator(client=FakeClaude(answer or "{}"), model="claude-test")
    op = OpenPlay(events_log=None, auto_play=auto_play, workcell=wcp,
                  pick_opts={"frame_source": sim.frame, "locator": locator, "intrinsics": cam, "handeye": he,
                             "settle_s": 0.0, "grip_settle_s": 0.0,
                             "pick_cfg": T.PickConfig(look_xy=(0.02, -0.13))})
    op.arm = SimBridge(sim)
    disp = Conn()
    op.display_joined(disp)
    phones = []
    for name in ("Asha", "Ben"):
        c = Conn()
        await op.phone_msg(c, {"op": "hello", "pid": "p-" + name.lower(), "name": name})
        await op.phone_msg(c, {"op": "join"})
        phones.append(c)
    op._lobby_events(op.lobby.tick(can_start=True))
    assert op.lobby.active.pid == "p-asha"
    return op, disp, phones, world


async def until(pred, timeout=120.0, what=""):
    for _ in range(int(timeout / 0.02)):
        if pred():
            return
        await asyncio.sleep(0.02)
    raise AssertionError(f"condition not met: {what}")


def stage(c):
    m = c.last("pick")
    return m and m["stage"]


def box_tap(op):
    """Normalised (x, y) of the box's top centre in the still the phone was sent."""
    lf = op.pick.frame
    wc = op.pick.workcell()
    v = lf.view(CAM, T_TC, wc.T_base_table)
    uv = v.project_table([[BOX_XY[0], BOX_XY[1], BOX_SIZE[2]]])[0]
    return float(uv[0] / lf.size[0]), float(uv[1] / lf.size[1]), v


async def look(op, a):
    await op.phone_msg(a, {"op": "pick.look"})
    await until(lambda: stage(a) == "frame", what="the still")
    f = a.last("pick.frame")
    assert f and f["kind"] == "still" and f["jpeg"].startswith("data:image/jpeg;base64,") and f["id"] == op.pick.frame.id
    return f


def test_tap_confirm_pick_lift_and_put_back(tmp_path):
    async def main():
        op, disp, (a, b), world = await station(tmp_path)
        assert a.last("welcome")["features"]["pick"] is True
        assert a.last("pick")["refusal"] is None
        # not your turn
        await op.phone_msg(b, {"op": "pick.look"})
        assert b.refusals()[-1] == "not your turn"
        # tap before looking
        await op.phone_msg(a, {"op": "pick.tap", "frame": "x", "x": 0.5, "y": 0.5})
        assert "take a picture first" in a.refusals()[-1]
        f = await look(op, a)
        assert disp.last("pick.frame")["id"] == f["id"]
        assert op.arm.cmds[0][0] == "move_joints"                 # the look pose, one checked move
        # a tap on the jaw is refused with a reason, the picture stays
        await op.phone_msg(a, {"op": "pick.tap", "frame": f["id"], "x": 0.01, "y": 0.99})
        await until(lambda: a.last("pick")["error"], what="jaw refusal")
        assert "jaw" in a.last("pick")["error"] and stage(a) == "frame"
        # an old picture is refused
        await op.phone_msg(a, {"op": "pick.tap", "frame": "old", "x": 0.5, "y": 0.5})
        assert "out of date" in a.refusals()[-1]
        # tap the box
        x, y, _ = box_tap(op)
        await op.phone_msg(a, {"op": "pick.tap", "frame": f["id"], "x": x + 0.004, "y": y - 0.004})
        await until(lambda: stage(a) == "target", what="target")
        t = a.last("pick")["target"]
        assert t["refusal"] is None and len(t["outline"]) >= 4 and "Pick this?" in a.last("pick")["text"]
        assert np.hypot(t["xy_table"][0] - BOX_XY[0], t["xy_table"][1] - BOX_XY[1]) < 0.003
        assert disp.last("pick")["target"]["id"] == t["id"] and disp.last("pick")["who"] == "Asha"
        # confirm a stale id: refused; the right one: the arm picks it
        await op.phone_msg(a, {"op": "pick.confirm", "target": "nope"})
        assert "nothing to confirm" in a.refusals()[-1]
        box = world.boxes[0]
        heights = []
        await op.phone_msg(a, {"op": "pick.confirm", "target": t["id"]})
        assert op.busy and op.busy["kind"] == "pick"
        await op.phone_msg(a, {"op": "pick.look"})
        assert "busy" in a.refusals()[-1]

        def watch():
            heights.append(box.z0)
            return stage(a) in ("done", "failed")
        await until(watch, 300, "pick done")
        m = a.last("pick")
        print("\nprogress:", [p["text"] for p in m["progress"]], m["result"])
        assert stage(a) == "done", m
        assert m["result"]["ok"] and m["result"]["lifted"]
        assert max(heights) > 0.05                               # it was in the air
        assert box.z0 == 0.0 and np.hypot(box.xy[0] - BOX_XY[0], box.xy[1] - BOX_XY[1]) < 0.006
        assert tuple(world.cups[0].xy) == CUP_XY
        assert not world.collisions
        texts = [p["text"] for p in m["progress"]]
        assert any("Lifted it" in s for s in texts) and any("Grabbing" in s for s in texts)
        assert any(x.get("op") == "pick.frame" and x["kind"] == "live" for x in disp.msgs)
        assert any(x.get("op") == "event" and "picked up" in x["text"] for x in disp.msgs)
        assert all(kw["speed"] <= 0.3 for c, kw in op.arm.cmds if c == "move_joints")
        await until(lambda: not op.busy, 10, "free")
    asyncio.run(main())


def test_prompt_found_and_ambiguous(tmp_path):
    answers = []

    def answer(kw):
        return answers.pop(0)

    async def main():
        op, disp, (a, b), world = await station(tmp_path, answer=answer)
        f = await look(op, a)
        x, y, v = box_tap(op)
        s = 768 / 1280
        u, vv = x * 1280 * s, y * 720 * s
        # ambiguous: candidates for the player to tap
        answers.append(json.dumps({"found": False, "reason": "two things could be it", "candidates": [
            {"point": [u, vv], "label": "green box"}, {"point": [200, 200], "label": "red cup"}]}))
        await op.phone_msg(a, {"op": "pick.ask", "frame": f["id"], "text": "the thing"})
        await until(lambda: a.last("pick")["candidates"], what="candidates")
        cands = a.last("pick")["candidates"]
        assert [c["label"] for c in cands] == ["green box", "red cup"] and abs(cands[0]["x"] - x) < 0.01
        assert "Which one?" in a.last("pick")["text"]
        # found
        answers.append(json.dumps({"found": True, "point": [u, vv], "box": [u - 40, vv - 40, u + 40, vv + 40],
                                   "label": "green box", "confidence": 0.95, "reason": "", "candidates": []}))
        await op.phone_msg(a, {"op": "pick.ask", "frame": f["id"], "text": "pick up the green box"})
        await until(lambda: stage(a) == "target", what="target from the prompt")
        t = a.last("pick")["target"]
        assert t["label"] == "green box" and np.hypot(t["xy_table"][0] - BOX_XY[0], t["xy_table"][1] - BOX_XY[1]) < 0.003
        req = op.pick.locator._client.calls[-1]
        assert "pick up the green box" in req["messages"][0]["content"][1]["text"]
        # not found at all
        answers.append(json.dumps({"found": False, "reason": "There is no banana", "candidates": []}))
        await op.phone_msg(a, {"op": "pick.ask", "frame": f["id"], "text": "the banana"})
        await until(lambda: "banana" in (a.last("pick")["text"] or ""), what="not found")
        assert stage(a) == "frame"
        # STOP keeps the picture; the turn ending forgets it
        await op.phone_msg(a, {"op": "stop"})
        assert op.pick.frame is not None
        op.pick.cancel("p-asha", "the turn ended")
        assert op.pick.frame is None and stage(a) == "idle"
    asyncio.run(main())


def test_refused_until_the_camera_is_calibrated(tmp_path):
    async def main():
        op, disp, (a, b), world = await station(tmp_path, calibrated=False)
        assert "camera not calibrated yet" in a.last("pick")["refusal"]
        await op.phone_msg(a, {"op": "pick.look"})
        assert "camera not calibrated yet" in a.refusals()[-1]
        assert not op.arm.cmds                                # nothing moved
        (tmp_path / "handeye.json").unlink()
        Workcell(T_base_table=world.T_base_table.tolist(), calibrated=True).save(tmp_path / "workcell.json")
        await op.phone_msg(a, {"op": "pick.look"})
        assert "hand-eye" in a.refusals()[-1] and not op.arm.cmds
    asyncio.run(main())


def test_operator_approves_the_look_and_the_pick(tmp_path):
    async def main():
        op, disp, (a, b), world = await station(tmp_path, auto_play="off")
        await op.phone_msg(a, {"op": "pick.look"})
        await until(lambda: op.pending is not None, 10, "pending look")
        assert "look over the table" in op.pending["title"] and len(op.pending["keyframes"]) == 2
        assert not op.arm.cmds
        await op.display_msg(disp, {"op": "approve", "id": op.pending["id"]})
        await until(lambda: stage(a) == "frame", 60, "the still")
        x, y, _ = box_tap(op)
        await op.phone_msg(a, {"op": "pick.tap", "frame": op.pick.frame.id, "x": x, "y": y})
        await until(lambda: stage(a) == "target", 60, "target")
        n = len(op.arm.cmds)
        await op.phone_msg(a, {"op": "pick.confirm", "target": a.last("pick")["target"]["id"]})
        await until(lambda: op.pending is not None, 10, "pending pick")
        assert "Pick it up: the object" in op.pending["title"]
        await op.display_msg(disp, {"op": "deny", "id": op.pending["id"]})
        await until(lambda: not op.busy, 10, "denied")
        assert len(op.arm.cmds) == n                          # denied: nothing moved
        assert any("said no" in m.get("text", "") for m in a.msgs if m.get("op") == "status")
    asyncio.run(main())


def test_detector_lists_what_it_sees_and_picks_a_named_box(tmp_path):
    """With an open-vocabulary detector (ground truth here; YOLO-E on the laptop): the look lists
    what it sees, a typed name is found without Claude, a tap uses the detector's outline, and
    the closed loop re-detects the named box at every look."""
    from interfaces.wristcam import detect as D

    async def main():
        op, disp, (a, b), world = await station(tmp_path, answer='{"found": false, "reason": "should not be asked"}')
        world.cups[0].label, world.boxes[0].label = "red cup", "green box"
        op.pick._det = D.SynthDetector(world)
        f = await look(op, a)
        p = a.last("pick")
        assert sorted(c["label"] for c in p["candidates"]) == ["green box", "red cup"]
        assert "I can see" in p["text"]
        await op.phone_msg(a, {"op": "pick.ask", "frame": f["id"], "text": "green box"})
        await until(lambda: stage(a) == "target", what="target by name")
        t = a.last("pick")["target"]
        assert t["label"] == "green box" and t["refusal"] is None
        assert op.pick.target_prompt == "green box"
        assert np.hypot(op.pick.target.xy_table[0] - BOX_XY[0], op.pick.target.xy_table[1] - BOX_XY[1]) < 0.004
        # a tap on the cup uses the detector's outline (labelled), not GrabCut
        lf = op.pick.frame
        v = lf.view(CAM, T_TC, op.pick.workcell().T_base_table)
        uv = v.project_table([[CUP_XY[0] + 0.02, CUP_XY[1], 0.08]])[0]
        await op.phone_msg(a, {"op": "pick.tap", "frame": f["id"], "x": float(uv[0] / lf.size[0]), "y": float(uv[1] / lf.size[1])})
        await until(lambda: a.last("pick")["target"] and a.last("pick")["target"]["label"] == "red cup", what="cup by tap")
        # back to the box by name, then pick it
        await op.phone_msg(a, {"op": "pick.ask", "frame": f["id"], "text": "green box"})
        await until(lambda: a.last("pick")["target"]["label"] == "green box", what="box again")
        await op.phone_msg(a, {"op": "pick.confirm", "target": a.last("pick")["target"]["id"]})
        await until(lambda: stage(a) in ("done", "failed"), timeout=300, what="the pick")
        res = a.last("pick")["result"]
        print("\npick by name:", res)
        assert stage(a) == "done" and res["lifted"]
        box = world.boxes[0]
        assert np.hypot(box.xy[0] - BOX_XY[0], box.xy[1] - BOX_XY[1]) < 0.006 and box.z0 == 0.0
    asyncio.run(main())
