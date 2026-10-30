"""🤳 Selfie (openplay/selfie.py + the server's selfie.aim / selfie.snap): the aim pose points the
wrist camera at the audience inside the daemon's limits, photos are saved on the laptop (the
camera's original + the phone's framed copy), only the active player can take or fetch them."""
from __future__ import annotations

import asyncio

import numpy as np
import pytest

from armctl import kin
from openplay import selfie as SF
from openplay.server import OpenPlay
from openplay.tests.test_chess import Conn, FakeArm


def test_aim_pose_looks_at_the_audience_inside_the_limits():
    eye, label = SF.audience_eye("standing")
    q, off = SF.aim_pose(eye, {"j1": [-0.6, 1.2]})
    assert -0.5 <= q[0] <= 1.1 + 1e-9 and q[1:3] == [0.5, 0.8] and q[4:] == [0.0, 0.0]
    assert off < 15                                   # the camera sees ~100 deg across
    d = eye - kin.tip(q)
    assert np.dot(kin.approach(q), d / np.linalg.norm(d)) > 0.95
    q2, off2 = SF.aim_pose(SF.audience_eye("down-table")[0], {"j1": [-0.6, 1.2]})
    assert abs(q2[0]) < 0.05 and off2 < 2              # straight down the table
    q3, _ = SF.aim_pose(np.array([0.3, -1.0, 0.8]), {"j1": [-0.6, 1.2]})
    assert q3[0] >= -0.5 - 1e-9                        # a spot past the glass wall: clamped to the limit
    with pytest.raises(ValueError):
        SF.audience_eye("1,2")


def test_store_saves_the_original_and_the_framed_copy(tmp_path):
    st = SF.SelfieStore(tmp_path)
    rec = st.new(b"\xff\xd8raw", "p-asha", "Asha Rao", [0.0] * 6)
    assert (tmp_path / rec["original"]).read_bytes() == b"\xff\xd8raw" and "asha-rao" in rec["original"]
    with pytest.raises(KeyError):
        st.framed(rec["id"], "p-ben", b"\xff\xd8x")      # someone else's selfie
    with pytest.raises(ValueError):
        st.framed(rec["id"], "p-asha", b"not a jpeg")
    st.framed(rec["id"], "p-asha", b"\xff\xd8framed")
    assert (tmp_path / rec["framed"]).read_bytes() == b"\xff\xd8framed"
    assert st.files()[0].endswith(".jpg") and len(st.files()) == 2
    assert (tmp_path / "selfies.jsonl").read_text().count("\n") == 2 and "p-asha" not in (tmp_path / "selfies.jsonl").read_text()
    assert st.path("../../etc/passwd") is None


def test_server_aims_then_snaps_and_saves_on_the_laptop(tmp_path):
    async def main():
        op = OpenPlay(events_log=None, auto_play="on", cam_source="test", selfie_dir=tmp_path)
        op.arm = FakeArm("mock")
        calls = []

        async def do(cmd, **kw):
            calls.append((cmd, kw))
            return {"op": "done"}
        op.arm.do = do
        disp = Conn()
        op.display_joined(disp)
        a, b = Conn(), Conn()
        for c, name in ((a, "Asha"), (b, "Ben")):
            await op.phone_msg(c, {"op": "hello", "pid": "p-" + name.lower(), "name": name})
            await op.phone_msg(c, {"op": "join"})
        op._lobby_events(op.lobby.tick(can_start=True))
        assert a.last("welcome")["features"]["selfie"] is True

        await op.phone_msg(b, {"op": "selfie.snap"})
        assert b.refusals()[-1] == "not your turn"

        await op.phone_msg(a, {"op": "selfie.aim"})
        for _ in range(200):
            if a.last("selfie") and a.last("selfie")["stage"] == "aimed":
                break
            await asyncio.sleep(0.01)
        assert a.last("selfie")["stage"] == "aimed"
        cmd, kw = calls[-1]
        assert cmd == "move_joints" and kw["speed"] <= 0.3 and 0.9 < kw["q"][0] <= 1.2

        await op.phone_msg(a, {"op": "selfie.snap"})
        for _ in range(300):
            if a.last("selfie")["stage"] == "taken":
                break
            await asyncio.sleep(0.01)
        m = a.last("selfie")
        assert m["stage"] == "taken"
        f = op.selfies.path(m["id"])
        assert f.is_file() and f.read_bytes()[:2] == b"\xff\xd8" and f.parent == tmp_path
        assert disp.last("selfie")["file"] == f.name
        assert any(e.get("kind") == "selfie" for e in disp.msgs if e.get("op") == "event")
        op._selfie_framed(m["id"], "p-asha", b"\xff\xd8framed")
        assert (tmp_path / op.selfies.items[m["id"]]["framed"]).is_file()
        assert disp.last("selfie")["file"].endswith("-framed.jpg")
    asyncio.run(main())


def test_no_camera_no_selfie(tmp_path):
    async def main():
        op = OpenPlay(events_log=None, auto_play="on", cam_source="off", selfie_dir=tmp_path)
        op.arm = FakeArm("mock")
        a = Conn()
        await op.phone_msg(a, {"op": "hello", "pid": "p-asha", "name": "Asha"})
        await op.phone_msg(a, {"op": "join"})
        op._lobby_events(op.lobby.tick(can_start=True))
        assert a.last("welcome")["features"]["selfie"] is False
        await op.phone_msg(a, {"op": "selfie.aim"})
        assert "camera" in a.refusals()[-1]
    asyncio.run(main())
