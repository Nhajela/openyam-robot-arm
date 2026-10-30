"""--chess / --pick (auto | on | off) and the wrist-camera relay (openplay/cam.py).

auto offers chess and Pick it up to phones only in rehearsal (mock backend or the DimOS sim); on
also offers them on the real arm, but only once they could run (calibrated board / camera); off
refuses them. The server-side refusals are unchanged. The camera games appear only while the
camera answers.
"""
from __future__ import annotations

import asyncio
import json
import time

import pytest

from openplay.cam import CamRelay, synthetic_frame
from openplay.server import OpenPlay
from openplay.tests.test_chess import WORKCELL, Conn, FakeArm


def make(adapter="mock", chess="auto", pick="auto", workcell=None, cam="off"):
    op = OpenPlay(events_log=None, auto_play="on", workcell=workcell, chess_mode=chess, pick_mode=pick, cam_source=cam)
    op.arm = FakeArm(adapter)
    return op


class FakePick:
    def __init__(self, refusal=None):
        self._refusal = refusal

    def refusal(self):
        return self._refusal


def workcell(tmp_path, calibrated):
    raw = json.loads(WORKCELL.read_text())
    raw["calibrated"] = calibrated
    f = tmp_path / f"workcell-{calibrated}.json"
    f.write_text(json.dumps(raw))
    return f


def test_auto_offers_chess_and_pick_only_in_rehearsal(tmp_path):
    for adapter in ("mock", "sim"):
        op = make(adapter)
        op.pick = FakePick()
        f = op.phone_features()
        assert f["chess"] is True and f["pick"] is True, adapter
    op = make("real", workcell=workcell(tmp_path, True))  # even calibrated: auto keeps them off
    op.pick = FakePick()
    f = op.phone_features()
    assert f["chess"] is False and f["pick"] is False


def test_on_offers_them_on_the_real_arm_once_they_could_run(tmp_path):
    op = make("real", chess="on", pick="on", workcell=workcell(tmp_path, False))  # board and camera not calibrated
    op.pick = FakePick("camera not calibrated yet")
    f = op.phone_features()
    assert f["chess"] is False and f["pick"] is False
    op = make("real", chess="on", pick="on", workcell=workcell(tmp_path, True))
    op.pick = FakePick()
    f = op.phone_features()
    assert f["chess"] is True and f["pick"] is True


def test_off_refuses_them():
    async def main():
        op = make("mock", chess="off", pick="off")
        assert op.phone_features()["chess"] is False and op.phone_features()["pick"] is False
        c = Conn()
        await op.phone_msg(c, {"op": "hello", "pid": "p-asha", "name": "Asha"})
        assert c.last("welcome")["features"]["chess"] is False
        await op.phone_msg(c, {"op": "join"})
        op._lobby_events(op.lobby.tick(can_start=True))
        await op.phone_msg(c, {"op": "chess.new", "opponent": "engine", "color": "white"})
        assert "not available" in c.refusals()[-1]
        if op.pick is not None:
            await op.phone_msg(c, {"op": "pick.look"})
            assert "switched off" in c.refusals()[-1]
    asyncio.run(main())


def test_bad_mode_is_an_error():
    with pytest.raises(ValueError):
        OpenPlay(events_log=None, chess_mode="maybe")


def test_camera_feature_follows_the_relay():
    op = make(cam="off")
    assert op.phone_features()["cam"] is False
    op = make(cam="test")
    assert op.phone_features()["cam"] is True


def test_relay_serves_newer_frames_and_reports_failures():
    n = {"i": 0}

    def fetch():
        n["i"] += 1
        return b"\xff\xd8frame%d" % n["i"]

    changes = []
    r = CamRelay("console", fetch=fetch, fps=50, probe_s=0.05, on_change=changes.append)
    r.start()
    try:
        jpeg, seq = r.latest(0, wait_s=2.0)
        assert jpeg.startswith(b"\xff\xd8") and seq >= 1 and r.ok and changes == [True]
        jpeg2, seq2 = r.latest(seq, wait_s=2.0)                  # asked again: a newer frame
        assert seq2 > seq and jpeg2 != jpeg
    finally:
        r.stop()

    def broken():
        raise OSError("console down")

    r = CamRelay("console", fetch=broken, probe_s=0.05)
    r.ok = True
    r.start()
    try:
        for _ in range(100):
            if not r.ok:
                break
            time.sleep(0.02)
        assert r.ok is False and "console down" in r.error
    finally:
        r.stop()


def test_synthetic_frame_follows_the_arm():
    a = synthetic_frame([0.0] * 6, 0.0)
    b = synthetic_frame([0.5, 0, 0, 0.3, 0, 0], 0.0)
    assert a[:2] == b"\xff\xd8" and b[:2] == b"\xff\xd8" and a != b
