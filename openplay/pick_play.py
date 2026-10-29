"""openplay.pick_play: the "Pick it up" mode (PROTOCOL.md "Pick it up").

The player's phone shows a still wrist-camera picture taken over the table; they tap the thing
they want (or type / say "the cup on the table", which Claude finds in the picture), the phone
shows the outline that was found and asks "Pick this?", and on yes the arm homes in on it with
the wrist camera and picks it up (interfaces/wristcam/target.py + engine.GraspEngine), lifts it,
checks it's really in the gripper, puts it back where it was and returns to ready.

Every arm motion is a daemon ``move_joints`` / ``gripper`` through the open play lease (the
daemon's envelope, speed caps, deadman and E-STOP apply), and each step is checked by the engine
(soft box, table guard). With auto-play off the operator approves twice on the laptop: the move to
the look pose, then the pick itself. STOP, a turn end or E-STOP aborts the engine at its next step.

Refused unless the camera is calibrated: workcell.json ``calibrated: true`` plus
``interfaces/wristcam/calib/handeye.json`` and ``wristcam.json``.

Self-contained: the server calls ``PickStation.phone / display / display_joined / hello / cancel``
and nothing else. Frames come from the lab console (``$VISION_DASHBOARD_URL``, default
http://127.0.0.1:8090/camera.jpg) unless a ``frame_source`` is injected (tests).
"""
from __future__ import annotations

import asyncio
import base64
import contextlib
import logging
import os
import threading
import time
import urllib.request
from pathlib import Path
from typing import Any, Callable

import numpy as np

log = logging.getLogger("openplay.pick")

DASHBOARD_URL = os.environ.get("VISION_DASHBOARD_URL", "http://127.0.0.1:8090/camera.jpg")
LOOK_SPEED = 0.25          # rad/s to the look pose
SETTLE_S = 1.0             # wait after a move before taking the still
PHONE_FRAME_W = 960        # px wide, the still sent to the phone and the display
LIVE_FRAME_W = 640         # px wide, the engine's looks on the display
ASK_TIMEOUT_S = 60.0
RESOLVE_TIMEOUT_S = 30.0
PICK_TIMEOUT_S = 600.0
STAGE_TEXT = {
    "search": "👀 Looking for it", "center": "🎯 Centring over it", "approach": "🖐️ Lining up the fingers",
    "descend": "⬇️ Going down", "close": "✊ Grabbing", "lift": "⬆️ Lifting", "verify": "🔍 Checking it's in the gripper",
    "miss": "😅 Missed: trying again", "place": "📦 Putting it back", "ready": "🏠 Back to ready",
}


def dashboard_frame(url: str = DASHBOARD_URL, timeout: float = 5.0) -> np.ndarray:
    """One JPEG from the lab console's camera endpoint (read-only HTTP GET)."""
    import cv2

    with urllib.request.urlopen(url, timeout=timeout) as r:
        data = r.read()
    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError(f"no picture from {url}")
    return img


def jpeg_data_url(img, width: int, quality: int = 80) -> str:
    import cv2

    h, w = img.shape[:2]
    if w > width:
        img = cv2.resize(img, (width, int(round(h * width / w))), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode("ascii")


class Aborted(RuntimeError):
    pass


class BridgeArm:
    """The engine's ``Arm`` protocol (q, grip, move_joints, gripper, frame), called from the
    engine's worker thread and carried out on the server's event loop through the open play
    ArmBridge (the daemon's lease, envelope and E-STOP). ``abort`` stops it at the next call."""

    def __init__(self, bridge, loop: asyncio.AbstractEventLoop, frame_source: Callable[[], np.ndarray],
                 abort: threading.Event, settle_s: float = SETTLE_S, grip_settle_s: float = 1.5,
                 on_frame: Callable[[np.ndarray], None] | None = None):
        self.bridge, self.loop, self.frame_source = bridge, loop, frame_source
        self.abort, self.settle_s, self.grip_settle_s = abort, settle_s, grip_settle_s
        self.on_frame = on_frame
        self.last_frame: np.ndarray | None = None

    def _check(self):
        if self.abort.is_set():
            from interfaces.wristcam.engine import Unsafe
            raise Unsafe("stopped")

    def _call(self, coro, timeout: float):
        from interfaces.wristcam.engine import Unsafe
        from openplay.arm import Refused

        fut = asyncio.run_coroutine_threadsafe(coro, self.loop)
        try:
            return fut.result(timeout)
        except Refused as e:
            raise Unsafe(f"the arm refused: {e}") from None
        except TimeoutError:
            fut.cancel()
            raise Unsafe("the arm didn't answer in time") from None

    def q(self) -> np.ndarray:
        q = (self.bridge.state or {}).get("q")
        if not q:
            from interfaces.wristcam.engine import Unsafe
            raise Unsafe("no joint reading from the arm")
        return np.array(q, float)

    def grip(self) -> float:
        return float((self.bridge.state or {}).get("grip", np.nan))

    def move_joints(self, q, speed: float | None = None) -> None:
        self._check()
        q = [round(float(x), 4) for x in q]
        self._call(self.bridge.do("move_joints", q=q, speed=min(float(speed or 0.25), 0.3), timeout=120.0), 150.0)
        self._check()
        time.sleep(0.15)   # let the state stream catch up with the arrival

    def gripper(self, g: float) -> None:
        self._check()
        self._call(self.bridge.do("gripper", width=round(float(g), 3)), 20.0)
        time.sleep(self.grip_settle_s)
        self._check()

    def frame(self) -> np.ndarray:
        self._check()
        if self.settle_s:
            time.sleep(self.settle_s)
        img = self.frame_source()
        self.last_frame = img
        if self.on_frame:
            self.on_frame(img)
        return img


class PickStation:
    def __init__(self, op, frame_source: Callable[[], np.ndarray] | None = None, locator=None,
                 intrinsics: str | Path | None = None, handeye: str | Path | None = None,
                 settle_s: float = SETTLE_S, grip_settle_s: float = 1.5, pick_cfg=None):
        from interfaces.wristcam import target as TG

        self.op = op
        self.frame_source = frame_source or dashboard_frame
        self.locator = locator
        self.intrinsics, self.handeye = intrinsics, handeye
        self.settle_s, self.grip_settle_s = settle_s, grip_settle_s
        self.cfg = pick_cfg or TG.PickConfig.load()
        self.s: dict[str, Any] = self._fresh(None)
        self.frame = None           # target.LookFrame shown to the player
        self.target = None          # target.Target waiting for "Pick this?"
        self.abort = threading.Event()
        self.task: asyncio.Task | None = None

    @staticmethod
    def _fresh(pid):
        return {"pid": pid, "stage": "idle", "text": "", "error": None, "candidates": [], "progress": [],
                "result": None, "engine_state": None}

    # ------------------------------------------------------------------ calibration
    def _paths(self):
        from interfaces.wristcam.camera import DEFAULT_HANDEYE, DEFAULT_INTRINSICS

        return Path(self.intrinsics or DEFAULT_INTRINSICS), Path(self.handeye or DEFAULT_HANDEYE)

    def workcell(self):
        from interfaces.grasp.grasp import WORKCELL, Workcell

        return Workcell.load(Path(self.op.workcell_path) if self.op.workcell_path else WORKCELL)

    def refusal(self) -> str | None:
        from interfaces.wristcam import target as TG

        try:
            wc = self.workcell()
        except Exception as e:
            return f"camera not calibrated yet (can't read the workcell: {type(e).__name__})"
        cam, he = self._paths()
        return TG.calibration_refusal(wc, cam, he)

    def calibration(self):
        from interfaces.wristcam.camera import CameraModel, load_handeye

        cam, he = self._paths()
        return CameraModel.load(cam), load_handeye(he), self.workcell()

    # ------------------------------------------------------------------ messages
    def msg(self, display: bool = False) -> dict:
        s = self.s
        t = self.target
        m = {"op": "pick", "stage": s["stage"], "text": s["text"], "error": s["error"], "refusal": self.refusal(),
             "frame": self.frame.id if self.frame is not None else None,
             "target": t.to_dict(self.frame.size) if (t is not None and self.frame is not None) else None,
             "candidates": s["candidates"], "progress": s["progress"][-12:], "result": s["result"],
             "engine_state": s["engine_state"]}
        if m["target"]:
            m["target"].pop("request", None)
            m["target"].pop("outline_px", None)
        if display:
            m["who"] = self.op.name(s["pid"]) if s["pid"] else None
        return m

    def push(self) -> None:
        if self.s["pid"]:
            self.op.to_pid(self.s["pid"], self.msg())
        self.op.to_displays(self.msg(display=True))

    def _frame_msg(self, kind: str, img, width: int) -> dict:
        return {"op": "pick.frame", "kind": kind, "id": self.frame.id if (kind == "still" and self.frame) else None,
                "w": int(img.shape[1]), "h": int(img.shape[0]), "jpeg": jpeg_data_url(img, width)}

    def set(self, **kw) -> None:
        self.s.update(kw)
        self.push()

    def progress(self, text: str, stage: str | None = None) -> None:
        self.s["progress"].append({"t": round(time.time(), 2), "text": text})
        if stage:
            self.s["engine_state"] = stage
        self.push()

    def hello(self, c, pid: str) -> None:
        if self.s["pid"] == pid:
            if self.frame is not None:
                c.send(self._frame_msg("still", self.frame.image, PHONE_FRAME_W))
            c.send(self.msg())
        else:
            c.send({**self._fresh(None), "op": "pick", "refusal": self.refusal(), "frame": None, "target": None})

    def display_joined(self, c) -> None:
        if self.frame is not None:
            c.send(self._frame_msg("still", self.frame.image, PHONE_FRAME_W))
        c.send(self.msg(display=True))

    def cancel(self, pid: str | None, why: str = "stopped") -> None:
        """STOP, turn end, operator cancel: abort a running pick, forget the picture."""
        if pid is not None and self.s["pid"] not in (pid, None):
            return
        running = self.task is not None and not self.task.done()
        if running:
            self.abort.set()     # the engine stops at its next step; the arm is stopped by the caller
        elif why != "stopped":   # turn over / operator: forget the picture (STOP alone keeps it)
            self._reset(None)

    def _reset(self, pid):
        old = self.s["pid"]
        self.s = self._fresh(pid)
        self.frame = None
        self.target = None
        if old and old != pid:
            self.op.to_pid(old, self.msg())
        self.push()

    # ------------------------------------------------------------------ phone ops
    async def phone(self, c, pid: str, op: str, m: dict) -> None:
        o = self.op
        if not o.lobby.is_active(pid):
            return o.refuse(c, "not your turn")
        o.lobby.touch(pid)
        reason = self.refusal()
        if reason:
            return o.refuse(c, reason)
        if op == "pick.cancel":
            if self.task is not None and not self.task.done():
                return o.refuse(c, "the arm is busy picking: press STOP to stop it")
            if self.s["pid"] != pid or self.frame is None:
                return self._reset(pid)
            self.target = None   # "not that": keep the picture, choose again
            return self.set(stage="frame", text="Tap the thing to pick up, or say what it is", error=None,
                            candidates=[])
        if self.task is not None and not self.task.done():
            return o.refuse(c, "one thing at a time: the arm is still busy with your pick")
        if op == "pick.look":
            if o.handover:
                return o.refuse(c, "your turn is ending")
            r = o.arm.unavailable_reason()
            if r:
                return o.refuse(c, r)
            if o.busy or o.pending:
                return o.refuse(c, o.busy_reason())
            self._reset(pid)
            return self._start(pid, "look", "moving the camera over the table", self._do_look(pid))
        if self.s["pid"] != pid or self.frame is None:
            return o.refuse(c, "take a picture first (Look)")
        if op in ("pick.tap", "pick.ask"):
            if m.get("frame") != self.frame.id:
                return o.refuse(c, "that picture is out of date: look again")
            if op == "pick.tap":
                try:
                    x, y = float(m.get("x")), float(m.get("y"))
                except (TypeError, ValueError):
                    return o.refuse(c, "pick.tap needs x and y (0..1)")
                if not (0 <= x <= 1 and 0 <= y <= 1):
                    return o.refuse(c, "pick.tap: x and y are 0..1 across the picture")
                return self._start(pid, None, None, self._do_tap(pid, x, y))
            text = " ".join(str(m.get("text") or "").split())[:200]
            if not text:
                return o.refuse(c, "say what to pick up (type something)")
            return self._start(pid, None, None, self._do_ask(pid, text))
        if op == "pick.confirm":
            t = self.target
            if t is None or m.get("target") != t.id:
                return o.refuse(c, "nothing to confirm: tap the object first")
            if t.refusal:
                return o.refuse(c, f"can't pick that: {t.refusal}")
            if o.handover:
                return o.refuse(c, "your turn is ending")
            r = o.arm.unavailable_reason()
            if r:
                return o.refuse(c, r)
            if o.busy or o.pending:
                return o.refuse(c, o.busy_reason())
            return self._start(pid, "pick", f"picking up the {t.label}", self._do_pick(pid))
        return o.refuse(c, f"unknown op {op!r}")

    async def display(self, c, op: str, m: dict) -> None:
        if op == "pick.cancel":
            self.op.event("pick", "Operator cancelled Pick it up")
            self.cancel(None, "cancelled by the operator")
            if self.task is not None and not self.task.done():
                with contextlib.suppress(Exception):
                    await self.op.arm.stop()
            return
        c.send({"op": "refused", "reason": f"unknown op {op!r}"})

    def _start(self, pid: str, busy_kind: str | None, doing: str | None, coro) -> None:
        """Arm work takes the server's busy slot (one discrete action at a time); looking at a
        picture (tap / ask) doesn't move anything and runs alongside."""
        self.abort.clear()
        if busy_kind is None:
            self.task = asyncio.create_task(self._guard(pid, coro))
            return
        done = asyncio.get_running_loop().create_future()

        async def run():
            try:
                await self._guard(pid, coro, reraise=True)
            finally:
                if not done.done():
                    done.set_result(None)
        self.op._begin(pid, "pick", "Pick it up", doing, run())
        self.task = self.op.busy["task"]

    async def _guard(self, pid, coro, reraise: bool = False):
        from interfaces.wristcam.target import LocatorUnavailable, TargetRefused

        try:
            await coro
        except TargetRefused as e:
            self.set(stage="frame" if self.frame is not None else "idle", error=str(e), text="")
            self.op.status(pid, str(e), "error")
        except LocatorUnavailable as e:
            self.set(stage="frame", error=str(e), text="")
            self.op.status(pid, str(e), "error")
        except asyncio.CancelledError:
            self.set(stage="failed", error="stopped")
            raise
        except Exception as e:
            from openplay.server import ActionError
            from openplay.arm import Refused

            if isinstance(e, (ActionError, Refused)):
                self.set(stage="frame" if (self.frame is not None and self.s["stage"] != "picking") else "failed",
                         error=str(e))
                if reraise:
                    raise
                return
            log.exception("pick")
            self.set(stage="failed", error=f"something went wrong ({type(e).__name__})")
            if reraise:
                raise

    # ------------------------------------------------------------------ the work
    async def _do_look(self, pid: str) -> None:
        from interfaces.wristcam import target as TG
        from openplay.server import ActionError

        o = self.op
        cam, T_tc, wc = await asyncio.to_thread(self.calibration)
        q0 = list((o.arm.state or {}).get("q") or [])
        if len(q0) != 6:
            raise ActionError("no joint reading from the arm")
        self.set(stage="moving", text="Moving the camera over the table", error=None)
        q, h = await asyncio.to_thread(TG.look_pose, wc, cam, T_tc, q0, None, None, self.cfg)
        if q is None:
            raise ActionError("the look pose over the table is out of reach (check pick_config.json look_xy)")
        dq = float(np.max(np.abs(np.array(q) - np.array(q0))))
        dur = round(max(1.0, dq / LOOK_SPEED * 1.3), 1)
        grip = float((o.arm.state or {}).get("grip") or 0.0)
        kfs = [{"t": 0.0, "q": [round(float(x), 4) for x in q0], "grip": grip},
               {"t": dur, "q": [round(float(x), 4) for x in q], "grip": grip}]
        await o._approve(pid, "pick", "Pick it up: look over the table", dur, kfs)
        o.status(pid, "Moving the camera over the table", "busy")
        o.event("pick", f"{o.name(pid)} is looking for something to pick up", o.name(pid))
        await o.arm.do("move_joints", q=[round(float(x), 4) for x in q], speed=LOOK_SPEED, timeout=60.0)
        self.set(stage="moving", text="Taking a picture")
        lf = await self._still()
        self.frame = lf
        msg = self._frame_msg("still", lf.image, PHONE_FRAME_W)
        o.to_pid(pid, msg)
        o.to_displays(msg)
        self.set(stage="frame", text="Tap the thing to pick up, or say what it is", error=None)
        o.status(pid, "Tap what the arm should pick up", "done")

    async def _still(self):
        """A frame and the q it was taken at: the arm must be still across the grab."""
        from interfaces.wristcam.target import LookFrame
        from openplay.server import ActionError

        for _ in range(3):
            await asyncio.sleep(self.settle_s)
            q1 = list((self.op.arm.state or {}).get("q") or [])
            try:
                img = await asyncio.to_thread(self.frame_source)
            except Exception as e:
                raise ActionError(f"no picture from the wrist camera ({type(e).__name__}: {e})") from None
            q2 = list((self.op.arm.state or {}).get("q") or [])
            if len(q1) == 6 and len(q2) == 6 and float(np.max(np.abs(np.array(q1) - np.array(q2)))) < 0.005:
                return LookFrame(img, [float(x) for x in q2])
        raise ActionError("the arm kept moving while taking the picture")

    async def _do_tap(self, pid: str, x: float, y: float) -> None:
        from interfaces.wristcam import target as TG

        lf = self.frame
        uv = (x * lf.size[0], y * lf.size[1])
        self.set(stage="finding", text="Finding the edges of what you tapped", error=None, candidates=[])
        req = TG.TargetRequest(lf.id, lf.q, "tap", uv)
        await self._resolve(pid, req)

    async def _resolve(self, pid: str, req) -> None:
        from interfaces.wristcam import target as TG

        lf = self.frame
        cam, T_tc, wc = await asyncio.to_thread(self.calibration)
        view = lf.view(cam, T_tc, wc.T_base_table)
        t = await asyncio.wait_for(asyncio.to_thread(TG.resolve, lf.image, view, req, wc, self.cfg),
                                   RESOLVE_TIMEOUT_S)
        if self.frame is not lf:
            return
        self.target = t
        if t.refusal:
            self.set(stage="target", text=f"Found it, but it's {t.refusal}", error=None)
        else:
            self.set(stage="target", text=f"Pick this? ({t.label}, {t.width * 100:.1f} cm across, "
                                          f"{t.height * 100:.0f} cm tall)", error=None)
        self.op.event("pick", f"{self.op.name(pid)} chose the {t.label} at table "
                              f"({t.xy_table[0]:+.3f}, {t.xy_table[1]:+.3f}) m", self.op.name(pid))

    async def _do_ask(self, pid: str, text: str) -> None:
        from interfaces.wristcam import target as TG

        lf = self.frame
        self.set(stage="finding", text=f'Asking Claude to find "{text}"', error=None, candidates=[])
        self.op.event("pick", f'{self.op.name(pid)} asked for "{text}"', self.op.name(pid))
        if self.locator is None:
            self.locator = TG.ClaudeLocator(model=self.cfg.model or None)
        ans = await asyncio.wait_for(asyncio.to_thread(self.locator.locate, lf.image, text), ASK_TIMEOUT_S)
        if self.frame is not lf:
            return
        if not ans.found:
            w, h = lf.size
            cands = [{"x": round(cd["point"][0] / w, 4), "y": round(cd["point"][1] / h, 4), "label": cd["label"]}
                     for cd in ans.candidates]
            why = ans.reason or "couldn't find it"
            self.set(stage="frame", candidates=cands, error=None,
                     text=(f"Which one? {why}: tap one" if cands else f"Claude {why[0].lower() + why[1:]}: tap it instead"))
            return
        req = TG.request_from_answer(ans, lf.id, lf.q, text)
        self.set(stage="finding", text=f"Claude found {ans.label or 'it'}: finding its edges")
        await self._resolve(pid, req)

    async def _do_pick(self, pid: str) -> None:
        from interfaces.wristcam import target as TG
        from openplay.server import ActionError

        o = self.op
        t = self.target
        cam, T_tc, wc = await asyncio.to_thread(self.calibration)
        await o._approve(pid, "pick", f"Pick it up: the {t.label} ({t.width * 100:.1f} cm)", 60.0, [])
        if self.abort.is_set():
            raise ActionError("Stopped")
        self.set(stage="picking", text=f"Picking up the {t.label}", error=None, progress=[], result=None)
        o.status(pid, f"Picking up the {t.label}", "busy")
        o.event("pick", f"The arm is picking up {o.name(pid)}'s {t.label}", o.name(pid))
        loop = asyncio.get_running_loop()

        def live(img):
            with contextlib.suppress(Exception):
                msg = {"op": "pick.frame", "kind": "live", "id": None, "w": int(img.shape[1]), "h": int(img.shape[0]),
                       "jpeg": jpeg_data_url(img, LIVE_FRAME_W, 70)}
                loop.call_soon_threadsafe(o.to_displays, msg)

        arm = BridgeArm(o.arm, loop, self.frame_source, self.abort, self.settle_s, self.grip_settle_s, on_frame=live)

        def on_event(state, event, e):
            text = None
            if event == "transition" and e.get("to") in STAGE_TEXT:
                text, stage = STAGE_TEXT[e["to"]], e["to"]
            elif event == "decision":
                d = e.get("decision")
                stage = state
                text = {"closer": "🔎 Moving closer", "converged": "✅ Right above it", "holding": "🤏 Got a grip",
                        "verified": "🎉 Lifted it!", "lost": "🤔 Lost sight of it", "miss": "😅 Missed",
                        "abort_descent": "↩️ Off centre: backing up to re-centre"}.get(d)
            if text:
                loop.call_soon_threadsafe(self.progress, text, stage)

        res = await asyncio.wait_for(asyncio.to_thread(TG.run_pick, arm, t, cam, T_tc, wc, self.cfg, None, on_event),
                                     PICK_TIMEOUT_S)
        lifted = "verify" in res.get("trace", []) and "place" in res.get("trace", [])
        summary = {"ok": bool(res.get("ok")), "lifted": lifted, "reason": res.get("reason", ""),
                   "trace": res.get("trace"), "observations": res.get("observations")}
        if res.get("ok"):
            self.set(stage="done", result=summary, text=f"Picked up the {t.label} and put it back 🎉")
            o.status(pid, f"Picked up the {t.label}!", "done")
            o.event("pick", f"The arm picked up {o.name(pid)}'s {t.label} 🎉", o.name(pid))
        else:
            why = "stopped" if self.abort.is_set() else res.get("reason") or "it didn't work"
            self.set(stage="failed", result=summary, text=f"Couldn't pick it up: {why}")
            o.status(pid, f"Couldn't pick it up: {why}", "error")
            o.event("pick", f"Pick it up failed: {why}", o.name(pid))
        self.target = None
