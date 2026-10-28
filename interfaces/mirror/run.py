"""Pose mirroring: a camera watches a person, the arm copies their arm, live.

    # no camera, no daemon: print what the arm would do for a fake waving person
    .venv/bin/python -m interfaces.mirror.run --source synthetic --dry-run --duration 5
    # one photo, write an overlay to look at
    .venv/bin/python -m interfaces.mirror.run --source photo.jpg --dry-run --overlay /tmp/overlay.jpg
    # live against the armctl daemon (mock or real), USB camera, follow the right arm
    .venv/bin/python -m interfaces.mirror.run --source /dev/video0 --side right
    # phone camera through the lab console, IK "hand" mode, only while the left hand is up
    .venv/bin/python -m interfaces.mirror.run --source phone:front --mode hand --clutch

Pipeline: frame -> MediaPipe pose -> retarget (joints | hand) -> make_safe (soft box, table)
-> One Euro filter -> servo (P on joint error, capped by the daemon's speed/accel caps) ->
``stream qd`` with a 200 ms deadman. No person, a hidden arm or a lowered clutch hand: we stop
sending and the daemon brings the arm to rest. ``--record`` saves what the arm actually did
as a recording the timeline tool can turn into a playable.

The daemon still gates everything (lease, envelope, E-STOP). On the real arm the operator
approves the "mirror" program in the console, and the person stays out of the arm's reach.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import sys
import threading
import time
from pathlib import Path

import numpy as np

from armctl import envelope as envm

from ..client import DEFAULT_URL, ArmClient, Refused
from ..servo import OneEuro, TargetServo, caps_from_envelope, table_guard
from .pose import PoseTracker, SyntheticBody, draw
from .retarget import Mapping, Retargeter, clutch_engaged

REPO = Path(__file__).resolve().parents[2]
VIDEO_EXT = (".mp4", ".mov", ".avi", ".mkv", ".webm")
CONTROL_HZ = 30.0
STALE_S = 0.3


class Frames:
    """Latest-frame grabber in a thread over vision/sources.py specs, video files or 'synthetic'."""

    def __init__(self, spec: str):
        self.spec = spec
        self.synthetic = spec == "synthetic"
        self.frame: np.ndarray | None = None
        self.t = 0.0
        self.n = 0
        self.done = False
        if self.synthetic:
            return
        if spec.lower().endswith(VIDEO_EXT):
            import cv2
            cap = cv2.VideoCapture(spec)
            if not cap.isOpened():
                raise SystemExit(f"can't open video {spec}")
            fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
            self._read = lambda: (cap.read()[1], time.sleep(1.0 / fps))[0]
            self._close = cap.release
        else:
            sys.path.insert(0, str(REPO / "vision"))
            import sources  # type: ignore

            src = sources.open_source(spec)
            self._read, self._close = src.read, src.close
        self._lock = threading.Lock()
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        still = Path(self.spec).suffix.lower() in (".jpg", ".jpeg", ".png")
        while not self.done:
            f = self._read()
            if f is None:
                if still and self.frame is not None:
                    time.sleep(0.03)
                    with self._lock:
                        self.t = time.monotonic()
                    continue
                if Path(self.spec).exists():
                    self.done = True
                    break
                time.sleep(0.01)
                continue
            with self._lock:
                self.frame, self.t, self.n = f, time.monotonic(), self.n + 1

    def latest(self):
        if self.synthetic:
            return None, time.monotonic()
        with self._lock:
            return self.frame, self.t

    def close(self):
        self.done = True
        if not self.synthetic:
            self._close()


async def run(a: argparse.Namespace) -> int:
    frames = Frames(a.source)
    tracker = None if a.source == "synthetic" else PoseTracker(a.model)
    synth = SyntheticBody() if a.source == "synthetic" else None
    m = Mapping(wrist=a.wrist)
    rt = Retargeter(mode=a.mode, side=a.side, m=m)
    filt = OneEuro(min_cutoff=a.smooth, beta=0.6)
    shared = {"target": None, "t": 0.0, "info": {}, "clutch": not a.clutch, "body": None}
    t0 = time.monotonic()
    stop = asyncio.Event()

    async def perceive():
        last_seen = -1.0
        while not stop.is_set():
            now = time.monotonic()
            if a.duration and now - t0 > a.duration:
                stop.set()
                break
            if synth is not None:
                body = synth(now - t0)
                await asyncio.sleep(1 / 30)
            else:
                frame, ft = frames.latest()
                if frame is None or ft == last_seen:
                    if frames.done:
                        stop.set()
                        break
                    await asyncio.sleep(0.005)
                    continue
                last_seen = ft
                body = await asyncio.to_thread(tracker.detect, frame, ft - t0)
                if a.overlay:
                    shared["frame"] = frame
            q, info = rt(body)
            shared["body"] = body
            shared["clutch"] = (not a.clutch) or (body is not None and clutch_engaged(body, a.side))
            if q is not None:
                shared["target"] = filt(q, now)
                shared["t"] = now
            else:
                filt.reset()
            shared["info"] = info

    async def report():
        while not stop.is_set():
            await asyncio.sleep(a.print_every)
            tg = shared["target"]
            fresh = tg is not None and time.monotonic() - shared["t"] < STALE_S
            info = shared["info"]
            line = (f"t={time.monotonic() - t0:5.1f}s target={'-' if tg is None else np.round(tg, 2).tolist()} "
                    f"{'' if fresh else '(stale) '}{'' if shared['clutch'] else '(clutch up: holding) '}"
                    + " ".join(f"{k}={v}" for k, v in info.items() if k in ("why", "adjusted", "guard", "elev", "yaw", "opening")))
            if arm is not None and arm.state:
                line += f" q={np.round(arm.q, 2).tolist()}"
            print(line, flush=True)

    arm: ArmClient | None = None
    recording: list[dict] = []
    if a.dry_run:
        tasks = [asyncio.create_task(perceive()), asyncio.create_task(report())]
        await stop.wait()
        for t in tasks:
            t.cancel()
    else:
        async with ArmClient(a.url, program="mirror", name=f"pose mirror ({a.mode}, {a.side} arm)") as arm:
            print(f"connected: backend {arm.backend}; asking for the lease (approve 'mirror' in the console)", flush=True)
            await arm.acquire()
            vcap, acap = caps_from_envelope(arm.envelope, a.speed)
            servo = TargetServo(vcap, acap, gain=a.gain)
            await arm.wait_state()
            if np.abs(arm.q - np.array(envm.POSES["ready"])).max() > 0.05:
                print("moving to ready first", flush=True)
                await arm.do("ready", timeout=120)
            tasks = [asyncio.create_task(perceive()), asyncio.create_task(report())]
            dt = 1.0 / CONTROL_HZ
            next_rec = 0.0
            try:
                while not stop.is_set():
                    await asyncio.sleep(dt)
                    now = time.monotonic()
                    if a.record and now >= next_rec:
                        recording.append({"t": round(now - t0, 3), "q": [round(float(x), 4) for x in arm.q],
                                          "grip": arm.state.get("grip", 0.0)})
                        next_rec = now + 1.0 / a.record_hz
                    tg = shared["target"]
                    if tg is None or now - shared["t"] > STALE_S or not shared["clutch"]:
                        servo.reset()   # send nothing: the deadman stops the arm
                        continue
                    qd = servo.step(arm.q, tg, dt)
                    qd, guarded = table_guard(arm.q, qd)
                    if guarded:
                        servo.qd = qd
                        shared["info"]["guard"] = "table guard: held a joint that would dip"
                    try:
                        await arm.stream_qd(qd, ttl_ms=200)
                    except Exception as e:
                        print(f"stream failed: {e}", flush=True)
                        break
            finally:
                for t in tasks:
                    t.cancel()
                await arm.stop()
                for r in arm.refusals[-5:]:
                    print(f"refused: {r.get('reason')}", flush=True)
    frames.close()
    if a.overlay and shared.get("frame") is not None:
        import cv2
        info = shared["info"]
        txt = [f"{a.mode} mode, {a.side} arm", *(f"{k}: {v}" for k, v in info.items())]
        tg = shared["target"]
        if tg is not None:
            txt.append("q: " + " ".join(f"{x:+.2f}" for x in tg))
        cv2.imwrite(a.overlay, draw(shared["frame"], shared["body"], txt, a.side))
        print(f"overlay -> {a.overlay}")
    if a.record:
        out = {"format": "openyam-recording", "version": "1.0", "source": f"mirror:{a.mode}:{a.side}",
               "created": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "rate_hz": a.record_hz, "samples": recording}
        Path(a.record).write_text(json.dumps(out))
        print(f"recording ({len(recording)} samples) -> {a.record}; turn it into a playable with:\n"
              f"  .venv/bin/python -m interfaces.timeline.record {a.record} --name my-move")
    if tracker:
        tracker.close()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="synthetic", help="synthetic | /dev/video0 | phone:<name> | dashboard | http://...jpg | video file | image")
    ap.add_argument("--mode", choices=("joints", "hand"), default="joints")
    ap.add_argument("--side", choices=("right", "left"), default="right", help="which of the person's arms to copy")
    ap.add_argument("--clutch", action="store_true", help="follow only while the other hand is raised above its shoulder")
    ap.add_argument("--wrist", action="store_true", help="also copy the wrist bend (noisy)")
    ap.add_argument("--model", choices=("lite", "full", "heavy"), default="lite")
    ap.add_argument("--smooth", type=float, default=1.2, help="One Euro min cutoff, Hz (lower = smoother, laggier)")
    ap.add_argument("--speed", type=float, default=1.0, help="fraction of the daemon's speed/accel caps to use")
    ap.add_argument("--gain", type=float, default=3.0, help="servo gain, 1/s")
    ap.add_argument("--url", default=DEFAULT_URL)
    ap.add_argument("--dry-run", action="store_true", help="no daemon: just print targets")
    ap.add_argument("--duration", type=float, default=0.0, help="stop after this many seconds (0 = until Ctrl-C)")
    ap.add_argument("--record", help="save what the arm did to this recording JSON")
    ap.add_argument("--record-hz", type=float, default=20.0)
    ap.add_argument("--overlay", help="write the last frame with the skeleton drawn to this image")
    ap.add_argument("--print-every", type=float, default=0.5)
    a = ap.parse_args()
    try:
        return asyncio.run(run(a))
    except KeyboardInterrupt:
        return 0
    except (Refused, TimeoutError) as e:
        print(f"refused: {e}")
        return 2


if __name__ == "__main__":
    sys.exit(main())
