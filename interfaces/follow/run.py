"""interfaces.follow.run: follow an object with the wrist camera, through the armctl daemon.

    # rehearse: mock daemon + synthetic camera (a cup moving in a circle), nothing real
    .venv/bin/python -m armctl.daemon --backend mock --auto-approve
    .venv/bin/python -m interfaces.follow.run --source synthetic --sim-path circle --expect 0.0,-0.12 --duration 20

    # real arm, step 1: track only (no lease, nothing moves), arm parked at a look pose
    .venv/bin/python -m interfaces.follow.run --source dashboard --finder cup --dry-run --duration 30

    # real arm, step 2: follow (asks y/N before the look move and before following)
    .venv/bin/python -m interfaces.follow.run --source dashboard --finder cup --expect <x,y> --v-max 0.03

What it does:
1. Connects to the daemon. Without ``--dry-run`` it asks for the lease (approve "follow" in the
   console). On the real arm it refuses unless ``interfaces/grasp/workcell.json`` is calibrated.
2. With ``--expect x,y`` (table frame), it moves the camera straight down over that point
   (``look.look_pose``, one ``move_joints`` at 0.2 rad/s, confirmed at the terminal on a real arm).
3. Waits for the arm to be still, takes a frame and runs the slow finder over all of it. If it
   doesn't find the object, it stops there.
4. Follows: every new frame goes to the tracker (q at the frame's capture time = arrival time -
   ``--latency``), the slow finder re-anchors about once a second in a thread, and the control
   loop (30 Hz) streams joint velocities (``FollowController``) with a 200 ms deadman.
   **Nothing is streamed** while the track is lost, the newest frame is older than 0.35 s, or the
   joint state is older than 0.3 s: the daemon's deadman stops the arm.
5. Stops after ``--duration`` s, when lost for ``--give-up`` s, or on Ctrl-C: sends ``stop`` and
   releases the lease. On the real arm, then: support the arm, ``tools/disable_all.py``, LEDs red.

Logs one JSON line per event to ``--log`` (default ``interfaces/follow/runs/<time>/run.jsonl``),
with annotated frames every ``--frames-every`` frames.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from armctl import kin

from interfaces.client import DEFAULT_URL, ArmClient, Refused
from interfaces.servo import caps_from_envelope
from interfaces.grasp.grasp import Workcell
from interfaces.wristcam.locate import Hint, View, annotate, load_view_inputs, make_finder

from .control import FollowConfig, FollowController
from .look import look_pose
from .track import AnchorWorker, QHistory, TrackConfig, Tracker

REPO = Path(__file__).resolve().parents[2]
CONTROL_HZ = 30.0
FRAME_STALE_S = 0.35
Q_STALE_S = 0.3
SIM_ADAPTERS = ("mock", "sim")


# --------------------------------------------------------------------------- frames
class LiveFrames:
    """Latest frame from a ``vision/sources.py`` spec in a thread, stamped with its estimated
    capture time (arrival - latency, monotonic clock)."""

    def __init__(self, spec: str, latency: float):
        sys.path.insert(0, str(REPO / "vision"))
        import sources  # type: ignore

        self.src = sources.open_source(spec)
        self.latency = latency
        self.frame, self.t, self.n, self.err = None, -1e9, 0, None
        self._lock = threading.Lock()
        self.done = False
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while not self.done:
            try:
                f = self.src.read()
            except Exception as e:     # keep the last error for the report; try again
                self.err = str(e)
                time.sleep(0.2)
                continue
            if f is None:
                time.sleep(0.005)
                continue
            with self._lock:
                self.frame, self.t, self.n = f, time.monotonic() - self.latency, self.n + 1

    def latest(self):
        with self._lock:
            return self.frame, self.t, self.n

    def close(self):
        self.done = True
        self.thread.join(2.0)
        try:
            self.src.close()
        except Exception:
            pass


class SynthFrames:
    """The synthetic wrist camera over a moving cup, rendered from the daemon's q (mock backend):
    the whole program runs on serenity or a laptop with no camera and no arm."""

    def __init__(self, q_getter, path: str, start, speed: float, fps: float = 12.0, size=(640, 360)):
        from interfaces.wristcam import synth
        from .sim import cup_path

        self.synth = synth
        self.cup = synth.Cup(tuple(start))
        self.cup.label = "red cup"
        self.world = synth.World(cups=[self.cup], board=False)
        self.cam = synth.TRUE_CAMERA.scaled(size)
        self.path = cup_path(path, start, speed)
        self.q_getter, self.fps = q_getter, fps
        self.frame, self.t, self.n = None, -1e9, 0
        self.t0: float | None = None       # the cup stays put until start_motion()
        self._lock = threading.Lock()
        self.done = False
        self.err = None
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while not self.done:
            t_cap = time.monotonic()
            q = self.q_getter()
            if q is None or not np.isfinite(q).all():
                time.sleep(0.05)
                continue
            self.cup.xy = self.path(0.0 if self.t0 is None else t_cap - self.t0)
            img = self.world.render(self.cam, kin.fk_vision(q) @ self.synth.TRUE_T_TIP_CAM, ss=1, seed=self.n)
            with self._lock:
                self.frame, self.t, self.n = img, t_cap, self.n + 1
            time.sleep(max(0.0, 1.0 / self.fps - (time.monotonic() - t_cap)))

    def latest(self):
        with self._lock:
            return self.frame, self.t, self.n

    def start_motion(self) -> None:
        """Start the cup's path (like a person who starts moving it once the arm follows)."""
        self.t0 = time.monotonic()

    def true_xy(self):
        return tuple(self.cup.xy)

    def close(self):
        self.done = True
        self.thread.join(2.0)


# --------------------------------------------------------------------------- the program
async def _confirm(what: str, needed: bool) -> None:
    """y/N at the terminal (real arm). In a thread, so the daemon connection stays alive while
    the person at the arm reads it."""
    if not needed:
        return
    a = await asyncio.to_thread(input, f"\n>>> {what}\n    type y + Enter to do it, anything else aborts: ")
    if a.strip().lower() != "y":
        raise SystemExit(f"operator declined: {what}")


def _adapter(arm: ArmClient) -> str:
    if arm.backend == "mock":
        return "mock"
    return str(arm.state.get("adapter") or arm.hello.get("adapter") or "real")


class Log:
    def __init__(self, path: Path):
        self.path = path
        path.parent.mkdir(parents=True, exist_ok=True)
        self.f = path.open("w")
        self.t0 = time.monotonic()

    def __call__(self, event: str, **kw) -> None:
        e = {"t": round(time.monotonic() - self.t0, 4), "event": event}
        for k, v in kw.items():
            if isinstance(v, np.ndarray):
                v = np.round(v, 5).tolist()
            elif isinstance(v, (np.floating, np.integer)):
                v = v.item()
            e[k] = v
        self.f.write(json.dumps(e, default=float) + "\n")
        self.f.flush()

    def close(self):
        self.f.close()


async def run(a: argparse.Namespace) -> int:
    synthetic = a.source == "synthetic"
    if synthetic:
        from interfaces.wristcam import synth
        cam, T_tc = synth.TRUE_CAMERA.scaled((640, 360)), synth.TRUE_T_TIP_CAM
        wc = Workcell(T_base_table=synth.T_BASE_TABLE_TRUE.tolist(), calibrated=True)
        notes = ["synthetic camera and table (interfaces.wristcam.synth): the mock backend only"]
    else:
        cam, T_tc, notes = load_view_inputs()
        wc = Workcell.load()
        if getattr(wc, "provisional", False):
            notes.append("workcell calibration is PROVISIONAL (hand-eye precision gates missed): expect a few mm")
    obj = (wc.objects or {}).get(a.object, {}) if not synthetic else {"width": 0.06, "height": 0.08}
    height = a.height if a.height is not None else float(obj.get("height", 0.08))
    size = a.size if a.size is not None else float(obj.get("width", 0.07))
    prompt_detector = None
    if a.prompt:                     # follow whatever is named: the open-vocabulary detector anchors the track
        from interfaces.wristcam import detect as D
        if a.detector == "synth" and not synthetic:
            raise SystemExit("--detector synth reads the synthetic world: use it with --source synthetic")
        prompt_detector = None if a.detector == "synth" else D.make_detector(a.detector)
        finder = None                # built at the seed, from the named object's footprint
    elif a.finder == "cup":
        from interfaces.wristcam.locate import CupFinder
        finder = CupFinder(diameter=size, height=height)
    else:
        finder = make_finder(a.finder)
    tcfg = TrackConfig(height=height, size=size, anchor_period=a.anchor_period)
    fcfg = FollowConfig(follow_h=a.follow_h, v_max=a.v_max, speed_scale=a.speed_scale, max_tilt_deg=a.max_tilt,
                        ff_gain=0.0 if a.no_ff else 1.0)
    run_dir = Path(a.log).parent if a.log else REPO / "interfaces" / "follow" / "runs" / time.strftime("%Y%m%d-%H%M%S")
    log = Log(Path(a.log) if a.log else run_dir / "run.jsonl")
    frames_dir = run_dir / "frames"
    if a.frames_every:
        frames_dir.mkdir(parents=True, exist_ok=True)
    for n in notes:
        print("note:", n, flush=True)
    log("start", args=vars(a), notes=notes, track=tcfg.__dict__, follow=fcfg.__dict__)

    qh = QHistory(keep_s=3.0)
    async with ArmClient(a.url, program="follow", name=f"follow {a.object} ({a.finder})") as arm:
        await arm.wait_state(3.0)
        adapter = _adapter(arm)
        real = adapter not in SIM_ADAPTERS
        print(f"connected: backend {arm.backend}, adapter {adapter}{' (REAL ARM)' if real else ''}", flush=True)
        if synthetic and arm.backend != "mock":
            raise SystemExit("--source synthetic renders a made-up table: use it with the mock backend only")
        if real and not wc.calibrated:
            raise SystemExit("follow is off on the real arm until interfaces/grasp/workcell.json is calibrated "
                             "(interfaces/wristcam/README.md, hand-eye)")
        frames = SynthFrames(lambda: qh.at(time.monotonic()), a.sim_path, a.expect or (0.0, -0.12), a.sim_speed) \
            if synthetic else LiveFrames(a.source, a.latency)
        stop = asyncio.Event()

        async def record_q():
            last = None
            while not stop.is_set():
                st = arm.state
                if st is not last and st.get("q"):
                    qh.add(time.monotonic(), arm.q)
                    last = st
                await asyncio.sleep(0.005)

        qtask = asyncio.create_task(record_q())
        try:
            if not a.dry_run:
                print("asking for the lease: approve 'follow' in the console", flush=True)
                await arm.acquire()
            # 2. the look pose
            if a.expect is not None and not a.dry_run:
                q_look = look_pose(a.expect, height, a.follow_h, T_tc, wc.T_base_table, seed=arm.q)
                if q_look is None:
                    raise SystemExit(f"no camera-down look pose over table {a.expect} at {a.follow_h} m")
                tip = kin.tip(q_look, kin.vision_correction())
                await _confirm(f"MOVE to the look pose over table {a.expect}: q={np.round(q_look, 3).tolist()}, "
                         f"tip ({tip[0]:.3f}, {tip[1]:.3f}, {tip[2]:.3f}) m, 0.2 rad/s", real)
                log("look_move", q=q_look)
                await arm.do("move_joints", q=[round(float(x), 4) for x in q_look], speed=0.2, timeout=120)
            # 3. seed: a still arm, a fresh frame, the slow finder over all of it
            await asyncio.sleep(a.settle)
            t_still = time.monotonic()
            while True:
                f, ft, _ = frames.latest()
                if f is not None and ft > t_still:
                    break
                if time.monotonic() - t_still > 5:
                    raise SystemExit(f"no frame from {a.source} in 5 s{': ' + frames.err if frames.err else ''}")
                await asyncio.sleep(0.02)
            q0 = qh.at(ft)
            view0 = View.from_q(q0, cam.for_image(f), T_tc, wc.T_base_table)
            # near --expect if given (another identical object elsewhere must not be taken for it)
            hint = Hint(tuple(a.expect), a.seed_radius) if a.expect is not None else None
            if a.prompt:
                from interfaces.wristcam import target as TG
                if prompt_detector is None:      # --detector synth: ground truth from the synthetic world
                    from interfaces.wristcam import detect as D
                    prompt_detector = D.SynthDetector(frames.world)
                try:
                    tgt, cands = await asyncio.to_thread(TG.resolve_prompt, f, view0, a.prompt, prompt_detector, wc)
                except TG.TargetRefused as e:
                    print(f"can't follow {a.prompt!r}: {e}", flush=True)
                    log("end", reason=str(e))
                    return 2
                height, size = float(tgt.height), float(max(tgt.size))
                tcfg.height, tcfg.size = height, size
                finder = TG.DetectorFinder(tgt, prompt_detector, a.prompt, fallback=False, gate=0.06)
                log("prompt", target=tgt.to_dict(), candidates=cands)
            det = await asyncio.to_thread(finder.find, f, view0, hint)
            if det is None:
                cv2.imwrite(str(run_dir / "seed.jpg"), annotate(f, view0, None, "seed: not found"))
                print(f"the {a.finder} finder found no {a.object} in the first frame ({run_dir / 'seed.jpg'})", flush=True)
                log("end", reason="not found at the start")
                return 2
            trk = Tracker(tcfg)
            trk.t_frame = ft
            print(f"found {det.kind} at table ({det.xyz_table[0]:+.3f}, {det.xyz_table[1]:+.3f}) "
                  f"+-{det.sigma_xy * 1000:.1f} mm", flush=True)
            log("seed", det=det.to_dict(), q=q0, result=trk.anchor(det, f, view0, ft))
            cv2.imwrite(str(run_dir / "seed.jpg"), annotate(f, view0, det, "seed"))
            anchor = AnchorWorker(finder, threaded=True)
            ctrl = None
            if not a.dry_run:
                vcap, acap = caps_from_envelope(arm.envelope)
                ctrl = FollowController(T_tc, wc.T_base_table, vcap, acap, fcfg)
                await _confirm(f"FOLLOW: the arm moves by itself to keep the camera {a.follow_h:.2f} m over the {a.object}, "
                         f"camera at most {a.v_max:.2f} m/s, joints at most {a.speed_scale:.0%} of the caps, for "
                         f"{a.duration:.0f} s. Ctrl-C stops (the deadman stops the arm in 0.2 s).", real)
            t_start = time.monotonic()
            if synthetic:
                frames.start_motion()
            shared = {"lost_since": None, "sent": 0, "held": 0, "why_hold": ""}

            async def perceive():
                last_n = -1
                while not stop.is_set():
                    f, ft, n = frames.latest()
                    if f is None or n == last_n:
                        await asyncio.sleep(0.003)
                        continue
                    last_n = n
                    q = qh.at(ft)
                    if q is None:
                        continue
                    view = View.from_q(q, cam.for_image(f), T_tc, wc.T_base_table)
                    m = await asyncio.to_thread(trk.update, f, view, ft)
                    r = anchor.poll()
                    if r is not None:
                        ta, d, fa, va = r
                        res = trk.anchor(d, fa, va, ta)
                        log("anchor", result=res, det=d.to_dict() if d else None, ms=round(anchor.last_ms, 1))
                    if trk.wants_anchor(ft) and not anchor.busy:
                        anchor.submit(f, view, ft, trk.hint(ft))
                    log("frame", n=n, state=trk.state, match=None if m is None else m.xy, score=None if m is None else
                        round(m.score, 3), ms=None if m is None else round(m.ms, 2),
                        true=frames.true_xy() if synthetic else None)
                    if a.frames_every and n % a.frames_every == 0:
                        est = trk.estimate(ft)
                        dets = []
                        if est is not None:
                            from interfaces.wristcam.locate import Detection
                            dets = [Detection("track", view.table_to_base([[*est.xy, height]])[0].tolist(),
                                              [*est.xy, height], sigma_xy=est.sigma)]
                        cv2.imwrite(str(frames_dir / f"f{n:05d}.jpg"),
                                    annotate(f, view, dets[0] if dets else None, f"{trk.state} n={n}"))

            async def control():
                dt = 1.0 / CONTROL_HZ
                while not stop.is_set():
                    await asyncio.sleep(dt)
                    now = time.monotonic()
                    if a.duration and now - t_start > a.duration:
                        stop.set()
                        break
                    est = trk.estimate(now + a.lead)
                    _, ft, _ = frames.latest()
                    q_t = qh.newest_t
                    why = ""
                    if est is None:
                        why = f"track {trk.state}"
                    elif now - ft > FRAME_STALE_S:
                        why = f"frame {now - ft:.2f} s old"
                    elif q_t is None or now - q_t > Q_STALE_S:
                        why = "joint state stale"
                    if why:
                        shared["why_hold"] = why
                        if est is None:
                            shared["lost_since"] = shared["lost_since"] or now
                            if now - shared["lost_since"] > a.give_up:
                                print(f"lost for {a.give_up:.0f} s: stopping", flush=True)
                                stop.set()
                        if ctrl is not None:
                            ctrl.reset()      # send nothing: the deadman stops the arm
                        shared["held"] += 1
                        continue
                    shared["lost_since"] = None
                    shared["why_hold"] = ""
                    if ctrl is None:
                        continue
                    q = arm.q
                    qd, info = ctrl.step(q, [*est.xy, height], dt, v_table=est.v)
                    try:
                        await arm.stream_qd(qd, ttl_ms=200)
                    except Exception as e:
                        print(f"stream failed: {e}", flush=True)
                        stop.set()
                        break
                    shared["sent"] += 1
                    log("control", est=est.xy, v=est.v, sigma=est.sigma, age=est.age, qd=qd, q=q, **info)

            async def report():
                while not stop.is_set():
                    await asyncio.sleep(a.print_every)
                    est = trk.estimate(time.monotonic())
                    line = f"t={time.monotonic() - t_start:5.1f}s {trk.state:9s}"
                    if est is not None:
                        line += f" at ({est.xy[0]:+.3f}, {est.xy[1]:+.3f}) v {np.linalg.norm(est.v) * 100:4.1f} cm/s"
                    if synthetic:
                        tx, ty = frames.true_xy()
                        line += f" true ({tx:+.3f}, {ty:+.3f})"
                        if ctrl is not None:
                            line += f" centring {ctrl.centring_error(arm.q, [tx, ty, height]) * 1000:5.1f} mm"
                    if shared["why_hold"]:
                        line += f" HOLD ({shared['why_hold']})"
                    for r in arm.refusals[-1:]:
                        line += f" | refused: {r.get('reason')}"
                    print(line, flush=True)

            tasks = [asyncio.create_task(x()) for x in (perceive, control, report)]
            try:
                await stop.wait()
            finally:
                for t in tasks:
                    t.cancel()
                await asyncio.gather(*tasks, return_exceptions=True)
                if not a.dry_run:
                    await arm.stop()
                await asyncio.to_thread(anchor.join)
            summary = {"track": trk.summary(), "control_ticks_sent": shared["sent"], "held_ticks": shared["held"],
                       "refusals": [r.get("reason") for r in arm.refusals[-5:]], "anchors": anchor.n,
                       "frames": frames.latest()[2]}
            log("end", **summary)
            print(json.dumps(summary, indent=1, default=float), flush=True)
            print(f"log: {log.path}", flush=True)
            if real and not a.dry_run:
                print("REAL ARM: support it, then .venv/bin/python -I tools/disable_all.py can0 and check every LED is "
                      "steady red (or fold home first).", flush=True)
        finally:
            stop.set()
            qtask.cancel()
            frames.close()
            log.close()
    return 0


def _xy(s: str) -> tuple:
    x, y = (float(v) for v in s.split(","))
    return (x, y)


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default=DEFAULT_URL)
    ap.add_argument("--source", default="dashboard", help="vision/sources.py spec (dashboard, /dev/video0, URL) or 'synthetic'")
    ap.add_argument("--finder", default="cup", choices=("cup", "tags", "blob"), help="the slow finder that anchors the track")
    ap.add_argument("--prompt", default="", help="follow whatever this names (open-vocabulary detector; overrides --finder)")
    ap.add_argument("--detector", default="yoloe", choices=("yoloe", "claude", "synth"),
                    help="with --prompt (synth: the synthetic world's ground truth, --source synthetic only)")
    ap.add_argument("--object", default="cup", help="workcell object (height, width)")
    ap.add_argument("--height", type=float, default=None, help="object top above the table, m (default: workcell)")
    ap.add_argument("--size", type=float, default=None, help="object width across its top, m (default: workcell)")
    ap.add_argument("--expect", type=_xy, default=None, help="table x,y to look at first (moves the arm there)")
    ap.add_argument("--seed-radius", type=float, default=0.06, help="with --expect: look for the object within this of it, m")
    ap.add_argument("--follow-h", type=float, default=0.15, help="camera height above the object's top, m")
    ap.add_argument("--v-max", type=float, default=0.03, help="camera speed cap, m/s (sim: 0.06)")
    ap.add_argument("--speed-scale", type=float, default=0.5, help="fraction of the daemon's joint caps")
    ap.add_argument("--max-tilt", type=float, default=25.0, help="camera tilt from straight down when looking at the object, deg")
    ap.add_argument("--no-ff", action="store_true", help="no velocity feedforward")
    ap.add_argument("--latency", type=float, default=0.12, help="camera latency (capture -> here), s; tune from the log")
    ap.add_argument("--lead", type=float, default=0.05, help="aim this far ahead of the newest estimate, s")
    ap.add_argument("--anchor-period", type=float, default=1.0)
    ap.add_argument("--settle", type=float, default=1.0, help="s to wait still before the first frame")
    ap.add_argument("--duration", type=float, default=45.0, help="s (j4 warms while it holds a look pose: keep runs short)")
    ap.add_argument("--give-up", type=float, default=5.0, help="stop after this long lost, s")
    ap.add_argument("--dry-run", action="store_true", help="track only: no lease, nothing moves")
    ap.add_argument("--frames-every", type=int, default=0, help="save every Nth annotated frame")
    ap.add_argument("--print-every", type=float, default=1.0)
    ap.add_argument("--log", default=None, help="run.jsonl path (default interfaces/follow/runs/<time>/run.jsonl)")
    ap.add_argument("--sim-path", default="circle", help="synthetic: cup path (static line circle stopgo jump vanish)")
    ap.add_argument("--sim-speed", type=float, default=0.03, help="synthetic: cup speed, m/s")
    return ap


def main(argv=None) -> int:
    a = build_parser().parse_args(argv)
    try:
        return asyncio.run(run(a))
    except KeyboardInterrupt:
        print("stopped (Ctrl-C): the deadman stops the arm", flush=True)
        return 130
    except Refused as e:
        print(f"refused by the daemon: {e}", flush=True)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
