"""interfaces.wristcam.pick_anything: name an object (or nothing: "whatever is there"), see what the
camera finds, and pick it up through the same closed loop as tap-to-pick.

    # 0. no hardware: the synthetic table (red cup, green box, blue box), ground-truth detector
    .venv/bin/python -m interfaces.wristcam.pick_anything --simulate --prompt "blue box" --out /tmp/pick-anything

    # 1. lab laptop, GPU: how fast and how good is YOLO-E on our wrist camera? (nothing moves)
    .venv/bin/python -m interfaces.wristcam.pick_anything --bench 30 --prompt "cup,marker,tape,box" --source dashboard

    # 2. detect only, on a saved frame or the live camera (nothing moves): annotated image + targets
    .venv/bin/python -m interfaces.wristcam.pick_anything --image look.jpg --q 0.37,1.5,1.3,-0.6,0,0 --prompt "the tape roll"
    .venv/bin/python -m interfaces.wristcam.pick_anything --source dashboard --prompt "marker"      # q read from the daemon

    # 3. REAL ARM: pick it up (each motion asks y/N; the daemon validates every one)
    .venv/bin/python -m interfaces.wristcam.pick_anything --source dashboard --prompt "marker" --execute

Detectors (``--detector``): ``yoloe`` (default; DimOS's YOLO-E, open-vocabulary detect + segment,
GPU when there is one) or ``claude`` (Claude vision box + GrabCut; slow, for prompts YOLO-E can't
ground). A comma list in ``--prompt`` asks for several names at once (bench and detect); ``--execute``
picks the best match of the first. No ``--prompt``: everything the prompt-free model sees.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from interfaces.grasp.grasp import Workcell

from . import detect as D
from . import target as T
from .locate import View, load_view_inputs

REPO = Path(__file__).resolve().parents[2]
COLORS = [(66, 135, 245), (80, 200, 120), (245, 166, 35), (200, 80, 200), (60, 60, 230), (230, 220, 60)]


def _floats(s):
    return [float(x) for x in s.split(",")]


def draw(frame, dets, best: T.Target | None, view: View | None, note: str = "") -> np.ndarray:
    """Masks + labels for every detection, the chosen target's footprint and closing line."""
    out = frame.copy()
    for k, d in enumerate(dets):
        c = COLORS[k % len(COLORS)]
        tint = np.zeros_like(out)
        tint[d.mask > 0] = c
        out = cv2.addWeighted(out, 1.0, tint, 0.35, 0)
        cnts, _ = cv2.findContours((d.mask > 0).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(out, cnts, -1, c, 2)
        u0, v0 = int(d.box[0]), int(d.box[1])
        cv2.putText(out, f"{d.label} {d.score:.2f}", (u0, max(v0 - 6, 14)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2, cv2.LINE_AA)
    if best is not None and view is not None:
        P = np.c_[np.asarray(best.footprint_table, float), np.full(len(best.footprint_table), best.height)]
        uv = view.project_table(P)
        if np.isfinite(uv).all():
            cv2.polylines(out, [np.round(uv).astype(np.int32)], True, (255, 255, 255), 2, cv2.LINE_AA)
        a = best.grasp_yaw_table
        e = 0.5 * (best.width + 0.02) * np.array([np.cos(a), np.sin(a)])
        ends = view.project_table([[best.xy_table[0] - e[0], best.xy_table[1] - e[1], best.height],
                                   [best.xy_table[0] + e[0], best.xy_table[1] + e[1], best.height]])
        if np.isfinite(ends).all():
            cv2.line(out, tuple(np.round(ends[0]).astype(int)), tuple(np.round(ends[1]).astype(int)), (0, 0, 255), 3)
        txt = f"pick: {best.label} at ({best.xy_table[0]:+.3f}, {best.xy_table[1]:+.3f}) h {best.height * 100:.1f} cm"
        if best.refusal:
            txt += "  REFUSED: " + best.refusal
        note = (note + "\n" if note else "") + txt
    for k, line in enumerate([l for l in note.split("\n") if l]):
        cv2.putText(out, line, (10, 26 + 24 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    return out


def _source_frame(spec: str):
    sys.path.insert(0, str(REPO / "vision"))
    from sources import grab_one, open_source  # type: ignore
    src = open_source(spec, new_only=False) if spec == "dashboard" or spec.startswith("http") else open_source(spec)
    return src, grab_one


def bench(a, detector) -> int:
    src, grab_one = _source_frame(a.source)
    prompts = [p.strip() for p in a.prompt.split(",")] if a.prompt else None
    ms, counts = [], {}
    try:
        for k in range(a.bench):
            f = grab_one(src)
            if f is None:
                print("no frame", flush=True)
                continue
            dets = detector.detect(f, prompts)
            if k > 0:                    # the first call loads the model
                ms.append(detector.last_ms)
            for d in dets:
                counts[d.label] = counts.get(d.label, 0) + 1
            if a.out and k % 10 == 0:
                Path(a.out).mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(Path(a.out) / f"bench_{k:03d}.jpg"), draw(f, dets, None, None, f"{detector.name} {detector.last_ms:.0f} ms"))
    finally:
        src.close()
    info = {"detector": detector.name, "backend": getattr(detector, "backend", None), "device": getattr(detector, "device", None),
            "frames": a.bench, "ms_median": round(float(np.median(ms)), 1) if ms else None,
            "ms_p95": round(float(np.percentile(ms, 95)), 1) if ms else None, "seen": counts}
    print(json.dumps(info, indent=1))
    return 0


def simulate(a) -> dict:
    from . import synth
    cup = synth.Cup((-0.02, -0.20))
    green = synth.Box((0.06, -0.08), size=(0.05, 0.035, 0.04), yaw=0.5)
    blue = synth.Box((-0.05, -0.09), size=(0.04, 0.03, 0.05), yaw=-0.3, color=(190, 90, 40))
    cup.label, green.label, blue.label = "red cup", "green box", "blue box"
    world = synth.World(board=False, cups=[cup], boxes=[green, blue])
    arm = synth.SimArm(world, ss=1, joint_noise=0.0005)
    wc = Workcell(T_base_table=world.T_base_table.tolist(), calibrated=True)
    cam, T_tc = synth.TRUE_CAMERA, synth.TRUE_T_TIP_CAM
    q, _ = T.look_pose(wc, cam, T_tc, arm.q(), (0.01, -0.13), 0.25)
    arm.move_joints(q)
    frame = arm.frame()
    view = View.from_q(q, cam, T_tc, world.T_base_table)
    det = D.SynthDetector(world)
    out = Path(a.out) if a.out else None
    try:
        best, cands = T.resolve_prompt(frame, view, a.prompt or None, det, wc)
    except T.TargetRefused as e:
        print(f"refused: {e}")
        return {"ok": False, "reason": str(e)}
    D_all = det.detect(frame, [a.prompt] if a.prompt else None)
    if out:
        out.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out / "look.jpg"), draw(frame, D_all, best, view, "simulated: nothing real moved"))
    print(json.dumps({"target": best.to_dict(), "candidates": cands}, indent=1, default=float))
    res = T.run_pick(arm, best, cam, T_tc, wc, finder=T.DetectorFinder(best, D.SynthDetector(world), a.prompt or best.label),
                     log_path=out / "run.jsonl" if out else None, frames_dir=out / "frames" if out else None)
    res = {k: res[k] for k in ("ok", "reason", "trace", "estimate_table", "observations")}
    print(json.dumps(res, indent=1, default=float))
    return res


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--prompt", default="", help="what to pick ('' = whatever the prompt-free model sees); a comma list for bench/detect")
    ap.add_argument("--detector", default="yoloe", choices=("yoloe", "claude"))
    ap.add_argument("--conf", type=float, default=0.15, help="yoloe: confidence threshold")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--simulate", action="store_true", help="the synthetic table and arm (no hardware, no models)")
    mode.add_argument("--bench", type=int, default=0, metavar="N", help="time the detector on N live frames (nothing moves)")
    mode.add_argument("--execute", action="store_true", help="REAL ARM: pick it up through the armctl daemon")
    ap.add_argument("--image", default=None, help="detect on a saved frame (with --q)")
    ap.add_argument("--q", default=None, help="joint angles the frame was taken at (rad, comma separated)")
    ap.add_argument("--source", default="dashboard", help="vision/sources.py spec for live frames")
    ap.add_argument("--place", default=None, help="table x,y to put it down (default: where it was)")
    ap.add_argument("--settle", type=float, default=1.0, help="execute: seconds to settle before each frame")
    ap.add_argument("--out", default=None, help="folder for the annotated image, run.jsonl and frames")
    # grasp tuning, as in look_and_grasp (29 Oct real-arm runs)
    ap.add_argument("--grasp-frac", type=float, default=None, help="grip height / object height (pinch from the top: ~0.8)")
    ap.add_argument("--height", type=float, default=None, help="m, the object's real height (the camera can't see it from above)")
    ap.add_argument("--open-margin", type=float, default=None, help="m wider than the object when opening (workcell default 0.02)")
    ap.add_argument("--min-tip-z", type=float, default=None, help="m, lowest fingertip above the calibrated table")
    ap.add_argument("--realign-max", type=float, default=None, help="m, largest offset re-aligned during the descent")
    ap.add_argument("--no-grip-check", action="store_true", help="don't judge holding from the grip reading (DimOS echoes it)")
    ap.add_argument("--max-size", type=float, default=None, help="m: refuse detections longer than this (two objects in one box)")
    ap.add_argument("--finger-yaw", type=float, default=None, help="rad (base frame): direction the fingers close along")
    a = ap.parse_args(argv)
    if a.simulate:
        return 0 if simulate(a).get("ok") else 1
    detector = D.make_detector(a.detector, **({"conf": a.conf} if a.detector == "yoloe" else {}))
    if a.bench:
        return bench(a, detector)

    wc = Workcell.load()
    cam, T_tc, notes = load_view_inputs()
    for n in notes:
        print("note:", n)
    prompts = [p.strip() for p in a.prompt.split(",") if p.strip()] or [None]
    out = Path(a.out or REPO / "interfaces" / "wristcam" / "runs" / time.strftime("pick-%Y%m%d-%H%M%S"))
    out.mkdir(parents=True, exist_ok=True)

    def look(frame, q):
        view = View.from_q(q, cam.for_image(frame), T_tc, wc.T_base_table)
        best, cands, err = None, [], None
        try:
            best, cands = T.resolve_prompt(frame, view, prompts[0], detector, wc, max_size=a.max_size)
        except T.TargetRefused as e:
            err = str(e)
        dets = detector.detect(frame, [p for p in prompts if p] or None)
        cv2.imwrite(str(out / "look.jpg"), draw(frame, dets, best, view, err or f"{detector.name}: {len(dets)} found"))
        print(json.dumps({"q": [round(float(x), 4) for x in q], "target": best.to_dict() if best else None,
                          "candidates": cands, "refused": err, "detector_ms": round(detector.last_ms, 1)}, indent=1, default=float))
        print(f"annotated: {out / 'look.jpg'}")
        return best, view

    if a.image:
        if not a.q:
            sys.exit("--image needs --q (the joint angles the frame was taken at)")
        frame = cv2.imread(a.image, cv2.IMREAD_COLOR)
        look(frame, _floats(a.q))
        return 0
    if not a.execute:
        from .engine import read_q_from_daemon
        q = read_q_from_daemon()
        src, grab_one = _source_frame(a.source)
        try:
            frame = grab_one(src)
        finally:
            src.close()
        look(frame, q)
        print("dry run: nothing moved. Add --execute to pick it up.")
        return 0

    refusal = T.calibration_refusal(wc)
    if refusal:
        sys.exit(f"refusing --execute: {refusal}")
    from .engine import DaemonArm
    if a.min_tip_z is not None:
        wc.min_tip_z = a.min_tip_z
    if a.open_margin is not None:
        wc.open_margin = a.open_margin
    gkw = {k: v for k, v in (("grasp_frac", a.grasp_frac), ("realign_max", a.realign_max),
                              ("finger_yaw", a.finger_yaw)) if v is not None}
    if a.no_grip_check:
        gkw["grip_check"] = False
    with DaemonArm(source=a.source, program="pick-anything") as arm:
        time.sleep(a.settle)
        best, view = look(arm.frame(), arm.q())
        if best is not None and a.height is not None:
            best.height, best.height_known = float(a.height), True
        if np.any(arm.q()[1:3] < 0.1):   # folded: every planned look fails the path check (29 Oct)
            arm.ready()
        if best is None:
            return 2
        if best.refusal:
            print(f"won't pick it: {best.refusal}")
            return 2
        place = tuple(_floats(a.place)) if a.place else None
        res = T.run_pick(arm, best, cam, T_tc, wc, place_table=place, settle_s=a.settle,
                         finder=T.DetectorFinder(best, detector, prompts[0] or best.label),
                         log_path=out / "run.jsonl", frames_dir=out / "frames", **gkw)
        print(json.dumps({k: res.get(k) for k in ("ok", "reason", "trace", "estimate_table", "observations")}, indent=1, default=float))
        print("REAL ARM: support it, then .venv/bin/python -I tools/disable_all.py can0 and check every LED is steady red.")
        return 0 if res.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
