"""interfaces.wristcam.look_and_grasp: find a static object with the wrist camera and pick it up.

Library: ``plan_from_frame`` (one frame at a known q -> detection + a validated pick_place
playable + the closed-loop look poses) and ``engine.GraspEngine`` (the closed loop itself).

Three modes, ``--dry-run`` is the default:

    # DRY RUN (nothing moves): one frame + the q it was taken at -> plan, playable JSON, annotated
    # image. q is read (read-only) from the daemon unless --q is given; the frame comes from
    # --source (default the lab console's camera) or --image.
    .venv/bin/python -m interfaces.wristcam.look_and_grasp --object cup --place 0.08,-0.05
    .venv/bin/python -m interfaces.wristcam.look_and_grasp --image f.jpg --q -0.40,1.70,1.59,-1.46,0,0

    # SIMULATE: the whole closed loop against the synthetic world + fake arm (tests use this)
    .venv/bin/python -m interfaces.wristcam.look_and_grasp --simulate --out /tmp/wc_sim

    # EXECUTE on the real arm (through the armctl daemon; each motion asks y/N at the terminal)
    .venv/bin/python -m interfaces.wristcam.look_and_grasp --execute --object cup --expect 0.0,-0.15 --place 0.08,-0.05

Positions (--expect, --place) are TABLE frame metres (origin at the calibration board centre,
see board.py); workcell.json's T_base_table maps them to the base. Outputs go to --out
(default interfaces/wristcam/runs/<timestamp>/): plan.json, <name>.playable.json, look.jpg,
and for simulate/execute run.jsonl (every observation and decision) + frames/.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import sys
from pathlib import Path

import cv2
import numpy as np

from armctl import envelope as envm
from armctl import kin
from armctl import playable as pl

from interfaces.grasp.grasp import Unreachable, Workcell, pick_place
from interfaces.timeline import coach

from .camera import DEFAULT_HANDEYE, DEFAULT_INTRINSICS
from .engine import GraspConfig, GraspEngine, check_step
from .locate import View, annotate, load_view_inputs, make_finder

HERE = Path(__file__).resolve().parent
RUNS = HERE / "runs"


def _floats(s, n=None):
    v = [float(x) for x in str(s).split(",")]
    if n and len(v) != n:
        raise argparse.ArgumentTypeError(f"need {n} comma-separated numbers")
    return v


def object_dims(wc_raw: dict, obj: str) -> tuple[float, float]:
    o = (wc_raw.get("objects") or {}).get(obj)
    if not o:
        raise ValueError(f"unknown object {obj!r} in workcell.json objects ({', '.join(wc_raw.get('objects', {}))})")
    return float(o["width"]), float(o["height"])


def plan_from_frame(frame, q, wc: Workcell, cam, T_tip_cam, finder, cfg: GraspConfig, name: str = "wristcam-pick"):
    """One frame taken at q -> {detection, playable, validation, look_poses, notes}. Never moves."""
    view = View.from_q(q, cam.for_image(frame), T_tip_cam, wc.T_base_table)
    det = finder.find(frame, view)
    out = {"q_frame": [round(float(x), 4) for x in q], "detection": det.to_dict() if det else None,
           "playable": None, "validation": None, "look_poses": [], "notes": []}
    if det is None:
        out["notes"].append("object not found in this frame: move the look pose (or --expect) over it")
        return out, view, det
    width = det.diameter if (det.diameter and abs(det.diameter - cfg.width) < 0.25 * cfg.width) else cfg.width
    pick = (det.xyz_table[0], det.xyz_table[1], 0.0)
    place = tuple(cfg.place_table) + (0.0,) if cfg.place_table is not None else pick
    try:
        p, info = pick_place(wc, pick, place, height=cfg.height, width=width, name=name, grasp_frac=cfg.grasp_frac)
    except Unreachable as e:
        out["notes"].append(f"pick_place: {e}")
        return out, view, det
    r = pl.validate(p)
    ex = coach.explain(p)
    out.update(playable=p, plan_info=info,
               validation={"ok": bool(r["ok"]), "duration": r["duration"],
                           "violations": [v["msg"] for v in r["violations"]][:10], "coach_ok": bool(ex["ok"])})
    # the closed loop's centring poses over this estimate (checked, not executed)
    q_prev = np.asarray(q, float)
    for h in cfg.look_heights:
        from .engine import GraspEngine as _E

        class _Q:   # minimal Arm stand-in so the engine's geometry helpers can be reused
            def q(self_inner):
                return q_prev

        eng = _E(_Q(), finder, cam, T_tip_cam, wc, cfg)
        qq, hu = eng.camera_over(np.array(det.xyz_table[:2]), h)
        if qq is None:
            out["look_poses"].append({"height": h, "q": None, "why": "unreachable"})
            continue
        why = check_step(q_prev, qq, kin.MIN_TIP_Z)
        out["look_poses"].append({"height": round(hu, 3), "q": [round(float(x), 4) for x in qq],
                                  "tip": kin.tip(qq, kin.vision_correction()).round(4).tolist(), "check": why or "ok"})
        q_prev = qq
    if not wc.calibrated:
        out["notes"].append("workcell NOT calibrated: T_base_table is the placeholder (run handeye solve --write-workcell)")
    return out, view, det


def simulate(out: Path, miss_first: bool = False, seed: int = 0, cup_xy=(-0.01, -0.16), expect=(0.02, -0.13),
             place=(0.08, -0.05), log=True) -> dict:
    from . import synth
    from .locate import CupFinder
    cup = synth.Cup(tuple(cup_xy))
    world = synth.World(cups=[cup])
    arm = synth.SimArm(world, miss_first=miss_first, joint_noise=0.0005, seed=seed)
    wc = Workcell(T_base_table=world.T_base_table.tolist(), calibrated=True)
    cfg = GraspConfig(expected_table=tuple(expect), place_table=tuple(place))
    out.mkdir(parents=True, exist_ok=True)
    # dry-run plan from the first look (what --dry-run would print on the real arm)
    eng0 = GraspEngine(arm, CupFinder(), synth.TRUE_CAMERA, synth.TRUE_T_TIP_CAM, wc, cfg)
    q_look, _ = eng0.camera_over(np.array(expect), cfg.look_heights[0])
    arm.move_joints(q_look)
    frame = arm.frame()
    plan, view, det = plan_from_frame(frame, arm.q(), wc, synth.TRUE_CAMERA, synth.TRUE_T_TIP_CAM, CupFinder(), cfg)
    _write_plan(out, plan, frame, view, det)
    arm.move_joints(envm.POSES["ready"])
    eng = GraspEngine(arm, CupFinder(), synth.TRUE_CAMERA, synth.TRUE_T_TIP_CAM, wc, cfg,
                      log_path=out / "run.jsonl" if log else None, frames_dir=out / "frames" if log else None)
    res = eng.run()
    res.update(cup_final_table=[round(float(v), 4) for v in cup.xy], cup_true_start=list(cup_xy),
               collisions=world.collisions, plan_ok=bool(plan["validation"] and plan["validation"]["ok"]))
    (out / "result.json").write_text(json.dumps(res, indent=1, default=float) + "\n")
    return res


def _write_plan(out: Path, plan: dict, frame, view, det) -> None:
    out.mkdir(parents=True, exist_ok=True)
    (out / "plan.json").write_text(json.dumps({k: v for k, v in plan.items() if k != "playable"}, indent=1, default=float) + "\n")
    if plan.get("playable"):
        (out / f"{plan['playable']['name']}.playable.json").write_text(json.dumps(plan["playable"], indent=1) + "\n")
    txt = "dry run: nothing moved"
    if det is not None:
        txt += f"\n{det.kind} at table ({det.xyz_table[0]:+.3f}, {det.xyz_table[1]:+.3f}) +-{det.sigma_xy * 1000:.1f} mm"
    cv2.imwrite(str(out / "look.jpg"), annotate(frame, view, det, txt))


def print_plan(plan: dict) -> None:
    d = plan["detection"]
    print(f"frame taken at q = {plan['q_frame']}")
    if d is None:
        print("NO object found.")
    else:
        print(f"{d['kind']} at table ({d['xyz_table'][0]:+.4f}, {d['xyz_table'][1]:+.4f}) m, base "
              f"({d['xyz_base'][0]:+.4f}, {d['xyz_base'][1]:+.4f}, {d['xyz_base'][2]:+.4f}) m, sigma "
              f"{d['sigma_xy'] * 1000:.1f} mm" + (f", diameter {d['diameter'] * 1000:.1f} mm" if d.get("diameter") else ""))
    if plan.get("playable"):
        v = plan["validation"]
        print(f"open-loop pick_place playable: valid={v['ok']} (coach {v['coach_ok']}), {v['duration']} s")
        for s in plan["plan_info"]["steps"]:
            print("   ", s)
        for msg in v["violations"]:
            print("    VIOLATION:", msg)
    if plan["look_poses"]:
        print("closed-loop centring poses (camera above the object top):")
        for lp in plan["look_poses"]:
            print(f"   h {lp['height']}: q={lp['q']} {lp.get('check', lp.get('why', ''))}")
    for n in plan["notes"]:
        print("note:", n)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", default=True, help="(default) plan only, nothing moves")
    mode.add_argument("--simulate", action="store_true", help="closed loop against the synthetic world")
    mode.add_argument("--execute", action="store_true", help="REAL ARM: closed loop through the armctl daemon")
    ap.add_argument("--object", default="cup", help="workcell.json objects key (default cup)")
    ap.add_argument("--finder", default=None, choices=("cup", "tags", "blob"), help="default: cup for 'cup', else tags")
    ap.add_argument("--source", default="dashboard", help="frame source (vision/sources.py spec)")
    ap.add_argument("--image", default=None, help="dry run on a saved frame instead of --source")
    ap.add_argument("--q", default=None, help="joint angles of that frame (default: read from the daemon)")
    ap.add_argument("--expect", default="0.0,-0.15", help="table x,y where the object roughly is")
    ap.add_argument("--place", default="0.08,-0.05", help="table x,y to put it down")
    ap.add_argument("--grasp-frac", type=float, default=0.6, help="grip height / object height (cups with flared fingertips: 0.85)")
    ap.add_argument("--calib", default=str(DEFAULT_INTRINSICS))
    ap.add_argument("--handeye", default=str(DEFAULT_HANDEYE))
    ap.add_argument("--out", default=None)
    ap.add_argument("--settle", type=float, default=1.0, help="execute: seconds to settle before each frame")
    ap.add_argument("--ring-std", type=float, default=None, help="cup finder: max grey spread of the band next to the rim (default 22)")
    ap.add_argument("--hsv", default=None, help="blob finder: h,s,v,h,s,v colour range (OpenCV HSV; orange 3,120,100,22,255,255)")
    ap.add_argument("--open-margin", type=float, default=None, help="open this much wider than the object (m; workcell default 0.02)")
    ap.add_argument("--realign-max", type=float, default=None, help="descent: largest offset re-aligned on the way down (m; default 0.008)")
    ap.add_argument("--min-tip-z", type=float, default=None, help="lowest fingertip height above the calibrated table (m; workcell default)")
    ap.add_argument("--no-grip-check", action="store_true", help="don't judge holding from the grip reading (it echoes the target on DimOS); camera verify only")
    ap.add_argument("--miss-first", action="store_true", help="simulate: the first grasp misses")
    a = ap.parse_args(argv)
    out = Path(a.out) if a.out else RUNS / dt.datetime.now().strftime("%Y%m%d-%H%M%S")

    if a.simulate:
        res = simulate(out, miss_first=a.miss_first)
        print(json.dumps(res, indent=1, default=float))
        print("wrote", out)
        return 0 if res["ok"] else 1

    wc = Workcell.load()
    if a.min_tip_z is not None:
        wc.min_tip_z = a.min_tip_z   # 2026-10-29: 0.01 put the fingertips on the table (provisional table z + j2 sag)
    if a.open_margin is not None:
        wc.open_margin = a.open_margin   # more finger clearance for the descent's alignment check
    raw = json.loads((Path(__file__).resolve().parents[1] / "grasp" / "workcell.json").read_text())
    width, height = object_dims(raw, a.object)
    cfg = GraspConfig(obj=a.object, width=width, height=height, grasp_frac=a.grasp_frac, expected_table=tuple(_floats(a.expect, 2)),
                      place_table=tuple(_floats(a.place, 2)), settle_s=a.settle if a.execute else 0.0)
    cfg.grip_check = not a.no_grip_check
    if a.realign_max is not None:
        cfg.realign_max = a.realign_max
    cam, T_tc, notes = load_view_inputs(a.calib, a.handeye)
    for n in notes:
        print("note:", n)
    kind = a.finder or ("cup" if a.object == "cup" else "tags")
    fkw = {"obj": a.object} if kind == "cup" else {}
    if kind == "cup" and a.ring_std is not None:
        fkw["max_ring_std"] = a.ring_std   # 2026-10-29: real cup by the tag sheet scored 23.8 against 22
    if kind == "blob":
        if a.hsv:
            hsv = [int(v) for v in a.hsv.split(",")]
            fkw.update(hsv_lo=tuple(hsv[:3]), hsv_hi=tuple(hsv[3:]))
        fkw["plane_height"] = height / 2   # blob centre on the object's mid-height plane (a ball's equator)
        fkw["size_range"] = (0.5 * width, 1.6 * width)
    finder = make_finder(kind, **fkw)

    if a.execute:
        if notes:
            sys.exit("refusing --execute without lens + hand-eye calibration (see README)")
        if not wc.calibrated:
            sys.exit("refusing --execute: workcell.json is not calibrated (handeye solve --write-workcell)")
        from .engine import DaemonArm
        with DaemonArm(source=a.source, program="wristcam-grasp", speed=cfg.speed) as arm:
            # From folded home (j2/j3 inside the soft margin) every planned look fails the path
            # check, so start from ready (2026-10-29: all 5 search views "unreachable" from home).
            if np.any(arm.q()[1:3] < 0.1):
                arm.ready()
            eng = GraspEngine(arm, finder, cam, T_tc, wc, cfg, log_path=out / "run.jsonl", frames_dir=out / "frames")
            res = eng.run()
        print(json.dumps(res, indent=1, default=float))
        print("log:", out / "run.jsonl")
        return 0 if res["ok"] else 1

    # dry run
    if a.image:
        frame = cv2.imread(a.image, cv2.IMREAD_COLOR)
        if frame is None:
            sys.exit(f"cannot read {a.image}")
    else:
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "vision"))
        from sources import grab_one, open_source  # noqa: E402
        with open_source(a.source) as src:
            frame = grab_one(src)
        if frame is None:
            sys.exit(f"no frame from {a.source}")
    if a.q:
        q = _floats(a.q, 6)
    else:
        from .engine import read_q_from_daemon
        q = read_q_from_daemon().tolist()
    plan, view, det = plan_from_frame(frame, q, wc, cam, T_tc, finder, cfg)
    print_plan(plan)
    _write_plan(out, plan, frame, view, det)
    print("wrote", out, "(nothing moved)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
