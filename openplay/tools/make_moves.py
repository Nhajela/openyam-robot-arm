"""Generate the open-play moves library additions (playables/<name>.json).

Run from the repo root:

    .venv/bin/python openplay/tools/make_moves.py            # write + print a summary
    .venv/bin/python openplay/tools/make_moves.py --check    # summary only, write nothing
    .venv/bin/python openplay/tools/make_moves.py --sheet /tmp/moves.png   # + tip-path contact sheet

Every move starts and ends at "ready" ([0, 0.5, 0.8, 0, 0, 0]), uses cubic (or hold) easing,
and must pass armctl.playable.validate and interfaces.timeline.coach.explain (which adds the FK
table-clearance check). The builder below times each segment from the caps (cubic peaks at
1.5 d/T velocity and 6 d/T^2 acceleration) unless a time is given, in which case it asserts the
segment fits. Rhythmic moves give explicit times; amplitudes were chosen to fit them.

Joint facts (checked with armctl.kin FK, not guessed):
  +j1 swings the tip to the arm's left; j2 bigger = upper arm pitches forward (tip goes forward
  and down); j3 bigger = elbow opens (tip goes up, then back over the base past ~1.8 at low j2);
  +j4 tilts the fingers up; j5 turns the fingers left/right (+j5 points them to the arm's right,
  so pair +j1 with -j5 to "look" left); j6 rolls the gripper (a head tilt).
  ready tip = (0.205, 0, 0.432) m. To LOWER the tip: raise j2 and/or lower j3, tilt j4 down.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from armctl import kin  # noqa: E402
from armctl import playable as P  # noqa: E402
from interfaces.timeline import coach  # noqa: E402

READY = [0.0, 0.5, 0.8, 0.0, 0.0, 0.0]
AUTHOR = "human:openplay library (claude agent)"
CREATED = "2026-10-28T12:00:00+05:30"  # fixed so regenerating gives identical files
J2_RANGE = (0.3, 3.0)
J3_MAX = 2.8
MIN_TIP_Z = 0.10  # m: stricter than kin.MIN_TIP_Z (0.08) for public play
MARGIN = 1.06     # auto-timed segments get 6 % slack over the caps


class Move:
    def __init__(self, name: str, title: str, notes: str, caps: str = "dance"):
        self.name, self.title, self.notes, self.caps = name, title, notes, caps
        c = P.CAP_PRESETS[caps]
        if c is None:
            env = P.envelope()
            self.V, self.A = min(env.vel_cap), min(env.acc_cap)
        else:
            self.V, self.A = c
        self.G = P.envelope().grip_vel_cap
        self.t = 0.0
        self.k = [{"t": 0.0, "q": list(READY), "grip": 0.0, "ease": "cubic"}]

    @property
    def q(self) -> list[float]:
        return self.k[-1]["q"]

    @property
    def grip(self) -> float:
        for k in reversed(self.k):
            if "grip" in k:
                return k["grip"]
        return 0.0

    def need(self, q, grip=None) -> float:
        d = float(np.max(np.abs(np.array(q) - np.array(self.q))))
        tq = max(1.5 * d / self.V, math.sqrt(6 * d / self.A)) if d > 0 else 0.0
        tg = 1.5 * abs((grip if grip is not None else self.grip) - self.grip) / self.G
        return max(tq, tg)

    def to(self, q, grip=None, T=None, min_T=0.4):
        q = [round(float(x), 3) for x in q]
        assert J2_RANGE[0] <= q[1] <= J2_RANGE[1], f"{self.name}: j2 {q[1]} outside {J2_RANGE}"
        assert q[2] <= J3_MAX, f"{self.name}: j3 {q[2]} > {J3_MAX}"
        need = self.need(q, grip)
        if T is None:
            T = math.ceil(max(need * MARGIN, min_T) * 20) / 20
        else:
            assert need <= T + 1e-9, (f"{self.name}: segment to t={self.t + T:.2f} needs {need:.3f} s, "
                                      f"has {T:.3f} s; q {self.q} -> {q}")
        self.t = round(self.t + T, 3)
        kf = {"t": self.t, "q": q, "ease": "cubic"}
        if grip is not None:
            kf["grip"] = round(float(grip), 3)
        self.k.append(kf)
        return self

    def by(self, dq, grip=None, T=None, min_T=0.4):
        return self.to([a + b for a, b in zip(self.q, dq)], grip, T, min_T)

    def hold(self, T: float):
        self.t = round(self.t + T, 3)
        self.k.append({"t": self.t, "q": list(self.q), "ease": "hold"})
        return self

    def home(self, T=None):
        return self.to(READY, 0.0, T)

    def done(self) -> dict:
        assert self.q == READY, f"{self.name} does not end at ready"
        return {"format": "openyam-playable", "version": "1.0", "name": self.name,
                "title": self.title, "author": AUTHOR, "created": CREATED, "notes": self.notes,
                "space": "joint", "start": "ready", "caps": self.caps, "loop": 1, "speed": 1.0,
                "keyframes": self.k}


def add(j1=0.0, j2=0.0, j3=0.0, j4=0.0, j5=0.0, j6=0.0):
    return [j1, j2, j3, j4, j5, j6]


def pose(base, **kw):
    q = list(base)
    for k, v in kw.items():
        q[int(k[1]) - 1] = v
    return q


# --------------------------------------------------------------------------- moves

def shake_no() -> dict:
    m = Move("shake-no", "No no no",
             "Lifts its head to face you, then shakes it side to side three times: the base swings "
             "and the wrist turns the same way so the whole head says 'no no no', getting a little "
             "smaller each time, ends with a disappointed little head drop, back to ready.")
    face = [0.0, 0.6, 1.1, 0.1, 0.0, 0.0]
    m.to(face)
    for a in (0.3, 0.25, 0.2):
        m.to(pose(face, j1=a, j5=-a * 0.8))
        m.to(pose(face, j1=-a, j5=a * 0.8))
    m.to(face)
    m.to(pose(face, j4=-0.25, j3=1.0)).hold(0.4)
    return m.home().done()


def shrug() -> dict:
    m = Move("shrug", "Shrug",
             "'I dunno': sinks a little, then pops the forearm up with the hand tilted palm-up, "
             "head cocked and gripper wide open, holds the shrug, drops, and shrugs again to the "
             "other side before settling back to ready.")
    low = [0.0, 0.55, 0.75, -0.15, 0.0, 0.0]
    m.to(low)
    m.to([0.08, 0.45, 1.3, 0.5, 0.0, 0.35], grip=1.0).hold(1.0)
    m.to(low, grip=0.2)
    m.to([-0.08, 0.45, 1.3, 0.5, 0.0, -0.35], grip=1.0).hold(1.0)
    m.to(low, grip=0.0)
    return m.home().done()


def celebrate() -> dict:
    m = Move("celebrate", "Celebrate!",
             "Victory: rises tall with the gripper closed like a fist, then pumps it up twice "
             "(drop the elbow, punch up to the left, drop, punch up to the right), twists the "
             "fist at the top, and comes back down to ready.")
    high = [0.0, 1.25, 2.05, 0.45, 0.0, 0.0]  # tip ~15 cm in front of the base, 0.77 m up
    m.to(high, grip=0.0)
    for side in (0.3, -0.3):
        m.to(pose(high, j2=1.35, j3=1.6, j4=0.1))           # elbow drops
        m.to(pose(high, j1=side, j3=2.15, j4=0.55))        # punch up
    m.to(pose(high, j6=0.45)).to(pose(high, j6=-0.45)).to(high)
    return m.home().done()


def robot_dance() -> dict:
    m = Move("robot-dance", "Robot dance",
             "Stiff, angular pops: one joint snaps at a time and freezes (turn, head drop, forearm "
             "up, head up, swing across, head roll, forearm down), then a beep-boop gripper "
             "snap, like a 1980s robot dancer.")
    c = [0.0, 0.6, 1.0, 0.0, 0.0, 0.0]
    m.to(c)
    seq = [pose(c, j1=0.3), pose(c, j1=0.3, j4=-0.3), pose(c, j1=0.3, j4=-0.3, j3=1.35),
           pose(c, j1=0.3, j4=0.25, j3=1.35), pose(c, j1=-0.3, j4=0.25, j3=1.35),
           pose(c, j1=-0.3, j4=0.25, j3=1.35, j6=0.6), pose(c, j1=-0.3, j4=0.25, j3=1.0, j6=0.6),
           pose(c, j1=-0.3, j3=1.0)]
    for q in seq:
        m.to(q, min_T=0.7).hold(0.3)
    m.to(c, min_T=0.7).hold(0.2)
    for g in (0.8, 0.0, 0.8, 0.0):
        m.to(c, grip=g, T=0.4)
    return m.home().done()


def sleepy_stretch() -> dict:
    m = Move("sleepy-stretch", "Sleepy stretch",
             "Dozes off (head droops low and tilted), wakes with a slow yawn (head lifts, gripper "
             "opens wide like a mouth and closes), then stretches up tall and leans to one side "
             "and the other before settling back to ready. Slow and lazy on purpose.")
    slow = 1.2  # a bit slower than the dance caps' fastest timing
    droop = [0.0, 0.62, 0.55, -0.7, 0.0, 0.3]
    m.to(droop, T=math.ceil(m.need(droop) * slow * 20) / 20).hold(0.8)
    yawn = [0.0, 0.5, 0.95, 0.45, 0.0, 0.0]
    m.to(yawn, grip=0.0, T=math.ceil(m.need(yawn) * slow * 20) / 20)
    m.to(yawn, grip=1.0, T=1.0).hold(0.5).to(yawn, grip=0.0, T=0.8)
    tall = [0.0, 1.25, 2.15, 0.5, 0.0, 0.0]
    m.to(tall, T=math.ceil(m.need(tall) * 1.1 * 20) / 20).hold(0.4)
    for side in (0.3, -0.3):
        q = pose(tall, j1=side, j6=side * 1.2)
        m.to(q, T=math.ceil(m.need(q) * slow * 20) / 20).hold(0.3)
    return m.home().done()


def thinking() -> dict:
    m = Move("thinking", "Hmm, thinking",
             "Draws the hand in and up, cocks its head and looks up to one side, taps its 'chin' "
             "(wrist taps with little gripper flicks), switches sides and taps again, then 'aha!': "
             "pops up with the gripper open, and back to ready.")
    left = [0.25, 0.5, 1.2, 0.35, -0.2, 0.45]
    right = [-0.25, 0.5, 1.2, 0.35, 0.2, -0.45]
    for side in (left, right):
        m.to(side, grip=0.0).hold(0.6)
        for _ in range(3):
            m.to(pose(side, j4=side[3] - 0.15), grip=0.35, T=0.7)
            m.to(side, grip=0.0, T=0.7)
    m.to([0.0, 0.6, 1.5, 0.6, 0.0, 0.0], grip=1.0).hold(0.8)
    return m.home().done()


def peekaboo() -> dict:
    m = Move("peekaboo", "Peekaboo",
             "Ducks down low and tucks its head away (hiding), peeks up twice, then pops up tall "
             "with the gripper flung open ('boo!'), hides again and pops up on the other side.")
    hide = [0.0, 0.45, 0.42, -0.55, 0.0, 0.0]
    m.to(hide, grip=0.0).hold(1.0)
    for _ in range(2):
        m.to(pose(hide, j4=hide[3] + 0.25)).hold(0.3).to(hide)
    m.hold(0.4)
    m.to([0.2, 0.55, 1.15, 0.3, -0.2, 0.0], grip=1.0).hold(1.0)
    m.to(pose(hide, j1=-0.2), grip=0.0).hold(1.0)
    m.to([-0.25, 0.55, 1.15, 0.3, 0.25, 0.0], grip=1.0).hold(1.0)
    return m.home().done()


def conductor(bpm: float = 90.0, bars: int = 4) -> dict:
    m = Move("conductor", "Conductor",
             f"Raises its baton (gripper closed), then conducts {bars} bars of 4/4 at {bpm:g} bpm: "
             "a big down-stroke on 1 (shoulder, elbow and wrist together), out to the left on 2, "
             "across to the right on 3, up on 4. Ends with a looping cut-off and a bow of the "
             "baton, back to ready.")
    beat = 60.0 / bpm
    dmax = min(m.V * beat / 1.5, m.A * beat * beat / 6.0) * 0.98  # per joint per beat
    top = [0.0, 0.75, 1.35, 0.25, 0.0, 0.0]
    v = dmax  # full vertical stroke on every joint that lifts the tip
    h = dmax / 2  # left/right half-width (2 -> 3 crosses twice this)
    ictus = pose(top, j2=top[1] + v, j3=top[2] - v, j4=top[3] - v)
    b1 = ictus
    b2 = pose(ictus, j1=h, j5=-h, j3=ictus[2] + v * 0.4, j2=ictus[1] - v * 0.4, j4=ictus[3] + 0.3 * v)
    b3 = pose(b2, j1=-h, j5=h)
    b4 = top
    m.to(top, grip=0.0).hold(0.6)
    # one-beat upbeat breath (prep)
    m.to(pose(top, j3=top[2] + v * 0.6, j4=top[3] + v * 0.6), T=beat)
    m.to(top, T=beat)
    for _ in range(bars):
        for q in (b1, b2, b3, b4):
            m.to(q, T=beat)
    # cut-off: loop up and close
    m.to(pose(top, j1=0.15, j3=top[2] + 0.2, j4=0.45))
    m.to(pose(ictus, j4=-0.2)).hold(0.6)
    return m.home().done()


def disco_point() -> dict:
    m = Move("disco-point", "Disco point",
             "Saturday Night Fever: points the closed gripper high up to its left, then swings "
             "down and across to point low on the right, twice, with a little twist on every "
             "point, back to ready.")
    up_left = [0.62, 1.45, 2.2, 0.55, -0.25, 0.0]  # tip forward of the base, so +j1 really is left
    down_right = [-0.45, 1.4, 1.35, -0.25, 0.2, 0.0]
    for i, p in enumerate((up_left, down_right, up_left, down_right)):
        m.to(p, grip=0.0)
        if i < 3:
            m.to(pose(p, j6=0.15, j4=p[3] + 0.15), T=0.7).to(p, T=0.7)  # jab
    return m.home().done()


def high_five_offer() -> dict:
    m = Move("high-five-offer", "High five?",
             "Reaches up and out toward you with the gripper open like a raised palm, holds the "
             "offer, waggles 'come on', then a quick forward slap, and back to ready.")
    offer = [0.0, 1.55, 1.9, 0.6, 0.0, 0.0]  # tip 35 cm out, 0.69 m up, fingers up = palm out
    m.to(offer, grip=1.0).hold(1.5)
    for _ in range(2):
        m.to(pose(offer, j4=0.3, j6=0.15)).to(pose(offer, j4=0.5, j6=-0.15))
    m.to(offer).hold(0.8)
    m.to(pose(offer, j2=1.72, j3=1.8, j4=0.45)).to(offer).hold(0.6)  # the slap
    return m.home().done()


def curious_sniff() -> dict:
    m = Move("curious-sniff", "Curious sniff",
             "Leans far out toward the audience with its head low, sniffs (quick little head "
             "bobs), looks left and right sniffing, then startles back up and away, and "
             "settles back to ready.")
    lean = [0.0, 1.8, 1.6, -0.4, 0.0, 0.0]  # tip 51 cm out, down to 0.26 m
    m.to(lean, grip=0.3).hold(0.4)
    for look in (0.0, 0.25, -0.25):
        q = pose(lean, j1=look, j5=-look)
        if look:
            m.to(q)
        for _ in range(1 if look else 2):
            m.to(pose(q, j4=q[3] - 0.11), T=0.6).to(q, T=0.6)
    m.to(lean, grip=0.3)
    m.to([0.0, 0.45, 1.15, 0.35, 0.0, 0.0], grip=1.0).hold(1.0)
    return m.home().done()


def gripper_clap(bpm: float = 80.0, claps: int = 8) -> dict:
    m = Move("gripper-clap", "Gripper clap",
             f"Claps along at {bpm:g} bpm: the gripper opens on the off-beat and snaps shut on "
             f"the beat while the whole arm bobs down and up and sways side to side, {claps} "
             "claps, then back to ready.")
    T = 60.0 / bpm  # open on one beat, clap on the next
    d = min(m.V * T / 1.5, m.A * T * T / 6.0) * 0.98  # largest bob that fits a beat
    c = [0.0, 0.62, 1.05, 0.1, 0.0, 0.0]
    m.to(c, grip=0.0)
    for i in range(claps):
        s = 1 if i % 2 == 0 else -1
        m.to(pose(c, j1=s * d * 0.95, j2=c[1] + d, j3=c[2] - d), grip=0.9, T=T)  # down, sway, open
        m.to(c, grip=0.0, T=T)                                                 # up + clap
    m.to(c, grip=0.0)
    return m.home().done()


def bow() -> dict:
    m = Move("bow", "Take a bow",
             "A proper bow that really goes down: stands up proud with the hand open, then "
             "pitches the shoulder forward and drops the head so the tip sinks about 25 cm below "
             "ready (checked with FK), holds the bow, and rises slowly back to ready. Calm "
             "'default' caps. (graceful-bow only ever lifts the tip, so it doesn't read as a bow.)",
             caps="default")
    proud = [0.0, 0.4, 1.0, 0.3, 0.0, 0.0]       # tip 0.57 m, a bit back
    low = [0.0, 1.35, 0.95, -0.55, 0.0, 0.0]     # tip ~0.16 m, 34 cm out, fingers down
    m.to(proud, grip=1.0).hold(0.5)
    m.to(low, grip=0.3).hold(1.5)
    m.to(proud, grip=1.0).hold(0.4)
    return m.home().done()


MOVES = [bow, shake_no, shrug, celebrate, robot_dance, sleepy_stretch, thinking, peekaboo,
         conductor, disco_point, high_five_offer, curious_sniff, gripper_clap]


# --------------------------------------------------------------------------- checks

def summarise(p: dict) -> dict:
    v = P.validate(p)
    r = coach.explain(p)
    s = P.sample(p, rate=50.0)
    Q = np.asarray(s["q"], float)
    tips = np.array([kin.tip(q) for q in Q])
    lows = [kin.clearance(q)[1] for q in Q]
    return {"name": p["name"], "ok": bool(v["ok"] and r["ok"]), "duration": v["duration"],
            "max_vel": v["max_vel"], "max_acc": v["max_acc"], "caps": p.get("caps", "default"),
            "min_tip_z": float(tips[:, 2].min()), "max_tip_z": float(tips[:, 2].max()),
            "min_link_z": float(min(lows)),
            "tip_span_cm": (100 * (tips.max(0) - tips.min(0))).round(0).tolist(),
            "j2": (float(Q[:, 1].min()), float(Q[:, 1].max())), "j3_max": float(Q[:, 2].max()),
            "violations": v["violations"] + ([{"msg": x} for x in r["summary"][1:]] if not r["ok"] else []),
            "tips": tips, "t": np.asarray(s["t"], float)}


def check(p: dict, s: dict) -> list[str]:
    bad = [x["msg"] for x in s["violations"]]
    if not s["ok"] and not bad:
        bad.append("validate/explain not ok")
    if not 4.0 <= s["duration"] <= 20.0:
        bad.append(f"duration {s['duration']} s outside 4..20 s")
    if s["min_tip_z"] < MIN_TIP_Z:
        bad.append(f"tip down to {s['min_tip_z']:.3f} m < {MIN_TIP_Z} m")
    if s["j3_max"] > J3_MAX + 1e-6:
        bad.append(f"j3 reaches {s['j3_max']:.3f} > {J3_MAX}")
    if not (J2_RANGE[0] - 1e-6 <= s["j2"][0] and s["j2"][1] <= J2_RANGE[1] + 1e-6):
        bad.append(f"j2 range {s['j2']} outside {J2_RANGE}")
    kf = p["keyframes"]
    if kf[0]["q"] != READY or kf[-1]["q"] != READY:
        bad.append("does not start and end at ready")
    if any(k.get("ease", "cubic") not in ("cubic", "hold") for k in kf):
        bad.append("ease other than cubic/hold")
    return bad


def contact_sheet(rows: list[dict], path: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    n = len(rows)
    cols = 4
    fig, axes = plt.subplots(math.ceil(n / cols), cols, figsize=(4 * cols, 3.4 * math.ceil(n / cols)))
    for ax, s in zip(axes.flat, rows):
        tp = s["tips"]
        # front view as the audience sees it (they stand at +x looking back at the arm):
        # the arm's left (+y) is on the audience's right, so plot +y to the right
        sc = ax.scatter(tp[:, 1], tp[:, 2], c=s["t"], s=3, cmap="viridis")
        ax.plot(tp[:, 1], tp[:, 2], lw=0.4, color="0.6")
        r = kin.tip(READY)
        ax.plot([r[1]], [r[2]], "k+", ms=10)
        ax.axhline(MIN_TIP_Z, color="r", lw=0.6, ls="--")
        ax.set_title(f"{s['name']}  {s['duration']:.1f}s", fontsize=9)
        ax.set_xlim(-0.6, 0.6)
        ax.set_ylim(0, 0.9)
        ax.set_aspect("equal")
        ax.tick_params(labelsize=7)
        fig.colorbar(sc, ax=ax, fraction=0.04)
    for ax in list(axes.flat)[n:]:
        ax.axis("off")
    fig.suptitle("tip paths, front view from the audience (m); + = ready, red = 10 cm", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=90)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="don't write playables")
    ap.add_argument("--sheet", help="write a contact-sheet PNG of tip paths here (needs matplotlib)")
    a = ap.parse_args()
    rows, failed = [], 0
    for fn in MOVES:
        p = fn()
        s = summarise(p)
        bad = check(p, s)
        rows.append(s)
        vel = " ".join(f"{x:.2f}" for x in s["max_vel"])
        print(f"{'OK ' if not bad else 'BAD'} {p['name']:16s} {s['duration']:5.1f}s caps={s['caps']:7s} "
              f"maxvel[{vel}] tip z {s['min_tip_z']:.2f}..{s['max_tip_z']:.2f} m "
              f"span(x,y,z) {s['tip_span_cm']} cm  links>={s['min_link_z']:.2f}")
        for b in bad:
            print("      -", b)
        if bad:
            failed += 1
        elif not a.check:
            path = P.PLAYABLES_DIR / f"{p['name']}.json"
            if path.exists() and json.loads(path.read_text()).get("author") != AUTHOR:
                print(f"      - NOT written: {path.name} exists and was authored by someone else")
                failed += 1
                continue
            P.save(p)
    if a.sheet:
        try:
            contact_sheet(rows, a.sheet)
            print("contact sheet:", a.sheet)
        except ImportError:
            print("matplotlib not installed; no contact sheet")
    print(f"{len(MOVES) - failed}/{len(MOVES)} moves OK" + ("" if a.check else " (written to playables/)"))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
