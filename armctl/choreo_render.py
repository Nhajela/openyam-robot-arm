"""armctl.choreo_render: offscreen "contact sheets" of a playable, for people and for Claude.

For a candidate playable we draw the arm (forward kinematics, armctl.kin) at every keyframe
AND at the midpoint of every moving segment (plus quarter points on long segments), each pose
seen first from where the AUDIENCE stands (perspective, configurable per site in
armctl/audience.json) and then, smaller, from down the table and from above. Under the tiles
a timeline strip shows every joint as a fraction of its soft range (red = within 10 % of a
limit) and the speed headroom (peak |velocity| / cap and |acceleration| / cap). An overview
sheet comes first: an onion-skin of the whole move from the audience, the tip trace, and the
full per-joint timeline.

Nothing here moves anything or opens a display (matplotlib Agg).

    from armctl import choreo_render as R
    sheets = R.render_sheets(playable, out_dir)          # -> [Path(overview.png), Path(poses-1.png), ...]
    poses = R.pose_list(playable)                         # what the tiles show
    R.metrics_text(playable)                              # the same numbers, in words, for the model

CLI:  .venv/bin/python -m armctl.choreo_render playables/wave-hello.json --out /tmp/sheets
"""
from __future__ import annotations

import argparse
import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.gridspec import GridSpec, GridSpecFromSubplotSpec  # noqa: E402

from armctl import kin  # noqa: E402
from armctl import playable as P  # noqa: E402

AUDIENCE_FILE = Path(os.environ.get("ARMCTL_AUDIENCE", Path(__file__).with_name("audience.json")))
POSES_PER_SHEET = 8
QUARTER_OVER_S = 2.0     # authored seconds: longer moving segments also get quarter points
MAX_SHEETS = 6           # pose sheets; beyond this midpoints, then keyframes, are thinned
NEAR_LIMIT = 0.15        # rad from a soft limit counts as "near"
SHEET_W_IN, DPI = 18.0, 100  # 1800 px wide
CHAIN = ["base", "link1", "link2", "link3", "link4", "link5", "gripper"]
# colour per link segment: pedestal, shoulder, upper arm, elbow, forearm, wrist
SEG_STYLE = [("#555555", 9), ("#555555", 8), ("#2b6cb0", 7), ("#2b6cb0", 7), ("#dd6b20", 6), ("#38a169", 5)]
FINGER_COLOR = "#805ad5"
JOINT_COLORS = ["#e53e3e", "#dd6b20", "#d69e2e", "#38a169", "#3182ce", "#805ad5", "#222222"]


# --- cameras --------------------------------------------------------------------------------

@dataclass
class View:
    name: str
    eye: np.ndarray
    target: np.ndarray
    ortho: bool = False
    fov_deg: float = 40.0
    label: str = ""

    def basis(self):
        f = self.target - self.eye
        f = f / np.linalg.norm(f)
        up = np.array([0.0, 0.0, 1.0]) if abs(f[2]) < 0.95 else np.array([1.0, 0.0, 0.0])
        r = np.cross(f, up)
        r /= np.linalg.norm(r)
        u = np.cross(r, f)
        return r, u, f

    def project(self, pts) -> np.ndarray:
        """(N,3) base-frame points -> (N,3): u, v (metres at the target's depth), depth."""
        pts = np.atleast_2d(np.asarray(pts, float))
        r, u, f = self.basis()
        rel = pts - self.eye
        x, y, z = rel @ r, rel @ u, rel @ f
        if self.ortho:
            return np.c_[x, y, z]
        d0 = float(np.linalg.norm(self.target - self.eye))
        z = np.maximum(z, 1e-3)
        return np.c_[x / z * d0, y / z * d0, z]


def load_audience(spec: str | None = None) -> View:
    """Audience camera: preset name ("standing", "seated", ...), "x,y,z" eye (base frame), or
    None for the site default in armctl/audience.json."""
    cfg: dict[str, Any] = {"default": "standing", "presets": {
        "standing": {"eye": [0.45, 1.10, 0.85], "target": [0.15, 0.0, 0.30], "fov_deg": 38,
                     "label": "standing at the table's front edge, arm's left"}}}
    try:
        cfg = json.loads(AUDIENCE_FILE.read_text())
    except (OSError, ValueError):
        pass
    presets = cfg.get("presets", {})
    name = spec or cfg.get("default", "standing")
    if name in presets:
        p = presets[name]
        return View("audience", np.array(p["eye"], float), np.array(p.get("target", [0.15, 0, 0.3]), float),
                    False, float(p.get("fov_deg", 38)), f"AUDIENCE ({name}): {p.get('label', '')}")
    try:
        eye = np.array([float(x) for x in str(name).split(",")], float)
        assert eye.shape == (3,)
    except Exception:
        raise ValueError(f"audience {name!r}: not a preset ({sorted(presets)}) or 'x,y,z'") from None
    return View("audience", eye, np.array([0.15, 0.0, 0.30]), False, 38.0, f"AUDIENCE eye at {name} m")


def secondary_views() -> list[View]:
    return [View("down-table", np.array([2.0, 0.0, 0.3]), np.array([0.0, 0.0, 0.3]), True,
                 label="from down the table (+x), orthographic"),
            View("top", np.array([0.15, 0.0, 2.5]), np.array([0.15, 0.0, 0.0]), True,
                 label="from above, orthographic")]


# --- poses ------------------------------------------------------------------------------------

def _grips(kfs) -> list[float]:
    out, g = [], 0.0
    for k in kfs:
        g = float(k.get("grip", g) if k.get("grip") is not None else g)
        out.append(g)
    return out


def _path(p: dict) -> dict[str, Any] | None:
    """One loop at speed 1 sampled at 100 Hz, with the authored-time offset (transition)."""
    one = {**p, "loop": 1, "speed": 1.0}
    try:
        segs, transition, _ = P.timeline(one)
        s = P.sample(one)
    except Exception:
        return None
    off = float(transition["duration"]) if transition else 0.0
    return {"t": np.asarray(s["t"]), "q": np.asarray(s["q"]), "grip": np.asarray(s["grip"]), "offset": off}


def pose_list(p: dict, *, quarter_over_s: float = QUARTER_OVER_S, max_sheets: int = MAX_SHEETS,
              per_sheet: int = POSES_PER_SHEET) -> list[dict[str, Any]]:
    """Keyframes + segment midpoints (+ quarter points on long segments), in time order.

    Each: {id, kind: key|mid|quarter, t (authored s), q[6], grip, seg}. Hold segments have no
    in-between pose. If there would be more than ``max_sheets * per_sheet`` poses, quarter
    points, then midpoints, then evenly spaced keyframes are dropped (``thinned`` is set)."""
    kfs = p.get("keyframes") or []
    if not kfs:
        return []
    grips = _grips(kfs)
    path = _path(p)
    out: list[dict[str, Any]] = []

    def at(t: float, i: int, s: float):
        if path is not None:
            ti = t + path["offset"]
            k = int(np.clip(np.searchsorted(path["t"], ti), 0, len(path["t"]) - 1))
            return path["q"][k].tolist(), float(path["grip"][k])
        a, b = np.array(kfs[i - 1]["q"], float), np.array(kfs[i]["q"], float)
        return (a + (b - a) * s).tolist(), grips[i - 1] + (grips[i] - grips[i - 1]) * s

    for i, k in enumerate(kfs):
        if i > 0 and k.get("ease", "cubic") != "hold":
            t0, t1 = float(kfs[i - 1]["t"]), float(k["t"])
            fr = (0.25, 0.5, 0.75) if t1 - t0 > quarter_over_s else (0.5,)
            for s in fr:
                q, g = at(t0 + (t1 - t0) * s, i, s)
                kind = "mid" if s == 0.5 else "quarter"
                out.append({"id": f"K{i - 1}>{i} {'½' if s == 0.5 else ('¼' if s < 0.5 else '¾')}",
                            "kind": kind, "t": round(t0 + (t1 - t0) * s, 3), "q": q, "grip": g, "seg": i})
        out.append({"id": f"K{i}", "kind": "key", "t": float(k["t"]), "q": [float(x) for x in k["q"]],
                    "grip": grips[i], "seg": i, "ease": k.get("ease", "cubic")})
    cap = max_sheets * per_sheet
    thinned = False
    for drop in ("quarter", "mid"):
        if len(out) > cap:
            out = [x for x in out if x["kind"] != drop]
            thinned = True
    if len(out) > cap:
        keep = set(np.linspace(0, len(out) - 1, cap).round().astype(int).tolist())
        out = [x for j, x in enumerate(out) if j in keep]
        thinned = True
    if thinned:
        for x in out:
            x["thinned"] = True
    return out


# --- geometry for drawing -----------------------------------------------------------------------

def arm_geometry(q, grip: float) -> dict[str, Any]:
    f = kin.frames(q)
    pts = np.array([f[n][:3, 3] for n in CHAIN])
    G = f["gripper"]
    tip = f["gripper_tip"][:3, 3]
    side = G[:3, 1]                       # fingers open along the gripper's y axis
    axis = tip - G[:3, 3]
    half = 0.012 + 0.035 * float(np.clip(grip, 0, 1))
    root = G[:3, 3] + axis * 0.35
    fingers = [(root + side * half, tip + side * half), (root - side * half, tip - side * half)]
    gaze = (tip, tip + kin.approach(q) * 0.09)
    tz, lz, ln = kin.clearance(q)
    return {"chain": pts, "fingers": fingers, "gaze": gaze, "tip": tip, "tip_z": tz,
            "low_link": (lz, ln), "palm": (root + side * half, root - side * half)}


def _table_lines():
    xs, ys = (-0.3, 0.6), (-0.4, 0.4)
    lines = []
    for x in np.arange(-0.2, 0.55, 0.1):
        lines.append(([x, ys[0], 0], [x, ys[1], 0]))
    for y in np.arange(-0.3, 0.35, 0.1):
        lines.append(([xs[0], y, 0], [xs[1], y, 0]))
    edge = [([xs[0], ys[0], 0], [xs[1], ys[0], 0]), ([xs[1], ys[0], 0], [xs[1], ys[1], 0]),
            ([xs[1], ys[1], 0], [xs[0], ys[1], 0]), ([xs[0], ys[1], 0], [xs[0], ys[0], 0])]
    return lines, edge


def draw_arm(ax, view: View, q, grip: float, *, alpha: float = 1.0, ghost: bool = False,
             table: bool = True, scale: float = 1.0, number: str | None = None):
    g = arm_geometry(q, grip)
    if table:
        lines, edge = _table_lines()
        for a, b in lines:
            uv = view.project([a, b])
            ax.plot(uv[:, 0], uv[:, 1], color="#d9d4c7", lw=0.6, zorder=0)
        for a, b in edge:
            uv = view.project([a, b])
            ax.plot(uv[:, 0], uv[:, 1], color="#a89f88", lw=1.2, zorder=0)
    items = []  # (depth, kind, uv, color, lw)
    ch = g["chain"]
    for i in range(len(ch) - 1):
        uv = view.project(ch[i:i + 2])
        c, w = SEG_STYLE[i]
        items.append((uv[:, 2].mean(), uv, c, w))
    for a, b in g["fingers"]:
        uv = view.project([a, b])
        items.append((uv[:, 2].mean(), uv, FINGER_COLOR, 3.5))
    uv = view.project(list(g["palm"]))
    items.append((uv[:, 2].mean(), uv, FINGER_COLOR, 3.5))
    items.sort(key=lambda x: -x[0])
    for z, (_d, uv, c, w) in enumerate(items):
        col = "#bbbbbb" if ghost else c
        ax.plot(uv[:, 0], uv[:, 1], color=col, lw=w * scale, alpha=alpha, solid_capstyle="round",
                zorder=2 + z * 0.01 + (0 if ghost else 1))
    if not ghost:
        uv = view.project(list(g["gaze"]))
        ax.plot(uv[:, 0], uv[:, 1], ls=":", color="#805ad5", lw=1.2, zorder=4)
        b = view.project([[0, 0, 0]])[0]
        ax.plot([b[0]], [b[1]], marker="s", ms=7 * scale, color="#333333", zorder=2)
    if number is not None:
        t = view.project([g["tip"]])[0]
        ax.text(t[0], t[1], number, fontsize=7, color="#222", zorder=6,
                bbox=dict(boxstyle="round,pad=0.1", fc="white", ec="none", alpha=0.7))
    return g


def _limits(view: View) -> tuple[tuple[float, float], tuple[float, float]]:
    """Fixed axis limits per view so every tile has the same scale."""
    box = np.array([[x, y, z] for x in (-0.3, 0.6) for y in (-0.4, 0.4) for z in (0.0, 0.72)])
    uv = view.project(box)
    pad = 0.01
    return (uv[:, 0].min() - pad, uv[:, 0].max() + pad), (uv[:, 1].min() - pad, uv[:, 1].max() + pad)


def _setup(ax, view: View, title: str | None = None, fs: int = 8):
    xl, yl = _limits(view)
    ax.set_xlim(*xl)
    ax.set_ylim(*yl)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_color("#cccccc")
    if title:
        ax.set_title(title, fontsize=fs, loc="left", pad=2)


def _floor_line(ax, view: View):
    if view.name == "down-table":
        xl, _ = _limits(view)
        ax.plot(xl, [kin.MIN_TIP_Z] * 2, ls="--", color="#e53e3e", lw=0.8)
        ax.text(xl[0] + 0.01, kin.MIN_TIP_Z + 0.01, "tip floor 8 cm", fontsize=6, color="#e53e3e")
    if view.name == "top":
        aud = load_audience()
        e = view.project([aud.eye])[0]
        xl, yl = _limits(view)
        ex, ey = np.clip(e[0], *xl), np.clip(e[1], *yl)
        ax.annotate("audience", (ex, ey), fontsize=6, color="#2b6cb0", ha="center")


# --- labels / metrics ------------------------------------------------------------------------

def near_limits(q) -> list[str]:
    env = P.envelope()
    out = []
    for j, (x, (lo, hi)) in enumerate(zip(q, env.soft)):
        d = min(x - lo, hi - x)
        if d < NEAR_LIMIT:
            out.append(f"j{j + 1} {d:+.2f} from limit")
    return out


def pose_label(pose: dict) -> list[str]:
    g = arm_geometry(pose["q"], pose["grip"])
    q = pose["q"]
    lines = [f"{pose['id']}   t={pose['t']:.2f}s" + (f"  ({pose['ease']})" if pose.get("ease") else ""),
             "q " + " ".join(f"{x:+.2f}" for x in q),
             f"tip z {g['tip_z'] * 100:.0f} cm   grip {pose['grip']:.2f}"]
    warn = near_limits(q)
    lz, ln = g["low_link"]
    if g["tip_z"] < kin.MIN_TIP_Z + 0.04 or lz < kin.MIN_LINK_Z + 0.03:
        warn.append(f"near table (tip {g['tip_z'] * 100:.0f} cm, {ln} {lz * 100:.0f} cm)")
    return lines + (["! " + "; ".join(warn)] if warn else [])


def _caps(p: dict):
    from interfaces.timeline import coach
    vcap, acap, gcap, preset = coach.caps_for(p)
    return np.asarray(vcap), np.asarray(acap), gcap, preset


def path_stats(p: dict) -> dict[str, Any] | None:
    path = _path(p)
    if path is None:
        return None
    t, Q = path["t"], path["q"]
    if len(t) < 3:
        return None
    V = np.gradient(Q, t, axis=0)
    A = np.gradient(V, t, axis=0)
    vcap, acap, _, preset = _caps(p)
    return {**path, "V": V, "A": A, "vfrac": np.abs(V) / vcap, "afrac": np.abs(A) / acap,
            "vcap": vcap, "acap": acap, "preset": preset}


def audience_travel(p: dict, view: View | None = None) -> list[dict[str, Any]]:
    """Per authored segment: how far the hand (tip) and the elbow move ON THE AUDIENCE'S IMAGE
    (cm at the arm's distance), the biggest joint move and its duration."""
    view = view or load_audience()
    kfs = p.get("keyframes") or []
    out = []
    for i in range(1, len(kfs)):
        a, b = np.array(kfs[i - 1]["q"], float), np.array(kfs[i]["q"], float)
        fa, fb = kin.frames(a), kin.frames(b)
        tip = view.project([fa["gripper_tip"][:3, 3], fb["gripper_tip"][:3, 3]])
        elb = view.project([fa["link4"][:3, 3], fb["link4"][:3, 3]])
        d = np.abs(b - a)
        out.append({"seg": i, "t0": float(kfs[i - 1]["t"]), "t1": float(kfs[i]["t"]),
                    "ease": kfs[i].get("ease", "cubic"),
                    "tip_cm": float(np.linalg.norm(tip[1, :2] - tip[0, :2]) * 100),
                    "elbow_cm": float(np.linalg.norm(elb[1, :2] - elb[0, :2]) * 100),
                    "max_joint": f"j{int(d.argmax()) + 1}", "max_delta": float(d.max())})
    return out


def metrics_text(p: dict, view: View | None = None) -> str:
    """The numbers behind the sheets, in words, for the model's critique."""
    view = view or load_audience()
    kfs = p.get("keyframes") or []
    lines = [f"Audience camera: {view.label} (eye {np.round(view.eye, 2).tolist()} m, base frame)."]
    if len(kfs) >= 2:
        Q = np.array([k["q"] for k in kfs], float)
        t = np.array([k["t"] for k in kfs], float)
        dt = np.diff(t)
        rng = Q.max(0) - Q.min(0)
        lines.append("Range used per joint (rad): " + " ".join(f"j{j + 1} {x:.2f}" for j, x in enumerate(rng)))
        lines.append(f"Segments: {len(dt)}, durations {dt.min():.2f}-{dt.max():.2f} s "
                     f"(tempo variety CV {dt.std() / dt.mean():.2f}; <0.2 = metronomic), "
                     f"holds {sum(k.get('ease') == 'hold' for k in kfs)}, "
                     f"grip values {sorted({round(g, 2) for g in _grips(kfs)})}")
        for s in audience_travel(p, view):
            flag = "  <- barely visible from the audience" if max(s["tip_cm"], s["elbow_cm"]) < 3 and \
                s["ease"] != "hold" else ""
            lines.append(f"  K{s['seg'] - 1}->K{s['seg']} {s['t0']:.2f}-{s['t1']:.2f}s {s['ease']}: "
                         f"hand moves {s['tip_cm']:.0f} cm, elbow {s['elbow_cm']:.0f} cm on the audience's "
                         f"view; biggest joint {s['max_joint']} {s['max_delta']:.2f} rad{flag}")
    st = path_stats(p)
    if st is not None:
        lines.append(f"Speed headroom (caps '{st['preset']}', {st['vcap'][0]:.2f} rad/s, {st['acap'][0]:.1f} "
                     f"rad/s^2): peak velocity {st['vfrac'].max() * 100:.0f} % of cap, peak acceleration "
                     f"{st['afrac'].max() * 100:.0f} % of cap.")
        cl = [kin.clearance(q) for q in st["q"][::5]]
        m = min(min(tz - kin.MIN_TIP_Z, lz - kin.MIN_LINK_Z) for tz, lz, _ in cl)
        lines.append(f"Closest approach to the table: {m * 100:.1f} cm above the safe height.")
        near = sorted({w.split()[0] for q in st["q"][::10] for w in near_limits(q)})
        if near:
            lines.append(f"Path comes within {NEAR_LIMIT} rad of a soft limit on: {', '.join(near)}.")
    return "\n".join(lines)


# --- sheets -----------------------------------------------------------------------------------

def _timeline(fig, spec, p: dict, st, marks: list[dict], highlight: set[str], compact: bool):
    n = 2 if compact else 8
    gs = GridSpecFromSubplotSpec(n, 1, subplot_spec=spec, hspace=0.15 if compact else 0.1,
                                 height_ratios=[2, 1] if compact else [1] * 7 + [1.2])
    env = P.envelope()
    if st is None:
        ax = fig.add_subplot(gs[0])
        ax.text(0.5, 0.5, "path could not be sampled (see validator)", ha="center", transform=ax.transAxes)
        ax.axis("off")
        return
    t = st["t"] - st["offset"]   # authored seconds
    lo = np.array([a for a, _ in env.soft])
    hi = np.array([b for _, b in env.soft])
    norm = (st["q"] - lo) / (hi - lo)
    axes = []
    if compact:
        ax = fig.add_subplot(gs[0])
        ax.axhspan(0, 0.1, color="#fed7d7")
        ax.axhspan(0.9, 1.0, color="#fed7d7")
        for j in range(6):
            ax.plot(t, norm[:, j], color=JOINT_COLORS[j], lw=1.3, label=f"j{j + 1}")
        ax.plot(t, st["grip"], color=JOINT_COLORS[6], lw=1, ls="--", label="grip")
        ax.set_ylim(-0.05, 1.05)
        ax.set_ylabel("fraction of\nsoft range", fontsize=7)
        ax.legend(ncol=7, fontsize=7, loc="upper right", frameon=False, bbox_to_anchor=(1, 1.25))
        axes.append(ax)
    else:
        for j in range(7):
            ax = fig.add_subplot(gs[j])
            if j < 6:
                ax.axhspan(lo[j], lo[j] + NEAR_LIMIT, color="#fed7d7")
                ax.axhspan(hi[j] - NEAR_LIMIT, hi[j], color="#fed7d7")
                y = st["q"][:, j]
                pad = 0.1
                ax.set_ylim(max(lo[j], y.min() - 0.3) - pad, min(hi[j], y.max() + 0.3) + pad)
                ax.plot(t, y, color=JOINT_COLORS[j], lw=1.4)
                ax.set_ylabel(f"j{j + 1}", fontsize=8, rotation=0, labelpad=12)
            else:
                ax.plot(t, st["grip"], color=JOINT_COLORS[6], lw=1.2)
                ax.set_ylim(-0.05, 1.05)
                ax.set_ylabel("grip", fontsize=8, rotation=0, labelpad=12)
            ax.tick_params(labelsize=6, labelbottom=False)
            axes.append(ax)
    ax = fig.add_subplot(gs[-1], sharex=axes[0])
    ax.plot(t, st["vfrac"].max(1) * 100, color="#2b6cb0", lw=1.2, label="peak |vel| / cap")
    ax.plot(t, st["afrac"].max(1) * 100, color="#c05621", lw=1.0, label="peak |acc| / cap")
    ax.axhline(100, color="#e53e3e", lw=1)
    ax.set_ylim(0, max(110, float(min(250, max(st["vfrac"].max(), st["afrac"].max()) * 100 + 10))))
    ax.set_ylabel("% of cap", fontsize=7)
    ax.set_xlabel("authored time (s)", fontsize=7)
    ax.tick_params(labelsize=6)
    ax.legend(ncol=2, fontsize=6, loc="upper right", frameon=False)
    axes.append(ax)
    for a in axes:
        a.set_xlim(t[0], t[-1])
        for m in marks:
            hot = m["id"] in highlight
            a.axvline(m["t"], color="#333" if hot else "#ccc", lw=0.8 if hot else 0.4,
                      ls="-" if m["kind"] == "key" else ":")
    for m in marks:
        if m["id"] in highlight:
            axes[0].text(m["t"], 1.02, m["id"].split()[0] if m["kind"] == "key" else "·", fontsize=6,
                         ha="center", va="bottom", transform=axes[0].get_xaxis_transform())


def _header(fig, p: dict, extra: str):
    v = P.validate(p)
    try:
        from interfaces.timeline import coach
        cl = coach.explain(p).get("clearance") or {}
    except Exception:
        cl = {}
    ok = v["ok"] and cl.get("ok", True)
    h = fig.get_figheight() * DPI
    y = lambda px: 1 - px / h  # noqa: E731
    head = (f"{p.get('name', '?')}   {'VALID' if ok else 'NOT PLAYABLE'}   duration {v.get('duration')} s   "
            f"loop {p.get('loop', 1)}  speed {p.get('speed', 1)}  caps {p.get('caps', 'default')}")
    fig.text(0.01, y(6), head, fontsize=14, weight="bold", va="top", color="#1a202c" if ok else "#c53030")
    fig.text(0.01, y(28), extra, fontsize=9, va="top", color="#1a202c")
    if p.get("notes") or p.get("prompt"):
        fig.text(0.01, y(44), f"prompt: {p.get('prompt', '')!s:.80}   notes: {p.get('notes', '')!s:.140}",
                 fontsize=8, va="top", color="#4a5568")


def render_overview(p: dict, path: Path, audience: View | None = None) -> Path:
    aud = audience or load_audience()
    sec = secondary_views()
    kfs = p.get("keyframes") or []
    st = path_stats(p)
    fig = plt.figure(figsize=(SHEET_W_IN, 13.5), dpi=DPI)
    gs = GridSpec(2, 3, figure=fig, height_ratios=[1.25, 1], width_ratios=[1.6, 1, 1],
                  left=0.04, right=0.99, top=0.94, bottom=0.04, hspace=0.12, wspace=0.05)
    _header(fig, p, "OVERVIEW (onion skin: light = early, dark = late; numbers = keyframes)")
    for col, view in enumerate([aud] + sec):
        ax = fig.add_subplot(gs[0, col])
        _setup(ax, view, view.label if col else f"{view.label}  <- what people see", fs=9)
        n = max(1, len(kfs))
        if st is not None:
            tips = np.array([kin.tip(q) for q in st["q"][::4]])
            uv = view.project(tips)
            ax.scatter(uv[:, 0], uv[:, 1], c=np.linspace(0, 1, len(uv)), cmap="viridis", s=6, zorder=8)
        for i, k in enumerate(kfs):
            draw_arm(ax, view, k["q"], _grips(kfs)[i], alpha=0.15 + 0.75 * (i + 1) / n, ghost=i < n - 1,
                     table=(i == 0), scale=1.0 if col == 0 else 0.7, number=f"K{i}")
        _floor_line(ax, view)
    _timeline(fig, gs[1, :], p, st, pose_list(p), {m["id"] for m in pose_list(p) if m["kind"] == "key"},
              compact=False)
    fig.savefig(path, dpi=DPI)
    plt.close(fig)
    return path


def render_pose_sheet(p: dict, poses: list[dict], path: Path, sheet_i: int, n_sheets: int,
                      all_poses: list[dict], audience: View | None = None) -> Path:
    aud = audience or load_audience()
    sec = secondary_views()
    cols = 4
    rows = math.ceil(len(poses) / cols) or 1
    st = path_stats(p)
    H = 5.0 * rows + 3.2
    fig = plt.figure(figsize=(SHEET_W_IN, H), dpi=DPI)
    outer = GridSpec(rows + 1, 1, figure=fig, height_ratios=[5.0] * rows + [2.4],
                     left=0.04, right=0.99, top=1 - 0.65 / H, bottom=0.45 / H, hspace=0.12)
    _header(fig, p, f"POSES sheet {sheet_i}/{n_sheets}: keyframes (K) and in-betweens (½ ¼ ¾); "
                    "big = audience view, small = down-table and top; grey ghost = previous pose")
    idx = {x["id"]: j for j, x in enumerate(all_poses)}
    for r in range(rows):
        row = GridSpecFromSubplotSpec(1, cols, subplot_spec=outer[r], wspace=0.06)
        for c in range(cols):
            k = r * cols + c
            if k >= len(poses):
                continue
            pose = poses[k]
            tile = GridSpecFromSubplotSpec(3, 2, subplot_spec=row[c], height_ratios=[3.2, 1.5, 0.75],
                                           hspace=0.05, wspace=0.03)
            prev = all_poses[idx[pose["id"]] - 1] if idx.get(pose["id"], 0) > 0 else None
            ax = fig.add_subplot(tile[0, :])
            _setup(ax, aud)
            if prev is not None:
                draw_arm(ax, aud, prev["q"], prev["grip"], alpha=0.35, ghost=True, table=False)
            draw_arm(ax, aud, pose["q"], pose["grip"])
            ax.text(0.01, 0.98, pose["id"], transform=ax.transAxes, fontsize=13, weight="bold", va="top",
                    color="#1a202c" if pose["kind"] == "key" else "#4a5568")
            for cc, view in enumerate(sec):
                a2 = fig.add_subplot(tile[1, cc])
                _setup(a2, view)
                if prev is not None:
                    draw_arm(a2, view, prev["q"], prev["grip"], alpha=0.35, ghost=True, table=False, scale=0.55)
                draw_arm(a2, view, pose["q"], pose["grip"], scale=0.55, table=view.name != "top")
                _floor_line(a2, view)
                a2.text(0.02, 0.95, view.name, transform=a2.transAxes, fontsize=6, va="top", color="#718096")
            at = fig.add_subplot(tile[2, :])
            at.axis("off")
            lab = pose_label(pose)
            at.text(0, 1, "\n".join(lab[:3]), fontsize=7.5, va="top", family="monospace")
            if len(lab) > 3:
                at.text(0, 0.05, lab[3], fontsize=7.5, va="top", color="#c53030", family="monospace")
    _timeline(fig, outer[rows], p, st, all_poses, {x["id"] for x in poses}, compact=True)
    fig.savefig(path, dpi=DPI)
    plt.close(fig)
    return path


def render_sheets(p: dict, out_dir: str | Path, *, prefix: str = "", audience: str | View | None = None,
                  per_sheet: int = POSES_PER_SHEET, max_sheets: int = MAX_SHEETS,
                  overview: bool = True) -> list[Path]:
    """Write overview.png + poses-N.png for playable ``p`` into ``out_dir``. Returns the paths."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    aud = audience if isinstance(audience, View) else load_audience(audience)
    poses = pose_list(p, max_sheets=max_sheets, per_sheet=per_sheet)
    chunks = [poses[i:i + per_sheet] for i in range(0, len(poses), per_sheet)]
    paths = []
    if overview:
        paths.append(render_overview(p, out / f"{prefix}overview.png", aud))
    for i, ch in enumerate(chunks, 1):
        paths.append(render_pose_sheet(p, ch, out / f"{prefix}poses-{i}.png", i, len(chunks), poses, aud))
    return paths


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m armctl.choreo_render", description=__doc__.split("\n")[0])
    ap.add_argument("playable", help="name in playables/ or a path")
    ap.add_argument("--out", required=True, help="directory for the PNGs")
    ap.add_argument("--audience", help="preset in armctl/audience.json or 'x,y,z' eye (base frame, m)")
    ap.add_argument("--per-sheet", type=int, default=POSES_PER_SHEET)
    a = ap.parse_args(argv)
    p = P.load(a.playable)
    for path in render_sheets(p, a.out, audience=a.audience, per_sheet=a.per_sheet):
        print(path)
    print(metrics_text(p))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
