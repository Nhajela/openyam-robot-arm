"""The joint limits the DimOS Damiao adapter enforces on the REAL arm, read from the model file it
actually loads, compared with the values armctl pins in ``envelope.DIMOS_ADAPTER_LIMITS``.

Why (incident 2026-10-28): ``OpenYamDamiaoAdapter.kinematic_model`` is
``LfsPath("yam_description") / "urdf/yam_gripper_gravity.urdf"``; ``_load_kinematic_model`` takes
the arm position limits from it, and ``_normalize_arm_feedback`` latches a feedback fault (motors
disabled, arm limp) when a joint reading is more than ``feedback_clamp_margin_rad`` (0.05) past
one. armctl planned with the looser i2rt model, let j1 reach +1.62 and the arm dropped.

If a DimOS update (or a re-download of its data repo) makes that file TIGHTER than the pinned
values, armctl's envelope would again allow positions that fault the arm, so ``DimosBackend``
calls ``check_pinned()`` when it connects to a real adapter and refuses to start on a problem.

The path is resolved the way ``dimos.utils.data`` does it (no dimos import, no download):
the dimos source checkout if it is a git repo, else ``$VIRTUAL_ENV/.../site-packages/dimos/data``
when ``VIRTUAL_ENV`` is set, else ``$XDG_DATA_HOME/dimos`` or ``~/.local/share/dimos``, each
with ``repo/data/yam_description/urdf/yam_gripper_gravity.urdf``. The coordinator's environment
decides which one it used, so every candidate that exists is checked. ``ARMCTL_DIMOS_URDF``
overrides the search (one path).
"""
from __future__ import annotations

import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from armctl import envelope as env

MODEL_REL = Path("yam_description") / "urdf" / "yam_gripper_gravity.urdf"
DIMOS_JOINTS = {f"yam_joint{i}": f"j{i}" for i in range(1, 7)}
TOL = 1e-4  # rad: rounding in the pinned values


def candidate_paths(environ: dict | None = None) -> list[Path]:
    e = os.environ if environ is None else environ
    if e.get("ARMCTL_DIMOS_URDF"):
        return [Path(e["ARMCTL_DIMOS_URDF"]).expanduser()]
    out: list[Path] = []
    try:  # dimos/__init__ is light, but find the package without importing it
        import importlib.util
        spec = importlib.util.find_spec("dimos")
        if spec and spec.origin:
            root = Path(spec.origin).parent.parent  # DIMOS_PROJECT_ROOT
            if (root / ".git").exists():
                out.append(root / "data" / MODEL_REL)
    except Exception:
        pass
    if e.get("VIRTUAL_ENV"):
        v = sys.version_info
        out.append(Path(e["VIRTUAL_ENV"]) / f"lib/python{v.major}.{v.minor}/site-packages/dimos/data/repo/data" / MODEL_REL)
    xdg = e.get("XDG_DATA_HOME")
    if xdg:
        out.append(Path(xdg) / "dimos" / "repo" / "data" / MODEL_REL)
    out.append(Path.home() / ".local" / "share" / "dimos" / "repo" / "data" / MODEL_REL)
    seen: list[Path] = []
    for p in out:
        if p not in seen:
            seen.append(p)
    return seen


def read_limits(path: Path | str) -> dict[str, tuple[float, float]]:
    """{j1..j6: (lower, upper)} from a URDF's revolute ``yam_jointN`` limits. Raises ValueError
    when a joint or its limit is missing."""
    root = ET.parse(str(path)).getroot()
    out: dict[str, tuple[float, float]] = {}
    for jt in root.iter("joint"):
        name = jt.get("name")
        if name in DIMOS_JOINTS:
            lim = jt.find("limit")
            if lim is None or lim.get("lower") is None or lim.get("upper") is None:
                raise ValueError(f"{path}: {name} has no lower/upper limit")
            out[DIMOS_JOINTS[name]] = (float(lim.get("lower")), float(lim.get("upper")))
    missing = [j for j in DIMOS_JOINTS.values() if j not in out]
    if missing:
        raise ValueError(f"{path}: no limits for {', '.join(missing)}")
    return out


def compare(installed: dict[str, tuple[float, float]],
            pinned: dict[str, tuple[float, float]] | None = None) -> list[str]:
    """Problems where ``installed`` is TIGHTER than ``pinned`` (looser is fine: armctl stays
    inside the pinned box)."""
    pinned = env.DIMOS_ADAPTER_LIMITS if pinned is None else pinned
    probs = []
    for j, (plo, phi) in pinned.items():
        lo, hi = installed[j]
        if lo > plo + TOL:
            probs.append(f"{j} lower limit {lo:.5f} in DimOS's model is tighter than armctl's pinned {plo:.5f}")
        if hi < phi - TOL:
            probs.append(f"{j} upper limit {hi:.5f} in DimOS's model is tighter than armctl's pinned {phi:.5f}")
    return probs


def check_pinned(paths: list[Path] | None = None,
                 pinned: dict[str, tuple[float, float]] | None = None) -> list[str]:
    """Problems (empty = OK) between every existing DimOS model file and the pinned limits.
    No file found at all is a problem too: we cannot vouch for the limits then."""
    paths = candidate_paths() if paths is None else paths
    found = [p for p in paths if Path(p).is_file()]
    if not found:
        return ["DimOS adapter model yam_gripper_gravity.urdf not found (looked in: "
                + ", ".join(str(p) for p in paths) + "); set ARMCTL_DIMOS_URDF to it"]
    probs = []
    for p in found:
        try:
            probs += [f"{p}: {m}" for m in compare(read_limits(p), pinned)]
        except (OSError, ValueError, ET.ParseError) as e:
            probs.append(f"{p}: unreadable ({e})")
    return probs


if __name__ == "__main__":  # read-only: prints the comparison
    for p in candidate_paths():
        print(p, "exists" if p.is_file() else "missing")
    pr = check_pinned()
    print("OK: installed DimOS limits match armctl's pinned DIMOS_ADAPTER_LIMITS" if not pr else "\n".join(pr))
