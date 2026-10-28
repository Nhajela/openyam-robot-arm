"""Per-joint PD gain overrides for the DimOS OpenYAM hardware component (pure, no DimOS import).

DimOS's ``openyam_hardware()`` hard-codes ``WholeBodyConfig(kp=(80,80,80,10,10,10,0),
kd=(5,5,5,1.5,1.5,1.5,0))`` with gravity compensation on. On the real arm j4 holds 0.06-0.12 rad
off target because kp=10 can't absorb the gravity-model error (experiments/2026-10-28-laptop-
systems-check/LOG.md, 12:55). ``armctl.dimos_coordinator --kp j4=22 --kd j4=2`` overrides
single joints; everything not named keeps DimOS's value. The gripper (7th entry) is never touched.

Hard caps (refused otherwise): kp in (0, 120] on j1-j3 and (0, 40] on j4-j6;
kd in [0, 8] on j1-j3 and [0, 5] on j4-j6.

Try new gains in ``--simulation`` first (the mock adapter ignores them, so this only checks the
plumbing). On the real arm watch j4's temperature and listen for buzz/chatter; drop back to the
defaults (no ``--kp/--kd``) at the first sign of either.
"""
from __future__ import annotations

import math

ARM_JOINTS = ("j1", "j2", "j3", "j4", "j5", "j6")
WRIST = frozenset({"j4", "j5", "j6"})
KP_CAP = {j: (40.0 if j in WRIST else 120.0) for j in ARM_JOINTS}
KD_CAP = {j: (5.0 if j in WRIST else 8.0) for j in ARM_JOINTS}
# DimOS 0.0.14 openyam defaults (7 entries: 6 arm joints + gripper).
DIMOS_DEFAULT_KP = (80.0, 80.0, 80.0, 10.0, 10.0, 10.0, 0.0)
DIMOS_DEFAULT_KD = (5.0, 5.0, 5.0, 1.5, 1.5, 1.5, 0.0)


def parse_overrides(specs: list[str] | None, kind: str) -> dict[str, float]:
    """``["j4=22", "j5=15"]`` -> ``{"j4": 22.0, "j5": 15.0}``; ValueError on anything off.

    ``kind`` is ``"kp"`` or ``"kd"`` and selects the caps. Joints are j1..j6 (``yam_joint4``
    is accepted as j4). A joint may appear once only. Comma-separated lists are allowed.
    """
    if kind not in ("kp", "kd"):
        raise ValueError(f"kind must be 'kp' or 'kd', not {kind!r}")
    caps = KP_CAP if kind == "kp" else KD_CAP
    out: dict[str, float] = {}
    for spec in specs or []:
        for item in str(spec).split(","):
            item = item.strip()
            if not item:
                continue
            name, sep, val = item.partition("=")
            if not sep:
                raise ValueError(f"--{kind} {item!r}: expected JOINT=VALUE, e.g. j4=22")
            name = name.strip().lower()
            if name.startswith("yam_joint"):
                name = "j" + name[len("yam_joint"):]
            if name not in caps:
                raise ValueError(f"--{kind} {item!r}: joint must be one of {', '.join(ARM_JOINTS)} "
                                 "(the gripper gain is not overridable)")
            if name in out:
                raise ValueError(f"--{kind}: {name} given twice")
            try:
                v = float(val)
            except ValueError:
                raise ValueError(f"--{kind} {item!r}: {val!r} is not a number") from None
            if not math.isfinite(v):
                raise ValueError(f"--{kind} {item!r}: must be finite")
            cap = caps[name]
            if kind == "kp" and not 0.0 < v <= cap:
                raise ValueError(f"--kp {name}={v:g} refused: must be in (0, {cap:g}]")
            if kind == "kd" and not 0.0 <= v <= cap:
                raise ValueError(f"--kd {name}={v:g} refused: must be in [0, {cap:g}]")
            out[name] = v
    return out


def apply_overrides(base: tuple[float, ...], overrides: dict[str, float]) -> tuple[float, ...]:
    """Return ``base`` with ``overrides`` (j1..j6 -> index 0..5) replaced; other entries kept."""
    if len(base) < len(ARM_JOINTS):
        raise ValueError(f"gain vector has {len(base)} entries, need >= {len(ARM_JOINTS)}")
    out = [float(x) for x in base]
    for name, v in overrides.items():
        out[ARM_JOINTS.index(name)] = float(v)
    return tuple(out)


def override_wb_config(wb, kp: dict[str, float], kd: dict[str, float]):
    """A new WholeBodyConfig (frozen dataclass) with the overrides applied; ``wb`` unchanged.

    ``wb`` None (DimOS would fall back to its global default) is refused when overriding, so we
    never silently replace an unknown gain vector.
    """
    import dataclasses

    if not kp and not kd:
        return wb
    if wb is None or wb.kp is None or wb.kd is None:
        raise ValueError("hardware component has no explicit kp/kd to override")
    return dataclasses.replace(wb, kp=apply_overrides(tuple(wb.kp), kp),
                               kd=apply_overrides(tuple(wb.kd), kd))


def gains_dict(wb) -> dict:
    """``{"kp": {...}, "kd": {...}}`` keyed j1..j6 (+ ``gripper``) for logs and armctl_info."""
    if wb is None:
        return {"kp": None, "kd": None}
    names = list(ARM_JOINTS) + ["gripper"]

    def named(vec):
        return None if vec is None else {n: float(v) for n, v in zip(names, vec)}

    return {"kp": named(wb.kp), "kd": named(wb.kd)}


def describe(wb) -> str:
    """Compact startup-line text, e.g. ``kp=[80,80,80,22,10,10] kd=[5,5,5,2,1.5,1.5]``."""
    if wb is None or wb.kp is None or wb.kd is None:
        return "kp=dimos-default kd=dimos-default"
    fmt = lambda vec: "[" + ",".join(f"{v:g}" for v in list(vec)[:len(ARM_JOINTS)]) + "]"  # noqa: E731
    return f"kp={fmt(wb.kp)} kd={fmt(wb.kd)}"
