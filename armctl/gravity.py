"""Per-joint gravity-feedforward scale (and optional gravity-model swap) for the real OpenYAM arm.

Why (experiments/2026-10-28-j4-gravity-scale/LOG.md): DimOS's Damiao adapter feeds forward
``pinocchio.computeGeneralizedGravity`` on ``yam_description/urdf/yam_gripper_gravity.urdf``
(``dimos/hardware/whole_body/damiao/adapter.py`` ``write_motor_commands`` ``tau + gravity``,
``_gravity_torques``). That model's gripper (``yam_link_6`` 0.95 kg + fingers 0.24 kg = 1.19 kg)
is about twice i2rt's (0.69 kg; crank_4310 0.60 kg), so j4 gets ~1 N·m too much feedforward at
the heavy poses and holds 0.04-0.11 rad off target. i2rt itself ships per-joint
``gravity_comp_factor`` ([1.0, 1.1, 1.1, 1.2, 1.0, 1.0] in ``yam_v1.yml``).

How (no edit to the venv): ``armctl.dimos_coordinator --gravity-scale j4=0.83`` swaps the real
hardware component's adapter type to ``armctl_openyam_damiao`` (``armctl/gravity_adapter.py``), a
subclass of DimOS's ``OpenYamDamiaoAdapter`` whose ``_gravity_torques`` returns DimOS's vector
times the per-joint scale. ``ArmctlTwistCoordinator._create_whole_body_adapter`` builds it (in the
DimOS worker). ``--gravity-urdf PATH`` makes the same subclass load another gravity model; it must
match the stock file's joints, joint origins, axes and LIMITS exactly (the adapter takes its
fault-latching limits from this file), only inertials may differ.

Defaults unchanged: no flag -> the stock ``openyam_damiao`` adapter, untouched component.
Scales outside [0.5, 1.5] are refused. Only the gravity feedforward is scaled; kp/kd/q targets
and the adapter's limits are not touched. In ``--simulation`` the mock adapter has no gravity:
the request is carried to the coordinator (``armctl_info()["gravity"]``) as a plumbing check
and the startup line shows the scaled feedforward computed offline through the real subclass.

This module is pure (no DimOS import) except ``preview()``.
"""
from __future__ import annotations

import dataclasses
import math
import xml.etree.ElementTree as ET
from pathlib import Path

ARM_JOINTS = ("j1", "j2", "j3", "j4", "j5", "j6")
SCALE_MIN, SCALE_MAX = 0.5, 1.5
MASS_RATIO_MIN, MASS_RATIO_MAX = 0.5, 1.5  # --gravity-urdf total mass vs the stock model
STOCK_ADAPTER = "openyam_damiao"
SIM_ADAPTER = "mock_whole_body"
ARMCTL_ADAPTER = "armctl_openyam_damiao"
KWARG = "armctl_gravity"  # key in HardwareComponent.adapter_kwargs; popped before any DimOS factory
UNITY = (1.0,) * len(ARM_JOINTS)
URDF_TOL = 1e-6
READY = (0.0, 0.5, 0.8, 0.0, 0.0, 0.0)


def parse_scales(specs: list[str] | None) -> dict[str, float]:
    """``["j4=0.83"]`` -> ``{"j4": 0.83}``; ValueError on anything off (joint, number, range, dup)."""
    out: dict[str, float] = {}
    for spec in specs or []:
        for item in str(spec).split(","):
            item = item.strip()
            if not item:
                continue
            name, sep, val = item.partition("=")
            if not sep:
                raise ValueError(f"--gravity-scale {item!r}: expected JOINT=FACTOR, e.g. j4=0.83")
            name = name.strip().lower()
            if name.startswith("yam_joint"):
                name = "j" + name[len("yam_joint"):]
            if name not in ARM_JOINTS:
                raise ValueError(f"--gravity-scale {item!r}: joint must be one of {', '.join(ARM_JOINTS)}")
            if name in out:
                raise ValueError(f"--gravity-scale: {name} given twice")
            try:
                v = float(val)
            except ValueError:
                raise ValueError(f"--gravity-scale {item!r}: {val!r} is not a number") from None
            if not math.isfinite(v) or not SCALE_MIN <= v <= SCALE_MAX:
                raise ValueError(f"--gravity-scale {name}={val.strip()} refused: must be in "
                                 f"[{SCALE_MIN:g}, {SCALE_MAX:g}]")
            out[name] = v
    return out


def scale_vector(scales: dict[str, float] | None) -> tuple[float, ...]:
    """6-vector j1..j6 (unnamed joints 1.0)."""
    return tuple(float((scales or {}).get(j, 1.0)) for j in ARM_JOINTS)


def check_vector(vec) -> tuple[float, ...]:
    """Validate a 6-vector of scales (the adapter re-checks what reaches the worker)."""
    v = tuple(float(x) for x in vec)
    if len(v) != len(ARM_JOINTS):
        raise ValueError(f"gravity scale needs {len(ARM_JOINTS)} entries, got {len(v)}")
    for j, x in zip(ARM_JOINTS, v):
        if not math.isfinite(x) or not SCALE_MIN <= x <= SCALE_MAX:
            raise ValueError(f"gravity scale {j}={x!r} refused: must be in [{SCALE_MIN:g}, {SCALE_MAX:g}]")
    return v


# ---- --gravity-urdf validation ------------------------------------------------------------------

def _joint_sig(path: Path | str) -> dict[str, tuple]:
    root = ET.parse(str(path)).getroot()
    out = {}
    for jt in root.iter("joint"):
        o, a, lim = jt.find("origin"), jt.find("axis"), jt.find("limit")
        nums = lambda s: tuple(float(x) for x in (s or "0 0 0").split())  # noqa: E731
        out[jt.get("name")] = (
            jt.get("type"), jt.find("parent").get("link"), jt.find("child").get("link"),
            nums(o.get("xyz") if o is not None else None), nums(o.get("rpy") if o is not None else None),
            nums(a.get("xyz") if a is not None else "1 0 0"),
            None if lim is None else (float(lim.get("lower", "nan")), float(lim.get("upper", "nan"))),
        )
    return out


def _total_mass(path: Path | str) -> float:
    root = ET.parse(str(path)).getroot()
    tot = 0.0
    for link in root.iter("link"):
        m = link.find("inertial/mass")
        if m is not None:
            v = float(m.get("value"))
            if not math.isfinite(v) or v < 0:
                raise ValueError(f"{path}: link {link.get('name')} has mass {v}")
            tot += v
    return tot


def check_gravity_urdf(path: Path | str, stock: Path | str) -> list[str]:
    """Problems (empty = OK) with a replacement gravity URDF vs the stock DimOS one.

    Same joints (names, types, parent/child, origins, axes) and the same limits within 1e-6;
    total mass within [0.5, 1.5] x stock. Only masses/COMs/inertias may differ."""
    probs: list[str] = []
    try:
        new, ref = _joint_sig(path), _joint_sig(stock)
    except (ET.ParseError, OSError, AttributeError, ValueError) as e:
        return [f"cannot read {path}: {e}"]
    if set(new) != set(ref):
        probs.append(f"joint set differs from stock: extra {sorted(set(new) - set(ref))}, "
                     f"missing {sorted(set(ref) - set(new))}")
    for name in sorted(set(new) & set(ref)):
        a, b = new[name], ref[name]
        if a[:3] != b[:3]:
            probs.append(f"{name}: type/parent/child {a[:3]} != stock {b[:3]}")
        for label, x, y in (("origin xyz", a[3], b[3]), ("origin rpy", a[4], b[4]), ("axis", a[5], b[5])):
            if len(x) != len(y) or any(abs(p - q) > URDF_TOL for p, q in zip(x, y)):
                probs.append(f"{name}: {label} {x} != stock {y}")
        if (a[6] is None) != (b[6] is None) or (a[6] and any(abs(p - q) > URDF_TOL for p, q in zip(a[6], b[6]))):
            probs.append(f"{name}: limits {a[6]} != stock {b[6]} (the adapter latches faults on these)")
    try:
        ratio = _total_mass(path) / _total_mass(stock)
        if not MASS_RATIO_MIN <= ratio <= MASS_RATIO_MAX:
            probs.append(f"total mass is {ratio:.2f} x stock, outside [{MASS_RATIO_MIN}, {MASS_RATIO_MAX}]")
    except (ValueError, ZeroDivisionError) as e:
        probs.append(str(e))
    return probs


def stock_urdf() -> Path:
    """The gravity URDF DimOS's adapter loads (first existing candidate, as armctl.dimos_limits finds it)."""
    from armctl import dimos_limits

    for p in dimos_limits.candidate_paths():
        if p.exists():
            return p
    raise FileNotFoundError("DimOS's yam_gripper_gravity.urdf not found (see armctl/dimos_limits.py)")


# ---- hardware component -----------------------------------------------------------------------

def request(scales: dict[str, float] | None, urdf: str | Path | None) -> dict | None:
    """The picklable request carried in ``adapter_kwargs[KWARG]``; None = no override."""
    if not scales and not urdf:
        return None
    return {"scale": check_vector(scale_vector(scales)), "urdf": str(Path(urdf).resolve()) if urdf else None}


def apply_to_hardware(hw, scales: dict[str, float] | None, urdf: str | Path | None = None):
    """A new HardwareComponent carrying the gravity request; ``hw`` itself is never mutated.

    No override -> ``hw`` returned as is (defaults unchanged). Real (``openyam_damiao``) -> adapter
    type ``armctl_openyam_damiao``. Sim (``mock_whole_body``) -> adapter kept (it has no gravity),
    request recorded for armctl_info. Any other adapter is refused."""
    req = request(scales, urdf)
    if req is None:
        return hw
    if hw.adapter_type not in (STOCK_ADAPTER, SIM_ADAPTER):
        raise ValueError(f"gravity override only knows {STOCK_ADAPTER!r}/{SIM_ADAPTER!r}, not {hw.adapter_type!r}")
    kwargs = {**hw.adapter_kwargs, KWARG: req}
    adapter = ARMCTL_ADAPTER if hw.adapter_type == STOCK_ADAPTER else hw.adapter_type
    return dataclasses.replace(hw, adapter_type=adapter, adapter_kwargs=kwargs)


def split_kwargs(component) -> tuple[dict | None, dict]:
    """(gravity request or None, the remaining adapter kwargs for DimOS's factory)."""
    kw = dict(component.adapter_kwargs)
    return kw.pop(KWARG, None), kw


def info(component) -> dict:
    """For armctl_info()/logs."""
    req = component.adapter_kwargs.get(KWARG)
    return {"adapter": component.adapter_type,
            "scale": None if req is None else dict(zip(ARM_JOINTS, req["scale"])),
            "urdf": None if req is None else req["urdf"],
            "applied": component.adapter_type == ARMCTL_ADAPTER,
            "note": ("mock adapter has no gravity: request recorded only" if req and component.adapter_type == SIM_ADAPTER
                     else None)}


def describe(scales: dict[str, float] | None, urdf: str | Path | None) -> str:
    if not scales and not urdf:
        return "gravity[dimos-default]"
    parts = [",".join(f"{j}x{v:g}" for j, v in sorted((scales or {}).items()))] if scales else []
    if urdf:
        parts.append(f"urdf={urdf}")
    return "gravity[OVERRIDE " + " ".join(parts) + "]"


def preview(scales: dict[str, float] | None, urdf: str | Path | None, q=READY):
    """(stock, scaled) gravity torque vectors at ``q``, through the real ArmctlOpenYamDamiaoAdapter
    (constructed only: no connect, no CAN). Imports DimOS."""
    import numpy as np

    from armctl.gravity_adapter import ArmctlOpenYamDamiaoAdapter, offline_gravity
    from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig

    cfg = DamiaoRuntimeConfig(gravity_comp=True)
    stock = offline_gravity(ArmctlOpenYamDamiaoAdapter(runtime_config=cfg), q)
    new = offline_gravity(ArmctlOpenYamDamiaoAdapter(runtime_config=cfg, armctl_gravity=request(scales, urdf)), q)
    return np.asarray(stock), np.asarray(new)
