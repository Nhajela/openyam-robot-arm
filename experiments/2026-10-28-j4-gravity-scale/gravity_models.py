#!/usr/bin/env python3
"""Compare gravity models for the OpenYAM wrist and fit a per-joint gravity scale (offline, no hardware).

Inputs: the j4 hold-test CSVs (experiments/2026-10-28-laptop-systems-check/logs/j4-hold-*.csv) and
four models:
  dimos_gravity   ~/.local/share/dimos/repo/data/yam_description/urdf/yam_gripper_gravity.urdf
                  (what DimOS's Damiao adapter feeds forward on the real arm)
  dimos_i2rt      ~/.local/share/dimos/repo/data/yam_description/i2rt/yam.urdf (DimOS planning model)
  i2rt_urdf       ref/i2rt/yam.urdf (i2rt main, fetched 2026-10-28)
  i2rt_crank      ref/i2rt/yam.xml + ref/i2rt/crank_4310.xml (i2rt's own arm + gripper MJCF, 0.60 kg gripper)

For each pose it uses the measured q (mean of the last 5 s of the hold) and prints tau_j4 per model,
the per-body breakdown of tau_j4, and the "true" holding torque two ways:
  A  tau_ff - kp*err  (MIT-mode balance: the torque the motor was told to make; friction ignored)
  B  the motor's own torque reply on CAN (console c_tau)
Then least-squares fits tau_true = s*tau_model and tau_true = s*tau_model + c.

    .venv/bin/python experiments/2026-10-28-j4-gravity-scale/gravity_models.py
"""
from __future__ import annotations

import csv
import re
import tempfile
from pathlib import Path

import numpy as np
import pinocchio as pin

HERE = Path(__file__).resolve().parent
LOGS = HERE.parent / "2026-10-28-laptop-systems-check" / "logs"
DATA = Path.home() / ".local/share/dimos/repo/data/yam_description"
RUNS = {  # csv, kp per joint (j2, j3, j4)
    "run1 kp4=10": (LOGS / "j4-hold-20261028-130801.csv", (80.0, 80.0, 10.0)),
    "run2 kp4=22": (LOGS / "j4-hold-20261028-132005.csv", (80.0, 80.0, 22.0)),
}
TARGET = {"j4=+0.0": 0.0, "j4=-0.3": -0.3, "j4=+0.3": 0.3, "j4=+0.7": 0.7}
READY_J2, READY_J3 = 0.5, 0.8


def arm_model(model: pin.Model, arm: list[str]) -> pin.Model:
    locked = [i for i, n in enumerate(model.names) if i and n not in arm]
    if locked:
        model = pin.buildReducedModel(model, locked, pin.neutral(model))
    assert tuple(model.names[1:]) == tuple(arm), model.names
    return model


def load_models() -> dict[str, pin.Model]:
    out = {}
    out["dimos_gravity"] = arm_model(pin.buildModelFromUrdf(str(DATA / "urdf/yam_gripper_gravity.urdf")),
                                     [f"yam_joint{i}" for i in range(1, 7)])
    out["dimos_i2rt"] = arm_model(pin.buildModelFromUrdf(str(DATA / "i2rt/yam.urdf")),
                                  [f"joint{i}" for i in range(1, 7)])
    out["i2rt_urdf"] = arm_model(pin.buildModelFromUrdf(str(HERE / "ref/i2rt/yam.urdf")),
                                 [f"joint{i}" for i in range(1, 7)])
    # i2rt's own composition: yam.xml's "gripper" mount body + the crank_4310 gripper body inside it.
    arm_xml = (HERE / "ref/i2rt/yam.xml").read_text()
    grip_xml = (HERE / "ref/i2rt/crank_4310.xml").read_text()
    body = re.search(r"<worldbody>\s*(<body.*</body>)\s*</worldbody>", grip_xml, re.S).group(1)
    body = re.sub(r"<geom[^>]*/>", "", body).replace('name="gripper"', 'name="crank_gripper"', 1)
    arm_xml = re.sub(r"<geom[^>]*/>", "", arm_xml)
    arm_xml = re.sub(r"<asset>.*?</asset>", "", arm_xml, flags=re.S)
    arm_xml = arm_xml.replace(
        '<joint name="joint6" pos="0 0 0" axis="0 0 1" type="hinge" range="-2.0944 2.0944" actuatorfrcrange="-10 10"/>',
        '<joint name="joint6" pos="0 0 0" axis="0 0 1" type="hinge" range="-2.0944 2.0944" actuatorfrcrange="-10 10"/>'
        + body)
    with tempfile.TemporaryDirectory() as d:
        tmp = Path(d) / "i2rt_crank_combined.xml"
        tmp.write_text(arm_xml)
        out["i2rt_crank"] = arm_model(pin.buildModelFromMJCF(str(tmp)), [f"joint{i}" for i in range(1, 7)])
    return out


def tau(model: pin.Model, q6) -> np.ndarray:
    return np.asarray(pin.computeGeneralizedGravity(model, model.createData(), np.asarray(q6, float)))


def breakdown_j4(model: pin.Model, q6) -> dict[str, float]:
    """tau_j4 contributed by each body (all other inertias zeroed)."""
    out = {}
    for jid in range(4, model.njoints):  # bodies moved by j4: joints 4..6 (+ fixed children lumped in)
        m = model.copy()
        for k in range(1, m.njoints):
            if k != jid:
                m.inertias[k] = pin.Inertia.Zero()
        out[f"{model.names[jid]} body (m={model.inertias[jid].mass:.3f})"] = float(tau(m, q6)[3])
    return out


def load_runs():
    rows = []
    for run, (path, kps) in RUNS.items():
        data = list(csv.DictReader(path.open()))
        for pose, t4 in TARGET.items():
            r = [x for x in data if x["pose"] == pose]
            tend = max(float(x["t"]) for x in r)
            r = [x for x in r if float(x["t"]) >= tend - 5]
            q = np.array([[float(x[f"q{i}"]) for i in range(1, 7)] for x in r]).mean(0)
            ct = np.mean([float(x["c_tau"]) for x in r if x["c_tau"]])
            rows.append({"run": run, "pose": pose, "q": q, "c_tau": ct, "kp": kps,
                         "target": np.array([0.0, READY_J2, READY_J3, t4, 0.0, 0.0])})
    return rows


def fit(x, y):
    x, y = np.asarray(x), np.asarray(y)
    s = float(x @ y / (x @ x))
    r1 = y - s * x
    A = np.c_[x, np.ones_like(x)]
    (s2, c2), *_ = np.linalg.lstsq(A, y, rcond=None)
    r2 = y - A @ [s2, c2]
    return s, r1, float(s2), float(c2), r2


def main() -> None:
    models = load_models()
    print("Mass carried by j4 (link4 + everything outboard), kg:")
    for name, m in models.items():
        print(f"  {name:14s} {sum(m.inertias[k].mass for k in range(4, m.njoints)):.3f}"
              f"   per body: " + ", ".join(f"{m.names[k]}={m.inertias[k].mass:.3f}" for k in range(4, m.njoints)))
    rows = load_runs()
    print("\ntau_j4 at the measured hold q (N·m):")
    print(f"  {'run':12s} {'pose':8s} " + " ".join(f"{n:>13s}" for n in models) + f" {'A=ff-kp*err':>11s} {'B=CAN tau':>9s}")
    for r in rows:
        ff = tau(models["dimos_gravity"], r["q"])
        err = r["q"] - r["target"]
        r["ff"] = ff
        r["A"] = ff - np.array([0, *r["kp"], 0, 0]) * err  # per joint j1..j6 (j1/j5/j6 kp unused)
        r["err"] = err
        print(f"  {r['run']:12s} {r['pose']:8s} " + " ".join(f"{tau(m, r['q'])[3]:13.3f}" for m in models.values())
              + f" {r['A'][3]:11.3f} {r['c_tau']:9.3f}")
    print("\nPer-body share of tau_j4 at run1 j4=-0.3 (heaviest pose):")
    q = rows[1]["q"]
    for name in ("dimos_gravity", "i2rt_urdf", "i2rt_crank"):
        print(f"  {name}: " + ", ".join(f"{k}: {v:.3f}" for k, v in breakdown_j4(models[name], q).items()))

    x = [r["ff"][3] for r in rows]
    for label, y in (("A (ff - kp*err)", [r["A"][3] for r in rows]), ("B (CAN tau)", [r["c_tau"] for r in rows])):
        s, r1, s2, c2, r2 = fit(x, y)
        print(f"\nj4 fit vs DimOS feedforward, truth={label}, n={len(x)}:")
        print(f"  (a) tau = {s:.3f}*model           residuals {np.round(r1, 3).tolist()}  rms {np.sqrt(np.mean(r1**2)):.3f}")
        print(f"  (b) tau = {s2:.3f}*model {c2:+.3f}    residuals {np.round(r2, 3).tolist()}  rms {np.sqrt(np.mean(r2**2)):.3f}")
    # Mass fit: tau4 = (link4+link5 share) + k * (gripper body share) in the DimOS model, truth A.
    dm = models["dimos_gravity"]
    t6 = np.array([breakdown_j4(dm, r["q"])[f"{dm.names[6]} body (m={dm.inertias[6].mass:.3f})"] for r in rows])
    t45 = np.array([r["ff"][3] for r in rows]) - t6
    yA = np.array([r["A"][3] for r in rows])
    k = float(t6 @ (yA - t45) / (t6 @ t6))
    res = yA - t45 - k * t6
    print(f"\nGripper-mass fit on the DimOS model (truth A): k = {k:.3f} -> gripper body "
          f"{k * dm.inertias[6].mass:.3f} kg (stock {dm.inertias[6].mass:.3f}); residuals {np.round(res, 3).tolist()} "
          f"rms {np.sqrt(np.mean(res**2)):.3f}")
    for j, name in ((1, "j2"), (2, "j3")):
        x = [r["ff"][j] for r in rows]
        y = [r["A"][j] for r in rows]
        s, r1, *_ = fit(x, y)
        print(f"\n{name} (no torque data; A = ff - 80*err only): ff {np.round(x, 3).tolist()}"
              f"\n   err {np.round([r['err'][j] for r in rows], 4).tolist()}\n   A {np.round(y, 3).tolist()}"
              f"  -> s = {s:.3f} (rms {np.sqrt(np.mean(r1**2)):.3f}); j2/j3 barely move between poses, so no affine fit")
        for mname in ("i2rt_urdf", "i2rt_crank"):
            print(f"   {mname} tau_{name}: {np.round([tau(models[mname], r['q'])[j] for r in rows], 3).tolist()}")


if __name__ == "__main__":
    main()
