#!/usr/bin/env python3
"""DimOS coordinator for armctl's DimosBackend: eef-twist IK + joint trajectory + gripper.

One ``ArmTwistCoordinator`` (served as ``ControlCoordinator``) with exactly three tasks:

* ``eef_twist_arm``    (type eef_twist, priority 10): Pink differential IK on twists from
  ``/ee_twist_command``. ``command_limit_margin`` 0.1 rad keeps every IK command 0.1 rad inside
  the URDF limits (the same soft box as armctl), joint speed capped at 0.3 rad/s (armctl's caps),
  and its own 0.3 s command timeout (a third deadman layer behind the daemon and the backend).
* ``joint_trajectory`` (type trajectory, priority 20, arm joints only): home / ready /
  move_joints / playables and joint-velocity streams. Its per-joint velocity limit (0.5 rad/s)
  is a backstop: armctl plans at 0.3 rad/s. Priority 20 preempts the twist task.
* ``openyam_gripper``  (type gripper, priority 20): normalised gripper target.

No keyboard module, no planner, no viewer. The class adds one RPC, ``armctl_info()``, that the
DimosBackend uses to check what it is talking to (adapter type, i.e. sim or real, and the
task parameters above) before it sends anything.

Safety (CLAUDE.md):
* ``--simulation`` uses the in-memory ``mock_whole_body`` adapter (no CAN at all).
* Real hardware needs BOTH ``--real`` and ``--can-port can0`` AND an explicitly exported,
  non-default ``ZENOH_SCOUT_ADDR`` (so no other DimOS process on the default bus can attach to
  this coordinator, and armctl's private-bus client finds it). Otherwise it refuses.
* It refuses to start when another DimOS run is active (``pgrep -af bin/dimos``).
* Stopping (Ctrl-C) does NOT reliably drop torque: run ``tools/disable_all.py`` afterwards.

Exact commands (repo root)
--------------------------
Simulation (what the tests use)::

    ZENOH_SCOUT_ADDR=224.0.0.123:7449 .venv/bin/python -m armctl.dimos_coordinator --simulation

Real arm (ONLY with the person at the arm agreeing; arm folded on its stops; in tmux)::

    ZENOH_SCOUT_ADDR=224.0.0.123:7449 .venv/bin/python -m armctl.dimos_coordinator --real --can-port can0

Then the daemon on the same bus::

    ZENOH_SCOUT_ADDR=224.0.0.123:7449 .venv/bin/python -m armctl.daemon --backend dimos

Options: ``--ik-margin`` (>= 0.1), ``--joint-vel`` (eef-twist joint speed cap, <= 0.3 on real),
``--traj-vel-limit`` (trajectory backstop, <= 1.0), ``--twist-timeout`` (s), ``--with-planner``.

Holding gains (``armctl/gains.py``): ``--kp JOINT=VALUE`` / ``--kd JOINT=VALUE``, repeatable, override
single entries of DimOS's WholeBodyConfig (default kp 80,80,80,10,10,10 / kd 5,5,5,1.5,1.5,1.5);
joints not named keep the default. Caps: kp <= 120 on j1-j3, <= 40 on j4-j6; kd <= 8 on j1-j3,
<= 5 on j4-j6. E.g. stiffer j4 against its gravity sag::

    ... -m armctl.dimos_coordinator --simulation --kp j4=22 --kd j4=2     # plumbing check first
    ... -m armctl.dimos_coordinator --real --can-port can0 --kp j4=22 --kd j4=2

Try in ``--simulation`` first (the mock adapter ignores gains: it only checks the plumbing). On
the real arm watch j4's temperature and listen for buzz/chatter; stop and drop the override at the
first sign of either. The effective gains are in the startup line and in ``armctl_info()["gains"]``.
"""
from __future__ import annotations

import os
import subprocess
import sys

# ---- bus isolation: before any dimos import -------------------------------------------
PRIVATE_SCOUT_ADDR = "224.0.0.123:7449"
DEFAULT_SCOUT_ADDRS = ("", "224.0.0.224:7446")
# "Explicit" must survive the re-import in __main__ below (the first import's setdefault would
# otherwise make the second import think the user exported it): remember defaulting in the env.
if os.environ.get("ZENOH_SCOUT_ADDR", "").strip() in DEFAULT_SCOUT_ADDRS and "ZENOH_SCOUT_ADDR" not in os.environ:
    os.environ["ZENOH_SCOUT_ADDR"] = PRIVATE_SCOUT_ADDR
    os.environ["ARMCTL_SCOUT_DEFAULTED"] = "1"
SCOUT_EXPLICIT = (os.environ.get("ARMCTL_SCOUT_DEFAULTED") != "1"
                  and os.environ["ZENOH_SCOUT_ADDR"].strip() not in DEFAULT_SCOUT_ADDRS)
if os.environ["ZENOH_SCOUT_ADDR"].strip() in DEFAULT_SCOUT_ADDRS:
    raise SystemExit(
        "armctl.dimos_coordinator refuses the default zenoh bus (anything on the LAN could attach). "
        f"export ZENOH_SCOUT_ADDR={PRIVATE_SCOUT_ADDR}"
    )
os.environ.setdefault("DIMOS_TRANSPORT", "zenoh")

from dimos.control.coordinator import TaskConfig  # noqa: E402
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task  # noqa: E402
from dimos.core.core import rpc  # noqa: E402
from dimos.core.global_config import global_config  # noqa: E402
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator  # noqa: E402
from dimos.robot.manipulators.common.topics import EEF_TWIST_TASK_NAME  # noqa: E402
from armctl import gains as _gains  # noqa: E402
from dimos.robot.manipulators.openyam.config import (  # noqa: E402
    OPENYAM_ARM_JOINTS,
    OPENYAM_GRIPPER_JOINT,
    OPENYAM_HARDWARE_ID,
    make_openyam_model_config,
    openyam_hardware,
)

LAUNCHER = "armctl.dimos_coordinator/1"
TWIST_TASK = EEF_TWIST_TASK_NAME  # "eef_twist_arm"
TRAJ_TASK = "joint_trajectory"
GRIPPER_TASK = f"{OPENYAM_HARDWARE_ID}_gripper"  # "openyam_gripper"
EXPECTED_TASKS = (TWIST_TASK, TRAJ_TASK, GRIPPER_TASK)
SIM_ADAPTER = "mock_whole_body"


class ArmctlTwistCoordinator(ArmTwistCoordinator):
    """ArmTwistCoordinator + ``armctl_info()`` so a client can verify what it drives."""

    @rpc
    def armctl_info(self) -> dict:
        hw = {c.hardware_id: c.adapter_type for c in self.config.hardware}
        tasks = {t.name: {"type": t.type, "priority": t.priority, "joints": list(t.joint_names),
                          "params": {k: v for k, v in t.params.items() if k != "robot_model"}}
                 for t in self.config.tasks}
        return {"launcher": LAUNCHER, "adapters": hw,
                "simulation": all(a == SIM_ADAPTER for a in hw.values()),
                "can_port": global_config.can_port, "tasks": tasks, "pid": os.getpid(),
                "gains": {c.hardware_id: _gains.gains_dict(c.wb_config) for c in self.config.hardware},
                "scout_addr": os.environ.get("ZENOH_SCOUT_ADDR")}


def build(*, ik_margin: float, joint_vel: float, traj_vel: float, twist_timeout: float,
          with_planner: bool, kp: dict[str, float] | None = None, kd: dict[str, float] | None = None):
    hw = openyam_hardware()
    hw.wb_config = _gains.override_wb_config(hw.wb_config, kp or {}, kd or {})
    model = make_openyam_model_config()
    arm = list(OPENYAM_ARM_JOINTS)
    coord = ArmctlTwistCoordinator.blueprint(
        instance_name="ControlCoordinator",
        hardware=[hw],
        tasks=[
            TaskConfig(
                name=TWIST_TASK, type="eef_twist", joint_names=arm, priority=10,
                params={
                    "robot_model": model,
                    "target_frame": "gripper_tip",
                    "command_limit_margin": ik_margin,
                    "max_joint_velocity_rad_s": joint_vel,
                    "joint_velocity_limits_rad_s": {j: joint_vel for j in arm},
                    "timeout": twist_timeout,
                },
            ),
            joint_trajectory_task(arm, priority=20, start_position_tolerance=0.05,
                                  velocity_limits={j: traj_vel for j in arm}),
            TaskConfig(name=GRIPPER_TASK, type="gripper", joint_names=[OPENYAM_GRIPPER_JOINT], priority=20),
        ],
    )
    if with_planner:
        from dimos.core.coordination.blueprints import autoconnect
        from dimos.robot.manipulators.common.blueprints import planner
        return hw, autoconnect(planner(model=model), coord)
    return hw, coord


RUN_PATTERN = "bin/dimos|armctl.dimos_coordinator|armctl_blueprints.py"


def _other_dimos_running() -> list[str]:
    """Other DimOS runs: `dimos run`, this launcher, the experiment launcher (python processes)."""
    try:
        out = subprocess.run(["pgrep", "-af", RUN_PATTERN], capture_output=True, text=True).stdout
    except OSError:
        return []
    mine = {str(os.getpid()), str(os.getppid())}
    return [ln for ln in out.splitlines() if len(ln.split()) > 1 and "python" in ln.split()[1]
            and ln.split()[0] not in mine]


def main(argv: list[str] | None = None) -> int:
    import argparse

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--simulation", action="store_true", help="in-memory mock adapter, no hardware")
    mode.add_argument("--real", action="store_true", help="REAL motors (needs --can-port and a private ZENOH_SCOUT_ADDR)")
    ap.add_argument("--can-port", help="CAN interface for --real (can0)")
    ap.add_argument("--ik-margin", type=float, default=0.1, help="eef-twist command_limit_margin, rad (>= 0.1)")
    ap.add_argument("--joint-vel", type=float, default=0.3, help="eef-twist joint speed cap, rad/s")
    ap.add_argument("--traj-vel-limit", type=float, default=0.5, help="trajectory task velocity backstop, rad/s")
    ap.add_argument("--twist-timeout", type=float, default=0.3, help="eef-twist command timeout, s")
    ap.add_argument("--with-planner", action="store_true", help="also run the DimOS planner (not needed by armctl)")
    ap.add_argument("--kp", action="append", metavar="JOINT=VALUE",
                    help="override one joint's holding kp, repeatable (e.g. --kp j4=22); "
                         "caps: j1-j3 <= 120, j4-j6 <= 40")
    ap.add_argument("--kd", action="append", metavar="JOINT=VALUE",
                    help="override one joint's holding kd, repeatable (e.g. --kd j4=2); "
                         "caps: j1-j3 <= 8, j4-j6 <= 5")
    a = ap.parse_args(argv)

    if a.ik_margin < 0.1:
        ap.error("--ik-margin must be >= 0.1 (armctl's soft box)")
    if not 0 < a.joint_vel <= (0.3 if a.real else 1.0):
        ap.error("--joint-vel must be in (0, 0.3] on the real arm (armctl caps), (0, 1] in sim")
    if not 0 < a.traj_vel_limit <= 1.0:
        ap.error("--traj-vel-limit must be in (0, 1]")
    if not 0 < a.twist_timeout <= 0.5:
        ap.error("--twist-timeout must be in (0, 0.5]")
    try:
        kp_over = _gains.parse_overrides(a.kp, "kp")
        kd_over = _gains.parse_overrides(a.kd, "kd")
    except ValueError as e:
        ap.error(str(e))
    if a.simulation:
        if a.can_port or os.environ.get("CAN_PORT"):
            ap.error("--simulation must not have a CAN port (unset CAN_PORT, drop --can-port)")
        os.environ["SIMULATION"] = "1"
        os.environ.pop("CAN_PORT", None)
        global_config.update(simulation="1", can_port=None)
    else:
        if not a.can_port:
            ap.error("--real needs --can-port can0")
        if not SCOUT_EXPLICIT:
            ap.error(f"--real needs an explicitly exported private bus: export ZENOH_SCOUT_ADDR={PRIVATE_SCOUT_ADDR}")
        os.environ.pop("SIMULATION", None)
        os.environ["CAN_PORT"] = a.can_port
        global_config.update(simulation="", can_port=a.can_port)
    global_config.update(viewer="none")

    others = _other_dimos_running()
    if others:
        print("refusing: another DimOS run is active:\n  " + "\n  ".join(others), file=sys.stderr)
        return 1

    hw, bp = build(ik_margin=a.ik_margin, joint_vel=a.joint_vel, traj_vel=a.traj_vel_limit,
                   twist_timeout=a.twist_timeout, with_planner=a.with_planner, kp=kp_over, kd=kd_over)
    if a.simulation and hw.adapter_type != SIM_ADAPTER:
        print(f"refusing: --simulation but adapter is {hw.adapter_type!r}", file=sys.stderr)
        return 1
    if a.real and hw.adapter_type == SIM_ADAPTER:
        print("refusing: --real but DimOS chose the mock adapter", file=sys.stderr)
        return 1
    print(f"armctl.dimos_coordinator: mode={'SIMULATION' if a.simulation else 'REAL can=' + a.can_port} "
          f"adapter={hw.adapter_type} bus={os.environ['ZENOH_SCOUT_ADDR']} tasks={list(EXPECTED_TASKS)} "
          f"ik_margin={a.ik_margin} joint_vel={a.joint_vel} traj_vel_limit={a.traj_vel_limit} "
          f"twist_timeout={a.twist_timeout} planner={a.with_planner} "
          f"gains[{'OVERRIDE ' + ','.join(sorted({*kp_over, *kd_over})) if kp_over or kd_over else 'dimos-default'}]: "
          f"{_gains.describe(hw.wb_config)}", flush=True)
    if a.real:
        print("REAL ARM: Ctrl-C stops the coordinator but may leave torque on: run tools/disable_all.py after.",
              flush=True)

    from dimos.core.coordination.module_coordinator import ModuleCoordinator

    # build().loop() serves the coordinator RPC (needed by Dimos.connect) and blocks until Ctrl-C.
    ModuleCoordinator.build(bp).loop()
    return 0


if __name__ == "__main__":
    # Re-import under the package name so DimOS workers unpickle ArmctlTwistCoordinator as
    # armctl.dimos_coordinator.ArmctlTwistCoordinator, not __main__.ArmctlTwistCoordinator.
    from armctl import dimos_coordinator as _self

    raise SystemExit(_self.main())
