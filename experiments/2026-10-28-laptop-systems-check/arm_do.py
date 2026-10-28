#!/usr/bin/env python3
"""Run one or more armctl commands against the local daemon and print the joint state.

    .venv/bin/python experiments/2026-10-28-laptop-systems-check/arm_do.py state
    .venv/bin/python experiments/2026-10-28-laptop-systems-check/arm_do.py ready home
    .venv/bin/python experiments/2026-10-28-laptop-systems-check/arm_do.py gripper=1

Every motion goes through the daemon (soft limits, speed caps). Ctrl-C sends stop.
"""
import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from interfaces.client import ArmClient  # noqa: E402

URL = "ws://127.0.0.1:8091/armctl"


def show(arm: ArmClient, label: str) -> None:
    s = arm.state
    q = " ".join(f"{x:+.3f}" for x in arm.q)
    print(f"[{label}] q=[{q}] grip={s.get('grip')} faults={s.get('faults')} backend={arm.backend}", flush=True)


async def main(cmds: list[str]) -> None:
    async with ArmClient(URL, program="script", name="systems-check") as arm:
        await arm.wait_state()
        show(arm, "start")
        if cmds == ["state"]:
            return
        await arm.acquire(timeout=10)
        for c in cmds:
            name, _, val = c.partition("=")
            args = {"width": float(val)} if name == "gripper" else {}
            print(f"-> {name} {args}", flush=True)
            try:
                # gripper is ack-only (the daemon never sends done for it)
                done = await arm.do(name, wait=name != "gripper", timeout=90, **args)
                if name == "gripper":
                    await asyncio.sleep(2.0)
            except (asyncio.CancelledError, KeyboardInterrupt):
                await arm.stop()
                raise
            print(f"   {done}", flush=True)
            await asyncio.sleep(0.3)
            show(arm, name)


if __name__ == "__main__":
    try:
        asyncio.run(main(sys.argv[1:] or ["state"]))
    except KeyboardInterrupt:
        print("interrupted (stop sent)")
