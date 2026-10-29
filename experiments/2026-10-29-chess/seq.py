"""Run a short sequence through the daemon: steps are 'q:<6 comma floats>' (slow move_joints),
'g:<width 0..1>' (gripper), 'home'. An arrival miss (j2 sag) is reported and the sequence goes on.
Usage: seq.py STEP [STEP ...]"""
import asyncio
import sys
from interfaces.client import DEFAULT_URL, ArmClient, Refused


async def go(steps):
    async with ArmClient(DEFAULT_URL, program="seq", name="seq") as c:
        await c.acquire()
        for s in steps:
            try:
                if s.startswith("q:"):
                    print(s[:40], await c.do("move_joints", q=[float(v) for v in s[2:].split(",")], speed=0.12, timeout=120))
                elif s.startswith("g:"):
                    print(s, await c.do("gripper", wait=False, width=float(s[2:]))); await asyncio.sleep(2.0)
                elif s == "home":
                    print(s, await c.do("home", timeout=120))
            except Refused as e:
                print(s[:40], "refused:", e)

asyncio.run(go(sys.argv[1:]))
