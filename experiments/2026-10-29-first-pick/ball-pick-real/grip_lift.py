"""Close the gripper on what's between the jaws, then lift to a given pose (slow). Through the daemon.
Usage: grip_lift.py GRIP q1,..,q6"""
import asyncio
import sys
from interfaces.client import DEFAULT_URL, ArmClient


async def go(g, q):
    async with ArmClient(DEFAULT_URL, program="grip-lift", name="grip-lift") as c:
        await c.acquire()
        print(await c.do("gripper", wait=False, width=g))
        await asyncio.sleep(2.0)
        await c.wait_state(2.0)
        print("grip after close:", c.state.get("grip"))
        print(await c.do("move_joints", q=q, speed=0.12, timeout=120))
        await c.wait_state(2.0)
        print("grip after lift:", c.state.get("grip"))

asyncio.run(go(float(sys.argv[1]), [float(v) for v in sys.argv[2].split(",")]))
