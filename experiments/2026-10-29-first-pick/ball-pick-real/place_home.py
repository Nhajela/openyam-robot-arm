"""Lower to a joint pose, open the gripper, back off up, then fold home. Through the daemon.
Usage: place_home.py q_down q_up   (comma lists)"""
import asyncio
import sys
from interfaces.client import DEFAULT_URL, ArmClient, Refused


async def go(q_down, q_up):
    async with ArmClient(DEFAULT_URL, program="place-home", name="place-home") as c:
        await c.acquire()
        for q, what in ((q_down, "down to release"), (None, "open"), (q_up, "back off up")):
            try:
                if q is None:
                    print(what, await c.do("gripper", wait=False, width=0.73)); await asyncio.sleep(2.0)
                else:
                    print(what, await c.do("move_joints", q=q, speed=0.12, timeout=120))
            except Refused as e:   # an arrival miss (j2 sag) still leaves the arm where it got to
                print(what, "refused:", e)
        print("home", await c.do("home", timeout=120))

f = lambda s: [float(v) for v in s.split(",")]
asyncio.run(go(f(sys.argv[1]), f(sys.argv[2])))
