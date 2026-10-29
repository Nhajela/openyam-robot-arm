"""Open the gripper (release), wait, then fold home. Through the daemon."""
import asyncio
from interfaces.client import DEFAULT_URL, ArmClient


async def go():
    async with ArmClient(DEFAULT_URL, program="release-home", name="release-home") as c:
        await c.acquire()
        print(await c.do("gripper", wait=False, width=1.0))
        await asyncio.sleep(3.0)
        print(await c.do("home", timeout=120))

asyncio.run(go())
