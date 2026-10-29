"""Send one discrete armctl command (home / ready) through the daemon, waiting for the lease.
Usage: do_cmd.py home|ready"""
import asyncio
import sys
from interfaces.client import DEFAULT_URL, ArmClient


async def go(cmd):
    async with ArmClient(DEFAULT_URL, program="cup-park", name=f"cup-park {cmd}") as c:
        await c.acquire()
        print(await c.do(cmd, timeout=120))

asyncio.run(go(sys.argv[1]))
