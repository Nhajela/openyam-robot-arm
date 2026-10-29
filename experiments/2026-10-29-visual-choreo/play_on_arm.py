#!/usr/bin/env python3
"""Validate a playable on the real daemon, go to ready, and request Play (the operator presses
Play in the lab console). Usage: play_on_arm.py playables/x.json"""
import asyncio, json, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from interfaces.client import ArmClient, Refused

async def main(path):
    p = json.loads(Path(path).read_text())
    async with ArmClient("ws://127.0.0.1:8091/armctl", program="script", name="fable-play") as a:
        await a.wait_state(); await a.acquire(timeout=10)
        r = await a.validate(p); print("validate:", r.get("ok"), r.get("violations", [])[:3], flush=True)
        if not r.get("ok"): return
        try: await a.do("ready", timeout=60)
        except Refused as e: print("ready:", e)
        print("REQUESTED: press Play in the console (http://127.0.0.1:8090) for", r["name"], flush=True)
        done = await a.play(p, timeout=600); print("done:", done, flush=True)

asyncio.run(main(sys.argv[1]))
