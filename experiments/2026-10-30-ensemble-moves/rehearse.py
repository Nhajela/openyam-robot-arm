#!/usr/bin/env python3
"""Play playables on a MOCK armctl daemon (no hardware): validate, ready, play with auto-confirm.
Refuses any daemon whose backend isn't 'mock'. Usage: rehearse.py ws://127.0.0.1:8692/armctl a.json [b.json ...]"""
import asyncio, json, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from interfaces.client import ArmClient, Refused

async def main(url, paths):
    async with ArmClient(url, program="script", name="rehearse") as a:
        await a.wait_state()
        if a.backend != "mock":
            sys.exit(f"refusing: backend is {a.backend!r}, not mock")
        await a.acquire(timeout=10)
        for path in paths:
            p = json.loads(Path(path).read_text())
            r = await a.validate(p)
            print(f"{p['name']}: validate ok={r.get('ok')} {r.get('violations', [])[:3]}", flush=True)
            if not r.get("ok"):
                continue
            await a.do("ready", timeout=60)
            t = time.monotonic()
            try:
                done = await a.play(p, auto_confirm=True, timeout=120)
                print(f"{p['name']}: done in {time.monotonic() - t:.1f} s: {done}", flush=True)
            except Refused as e:
                print(f"{p['name']}: REFUSED {e}", flush=True)
        await a.do("home", timeout=60)

asyncio.run(main(sys.argv[1], sys.argv[2:]))
