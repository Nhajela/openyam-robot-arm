"""Phone-size screenshots of the open play phone page (mode select, Tilt get-set, play).

No Playwright needed: drives a headless Chromium over the DevTools protocol (websockets).
Point it at an openplay.server you started yourself; it never needs the arm. The page is put
into each view through its debug hooks (window.openplay) with a fake arm state and synthetic
device-orientation events, so nothing is sent that could move anything.

    # an openplay.server whose daemon URL points at nothing, on ephemeral ports:
    .venv/bin/python -m openplay.server --daemon ws://127.0.0.1:<free>/armctl --phone-host 127.0.0.1 \
        --phone-port <free> --local-port <free> --key shotkey --host-ip 127.0.0.1
    .venv/bin/python openplay/tools/phone_screens.py "https://127.0.0.1:<phone port>/?k=shotkey" openplay/screenshots
"""
from __future__ import annotations

import asyncio
import base64
import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import urllib.request
from pathlib import Path

W, H, DPR = 390, 844, 3
READY_Q = [0.0, 0.5, 0.8, 0.0, 0.0, 0.0]


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class CDP:
    def __init__(self, ws):
        self.ws, self.n, self.pending = ws, 0, {}
        self.reader = asyncio.create_task(self._rx())

    async def _rx(self):
        async for raw in self.ws:
            m = json.loads(raw)
            f = self.pending.pop(m.get("id"), None)
            if f and not f.done():
                f.set_result(m)

    async def call(self, method: str, **params):
        self.n += 1
        f = asyncio.get_running_loop().create_future()
        self.pending[self.n] = f
        await self.ws.send(json.dumps({"id": self.n, "method": method, "params": params}))
        m = await asyncio.wait_for(f, 30)
        if "error" in m:
            raise RuntimeError(f"{method}: {m['error']}")
        return m.get("result", {})

    async def js(self, expr: str):
        r = await self.call("Runtime.evaluate", expression=expr, awaitPromise=True, returnByValue=True)
        if r.get("exceptionDetails"):
            raise RuntimeError(f"JS error in {expr[:80]!r}: {r['exceptionDetails']}")
        return r.get("result", {}).get("value")

    async def shot(self, path: Path, full: bool = False):
        params = {"format": "png"}
        if full:
            h = await self.js("document.documentElement.scrollHeight")
            params.update(captureBeyondViewport=True, clip={"x": 0, "y": 0, "width": W, "height": h, "scale": 1})
        r = await self.call("Page.captureScreenshot", **params)
        path.write_bytes(base64.b64decode(r["data"]))
        print("wrote", path)


async def orient(c: CDP, beta: float, gamma: float, seconds: float = 0.0, hz: float = 30):
    end = time.monotonic() + seconds
    while True:
        await c.js(f"window.dispatchEvent(Object.assign(new Event('deviceorientation'), {{beta: {beta}, gamma: {gamma}}}))")
        if time.monotonic() >= end:
            return
        await asyncio.sleep(1 / hz)


async def run(url: str, out: Path, chrome: str) -> None:
    import websockets

    port = _free_port()
    # snap Chromium can only write under $HOME
    prof = Path(tempfile.mkdtemp(prefix="openplay-shot-", dir=Path.home() / "snap/chromium/common")
                if "snap" in chrome else tempfile.mkdtemp(prefix="openplay-shot-"))
    proc = subprocess.Popen([chrome, "--headless=new", f"--remote-debugging-port={port}", f"--user-data-dir={prof}",
                             "--ignore-certificate-errors", "--no-first-run", "--hide-scrollbars",
                             f"--window-size={W},{H}", "about:blank"],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for _ in range(100):
            try:
                tabs = json.load(urllib.request.urlopen(f"http://127.0.0.1:{port}/json", timeout=1))
                page = next(t for t in tabs if t.get("type") == "page")
                break
            except Exception:
                await asyncio.sleep(0.2)
        else:
            raise RuntimeError("chromium did not start")
        async with websockets.connect(page["webSocketDebuggerUrl"], max_size=None) as ws:
            c = CDP(ws)
            await c.call("Page.enable")
            await c.call("Emulation.setDeviceMetricsOverride", width=W, height=H, deviceScaleFactor=DPR, mobile=True)
            await c.call("Emulation.setTouchEmulationEnabled", enabled=True, maxTouchPoints=5)
            await c.call("Page.navigate", url=url)
            for _ in range(100):
                if await c.js("!!(window.openplay && window.openplay.S.welcome)"):
                    break
                await asyncio.sleep(0.1)
            else:
                raise RuntimeError("the page never got a welcome from the server")
            out.mkdir(parents=True, exist_ok=True)
            arm = lambda q, busy="null": c.js(  # noqa: E731  a fake arm state, page-side only
                f"openplay.handle({{op:'arm', q:{json.dumps(q)}, grip:1, moving:false, busy:{busy}, mode:null}})")

            await c.js("document.getElementById('name').value='Priya'; localStorage.setItem('openplay-name','Priya')")
            await c.js("openplay.startTurnScreen(600)")
            await arm(READY_Q)
            await asyncio.sleep(0.9)
            await c.shot(out / "1-mode-select.png")

            await c.js("openplay.openMode('tilt', {ready: false})")
            await asyncio.sleep(0.5)
            await c.shot(out / "2-tilt-getset.png")
            await c.js("document.getElementById('sensorsBtn').click()")
            await orient(c, 17, 5, 0.3)
            await c.js("document.getElementById('aim').scrollIntoView({block:'center'})")
            await asyncio.sleep(0.2)
            await c.shot(out / "3-tilt-getset-aiming.png")
            await orient(c, 29, 1.5, 0.55)
            await c.shot(out / "4-tilt-getset-holding.png")
            await orient(c, 30, 1, 0.6)
            await asyncio.sleep(1.2)  # saved -> the play view opens by itself
            assert await c.js("openplay.S.view") == "play", "tilt did not reach the play view"
            await c.shot(out / "5-play-tilt.png")

            # press the pad while the phone is 20 degrees off the start pose: it waits
            await orient(c, 50, 2)
            await c.js("""(() => { const p = document.getElementById('tiltPad'), r = p.getBoundingClientRect();
                p.dispatchEvent(new PointerEvent('pointerdown', {pointerId: 7, bubbles: true, clientX: r.x + r.width/2, clientY: r.y + r.height/2})); })()""")
            await asyncio.sleep(0.3)
            await c.shot(out / "6-play-tilt-back-to-start.png")
            await c.js("document.getElementById('tiltPad').dispatchEvent(new PointerEvent('pointerup', {pointerId: 7, bubbles: true}))")

            await arm([0.0] * 6)  # folded at home
            await c.js("openplay.showView('modes')")
            await asyncio.sleep(0.9)
            await c.shot(out / "7-modes-after-home.png")

            await arm(READY_Q)
            await c.js("openplay.openMode('sticks', {ready: false})")
            await asyncio.sleep(0.5)
            await c.shot(out / "8-sticks-getset.png")
            await c.js("openplay.startPlay()")
            await asyncio.sleep(0.5)
            await c.shot(out / "9-play-sticks.png")
            c.reader.cancel()
    finally:
        proc.terminate()
        try:
            proc.wait(5)
        except subprocess.TimeoutExpired:
            proc.kill()
        shutil.rmtree(prof, ignore_errors=True)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    chrome = os.environ.get("CHROME") or shutil.which("chromium") or shutil.which("google-chrome") or shutil.which("chromium-browser")
    if not chrome:
        sys.exit("no chromium/google-chrome on PATH (set CHROME=...)")
    asyncio.run(run(sys.argv[1], Path(sys.argv[2]), chrome))
