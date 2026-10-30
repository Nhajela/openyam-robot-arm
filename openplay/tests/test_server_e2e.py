"""End to end: mock armctl daemon + openplay.server as subprocesses, two phones over wss and
the display over ws. Mock only (never hardware). Ports 8591 (daemon), 8550 (phones), 8551
(display); the test refuses to run if any is taken, and kills exactly the processes it started.

    .venv/bin/python -m pytest -q openplay/tests/test_server_e2e.py     # about 60 s
"""
from __future__ import annotations

import asyncio
import json
import os
import socket
import ssl
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
DAEMON_PORT, PHONE_PORT, LOCAL_PORT = 8591, 8550, 8551
KEY = "e2e-room-key-123"
TURN_S = 8
SSL_CTX = ssl.create_default_context()
SSL_CTX.check_hostname = False
SSL_CTX.verify_mode = ssl.CERT_NONE


def _port_free(port: int) -> bool:
    with socket.socket() as s:
        return s.connect_ex(("127.0.0.1", port)) != 0


def _wait_http(url: str, timeout: float, ctx=None) -> None:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            urllib.request.urlopen(url, timeout=1, context=ctx).read()
            return
        except urllib.error.HTTPError:
            return  # it answered
        except OSError:
            time.sleep(0.2)
    raise TimeoutError(f"{url} did not come up")


@pytest.fixture(scope="module")
def stack(tmp_path_factory):
    for p in (DAEMON_PORT, PHONE_PORT, LOCAL_PORT):
        if not _port_free(p):
            pytest.fail(f"port {p} is in use: stop whatever holds it (not killed by this test)")
    tmp = tmp_path_factory.mktemp("openplay-e2e")
    env = {k: v for k, v in os.environ.items() if k != "ANTHROPIC_API_KEY"}  # Say it falls back
    env["PYTHONUNBUFFERED"] = "1"
    py = sys.executable
    procs: list[subprocess.Popen] = []
    logs = []
    try:
        dlog = open(tmp / "daemon.log", "w")
        logs.append(dlog)
        procs.append(subprocess.Popen(
            [py, "-m", "armctl.daemon", "--backend", "mock", "--auto-approve", "--port", str(DAEMON_PORT),
             "--vel-cap", "0.6", "--acc-cap", "2.0", "--log", ""],
            cwd=REPO, env=env, stdout=dlog, stderr=subprocess.STDOUT))
        _wait_http(f"http://127.0.0.1:{DAEMON_PORT}/status", 20)
        olog = open(tmp / "openplay.log", "w")
        logs.append(olog)
        events = tmp / "events.jsonl"
        procs.append(subprocess.Popen(
            [py, "-m", "openplay.server", "--daemon", f"ws://127.0.0.1:{DAEMON_PORT}/armctl",
             "--phone-host", "127.0.0.1", "--phone-port", str(PHONE_PORT), "--local-port", str(LOCAL_PORT),
             "--key", KEY, "--turn", str(TURN_S), "--auto-play", "on", "--host-ip", "127.0.0.1",
             "--events-log", str(events), "--trace-log", str(tmp / "trace.jsonl")],
            cwd=REPO, env=env, stdout=olog, stderr=subprocess.STDOUT))
        _wait_http(f"http://127.0.0.1:{LOCAL_PORT}/api/join", 20)
        _wait_http(f"https://127.0.0.1:{PHONE_PORT}/?k={KEY}", 20, SSL_CTX)
        yield {"tmp": tmp, "events": events, "procs": procs}
    finally:
        for p in procs:  # exactly the processes we started
            if p.poll() is None:
                p.terminate()
        for p in procs:
            try:
                p.wait(8)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait(3)
        for f in logs:
            f.close()
        print("\n--- openplay log (tail) ---\n" + "".join((tmp / "openplay.log").read_text().splitlines(True)[-25:]))


class Client:
    """A WebSocket that records every message; ``wait`` finds the first match after a mark."""

    def __init__(self, ws, label: str):
        self.ws, self.label = ws, label
        self.msgs: list[dict] = []
        self.task = asyncio.create_task(self._rx())

    @classmethod
    async def connect(cls, url: str, label: str, **kw) -> "Client":
        import websockets

        return cls(await websockets.connect(url, max_size=None, **kw), label)

    async def _rx(self) -> None:
        try:
            async for raw in self.ws:
                self.msgs.append(json.loads(raw))
        except Exception:
            pass

    def mark(self) -> int:
        return len(self.msgs)

    async def send(self, op: str, **kw) -> None:
        await self.ws.send(json.dumps({"op": op, **kw}))

    async def wait(self, op: str, pred=lambda m: True, since: int = 0, timeout: float = 10.0) -> dict:
        end = time.monotonic() + timeout
        i = since
        while time.monotonic() < end:
            while i < len(self.msgs):
                m = self.msgs[i]
                i += 1
                if m.get("op") == op and pred(m):
                    return m
            await asyncio.sleep(0.02)
        recent = [m for m in self.msgs[since:] if m.get("op") not in ("arm", "lobby", "turn")][-12:]
        raise AssertionError(f"{self.label}: no {op} matching within {timeout} s; recent: {recent}")

    def last(self, op: str) -> dict | None:
        return next((m for m in reversed(self.msgs) if m.get("op") == op), None)

    async def close(self) -> None:
        await self.ws.close()
        self.task.cancel()


def _get(url: str, ctx=None) -> tuple[int, bytes, dict]:
    try:
        with urllib.request.urlopen(url, timeout=5, context=ctx) as r:
            return r.status, r.read(), dict(r.headers)
    except urllib.error.HTTPError as e:
        return e.code, e.read(), dict(e.headers)


async def _drive(phone: Client, qd: list[float], seconds: float) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        await phone.send("stream", qd=qd)
        await asyncio.sleep(0.05)


def test_open_play_end_to_end(stack):
    asyncio.run(asyncio.wait_for(_scenario(stack), 85))


async def _scenario(stack) -> None:
    import websockets

    t0 = time.monotonic()
    # ---- HTTP: display is open on loopback, phone routes need the room key
    code, body, _ = _get(f"http://127.0.0.1:{LOCAL_PORT}/api/join")
    assert code == 200 and json.loads(body)["url"] == f"https://127.0.0.1:{PHONE_PORT}/?k={KEY}"
    code, body, h = _get(f"http://127.0.0.1:{LOCAL_PORT}/qr.png")
    assert code == 200 and body[:8] == b"\x89PNG\r\n\x1a\n"
    code, body, _ = _get(f"http://127.0.0.1:{LOCAL_PORT}/learn-qr.png")      # the explainer's QR (public URL)
    assert code == 200 and body[:8] == b"\x89PNG\r\n\x1a\n"
    assert _get(f"http://127.0.0.1:{LOCAL_PORT}/")[0] == 200
    assert _get(f"https://127.0.0.1:{PHONE_PORT}/", SSL_CTX)[0] == 403
    code, _, h = _get(f"https://127.0.0.1:{PHONE_PORT}/?k={KEY}", SSL_CTX)
    assert code == 200 and "op_key=" in h.get("set-cookie", "")
    with pytest.raises(Exception):  # no key: the socket is closed before accept
        ws = await websockets.connect(f"wss://127.0.0.1:{PHONE_PORT}/ws", ssl=SSL_CTX)
        await asyncio.wait_for(ws.recv(), 3)

    # ---- display
    disp = await Client.connect(f"ws://127.0.0.1:{LOCAL_PORT}/display/ws", "display")
    hello = await disp.wait("hello", lambda m: m.get("backend") == "mock", timeout=15)
    assert hello["join_url"].endswith(f"?k={KEY}") and hello["turn_s"] == TURN_S
    assert hello["learn_url"].startswith("https://") and "/learn/" in hello["learn_url"]
    assert hello["envelope"] and "ready" in hello["poses"]
    hello = await disp.wait("hello", lambda m: m["lease"] is True and m["daemon"] == "connected", timeout=15)
    if hello["urdf"]:
        assert _get(f"http://127.0.0.1:{LOCAL_PORT}/urdf/i2rt/yam.urdf")[0] == 200
    await disp.wait("config", lambda m: m["auto_play"] is True)
    assert (await disp.wait("arm", lambda m: m["lease"] is True, timeout=10))["estop"] is False

    # ---- two phones
    url = f"wss://127.0.0.1:{PHONE_PORT}/ws?k={KEY}"
    a = await Client.connect(url, "asha", ssl=SSL_CTX)
    b = await Client.connect(url, "ben", ssl=SSL_CTX)
    await a.send("hello", pid="pid-asha", name="Asha")
    w = await a.wait("welcome")
    assert w["name"] == "Asha" and w["turn_s"] == TURN_S and w["speeds"]["joint_rps"] == 0.5
    feats = w["features"]
    moves = {m["name"]: m for m in w["moves"]}
    await a.send("join")
    await a.wait("turn", timeout=15)
    await disp.send("extend", s=300)  # time for everything below; shortened again later
    await disp.wait("event", lambda m: m["kind"] == "operator" and "300 s" in m["text"])
    await b.send("hello", pid="pid-ben", name="")  # the page says hello before a name is typed
    assert (await b.wait("welcome"))["name"].startswith("Player")
    mb = b.mark()
    await b.send("hello", pid="pid-ben", name="Ben")  # then again with the name, right before join
    assert (await b.wait("welcome", since=mb))["name"] == "Ben"
    await b.send("join")
    lob = await b.wait("lobby", lambda m: m["position"] == 1, since=mb)
    mb = b.mark()
    await b.send("hello", pid="pid-ben", name="Ben")  # a repeat hello keeps the place
    await b.wait("welcome", since=mb)
    assert (await b.wait("lobby", since=mb))["position"] == 1
    assert lob["active"]["name"] == "Asha" and lob["eta_s"] and lob["eta_s"] > 200
    dl = await disp.wait("lobby", lambda m: m["active"] and m["queue"] == [{"pid": "pid-ben", "name": "Ben"}])
    assert dl["players"] == 2

    # ---- refusals for the player who is not active
    mb = b.mark()
    await b.send("move", name=next(iter(moves), "nod"))
    assert (await b.wait("refused", since=mb))["reason"] == "not your turn"
    await b.send("stream", qd=[1, 0, 0, 0, 0, 0])
    assert (await b.wait("refused", since=mb + 1))["reason"] == "not your turn"
    await asyncio.sleep(1.1)  # past the stream-refusal throttle
    mb = b.mark()
    await b.send("stream", v={"x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 0})  # finger lifted
    await b.send("stream", qd=[0, 0, 0, 0, 0, 0])
    await asyncio.sleep(0.4)
    assert not [m for m in b.msgs[mb:] if m.get("op") == "refused"], "a zero stream must never be refused"
    await a.send("bogus")
    assert "unknown op" in (await a.wait("refused"))["reason"]

    # ---- live control moves the arm (display sees q change)
    q_start = disp.last("arm")["q"]
    await _drive(a, [1, 0, 0, 0, 0, 0], 1.2)
    await asyncio.sleep(0.8)
    md = disp.mark()
    q1 = (await disp.wait("arm", lambda m: not m["moving"], since=md, timeout=5))["q"]
    assert q1[0] - q_start[0] > 0.2, (q_start, q1)

    # ---- poses: save, move, save, play them as one sequence
    ma = a.mark()
    await a.send("pose.save")
    p = await a.wait("poses", lambda m: len(m["list"]) == 1, since=ma)
    assert p["list"][0]["q"][0] == pytest.approx(q1[0], abs=0.02)
    await _drive(a, [-1, 0, 0, 0.6, 0, 0], 1.0)
    await asyncio.sleep(0.6)
    await a.send("pose.save")
    await a.wait("poses", lambda m: len(m["list"]) == 2, since=ma)
    await a.send("pose.save")
    await a.wait("poses", lambda m: len(m["list"]) == 3, since=ma)
    mp = a.mark()
    await a.send("pose.delete", i=1)  # ids are stable: 0 and 2 stay 0 and 2
    pl = await a.wait("poses", lambda m: len(m["list"]) == 2, since=mp)
    assert [x["i"] for x in pl["list"]] == [0, 2]
    await a.send("stream", qd=[0, 0, 0, 0, 0, 0])  # release while active: forwarded, not refused
    mp = a.mark()
    await a.send("pose.goto", i=0)  # by id: the first saved pose
    await a.wait("status", lambda m: m["text"] == "At pose 1", since=mp, timeout=15)
    q_at = disp.last("arm")["q"]
    assert q_at == pytest.approx(pl["list"][0]["q"], abs=0.03)
    await a.send("pose.goto", i=1)
    assert (await a.wait("refused", since=mp))["reason"] == "no such pose"
    if feats["poses"]:
        ma = a.mark()
        await a.send("seq.play", style="lively")
        s = await a.wait("seq", lambda m: m["stage"] == "playing", since=ma, timeout=10)
        assert s["duration_s"] > 0
        await a.send("ready")
        assert (await a.wait("refused", since=ma))["reason"].startswith("busy:")
        await disp.wait("arm", lambda m: m["busy"] and m["moving"], since=md, timeout=5)
        await a.wait("seq", lambda m: m["stage"] == "done", since=ma, timeout=30)
        await disp.wait("event", lambda m: m["kind"] == "seq" and "Asha" in m["text"], since=md)

    # ---- a library move: plays, refuses others while busy, stop interrupts
    if moves:
        name = "nod" if "nod" in moves else next(iter(moves))
        title = moves[name]["title"]
        ma, md = a.mark(), disp.mark()
        await a.send("move", name=name)
        st = await a.wait("status", lambda m: m["kind"] == "busy" and m["text"].startswith(f"Playing {title}"),
                          since=ma, timeout=10)
        assert "s)" in st["text"]
        await disp.wait("event", lambda m: m["kind"] == "move" and m["text"].startswith(f"Playing {title}"), since=md)
        await disp.wait("arm", lambda m: m["busy"] == title and m["moving"], since=md, timeout=5)
        await a.send("stream", qd=[0.5, 0, 0, 0, 0, 0])
        r = await a.wait("refused", since=ma)
        assert r["reason"].startswith("busy: playing") and "stop" in r["reason"]
        await asyncio.sleep(1.0)
        await a.send("stop")
        await a.wait("status", lambda m: m["text"] == "Stopped", since=ma, timeout=5)
        await disp.wait("arm", lambda m: m["busy"] is None and not m["moving"], since=disp.mark(), timeout=5)

    # ---- Say it without a Claude key: library fallback, says so
    if feats["say"]:
        ma, md = a.mark(), disp.mark()
        await a.send("say", text="wave at the kids")
        await a.wait("say", lambda m: m["stage"] == "thinking" and m["text"] == "wave at the kids", since=ma)
        await disp.wait("event", lambda m: m["kind"] == "say" and m["text"] == 'Asha asked: "wave at the kids"',
                        since=md)
        sp = await a.wait("say", lambda m: m["stage"] == "playing", since=ma, timeout=15)
        assert "no Claude key" in sp["note"] and sp["duration_s"] > 0
        ds = await disp.wait("say", lambda m: m["stage"] == "playing", since=md)
        assert ds["who"] == "Asha" and ds["keyframes"]
        await asyncio.sleep(0.8)
        await a.send("stop")
        await a.wait("say", lambda m: m["stage"] == "error", since=ma, timeout=5)
        await disp.wait("arm", lambda m: m["busy"] is None, since=disp.mark(), timeout=5)

    # ---- auto_play off: the move waits for the operator, approve plays it to the end
    if moves:
        name = "nod" if "nod" in moves else next(iter(moves))
        title = moves[name]["title"]
        md = disp.mark()
        await disp.send("auto_play", on=False)
        await disp.wait("config", lambda m: m["auto_play"] is False, since=md)
        ma = a.mark()
        await a.send("move", name=name)
        pend = await disp.wait("pending", lambda m: m["pending"], since=md, timeout=10)
        pd = pend["pending"]
        assert pd["who"] == "Asha" and pd["title"] == title and pd["keyframes"] and pd["duration_s"] > 0
        await a.wait("status", lambda m: m["text"].startswith("Waiting for the operator"), since=ma)
        # a display that (re)connects now gets the full state right after hello, approval card included
        d2 = await Client.connect(f"ws://127.0.0.1:{LOCAL_PORT}/display/ws", "display-2")
        await d2.wait("pending", lambda m: m["pending"] and m["pending"]["id"] == pd["id"])
        ops = [m["op"] for m in d2.msgs[:3]]
        assert ops == ["hello", "lobby", "pending"], ops
        assert d2.msgs[1]["active"]["pid"] == "pid-asha"
        await d2.close()
        await a.send("ready")
        assert "waiting for the operator" in (await a.wait("refused", since=ma))["reason"]
        await disp.send("approve", id="wrong-id")
        assert "nothing with that id" in (await disp.wait("refused", since=md))["reason"]
        await disp.send("approve", id=pd["id"])
        await disp.wait("pending", lambda m: m["pending"] is None, since=md)
        await a.wait("status", lambda m: m["text"].startswith(f"Playing {title}"), since=ma, timeout=5)
        await a.wait("status", lambda m: m["text"] == f"Done: {title}", since=ma, timeout=30)
        # and a deny
        ma, md = a.mark(), disp.mark()
        await a.send("move", name=name)
        pd = (await disp.wait("pending", lambda m: m["pending"], since=md, timeout=10))["pending"]
        await disp.send("deny", id=pd["id"])
        st = await a.wait("status", lambda m: m["kind"] == "error", since=ma, timeout=5)
        assert "said no" in st["text"]
        await disp.send("auto_play", on=True)
        await disp.wait("config", lambda m: m["auto_play"] is True, since=md)

    # ---- reconnect during the turn: the new socket gets the turn back at once
    await a.close()
    a = await Client.connect(url, "asha-2", ssl=SSL_CTX)
    await a.send("hello", pid="pid-asha", name="Asha")
    await a.wait("welcome")
    assert (await a.wait("turn", timeout=2))["remaining_s"] > 100

    # ---- time-out hands over: shorten Asha's turn, Ben gets the arm after the ready move
    ma, mb, md = a.mark(), b.mark(), disp.mark()
    await disp.send("extend", s=-1000)
    assert (await a.wait("turn_end", since=ma, timeout=5))["reason"] == "time"
    await disp.wait("event", lambda m: m["kind"] == "turn_end" and "Asha" in m["text"], since=md)
    await b.wait("turn", since=mb, timeout=20)
    rd = await disp.wait("arm", since=disp.mark())  # the next turn starts only after the ready move
    assert rd["q"] == pytest.approx(hello["poses"]["ready"], abs=0.03)  # returned to ready between turns
    mb = b.mark()
    await a.send("stream", qd=[1, 0, 0, 0, 0, 0])
    assert (await a.wait("refused", since=ma))["reason"] == "not your turn"

    # ---- Ben's 8 s turn runs out by itself; Asha (re-joined) is next
    ma = a.mark()
    await a.send("join")
    la = await a.wait("lobby", lambda m: m["position"] == 1, since=ma)
    assert la["eta_s"] is not None and la["eta_s"] <= TURN_S
    assert (await b.wait("turn_end", since=mb, timeout=TURN_S + 4))["reason"] == "time"
    await a.wait("turn", since=ma, timeout=20)
    ma = a.mark()

    # ---- operator: pause, skip, resume, kick
    md, mb = disp.mark(), b.mark()
    await b.send("join")
    await disp.send("pause", on=True)
    await disp.wait("config", lambda m: m["paused"] is True, since=md)
    await disp.send("skip")
    assert (await a.wait("turn_end", since=ma, timeout=5))["reason"] == "skipped"
    await asyncio.sleep(2.0)
    lb = await disp.wait("lobby", lambda m: m["active"] is None and m["queue"][:1] == [{"pid": "pid-ben", "name": "Ben"}],
                         since=md, timeout=10)
    assert lb
    assert b.last("lobby")["eta_s"] is None  # paused: unknown
    mb = b.mark()
    await disp.send("pause", on=False)
    await b.wait("turn", since=mb, timeout=15)
    await disp.send("kick", pid="pid-ben")
    assert (await b.wait("turn_end", since=mb, timeout=5))["reason"] == "operator"
    mb = b.mark()
    await b.send("join")
    assert "removed you" in (await b.wait("refused", since=mb))["reason"]
    await disp.send("turn_s", s=30)
    assert (await disp.wait("config", lambda m: m["turn_s"] == 30, since=md))

    for c in (a, b, disp):
        await c.close()

    # ---- the event log has the story and no room key
    log = stack["events"].read_text()
    assert KEY not in log
    kinds = {json.loads(line)["kind"] for line in log.splitlines()}
    assert {"join", "turn", "turn_end", "stop", "operator"} <= kinds
    print(f"\nscenario took {time.monotonic() - t0:.1f} s")


def test_home_op(stack):
    """`home` behaves like `ready`: only the active player, one action at a time, then folded."""
    asyncio.run(asyncio.wait_for(_home_scenario(), 90))


async def _home_scenario() -> None:
    disp = await Client.connect(f"ws://127.0.0.1:{LOCAL_PORT}/display/ws", "display-home")
    hello = await disp.wait("hello", lambda m: m["lease"] is True, timeout=15)
    url = f"wss://127.0.0.1:{PHONE_PORT}/ws?k={KEY}"
    c = await Client.connect(url, "cara", ssl=SSL_CTX)
    d = await Client.connect(url, "dev", ssl=SSL_CTX)
    await c.send("hello", pid="pid-cara", name="Cara")
    await c.wait("welcome")
    await d.send("hello", pid="pid-dev", name="Dev")
    await d.wait("welcome")
    await c.send("join")
    await c.wait("turn", timeout=25)
    await disp.send("extend", s=300)
    await d.send("join")
    await d.wait("lobby", lambda m: m["position"] == 1)

    # not your turn
    md = d.mark()
    await d.send("home")
    assert (await d.wait("refused", since=md))["reason"] == "not your turn"

    # accepted on your turn; a second one while it runs is refused as busy
    mc, mdisp = c.mark(), disp.mark()
    await c.send("home")
    await c.send("home")
    assert (await c.wait("refused", since=mc))["reason"] == "busy: going home"
    await c.wait("status", lambda m: m["text"] == "Going home" and m["kind"] == "busy", since=mc)
    await disp.wait("arm", lambda m: m["busy"] == "home", since=mdisp, timeout=5)
    await c.send("ready")
    assert (await c.wait("refused", lambda m: m["reason"].startswith("busy"), since=mc + 1))["reason"] == "busy: going home"
    await c.wait("status", lambda m: m["text"] == "Home" and m["kind"] == "done", since=mc, timeout=40)
    folded = (await disp.wait("arm", lambda m: m["busy"] is None, since=disp.mark(), timeout=5))["q"]
    assert folded == pytest.approx([0.0] * 6, abs=0.03)

    # and back to ready, as the phone's get-set step does before driving
    mc = c.mark()
    await c.send("ready")
    await c.wait("status", lambda m: m["text"] == "Ready", since=mc, timeout=40)
    q = (await disp.wait("arm", lambda m: m["busy"] is None, since=disp.mark(), timeout=5))["q"]
    assert q == pytest.approx(hello["poses"]["ready"], abs=0.03)
    for x in (c, d, disp):
        await x.close()


def test_phone_page_routes(stack):
    """One phone page (play.html, with the camera modes) on "/", /classic and /arcade (old links);
    welcome carries the daemon's soft limits (Claw Cam keeps the claw inside j1's range)."""
    for path in ("/", "/classic", "/arcade"):
        code, body, _ = _get(f"https://127.0.0.1:{PHONE_PORT}{path}?k={KEY}", SSL_CTX)
        assert code == 200 and b"/static/play.js" in body, path
    assert _get(f"https://127.0.0.1:{PHONE_PORT}/arcade", SSL_CTX)[0] == 403   # still needs the room key
    for f in ("camgames.js", "camgames.css", "kin.js"):
        assert _get(f"https://127.0.0.1:{PHONE_PORT}/static/{f}?k={KEY}", SSL_CTX)[0] == 200
    asyncio.run(asyncio.wait_for(_arcade_scenario(), 30))


async def _arcade_scenario() -> None:
    disp = await Client.connect(f"ws://127.0.0.1:{LOCAL_PORT}/display/ws", "display-arcade")
    await disp.wait("hello", lambda m: m["lease"] is True, timeout=15)
    e = await Client.connect(f"wss://127.0.0.1:{PHONE_PORT}/ws?k={KEY}", "esha", ssl=SSL_CTX)
    await e.send("hello", pid="pid-esha", name="Esha", ui="classic")
    w = await e.wait("welcome")
    j1 = w["limits"]["j1"]
    assert len(j1) == 2 and j1[0] < 0 < j1[1]
    await e.close()
    await disp.close()
