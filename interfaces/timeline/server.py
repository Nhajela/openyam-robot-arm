"""Timeline editor: record, edit and check choreography against what the arm can do.

    .venv/bin/python -m interfaces.timeline.server            # http://127.0.0.1:8097
    .venv/bin/python -m interfaces.timeline.server --daemon ws://127.0.0.1:8091/armctl

The page shows one lane per joint (plus the gripper) with the playback curve, the keyframes
(drag them), and for each keyframe the band that joint can reach from the previous keyframe
in the time given. Segments the arm can't do turn red with the coach's reason. "Stretch"
slows them down, "Shrink" keeps the timing and makes the moves smaller. "Record" samples a
running armctl daemon (observer only, it never takes the lease) while you drive the arm with
any other interface, and turns that into keyframes. "Send to arm" validates and asks the
daemon to play: the operator still approves the program and presses Play in the console.

Loopback only, like the console.
"""
from __future__ import annotations

import argparse
import asyncio
import time
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse

from armctl import envelope as envm
from armctl import playable as pl

from . import coach, record

STATIC = Path(__file__).resolve().parent / "static"


def build_app(daemon_url: str) -> FastAPI:
    app = FastAPI(title="openyam timeline")
    rec_state: dict[str, Any] = {"task": None, "samples": [], "t0": None, "error": None}

    @app.get("/")
    def index():
        return FileResponse(STATIC / "timeline.html")

    @app.get("/api/info")
    def info():
        return {"envelope": envm.DEFAULT.describe(), "caps_presets": pl.CAP_PRESETS, "daemon": daemon_url,
                "playables": pl.list_playables()}

    @app.get("/api/playable/{name}")
    def get_playable(name: str):
        try:
            return pl.load(name)
        except Exception as e:
            raise HTTPException(404, str(e))

    @app.post("/api/check")
    def check(body: dict):
        return _explain(body["playable"])

    @app.post("/api/fit")
    def fit(body: dict):
        p = coach.fit(body["playable"], body.get("how", "stretch"))
        return {"playable": p, "report": _explain(p)}

    @app.post("/api/save")
    def save(body: dict):
        p = body["playable"]
        r = _explain(p)
        if not r["ok"]:
            raise HTTPException(400, "not saved, it isn't playable yet: " + "; ".join(r["summary"][1:4]))
        p["name"] = pl.slugify(p.get("name") or "untitled")
        return {"path": str(pl.save(p)), "name": p["name"]}

    @app.post("/api/from-recording")
    def from_recording(body: dict):
        p, inf = record.to_playable(body["recording"], body.get("name", "recorded-move"),
                                    float(body.get("tol", 0.04)), body.get("caps", "default"),
                                    body.get("fit", "stretch"))
        return {"playable": p, "info": inf, "report": _explain(p)}

    @app.post("/api/record/start")
    async def record_start():
        if rec_state["task"] and not rec_state["task"].done():
            raise HTTPException(409, "already recording")
        rec_state.update(samples=[], t0=time.monotonic(), error=None)

        async def run():
            from ..client import ArmClient
            try:
                async with ArmClient(daemon_url, program="recorder", name="timeline recorder") as arm:
                    await arm.wait_state()
                    while True:
                        rec_state["samples"].append({"t": round(time.monotonic() - rec_state["t0"], 3),
                                                     "q": [round(float(x), 4) for x in arm.q],
                                                     "grip": arm.state.get("grip", 0.0)})
                        await asyncio.sleep(0.05)
            except asyncio.CancelledError:
                pass
            except Exception as e:
                rec_state["error"] = f"{type(e).__name__}: {e}"

        rec_state["task"] = asyncio.create_task(run())
        await asyncio.sleep(0.3)
        if rec_state["error"]:
            raise HTTPException(502, f"can't reach the armctl daemon at {daemon_url}: {rec_state['error']}")
        return {"recording": True}

    @app.get("/api/record/status")
    def record_status():
        s = rec_state["samples"]
        return {"recording": bool(rec_state["task"] and not rec_state["task"].done()), "n": len(s),
                "seconds": s[-1]["t"] if s else 0, "last_q": s[-1]["q"] if s else None, "error": rec_state["error"]}

    @app.post("/api/record/stop")
    async def record_stop(body: dict | None = None):
        body = body or {}
        t = rec_state["task"]
        if t:
            t.cancel()
            await asyncio.sleep(0)
        rec = {"format": "openyam-recording", "version": "1.0", "source": "timeline", "rate_hz": 20,
               "samples": rec_state["samples"]}
        if len(rec["samples"]) < 3:
            raise HTTPException(400, "recording is empty: is anything driving the arm?")
        return from_recording({"recording": rec, **body})

    @app.post("/api/send")
    async def send(body: dict):
        """Validate and ask the daemon to play. The operator approves and presses Play."""
        from ..client import ArmClient, Refused
        p = body["playable"]
        try:
            async with ArmClient(daemon_url, program="timeline", name="timeline editor") as arm:
                await arm.acquire(timeout=60)
                done = await arm.play(p, auto_confirm=bool(body.get("auto_confirm")) and arm.backend == "mock")
                return {"ok": True, "done": done}
        except (Refused, TimeoutError) as e:
            raise HTTPException(409, str(e))
        except OSError as e:
            raise HTTPException(502, f"can't reach the armctl daemon at {daemon_url}: {e}")

    return app


def _explain(p: dict) -> dict:
    try:
        return coach.explain(p)
    except Exception as e:
        raise HTTPException(400, f"{type(e).__name__}: {e}")


def main() -> None:
    import uvicorn

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8097)
    ap.add_argument("--daemon", default="ws://127.0.0.1:8091/armctl")
    a = ap.parse_args()
    print(f"timeline editor on http://127.0.0.1:{a.port}  (daemon {a.daemon})")
    uvicorn.run(build_app(a.daemon), host="127.0.0.1", port=a.port, log_level="warning")


if __name__ == "__main__":
    main()
