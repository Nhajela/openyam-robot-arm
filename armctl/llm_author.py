"""English -> playable: the LLM program from ARMCTL-SPEC.md ("English -> playable").

The model only AUTHORS a playable (JSON). It never streams motion. Every playable
goes through armctl.playable.validate; violations are fed back for up to N repair
rounds; if only speed violations remain, retime() is offered (applied with --retime).

Backends (picked automatically):
  sdk  Anthropic Python SDK, if ANTHROPIC_API_KEY is set and `anthropic` is installed
       (structured output via output_config.format json_schema).
  cli  the logged-in Claude Code CLI: `claude -p ... --output-format json --json-schema ...`
       with all tools disabled, no MCP servers, no session persistence.

Usage:
  .venv/bin/python -m armctl.llm_author "do a happy little dance" [--save] [--retime]
       [--model claude-sonnet-5-5] [--backend sdk|cli] [--rounds 3] [--json]
  .venv/bin/python -m armctl.llm_author "a happy wave" --visual --passes 3 --model claude-fable-5-1 \
       --out playables/happy-wave.json --sheets-dir experiments/<exp>/sheets/happy-wave [--camera auto]
       (visual: armctl.visual_author; render sheets of each candidate go back to the model for review)

Prompts and responses are appended to
experiments/2026-10-16-linux-real-arm/logs/llm-author.log (no secrets are logged).
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

from armctl import playable as P

DEFAULT_MODEL = os.environ.get("ARMCTL_LLM_MODEL", "claude-sonnet-5-5")
LOG_PATH = P.REPO / "experiments/2026-10-16-linux-real-arm/logs/llm-author.log"
CLI_TIMEOUT_S = 300

# JSON schema for structured output (kept within what structured outputs accept).
PLAYABLE_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "name": {"type": "string", "description": "lowercase slug, letters digits and '-'"},
        "space": {"type": "string", "enum": ["joint"]},
        "start": {"type": "string", "enum": ["ready"]},
        "keyframes": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "t": {"type": "number"},
                    "q": {"type": "array", "items": {"type": "number"}},
                    "grip": {"type": "number"},
                    "ease": {"type": "string", "enum": ["cubic", "linear", "hold"]},
                },
                "required": ["t", "q", "grip", "ease"],
                "additionalProperties": False,
            },
        },
        "loop": {"type": "integer"},
        "speed": {"type": "number"},
        "notes": {"type": "string", "description": "one sentence on what the motion does"},
    },
    "required": ["name", "space", "start", "keyframes", "loop", "speed", "notes"],
    "additionalProperties": False,
}

JOINT_DESCRIPTIONS = [
    ("j1", "base yaw: turns the whole arm left/right about the vertical axis (+ = counterclockwise seen from above)"),
    ("j2", "shoulder pitch: lifts the upper arm up from the folded rest (larger = raised further)"),
    ("j3", "elbow: unfolds the forearm (larger = more extended; 0 is fully folded)"),
    ("j4", "wrist pitch: tilts the hand up/down (a 'nod')"),
    ("j5", "wrist yaw/roll: turns the hand sideways"),
    ("j6", "wrist roll: spins the gripper about its own axis (a 'twist' or 'wave' of the hand)"),
]


def _example(name: str) -> str:
    try:
        p = P.load(name)
    except FileNotFoundError:
        return ""
    keep = {k: p[k] for k in ("name", "space", "start", "keyframes", "loop", "speed")}
    return json.dumps(keep, separators=(",", ":"))


def _min_T_examples(env) -> str:
    import math
    v, a = min(env.vel_cap), min(env.acc_cap)
    out = []
    for d in (0.1, 0.2, 0.3, 0.5):
        out.append(f"d={d} rad needs T >= {max(1.5 * d / v, math.sqrt(6 * d / a)):.2f} s")
    return "; ".join(out)


def system_prompt(*, modest: bool = True, examples: tuple[str, ...] = ("wave", "nod"),
                  flow: bool = False) -> str:
    """The envelope + schema prompt. ``modest`` keeps the original "stay small" advice (the text
    path); the visual author turns it off and adds armctl/choreo_guidelines.md instead."""
    env = P.envelope()
    ease_kinds = '"cubic"|"linear"|"hold"' + ('|"flow"' if flow else '')
    flow_rule = ('\n  - "flow" passes smoothly THROUGH its keyframe (non-zero velocity there) and only stops where a '
                 'joint reverses or at a cubic/hold neighbour: use it for arcs and continuous phrases.' if flow else '')
    style = ("  - Keep motions modest: stay at least 0.2 rad inside every soft limit; j2 in [0.4, 1.2] and\n"
             "    j3 in [0.6, 1.6] are comfortable. Expressive motion comes from j1, j4, j5, j6 and the gripper."
             if modest else
             "  - Stay at least 0.2 rad inside every soft limit; otherwise use the range (see the guidelines).")
    ex = "\n".join(f"{n}: {_example(n)}" for n in examples if _example(n))
    lims = "\n".join(
        f"  {n}: soft limits [{lo:.3f}, {hi:.3f}] rad; {desc}"
        for (n, desc), (lo, hi) in zip(JOINT_DESCRIPTIONS, env.soft))
    poses = "\n".join(f"  {k}: {[round(x, 3) for x in v]}" for k, v in env.poses.items())
    return f"""You author motion "playables" for a 6-joint desktop robot arm (OpenYAM) with a gripper.
You output ONE playable as JSON and nothing else. A program validates it before a human previews it
and clicks Play; anything outside the limits below is rejected, so stay well inside them.

Joints (q is a list of 6 joint angles in radians, in this order):
{lims}
Gripper: grip 0 = closed .. 1 = open.

Caps (checked on the path interpolated at 100 Hz, after applying speed):
  joint velocity <= {min(env.vel_cap)} rad/s, joint acceleration <= {min(env.acc_cap)} rad/s^2, gripper <= {env.grip_vel_cap}/s.
  With ease "cubic" a segment of size d radians lasting T seconds has peak velocity 1.5*d/T and
  peak acceleration 6*d/T^2. So T >= 1.5*d/{min(env.vel_cap)} and T >= sqrt(6*d/{min(env.acc_cap)}). Add ~10% margin.
  Examples (minimum cubic segment durations): {_min_T_examples(env)}.
Named poses:
{poses}
  "home" is folded on the hard stops (j2=j3=0) and is NOT inside the soft box. Always use start "ready".

Schema:
  {{"name": slug, "space": "joint", "start": "ready",
    "keyframes": [{{"t": seconds, "q": [6 rad], "grip": 0..1, "ease": {ease_kinds}}}, ...],
    "loop": 1..10, "speed": 0.1..2 (playback time = t / speed), "notes": one sentence}}
Rules:
  - First keyframe at t=0 and equal to the ready pose {[round(x, 3) for x in env.poses['ready']]}; end back at ready.
  - t strictly increasing. Total playback (t_last / speed * loop) <= 60 s; aim for 5-15 s.
  - ease applies to the segment ending at that keyframe. Prefer "cubic" (smooth stop at each keyframe).
    "linear" starts/stops with a velocity jump and will fail the acceleration cap. "hold" = pause; its q
    must equal the previous keyframe's q.{flow_rule}
  - If loop > 1 the last keyframe must equal the first.
{style}

Example playables (valid):
{ex}
"""


# --- backends -----------------------------------------------------------------------------

class LLMError(RuntimeError):
    pass


def _log(entry: dict[str, Any]) -> None:
    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
    entry = {"ts": P.now_iso(), **entry}
    with LOG_PATH.open("a") as f:
        f.write(json.dumps(entry, ensure_ascii=False) + "\n")


def _extract_json(text: str) -> dict[str, Any]:
    text = text.strip()
    m = re.search(r"```(?:json)?\s*(\{.*\})\s*```", text, re.S)
    if m:
        text = m.group(1)
    start, end = text.find("{"), text.rfind("}")
    if start < 0 or end < 0:
        raise LLMError(f"no JSON object in model output: {text[:200]!r}")
    return json.loads(text[start:end + 1])


def pick_backend(requested: str | None = None) -> str:
    if requested:
        return requested
    if os.environ.get("ANTHROPIC_API_KEY"):
        try:
            import anthropic  # noqa: F401
            return "sdk"
        except ImportError:
            pass
    if shutil.which("claude"):
        return "cli"
    raise LLMError("no backend: set ANTHROPIC_API_KEY (and pip install anthropic) or install the claude CLI")


def call_sdk(system: str, user: str, model: str) -> tuple[dict[str, Any], dict[str, Any]]:
    import anthropic

    client = anthropic.Anthropic()
    resp = client.messages.create(
        model=model,
        max_tokens=16000,
        system=system,
        messages=[{"role": "user", "content": user}],
        output_config={"format": {"type": "json_schema", "schema": PLAYABLE_SCHEMA}},
    )
    if resp.stop_reason == "refusal":
        raise LLMError(f"model refused: {resp.stop_details}")
    text = next(b.text for b in resp.content if b.type == "text")
    meta = {"usage": resp.usage.model_dump() if hasattr(resp.usage, "model_dump") else str(resp.usage),
            "stop_reason": resp.stop_reason}
    return json.loads(text), meta


def call_cli(system: str, user: str, model: str) -> tuple[dict[str, Any], dict[str, Any]]:
    cmd = ["claude", "-p", user, "--output-format", "json", "--model", model,
           "--system-prompt", system, "--json-schema", json.dumps(PLAYABLE_SCHEMA),
           "--tools", "", "--strict-mcp-config", "--no-session-persistence"]
    # neutral cwd so no project CLAUDE.md / settings leak into the prompt
    with tempfile.TemporaryDirectory(prefix="armctl-llm-") as cwd:
        try:
            out = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, timeout=CLI_TIMEOUT_S)
        except subprocess.TimeoutExpired as e:
            raise LLMError(f"claude CLI timed out after {CLI_TIMEOUT_S}s") from e
    if out.returncode != 0:
        raise LLMError(f"claude CLI exit {out.returncode}: {(out.stderr or out.stdout)[:500]}")
    try:
        res = json.loads(out.stdout)
    except json.JSONDecodeError as e:
        raise LLMError(f"claude CLI printed non-JSON: {out.stdout[:300]!r}") from e
    if res.get("is_error"):
        raise LLMError(f"claude CLI error: {res.get('result') or res.get('subtype')}")
    meta = {k: res.get(k) for k in ("duration_ms", "total_cost_usd", "num_turns", "stop_reason")}
    meta["model_usage"] = list((res.get("modelUsage") or {}).keys())
    if isinstance(res.get("structured_output"), dict):
        return res["structured_output"], meta
    return _extract_json(res.get("result") or ""), meta


def call_llm(backend: str, system: str, user: str, model: str, tag: str) -> dict[str, Any]:
    t0 = time.monotonic()
    err = None
    data: dict[str, Any] | None = None
    meta: dict[str, Any] = {}
    try:
        fn = call_sdk if backend == "sdk" else call_cli
        data, meta = fn(system, user, model)
    except Exception as e:  # logged, then re-raised
        err = f"{type(e).__name__}: {e}"
    dt = round(time.monotonic() - t0, 2)
    _log({"tag": tag, "backend": backend, "model": model, "latency_s": dt, "user_prompt": user,
          "response": data, "meta": meta, "error": err})
    if err:
        raise LLMError(err)
    assert data is not None
    data["_latency_s"] = dt
    return data


# --- authoring loop ---------------------------------------------------------------------------

def summarize_violations(violations: list[dict[str, Any]], max_items: int = 25) -> str:
    """Group repeated violations (same kind/joint) and keep the worst instances, for the model."""
    groups: dict[tuple, list[dict[str, Any]]] = {}
    for v in violations:
        groups.setdefault((v["kind"], v.get("joint")), []).append(v)
    lines = []
    for (kind, joint), vs in groups.items():
        vs_sorted = sorted(vs, key=lambda v: -abs(v["value"]) if isinstance(v.get("value"), (int, float)) else 0)
        head = f"{kind}{' ' + joint if joint else ''}: {len(vs)} occurrence(s)"
        lines.append(head)
        for v in vs_sorted[:4]:
            lines.append(f"  - {v['msg']}")
    return "\n".join(lines[:max_items])


def _normalize(raw: dict[str, Any], request: str, model: str) -> dict[str, Any]:
    p = {k: v for k, v in raw.items() if not k.startswith("_")}
    name = p.get("name")
    if not isinstance(name, str) or not P.NAME_RE.match(name):
        p["name"] = P.slugify(name if isinstance(name, str) else request)
    p["author"] = f"llm:{model}"
    p["created"] = P.now_iso()
    p["prompt"] = request
    p.setdefault("space", "joint")
    p.setdefault("start", "ready")
    p.setdefault("loop", 1)
    p.setdefault("speed", 1.0)
    order = ["name", "author", "created", "prompt", "notes", "space", "start", "keyframes", "loop", "speed"]
    return {k: p[k] for k in order if k in p} | {k: v for k, v in p.items() if k not in order}


def author(request: str, *, model: str = DEFAULT_MODEL, backend: str | None = None,
           rounds: int = 3, allow_retime: bool = False, echo=print) -> dict[str, Any]:
    """English -> validated playable. Returns {ok, playable, report, attempts, latency_s, ...}."""
    backend = pick_backend(backend)
    system = system_prompt()
    _log({"tag": "session", "backend": backend, "model": model, "request": request,
          "system_prompt": system, "envelope": P.envelope().source})
    attempts = []
    total = 0.0
    user = f'Write a playable for: "{request}"'
    pl: dict[str, Any] | None = None
    r: dict[str, Any] | None = None
    for i in range(rounds + 1):
        tag = "initial" if i == 0 else f"repair-{i}"
        try:
            raw = call_llm(backend, system, user, model, tag)
        except LLMError as e:
            attempts.append({"round": tag, "error": str(e)})
            echo(f"[{tag}] LLM call failed: {e}")
            break
        total += raw["_latency_s"]
        try:
            pl = _normalize(raw, request, model)
            r = P.validate(pl)
        except Exception as e:  # malformed beyond the validator's schema checks
            r = {"ok": False, "violations": [{"kind": "schema", "msg": f"{type(e).__name__}: {e}",
                                              "t": None, "joint": None, "value": None, "limit": None}],
                 "duration": None}
        attempts.append({"round": tag, "latency_s": raw["_latency_s"], "ok": r["ok"],
                         "n_violations": len(r["violations"]),
                         "kinds": sorted({v["kind"] for v in r["violations"]})})
        echo(f"[{tag}] {raw['_latency_s']}s  "
             + ("OK" if r["ok"] else f"{len(r['violations'])} violation(s)"))
        _log({"tag": f"{tag}-validation", "ok": r["ok"], "violations": r["violations"][:50]})
        if r["ok"] or i == rounds:
            break
        user = (f'Write a playable for: "{request}"\n\nYour previous attempt:\n'
                f"{json.dumps({k: pl.get(k) for k in ('name', 'space', 'start', 'keyframes', 'loop', 'speed')}, separators=(',', ':'))}\n\n"
                f"was REJECTED by the validator:\n{summarize_violations(r['violations'])}\n\n"
                "Fix every violation (move keyframes inside the limits, lengthen segments that are too fast: "
                "T >= 1.5*d/vel_cap and T >= sqrt(6*d/acc_cap)) and return the corrected playable.")

    out: dict[str, Any] = {"ok": bool(r and r["ok"]), "playable": pl, "report": r, "attempts": attempts,
                           "latency_s": round(total, 2), "backend": backend, "model": model,
                           "retimed": False}
    if r and not r["ok"] and pl is not None:
        kinds = {v["kind"] for v in r["violations"]}
        if kinds <= P.SPEED_KINDS:
            out["retime_available"] = True
            if allow_retime:
                try:
                    fixed = P.retime(pl)
                    out.update(ok=True, playable=fixed, report=P.validate(fixed), retimed=True)
                    _log({"tag": "retime", "ok": True, "duration": out["report"]["duration"]})
                except ValueError as e:
                    out["retime_error"] = str(e)
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m armctl.llm_author", description=__doc__.split("\n")[0])
    ap.add_argument("request", help='English, e.g. "do a happy little dance"')
    ap.add_argument("--save", action="store_true", help="save to playables/<name>.json if valid")
    ap.add_argument("--retime", action="store_true", help="apply retime() if only speed violations remain")
    ap.add_argument("--model", default=None,
                    help=f"default {DEFAULT_MODEL} (text) / $ARMCTL_VISUAL_MODEL or claude-fable-5-1 (--visual)")
    ap.add_argument("--backend", choices=["sdk", "cli"])
    ap.add_argument("--rounds", type=int, default=3, help="repair rounds after the first attempt")
    ap.add_argument("--json", action="store_true", help="print the playable JSON")
    v = ap.add_argument_group("visual (armctl.visual_author: render sheets + review passes, SDK only)")
    v.add_argument("--visual", action="store_true", help="draft, then review render sheets of the motion")
    v.add_argument("--passes", type=int, default=2, help="visual review passes after the draft (0-3)")
    v.add_argument("--camera", choices=["auto", "always", "never"], default="auto",
                   help="also send a live wrist-camera snapshot (auto: only if the request refers to the scene)")
    v.add_argument("--sheets-dir", help="save sheets, per-pass JSON and CHANGELOG.md here")
    v.add_argument("--out", help="write the final playable JSON to this path (only if valid)")
    v.add_argument("--audience", help="audience camera preset (armctl/audience.json) or 'x,y,z' eye in m")
    v.add_argument("--effort", default=None, help="output_config.effort for the visual model (default high)")
    v.add_argument("--caps", choices=sorted(P.CAP_PRESETS), help="validator cap preset written into the playable")
    a = ap.parse_args(argv)

    if a.visual:
        return _main_visual(a)
    a.model = a.model or DEFAULT_MODEL
    res = author(a.request, model=a.model, backend=a.backend, rounds=a.rounds, allow_retime=a.retime)
    print(f"\nbackend={res['backend']} model={res['model']} total LLM latency {res['latency_s']}s "
          f"rounds={len(res['attempts'])}")
    if res["report"]:
        print(P.report(res["report"]))
    if res.get("retime_available") and not res["retimed"]:
        print("Only speed violations remain: rerun with --retime to slow it down just enough.")
    if res["retimed"]:
        print("retime() applied.")
    if a.json and res["playable"]:
        print(json.dumps(res["playable"], indent=2))
    if a.save and res["ok"]:
        pl = res["playable"]
        base, i = pl["name"], 2
        while (P.PLAYABLES_DIR / f"{pl['name']}.json").exists():  # never overwrite
            pl["name"] = f"{base}-{i}"
            i += 1
        path = P.save(pl)
        print(f"saved {path.relative_to(P.REPO)}")
    elif a.save:
        print("not saved (not valid)")
    return 0 if res["ok"] else 1


def _main_visual(a) -> int:
    from armctl import visual_author as V

    if a.backend == "cli":
        print("--visual needs the SDK backend (images): set ANTHROPIC_API_KEY")
        return 2
    res = V.author_visual(a.request, model=a.model or V.DEFAULT_MODEL, passes=a.passes, camera=a.camera,
                          sheets_dir=a.sheets_dir, audience=a.audience, effort=a.effort or V.DEFAULT_EFFORT,
                          caps=a.caps, progress=lambda s: print(f"... {s}"))
    print(f"\nmodel={res['model']} {len(res['passes'])} call(s) {res['latency_s']} s  stop: {res['stop_reason']}")
    if res["report"]:
        print(P.report(res["report"]))
    if not res["ok"]:
        print("no valid playable:", res.get("error"))
        return 1
    pl = res["playable"]
    if a.json:
        print(json.dumps(pl, indent=2))
    if a.out:
        out = Path(a.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        if out.parent.resolve() == P.PLAYABLES_DIR.resolve():
            pl["name"] = P.slugify(out.stem)
        out.write_text(json.dumps(pl, indent=2) + "\n")
        print(f"wrote {out}")
    if a.save:
        base, i = pl["name"], 2
        while (P.PLAYABLES_DIR / f"{pl['name']}.json").exists():  # never overwrite
            pl["name"] = f"{base}-{i}"
            i += 1
        print(f"saved {P.save(pl).relative_to(P.REPO)}")
    if a.sheets_dir:
        print(f"sheets + CHANGELOG.md in {a.sheets_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
