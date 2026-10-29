"""armctl.visual_author: English -> playable, with Claude looking at render sheets of its own motion.

Pass 1 (draft): Claude (default Claude Fable 5.1) authors a playable from the request, the safety
envelope and the choreography guidelines (armctl/choreo_guidelines.md). If the request refers to the
real scene ("wave at the person in front of you", "point at the cup"), a router decides to also send
a live wrist-camera snapshot (armctl.choreo_scene; override with camera="always"/"never").

Passes 2..N (review): every candidate is validated (armctl.playable.validate + the table check in
interfaces.timeline.coach), rendered as contact sheets (armctl.choreo_render: audience view first,
keyframes and in-betweens, timeline with limit bands and speed headroom) and sent back with the
validator report and the motion metrics. Claude critiques it as a spectator would, scores it on a
rubric, and returns a revised playable plus a changelog. We stop early when the candidate is valid
and Claude says it is satisfied.

Safety: nothing here moves the arm. The validator is never relaxed; a candidate that fails is fed
back with its violations, and only validated candidates are ever returned (the latest valid one).
If only speed violations remain, P.retime() (which only slows segments down) may fix them.

    res = author_visual("a happy wave", passes=2, sheets_dir="out/")
    res["ok"], res["playable"], res["passes"], res["route"]

Each request costs 1 + passes model calls (plus at most one tiny router call) and ~3-6 s of
rendering per pass; see the experiment log for measured numbers.
"""
from __future__ import annotations

import base64
import copy
import json
import os
import time
from pathlib import Path
from typing import Any, Callable

from armctl import choreo_render as R
from armctl import choreo_scene as S
from armctl import llm_author as L
from armctl import playable as P

DEFAULT_MODEL = os.environ.get("ARMCTL_VISUAL_MODEL", "claude-fable-5-1")
ROUTER_MODEL = os.environ.get("ARMCTL_ROUTER_MODEL", "claude-haiku-5-5")
DEFAULT_EFFORT = os.environ.get("ARMCTL_VISUAL_EFFORT", "high")
GUIDELINES = Path(__file__).with_name("choreo_guidelines.md")
FALLBACK_BETA = "server-side-fallback-2026-07-01"
# models that accept fallbacks: "default" (Claude API)
FALLBACK_MODELS = ("claude-fable-5-1", "claude-opus-5-5", "claude-opus-5", "claude-sonnet-5-5")
MAX_TOKENS = 32000
REVIEW_SHEETS = 4  # pose sheets per review (plus the overview); longer moves are thinned
EXAMPLES = ("curious-sniff", "sleepy-stretch")
SCORE_KEYS = ("readability", "silhouette", "energy_contrast", "timing", "safety_margin")

_PL = copy.deepcopy(L.PLAYABLE_SCHEMA)
_PL["properties"]["keyframes"]["items"]["properties"]["ease"]["enum"] = ["cubic", "flow", "linear", "hold"]
VISUAL_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "critique": {"type": "string", "description": "what a spectator sees in the current candidate; empty on the draft"},
        "scores": {
            "type": "object",
            "properties": {k: {"type": "integer"} for k in SCORE_KEYS},
            "required": list(SCORE_KEYS),
            "additionalProperties": False,
        },
        "changelog": {"type": "array", "items": {"type": "string"}},
        "satisfied": {"type": "boolean"},
        "playable": _PL,
    },
    "required": ["critique", "scores", "changelog", "satisfied", "playable"],
    "additionalProperties": False,
}


class VisualError(RuntimeError):
    pass


# --- prompt -----------------------------------------------------------------------------------

def system_prompt(audience: R.View | None = None) -> str:
    aud = audience or R.load_audience()
    try:
        guide = GUIDELINES.read_text()
    except OSError:
        guide = "(guidelines file missing)"
    return (L.system_prompt(modest=False, examples=EXAMPLES, flow=True) + f"""

# Choreography guidelines
{guide}

# How this session works
You are the choreographer AND your own critic. First you write a draft. Then, up to a few times, you get
render sheets of your candidate: an OVERVIEW (onion skin of all keyframes from the audience, the hand's
path, every joint against its soft limits, and speed headroom) and POSE sheets (each keyframe K and the
in-betweens ½ ¼ ¾ drawn from the audience's position: "{aud.label}", with small down-the-table and top
views; a grey ghost is the previous pose). You also get the validator's report and motion metrics.

Critique like a spectator standing there: is the intent obvious from the front within two seconds?
Does the silhouette change clearly between key poses? Is it legible at a distance (the hand should
travel well over 3 cm on the audience's view)? Is there a clear beginning, peak and ending? Is there
anticipation, contrast of big/small and fast/slow, accents and holds on the beats? Any awkward wrist
flips, near-limit poses, or the hand near the table?

Score the candidate you were shown, 1 (poor) to 5 (excellent), on: readability (intent obvious from
the audience), silhouette (clear, distinct key poses), energy_contrast (big vs small, fast vs slow,
stillness), timing (anticipation, accents, holds, varied tempo), safety_margin (inside limits, away from
the table, speed headroom). On the draft (nothing shown yet), score your own intent honestly.

Then return a revised playable (or the same one) and a short changelog (one line per change).
Set satisfied=true only when the candidate you were shown is valid, every score is at least 4, and
you would not change anything important; then return that candidate unchanged. A candidate that the
validator REJECTED can never be satisfying: fix every violation.
Reply with the JSON object only.""")


def _img_block(png: bytes, media: str = "image/png") -> dict[str, Any]:
    return {"type": "image", "source": {"type": "base64", "media_type": media,
                                        "data": base64.standard_b64encode(png).decode("ascii")}}


def first_message(request: str, snap: dict[str, Any] | None) -> dict[str, Any]:
    content: list[dict[str, Any]] = []
    if snap is not None:
        content.append(_img_block(snap["jpeg"], "image/jpeg"))
        content.append({"type": "text", "text": snap["text"]})
    content.append({"type": "text", "text":
                    f'Write a playable for: "{request}"\n\nThis is the DRAFT. Plan the beginning, peak and '
                    "ending first, then the keyframes. Make it read clearly from the audience's position."})
    return {"role": "user", "content": content}


def review_message(pass_no: int, n_passes: int, pl: dict, gate: dict, sheets: list[Path],
                   last: bool) -> dict[str, Any]:
    content: list[dict[str, Any]] = [{"type": "text", "text":
        f"REVIEW pass {pass_no} of {n_passes}. Render sheets of your current candidate follow "
        f"(overview first, then pose sheets).{' This is the LAST review: return your best final version.' if last else ''}"}]
    for p in sheets:
        content.append({"type": "text", "text": f"[{p.name}]"})
        content.append(_img_block(p.read_bytes()))
    content.append({"type": "text", "text":
                    "Validator: " + ("VALID (passes armctl's validator and the table check)." if gate["ok"] else
                                     "REJECTED. Fix every violation:\n" + gate["text"])
                    + ("\nNote: speed violations were fixed by retime() (segments slowed down); the sheets show "
                       "the retimed version." if gate.get("retimed") else "")
                    + "\n\nMotion metrics:\n" + R.metrics_text(pl)
                    + "\n\nCandidate JSON:\n" + json.dumps(_core(pl), separators=(",", ":"))})
    return {"role": "user", "content": content}


def _core(pl: dict) -> dict:
    return {k: pl.get(k) for k in ("name", "space", "start", "keyframes", "loop", "speed") if k in pl}


# --- gate -------------------------------------------------------------------------------------

def gate(pl: dict[str, Any], allow_retime: bool = True) -> tuple[dict[str, Any], dict[str, Any]]:
    """(playable, {ok, text, report, clearance, retimed}). Validator + table check, never relaxed."""
    from interfaces.timeline import coach

    def check(p):
        try:
            r = P.validate(p)
        except Exception as e:
            return {"ok": False, "violations": [{"kind": "schema", "msg": f"{type(e).__name__}: {e}"}]}, {}
        try:
            cl = coach.explain(p).get("clearance") or {"ok": True}
        except Exception as e:
            cl = {"ok": False, "msg": f"table check failed: {e}"}
        return r, cl

    r, cl = check(pl)
    retimed = False
    if not r["ok"] and allow_retime and r.get("violations") and \
            {v["kind"] for v in r["violations"]} <= P.SPEED_KINDS:
        try:
            fixed = P.retime(pl)
            r2, cl2 = check(fixed)
            if r2["ok"]:
                pl, r, cl, retimed = fixed, r2, cl2, True
        except ValueError:
            pass
    ok = bool(r.get("ok")) and bool(cl.get("ok", True))
    lines = []
    if r.get("violations"):
        lines.append(L.summarize_violations(r["violations"]))
    if not cl.get("ok", True):
        lines.append("table: " + cl.get("msg", "too close to the table"))
    return pl, {"ok": ok, "text": "\n".join(lines), "report": r, "clearance": cl, "retimed": retimed}


# --- model call -------------------------------------------------------------------------------

def _dump(b: Any) -> dict[str, Any]:
    if isinstance(b, dict):
        return b
    return b.model_dump(mode="json", exclude_none=True)


def echo_content(content: list[Any]) -> list[dict[str, Any]]:
    """Assistant content to append back unchanged; after a mid-output fallback, drop the thinking /
    tool blocks that precede the last ``fallback`` marker (API rule)."""
    blocks = [_dump(b) for b in content]
    last_fb = max((i for i, b in enumerate(blocks) if b.get("type") == "fallback"), default=-1)
    if last_fb < 0:
        return blocks
    drop = {"thinking", "redacted_thinking", "tool_use"}
    return [b for i, b in enumerate(blocks) if not (i < last_fb and b.get("type") in drop)]


def send(client: Any, model: str, system: str, messages: list[dict], effort: str
         ) -> tuple[dict[str, Any], list[dict[str, Any]], dict[str, Any]]:
    """One streamed structured-output request. Returns (parsed JSON, assistant content, meta)."""
    kw: dict[str, Any] = dict(
        model=model, max_tokens=MAX_TOKENS, system=system, messages=messages,
        cache_control={"type": "ephemeral"},
        output_config={"effort": effort, "format": {"type": "json_schema", "schema": VISUAL_SCHEMA}},
    )
    if model in FALLBACK_MODELS:
        kw.update(betas=[FALLBACK_BETA], fallbacks="default")
    t0 = time.monotonic()
    with client.beta.messages.stream(**kw) as stream:
        resp = stream.get_final_message()
    dt = round(time.monotonic() - t0, 2)
    if resp.stop_reason == "refusal":
        raise VisualError(f"model refused: {getattr(resp, 'stop_details', None)}")
    if resp.stop_reason == "max_tokens":
        raise VisualError("model hit max_tokens before finishing the JSON")
    text = next((b.text for b in resp.content if getattr(b, "type", None) == "text"), None)
    if text is None:
        raise VisualError("no text block in the response")
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        data = L._extract_json(text)
    u = getattr(resp, "usage", None)
    usage = u.model_dump(mode="json", exclude_none=True) if hasattr(u, "model_dump") else (u or {})
    meta = {"latency_s": dt, "usage": usage, "model": getattr(resp, "model", model),
            "stop_reason": resp.stop_reason}
    return data, echo_content(resp.content), meta


def make_client():
    try:
        import anthropic
    except ImportError as e:
        raise VisualError("the anthropic SDK is not installed") from e
    return anthropic.Anthropic()


# --- the loop ---------------------------------------------------------------------------------

def _save(d: Path | None, name: str, obj: Any) -> None:
    if d is not None:
        d.mkdir(parents=True, exist_ok=True)
        (d / name).write_text(json.dumps(obj, indent=2, ensure_ascii=False) + "\n")


def author_visual(request: str, *, model: str = DEFAULT_MODEL, passes: int = 2, camera: str = "auto",
                  sheets_dir: str | Path | None = None, audience: str | None = None,
                  effort: str = DEFAULT_EFFORT, client: Any = None, router_model: str = ROUTER_MODEL,
                  deadline_s: float | None = None, caps: str | None = None,
                  progress: Callable[[str], None] | None = None, echo: Callable[..., None] = print,
                  snapshot_fn: Callable[[], dict[str, Any]] | None = None,
                  render_final: bool = True) -> dict[str, Any]:
    """Draft + up to ``passes`` visual review passes. Returns
    {ok, playable, report, passes:[...], route, stop_reason, latency_s, model, sheets_dir}."""
    t_start = time.monotonic()
    passes = max(0, min(int(passes), 3))
    progress = progress or (lambda s: None)
    sd = Path(sheets_dir) if sheets_dir else None
    aud = R.load_audience(audience)
    client = client if client is not None else make_client()

    progress("routing")
    classify = S.make_classifier(client, router_model) if camera == "auto" else None
    route = S.route(request, camera, classify=classify)
    echo(f"[route] camera={'yes' if route['use_camera'] else 'no'} ({route['method']}): {route['reason']}")
    snap = None
    if route["use_camera"]:
        progress("looking (camera)")
        try:
            snap = (snapshot_fn or S.snapshot)()
            if sd is not None:
                sd.mkdir(parents=True, exist_ok=True)
                (sd / "camera.jpg").write_bytes(snap["jpeg"])
        except Exception as e:
            route = {**route, "snapshot_error": str(e)}
            echo(f"[route] snapshot failed, continuing without: {e}")
    L._log({"tag": "visual-session", "model": model, "request": request, "route": route,
            "passes": passes, "effort": effort, "audience": aud.label})

    system = system_prompt(aud)
    messages: list[dict[str, Any]] = [first_message(request, snap)]
    history: list[dict[str, Any]] = []
    best: tuple[dict, dict] | None = None
    stop = "passes exhausted"
    per_call: list[float] = []
    for i in range(passes + 1):
        stage = "drafting" if i == 0 else f"reviewing pass {i}/{passes}"
        progress(stage)
        try:
            data, content, meta = send(client, model, system, messages, effort)
        except Exception as e:
            history.append({"pass": i, "stage": stage, "error": f"{type(e).__name__}: {e}"})
            echo(f"[{stage}] call failed: {e}")
            stop = f"model call failed: {type(e).__name__}"
            break
        per_call.append(meta["latency_s"])
        messages.append({"role": "assistant", "content": content})
        raw = data.get("playable") if isinstance(data.get("playable"), dict) else {}
        pl = L._normalize(raw, request, model)
        if caps:
            pl["caps"] = caps
        pl, g = gate(pl)
        rec = {"pass": i, "stage": stage, "latency_s": meta["latency_s"], "usage": meta.get("usage"),
               "served_by": meta.get("model"), "ok": g["ok"], "retimed": g["retimed"],
               "violations": g["text"], "scores": data.get("scores"), "satisfied": bool(data.get("satisfied")),
               "critique": data.get("critique", ""), "changelog": data.get("changelog") or []}
        history.append(rec)
        _save(sd, f"pass-{i}.json", pl)
        L._log({"tag": f"visual-pass-{i}", **{k: v for k, v in rec.items() if k != "usage"},
                "usage": meta.get("usage")})
        echo(f"[{stage}] {meta['latency_s']}s  " + ("VALID" if g["ok"] else "REJECTED")
             + (f"  scores {data.get('scores')}" if data.get("scores") else "")
             + ("  satisfied" if data.get("satisfied") else ""))
        if g["ok"]:
            best = (pl, g)
        if i > 0 and g["ok"] and data.get("satisfied"):
            stop = "satisfied"
            break
        if i == passes:
            break
        if deadline_s is not None:
            left = deadline_s - (time.monotonic() - t_start)
            if left < max(per_call) * 1.2 + 8:
                stop = f"deadline ({left:.0f} s left)"
                break
        progress(f"rendering pass {i + 1}")
        try:
            out = (sd / f"pass-{i}") if sd else Path(_tmpdir()) / f"pass-{i}"
            sheets = R.render_sheets(pl, out, audience=aud, max_sheets=REVIEW_SHEETS)
        except Exception as e:  # unrenderable (schema-broken): text-only review
            echo(f"[render] failed: {e}")
            sheets = []
        messages.append(review_message(i + 1, passes, pl, g, sheets, last=(i + 1 == passes)))

    out_res: dict[str, Any] = {
        "ok": best is not None, "playable": best[0] if best else None,
        "report": best[1]["report"] if best else None, "passes": history, "route": route,
        "stop_reason": stop, "latency_s": round(time.monotonic() - t_start, 2), "model": model,
        "sheets_dir": str(sd) if sd else None, "retimed": bool(best and best[1]["retimed"]),
    }
    if best is None:
        out_res["error"] = "no candidate passed the validator"
    if sd is not None:
        if best is not None and render_final:
            try:
                R.render_sheets(best[0], sd / "final", audience=aud)
            except Exception as e:
                echo(f"[render final] failed: {e}")
        _save(sd, "session.json", {k: v for k, v in out_res.items() if k != "report"})
        (sd / "CHANGELOG.md").write_text(changelog_md(request, out_res))
    progress("final")
    L._log({"tag": "visual-result", "ok": out_res["ok"], "stop_reason": stop, "latency_s": out_res["latency_s"]})
    return out_res


def _tmpdir() -> str:
    import tempfile
    return tempfile.mkdtemp(prefix="armctl-sheets-")


def changelog_md(request: str, res: dict[str, Any]) -> str:
    r = res["route"]
    lines = [f"# Visual choreography: \"{request}\"", "",
             f"- model `{res['model']}`, {len(res['passes'])} call(s), {res['latency_s']} s total, "
             f"stopped: {res['stop_reason']}",
             f"- camera: {'yes' if r.get('use_camera') else 'no'} ({r.get('method')}): {r.get('reason')}"
             + (f"; snapshot failed: {r['snapshot_error']}" if r.get("snapshot_error") else ""),
             f"- result: {'VALID ' + res['playable']['name'] if res['ok'] else 'nothing valid (fallback)'}", "",
             "| pass | valid | " + " | ".join(SCORE_KEYS) + " | satisfied | latency s |",
             "|---|---|" + "---|" * len(SCORE_KEYS) + "---|---|"]
    for h in res["passes"]:
        if "error" in h:
            lines.append(f"| {h['pass']} | error: {h['error']} |" + " |" * (len(SCORE_KEYS) + 2))
            continue
        sc = h.get("scores") or {}
        lines.append(f"| {h['pass']} | {'yes' if h['ok'] else 'no'}{' (retimed)' if h['retimed'] else ''} | "
                     + " | ".join(str(sc.get(k, '')) for k in SCORE_KEYS)
                     + f" | {h['satisfied']} | {h['latency_s']} |")
    for h in res["passes"]:
        if "error" in h:
            continue
        lines += ["", f"## Pass {h['pass']} ({h['stage']})", ""]
        if h.get("critique"):
            lines += [f"Critique: {h['critique']}", ""]
        lines += [f"- {c}" for c in h.get("changelog") or []]
        if h.get("violations"):
            lines += ["", "Validator:", "```", h["violations"], "```"]
    return "\n".join(lines) + "\n"
