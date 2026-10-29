"""openplay.say: English to a playable for "Say it" ("wave at the kids").

    res = await author("wave at the kids")
    # {"playable": {...}, "title": "Wave at the kids", "source": "claude" | "library", "note": "..."}

With ANTHROPIC_API_KEY set, Claude authors the playable through ``armctl.llm_author``
(validated against the armctl envelope, with repair rounds) in a worker thread, because
that call blocks for many seconds. Without a key, or when Claude fails, times out or
returns something that doesn't validate, we play the closest library move
(``openplay.moves.match``) and say so in ``note``: a station at an event must always
do *something* friendly when asked.

The key is never logged or printed. ``llm_author`` logs prompts and responses (no
secrets) to its own experiment log.

Visual mode (``OPENPLAY_SAY_VISUAL=1``, off by default): ``armctl.visual_author`` drafts the move
with Claude Fable (``ARMCTL_VISUAL_MODEL``), renders sheets of it from the audience's position and
sends them back for review passes (``OPENPLAY_SAY_PASSES``, default 2), plus a live camera snapshot
when the request refers to the scene. Each request is then 1 + passes model calls and roughly 1-4
minutes, so its timeout is ``OPENPLAY_SAY_VISUAL_TIMEOUT`` (default 240 s); the loop returns its best
validated version before that. A short request that names a library move ("wave", "nod") plays the
stored move directly. Progress ("drafting", "reviewing pass 1/2", "final") goes to ``progress``.
Sheets and the changelog are kept under ``OPENPLAY_SAY_SHEETS`` (default the experiment's logs).
"""
from __future__ import annotations

import asyncio
import os
import re
from typing import Any

TITLE_MAX = 32
FALLBACK_MOVES = ("wave", "wave-hello", "nod", "happy-dance")
REPO = __import__("pathlib").Path(__file__).resolve().parents[1]
SHEETS_ROOT = os.environ.get("OPENPLAY_SAY_SHEETS",
                             str(REPO / "experiments/2026-10-29-visual-choreo/logs/say"))


def visual_enabled() -> bool:
    return os.environ.get("OPENPLAY_SAY_VISUAL", "").strip().lower() in ("1", "true", "yes", "visual")


def timeout_for(default_s: float) -> float:
    """The say timeout: the caller's default, or the (longer) visual one."""
    if visual_enabled():
        return float(os.environ.get("OPENPLAY_SAY_VISUAL_TIMEOUT", "240"))
    return default_s


class SayError(RuntimeError):
    """Nothing could be authored or matched; ``str(e)`` says why."""


def make_title(text: str, n: int = TITLE_MAX) -> str:
    """A short caption title from the request: "wave at the kids!!" -> "Wave at the kids"."""
    t = re.sub(r"\s+", " ", str(text or "")).strip().strip(".!?,;: ")
    if not t:
        return "Something new"
    if len(t) > n:
        cut = t[:n].rsplit(" ", 1)[0] if " " in t[:n] else t[:n]
        t = cut.rstrip(".,;: ") + "…"
    return t[0].upper() + t[1:]


def _claude(text: str) -> dict[str, Any]:
    """Blocking: llm_author with the SDK backend. Returns its result dict."""
    from armctl import llm_author

    return llm_author.author(text, backend="sdk", rounds=2, allow_retime=True, echo=lambda *_a, **_k: None)


def _claude_visual(text: str, progress, deadline_s: float) -> dict[str, Any]:
    """Blocking: the visual multi-pass author (Fable + render sheets). Returns its result dict."""
    import time as _t

    from armctl import llm_author, visual_author

    sheets = os.path.join(SHEETS_ROOT, _t.strftime("%Y%m%d-%H%M%S-") + (llm_author.P.slugify(text)[:40]))
    return visual_author.author_visual(
        text, passes=int(os.environ.get("OPENPLAY_SAY_PASSES", "2")), camera="auto", sheets_dir=sheets,
        deadline_s=deadline_s, progress=progress, echo=lambda *_a, **_k: None)


def library_first(text: str) -> str | None:
    """In visual mode, a short request that names a library move plays that move directly."""
    if len(re.findall(r"[a-z']+", text.lower())) > 3:
        return None
    try:
        from openplay import moves
        return moves.match(text)
    except Exception:
        return None


def _library(text: str, note: str) -> dict[str, Any]:
    try:
        from openplay import moves
    except Exception as e:  # the moves module is owned elsewhere and may be missing
        raise SayError(f"{note}; and the move library is unavailable ({type(e).__name__})") from None
    name = moves.match(text)
    if not name:
        try:
            known = {m["name"]: m for m in moves.catalogue()}
        except Exception:
            known = {}
        name = next((m for m in FALLBACK_MOVES if m in known), next(iter(known), None))
        if not name:
            raise SayError(f"{note}; and no library move matched")
        note += f" (nothing matched, so: {name})"
    try:
        playable = moves.load(name)
    except Exception as e:
        raise SayError(f"{note}; could not load move {name!r}: {e}") from None
    title = name
    try:
        title = next((m.get("title") or name for m in moves.catalogue() if m.get("name") == name), name)
    except Exception:
        pass
    return {"playable": playable, "title": title, "source": "library", "note": note}


async def author(text: str, timeout_s: float = 60, progress=None) -> dict[str, Any]:
    """English -> {"playable", "title", "source": "claude"|"library", "note"}. Raises SayError.

    ``progress(stage: str)`` (optional) is called on the event loop in visual mode."""
    text = str(text or "").strip()[:200]
    if not text:
        raise SayError("say something first")
    visual = visual_enabled()
    if visual and library_first(text):
        return await asyncio.to_thread(_library, text, "a library move matched: playing it as stored")
    if os.environ.get("ANTHROPIC_API_KEY"):
        try:
            if visual:
                loop = asyncio.get_running_loop()

                def prog(stage: str) -> None:
                    if progress is not None:
                        loop.call_soon_threadsafe(progress, stage)
                job = asyncio.to_thread(_claude_visual, text, prog, max(30.0, timeout_s - 15))
            else:
                job = asyncio.to_thread(_claude, text)
            # The worker thread can't be killed; on timeout it finishes in the background.
            res = await asyncio.wait_for(job, timeout_s)
            if res.get("ok") and isinstance(res.get("playable"), dict):
                note = "retimed to fit the speed limits" if res.get("retimed") else ""
                return {"playable": res["playable"], "title": make_title(text), "source": "claude", "note": note}
            note = "Claude's choreography didn't pass the safety check: playing the closest library move"
        except asyncio.TimeoutError:
            note = f"Claude took longer than {timeout_s:.0f} s: playing the closest library move"
        except Exception as e:
            note = f"Claude failed ({type(e).__name__}): playing the closest library move"
    else:
        note = "no Claude key: playing the closest library move"
    return await asyncio.to_thread(_library, text, note)
