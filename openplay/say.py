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
"""
from __future__ import annotations

import asyncio
import os
import re
from typing import Any

TITLE_MAX = 32
FALLBACK_MOVES = ("wave", "wave-hello", "nod", "happy-dance")


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


async def author(text: str, timeout_s: float = 60) -> dict[str, Any]:
    """English -> {"playable", "title", "source": "claude"|"library", "note"}. Raises SayError."""
    text = str(text or "").strip()[:200]
    if not text:
        raise SayError("say something first")
    if os.environ.get("ANTHROPIC_API_KEY"):
        try:
            # The worker thread can't be killed; on timeout it finishes in the background.
            res = await asyncio.wait_for(asyncio.to_thread(_claude, text), timeout_s)
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
