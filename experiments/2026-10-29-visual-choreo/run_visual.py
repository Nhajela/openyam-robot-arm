"""Run the visual author once with the key from the repo's .env (CLAUDE_API_KEY -> ANTHROPIC_API_KEY).

    /home/naman/openyam-robot-arm/.venv/bin/python experiments/2026-10-29-visual-choreo/run_visual.py \
        "a happy wave" --passes 1 --effort low --camera never --sheets-dir experiments/2026-10-29-visual-choreo/sheets/run1

Arguments after the script name go to `python -m armctl.llm_author ... --visual`. The key is never printed.
Nothing moves the arm: this only authors and validates a playable.
"""
import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

if not os.environ.get("ANTHROPIC_API_KEY"):
    for env in (REPO / ".env", Path("/home/naman/openyam-robot-arm/.env")):
        if env.exists():
            for line in env.read_text().splitlines():
                k, _, v = line.partition("=")
                if k.strip() in ("ANTHROPIC_API_KEY", "CLAUDE_API_KEY") and v.strip():
                    os.environ["ANTHROPIC_API_KEY"] = v.strip().strip('"').strip("'")
                    break
        if os.environ.get("ANTHROPIC_API_KEY"):
            break

from armctl import llm_author  # noqa: E402

sys.exit(llm_author.main(sys.argv[1:2] + ["--visual"] + sys.argv[2:]))
