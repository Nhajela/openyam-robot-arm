#!/usr/bin/env bash
# Author one ensemble move with the Fable visual loop. Usage: author.sh <name> "<request>"
# The key is mapped from .env for this process only (never exported or printed).
set -uo pipefail
cd "$(dirname "$0")/../.."
X=experiments/2026-10-30-ensemble-moves
name=$1; shift
ENSEMBLE="Lots of movement: every one of the six joints AND the gripper should be doing something through most of the move, overlapping (staggered starts, follow-through), not one joint at a time. Use big, readable amplitudes from the audience; vary the tempo with a build and at least one held accent. 15-25 seconds."
ANTHROPIC_API_KEY=$(sed -n 's/^CLAUDE_API_KEY=//p' .env) .venv/bin/python -m armctl.llm_author \
  "$* $ENSEMBLE" --visual --passes 3 --model claude-fable-5-1 --caps dance --camera never \
  --sheets-dir $X/runs/$name --out $X/$name.json 2>&1 | tee $X/logs/author-$name.log
