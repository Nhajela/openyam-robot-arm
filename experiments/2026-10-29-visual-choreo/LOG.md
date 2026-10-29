# 2026-10-29 visual choreography (Claude Fable + render sheets)

**Experimenters:** Naman (+ Claude agent)
**Goal:** Choreograph clearly better moves than today's library. The station owner: "let's use fable models, and give fable a multi view visual of keyframes and midpoints between keyframes as a sheet to look at, multiple sheets, with renders of the model, for it to do 2-3 passes of adjustment and general choreography guidelines"; "especially in how it looks like to someone"; "visual review for each choreography for sure, but camera feed snaps yes or no is the question".
**Status:** active: built and tested offline (mocked client); real API run below; real-arm playback of an authored move next.

## Environment
Lab laptop `pheonix-drone`, Linux 6.8.0-48 x86_64, Python 3.12.3 (`/home/naman/openyam-robot-arm/.venv`: numpy 2.5.4, matplotlib 3.11.2, OpenCV 4.14.0, anthropic 2.3.0), worktree branch off `main` @ `2d1ae2e`. The real arm was running in another session the whole time: nothing here touched CAN, DimOS, zenoh, the console ports or the camera. Envelope: `armctl.envelope` default caps 0.3 rad/s, 1.0 rad/s², `site_limits.json` j1 −0.6..+1.2.

## What was built
- [`armctl/choreo_render.py`](../../armctl/choreo_render.py): offscreen (matplotlib Agg) contact sheets from FK link frames. **Overview** sheet: onion skin of all keyframes from the audience + hand path, down-table and top views, every joint vs time against its soft limits (red = within 0.15 rad), speed headroom (peak |v|/cap, |a|/cap). **Pose** sheets: ≤ 8 tiles per sheet, keyframes `K` and in-betweens `½` (plus `¼ ¾` on segments > 2 s; none for holds), each tile = big **audience** view (perspective from `armctl/audience.json`), small down-table + top views, grey ghost of the previous pose, labels (t, q, tip height, grip, near-limit / near-table warnings), and a compact timeline strip marking that sheet's tiles. 1800 px wide. Long moves are thinned (quarters, then midpoints, then keyframes) to a sheet budget. `metrics_text()` gives the same numbers in words, including **how far the hand moves on the audience's image** per segment.
- [`armctl/audience.json`](../../armctl/audience.json): where people stand (base frame). Default "standing": table's front edge on the arm's left (+y), eye (0.45, 1.10, 0.85) m; presets "seated", "down-table"; override `ARMCTL_AUDIENCE` or `--audience x,y,z`. **Needs measuring at each site.**
- [`armctl/choreo_guidelines.md`](../../armctl/choreo_guidelines.md): the choreography rules in the system prompt (audience, joint "meanings" measured with FK, principles, the speed caps as the main constraint, what's wrong with the library today).
- [`armctl/choreo_scene.py`](../../armctl/choreo_scene.py): camera router (rules; a small `claude-haiku-5-5` structured call only when the rules are unsure; `--camera auto|always|never`) and the snapshot: `http://127.0.0.1:8090/camera.jpg` + the pose from `/api/state`, with a labelled grid mapped to base-frame yaw / table points (calibrated only if `handeye.json` exists and `workcell.json` says `calibrated: true`; otherwise a flagged nominal estimate).
- [`armctl/visual_author.py`](../../armctl/visual_author.py): the loop. Draft → (validate + table check → render → send sheets + validator report + metrics → critique, 5-score rubric, revised playable, changelog) × passes. Stops early when the shown candidate is valid and Claude says satisfied. Only validated candidates are ever returned (the latest valid one); speed-only violations may be fixed by `retime()` (slows down, never relaxes). Fable request: streamed, `output_config` effort + JSON schema, no `thinking` param, `fallbacks: "default"` (beta `server-side-fallback-2026-07-01`), refusal/max_tokens handled, assistant turns echoed back unchanged (append-only history), prompt caching on.
- CLI: `python -m armctl.llm_author "a happy wave" --visual --passes 3 --model claude-fable-5-1 --out playables/<name>.json --sheets-dir <dir> [--camera auto] [--audience seated] [--effort high] [--caps dance]`. Without `--visual` the old text path is unchanged.
- Open play "Say it": `OPENPLAY_SAY_VISUAL=1` (off by default), progress on the phone ("drafting", "reviewing pass 1/2", "final"), 240 s timeout with the loop's own deadline 15 s earlier, library fallback unchanged; short requests naming a library move play it directly.
- Tests (mocked client, no camera): `armctl/tests/test_choreo_render.py`, `armctl/tests/test_visual_author.py`, visual cases in `openplay/tests/test_say.py`.

## Log

### 08:30 Library study: what's wrong with today's moves
`.venv/bin/python experiments/2026-10-29-visual-choreo/library_stats.py` → [logs/library_stats.txt](logs/library_stats.txt)
```
median range per joint: [0.43 0.3  0.4  0.35 0.   0.24]  median tempo CV: 0.43
wave-hello   10 keys 12.4 s tempoCV 0.19  range 0.16 0.20 0.30 0.30 0.00 0.24  cubic only, no holds
happy-little-dance 9 keys tempoCV 0.00 (every segment the same length)
```
- **Observation:** amplitudes are small (median j5 range 0: the head never turns), tempo is often metronomic (wave, nod, happy-dance, gripper-clap, beat-dance-auto-hype CV ≤ 0.2), 13/29 have no hold, 16/29 never change the gripper.
- **Observation:** `python -m armctl.choreo_render playables/wave-hello.json` metrics: from the audience the hand moves **1–2 cm per wave segment** (j1 ±0.08, j6 ±0.12); every wave segment is flagged "barely visible from the audience". Sheets: [sheets/wave-hello/](sheets/wave-hello/) ([poses-1.png](sheets/wave-hello/poses-1.png)): the four wave tiles are visually identical.
- **Insight:** the default caps (0.3 rad/s) push authors to shrink moves to keep a tempo; the guidelines now say to spend the few big moves on the gesture that carries the meaning and use holds/grip/j6 for accents.

### 08:40 FK joint "meanings" from ready (for the guidelines)
Tip displacement for +0.4 rad per joint from ready: j1 +y 8 cm (turns toward the audience), j2 forward 11 cm + down 10 cm (lean in), j3 back 11 cm + up 15 cm (stand tall / look up), j4 up 7 cm (head up), j5 −y 5 cm (head turns away from the audience), j6 no tip motion (roll only).

### 08:50 One real API run (request shape check): "a happy wave", Fable, effort low, 1 review pass
`.venv/bin/python experiments/2026-10-29-visual-choreo/run_visual.py "a happy wave" --passes 1 --effort low --camera never --sheets-dir experiments/2026-10-29-visual-choreo/sheets/run1-happy-wave --out experiments/2026-10-29-visual-choreo/run1-happy-wave.json` (key from `.env` `CLAUDE_API_KEY`, never printed) → [logs/run1-happy-wave.txt](logs/run1-happy-wave.txt), [CHANGELOG](sheets/run1-happy-wave/CHANGELOG.md), sheets per pass in [sheets/run1-happy-wave/](sheets/run1-happy-wave/), final [overview](sheets/run1-happy-wave/final/overview.png), result [run1-happy-wave.json](run1-happy-wave.json).
```
[drafting] 30.51s  VALID (retimed)  scores readability 4 silhouette 4 energy_contrast 3 timing 4 safety_margin 4
[reviewing pass 1/1] 31.52s  VALID  scores 3 3 3 4 3   (scores are for the draft it was SHOWN)
model=claude-fable-5-1 2 call(s) 71.36 s
usage pass 0: cache write 7,429 in, 2,319 out (1,583 thinking); pass 1: cache read 7,429 + write 16,318 in, 2,303 out
```
- **Observation:** the request shape works on the real API: streamed, structured JSON, `fallbacks: "default"`, no fallback triggered (served by `claude-fable-5-1`), images accepted (overview + 3 pose sheets ≈ 9k input tokens).
- **Observation (the point of this experiment):** the review pass caught, from the sheets, exactly what was wrong with the draft: *"the actual wave is invisible: each wave segment moves the hand only 6 cm because the arm is stacked vertically over the base and j5/j6 only twiddle the fingers"*, and moved the wave onto big j1 swings 0.5↔1.0 rad across the audience's view, reached outward (j2 0.8, j3 1.3) for a longer lever, varied the third swing, and lengthened segments to get ~10 % velocity headroom instead of relying on retime. Compared with the library `wave-hello` (hand 1–2 cm per wave segment, 7 of 9 segments "barely visible", tempo CV 0.19): range j1 1.00 / j5 0.40, tempo CV 0.63, 2 holds, 3 grip values, 0 barely-visible segments.
- **Observation:** the draft was speed-only invalid and `retime()` fixed it (gate is never relaxed).
- **Insight:** cost at effort low ≈ $0.53 for draft + 1 review (cache writes $12.5/MTok, output $50/MTok); 71 s. Estimate at effort high, draft + 3 reviews: input grows ~9–10k tokens per review (images), output ~3–8k per call → about **$1.5–3 and 2–5 minutes per authored move**. Prompt caching makes each review pay mostly for the new images.
- **Problem / Idea:** the final playable (pass 1 output) is validated but not itself reviewed when passes run out; with `--passes 3` the last review's revision is the one returned. A spare "confirm" pass costs one more call.
- **Decision:** not played on the arm (the arm is in use by another session). Next: preview `run1-happy-wave.json` in sim, then on the real arm with Naman at the arm.

### Offline tests
`.venv/bin/python -m pytest -q armctl/tests openplay/tests interfaces/tests` → `474 passed, 1 xfailed in 286.92s` (new: 5 render tests, 12 visual-author tests with a fake client, 4 Say-it visual-mode tests; no network, no camera, the e2e test uses its mock daemon on 8591/8550/8551).
