# Tap to pick / say what to pick (2026-10-29)

Part of [LOG.md](LOG.md). The station owner's ask: *"target detection where people maybe tap on the thing they want the arm to pick up, and then it hones in, tries to pick up, eventually just with prompts people should be able to say, pick up the cup on the table"*.

**Status:** built and tested in simulation only. Nothing in this note ran on the real arm. It is off on the real arm until the wrist camera is calibrated (see "Needs from the real arm").

## Environment
Lab laptop, Linux 6.8.0-48 (Ubuntu 24.04), Python 3.12.3 (repo `.venv`, opencv-contrib 4.14.0, numpy, scipy 1.18.1, anthropic 2.3.0), worktree branch based on `2d1ae2e`. Tests only: synthetic renders (`interfaces/wristcam/synth.py`), a fake arm, a fake Claude client. No daemon, no camera, no CAN, no zenoh, no API calls.

## What was built
| Piece | Where |
|---|---|
| `TargetRequest` (a tap pixel or a prompt on a still taken at a known q), `LookFrame`, `Target` | `interfaces/wristcam/target.py` |
| `ClaudeLocator`: frame (768 px JPEG) + prompt → strict JSON `{found, point, box, label, confidence, reason, candidates}`, rescaled; not found / ambiguous / unsure → candidates to tap | same |
| `resolve`: tap → refusals (picture edge, jaws, off table, reach) → region (table-colour model + GrabCut) → **single-view prism fit** (footprint + height) → grasp across the narrow side, or a refusal (too wide / flat / tall) | same |
| `SeededFinder`: keeps the tapped target across the engine's looks (NCC of the top face on the plane at its height, re-segmentation, `CupFinder` for cups, gate) | same |
| `run_pick`: `GraspEngine` + `SeededFinder`, progress callback | same |
| Open play mode **Pick it up** (phone: look → tap/say → "Pick this?" → progress; display: camera with outline, live looks, engine state, Cancel) | `openplay/pick_play.py`, `static/pick.js`, `static/display_pick.js`, `static/pick.css`; protocol in `openplay/PROTOCOL.md` |
| `SimArm` grips boxes too (stall at the box's width across the fingers; box collisions recorded) | `interfaces/wristcam/synth.py` |

## Results (synthetic; `interfaces/tests/test_wristcam_target.py`, `openplay/tests/test_pick.py`)
Scene: red cup 6 × 8 cm at table (−0.02, −0.20), green box 5 × 3.5 × 4 cm at (0.06, −0.08), yaw 0.5; camera straight down 0.25 m over (0.02, −0.13); fisheye, jaws drawn, noise, JPEG.

| What | Result |
|---|---|
| Tap the box (6 px off its centre) | xy error **1.4 mm**, height 46 mm (true 40), size 49.8 × 35.6 mm, yaw 0.504 (true 0.5) |
| Tap the cup | xy error **0.4 mm**, 79 mm tall, 61 mm across, `kind: cup` (circle model wins) |
| Prompt, mocked Claude answers a point | xy error **0.2 mm** |
| Tracker over the engine's 0.18 / 0.12 / 0.08 m looks | 0.14 / 0.36 / 0.36 mm, never jumps to the cup 13 cm away |
| Engine on the tapped box in the cup+box scene | search → centre → approach → descend → close → lift → verify → place → ready: lifted (box > 5 cm up when verified), put back within 6 mm, cup untouched, no finger collisions, all moves ≤ 0.3 rad/s inside the soft box |
| Open play end to end (in-process server, fake ArmBridge over the sim arm) | look → tap → confirm → done; prompt found / ambiguous (candidates) / not found; operator approve / deny (deny: nothing moves); uncalibrated → "camera not calibrated yet", nothing moves |
| Full suite | `pytest -q interfaces/tests openplay/tests armctl/tests`: **465 passed, 1 xfailed** (the known board-circle xfail) |

## Findings
- **Problem → Insight:** the first height estimator, "largest footprint consistent with the silhouette for each h, pick the best re-cast IoU", returned h ≈ 0 for everything. **Insight:** along every ray from the nadir, any h up to the true one re-casts the silhouette exactly, so the silhouette alone can't give the height. What fixes it is a shape prior: the top face has the same shape as the footprint. **Fix:** fit a rectangle or circle prism (footprint + h) to the silhouette's hull (symmetric-difference area, Nelder-Mead). Errors went from 9–33 mm to 0.3–1.7 mm.
- **Problem → Fix:** `cv2.intersectConvexConvex` misbehaved on polygons in metres (fits ran off to metres away). Working in mm fixed it, plus clamping the intersection to min(areas).
- **Observation:** directly under the camera the height is unobservable (flat cost curve). The prior (6 cm) is used and marked `height_known: false`; the tracker refines the height from later oblique views. A 4 cm box tapped at ~7 cm from the nadir sometimes comes out "unknown", and then it's grasped at 0.6 × 5.8 cm. That works for the box but is the weakest part.
- **Problem → Fix (engine):** short objects (4 cm box) failed the table guard: the approach (top + 3.5 cm), the joint path from the camera-down pose to fingers-down, and the "lift clear" before ready were all below `kin.MIN_TIP_Z` (0.08 m). The approach height now never goes below the guard + 8 mm, the fingers turn down 4–8 cm higher first when the path would sag, and the lift-clear uses the same height. The cup tests are unchanged.
- **Decision:** after a successful pick the arm puts the object back where it was and goes to ready, so the next player finds the table as it was. A place target can be added later.
- **Decision:** with auto-play off the operator approves twice (the look pose, then the whole pick), not every engine step. Each step is still a daemon `move_joints`, checked by the engine and by the daemon.

## Needs from the real arm (in order)
1. **Lens + hand-eye calibration that passes the gates.** Until then the mode is refused: `workcell.json calibrated: true` (written only by `handeye solve --write-workcell` when every gate passes), `calib/handeye.json`, `calib/wristcam.json` (README §§ 0–3). Remove the sheet afterwards.
2. **The look pose.** Default: camera straight down 0.25 m over table (0, −0.15). Check that it's reachable and that the cup area is in view; override `look_xy` / `look_h` in `interfaces/wristcam/pick_config.json`. Also `table_extent` (the cloth area, table frame), so that taps beyond the cloth are refused.
3. **The gripper opening.** Measure the real finger stroke (i2rt crank_4310 may be ~71 mm, not the URDF's 94 mm) and set `max_open`. Also the measured grip → width mapping and the stall reading on a cup and on a block (`grasp.holding`).
4. **Jaw mask:** check `camera.JAW_POLYGONS` against a real frame (gripper open and closed); taps there are refused.
5. **Frame freshness after a move:** `pick_play` waits `settle_s` (1 s) and needs q unchanged across the grab. Check that the console frame is from after the move.
6. **Segmentation on the real cloth:** the white paper cup on light cloth is the hard case (the table-colour model needs contrast). Try a coloured cup first. Record a still + q from `pick.look` and replay `resolve` offline.
7. **First run:** in `--simulation` with the same calibration files, then the real arm with auto-play **off**, a soft object, a hand on E-STOP. Watch the descent (finger clearance 1 cm each side).

## Ideas
- **Idea:** refine the height with two looks (the look frame + the first centring look) instead of one: the second view is oblique to the first, so the height becomes observable even for a target right under the first camera.
- **Idea:** "put it in the bowl": a second tap (or prompt) for the place target.
- **Idea:** let Claude also suggest the grasp axis for odd shapes (mugs with handles), checked against the footprint.
