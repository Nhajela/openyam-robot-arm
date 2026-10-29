# 2026-10-29 Pick up anything you name

**Experimenters:** Naman (goal: "not just a cup, it should be able to pick up anything, at least try. Only then can it dream of playing chess") + Claude (cloud build on serenity)
**Goal:** name any object ("the tape roll", "marker", "blue box"), or tap it, and the arm picks it up. Reuse what exists (DimOS's open-vocabulary detector) instead of hand-building finders, per [2026-10-29-reuse](../2026-10-29-reuse/REPORT.md).
**Status:** built; synthetic tests green. The real detector (YOLO-E) has not run yet: its weights download on the lab laptop (GPU). The real arm is next.

## Environment
- serenity, repo `.venv` (Python 3.12.3, OpenCV 5.0.0). No GPU, no DimOS in this venv, no YOLO-E weights (the download on serenity was stopped by Naman: models run on the laptop's GPU).

## 11:50 IST: where the real-arm attempt broke, and the plan
- **Observation (29 Oct real runs, [first-pick LOG](../2026-10-29-first-pick/LOG.md)):** the geometry was fine (cup found to ±1 mm by the rim finder). What failed was **segmentation**. The tap path's GrabCut, seeded from one pixel and a table-colour model, refused every tap on the white cup on white paper and on the light cup on dark wood ("bare_table").
- **Insight:** the pipeline *after* the mask is already general: mask → footprint + height (prism fit on the table plane) → grasp across the narrow side → refusals (too wide, flat or tall) → the closed-loop engine. Only the mask step is cup-specific or fragile. Open-vocabulary segmentation is a solved problem: YOLO-E ships with DimOS (`dimos.perception.detection.detectors.yoloe`).
- **Decision:** one detector seam, then the existing geometry and engine.

## 11:50–12:30 IST: built
- `interfaces/wristcam/detect.py`: `Det2D` (box, full-frame mask, label, score, track id) and three detectors.
  - `YoloeDetector`: DimOS's YOLO-E when DimOS is installed, else Ultralytics directly; GPU auto. Text prompts, or prompt-free (names everything it sees).
  - `ClaudeBoxDetector`: Claude vision box → GrabCut inside the box. The fallback for prompts YOLO-E can't ground.
  - `SynthDetector`: ground-truth masks from the synthetic world, for tests.
- `target.target_from_mask`: the tap path's geometry, now callable with any mask. `resolve()` (tap) uses it unchanged.
- `target.resolve_prompt(frame, view, prompt, detector, wc)`: returns the best graspable match plus every candidate with its xy, score and refusal. `prompt=None` gives everything on the table.
- `target.DetectorFinder`: in the closed loop, it **re-detects the object by name at every look**. It keeps the detection nearest the prediction that has the target's size, and falls back to the tap path's template tracker when the detector misses.
- `interfaces/wristcam/pick_anything.py`, modes:
  - `--simulate`: the synthetic table, no models.
  - `--bench N`: detector speed on live frames; nothing moves.
  - detect only: a saved frame (`--image --q`) or the live camera, with q read from the daemon; nothing moves.
  - `--execute`: the real arm, each motion y/N, refused unless calibrated.
- `interfaces.follow.run --prompt X`: follow whatever is named (the detector seeds and anchors the track).
- **Observation (possible DimOS bug, to check on the laptop):**
  - DimOS's `Detection2DSeg.from_ultralytics_result` resizes `masks.data` straight to the frame size.
  - Ultralytics letterboxes a 1280×720 frame to 640×384 (a multiple of 32), so that resize stretches the mask vertically by ~6%.
  - Our adapter uses `masks.xy` (polygons already in frame pixels) instead.
  - Check: overlay both on a real frame.
- **Problem → Fix:** a tap-to-pick test (`test_tracker_keeps_the_box_over_three_homing_looks`) failed only when run after the new tests. GrabCut seeds its colour models from OpenCV's global RNG, so the same frame could give a different mask depending on what ran before. Fix: `cv2.setRNGSeed(0)` before each GrabCut, so segmentation is reproducible.

## Synthetic results (`interfaces/tests/test_pick_anything.py`, 6 tests)
Scene: a red cup, a green box (5 × 3.5 × 4 cm, yaw 0.5) and a blue box (4 × 3 × 5 cm), with the camera 0.25 m above the table.

| Check | Result |
|---|---|
| "green box" | xy within 3 mm, height 4.02 cm (true 4.0), width across the fingers 3.5 cm, 1 candidate |
| "cup" | the red cup, kind `cup`, within 4 mm |
| no prompt | all 3 objects, each with a footprint |
| "banana" | refused: `not_found`, "I can't see 'banana' here" |
| a 13 × 12 cm "shoebox" | refused: "too wide to grab" (`run_pick` won't move) |
| closed loop on "blue box" | search → center → approach → descend → close → lift → verify → place → ready → done. 8 looks, ≥ 3 re-detections by name. Lifted, put back within 6 mm, cup and green box untouched, no finger collisions |
| detector blind for a look | the template fallback finds it within 4 mm |
| Ultralytics adapter | polygons → full-frame masks, names, scores and track ids; specks dropped |

Annotated look ([sim-look-blue-box.jpg](sim-look-blue-box.jpg)): the white outline is the fitted footprint at the top height, and the red line is where the fingers close.

- **Observation:** during the last descent looks, the detector's footprint often fails the size check (the object fills the view, cut by the image edge and the jaws), so the finder reports "lost" there. The engine still converged and grasped, because those looks are only alignment checks. On the real arm, watch how often this happens, and relax the size check close up if it matters.

## Next (lab laptop, GPU)
1. `pick_anything --bench 30 --prompt "cup,marker,tape,box,pen"`: YOLO-E speed on the GPU and what it sees in our oblique fisheye view (nothing moves). The weights download on first use (DimOS `get_data`, ~185 MB, or Ultralytics).
2. Detect only on the live camera with a few everyday objects: check the masks (overlay), the refusals, and the footprint errors against a ruler.
3. `--execute` on a foam block or a marker, then a cup. Keep looks short (j4 heat).
4. Then chess pieces: a named piece on a known square is the same pipeline with a much smaller object. The grasp height and tolerances will need tuning.
