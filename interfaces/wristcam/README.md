# interfaces/wristcam: find and grasp an object with the wrist camera

The wide-angle "USB CAMERA 4K" module sits between the gripper fingers and looks along them. Its frames reach us through the lab console (`http://127.0.0.1:8090/camera.jpg`, source `dashboard`, 1280x720). Nothing is measured with calipers: the lens, the camera's mount on the gripper and the table are all calibrated with the camera and the arm's own kinematics, from **one printed sheet taped to the table**.

| File | What it does |
|---|---|
| `board.py` | The calibration sheet: ChArUco DICT_5X5_100, 9x6 squares of 28 mm, 21 mm markers, with a 50 mm scale bar. Writes `print/wristcam_calib_A4.pdf/.png`, `print/wristcam_tags_A4.pdf/.png` (three 40 mm DICT_4X4_50 tags, ids 2-4, for objects) and a screen version (`--screen WxH --pixel-mm P`). Defines the **table frame**: origin at the board centre, x along the printed "+x" arrow, y toward the top of the print, z up. |
| `camera.py` | `CameraModel` (fisheye / rational / pinhole behind one `project` / `rays` API), the nominal mount (camera at `gripper_tip`, optical axis along the fingers) and the jaw mask (the jaws hide the lower image corners). |
| `intrinsics.py` | Lens calibration: `capture` (frames from a source, diversity gate + coverage report) and `solve` (fits all three lens models and picks by reprojection error; a model with more parameters must win by >10%). |
| `handeye.py` | `plan_views` (feasible, varied looks at the board), `collect` (drives the arm through them, saving frame + measured q), `solve` (camera relative to `gripper_tip`, `cv2.calibrateHandEye` TSAI/PARK/HORAUD/ANDREFF/DANIILIDIS picked by consistency, then a least-squares refinement of the camera translation, kept only if it helps) and the table frame, written into `interfaces/grasp/workcell.json` with `calibrated: true` (old file backed up, other keys kept). |
| `locate.py` | Finders, all `find(frame, view, hint) -> Detection`: **`CupFinder`** (primary, tag-free: rim circle on the rim plane), `TagFinder`, `ColorBlobFinder`, and `ClaudeVisionFinder` (a documented seam, no API calls). |
| `engine.py` | The closed loop as a state machine with a JSON-lines log: search, centre-and-approach, descend with re-alignment, close, verify, retry, place. `DaemonArm` runs it on the real arm through the armctl daemon. |
| `look_and_grasp.py` | CLI: `--dry-run` (default), `--simulate`, `--execute`. |
| `target.py` | "Pick up **that**": a tap or a prompt on a still frame → `Target` (table xy, height, footprint, grasp width/yaw, outline) → `SeededFinder` that tracks it through the engine's closed loop. `ClaudeLocator` (prompt → pixel, Claude vision, mockable). See [below](#tap-or-say-what-to-pick-targetpy). Used by open play's "Pick it up" mode. |
| `detect.py` | **Pick up anything you name:** one seam for open-vocabulary detectors (`Det2D`: box, mask, label, score, track id). `YoloeDetector` (DimOS's YOLO-E, GPU on the laptop), `ClaudeBoxDetector` (Claude box + GrabCut, fallback), `SynthDetector` (tests). See [below](#pick-up-anything-you-name-detectpy-pick_anythingpy). |
| `pick_anything.py` | CLI: `--simulate`, `--bench N` (detector speed, nothing moves), detect-only on a frame or the live camera, `--execute`. |
| `synth.py` | Ray-cast synthetic wrist camera (any lens model) over a table with the sheet, tags, cups and boxes, plus `SimArm` (fake arm whose gripper stalls on a cup or a box). Used by the tests. |

## How it finds a cup

`workcell.json` `objects.cup` gives the cup's size (6 cm across, 8 cm tall). Every frame is resampled onto the **rim plane** (the table plane raised by the cup height) by intersecting each pixel's ray with that plane. There, the rim is an exact circle of the cup's radius, whatever the fisheye distortion or the view tilt. Hough circles in a ±25% radius window propose centres, and edge points with a radial gradient are fitted with RANSAC and least squares. A candidate is rejected if the edge support is sparse or if the band next to the circle isn't uniform; the second rule removes "circles" hallucinated on the checkerboard. The candidate with the widest angular support wins, so a jaw hiding part of the rim is fine. The finder returns:

- the rim centre (base and table frames),
- the diameter measured at the known height,
- a 1-sigma `sigma_xy`: fit noise ⊕ (rim-height uncertainty × how obliquely the rim is seen) ⊕ 1 mm.

## The closed loop (`engine.py`)

1. **SEARCH:** the camera looks straight down from 0.18 m above the cup top, over `--expect` and then 4 offsets of ±6 cm.
2. **CENTER:** move the camera so that its optical axis passes through the estimate, then move closer (0.18 → 0.12 → 0.08 m above the top) and estimate again. This converges when, at the last height, the cup is within 25 px of the image centre and the estimate moved less than 2 mm. The loop gives up after 7 iterations, or if the cup is lost twice in a row.
3. **APPROACH:** open the fingers to the cup width + 2 cm, with the tip 3.5 cm above the rim.
4. **DESCEND:** go down in 2 cm steps to 0.6 × the height. Look before each step: offsets over 1.5 mm are corrected, and an offset over min(8 mm, finger clearance - 3 mm) aborts back to CENTER. The camera must stay at least 1 cm above the rim.
5. **CLOSE:** close to the width - 1 cm. It's **holding** if the measured grip stopped short (`grasp.holding`).
6. **LIFT** 8 cm, then **VERIFY**: the grip must still be stalled, and the cup must no longer be on the table at its old position (another look).
7. **MISS:** open, back off, re-centre, and retry once.
8. **PLACE** at `--place` (travel height, down, open, up), then **READY**.

Every motion is a single `move_joints` that the daemon validates (soft box, speed caps, lease, E-STOP). Before sending it, the engine checks the joint-space path itself (`check_step`: soft box, tip ≥ 0.08 m except while descending, placing or releasing, never below the workcell's `min_tip_z`, links ≥ 0.03 m). Speeds are 0.25 rad/s, and 0.12 rad/s near the object (the daemon caps at 0.3). Every observation, decision and move goes to `run.jsonl`, with annotated frames in `frames/`.

**Reach:** with the fingers exactly down, the tip only reaches z ≈ 0.21 m (j4 hits its soft limit). The camera sits about 6 cm above the tip (to be measured by hand-eye), so the highest straight-down look is about 0.27 m above the table. Higher or wider views have to be tilted, and `plan_views` uses tilts for variety.

### Watching a run (`viz.py`)

```bash
.venv/bin/python -m interfaces.wristcam.viz experiments/2026-10-29-first-pick           # picker over every run under it, newest first
.venv/bin/python -m interfaces.wristcam.viz --latest-under interfaces/wristcam/runs   # follow whichever run is newest (start before the run)
```

Open http://127.0.0.1:8097/ (`--port` to change; 8090/8091 are refused). The page polls `run.jsonl` every 0.5 s, so it is live during a run and a replay afterwards. It shows the latest frame with a detection overlay (box and outline projected from the logged q and camera model, a `cup σ1.1mm 85mm` tag, the image-centre crosshair and the pixel-error line, a HUD with state and decision; ← → step through the observations), the state strip, the estimate (table/base xy, σ, measured width), px error and estimate change per look against their tolerances, the last decision, the last (or in-flight) move with its biggest joint change, a top-down table map (expected spot, search views, detections sized by σ, estimate, place spot, tip path, board outline, direction to the base) and the event log. A **LIVE** panel (next to the last look, or switch with the frames menu) shows the lab console's `/camera.jpg` at ~6 Hz with its age, and the current estimate projected at the arm's live q from the console's read-only `/armctl/status` (`--camera-url` / `--status-url`, `''` turns either off; "no live camera" when unreachable). Read-only: files plus GETs of those two URLs (any other `/armctl/` URL is refused); binds 127.0.0.1.

## Tap or say what to pick (`target.py`)

The engine above needs a finder for a *known* object (the cup). `target.py` makes "whatever the player tapped" work in the same loop. Open play's **Pick it up** mode (`openplay/pick_play.py`) is the front end; the library is usable on its own.

1. **Request.** A `LookFrame` is a still plus the q it was taken at (the arm must be still across the grab). A `TargetRequest` is a tap pixel on that frame, or a prompt. `ClaudeLocator` turns a prompt into a pixel: it sends the frame (longest side 768 px, JPEG) and the prompt to Claude (`$WRISTCAM_PICK_MODEL`, default `claude-opus-5-5`; `ANTHROPIC_API_KEY`), asks for strict JSON `{found, point, box, label, confidence, reason, candidates}` and rescales the pixels. Not found → the reason; ambiguous or confidence < 0.5 → `candidates` for the player to tap. Tests inject a fake client: no API calls.
2. **Refusals before any vision:** outside the picture, on the jaws (`camera.JAW_POLYGONS`), ray not hitting the table, outside `PickConfig.table_extent` (if set), outside the arm's reach (0.12–0.46 m from the base axis).
3. **Region.** A robust Lab colour model of the table around the tap (the region's border) marks "not table" pixels; the connected part at the tap seeds GrabCut; holes are filled (a cup's inside). A tap on bare table, a region touching the picture's edge, or one that runs into something else is refused.
4. **Footprint and height from one view.** Each outline pixel's ray hits the table plane: that's the object's "shadow" S0 seen from the camera centre. A vertical prism (rectangle or circle footprint, height h) casts the convex hull of its footprint and its top face projected onto the table. `fit_prism` fits footprint + h to S0 (Nelder-Mead on the symmetric-difference area, h on a 1 cm grid then refined). Off-nadir objects show their side walls and the height is well determined; directly under the camera it isn't (flat cost curve), so `PickConfig.default_height` (6 cm) is used, marked `height_known: false`, and the tracker refines it from later oblique views. The circle model winning = `kind: cup` (the rim finder helps later).
5. **Grasp choice:** top-down, closing across the footprint's narrow side (min-area rectangle). `refusal` is set (the player sees the outline and why) if narrow side + `open_margin` > the gripper opening (`Workcell.max_width` 94 mm from the URDF; **the real i2rt crank_4310 stroke may be ~71 mm**: set `max_open` in `pick_config.json` once measured), or the object is flatter than 1.5 cm or taller than 16 cm.
6. **Homing: `SeededFinder`.** In each engine look it predicts where the top is, resamples the frame onto the plane at the object's height (so the top face is the same size from any distance), NCC-matches the top-face template from the tap frame, re-segments there and refits the footprint (height fixed), and for a cup asks `CupFinder` for the rim. Candidates that jump outside the gate or don't match the size are rejected (the engine sees "lost", never a different object). `run_pick` wires it into `GraspEngine` with `pick_config` (width, height, finger yaw, put it back where it was) and reports progress through `on_event`.

Engine changes for short objects (4 cm box): the approach height is never below the general table guard (`kin.MIN_TIP_Z` + 8 mm), the fingers turn down higher up first when the joint path from the camera pose would sag below it, and the "lift clear" after a place uses the same height.

Synthetic results (`interfaces/tests/test_wristcam_target.py`, `openplay/tests/test_pick.py`; red cup 6×8 cm and a green 5×3.5×4 cm box at yaw 0.5, camera 0.25 m above the table, fisheye, jaws drawn): tap → box 1.4 mm (height 46 mm, size 49.8×35.6 mm, yaw 0.504), cup 0.4 mm (79 mm tall, 61 mm across); mocked-Claude prompt → box 0.2 mm; tracker over 0.18/0.12/0.08 m looks < 0.4 mm and never jumps to the cup; the engine picks the tapped box (not the cup), lifts it, verifies, puts it back within 6 mm, no finger collisions. Best case again: the renderer is the model.

## Synthetic accuracy (tests, 2026-10-28)

`interfaces/tests/test_wristcam_*.py`. The camera is a fisheye (f 700 px, ~102° HFOV) mounted 6.2 cm behind the tip, 8 mm off-axis and tilted 3°. Each render is supersampled, blurred, noisy and JPEG-compressed, with the jaws drawn in, and the joint readings carry 0.001 rad of noise.

| What | Result |
|---|---|
| Intrinsics (32 arm-moved views of the taped sheet) | fisheye chosen, RMS 0.06 px; fx within 0.1%; ray direction error median 0.006°, max 0.01° (inner 84% of the image) |
| Hand-eye (12 views, recovered lens, 0.001 rad joint noise) | rotation error 0.2°, translation error 0.5 mm (thresholds: 1°, 3 mm). Without joint noise: 0.02°, 0.1 mm |
| Table frame | 0.9 mm, 0.18° (with joint noise) |
| Cup at 24–25 cm (tilted 10–30°) | 0.05–0.10 mm, diameter within 0.3 mm; 32% of the rim behind a jaw: 0.06 mm |
| Tag on a box at 25 cm | 0.2 mm, yaw 0.1° |
| Closed loop | 9 looks: finds, centres, grasps, puts the cup down within 1 mm of the target; first-grasp miss → retry succeeds; no finger collisions; no false cup on a board-only table |

These numbers are best case: the renderer is the model. Expect a few mm on the real arm, from print flatness, focus, rim height and FK error.

## Real-arm procedure

Follow `CLAUDE.md`: try new behaviour in simulation first; ask the person at the arm before every command that moves it, and say what will move; after every stop, support the arm, run `tools/disable_all.py` and check that every LED is steady red.

### 0. Print

- Print `print/wristcam_calib_A4.pdf` on A4 at **100% / Actual size** (black and white is fine; matte paper is better). Measure the 50 mm bar. If it isn't 50.0 mm, pass `--square-mm 28*measured/50` to every `solve`.
- Optional: print `print/wristcam_tags_A4.pdf` for tagged objects.
- **Lab layout (measured 28 Oct):** the arm sits at the platform's edge **facing away from it**: straight ahead (j1 0) is off the table. The usable cloth surface is on the arm's right (j1 −40°…−80°, where the white cup stands at ≈ −60°) and left. So tape the sheet **flat** (all four edges, no bubbles) on the **arm's right, on the cloth next to the cup**, about 25–30 cm from the base, "+x" pointing radially away from the base. Use its rough centre for `--board-center` (e.g. base frame `0.15,-0.25,0` at ≈ −60°); the solve finds the exact pose.
- **Camera mounting (measured 28 Oct, scan 1):** the wrist camera looks **~60° off the finger axis** (rotated about the tip's y axis, toward tip +x), ~6 cm behind the tip; the jaws fill the lower corners. `camera.T_TIP_CAM_NOMINAL` encodes this, and `finger_dir_for_cam_axis` turns "camera looks along a" into the finger direction for IK: camera straight down = fingers ~30° below horizontal, pointing outward. Hand-eye replaces the nominal with the measured transform.

A board shown on a laptop screen also works for the lens (not for hand-eye, unless the laptop lies flat and still): run `python -m interfaces.wristcam.board --screen 1920x1080 --pixel-mm <screen width mm / 1920>`, show the PNG full screen at 100%, and pass the printed `--square-mm` to `solve`.

### 1. Look before moving

```
.venv/bin/python vision/sources.py dashboard --out /tmp/wrist.jpg       # read-only frame grab
.venv/bin/python -m interfaces.wristcam.handeye plan --board-center 0.15,-0.25,0 --n 15
.venv/bin/python -m interfaces.wristcam.handeye plan --board-center 0.15,-0.25,0 --n 20 --purpose intrinsics
```

`plan` moves nothing. It prints every view's q, tip, tilt and camera height. Each view has been checked against the soft box, the table guard (tip ≥ 0.08 m, links ≥ 0.03 m) and board visibility.

### 2. Collect views (the arm moves; each motion is approved)

```
.venv/bin/python -m interfaces.wristcam.handeye collect --board-center 0.15,-0.25,0 --n 20 --purpose intrinsics \
    --out interfaces/wristcam/data/intr1 --execute
.venv/bin/python -m interfaces.wristcam.handeye collect --board-center 0.15,-0.25,0 --n 15 \
    --out interfaces/wristcam/data/he1 --execute
```

The program asks for the lease (approve it in the console) and, before **each** move, prints the joint delta and the target tip and waits for `y`. It moves at 0.25 rad/s, settles for 1.5 s, saves the frame and the *measured* q, and returns to ready at the end. Arm-driven intrinsics need no one holding the board. You can also `intrinsics capture --source dashboard --n 25 --out ...` with the arm parked and a person moving a board (a printed board on cardboard or a laptop).

### 3. Solve

```
.venv/bin/python -m interfaces.wristcam.intrinsics solve --dir interfaces/wristcam/data/intr1
     # add the hand-eye frames too: copy them into the same folder (more coverage)
.venv/bin/python -m interfaces.wristcam.handeye solve --dir interfaces/wristcam/data/he1 --write-workcell
```

Check the output:

- **Intrinsics:** RMS < 0.5 px, coverage ≥ 70% of the cells, and a FOV that makes sense (~100°).
- **Hand-eye:** reprojection < 1 px, and the board-pose spread across views of a few mm / < 0.5°. The camera offset should look physically right (bracket camera: within ~10 cm of the tip, optical axis ~60° off the fingers).
- **Table:** origin near the taped sheet, z ≈ the table height in the base frame, tilt < 1°.

`solve` runs all five `cv2.calibrateHandEye` methods **and** `cv2.calibrateRobotWorldHandEye` (SHAH, LI), keeps only candidates that satisfy the **bracket prior** (optical axis within 25° of the nominal mounting, camera within 15 cm of the tip; roll about the axis is free), and picks the one whose board poses agree best. `--write-workcell` then **refuses** to mark the workcell `calibrated: true` unless every sanity gate passes (reprojection < 3 px, board spread < 6 mm and < 1°, table origin within 2 cm of `--table-z` (default 0; pass e.g. `-0.03` if the base sits on a 3 cm plate), optical axis within 25° of nominal, camera within 15 cm of the tip); it prints each gate and writes a failed result to `calib/handeye.rejected-<time>.json`, never to `handeye.json`. `solve` also estimates the board scale from the arm's kinematics (FK has metric scale, so PnP distances that disagree with the arm's geometry show up as a factor ≠ 1) and gates on |factor − 1| < 5%: a wrong square size fails here (28 Oct: the "29.97 mm" board fitted ≈ 22.4–22.9 mm from any starting guess). **Measure a printed square with a ruler** and pass `--square-mm` (or set `WRISTCAM_SQUARE_MM` for every tool).

`plan --purpose handeye` (default 15 views) now spreads the views over rotations about several axes: camera tilt 10–35° toward headings that walk round the board, plus j6 roll of alternating sign (0.15–0.6 rad); it prints a `rotation diversity` score (relative-rotation axis spread; 0 = all about one axis). A board right next to the base is barely reachable with tilted views: put it ~0.25–0.35 m out.

`workcell.json` gets the new `T_base_table` with `calibrated: true`, and the old file is kept as `workcell.json.bak-<time>`.

Check: `locate --image <a frame> --q=<its q>` with the cup on the table, then compare against a ruler.

### 4. Look and grasp: dry run, then execute

Use a cup like workcell `objects.cup` (6 cm, 8 cm tall), on the arm's right (table y ≈ -0.15).

```
# park the arm at a camera-down look pose over the cup (approved move): tip ~(0.134, -0.234, 0.22),
# camera straight down, q ~ [-1.06, 0.80, 0.64, -0.37, 0, 0] (the scan-3a pose); then (nothing moves):
.venv/bin/python -m interfaces.wristcam.look_and_grasp --object cup --place 0.08,-0.05
#   -> runs/<time>/plan.json, look.jpg (annotated), <name>.playable.json (validated open-loop pick_place)
# closed loop on the real arm (each motion asks y/N):
.venv/bin/python -m interfaces.wristcam.look_and_grasp --execute --object cup --expect <cup xy in the table frame, from the dry run> --place 0.08,-0.05
```

`--execute` refuses to start without a lens calibration, a hand-eye calibration and a calibrated workcell. Keep a hand near E-STOP. The first time, use something soft (a paper cup) and watch the descent: the fingers pass 1 cm either side of the rim.

### Safety notes

- **Speeds:** 0.25 rad/s for looks, 0.12 rad/s for the approach, descent and lift. The daemon caps everything at 0.3 rad/s, its default real-arm cap.
- **Table:** the tip goes below 0.08 m only in DESCEND, CLOSE, PLACE-down and release, and never below `min_tip_z` (0.01 m). The camera has to stay 1 cm above the rim, otherwise the grasp is refused.
- The engine never sends a single raw trajectory point. Every move is a daemon `move_joints`, which ramps with the caps and is gated by the envelope.
- **Never jog to full stretch.** The look heights stay well inside the reach.

## What still needs measuring on the real arm

1. Where the camera actually sits (hand-eye gives it), and whether the jaws hide the lower corners as assumed (`camera.JAW_POLYGONS`; check a frame with the gripper open and closed).
2. Whether frames from the console are fresh after a move (`--settle`). Also whether the stream is 1280x720; if it isn't, K is rescaled, but calibrate at the stream size.
3. Gripper: how measured grip relates to finger opening (assumed linear, 0..0.094 m), the stall reading on a cup, and `tip_to_pad`.
4. The cup's real rim height and diameter, and whether its rim contrasts with the table (a white cup on the white sheet won't).
5. FK accuracy at the look poses: the hand-eye residual shows it.

## Pick up anything you name (`detect.py`, `pick_anything.py`)

A hand-built finder per object doesn't scale, and the tap path's GrabCut failed on the real table (white cup on white paper, light cup on dark wood). Open-vocabulary segmentation is a solved problem, so the mask now comes from a detector, and everything after it is the tap path's geometry and engine, unchanged:

1. `detector.detect(frame, ["the tape roll"])` → masks. YOLO-E through DimOS (`dimos.perception.detection.detectors.yoloe`, the same weights DimOS's perception uses) or Ultralytics. Masks are filled from `masks.xy` polygons, which are already in frame pixels, not by resizing `masks.data` (that ignores the letterbox).
2. `target.resolve_prompt` → `target_from_mask` for each detection (footprint + height on the table plane, grasp across the narrow side, refusals). It returns the best graspable match, plus every candidate and why it was or wasn't chosen. With no prompt, it lists everything on the table.
3. `run_pick(..., finder=DetectorFinder(target, detector, prompt))`: the engine's closed loop re-detects the object by name at every look, and falls back to the template tracker when the detector misses.

```sh
.venv/bin/python -m interfaces.wristcam.pick_anything --simulate --prompt "blue box" --out /tmp/pick-anything   # no models
.venv/bin/python -m interfaces.wristcam.pick_anything --bench 30 --prompt "cup,marker,tape,box"                  # laptop GPU, nothing moves
.venv/bin/python -m interfaces.wristcam.pick_anything --source dashboard --prompt "marker"                       # detect only
.venv/bin/python -m interfaces.wristcam.pick_anything --source dashboard --prompt "marker" --execute             # REAL ARM, y/N per motion
```

`interfaces.follow.run --prompt "marker"` follows a named object with the same detector. Results and next steps: [experiments/2026-10-29-pick-anything/LOG.md](../../experiments/2026-10-29-pick-anything/LOG.md).

