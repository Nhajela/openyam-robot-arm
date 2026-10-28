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
| `synth.py` | Ray-cast synthetic wrist camera (any lens model) over a table with the sheet, tags, cups and boxes, plus `SimArm` (fake arm whose gripper stalls on the cup). Used by the tests. |

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
- Tape the sheet **flat** to the table (all four edges, no bubbles), with the "+x" arrow pointing straight away from the arm and the board centre about 30 cm in front of the base. Then table x ≈ base x, and table y points to the arm's left.

A board shown on a laptop screen also works for the lens (not for hand-eye, unless the laptop lies flat and still): run `python -m interfaces.wristcam.board --screen 1920x1080 --pixel-mm <screen width mm / 1920>`, show the PNG full screen at 100%, and pass the printed `--square-mm` to `solve`.

### 1. Look before moving

```
.venv/bin/python vision/sources.py dashboard --out /tmp/wrist.jpg       # read-only frame grab
.venv/bin/python -m interfaces.wristcam.handeye plan --board-center 0.30,0,0 --n 12
.venv/bin/python -m interfaces.wristcam.handeye plan --board-center 0.30,0,0 --n 20 --purpose intrinsics
```

`plan` moves nothing. It prints every view's q, tip, tilt and camera height. Each view has been checked against the soft box, the table guard (tip ≥ 0.08 m, links ≥ 0.03 m) and board visibility.

### 2. Collect views (the arm moves; each motion is approved)

```
.venv/bin/python -m interfaces.wristcam.handeye collect --board-center 0.30,0,0 --n 20 --purpose intrinsics \
    --out interfaces/wristcam/data/intr1 --execute
.venv/bin/python -m interfaces.wristcam.handeye collect --board-center 0.30,0,0 --n 12 \
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
- **Hand-eye:** reprojection < 1 px, and the board-pose spread across views of a few mm / < 0.5°. The camera offset should look physically right (a few cm toward the wrist, optical axis within a few degrees of the fingers).
- **Table:** origin near (0.30, 0, ~0) and tilt < 1°.

`workcell.json` gets the new `T_base_table` with `calibrated: true`, and the old file is kept as `workcell.json.bak-<time>`.

Check: `locate --image <a frame> --q=<its q>` with the cup on the table, then compare against a ruler.

### 4. Look and grasp: dry run, then execute

Use a cup like workcell `objects.cup` (6 cm, 8 cm tall), on the arm's right (table y ≈ -0.15).

```
# park the arm at the first look pose (approved move), e.g. tip (0.28, -0.12, 0.20) fingers down,
# q ~ [-0.40, 1.70, 1.59, -1.46, 0, 0]; then (nothing moves):
.venv/bin/python -m interfaces.wristcam.look_and_grasp --object cup --place 0.08,-0.05
#   -> runs/<time>/plan.json, look.jpg (annotated), <name>.playable.json (validated open-loop pick_place)
# closed loop on the real arm (each motion asks y/N):
.venv/bin/python -m interfaces.wristcam.look_and_grasp --execute --object cup --expect 0.0,-0.15 --place 0.08,-0.05
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
