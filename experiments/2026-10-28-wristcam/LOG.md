# Experiment: wrist camera: find a cup and grasp it

**Experimenters:** Naman (with Claude Code, as a subagent in a git worktree)
**Started:** 2026-10-28
**Status:** active. The code and the synthetic tests are green. Nothing in this entry touched the real arm, the CAN bus, the camera device or the live daemon. Next step: run the real-arm procedure in [`interfaces/wristcam/README.md`](../../interfaces/wristcam/README.md).

## Goal

Use the wide-angle wrist camera (between the fingers, looking along them) to find a static object (first: a cup on the arm's right) and grasp it **in a closed loop**. No calipers: lens, camera mount and table are calibrated with the camera and the arm's own FK, from **one A4 black-and-white print taped to the table**.

## Environment

- Machine: the lab laptop (Linux 6.8.0-48-generic), repo `.venv` Python 3.12.3, OpenCV 4.14.0 (contrib, aruco), scipy 1.18.1, numpy FK/IK in `armctl/kin.py`.
- Git: branch `worktree-agent-af4fa6da728a34397`, based on `0cdef77`.
- **Not touched:** the real arm (it was running a session at the time), `can0`, `/dev/video*`, ports 8090/8091/…, zenoh. All results are from synthetic renders (`interfaces/wristcam/synth.py`) and a fake arm.

## Log

### 2026-10-28 14:05 IST: reach limits for top-down looks

- **Observation:** `kin.ik(..., down=True)` succeeds only up to tip z ≈ 0.21–0.22 m (x 0.12–0.42 at y = 0). At z 0.25–0.30, j4 hits its soft lower limit (−1.593). The coordinator measured the same thing independently.
- **Decision:** The closed loop's straight-down looks are ≤ 0.18 m camera-above-cup-top (0.18 → 0.12 → 0.08). Higher views for calibration are tilted (`ik(approach_dir=...)`), up to 20° for hand-eye and 32° for intrinsics, plus a j6 roll for rotation diversity.

### 14:09: the calibration sheet

- **Decision:** One sheet does all three calibrations, after the person at the arm asked for it: ChArUco DICT_5X5_100, 9×6 squares of 28 mm (252×168 mm) on A4 landscape, with a 50 mm scale bar, centre ticks and a "+x (away from the arm)" arrow. The table frame's origin is the board centre. Object tags are DICT_4X4_50 ids 2–4 at 40 mm, on a second sheet. `python -m interfaces.wristcam.board` writes `interfaces/wristcam/print/*.pdf/png`.
- **Observation:** Checked the convention: OpenCV's board frame is the top-left corner with y going down the page (a generated image maps board (x, y) to pixel (x, y)/square). `BoardSpec.T_board_table` = diag(1, −1, −1) plus the centre.

### 14:15: renderer and calibration smoke test

- **Decision:** Synthetic frames are ray-cast (every pixel's ray from the lens model, intersected with the table texture and with cups and boxes), not warped. That way fisheye, rational and pinhole all render exactly, and a cup has a real rim, inner wall and occluding outer wall.
- **Observation:** Truth: fisheye f 700, ~102° HFOV, camera 62 mm behind the tip, (4, −7) mm off-axis, tilted 3°. 32 views (20 intrinsics + 12 hand-eye planned views) → the fisheye model is chosen, RMS 0.06 px, fx 700.0, ray error median 0.02° / max 0.2°. Hand-eye with the recovered lens: all 5 OpenCV methods agree to 0.05 mm; after refinement the error is 0.02° / 0.04 mm, and the table frame is off by 0.17 mm / 0.008°.
- **Problem:** `plan_views` took 56 s, because the numpy IK is slow (2.5 s per failed seed set). **Fix:** fewer restarts and iterations, and a single IK pass for the nominal camera, bringing it down to 6 s for 12 views.

### 14:30: CupFinder false positives on the checkerboard

- **Problem:** The first closed-loop simulation failed with `no convergence after 7 centring iterations`. The search frame had a "cup" fitted on the board: 0.98 angular coverage and 283 inliers, because checkerboard edges give support in every direction.
- **Fix:** Rim edge points must have a **radial** gradient. The support is measured as density per pixel of visible circumference (≥ 0.7). The band next to the circle must be **uniform** (std ≤ 22 grey levels), since a hallucinated circle crosses black and white. On 7 board-only views the finder now returns nothing, and the real cup is still found with an error under 0.1 mm.

### 14:35: closed loop against the fake arm

- **Observation:** `look_and_grasp --simulate`: search → centre (2× "closer", then "converged" at 10 px, 0.4 mm change) → approach → descend (3 re-alignments under 1 mm) → close (grip 0.643 against a target of 0.531: stalled on a 60 mm cup, so **holding**) → lift → verify (still stalled, no cup left on the table) → place (cup ends at (0.0795, −0.0498), target (0.08, −0.05)) → ready. That took 9 observations and ~14 s of compute. Run with `--miss-first`: close → miss → open, back off → centre → … → success.

### 14:50: hand-eye with joint noise

- **Problem:** With 0.001 rad of encoder noise on the reported q, the full 12-parameter reprojection refinement made hand-eye **worse**: rotation error went from 0.20° (closed-form HORAUD) to 0.78°, and translation from 0.5 to 2.6 mm. With views rotated only ±20°, the camera's rotation and translation trade against each other while fitting the FK noise.
- **Fix:** The refinement now changes only the camera translation and the board pose (the rotation stays closed-form), and the result is kept only if the board poses agree better. With noise, the closed form is kept: 0.20° / 0.46 mm, table 0.93 mm / 0.18°. Without noise: 0.017° / 0.08 mm.
- **Insight:** On the real arm, FK error (not image noise) will dominate hand-eye. The per-method spread in the solve output is the number to watch.

### 15:00: test suite

Command: `.venv/bin/python -m pytest -q -s armctl/tests interfaces/tests openplay/tests` → **400 passed in 280 s** (the wristcam tests are about 2.5 min of that, mostly ray-cast rendering and numpy IK).

| Synthetic check | Result |
|---|---|
| Lens (fisheye truth, 32 views) | fisheye chosen, RMS 0.058 px, fx 700.2 (truth 700), ray error median 0.006° / max 0.010° |
| Lens (standard-distortion truth, 14 views) | ties at 0.050 px, so the simpler fisheye fit is kept; ray error median 0.009° / max 0.018° |
| Hand-eye, 12 views, recovered lens, 0.001 rad joint noise | 0.197°, 0.46 mm |
| Table frame | 0.93 mm, 0.18° |
| Cup, camera 24–25 cm away (10–30° tilt) | 0.05–0.10 mm, diameter 60.1–60.3 mm (truth 60) |
| Cup with 32% of the rim behind a (bigger) jaw | 0.06 mm |
| Tag on a box, 25 cm | 0.23 mm, yaw 0.11° |
| Closed loop | success in 9 looks; cup placed within 1 mm; a miss on the first grasp → retry succeeds; nothing on the table → "not found" |

These are best case (the renderer is the model). Real errors will come from the print's flatness, focus, the cup's actual rim height and FK.
